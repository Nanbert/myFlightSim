//! Screenshots the F-22's condensation vapor: the wingtip trails in a hard pull
//! and the transonic cone near Mach 1, for eyeballing placement and tuning.
//!
//! Run: `FDM_BACKEND=rust AIRCRAFT=f22 cargo run --example vapor_shot`
//!
//! `SCENE=pull` (default) hauls into a hard pull so the wingtips, leading edge
//! and forebody all feed; `SCENE=roll` rolls while pulling, so the leading edge
//! lights up on the descending wing alone; `SCENE=sonic` flies fast for the
//! Mach-1 cone. Shots land in `/tmp/opencode/`.

#[path = "../src/aircraft_profile.rs"]
mod aircraft_profile;
#[path = "../src/app_state.rs"]
mod app_state;
#[path = "../src/config.rs"]
mod config;
#[path = "../src/fx.rs"]
mod fx;
#[path = "../src/input.rs"]
mod input;
#[path = "../src/sim.rs"]
mod sim;
#[path = "../src/vapor.rs"]
mod vapor;
#[path = "../src/view.rs"]
mod view;
#[path = "../src/weather.rs"]
mod weather;

use bevy::prelude::*;
use bevy::render::view::window::screenshot::{save_to_disk, Screenshot};
use bevy::winit::WinitSettings;

#[derive(Resource, Default)]
struct Frame(u32);

/// `CAM=game` leaves the camera to the game's own chase rig (`follow_camera` +
/// `camera_feel`) instead of the fixed test framing, so the buffet, bank
/// coupling and motion blur can be previewed.
#[derive(Resource, Default)]
struct GameCam(bool);

/// Which manoeuvre to fly for the shot.
#[derive(Resource, Clone, Copy, PartialEq, Eq)]
enum Scene {
    /// A hard sustained pull: the wingtips, leading edge and forebody all feed.
    Pull,
    /// A rolling pull: the leading edge lights up on the descending wing alone.
    Roll,
    /// Fast and level for the Mach-1 cone.
    Sonic,
    /// Fast and level, from a close front quarter: every emitter feeds near
    /// Mach 1, so the trails mark exactly where each anchor sits on the
    /// airframe. For checking placement, not for looking pretty.
    Place,
}

/// Mach the sonic run parks itself on: the peak of the vapor-cone band in
/// `vapor.rs`.
const MACH_TARGET: f64 = 1.02;

/// Integral throttle trim, used only by the sonic run. A fixed setting does not
/// work: the F-22 model either sags below Mach 1 at part throttle or blasts
/// through the band at full, so the run hunts for the setting that holds.
#[derive(Resource)]
struct MachHold {
    throttle: f64,
}

fn main() {
    let scene = match std::env::var("SCENE").as_deref() {
        Ok("sonic") => Scene::Sonic,
        Ok("roll") => Scene::Roll,
        Ok("place") => Scene::Place,
        _ => Scene::Pull,
    };
    let game_cam = std::env::var("CAM").is_ok_and(|v| v == "game");
    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "vapor_shot".into(),
                resolution: (1200, 800).into(),
                ..default()
            }),
            ..default()
        }))
        // Keep stepping while unfocused, so the frame-counted shots fire.
        .insert_resource(WinitSettings::continuous())
        .insert_resource(Time::<Fixed>::from_hz(1.0 / sim::DT))
        .add_plugins((
            sim::SimPlugin,
            config::ConfigPlugin,
            input::InputPlugin,
            view::ViewPlugin,
            vapor::VaporPlugin,
            fx::FxPlugin,
            weather::WeatherPlugin,
        ))
        .insert_state(app_state::AppState::Flying)
        .add_systems(
            OnEnter(app_state::AppState::Flying),
            (view::spawn_flight_aircraft, view::reset_control_rig).chain(),
        )
        .insert_resource(sim::Autopilot {
            enabled: false,
            phase: "manual",
        })
        .init_resource::<Frame>()
        .insert_resource(scene)
        .insert_resource(GameCam(game_cam))
        .insert_resource(MachHold { throttle: 0.75 })
        .add_systems(Startup, launch)
        .add_systems(Update, clear_weather)
        .add_systems(Update, report)
        .add_systems(Last, drive)
        .add_systems(Last, shot)
        .add_systems(PostUpdate, chase_cam)
        .run();
}

/// Start airborne and moving. At 30,000 ft Mach 1 is about 390 kt CAS, so the
/// sonic run seats itself just under the barrier and lets the throttle trim do
/// the rest; the pull run starts slow enough to haul AoA up quickly.
fn launch(mut sim: ResMut<sim::Sim>, scene: Res<Scene>) {
    match *scene {
        Scene::Sonic | Scene::Place => sim.reset_air(30_000.0, 390.0),
        // Slow enough that a pull climbs the angle of attack quickly — and in
        // arcade, so the manoeuvres match the game (the roll rate and the loads
        // the vapor keys off are arcade's, not the realistic model's).
        Scene::Pull | Scene::Roll => {
            sim.reset_air(12_000.0, 380.0);
            sim.model.set("fcs/arcade-enable", 1.0);
        }
    }
}

/// Pin the sky clear: rain on the lens would obscure the effect. (Rain cannot
/// reach the vapor itself here, since the shot run is a single fixed scene.)
fn clear_weather(mut weather: ResMut<weather::Weather>) {
    weather.rain = 0.0;
}

/// Log the signals that drive the effects, so runs are self-describing.
fn report(state: Res<sim::AircraftState>, f: Res<Frame>) {
    if f.0 % 40 == 0 {
        info!(
            "frame {} alpha {:.1} deg  mach {:.2}  ias {:.0} kt  alt {:.0} ft  \
             roll {:.0} deg  p {:.0} deg/s  r {:.0} deg/s",
            f.0,
            state.alpha_deg,
            state.mach,
            state.cas_kts,
            state.alt_ft,
            state.roll_deg,
            state.roll_rate_dps,
            state.yaw_rate_dps
        );
    }
}

/// Full throttle throughout; the pull run also hauls the stick back. The sonic
/// run trims throttle to hold `MACH_TARGET` instead, so the cone stays lit for
/// the whole shot window.
fn drive(
    scene: Res<Scene>,
    state: Res<sim::AircraftState>,
    mut hold: ResMut<MachHold>,
    mut bindings: ResMut<input::Bindings>,
) {
    let throttle = if matches!(*scene, Scene::Sonic | Scene::Place) {
        hold.throttle = (hold.throttle + (MACH_TARGET - state.mach) * 0.03).clamp(0.3, 1.0);
        hold.throttle
    } else {
        1.0
    };
    for b in &mut bindings.list {
        b.value = match b.property {
            "fcs/throttle-cmd-norm" => throttle,
            // Negative elevator is nose-up (the pull). The roll run trades some
            // pull for aileron, so the wing is loaded while it rolls.
            "fcs/elevator-cmd-norm" if *scene == Scene::Pull => -0.9,
            "fcs/elevator-cmd-norm" if *scene == Scene::Roll => -0.45,
            "fcs/aileron-cmd-norm" if *scene == Scene::Roll => 1.0,
            _ => 0.0,
        };
    }
}

/// Chase the jet from a quarter view: the trails run behind and below, so sit
/// off to one side. The sonic run favours a broadside to see the whole cone.
fn chase_cam(
    rp: Res<sim::RenderPose>,
    scene: Res<Scene>,
    game_cam: Res<GameCam>,
    mut q: Query<&mut Transform, With<Camera3d>>,
) {
    if game_cam.0 {
        return;
    }
    let Ok(mut t) = q.single_mut() else { return };
    if !rp.init {
        return;
    }
    let local = match *scene {
        Scene::Sonic => Vec3::new(40.0, 4.0, 10.0),
        // Abeam and level: the transonic cone reads as a cone along the body,
        // and the tip trails as two ribbons, so any vapor elsewhere is a
        // misplaced anchor.
        Scene::Place => Vec3::new(40.0, 6.0, 0.0),
        // Behind, left and high: the trails stream toward the camera and the
        // leading edge on the descending wing reads against the other side.
        Scene::Roll => Vec3::new(24.0, 17.0, -22.0),
        // Far enough back and out that the whole ribbon stays in frame once the
        // jet has pitched up into the pull.
        Scene::Pull => Vec3::new(34.0, 12.0, 46.0),
    };
    let pos = rp.curr_pos;
    let yaw = Quat::from_rotation_y(-rp.curr_heading);
    *t = Transform::from_translation(pos + yaw * local).looking_at(pos, Vec3::Y);
}

fn shot(
    mut commands: Commands,
    scene: Res<Scene>,
    game_cam: Res<GameCam>,
    mut f: ResMut<Frame>,
    mut exit: MessageWriter<AppExit>,
) {
    f.0 += 1;
    let name = match *scene {
        Scene::Pull => "pull",
        Scene::Roll => "roll",
        Scene::Sonic => "sonic",
        Scene::Place => "place",
    };
    let cam = if game_cam.0 { "game" } else { "fix" };
    let dir = format!("{name}_{cam}");
    for (n, tag) in [(150u32, "a"), (300, "b"), (450, "c")] {
        if f.0 == n {
            let path = format!("/tmp/opencode/vapor_{dir}_{tag}.png");
            commands
                .spawn(Screenshot::primary_window())
                .observe(save_to_disk(path.clone()));
            info!("screenshot -> {path}");
        }
    }
    if f.0 == 520 {
        exit.write(AppExit::Success);
    }
}
