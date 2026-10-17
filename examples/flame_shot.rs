//! Screenshots the F-22's exhaust: the dry military plume and the afterburner,
//! including the ignition flare, for eyeballing placement and tuning.
//!
//! Run: `FDM_BACKEND=rust AIRCRAFT=f22 SCENE=ab cargo run --example flame_shot`
//!
//! `SCENE=mil` (default) holds a dry power setting; `SCENE=ab` goes to full
//! reheat, so an early shot catches the light-up flare and a late one the
//! settled plume. `SCENE=arc` flies the arcade flight model and sweeps the lever
//! up and back down, so the plume's automatic growth and fade are on camera.
//! Shots land in `/tmp/opencode/`.

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
#[path = "../src/autopilot.rs"]
mod autopilot;
#[path = "../src/sim.rs"]
mod sim;
#[path = "../src/view.rs"]
mod view;
#[path = "../src/weather.rs"]
mod weather;

use bevy::prelude::*;
use bevy::render::view::window::screenshot::{save_to_disk, Screenshot};
use bevy::winit::WinitSettings;

#[derive(Resource, Default)]
struct Frame(u32);

/// Which power setting to fly for the shot.
#[derive(Resource, Default)]
struct Scene {
    afterburner: bool,
    /// Fly the F-22's arcade flight model and sweep the lever, so the plume's
    /// automatic growth and fade are captured.
    arcade: bool,
}

fn main() {
    let scene = std::env::var("SCENE").unwrap_or_default();
    let afterburner = scene == "ab";
    let arcade = scene == "arc";
    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "flame_shot".into(),
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
            autopilot::AutopilotPlugin,
            config::ConfigPlugin,
            input::InputPlugin,
            view::ViewPlugin,
            fx::FxPlugin,
            weather::WeatherPlugin,
        ))
        .insert_state(app_state::AppState::Flying)
        .add_systems(
            OnEnter(app_state::AppState::Flying),
            (view::spawn_flight_aircraft, view::reset_control_rig).chain(),
        )
        .init_resource::<Frame>()
        .insert_resource(Scene { afterburner, arcade })
        .add_systems(Startup, launch)
        .add_systems(Update, report)
        .add_systems(Last, drive)
        .add_systems(Last, shot)
        .add_systems(PostUpdate, chase_cam)
        .run();
}

/// Start airborne and moving, so the plume trails and the nozzle is not buried
/// in the runway behind it.
fn launch(scene: Res<Scene>, mut sim: ResMut<sim::Sim>) {
    // Arcade changes what trim means, so select it before the reset seats the
    // realistic trim (a no-op under arcade).
    sim.model.set("fcs/arcade-enable", scene.arcade as i32 as f64);
    sim.reset_air(12_000.0, 320.0);
}

/// Log the signals that drive the flame, so runs are self-describing.
fn report(state: Res<sim::AircraftState>, f: Res<Frame>) {
    if f.0 % 20 == 0 {
        info!(
            "frame {} throttle {:.2}  mach {:.2}  ias {:.0} kt",
            f.0, state.throttle, state.mach, state.cas_kts
        );
    }
}

/// Pin the throttle. Full reheat is `> 0.99`, which is what lights the burner;
/// anything under it stays on the dry branch.
///
/// The light-up is held back a few frames so the ignition flare lands after the
/// chase camera has settled — otherwise the early shots come out empty.
fn drive(scene: Res<Scene>, f: Res<Frame>, mut bindings: ResMut<input::Bindings>) {
    let throttle = if scene.arcade {
        // Sweep idle → full reheat → idle so both the plume's growth and its
        // fade are on camera.
        let t = f.0 as f32;
        if t < 60.0 {
            0.35
        } else if t < 140.0 {
            0.35 + ((t - 60.0) / 80.0) * 0.65
        } else if t < 260.0 {
            1.0
        } else if t < 360.0 {
            1.0 - ((t - 260.0) / 100.0) * 0.65
        } else {
            0.35
        }
    } else if scene.afterburner && f.0 > 20 {
        1.0
    } else {
        0.85
    };
    for b in &mut bindings.list {
        b.value = match b.property {
            "fcs/throttle-cmd-norm" => throttle as f64,
            _ => 0.0,
        };
    }
}

/// Sit off the tail, looking forward up the plume.
fn chase_cam(
    rp: Res<sim::RenderPose>,
    mut q: Query<&mut Transform, With<Camera3d>>,
) {
    let Ok(mut t) = q.single_mut() else { return };
    if !rp.init {
        return;
    }
    let pos = rp.curr_pos;
    let yaw = Quat::from_rotation_y(-rp.curr_heading);
    // The jet is ~19 m long and the plume runs 9–14 m aft of the CG, so sit
    // abeam of the tail and look at the tail itself, not at the CG.
    let local = Vec3::new(33.0, 1.5, 4.5);
    let aim = pos + yaw * Vec3::new(0.0, 0.0, 10.0);
    *t = Transform::from_translation(pos + yaw * local).looking_at(aim, Vec3::Y);
}

fn shot(
    mut commands: Commands,
    scene: Res<Scene>,
    mut f: ResMut<Frame>,
    mut exit: MessageWriter<AppExit>,
) {
    f.0 += 1;
    let dir = if scene.arcade {
        "arc"
    } else if scene.afterburner {
        "ab"
    } else {
        "mil"
    };
    // Realistic: light-up is at frame 20, so `ign` lands inside the 0.3 s flare
    // and the rest after it. Arcade: the lever sweeps, so these catch the plume
    // ramping up, holding at full reheat, and fading back to a dry stub.
    let shots: &[(u32, &str)] = if scene.arcade {
        &[(100, "ramp"), (170, "full"), (240, "hold"), (320, "fade"), (380, "dry")]
    } else {
        &[(26, "ign"), (46, "a"), (110, "b"), (220, "c")]
    };
    for &(n, tag) in shots {
        if f.0 == n {
            let path = format!("/tmp/opencode/flame_{dir}_{tag}.png");
            commands
                .spawn(Screenshot::primary_window())
                .observe(save_to_disk(path.clone()));
            info!("screenshot -> {path}");
        }
    }
    if f.0 == 410 {
        exit.write(AppExit::Success);
    }
}
