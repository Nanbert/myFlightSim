//! Screenshots the lens-moisture film: rain, aerodynamic condensation, the
//! dry-off after the source stops, and clear air. For eyeballing the gating and
//! the speed-scaled streaks.
//!
//! Run: `FDM_BACKEND=rust AIRCRAFT=f22 SCENE=rain cargo run --example lens_shot`
//!
//! `SCENE=rain` (default) wets the film with rain at 250 kt; `SCENE=dry` rains
//! for 2.5 s, then clears, so the sheared-off film is captured frame by frame;
//! `SCENE=condense` leaves the sky clear and hauls a hard pull instead;
//! `SCENE=clear` is the control shot — level flight, no source, no film.
//! Shots land in `/tmp/opencode/`.

#[path = "../src/cloud_camera.rs"]
mod cloud_camera;
#[path = "../src/aircraft_profile.rs"]
mod aircraft_profile;
#[path = "../src/app_state.rs"]
mod app_state;
#[path = "../src/config.rs"]
mod config;
#[path = "../src/input.rs"]
mod input;
#[path = "../src/lens.rs"]
mod lens;
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

#[derive(Resource, Clone, Copy, PartialEq, Eq)]
enum Scene {
    /// Steady rain at moderate speed: drops linger, no strong shear.
    Rain,
    /// Rain stops mid-run: the film dries off at fighter speed.
    Dry,
    /// Clear air, hard pull: only condensation wets the glass.
    Condense,
    /// Clear air, level flight: the control case, glass stays dry.
    Clear,
}

fn main() {
    let scene = match std::env::var("SCENE").as_deref() {
        Ok("dry") => Scene::Dry,
        Ok("condense") => Scene::Condense,
        Ok("clear") => Scene::Clear,
        _ => Scene::Rain,
    };
    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "lens_shot".into(),
                resolution: (1200, 800).into(),
                ..default()
            }),
            ..default()
        }))
        .insert_resource(WinitSettings::continuous())
        .insert_resource(Time::<Fixed>::from_hz(1.0 / sim::DT))
        .add_plugins((
            sim::SimPlugin,
            autopilot::AutopilotPlugin,
            config::ConfigPlugin,
            input::InputPlugin,
            view::ViewPlugin,
            weather::WeatherPlugin,
            lens::LensPlugin,
        ))
        .insert_state(app_state::AppState::Flying)
        .add_systems(
            OnEnter(app_state::AppState::Flying),
            (view::spawn_flight_aircraft, view::reset_control_rig).chain(),
        )
        .init_resource::<Frame>()
        .insert_resource(scene)
        .add_systems(Startup, launch)
        .add_systems(Update, (drive_weather, report))
        .add_systems(Last, drive)
        .add_systems(Update, shot)
        .run();
}

/// Rain scenes start slow enough that the film lingers; the dry-off and
/// condensation scenes need fighter-speed dynamic pressure. The pull scene is
/// in arcade mode so it can reach the load the vapor model keys off.
fn launch(mut sim: ResMut<sim::Sim>, scene: Res<Scene>) {
    match *scene {
        Scene::Condense => {
            sim.reset_air(12_000.0, 380.0);
            sim.model.set("fcs/arcade-enable", 1.0);
        }
        Scene::Dry => sim.reset_air(12_000.0, 400.0),
        Scene::Rain | Scene::Clear => sim.reset_air(12_000.0, 250.0),
    }
}

/// Rain source per scene. The dry scene cuts it at frame 150.
fn drive_weather(scene: Res<Scene>, f: Res<Frame>, mut weather: ResMut<weather::Weather>) {
    weather.rain = match *scene {
        Scene::Rain => 0.9,
        Scene::Dry => {
            if f.0 < 150 {
                0.9
            } else {
                0.0
            }
        }
        _ => 0.0,
    };
}

/// Log the signals that gate the film, so the shots are self-describing.
fn report(state: Res<sim::AircraftState>, weather: Res<weather::Weather>, f: Res<Frame>) {
    if f.0 % 30 == 0 {
        info!(
            "frame {} rain {:.2} alpha {:.1} deg  mach {:.2}  ias {:.0} kt",
            f.0, weather.rain, state.alpha_deg, state.mach, state.cas_kts
        );
    }
}

fn drive(scene: Res<Scene>, mut bindings: ResMut<input::Bindings>) {
    let (throttle, elevator) = match *scene {
        Scene::Condense => (1.0, -0.9),
        Scene::Rain | Scene::Dry | Scene::Clear => (0.6, 0.0),
    };
    for b in &mut bindings.list {
        b.value = match b.property {
            "fcs/throttle-cmd-norm" => throttle,
            "fcs/elevator-cmd-norm" => elevator,
            _ => 0.0,
        };
    }
}

fn shot(
    scene: Res<Scene>,
    mut f: ResMut<Frame>,
    mut commands: Commands,
    mut exit: MessageWriter<AppExit>,
) {
    f.0 += 1;
    let name = match *scene {
        Scene::Rain => "rain",
        Scene::Dry => "dry",
        Scene::Condense => "condense",
        Scene::Clear => "clear",
    };
    let frames: &[(u32, &str)] = match *scene {
        // Frame 150 is when the rain cuts: 120 is the wet film, 160/175/200
        // catch the sheared-off beads, 260 is dry.
        Scene::Dry => &[
            (120, "wet"),
            (160, "stop"),
            (175, "fade1"),
            (200, "fade2"),
            (260, "dry"),
        ],
        _ => &[(200, "a"), (300, "b")],
    };
    for (n, tag) in frames {
        if f.0 == *n {
            let path = format!("/tmp/opencode/lens_{name}_{tag}.png");
            commands
                .spawn(Screenshot::primary_window())
                .observe(save_to_disk(path.clone()));
            info!("screenshot -> {path}");
        }
    }
    if f.0 == 380 {
        exit.write(AppExit::Success);
    }
}
