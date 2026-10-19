//! Cockpit-panel render check.
//!
//! Runs the real app systems (including the live instruments) in the cockpit
//! view of the full C172, and screenshots the result — used to verify that the
//! gauges seat onto the model's painted panel (`examples/panel_calib` measures
//! the dial positions this relies on).
//!
//! `COCKPIT=0` renders the usual bottom-corner layout instead, for comparison.
//! `LOOK=<deg>` sets the cockpit head-look yaw (the same field the mouse
//! drives): the panel slides across the screen and the gauges must stay on
//! their dials.
//! Run: `BEVY_ASSET_ROOT="$PWD" cargo run --example panel_view_shot`

#[path = "../src/aircraft_profile.rs"]
mod aircraft_profile;
#[path = "../src/app_state.rs"]
mod app_state;
#[path = "../src/autopilot.rs"]
mod autopilot;
#[path = "../src/cloud_camera.rs"]
mod cloud_camera;
#[path = "../src/config.rs"]
mod config;
#[path = "../src/gauge/mod.rs"]
mod gauge;
#[path = "../src/input.rs"]
mod input;
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

fn main() {
    let cockpit = std::env::var("COCKPIT").map(|v| v != "0").unwrap_or(true);
    let out = std::env::var("OUT").unwrap_or_else(|_| "/tmp/panel_view.png".into());
    let look: f32 = std::env::var("LOOK")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(0.0);

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "panel_view_shot".into(),
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
            gauge::GaugePlugin,
            weather::WeatherPlugin,
        ))
        .insert_state(app_state::AppState::Flying)
        .add_systems(
            OnEnter(app_state::AppState::Flying),
            (view::spawn_flight_aircraft, view::reset_control_rig).chain(),
        )
        .init_resource::<Frame>()
        .insert_resource(Cockpit(cockpit))
        .insert_resource(Out(out))
        .insert_resource(Look(look))
        .add_systems(Startup, apply)
        .add_systems(Update, apply.before(view::follow_camera))
        .add_systems(Update, shot)
        .run();
}

#[derive(Resource)]
struct Cockpit(bool);

#[derive(Resource)]
struct Out(String);

/// Head-look yaw to apply through the real cockpit-look field, in degrees.
#[derive(Resource)]
struct Look(f32);

fn apply(cockpit: Res<Cockpit>, look: Res<Look>, mut ui: ResMut<view::Ui>) {
    ui.cockpit = cockpit.0;
    // Drive the same field the mouse drives, so this exercises the real path.
    ui.look_yaw = look.0.to_radians();
    ui.look_pitch = 0.0;
}

fn shot(
    mut commands: Commands,
    mut f: ResMut<Frame>,
    mut exit: MessageWriter<AppExit>,
    out: Res<Out>,
) {
    f.0 += 1;
    match f.0 {
        200 => {
            commands
                .spawn(Screenshot::primary_window())
                .observe(save_to_disk(out.0.clone()));
        }
        240 => {
            exit.write(AppExit::Success);
        }
        _ => {}
    }
}
