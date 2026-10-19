//! Screenshot of the 737 pilot-eye cockpit, with the PFDs on the display units.
//!
//! Loads the same plugins as `main.rs` (minus the menu), forces the cockpit
//! camera and grabs one frame once the interior has had time to load.
//!
//! Run: `cargo run --example b737_cockpit_shot` and open `/tmp/b737_cockpit_shot.png`.

#[path = "../src/aircraft_profile.rs"]
mod aircraft_profile;
#[path = "../src/app_state.rs"]
mod app_state;
#[path = "../src/autopilot.rs"]
mod autopilot;
#[path = "../src/cloud_camera.rs"]
mod cloud_camera;
#[path = "../src/cockpit.rs"]
mod cockpit;
#[path = "../src/config.rs"]
mod config;
#[path = "../src/input.rs"]
mod input;
#[path = "../src/mcp_ui.rs"]
mod mcp_ui;
#[path = "../src/pfd.rs"]
mod pfd;
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
    std::env::set_var("AIRCRAFT", "737");
    std::env::set_var("FDM_BACKEND", "rust");
    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "b737_cockpit_shot".into(),
                resolution: (1200, 800).into(),
                ..default()
            }),
            ..default()
        }))
        .insert_resource(ClearColor(Color::srgb(0.02, 0.03, 0.05)))
        .insert_resource(WinitSettings::continuous())
        .insert_resource(Time::<Fixed>::from_hz(1.0 / sim::DT))
        .add_plugins((
            sim::SimPlugin,
            autopilot::AutopilotPlugin,
            config::ConfigPlugin,
            input::InputPlugin,
            view::ViewPlugin,
            mcp_ui::McpUiPlugin,
            pfd::PfdPlugin,
            cockpit::CockpitPlugin,
            weather::WeatherPlugin,
        ))
        // No menu: start in `Flying` and spawn the aircraft directly.
        .insert_state(app_state::AppState::Flying)
        .add_systems(
            OnEnter(app_state::AppState::Flying),
            (view::spawn_flight_aircraft, view::reset_control_rig).chain(),
        )
        .init_resource::<Frame>()
        .add_systems(Update, (force_cockpit, shot))
        .run();
}

fn force_cockpit(mut ui: ResMut<view::Ui>) {
    ui.cockpit = true;
    // Point the crosshair at HDG SEL, so the shot also proves the pick works.
    ui.look_yaw = -26.69_f32.to_radians();
    ui.look_pitch = -2.64_f32.to_radians();
}

fn shot(mut commands: Commands, mut frame: ResMut<Frame>, mut exit: MessageWriter<AppExit>) {
    frame.0 += 1;
    if frame.0 == 600 {
        commands
            .spawn(Screenshot::primary_window())
            .observe(save_to_disk("/tmp/b737_cockpit_shot.png"));
        info!("screenshot -> /tmp/b737_cockpit_shot.png");
    }
    if frame.0 >= 700 {
        exit.write(AppExit::Success);
    }
}
