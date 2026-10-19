mod aerodrome;
mod aircraft_profile;
mod app_state;
mod assist;
mod autopilot;
mod cloud_camera;
mod cockpit;
mod config;
mod controls;
mod fx;
mod gauge;
mod hud;
mod input;
mod lens;
mod map;
mod mcp_ui;
mod menu;
mod pfd;
mod sim;
mod sky;
mod tutorial;
mod vapor;
mod view;
mod weather;

use bevy::prelude::*;
use bevy::render::view::window::screenshot::{save_to_disk, Screenshot};
use bevy::window::WindowMode;

#[derive(Resource)]
struct ShotTimer(Timer);

impl Default for ShotTimer {
    fn default() -> Self {
        Self(Timer::from_seconds(3.0, TimerMode::Once))
    }
}

fn main() {
    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "myFlightSim — Rust + Bevy".into(),
                // The instrument panel is laid out for a wide screen: three
                // gauges against each edge with the attitude indicator centred.
                // A half-width window would overlap the two trios, so start
                // fullscreen (as the sim is meant to be flown).
                mode: WindowMode::BorderlessFullscreen(MonitorSelection::Primary),
                ..default()
            }),
            ..default()
        }))
        .insert_resource(Time::<Fixed>::from_hz(1.0 / sim::DT))
        .add_plugins((
            sim::SimPlugin,
            autopilot::AutopilotPlugin,
            mcp_ui::McpUiPlugin,
            pfd::PfdPlugin,
            // Before `InputPlugin`: it needs `GameConfig` to build the startup
            // aircraft's bindings from the saved keymap.
            config::ConfigPlugin,
            input::InputPlugin,
            view::ViewPlugin,
            fx::FxPlugin,
            vapor::VaporPlugin,
            gauge::GaugePlugin,
            hud::HudPlugin,
            sky::SkyPlugin,
            weather::WeatherPlugin,
            lens::LensPlugin,
            menu::MenuPlugin,
        ))
        // After the menu: it builds terrain for whatever mission
        // `start_flight` just launched. (A separate call: Bevy's plugin tuples
        // top out at 15.)
        .add_plugins(map::MapPlugin)
        .add_plugins(cockpit::CockpitPlugin)
        .add_plugins(controls::ControlPlugin)
        .add_plugins(assist::AssistPlugin)
        .add_plugins(aerodrome::AerodromePlugin)
        .init_resource::<ShotTimer>()
        .add_systems(Update, auto_shot)
        .run();
}

/// Development hook: `SHOT=/tmp/game.png` grabs one frame after a few seconds,
/// so a desktop screenshot pass can inspect the real game view without driving
/// the menu or a window manager.
fn auto_shot(mut commands: Commands, time: Res<Time>, mut timer: ResMut<ShotTimer>) {
    let Ok(path) = std::env::var("SHOT") else {
        return;
    };
    timer.0.tick(time.delta());
    if timer.0.just_finished() {
        commands
            .spawn(Screenshot::primary_window())
            .observe(save_to_disk(path));
    }
}
