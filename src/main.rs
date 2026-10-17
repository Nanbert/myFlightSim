mod aircraft_profile;
mod app_state;
mod autopilot;
mod config;
mod fx;
mod gauge;
mod hud;
mod input;
mod lens;
mod mcp_ui;
mod menu;
mod pfd;
mod sim;
mod sky;
mod vapor;
mod view;
mod weather;

use bevy::prelude::*;
use bevy::window::WindowMode;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "myFlightSim — JSBSim + Bevy".into(),
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
        .run();
}
