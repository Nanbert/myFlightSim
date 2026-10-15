mod aircraft_profile;
mod app_state;
mod config;
mod input;
mod lens;
mod menu;
mod sim;
mod sky;
mod view;
mod weather;

use bevy::prelude::*;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "myFlightSim — JSBSim + Bevy".into(),
                ..default()
            }),
            ..default()
        }))
        .insert_resource(Time::<Fixed>::from_hz(1.0 / sim::DT))
        .add_plugins((
            sim::SimPlugin,
            // Before `InputPlugin`: it needs `GameConfig` to build the startup
            // aircraft's bindings from the saved keymap.
            config::ConfigPlugin,
            input::InputPlugin,
            view::ViewPlugin,
            sky::SkyPlugin,
            weather::WeatherPlugin,
            lens::LensPlugin,
            menu::MenuPlugin,
        ))
        .run();
}
