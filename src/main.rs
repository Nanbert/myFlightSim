mod input;
mod lens;
mod sim;
mod sky;
mod sound;
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
            input::InputPlugin,
            view::ViewPlugin,
            sky::SkyPlugin,
            sound::SoundPlugin,
            weather::WeatherPlugin,
            lens::LensPlugin,
        ))
        .run();
}
