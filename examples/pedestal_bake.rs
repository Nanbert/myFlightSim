//! Bake the 737 control stand's plate to `assets/ui/737_pedestal_plate.png`.
//!
//! Run: `cargo run --features shots --example pedestal_bake`. The window is
//! only a host for the render device; the plate is rendered to an offscreen
//! 4940x1080 target and saved, then the app exits.

use bevy::prelude::*;
use my_flight_sim::pedestal_bake::PedestalBakePlugin;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "pedestal_bake".into(),
                resolution: (800, 240).into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(PedestalBakePlugin::new(
            "assets/ui/737_pedestal_plate.png",
        ))
        .run();
}
