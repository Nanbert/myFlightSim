//! Screenshot of the 737 pilot-eye cockpit, with the PFDs on the display units.
//!
//! Loads the same plugins as `main.rs` (minus the menu), forces the cockpit
//! camera and grabs one frame once the interior has had time to load.
//!
//! Run: `cargo run --features shots --example b737_cockpit_shot` and open
//! `/tmp/b737_cockpit_shot.png`.

mod common;

use bevy::prelude::*;
use my_flight_sim::{cockpit, mcp_ui, pfd, view};

fn main() {
    std::env::set_var("AIRCRAFT", "737");
    std::env::set_var("FDM_BACKEND", "rust");
    common::app("b737_cockpit_shot", Some((1200, 800)))
        .insert_resource(ClearColor(Color::srgb(0.02, 0.03, 0.05)))
        .add_plugins((mcp_ui::McpUiPlugin, pfd::PfdPlugin, cockpit::CockpitPlugin))
        .insert_resource(common::Shots::new(
            &[(600, "/tmp/b737_cockpit_shot.png")],
            700,
        ))
        .add_systems(Update, (force_cockpit, common::drive_shots))
        .run();
}

fn force_cockpit(mut ui: ResMut<view::Ui>) {
    ui.cockpit = true;
    // Point the crosshair at HDG SEL, so the shot also proves the pick works.
    ui.look_yaw = -26.69_f32.to_radians();
    ui.look_pitch = -2.64_f32.to_radians();
}
