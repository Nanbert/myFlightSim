//! Screenshot of the 737 MCP raised as a 2D panel.
//!
//! Enters the cockpit, forces the MCP focus, and grabs one frame once the
//! panel has dollied in.
//!
//! Run: `cargo run --features shots --example mcp_focus_shot` and open
//! `/tmp/mcp_focus.png`.

mod common;

use bevy::prelude::*;
use my_flight_sim::{cockpit, mcp_panel, mcp_ui, pfd, view};

fn main() {
    std::env::set_var("AIRCRAFT", "737");
    std::env::set_var("FDM_BACKEND", "rust");
    common::app("mcp_focus_shot", Some((1600, 900)))
        .insert_resource(ClearColor(Color::srgb(0.02, 0.03, 0.05)))
        .add_plugins((
            mcp_ui::McpUiPlugin,
            mcp_panel::McpPanelPlugin,
            pfd::PfdPlugin,
            cockpit::CockpitPlugin,
            my_flight_sim::lights::LightsPlugin,
        ))
        .insert_resource(common::Shots::new(&[(90, "/tmp/mcp_focus.png")], 110))
        .init_resource::<my_flight_sim::map::ActiveMap>()
        .init_resource::<my_flight_sim::map::SunPose>()
        .init_resource::<my_flight_sim::airfield::Airfield>()
        .init_resource::<my_flight_sim::mission::ActiveLesson>()
        .init_resource::<my_flight_sim::taxi::Taximeter>()
        .add_systems(Update, (force_focus, common::drive_shots))
        .run();
}

fn force_focus(mut ui: ResMut<view::Ui>) {
    ui.cockpit = true;
    ui.mcp_focus = true;
}
