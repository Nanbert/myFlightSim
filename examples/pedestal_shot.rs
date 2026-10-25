//! Screenshot of the 737 control stand strip, a few seconds after flaps 15 and
//! 72% throttle, so the actual-flap pointer visibly lags the lever.
//!
//! Run: `cargo run --features shots --example pedestal_shot` and open
//! `/tmp/pedestal.png`.

mod common;

use bevy::prelude::*;
use bevy::window::WindowResolution;
use my_flight_sim::aircraft_profile::Action;
use my_flight_sim::input::Bindings;
use my_flight_sim::sim::Sim;
use my_flight_sim::{mcp_ui, pedestal_ui, pfd};

fn main() {
    std::env::set_var("AIRCRAFT", "737");
    std::env::set_var("FDM_BACKEND", "rust");
    common::app_with(Window {
        title: "pedestal_shot".into(),
        resolution: WindowResolution::new(1200, 1000).with_scale_factor_override(1.0),
        ..default()
    })
        .add_plugins((
            mcp_ui::McpUiPlugin,
            pedestal_ui::PedestalUiPlugin,
            pfd::PfdPlugin,
            my_flight_sim::lights::LightsPlugin,
        ))
        .init_resource::<my_flight_sim::map::ActiveMap>()
        .init_resource::<my_flight_sim::map::SunPose>()
        .init_resource::<my_flight_sim::airfield::Airfield>()
        .init_resource::<my_flight_sim::mission::ActiveLesson>()
        .init_resource::<my_flight_sim::taxi::Taximeter>()
        .insert_resource(common::Shots::new(&[(45, "/tmp/pedestal.png")], 60))
        .add_systems(Update, (dress, common::drive_shots))
        .run();
}

fn dress(mut bindings: ResMut<Bindings>, mut sim: ResMut<Sim>, f: Res<common::Frame>) {
    if f.0 == 2 {
        bindings.set(Action::Flap, 15.0 / 40.0);
        bindings.set(Action::Throttle, 0.72);
        // Exercise the latching indicators too: boards out, reversers armed.
        sim.model.set("fcs/speedbrake-cmd-norm", 1.0);
        sim.model.set("fcs/reverse-cmd-norm", 1.0);
    }
}
