//! Drives the C172 instrument cluster through a few flight conditions and
//! screenshots each, for eyeballing the gauges against the reference photos.
//!
//! Run: `cargo run --features shots --example gauge_shot`

mod common;

use bevy::prelude::*;
use my_flight_sim::{gauge, sim};

fn main() {
    common::app("gauge_shot", None)
        .add_plugins(gauge::GaugePlugin)
        .insert_resource(common::Shots::new(
            &[
                (70, "/tmp/gauge_level.png"),
                (130, "/tmp/gauge_bank.png"),
                (190, "/tmp/gauge_slip.png"),
            ],
            210,
        ))
        .add_systems(Update, (set_condition, common::drive_shots).chain())
        .run();
}

/// Reset the aircraft just before each screenshot, so the cluster is captured
/// at a known reading (minimal drift in the few frames before the shot).
fn set_condition(mut sim: ResMut<sim::Sim>, f: Res<common::Frame>) {
    match f.0 {
        65 => sim.reset_air(3_827.0, 90.0),
        125 => {
            // A banked, climbing, turning attitude: the horizon displaces and
            // rolls, the heading card swings, the turn symbol banks.
            common::set_ic(&mut sim, 3_000.0, 100.0, 60.0);
            sim.model.set("ic/phi-deg", 25.0);
            sim.model.set("ic/theta-deg", 8.0);
            sim.model.set("ic/r-rad_sec", 3.0_f64.to_radians());
            sim.model.apply_ic();
        }
        185 => {
            // Wings level but yawing through the air (a sideslip): the turn
            // symbol sits level while the ball rolls off centre.
            common::set_ic(&mut sim, 3_000.0, 100.0, 0.0);
            sim.model.set("ic/beta-deg", 8.0);
            sim.model.apply_ic();
        }
        _ => {}
    }
}
