//! Flies the F-22 through a few attitudes and screenshots the HUD, for eyeballing
//! the symbology.
//!
//! Run: `AIRCRAFT=f22 cargo run --features shots --example hud_shot`

mod common;

use bevy::prelude::*;
use my_flight_sim::{hud, sim};

fn main() {
    common::app("hud_shot", Some((1200, 800)))
        .add_plugins(hud::HudPlugin)
        .insert_resource(common::Shots::new(
            &[
                (200, "/tmp/hud_level.png"),
                (300, "/tmp/hud_bank.png"),
                (440, "/tmp/hud_gear.png"),
            ],
            480,
        ))
        .add_systems(Update, (set_condition, common::drive_shots).chain())
        .run();
}

/// Rewrite the state just before each capture so the reading is known.
fn set_condition(mut sim: ResMut<sim::Sim>, f: Res<common::Frame>) {
    match f.0 {
        170 => sim.reset_air(20_000.0, 420.0),
        255 => {
            // Banked and climbing: the ladder and horizon roll, the bank pointer
            // swings, the FPM walks off the boresight.
            common::set_ic(&mut sim, 18_000.0, 380.0, 120.0);
            sim.model.set("ic/phi-deg", 32.0);
            sim.model.set("ic/theta-deg", 9.0);
            sim.model.apply_ic();
        }
        365 => {
            // Gear down, slow: the approach-angle bar appears beside the FPM.
            common::set_ic(&mut sim, 3_000.0, 165.0, 0.0);
            sim.model.set("ic/theta-deg", 8.0);
            sim.model.apply_ic();
            sim.model.set("gear/gear-cmd-norm", 1.0);
            sim.model.set("gear/gear-pos-norm", 1.0);
        }
        _ => {}
    }
}
