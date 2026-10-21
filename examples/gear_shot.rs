//! Flies the F-22 airborne with the gear down, then commands it up and
//! screenshots each state from a low front-quarter view that follows the jet
//! (the parked interlock won't retract on the ground, so this has to fly).
//!
//! Run: `FDM_BACKEND=rust AIRCRAFT=f22 cargo run --features shots --example gear_shot`

mod common;

use bevy::prelude::*;
use my_flight_sim::{input, sim};

fn main() {
    common::app("gear_shot", Some((1200, 800)))
        .insert_resource(common::Shots::new(
            &[
                (100, "/tmp/gear_down.png"),
                (120, "/tmp/gear_mid.png"),
                (480, "/tmp/gear_transit.png"),
                (1100, "/tmp/gear_up.png"),
            ],
            1140,
        ))
        .add_systems(Startup, launch)
        .add_systems(Last, drive)
        .add_systems(Update, common::drive_shots)
        .add_systems(PostUpdate, side_cam)
        .run();
}

/// Start airborne so the gear can leave the wells.
fn launch(mut sim: ResMut<sim::Sim>) {
    sim.reset_air(8000.0, 300.0);
}

/// Fixed chase view: off the jet's left, low, looking slightly up at the belly.
fn side_cam(rp: Res<sim::RenderPose>, mut q: Query<&mut Transform, With<Camera3d>>) {
    let Ok(mut t) = q.single_mut() else { return };
    if !rp.init {
        return;
    }
    let pos = rp.curr_pos;
    let yaw = Quat::from_rotation_y(-rp.curr_heading);
    // 18 m to the side, 3 m below the CG, 4 m ahead.
    let eye = pos + yaw * Vec3::new(18.0, -3.0, -2.0);
    *t = Transform::from_translation(eye).looking_at(pos, Vec3::Y);
}

fn drive(
    frame: Res<common::Frame>,
    mut bindings: ResMut<input::Bindings>,
    mut sim: ResMut<sim::Sim>,
) {
    for b in &mut bindings.list {
        b.value = match b.property {
            "fcs/throttle-cmd-norm" => 0.75,
            _ => 0.0,
        };
    }
    if frame.0 == 120 {
        sim.model.set("gear/gear-cmd-norm", 0.0);
        info!("gear up commanded");
    }
}
