//! Screenshots the F-22's engine effects (afterburner flame and exhaust heat
//! haze) from a rear-quarter chase view, for eyeballing the anchor placement and
//! tuning.
//!
//! Run: `FDM_BACKEND=rust AIRCRAFT=f22 cargo run --features shots --example fx_shot`
//!
//! `VIEW=side` swaps the camera to a broadside view of the exhaust.

mod common;

use bevy::prelude::*;
use my_flight_sim::{fx, input, sim, weather};

/// `VIEW=side` puts the camera broadside to the exhaust instead of the default
/// rear-quarter view.
#[derive(Resource, Default)]
struct SideView(bool);

fn main() {
    common::app("fx_shot", Some((1200, 800)))
        .add_plugins(fx::FxPlugin)
        .init_resource::<SideView>()
        .insert_resource(common::Shots::new(
            &[
                (60, "/tmp/opencode/fx_level.png"),
                (200, "/tmp/opencode/fx_pull.png"),
                (340, "/tmp/opencode/fx_pull2.png"),
            ],
            380,
        ))
        .add_systems(Startup, (launch, read_view_env))
        .add_systems(Update, clear_weather)
        .add_systems(Last, drive)
        .add_systems(Update, (progress, common::drive_shots))
        .add_systems(PostUpdate, chase_cam)
        .run();
}

fn progress(f: Res<common::Frame>) {
    if f.0.is_multiple_of(100) {
        info!("frame {}", f.0);
    }
}

/// Start airborne so the gear is up and the jet is moving.
fn launch(mut sim: ResMut<sim::Sim>) {
    sim.reset_air(6000.0, 320.0);
}

/// Pin the sky clear: the saved config may have left it raining, and rain on the
/// lens would obscure the effects we're here to look at.
fn clear_weather(mut weather: ResMut<weather::Weather>) {
    weather.rain = 0.0;
}

/// `VIEW=side` switches the camera to a broadside view.
fn read_view_env(mut side: ResMut<SideView>) {
    side.0 = std::env::var("VIEW").is_ok_and(|v| v == "side");
}

/// Full throttle throughout, so the burner stays lit for the shot.
fn drive(mut bindings: ResMut<input::Bindings>) {
    for b in &mut bindings.list {
        b.value = match b.property {
            "fcs/throttle-cmd-norm" => 1.0,
            _ => 0.0,
        };
    }
}

/// Rear-quarter (default) or side-on, slightly above: the nozzles stay in view.
/// `VIEW=side` puts the camera broadside to the exhaust.
fn chase_cam(
    rp: Res<sim::RenderPose>,
    side: Res<SideView>,
    mut q: Query<&mut Transform, With<Camera3d>>,
) {
    let Ok(mut t) = q.single_mut() else { return };
    if !rp.init {
        return;
    }
    let local = if side.0 {
        Vec3::new(44.0, 3.0, 4.0)
    } else {
        Vec3::new(11.0, 6.0, 34.0)
    };
    let pos = rp.curr_pos;
    let yaw = Quat::from_rotation_y(-rp.curr_heading);
    *t = Transform::from_translation(pos + yaw * local).looking_at(pos, Vec3::Y);
}
