//! Calibration harness for the cockpit control-stand hit region.
//!
//! The cockpit crosshair has to open the stand when it is aimed at the
//! pedestal. The stand is a physical surface, so the target is a rectangle on
//! a horizontal plane in the cockpit model's frame, not a screen rect: it must
//! stay put while the pilot looks around.
//!
//! This example looks down at the pedestal and draws the candidate rectangle
//! (projected through the same lens the game uses) in red. Nudge `PED_*` until
//! the frame sits on the throttle quadrant, then read the constants off the
//! top of this file.
//!
//! Run: `cargo run --features shots --example pedestal_calib` and open
//! `/tmp/pedestal_calib.png`.

mod common;

use bevy::prelude::*;
use my_flight_sim::{cockpit, mcp_ui, pfd, view};

/// Top surface of the pedestal, in cockpit-model metres.
const PED_Y: f32 = 1.52;
/// Console bounds on that plane (+X left, +Z nose).
const PED_X: (f32, f32) = (-0.18, 0.06);
const PED_Z: (f32, f32) = (1.00, 1.58);

/// Downward look that brings the pedestal under the crosshair.
const LOOK_YAW_DEG: f32 = -52.0;
const LOOK_PITCH_DEG: f32 = -60.0;

#[derive(Component)]
struct CalibFrame;

fn main() {
    std::env::set_var("AIRCRAFT", "737");
    std::env::set_var("FDM_BACKEND", "rust");
    common::app("pedestal_calib", Some((1200, 800)))
        .insert_resource(ClearColor(Color::srgb(0.02, 0.03, 0.05)))
        .add_plugins((
            mcp_ui::McpUiPlugin,
            pfd::PfdPlugin,
            cockpit::CockpitPlugin,
            my_flight_sim::lights::LightsPlugin,
        ))
        .insert_resource(common::Shots::new(&[(140, "/tmp/pedestal_calib.png")], 170))
        .insert_resource(my_flight_sim::map::ActiveMap::default())
        .insert_resource(my_flight_sim::map::SunPose::default())
        .insert_resource(my_flight_sim::airfield::Airfield::default())
        .init_resource::<my_flight_sim::mission::ActiveLesson>()
        .init_resource::<my_flight_sim::taxi::Taximeter>()
        .add_systems(Startup, spawn_frame)
        .add_systems(Update, (force_look, project_frame, common::drive_shots).chain())
        .run();
}

fn spawn_frame(mut commands: Commands) {
    commands.spawn((
        CalibFrame,
        Node {
            position_type: PositionType::Absolute,
            border: UiRect::all(Val::Px(2.0)),
            ..default()
        },
        BorderColor::all(Color::srgb(1.0, 0.15, 0.10)),
        BackgroundColor(Color::NONE),
        GlobalZIndex(40),
    ));
}

fn force_look(mut ui: ResMut<view::Ui>) {
    ui.cockpit = true;
    ui.look_yaw = LOOK_YAW_DEG.to_radians();
    ui.look_pitch = LOOK_PITCH_DEG.to_radians();
}

fn project_frame(
    ui: Res<view::Ui>,
    mut q: Query<&mut Node, With<CalibFrame>>,
    cameras: Query<&Camera, (With<Camera3d>, Without<my_flight_sim::cloud_camera::CloudCamera>)>,
) {
    let Some(eye) = view::cockpit_eye_model("737", &view::CockpitTune::default()) else {
        return;
    };
    let Some(yaw) = view::cockpit_model_yaw("737") else {
        return;
    };
    let eye = Vec3::from_array(eye);
    let rot = Quat::from_rotation_y(-yaw) * view::look_rotation(&ui);
    let inv = rot.inverse();
    let Ok(camera) = cameras.single() else {
        return;
    };
    let Some(size) = camera.logical_viewport_size() else {
        return;
    };
    let tan_half = (std::f32::consts::FRAC_PI_4 * 0.5).tan();
    let aspect = size.x / size.y;
    let mut min = Vec2::splat(f32::MAX);
    let mut max = Vec2::splat(f32::MIN);
    for (x, z) in [
        (PED_X.0, PED_Z.0),
        (PED_X.1, PED_Z.0),
        (PED_X.0, PED_Z.1),
        (PED_X.1, PED_Z.1),
    ] {
        let d = inv * (Vec3::new(x, PED_Y, z) - eye);
        let depth = -d.z;
        if depth <= 0.05 {
            return;
        }
        let ndc = Vec2::new(d.x / (depth * tan_half * aspect), d.y / (depth * tan_half));
        let px = Vec2::new((ndc.x * 0.5 + 0.5) * size.x, (0.5 - ndc.y * 0.5) * size.y);
        min = min.min(px);
        max = max.max(px);
    }
    if let Ok(mut node) = q.single_mut() {
        node.left = Val::Px(min.x);
        node.top = Val::Px(min.y);
        node.width = Val::Px(max.x - min.x);
        node.height = Val::Px(max.y - min.y);
    }
}
