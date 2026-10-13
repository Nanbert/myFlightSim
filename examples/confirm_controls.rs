//! Confirmation harness for the rigged `cessna_172.glb`.
//!
//! Loads the glTF scene, finds the control-surface *bones* by name, and drives
//! them so we can visually verify the rigid skin moves the ailerons (plus
//! elevator / rudder / flaps / prop). Independent of the JSBSim app.
//!
//! Two screenshots are written: `_rest.png` (bind pose, before driving) and
//! `_def.png` (driven). Compare them to confirm the deflection.
//!
//! Run: `cargo run --example confirm_controls`

use bevy::prelude::*;
use bevy::render::view::window::screenshot::{save_to_disk, Screenshot};
use std::collections::HashMap;

const MODEL: &str = "cessna_172.glb";
const REST_SHOT: &str = "/tmp/c172_rest.png";
const DEF_SHOT: &str = "/tmp/c172_deflected.png";

/// (bone name, hinge axis in the bone's LOCAL frame, deflection radians).
/// The local axes were measured from the glTF bind pose (see session notes).
const BONES: &[(&str, [f32; 3], f32)] = &[
    ("aileron-left_3", [0.0419, -0.0981, 0.9943], 0.60), // +TE down -> roll right
    ("aileron-right_2", [-0.0388, 0.1054, 0.9937], -0.60), // +TE down -> roll left
    ("elevator_0", [0.0, 0.0, 1.0], 0.45),              // +TE down
    ("rudder_4", [-1.0, 0.018, 0.004], 0.50),           // +TE left (swept hinge)
    ("flaps_1", [0.0, 0.0, 1.0], 0.50),                 // +TE down
    ("prop_5", [0.0, 0.0, -1.0], 0.0),                  // spun continuously
];

/// Rest (bind-pose) rotations captured once, so we always apply the hinge
/// delta on top of the original pose rather than overwriting it.
#[derive(Resource, Default)]
struct Rest(HashMap<Entity, Quat>);

#[derive(Resource, Default)]
struct Frame(u32);

fn main() {
    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "confirm controls — cessna_172.glb".into(),
                resolution: (1200, 800).into(),
                ..default()
            }),
            ..default()
        }))
        .insert_resource(ClearColor(Color::srgb(0.09, 0.11, 0.15)))
        .init_resource::<Rest>()
        .init_resource::<Frame>()
        .add_systems(Startup, setup)
        .add_systems(
            Update,
            (
                capture_rest,
                log_bones,
                drive_bones,
                screenshot_then_quit,
            )
                .chain(),
        )
        .run();
}

fn setup(mut commands: Commands, assets: Res<AssetServer>) {
    // Behind and above the tail, looking forward along the fuselage: the
    // trailing-edge surfaces (both ailerons, elevator, rudder) read clearly.
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(-16.0, 4.0, 0.0).looking_at(Vec3::new(0.0, 1.3, 0.0), Vec3::Y),
        AmbientLight {
            color: Color::WHITE,
            brightness: 400.0,
            ..default()
        },
    ));
    commands.spawn((
        DirectionalLight {
            illuminance: 9_000.0,
            shadow_maps_enabled: false,
            ..default()
        },
        Transform::from_rotation(Quat::from_euler(EulerRot::XYZ, -0.6, -0.7, 0.0)),
    ));
    commands.spawn((
        WorldAssetRoot(assets.load(GltfAssetLabel::Scene(0).from_asset(MODEL))),
        Transform::default(),
    ));
}

/// Capture each bone's original rotation the first frame it exists.
fn capture_rest(names: Query<(Entity, &Name, &Transform)>, mut rest: ResMut<Rest>) {
    if !rest.0.is_empty() {
        return;
    }
    for (e, n, t) in &names {
        if BONES.iter().any(|(bn, _, _)| *bn == n.as_str()) {
            rest.0.insert(e, t.rotation);
        }
    }
}

/// One-shot dump of every skeleton-node name once the scene is loaded, so the
/// run is self-documenting and we can confirm our name matches are right.
fn log_bones(names: Query<&Name>, rest: Res<Rest>, mut done: Local<bool>) {
    if *done || rest.0.is_empty() {
        return;
    }
    *done = true;

    let mut skel: Vec<&str> = names
        .iter()
        .map(|n| n.as_str())
        .filter(|s| {
            ["bone", "aileron", "elevator", "rudder", "prop", "flap"]
                .iter()
                .any(|k| s.contains(k))
        })
        .collect();
    skel.sort_unstable();
    info!("skeleton-ish nodes ({}):", skel.len());
    for s in &skel {
        info!("  {s}");
    }
    info!("captured {} rest bones (of {} wanted)", rest.0.len(), BONES.len());
}

fn drive_bones(
    time: Res<Time>,
    rest: Res<Rest>,
    frame: Res<Frame>,
    mut names: Query<(&Name, Entity, &mut Transform)>,
) {
    // Hold at bind pose for the first stretch so `_rest.png` is truly at rest.
    if frame.0 < 60 {
        return;
    }
    let t = time.elapsed_secs();
    for (name, e, mut tf) in &mut names {
        let Some((_, axis, defl)) = BONES.iter().find(|(bn, _, _)| *bn == name.as_str()) else {
            continue;
        };
        let Some(base) = rest.0.get(&e) else { continue };
        let axis = Vec3::from_array(*axis).normalize();
        let angle = if name.as_str() == "prop_5" { t * 25.0 } else { *defl };
        tf.rotation = *base * Quat::from_axis_angle(axis, angle);
    }
}

fn screenshot_then_quit(
    mut commands: Commands,
    mut frame: ResMut<Frame>,
    mut exit: MessageWriter<AppExit>,
) {
    frame.0 += 1;
    if frame.0 == 50 {
        commands
            .spawn(Screenshot::primary_window())
            .observe(save_to_disk(REST_SHOT));
        info!("rest screenshot -> {REST_SHOT}");
    }
    if frame.0 == 90 {
        commands
            .spawn(Screenshot::primary_window())
            .observe(save_to_disk(DEF_SHOT));
        info!("deflected screenshot -> {DEF_SHOT}");
    }
    if frame.0 == 130 {
        exit.write(AppExit::Success);
    }
}
