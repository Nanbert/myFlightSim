//! Cockpit-view viewer: loads the big C172 model (interior **and** exterior,
//! `assets/c172_full.glb`) on its own with a free-fly camera, so the pilot-eye
//! framing can be dialled in by hand. Prints the camera pose once a second —
//! paste those numbers back and they can be hard-coded.
//!
//! Controls: right-drag = look, W/A/S/D = move, E/Q = up/down, Shift = faster,
//! P = screenshot to `/tmp/cockpit_view.png`, Esc = quit.
//!
//! Run: `cargo run --example cockpit_shot`

use bevy::input::mouse::MouseMotion;
use bevy::prelude::*;
use bevy::render::view::window::screenshot::{save_to_disk, Screenshot};
use bevy::winit::WinitSettings;

/// Mouse-look state. Kept out of the `Transform` so pitch can't gimbal-lock
/// against the yaw we read back each frame.
#[derive(Resource)]
struct Look {
    /// Rotation about +Y. The model's nose points +Z, so π faces down the nose.
    yaw: f32,
    /// Rotation about the camera's right axis, clamped just shy of ±90°.
    pitch: f32,
}

/// Cadence for the pose printout.
#[derive(Resource)]
struct ReportTimer(Timer);

const LOOK_SENS: f32 = 0.003;
const MOVE_SPEED: f32 = 1.5;
const FAST_MULT: f32 = 5.0;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "cockpit_view — right-drag look, WASD move, E/Q up-down".into(),
                resolution: (1280, 800).into(),
                ..default()
            }),
            ..default()
        }))
        .insert_resource(ClearColor(Color::srgb(0.42, 0.62, 0.88)))
        // Keep rendering while the window is unfocused (the compositor may steal
        // focus on launch; game mode would otherwise starve the loop to ~1 Hz).
        .insert_resource(WinitSettings::continuous())
        .insert_resource(Look {
            yaw: std::f32::consts::PI,
            pitch: 0.0,
        })
        .insert_resource(ReportTimer(Timer::from_seconds(1.0, TimerMode::Repeating)))
        .add_systems(Startup, setup)
        .add_systems(Update, (fly, report, screenshot, quit))
        .run();
}

fn setup(
    mut commands: Commands,
    assets: Res<AssetServer>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    // Model frame (measured): metres, X = span, Y = up, Z = fuselage, nose +Z,
    // origin in the cabin. The pose below was dialled in by hand in the viewer
    // (right of centre, high, slightly back of the seat) — it reads the panel
    // and the windshield together.
    commands.spawn((
        Camera3d::default(),
        Projection::Perspective(PerspectiveProjection {
            far: 20_000.0,
            ..default()
        }),
        bevy::core_pipeline::tonemapping::Tonemapping::None,
        Transform::from_xyz(0.209, 1.835, -0.350)
            .with_rotation(Quat::from_euler(EulerRot::YXZ, std::f32::consts::PI, 0.0, 0.0)),
        AmbientLight {
            color: Color::WHITE,
            brightness: 400.0,
            ..default()
        },
    ));

    commands.spawn((
        DirectionalLight {
            illuminance: 6_000.0,
            shadow_maps_enabled: false,
            ..default()
        },
        Transform::from_rotation(Quat::from_euler(EulerRot::XYZ, -0.9, 0.4, 0.0)),
    ));
    commands.spawn((
        DirectionalLight {
            illuminance: 3_000.0,
            shadow_maps_enabled: false,
            ..default()
        },
        Transform::from_rotation(Quat::from_euler(EulerRot::XYZ, -2.4, -0.3, 0.0)),
    ));

    // Ground at y = 0 (the model rests its wheels there) so there's a horizon.
    commands.spawn((
        Mesh3d(meshes.add(Plane3d::default().mesh().size(40_000.0, 40_000.0))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.30, 0.42, 0.24),
            ..default()
        })),
    ));

    commands.spawn(WorldAssetRoot(
        assets.load(GltfAssetLabel::Scene(0).from_asset("c172_full.glb")),
    ));

    println!("--- cockpit_view ---");
    println!("right-drag look | W/A/S/D move | E/Q up-down | Shift faster | P shot | Esc quit");
    println!("pose is printed once a second; model frame is metres, nose = +Z");
}

fn fly(
    keys: Res<ButtonInput<KeyCode>>,
    mouse: Res<ButtonInput<MouseButton>>,
    mut motion: MessageReader<MouseMotion>,
    time: Res<Time>,
    mut look: ResMut<Look>,
    mut q: Query<&mut Transform, With<Camera3d>>,
) {
    let Ok(mut t) = q.single_mut() else {
        return;
    };

    if mouse.pressed(MouseButton::Right) {
        for ev in motion.read() {
            look.yaw -= ev.delta.x * LOOK_SENS;
            look.pitch = (look.pitch - ev.delta.y * LOOK_SENS)
                .clamp(-1.5, 1.5);
        }
    } else {
        // Drain anyway so a later press doesn't replay stale motion.
        motion.clear();
    }

    let rot = Quat::from_euler(EulerRot::YXZ, look.yaw, look.pitch, 0.0);
    let (fwd, right) = (rot * Vec3::NEG_Z, rot * Vec3::X);

    let mut dir = Vec3::ZERO;
    if keys.pressed(KeyCode::KeyW) {
        dir += fwd;
    }
    if keys.pressed(KeyCode::KeyS) {
        dir -= fwd;
    }
    if keys.pressed(KeyCode::KeyD) {
        dir += right;
    }
    if keys.pressed(KeyCode::KeyA) {
        dir -= right;
    }
    if keys.pressed(KeyCode::KeyE) {
        dir += Vec3::Y;
    }
    if keys.pressed(KeyCode::KeyQ) {
        dir -= Vec3::Y;
    }

    let speed = if keys.pressed(KeyCode::ShiftLeft) || keys.pressed(KeyCode::ShiftRight) {
        MOVE_SPEED * FAST_MULT
    } else {
        MOVE_SPEED
    };
    if dir != Vec3::ZERO {
        t.translation += dir.normalize() * speed * time.delta_secs();
    }
    t.rotation = rot;
}

/// Print `pos=(x, y, z)` plus the yaw/pitch/roll and the forward vector, once a
/// second — grab the line you want and hand it over.
fn report(
    time: Res<Time>,
    mut timer: ResMut<ReportTimer>,
    q: Query<&Transform, With<Camera3d>>,
) {
    timer.0.tick(time.delta());
    if !timer.0.just_finished() {
        return;
    }
    let Ok(t) = q.single() else {
        return;
    };
    let p = t.translation;
    let (yaw, pitch, roll) = t.rotation.to_euler(EulerRot::YXZ);
    let f = t.rotation * Vec3::NEG_Z;
    println!(
        "pos=({:7.3},{:7.3},{:7.3})  yaw={:8.2} pitch={:7.2} roll={:6.2}  fwd=({:6.3},{:6.3},{:6.3})",
        p.x,
        p.y,
        p.z,
        yaw.to_degrees(),
        pitch.to_degrees(),
        roll.to_degrees(),
        f.x,
        f.y,
        f.z,
    );
}

fn screenshot(
    keys: Res<ButtonInput<KeyCode>>,
    mut commands: Commands,
) {
    if keys.just_pressed(KeyCode::KeyP) {
        let path = "/tmp/cockpit_view.png";
        commands
            .spawn(Screenshot::primary_window())
            .observe(save_to_disk(path));
        println!("screenshot -> {path}");
    }
}

fn quit(keys: Res<ButtonInput<KeyCode>>, mut exit: MessageWriter<AppExit>) {
    if keys.just_pressed(KeyCode::Escape) {
        exit.write(AppExit::Success);
    }
}
