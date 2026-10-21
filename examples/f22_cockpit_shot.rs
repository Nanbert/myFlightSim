//! F-22 cockpit-eye tuner: loads `f22_cockpit.glb` with the same transform the
//! sim uses and free-flies a camera inside it. The camera is parented to the
//! model, so the printed position is in the model's own units — exactly the
//! `eye` vector in `view::cockpit_spec` (10 units per metre).
//!
//! Controls: right-drag = look, W/A/S/D = move, E/Q = up/down,
//! Shift = faster, Ctrl = slower, P = screenshot, Esc = quit.
//!
//! Run: `cargo run --features shots --example f22_cockpit_shot`

use bevy::input::mouse::MouseMotion;
use bevy::prelude::*;
use bevy::render::view::window::screenshot::{save_to_disk, Screenshot};
use bevy::winit::WinitSettings;

/// Must match `view::cockpit_spec("f22")`.
const SCALE: f32 = 0.1;
const YAW: f32 = std::f32::consts::FRAC_PI_2;
const OFFSET: Vec3 = Vec3::new(0.0, 0.0, 4.77);
/// Starting point — the last tuned pose, so re-tuning resumes where it left off.
const START_EYE: Vec3 = Vec3::new(96.09, 11.22, 0.12);
/// Resting look yaw offset (degrees, positive = left) and pitch (negative =
/// down), matching `eye_yaw` / `eye_pitch` in the spec.
const START_YAW_DEG: f32 = -2.1;
const START_PITCH_DEG: f32 = -3.9;

/// Camera yaw/pitch in the model's frame. -90° yaw looks along the nose (+X).
#[derive(Resource)]
struct Look {
    yaw: f32,
    pitch: f32,
}

#[derive(Resource)]
struct ReportTimer(Timer);

const LOOK_SENS: f32 = 0.003;
const MOVE_SPEED: f32 = 10.0;
const FAST_MULT: f32 = 5.0;
const SLOW_MULT: f32 = 0.25;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "f22 cockpit eye — right-drag look, WASD move, E/Q up-down".into(),
                resolution: (1280, 800).into(),
                ..default()
            }),
            ..default()
        }))
        .insert_resource(ClearColor(Color::srgb(0.42, 0.62, 0.88)))
        .insert_resource(WinitSettings::continuous())
        .insert_resource(Look {
            yaw: -std::f32::consts::FRAC_PI_2 + START_YAW_DEG.to_radians(),
            pitch: START_PITCH_DEG.to_radians(),
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
    // Model root with the sim's exact transform. The camera is a child so its
    // local Transform stays in model units.
    let model = commands
        .spawn((
            Transform {
                translation: OFFSET,
                rotation: Quat::from_rotation_y(YAW),
                scale: Vec3::splat(SCALE),
            },
            Visibility::default(),
            WorldAssetRoot(assets.load(GltfAssetLabel::Scene(0).from_asset("f22_cockpit.glb"))),
        ))
        .id();

    commands.spawn((
        Camera3d::default(),
        Projection::Perspective(PerspectiveProjection {
            far: 20_000.0,
            ..default()
        }),
        bevy::core_pipeline::tonemapping::Tonemapping::None,
        Transform::from_translation(START_EYE).with_rotation(Quat::from_euler(
            EulerRot::YXZ,
            -std::f32::consts::FRAC_PI_2 + START_YAW_DEG.to_radians(),
            START_PITCH_DEG.to_radians(),
            0.0,
        )),
        ChildOf(model),
    ));

    commands.spawn((AmbientLight {
        color: Color::WHITE,
        brightness: 500.0,
        ..default()
    },));
    commands.spawn((
        DirectionalLight {
            illuminance: 7_000.0,
            shadow_maps_enabled: false,
            ..default()
        },
        Transform::from_rotation(Quat::from_euler(EulerRot::XYZ, -0.7, 0.3, 0.0)),
    ));

    // Wheels sit ~2.05 m below the model origin, so this gives a horizon at the
    // right height.
    commands.spawn((
        Mesh3d(meshes.add(Plane3d::default().mesh().size(4_000.0, 4_000.0))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.30, 0.42, 0.24),
            ..default()
        })),
        Transform::from_xyz(0.0, -2.05, 0.0),
    ));

    println!("--- f22 cockpit eye tuner ---");
    println!(
        "right-drag look | WASD move | E/Q up-down | Shift fast | Ctrl slow | P shot | Esc quit"
    );
    println!(
        "model frame: +X nose, +Y up, +Z right; {:.0} units = 1 m",
        1.0 / SCALE
    );
    println!("paste the printed eye line into view.rs::cockpit_spec(\"f22\")");
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
            look.pitch = (look.pitch - ev.delta.y * LOOK_SENS).clamp(-1.5, 1.5);
        }
    } else {
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

    let fast = keys.pressed(KeyCode::ShiftLeft) || keys.pressed(KeyCode::ShiftRight);
    let slow = keys.pressed(KeyCode::ControlLeft) || keys.pressed(KeyCode::ControlRight);
    let speed = if fast {
        MOVE_SPEED * FAST_MULT
    } else if slow {
        MOVE_SPEED * SLOW_MULT
    } else {
        MOVE_SPEED
    };
    if dir != Vec3::ZERO {
        t.translation += dir.normalize() * speed * time.delta_secs();
    }
    t.rotation = rot;
}

/// Prints the camera's model-local position once a second. That vector is the
/// `eye` value; the yaw offset is relative to straight ahead (0 = down the nose).
fn report(time: Res<Time>, mut timer: ResMut<ReportTimer>, q: Query<&Transform, With<Camera3d>>) {
    timer.0.tick(time.delta());
    if !timer.0.just_finished() {
        return;
    }
    let Ok(t) = q.single() else {
        return;
    };
    let p = t.translation;
    let (yaw, pitch, _) = t.rotation.to_euler(EulerRot::YXZ);
    println!(
        "eye=({:7.2}, {:7.2}, {:7.2})  yaw_offset={:6.1} pitch={:6.1}",
        p.x,
        p.y,
        p.z,
        (yaw + std::f32::consts::FRAC_PI_2).to_degrees(),
        pitch.to_degrees(),
    );
}

fn screenshot(keys: Res<ButtonInput<KeyCode>>, mut commands: Commands) {
    if keys.just_pressed(KeyCode::KeyP) {
        let path = "/tmp/f22_cockpit_shot.png";
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
