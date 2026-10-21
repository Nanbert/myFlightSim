//! Interactive AH-64D cockpit viewer.
//!
//! Loads `assets/ah64d_cockpit.glb`, puts the camera at the modelled pilot eye
//! and lets you look around / move inside the cockpit. Useful while iterating
//! on `tools/ah64_cockpit.py`: regenerate the GLB in another terminal, then
//! press F5 here to force a reload.
//!
//! Run: `cargo run --features shots --example ah64_cockpit_view`
//!
//! Controls:
//!   RMB drag .... look
//!   W A S D ..... move (camera-relative)
//!   E / Q ....... up / down
//!   Shift/Ctrl .. faster / slower
//!   1 .. 6 ...... forward / panel / left / right / canopy / floor presets
//!   R ........... reset to the pilot eye
//!   F5 .......... reload the GLB from disk
//!   G ........... hide/show canopy glass
//!   P ........... screenshot to /tmp/opencode/ah64_cockpit_view.png
//!   Esc ......... quit

use bevy::input::mouse::MouseMotion;
use bevy::prelude::*;
use bevy::render::view::window::screenshot::{save_to_disk, Screenshot};
use bevy::text::FontSize;
use bevy::winit::WinitSettings;

const EYE: Vec3 = Vec3::ZERO;
const START_YAW_DEG: f32 = 0.0;
const START_PITCH_DEG: f32 = -6.0;
const LOOK_SENS: f32 = 0.003;
const MOVE_SPEED: f32 = 1.2;
const FAST_MULT: f32 = 3.0;
const SLOW_MULT: f32 = 0.25;

#[derive(Resource)]
struct Look {
    yaw: f32,
    pitch: f32,
}

impl Default for Look {
    fn default() -> Self {
        Self {
            yaw: START_YAW_DEG.to_radians(),
            pitch: START_PITCH_DEG.to_radians(),
        }
    }
}

#[derive(Resource)]
struct ModelRoot(Entity);

#[derive(Resource)]
struct ReportTimer(Timer);

fn main() {
    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title:
                    "AH-64D cockpit viewer  —  RMB look · WASD move · F5 reload · Esc quit".into(),
                resolution: (1600, 900).into(),
                ..default()
            }),
            ..default()
        }))
        .insert_resource(ClearColor(Color::srgb(0.38, 0.57, 0.82)))
        .insert_resource(WinitSettings::continuous())
        .init_resource::<Look>()
        .insert_resource(ReportTimer(Timer::from_seconds(1.0, TimerMode::Repeating)))
        .add_systems(Startup, setup)
        .add_systems(
            Update,
            (
                look,
                fly,
                presets,
                reload_asset,
                toggle_glass,
                screenshot,
                report,
                quit,
            ),
        )
        .run();
}

fn setup(
    mut commands: Commands,
    assets: Res<AssetServer>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let root = commands
        .spawn((
            Transform::default(),
            Visibility::default(),
            WorldAssetRoot(assets.load(GltfAssetLabel::Scene(0).from_asset("ah64d_cockpit.glb"))),
        ))
        .id();
    commands.insert_resource(ModelRoot(root));

    commands.spawn((
        Camera3d::default(),
        Projection::Perspective(PerspectiveProjection {
            fov: 60_f32.to_radians(),
            near: 0.005,
            far: 20_000.0,
            ..default()
        }),
        bevy::core_pipeline::tonemapping::Tonemapping::None,
        Transform::from_translation(EYE)
            .with_rotation(look_rotation(START_YAW_DEG, START_PITCH_DEG)),
    ));

    // The cockpit is deliberately dark. These are the same sort of fills the
    // Blender preview uses; the game can replace them with its own lighting.
    commands.spawn((
        DirectionalLight {
            illuminance: 10_000.0,
            shadow_maps_enabled: false,
            ..default()
        },
        Transform::from_rotation(Quat::from_euler(EulerRot::XYZ, -0.55, 0.65, 0.0)),
    ));
    commands.spawn(AmbientLight {
        color: Color::srgb(0.68, 0.75, 0.88),
        brightness: 700.0,
        ..default()
    });
    for (pos, intensity, color) in [
        (
            Vec3::new(0.0, 0.55, -0.35),
            26_000.0,
            Color::srgb(1.0, 0.94, 0.86),
        ),
        (
            Vec3::new(0.0, -0.10, 0.30),
            12_000.0,
            Color::srgb(0.86, 0.92, 1.0),
        ),
    ] {
        commands.spawn((
            PointLight {
                intensity,
                range: 4.0,
                shadow_maps_enabled: false,
                color,
                ..default()
            },
            Transform::from_translation(pos),
        ));
    }

    commands.spawn((
        Mesh3d(meshes.add(Plane3d::default().mesh().size(8_000.0, 8_000.0))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.33, 0.30, 0.21),
            perceptual_roughness: 0.95,
            ..default()
        })),
        Transform::from_xyz(0.0, -60.0, 0.0),
    ));

    commands.spawn((
        Text::new(
            "AH-64D cockpit viewer\n\
             RMB drag: look   WASD: move   E/Q: up/down   Shift/Ctrl: speed\n\
             1-6: forward / panel / left / right / canopy / floor\n\
             R: reset   F5: reload GLB   G: glass   P: screenshot   Esc: quit",
        ),
        TextFont {
            font_size: FontSize::Px(14.0),
            ..default()
        },
        TextColor(Color::srgb(0.90, 0.94, 0.88)),
        Node {
            position_type: PositionType::Absolute,
            top: Val::Px(12.0),
            left: Val::Px(14.0),
            padding: UiRect::all(Val::Px(6.0)),
            ..default()
        },
        BackgroundColor(Color::srgba(0.0, 0.0, 0.0, 0.35)),
    ));

    println!("--- AH-64D cockpit viewer ---");
    println!("RMB drag look | WASD move | E/Q up-down | 1-6 presets | R reset | F5 reload | G glass | P shot | Esc quit");
    println!("asset frame: eye at origin, +X left, +Y up, +Z nose");
}

fn look_rotation(yaw_deg: f32, pitch_deg: f32) -> Quat {
    // Bevy cameras look down -Z. A 180° yaw turns that down the model's +Z
    // (nose); positive yaw then looks to the pilot's left.
    Quat::from_euler(
        EulerRot::YXZ,
        std::f32::consts::PI + yaw_deg.to_radians(),
        pitch_deg.to_radians(),
        0.0,
    )
}

fn look(
    mouse: Res<ButtonInput<MouseButton>>,
    mut motion: MessageReader<MouseMotion>,
    mut look: ResMut<Look>,
) {
    if mouse.pressed(MouseButton::Right) {
        for ev in motion.read() {
            look.yaw -= ev.delta.x * LOOK_SENS;
            look.pitch = (look.pitch - ev.delta.y * LOOK_SENS).clamp(-1.48, 1.48);
        }
    } else {
        motion.clear();
    }
}

fn fly(
    keys: Res<ButtonInput<KeyCode>>,
    time: Res<Time>,
    look: Res<Look>,
    mut camera: Query<&mut Transform, With<Camera3d>>,
) {
    let Ok(mut transform) = camera.single_mut() else {
        return;
    };

    let rot = look_rotation(look.yaw.to_degrees(), look.pitch.to_degrees());
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
        transform.translation += dir.normalize() * speed * time.delta_secs();
    }
    transform.rotation = rot;
}

fn presets(keys: Res<ButtonInput<KeyCode>>, mut look: ResMut<Look>) {
    let (yaw, pitch): (f32, f32) = if keys.just_pressed(KeyCode::Digit1) {
        (0.0, -6.0)
    } else if keys.just_pressed(KeyCode::Digit2) {
        (0.0, -16.0)
    } else if keys.just_pressed(KeyCode::Digit3) {
        (48.0, -8.0)
    } else if keys.just_pressed(KeyCode::Digit4) {
        (-48.0, -8.0)
    } else if keys.just_pressed(KeyCode::Digit5) {
        (0.0, 42.0)
    } else if keys.just_pressed(KeyCode::Digit6) {
        (0.0, -48.0)
    } else {
        return;
    };
    look.yaw = yaw.to_radians();
    look.pitch = pitch.to_radians();
}

fn reload_asset(
    keys: Res<ButtonInput<KeyCode>>,
    assets: Res<AssetServer>,
    mut commands: Commands,
    mut root: ResMut<ModelRoot>,
) {
    if !keys.just_pressed(KeyCode::F5) {
        return;
    }
    commands.entity(root.0).despawn();
    assets.reload("ah64d_cockpit.glb");
    let new_root = commands
        .spawn((
            Transform::default(),
            Visibility::default(),
            WorldAssetRoot(assets.load(GltfAssetLabel::Scene(0).from_asset("ah64d_cockpit.glb"))),
        ))
        .id();
    root.0 = new_root;
    println!("reloading ah64d_cockpit.glb ...");
}

fn toggle_glass(keys: Res<ButtonInput<KeyCode>>, mut nodes: Query<(&Name, &mut Visibility)>) {
    if !keys.just_pressed(KeyCode::KeyG) {
        return;
    }
    for (name, mut visibility) in &mut nodes {
        if name.as_str() == "CANOPY_GLASS" {
            *visibility = if *visibility == Visibility::Hidden {
                Visibility::Inherited
            } else {
                Visibility::Hidden
            };
            println!(
                "canopy glass: {}",
                if *visibility == Visibility::Hidden {
                    "hidden"
                } else {
                    "visible"
                }
            );
        }
    }
}

fn screenshot(keys: Res<ButtonInput<KeyCode>>, mut commands: Commands) {
    if keys.just_pressed(KeyCode::KeyP) {
        let path = "/tmp/opencode/ah64_cockpit_view.png";
        commands
            .spawn(Screenshot::primary_window())
            .observe(save_to_disk(path));
        println!("screenshot -> {path}");
    }
}

fn report(
    time: Res<Time>,
    mut timer: ResMut<ReportTimer>,
    camera: Query<&Transform, With<Camera3d>>,
) {
    timer.0.tick(time.delta());
    if !timer.0.just_finished() {
        return;
    }
    let Ok(t) = camera.single() else {
        return;
    };
    let p = t.translation;
    let (yaw, pitch, _) = t.rotation.to_euler(EulerRot::YXZ);
    let yaw_deg = (((yaw - std::f32::consts::PI).to_degrees() + 180.0).rem_euclid(360.0)) - 180.0;
    println!(
        "eye=({:6.3}, {:6.3}, {:6.3})  yaw={:6.1}  pitch={:6.1}  (model frame, +Z nose)",
        p.x,
        p.y,
        p.z,
        yaw_deg,
        pitch.to_degrees(),
    );
}

fn quit(keys: Res<ButtonInput<KeyCode>>, mut exit: MessageWriter<AppExit>) {
    if keys.just_pressed(KeyCode::Escape) {
        exit.write(AppExit::Success);
    }
}
