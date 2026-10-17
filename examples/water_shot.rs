//! Renders `assets/shaders/water.wesl` from the heights a mission flies at, for
//! choosing the wave/light tuning by eye.
//!
//! Four tunings × four altitudes -> `/tmp/sea_<variant>_<alt>m.png`.
//! Run: `cargo run --example water_shot`
//!
//! `MOTION=1` instead renders 60-frame clips at 10 m and the 500 m mission
//! minimum for judging the wave animation -> `/tmp/sea_motion/<alt>m_<frame>.png`.

use bevy::camera::Hdr;
use bevy::core_pipeline::tonemapping::Tonemapping;
use bevy::post_process::bloom::{Bloom, BloomCompositeMode, BloomPrefilter};
use bevy::prelude::*;
use bevy::render::render_resource::{AsBindGroup, ShaderType};
use bevy::render::view::window::screenshot::{save_to_disk, Screenshot};
use bevy::shader::ShaderRef;

#[derive(Clone, Copy, ShaderType, Default)]
struct WaterParams {
    camera: Vec4,
    sun: Vec4,
    deep: Vec4,
    fog: Vec4,
    tune: Vec4,
}

#[derive(Asset, TypePath, AsBindGroup, Clone)]
struct WaterMaterial {
    #[uniform(0)]
    params: WaterParams,
}

impl Material for WaterMaterial {
    fn fragment_shader() -> ShaderRef {
        "shaders/water.wesl".into()
    }
    fn enable_prepass() -> bool {
        false
    }
    fn enable_shadows() -> bool {
        false
    }
}

#[derive(Resource)]
struct Water(Handle<WaterMaterial>);

#[derive(Resource, Default)]
struct Frame(u32);

#[derive(Resource, Default)]
struct Motion(bool);

struct Variant {
    name: &'static str,
    /// x = LOD scale, y = swell gain, z = glitter gain, w = glitter exponent.
    tune: Vec4,
}

const VARIANTS: [Variant; 4] = [
    Variant {
        name: "a-baseline",
        tune: Vec4::new(1.0, 1.0, 1.0, 60.0),
    },
    Variant {
        name: "b-closer-detail",
        tune: Vec4::new(0.6, 1.05, 1.1, 65.0),
    },
    Variant {
        name: "c-swell",
        tune: Vec4::new(1.0, 1.5, 1.2, 60.0),
    },
    Variant {
        name: "d-glitter",
        tune: Vec4::new(0.85, 1.1, 1.9, 110.0),
    },
];

/// (eye height m, look-direction y): skimming the sea, low pass, mission
/// minimum, mission spawn. The look angle is the same at every height, so the
/// framing is self-similar and only the resolved wave detail changes.
const ALTS: [(f32, f32); 4] = [
    (10.0, -0.12),
    (80.0, -0.12),
    (500.0, -0.12),
    (1_200.0, -0.12),
];

const START: u32 = 60;
const SHOT_FRAMES: u32 = 12;
const SHOTS: u32 = (VARIANTS.len() * ALTS.len()) as u32;

const MOTION_START: u32 = 40;
const MOTION_ALTS: [f32; 2] = [10.0, 500.0];
const MOTION_FRAMES: u32 = 60;
/// Same 0.12 s of simulated time per saved frame: 7 s of rolling swell.
const MOTION_DT: f32 = 0.12;

/// Sun direction of the archipelago mission (35° elevation, north-west).
const SUN: Vec3 = Vec3::new(-0.492, 0.581, 0.648);

fn main() {
    let motion = std::env::var_os("MOTION").is_some();
    if motion {
        std::fs::create_dir_all("/tmp/sea_motion").unwrap();
    }
    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "water_shot".into(),
                resolution: (1280, 720).into(),
                ..default()
            }),
            ..default()
        }))
        .insert_resource(ClearColor(Color::linear_rgb(0.55, 0.70, 0.87)))
        .insert_resource(Frame::default())
        .insert_resource(Motion(motion))
        .add_plugins(MaterialPlugin::<WaterMaterial>::default())
        .add_systems(Startup, setup)
        .add_systems(Update, (drive, shoot).chain())
        .run();
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut mats: ResMut<Assets<WaterMaterial>>,
) {
    let water = mats.add(WaterMaterial {
        params: WaterParams {
            camera: Vec4::ZERO,
            sun: Vec4::new(SUN.x, SUN.y, SUN.z, 0.0),
            deep: Vec4::new(0.006, 0.028, 0.052, 1.0),
            fog: Vec4::new(0.55, 0.70, 0.87, 0.000032),
            tune: VARIANTS[0].tune,
        },
    });
    commands.insert_resource(Water(water.clone()));

    commands.spawn((
        Mesh3d(meshes.add(Plane3d::default().mesh().size(140_000.0, 140_000.0))),
        MeshMaterial3d(water),
        Transform::default(),
    ));

    commands.spawn((
        Camera3d::default(),
        Projection::Perspective(PerspectiveProjection {
            far: 80_000.0,
            ..default()
        }),
        Tonemapping::None,
        Hdr,
        // Same bloom as the game so the sun glitter is judged fairly.
        Bloom {
            intensity: 0.5,
            low_frequency_boost: 0.35,
            high_pass_frequency: 1.0,
            prefilter: BloomPrefilter {
                threshold: 1.5,
                threshold_softness: 0.5,
            },
            composite_mode: BloomCompositeMode::Additive,
            ..Bloom::NATURAL
        },
        Transform::from_xyz(0.0, ALTS[0].0, 0.0),
    ));
}

/// Which shot the current frame belongs to, or `None` before the first one.
fn shot_at(frame: u32) -> Option<(usize, usize)> {
    let f = frame.checked_sub(START)?;
    let i = f / SHOT_FRAMES;
    if i >= SHOTS {
        return None;
    }
    let i = i as usize;
    Some((i / ALTS.len(), i % ALTS.len()))
}

/// Motion clip: (altitude index, frame index within that clip).
fn motion_at(frame: u32) -> Option<(usize, u32)> {
    let f = frame.checked_sub(MOTION_START)?;
    let i = f / MOTION_FRAMES;
    if i >= MOTION_ALTS.len() as u32 {
        return None;
    }
    Some((i as usize, f % MOTION_FRAMES))
}

fn drive(
    frame: Res<Frame>,
    motion: Res<Motion>,
    water: Res<Water>,
    mut mats: ResMut<Assets<WaterMaterial>>,
    mut cam: Query<&mut Transform, With<Camera3d>>,
) {
    let Ok(mut t) = cam.single_mut() else {
        return;
    };
    if motion.0 {
        let Some((ai, j)) = motion_at(frame.0) else {
            return;
        };
        let eye = MOTION_ALTS[ai];
        let look = Vec3::new(SUN.x, -0.12, SUN.z).normalize();
        *t = Transform::from_xyz(0.0, eye, 0.0).looking_to(look, Vec3::Y);
        if let Some(mut m) = mats.get_mut(&water.0) {
            // The tuning the game ships with.
            m.params.tune = VARIANTS[1].tune;
            m.params.camera = Vec4::new(0.0, eye, 0.0, 20.0 + j as f32 * MOTION_DT);
            m.params.sun = Vec4::new(SUN.x, SUN.y, SUN.z, 0.0);
        }
        return;
    }
    let Some((vi, ai)) = shot_at(frame.0) else {
        return;
    };
    let (eye, look_dy) = ALTS[ai];
    let look = Vec3::new(SUN.x, look_dy, SUN.z).normalize();
    *t = Transform::from_xyz(0.0, eye, 0.0).looking_to(look, Vec3::Y);

    if let Some(mut m) = mats.get_mut(&water.0) {
        m.params.camera = Vec4::new(0.0, eye, 0.0, 24.0);
        m.params.sun = Vec4::new(SUN.x, SUN.y, SUN.z, 0.0);
        m.params.tune = VARIANTS[vi].tune;
    }
}

fn shoot(
    mut commands: Commands,
    mut frame: ResMut<Frame>,
    motion: Res<Motion>,
    mut exit: MessageWriter<AppExit>,
) {
    let now = frame.0;
    frame.0 += 1;

    if motion.0 {
        if let Some((ai, j)) = motion_at(now) {
            let path = format!("/tmp/sea_motion/{}m_{j:03}.png", MOTION_ALTS[ai] as u32);
            commands
                .spawn(Screenshot::primary_window())
                .observe(save_to_disk(path));
        }
        if frame.0 >= MOTION_START + MOTION_ALTS.len() as u32 * MOTION_FRAMES + 20 {
            exit.write(AppExit::Success);
        }
        return;
    }

    if let Some((vi, ai)) = shot_at(now) {
        let f = (now - START) % SHOT_FRAMES;
        if f == SHOT_FRAMES - 3 {
            let path = format!(
                "/tmp/sea_{}_{}m.png",
                VARIANTS[vi].name, ALTS[ai].0 as u32
            );
            commands
                .spawn(Screenshot::primary_window())
                .observe(save_to_disk(path.clone()));
            info!("screenshot -> {path}");
        }
    }
    if frame.0 >= START + SHOTS * SHOT_FRAMES + 20 {
        exit.write(AppExit::Success);
    }
}
