//! Renders the sky plugin alone from four useful altitudes/directions, so the
//! cloud look can be compared against the previous screenshots in `.tmp/`.
//!
//! Run: `cargo run --example cloud_shot` (set `RAIN=1` for the storm deck).

#[path = "../src/cloud_camera.rs"]
mod cloud_camera;

#[path = "../src/sky.rs"]
mod sky;
#[path = "../src/weather.rs"]
mod weather;

use bevy::core_pipeline::tonemapping::Tonemapping;
use bevy::prelude::*;
use bevy::render::view::window::screenshot::{save_to_disk, Screenshot};
use bevy::winit::WinitSettings;

#[derive(Resource, Default)]
struct Frame(u32);

#[derive(Resource)]
struct SunDir(Vec3);

/// Poses to render, one per `HOLD` frames. `POSES` overrides the defaults with
/// `x,y,z,dx,dy,dz;...`, so a density sweep needs one process, not ten.
#[derive(Resource)]
struct Poses(Option<Vec<(Vec3, Vec3)>>);

/// Frames per pose. The first shot must land after the WESL pipeline has
/// compiled, otherwise it captures Bevy's fallback material.
const HOLD: u32 = 40;

fn parse_poses() -> Option<Vec<(Vec3, Vec3)>> {
    let spec = std::env::var("POSES").ok()?;
    Some(
        spec.split(';')
            .filter_map(|pose| {
                let v: Vec<f32> = pose
                    .split(',')
                    .map(|n| n.trim().parse().ok())
                    .collect::<Option<_>>()?;
                (v.len() == 6)
                    .then(|| (Vec3::new(v[0], v[1], v[2]), Vec3::new(v[3], v[4], v[5])))
            })
            .collect(),
    )
}

fn main() {
    let rain = if std::env::var("RAIN").is_ok_and(|v| v == "1") {
        0.9
    } else {
        0.0
    };
    let poses = parse_poses();

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "cloud_shot".into(),
                resolution: (1000, 640).into(),
                ..default()
            }),
            ..default()
        }))
        .insert_resource(WinitSettings::continuous())
        .insert_resource(weather::Weather { rain })
        .add_plugins(sky::SkyPlugin)
        .insert_resource(Poses(poses))
        .add_systems(Startup, setup)
        .init_resource::<Frame>()
        .add_systems(Update, (pose, shot))
        .run();
}

fn setup(mut commands: Commands) {
    // The archipelago's afternoon sun, so the shots match the mission.
    let rot = Quat::from_euler(EulerRot::XYZ, -0.62, -0.65, 0.0);
    let sun = (rot * Vec3::Z).normalize();
    commands.insert_resource(SunDir(sun));

    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(0.0, 700.0, 0.0),
        Tonemapping::None,
    ));
    commands.spawn((
        DirectionalLight::default(),
        Transform::from_rotation(rot),
    ));
}

fn pose(
    frame: Res<Frame>,
    poses: Res<Poses>,
    sun: Res<SunDir>,
    mut cam: Query<&mut Transform, (With<Camera3d>, Without<cloud_camera::CloudCamera>)>,
) {
    let Ok(mut t) = cam.single_mut() else {
        return;
    };
    let (pos, dir) = match &poses.0 {
        Some(list) => {
            let Some(&(pos, dir)) = list.get(frame.0 as usize / HOLD as usize) else {
                return;
            };
            (pos, dir)
        }
        None => match frame.0 {
            0..=39 => (Vec3::new(0.0, 700.0, 0.0), Vec3::new(0.0, 0.55, -0.83)),
            40..=79 => (Vec3::new(0.0, 2_400.0, 0.0), Vec3::new(0.0, 0.0, -1.0)),
            80..=119 => (Vec3::new(0.0, 8_000.0, 0.0), Vec3::new(0.0, -0.45, -0.89)),
            _ => (Vec3::new(0.0, 1_200.0, 0.0), sun.0),
        },
    };
    *t = Transform::from_translation(pos).looking_to(dir, Vec3::Y);
}

fn shot(
    mut commands: Commands,
    mut frame: ResMut<Frame>,
    poses: Res<Poses>,
    mut exit: MessageWriter<AppExit>,
) {
    frame.0 += 1;
    let (path, done) = match &poses.0 {
        Some(list) => {
            let idx = frame.0 as usize / HOLD as usize;
            let path = (frame.0 % HOLD == 38 && idx < list.len()).then(|| {
                let out = std::env::var("OUT").unwrap_or_else(|_| "/tmp/".into());
                format!("{out}cloud_pose_{idx}.png")
            });
            (path, frame.0 == list.len() as u32 * HOLD + 10)
        }
        None => {
            let path = match frame.0 {
                40 => Some("/tmp/cloud_low.png".into()),
                80 => Some("/tmp/cloud_inside.png".into()),
                120 => Some("/tmp/cloud_above.png".into()),
                160 => Some("/tmp/cloud_sun.png".into()),
                _ => None,
            };
            (path, frame.0 == 170)
        }
    };
    if let Some(path) = path {
        commands
            .spawn(Screenshot::primary_window())
            .observe(save_to_disk(path.clone()));
        info!("screenshot -> {path}");
    }
    if done {
        exit.write(AppExit::Success);
    }
}
