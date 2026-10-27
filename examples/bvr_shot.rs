//! Screenshots the AIM-120's exhaust trail: the eject, the ignition bloom and
//! the long white column, for eyeballing width, density and fade.
//!
//! Run: `cargo run --features shots --example bvr_shot`
//!
//! No combat mission is needed. The example stands up a sky, then fires a
//! *synthetic* round carrying a real [`missile_smoke::Trail`] and accelerates it
//! on the AMRAAM's numbers. The camera is a photo ship that pulls out and away as
//! the column grows — viewed down the round's own flight path the trail
//! foreshortens into a single blob, so the interesting view is from the side, far
//! enough back that the whole streak is in frame. Shots land in `/tmp/opencode/`.

use bevy::camera::Hdr;
use bevy::core_pipeline::tonemapping::Tonemapping;
use bevy::post_process::bloom::Bloom;
use bevy::prelude::*;
use bevy::render::view::window::screenshot::{save_to_disk, Screenshot};
use bevy::render::view::{DebandDither, Msaa};
use bevy::window::WindowMode;
use bevy::winit::WinitSettings;
use my_flight_sim::{app_state, missile_smoke, sky, weather};

/// A stand-in for `combat::Missile`: it exists only to carry a `Transform` and a
/// `Trail`, and to fly like the real round so the smoke lays out the same way.
#[derive(Component)]
struct Round {
    speed: f32,
}

/// Frame the round leaves the jet. Everything else is relative to it.
const LAUNCH_FRAME: u32 = 60;

/// Where the jet would be, and the heading it is flying.
const ORIGIN: Vec3 = Vec3::new(0.0, 2_500.0, 0.0);
const HEADING: Vec3 = Vec3::NEG_Z;
/// Across the flight path, for the photo ship to sit on.
const ACROSS: Vec3 = Vec3::X;

/// Frame counter for the launch trigger; `Last` systems advance it, so `Update`
/// sees the previous value.
#[derive(Resource, Default)]
struct Frame(u32);

/// Sim time the round left, so the capture script can key off seconds since the
/// trigger rather than frames (the window renders at whatever rate it likes).
#[derive(Resource, Default)]
struct Launched(Option<f32>);

/// Index of the next shot to take.
#[derive(Resource, Default)]
struct Next(usize);

fn main() {
    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "bvr_shot".into(),
                mode: WindowMode::Windowed,
                resolution: (1280, 800).into(),
                ..default()
            }),
            ..default()
        }))
        // Keep stepping while the window is unfocused, or the compositor
        // throttles the frame-counted script to a crawl.
        .insert_resource(WinitSettings::continuous())
        .insert_state(app_state::AppState::Flying)
        .init_resource::<Frame>()
        .init_resource::<Launched>()
        .init_resource::<Next>()
        .add_plugins((
            weather::WeatherPlugin,
            sky::SkyPlugin,
            missile_smoke::MissileSmokePlugin,
        ))
        .add_systems(Startup, setup)
        .add_systems(Update, fire)
        .add_systems(Update, fly.after(fire))
        .add_systems(Update, photo_ship.after(fly))
        .add_systems(Last, shot)
        .run();
}

/// Sky, sun, a sea for a horizon, and the camera.
fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut mats: ResMut<Assets<StandardMaterial>>,
) {
    commands.spawn((
        Camera3d::default(),
        Hdr,
        Tonemapping::TonyMcMapface,
        DebandDither::Enabled,
        Bloom::NATURAL,
        Msaa::Sample4,
        Projection::Perspective(PerspectiveProjection {
            fov: std::f32::consts::FRAC_PI_4,
            ..default()
        }),
        Transform::from_translation(ORIGIN + ACROSS * 400.0 + Vec3::Y * 200.0)
            .looking_at(ORIGIN, Vec3::Y),
    ));

    commands.spawn((
        DirectionalLight {
            illuminance: 12_000.0,
            ..default()
        },
        Transform::from_rotation(Quat::from_rotation_x(-0.55) * Quat::from_rotation_y(0.8)),
    ));
    commands.spawn(AmbientLight {
        color: Color::srgb(0.75, 0.82, 0.92),
        brightness: 900.0,
        ..default()
    });

    // A sea to give the column a horizon to cross. Twelve kilometres a side, far
    // enough that its edge is under the haze.
    commands.spawn((
        Mesh3d(meshes.add(Plane3d::default().mesh().size(60_000.0, 60_000.0))),
        MeshMaterial3d(mats.add(StandardMaterial {
            base_color: Color::srgb(0.16, 0.28, 0.38),
            perceptual_roughness: 0.35,
            ..default()
        })),
        Transform::from_xyz(0.0, 0.0, 0.0),
    ));
    commands.insert_resource(ClearColor(Color::srgb(0.45, 0.60, 0.80)));
}

/// Fire the round at the trigger frame: the jet's speed, plus the AMRAAM's eject
/// charge, plus the downward kick that drops it clear before the light.
fn fire(
    mut commands: Commands,
    time: Res<Time>,
    f: Res<Frame>,
    mut launched: ResMut<Launched>,
    mut done: Local<bool>,
) {
    if *done || f.0 < LAUNCH_FRAME {
        return;
    }
    *done = true;
    launched.0 = Some(time.elapsed_secs());
    let vel = HEADING * 330.0 + HEADING * 14.0 - Vec3::Y * 2.5;
    commands.spawn((
        Round {
            speed: vel.length(),
        },
        missile_smoke::Trail::new(missile_smoke::BVR_TRAIL),
        Transform::from_translation(ORIGIN + HEADING * 2.0 - Vec3::Y * 2.0)
            .with_rotation(Quat::from_rotation_arc(Vec3::NEG_Z, HEADING)),
    ));
}

/// The AMRAAM's motor: 300 m/s² for six seconds, capped at 1150 m/s.
fn fly(time: Res<Time>, mut q: Query<(&mut Round, &mut Transform)>) {
    let dt = time.delta_secs();
    for (mut round, mut tf) in &mut q {
        round.speed = (round.speed + 300.0 * dt).min(1150.0);
        tf.translation += HEADING * round.speed * dt;
    }
}

/// Sit off the trail's flank, far enough back that its whole length is in frame,
/// and pull out as it grows — the shot a chase plane would get.
fn photo_ship(
    round: Query<&Transform, With<Round>>,
    mut cam: Query<&mut Transform, (With<Camera3d>, Without<Round>)>,
) {
    let Ok(head) = round.single() else { return };
    let Ok(mut cam) = cam.single_mut() else { return };
    let len = (head.translation - ORIGIN).length().max(40.0);
    let mid = ORIGIN + HEADING * (len * 0.45);
    // 1.3× the half-length (perpendicular) is a ~42° chord: just inside the
    // lens, so the column fills the frame corner to corner without clipping.
    let eye = mid + ACROSS * (len * 0.85 + 90.0) + Vec3::Y * (len * 0.10 + 18.0);
    *cam = Transform::from_translation(eye).looking_at(mid, Vec3::Y);
}

/// Seconds after the trigger at which to grab a frame: the eject, the ignition
/// bloom, then the column growing and its tail thinning out.
const SHOTS: &[(f32, &str)] = &[
    (0.10, "eject"),
    (0.30, "ignite"),
    (0.70, "early"),
    (1.50, "one"),
    (3.00, "three"),
    (5.00, "five"),
    (7.00, "seven"),
    (9.00, "nine"),
];

fn shot(
    mut commands: Commands,
    mut f: ResMut<Frame>,
    time: Res<Time>,
    launched: Res<Launched>,
    mut next: ResMut<Next>,
    mut exit: MessageWriter<AppExit>,
) {
    f.0 += 1;
    let Some(t0) = launched.0 else { return };
    let t = time.elapsed_secs() - t0;
    while let Some(&(at, tag)) = SHOTS.get(next.0) {
        if t < at {
            break;
        }
        next.0 += 1;
        let path = format!("/tmp/opencode/bvr_{tag}.png");
        commands
            .spawn(Screenshot::primary_window())
            .observe(save_to_disk(path.clone()));
        info!("screenshot -> {path} at {t:.2}s");
    }
    if next.0 == SHOTS.len() + 1 {
        exit.write(AppExit::Success);
    }
}
