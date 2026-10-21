//! Calibration harness for the cockpit-panel gauge overlay.
//!
//! In cockpit view the model's instrument panel sits at a fixed place in the
//! airframe, so the live UI gauges can be seated onto the painted dials. To do
//! that at any camera angle (free look!), each gauge needs a 3D anchor in the
//! body frame rather than a screen position. This harness turns the on-screen
//! dial centres into those anchors:
//!
//!   1. run once -> `/tmp/panel_calib_raw.png` (the untouched cockpit view),
//!   2. measure each painted dial's centre/radius there (fractions of the
//!      image), write them to `/tmp/dials.txt` as `name fx fy rfx rfy`,
//!   3. run again -> the fractions are ray-cast onto the panel plane, the
//!      body-frame anchor + in-plane basis of each dial is printed, and bright
//!      markers are dropped at the anchors so `/tmp/panel_calib_marked.png`
//!      shows whether they landed on the painted dials.
//!
//! Run: `cargo run --example panel_calib`

#[path = "../src/aircraft_profile.rs"]
mod aircraft_profile;
#[path = "../src/app_state.rs"]
mod app_state;
#[path = "../src/autopilot.rs"]
mod autopilot;
#[path = "../src/cloud_camera.rs"]
mod cloud_camera;
#[path = "../src/config.rs"]
mod config;
#[path = "../src/input.rs"]
mod input;
#[path = "../src/sim.rs"]
mod sim;
#[path = "../src/view.rs"]
mod view;
#[path = "../src/weather.rs"]
mod weather;
#[path = "../src/aerodrome.rs"]
mod aerodrome;
#[path = "../src/ils.rs"]
mod ils;
#[path = "../src/lights.rs"]
mod lights;
#[path = "../src/map.rs"]
mod map;
#[path = "../src/mission.rs"]
mod mission;
#[path = "../src/sky.rs"]
mod sky;
#[path = "../src/taxi.rs"]
mod taxi;
#[path = "../src/ui_theme.rs"]
mod ui_theme;

use bevy::prelude::*;
use bevy::render::view::window::screenshot::{save_to_disk, Screenshot};
use bevy::winit::WinitSettings;

#[derive(Resource, Default)]
struct Frame(u32);

/// Fallback dial list (fractions of the image + radius fractions), used until
/// `/tmp/dials.txt` is written. Name matches the gauge `setup` module. These are
/// the measured values the shipped [`PanelDial`] table was derived from.
const DEFAULT_DIALS: &[(&str, f32, f32, f32, f32)] = &[
    ("airspeed", 0.3051, 0.5872, 0.0594, 0.0475),
    ("vsi", 0.4387, 0.5872, 0.0594, 0.0475),
    ("altimeter", 0.5746, 0.5834, 0.0594, 0.0475),
    ("tach", 0.7105, 0.5834, 0.0594, 0.0475),
    ("heading", 0.5727, 0.6994, 0.0598, 0.0478),
    ("turn", 0.5828, 0.8250, 0.0594, 0.0475),
    ("attitude", 0.3031, 0.8131, 0.0602, 0.0481),
];

/// Panel plane in the `AircraftRoot` body frame: a point and a normal, chosen
/// to sit on the model's instrument panel (near-vertical, face toward the
/// pilot's eyes). Only affects parallax under free look — the design-pose
/// screen placement is exact whatever plane is used.
const PANEL_POINT: Vec3 = Vec3::new(0.0, 0.05, -0.675);
const PANEL_NORMAL: Vec3 = Vec3::new(0.0, 0.08, 0.997);

fn main() {
    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "panel_calib".into(),
                // Same window config as `sim_view_shot`, so the measured
                // fractions and the rays agree.
                resolution: (1200, 800).into(),
                ..default()
            }),
            ..default()
        }))
        .insert_resource(WinitSettings::continuous())
        .insert_resource(Time::<Fixed>::from_hz(1.0 / sim::DT))
        .add_plugins((
            sim::SimPlugin,
            autopilot::AutopilotPlugin,
            config::ConfigPlugin,
            input::InputPlugin,
            view::ViewPlugin,
            weather::WeatherPlugin,
        ))
        .insert_state(app_state::AppState::Flying)
        .add_systems(
            OnEnter(app_state::AppState::Flying),
            (view::spawn_flight_aircraft, view::reset_control_rig).chain(),
        )
        .init_resource::<Frame>()
        .add_systems(Startup, force_cockpit)
        .add_systems(Update, force_cockpit)
        .add_systems(Update, shots)
        .run();
}

fn force_cockpit(mut ui: ResMut<view::Ui>) {
    ui.cockpit = true;
}

/// The dial list, from `/tmp/dials.txt` if present, else the built-in default.
fn dials() -> Vec<(String, f32, f32, f32, f32)> {
    if let Ok(txt) = std::fs::read_to_string("/tmp/dials.txt") {
        let mut v = Vec::new();
        for line in txt.lines() {
            let line = line.trim();
            if line.is_empty() || line.starts_with('#') {
                continue;
            }
            let p: Vec<&str> = line.split_whitespace().collect();
            if p.len() >= 5 {
                if let (Ok(a), Ok(b), Ok(c), Ok(d)) =
                    (p[1].parse(), p[2].parse(), p[3].parse(), p[4].parse())
                {
                    v.push((p[0].to_string(), a, b, c, d));
                }
            }
        }
        if !v.is_empty() {
            return v;
        }
    }
    DEFAULT_DIALS
        .iter()
        .map(|(n, a, b, c, d)| (n.to_string(), *a, *b, *c, *d))
        .collect()
}

/// Intersect a ray with the (world) panel plane.
fn hit_plane(ray: Ray3d, point: Vec3, normal: Vec3) -> Option<Vec3> {
    let denom = ray.direction.dot(normal);
    if denom.abs() < 1e-6 {
        return None;
    }
    let t = (point - ray.origin).dot(normal) / denom;
    if t <= 0.0 {
        return None;
    }
    Some(ray.origin + *ray.direction * t)
}

fn shots(
    mut commands: Commands,
    mut f: ResMut<Frame>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut exit: MessageWriter<AppExit>,
    cam: Query<(&Camera, &GlobalTransform), With<Camera3d>>,
    root: Query<(Entity, &GlobalTransform), With<view::AircraftRoot>>,
) {
    f.0 += 1;
    match f.0 {
        120 => {
            commands
                .spawn(Screenshot::primary_window())
                .observe(save_to_disk("/tmp/panel_calib_raw.png"));
            info!("raw screenshot -> /tmp/panel_calib_raw.png");
        }
        200 => {
            let Ok((camera, cam_tf)) = cam.single() else {
                return;
            };
            let Ok((root_e, root_tf)) = root.single() else {
                return;
            };
            let Some(size) = camera.logical_viewport_size() else {
                return;
            };
            let inv = root_tf.affine().inverse();
            let p_w = root_tf.transform_point(PANEL_POINT);
            let n_w = root_tf.rotation() * PANEL_NORMAL;

            let mesh = meshes.add(Sphere::new(0.006).mesh().uv(8, 6));
            let mat = materials.add(StandardMaterial {
                base_color: Color::srgb(1.0, 0.0, 0.0),
                unlit: true,
                ..default()
            });

            for (name, fx, fy, rfx, rfy) in dials() {
                let at = |x: f32, y: f32| -> Option<Vec3> {
                    let ray = camera
                        .viewport_to_world(cam_tf, Vec2::new(x * size.x, y * size.y))
                        .ok()?;
                    hit_plane(ray, p_w, n_w).map(|h| inv.transform_point(h))
                };
                let (Some(anchor), Some(u), Some(v)) =
                    (at(fx, fy), at(fx + rfx, fy), at(fx, fy - rfy))
                else {
                    warn!("{name}: ray missed the panel plane");
                    continue;
                };
                info!(
                    "ANCHOR {name} p=[{:.4},{:.4},{:.4}] u=[{:.4},{:.4},{:.4}] v=[{:.4},{:.4},{:.4}]",
                    anchor.x, anchor.y, anchor.z,
                    (u - anchor).x, (u - anchor).y, (u - anchor).z,
                    (v - anchor).x, (v - anchor).y, (v - anchor).z,
                );
                commands.spawn((
                    Mesh3d(mesh.clone()),
                    MeshMaterial3d(mat.clone()),
                    Transform::from_translation(anchor),
                    ChildOf(root_e),
                ));
            }
        }
        280 => {
            commands
                .spawn(Screenshot::primary_window())
                .observe(save_to_disk("/tmp/panel_calib_marked.png"));
            info!("marked screenshot -> /tmp/panel_calib_marked.png");
        }
        340 => {
            exit.write(AppExit::Success);
        }
        _ => {}
    }
}
