//! Interactive calibration harness for the 737 pilot-eye cockpit view.
//!
//! Runs the real cockpit scene (model, PFDs, crosshair) and lets you slide the
//! pilot eye and change the lens while watching the result live. The numbers in
//! the overlay are exactly what `view.rs`'s `cockpit_spec("737")` uses: the eye
//! is in the cockpit model's own frame (+X left, +Y up, +Z toward the nose) and
//! `+Z` moves you toward the instrument panel.
//!
//! Controls:
//!   Up / Down       move the eye forward / back (+Z / -Z)
//!   Left / Right    move the eye left / right (+X / -X)
//!   PgUp / PgDn     move the eye up / down (+Y / -Y)
//!   [ / ]           narrow / widen the lens (FOV)
//!   Del             reset eye, FOV and head-look to the current spec
//!   P               print the pose and write /tmp/cockpit_eye.txt
//!   Mouse           free-look (the crosshair stays fixed at screen centre)
//!
//! Run: `cargo run --example cockpit_eye_calib`

#[path = "../src/aircraft_profile.rs"]
mod aircraft_profile;
#[path = "../src/app_state.rs"]
mod app_state;
#[path = "../src/autopilot.rs"]
mod autopilot;
#[path = "../src/cloud_camera.rs"]
mod cloud_camera;
#[path = "../src/cockpit.rs"]
mod cockpit;
#[path = "../src/config.rs"]
mod config;
#[path = "../src/input.rs"]
mod input;
#[path = "../src/mcp_ui.rs"]
mod mcp_ui;
#[path = "../src/pfd.rs"]
mod pfd;
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
use bevy::text::FontSize;
use bevy::winit::WinitSettings;

/// The shipped 737 cockpit pose, kept in sync with `cockpit_spec("737")`.
const DEFAULT_EYE: [f32; 3] = [0.2801, 2.06, 0.9355];
const DEFAULT_YAW_DEG: f32 = -11.76;
const DEFAULT_PITCH_DEG: f32 = -2.92;
const EYE_SPEED: f32 = 0.5;
const FOV_SPEED_DEG: f32 = 25.0;
const FOV_MIN_DEG: f32 = 25.0;
const FOV_MAX_DEG: f32 = 90.0;

#[derive(Component)]
struct CalibLabel;

fn main() {
    std::env::set_var("AIRCRAFT", "737");
    std::env::set_var("FDM_BACKEND", "rust");
    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "737 cockpit eye calibration".into(),
                resolution: (1400, 900).into(),
                ..default()
            }),
            ..default()
        }))
        .insert_resource(ClearColor(Color::srgb(0.02, 0.03, 0.05)))
        .insert_resource(WinitSettings::continuous())
        .insert_resource(Time::<Fixed>::from_hz(1.0 / sim::DT))
        .add_plugins((
            sim::SimPlugin,
            autopilot::AutopilotPlugin,
            config::ConfigPlugin,
            input::InputPlugin,
            view::ViewPlugin,
            mcp_ui::McpUiPlugin,
            pfd::PfdPlugin,
            cockpit::CockpitPlugin,
            weather::WeatherPlugin,
        ))
        .insert_state(app_state::AppState::Flying)
        .add_systems(
            OnEnter(app_state::AppState::Flying),
            (view::spawn_flight_aircraft, view::reset_control_rig).chain(),
        )
        .add_systems(Startup, (init_cockpit, spawn_overlay))
        .add_systems(Update, force_cockpit)
        .add_systems(Update, tune.before(view::follow_camera))
        .add_systems(Update, update_overlay.after(tune))
        .run();
}

fn init_cockpit(mut ui: ResMut<view::Ui>) {
    ui.cockpit = true;
    ui.look_yaw = DEFAULT_YAW_DEG.to_radians();
    ui.look_pitch = DEFAULT_PITCH_DEG.to_radians();
}

fn force_cockpit(mut ui: ResMut<view::Ui>) {
    ui.cockpit = true;
}

fn spawn_overlay(mut commands: Commands) {
    commands
        .spawn((
            Node {
                position_type: PositionType::Absolute,
                top: Val::Px(16.0),
                left: Val::Px(16.0),
                padding: UiRect::all(Val::Px(12.0)),
                ..default()
            },
            BackgroundColor(Color::srgba(0.0, 0.0, 0.0, 0.6)),
            GlobalZIndex(100),
        ))
        .with_children(|panel| {
            panel.spawn((
                CalibLabel,
                Text::new(""),
                TextFont {
                    font_size: FontSize::Px(15.0),
                    ..default()
                },
                TextColor(Color::srgb(0.85, 0.95, 1.0)),
                TextLayout::new(Justify::Left, LineBreak::NoWrap),
            ));
        });
}

/// Move the eye/lens every frame; the override starts from the shipped pose so
/// the first frame already matches what the game shows.
fn tune(
    keys: Res<ButtonInput<KeyCode>>,
    time: Res<Time>,
    mut ui: ResMut<view::Ui>,
    mut tune: ResMut<view::CockpitTune>,
) {
    let mut eye = tune.eye.unwrap_or(DEFAULT_EYE);
    let mut fov = tune.fov.unwrap_or(std::f32::consts::FRAC_PI_4);
    let dt = time.delta_secs();

    let step = EYE_SPEED * dt;
    if keys.pressed(KeyCode::ArrowUp) {
        eye[2] += step;
    }
    if keys.pressed(KeyCode::ArrowDown) {
        eye[2] -= step;
    }
    if keys.pressed(KeyCode::ArrowLeft) {
        eye[0] += step;
    }
    if keys.pressed(KeyCode::ArrowRight) {
        eye[0] -= step;
    }
    if keys.pressed(KeyCode::PageUp) {
        eye[1] += step;
    }
    if keys.pressed(KeyCode::PageDown) {
        eye[1] -= step;
    }

    let fov_step = FOV_SPEED_DEG.to_radians() * dt;
    if keys.pressed(KeyCode::BracketLeft) {
        fov -= fov_step;
    }
    if keys.pressed(KeyCode::BracketRight) {
        fov += fov_step;
    }
    fov = fov.clamp(FOV_MIN_DEG.to_radians(), FOV_MAX_DEG.to_radians());

    if keys.just_pressed(KeyCode::Delete) {
        eye = DEFAULT_EYE;
        fov = std::f32::consts::FRAC_PI_4;
        ui.look_yaw = DEFAULT_YAW_DEG.to_radians();
        ui.look_pitch = DEFAULT_PITCH_DEG.to_radians();
    }

    if keys.just_pressed(KeyCode::KeyP) {
        let report = format!(
            "eye=[{:.4}, {:.4}, {:.4}]\nfov_deg={:.2}\nlook_yaw_deg={:.2}\nlook_pitch_deg={:.2}\n",
            eye[0],
            eye[1],
            eye[2],
            fov.to_degrees(),
            ui.look_yaw.to_degrees(),
            ui.look_pitch.to_degrees(),
        );
        info!("COCKPIT_EYE\n{report}");
        if let Err(e) = std::fs::write("/tmp/cockpit_eye.txt", &report) {
            warn!("could not write /tmp/cockpit_eye.txt: {e}");
        }
    }

    tune.eye = Some(eye);
    tune.fov = Some(fov);
}

fn update_overlay(
    tune: Res<view::CockpitTune>,
    ui: Res<view::Ui>,
    mut label: Query<&mut Text, With<CalibLabel>>,
) {
    let eye = tune.eye.unwrap_or(DEFAULT_EYE);
    let fov = tune.fov.unwrap_or(std::f32::consts::FRAC_PI_4);
    let text = format!(
        "737 COCKPIT EYE CALIB\n\
         eye  x {:+8.3}   y {:+8.3}   z {:+8.3}   (z = toward panel)\n\
         fov  {:5.1} deg   look  yaw {:+.1}  pitch {:+.1} deg\n\
         \n\
         Up/Down fwd/back   Left/Right left/right   PgUp/PgDn up/down\n\
         [ ] fov   Del reset   P print pose + /tmp/cockpit_eye.txt",
        eye[0],
        eye[1],
        eye[2],
        fov.to_degrees(),
        ui.look_yaw.to_degrees(),
        ui.look_pitch.to_degrees(),
    );
    if let Ok(mut t) = label.single_mut() {
        if t.0 != text {
            t.0 = text;
        }
    }
}
