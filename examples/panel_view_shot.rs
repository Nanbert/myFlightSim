//! Cockpit-panel render check.
//!
//! Runs the real app systems (including the live instruments) in the cockpit
//! view of the full C172, and screenshots the result — used to verify that the
//! gauges seat onto the model's painted panel (`examples/panel_calib` measures
//! the dial positions this relies on).
//!
//! `COCKPIT=0` renders the usual bottom-corner layout instead, for comparison.
//! `LOOK=<deg>` / `PITCH=<deg>` set the cockpit head-look (the same fields the
//! mouse drives): the panel slides across the screen and the gauges must stay
//! on their dials.
//! `FOCUS=<name>` forces the instrument close-up (airspeed, altimeter, …) so a
//! screenshot shows the zoomed lens.
//! Run: `BEVY_ASSET_ROOT="$PWD" cargo run --example panel_view_shot`

#[path = "../src/aerodrome.rs"]
mod aerodrome;
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
#[path = "../src/gauge/mod.rs"]
mod gauge;
#[path = "../src/ils.rs"]
mod ils;
#[path = "../src/input.rs"]
mod input;
#[path = "../src/lights.rs"]
mod lights;
#[path = "../src/map.rs"]
mod map;
#[path = "../src/sim.rs"]
mod sim;
#[path = "../src/sky.rs"]
mod sky;
#[path = "../src/taxi.rs"]
mod taxi;
#[path = "../src/view.rs"]
mod view;
#[path = "../src/weather.rs"]
mod weather;
#[path = "../src/mission.rs"]
mod mission;
#[path = "../src/ui_theme.rs"]
mod ui_theme;

use bevy::prelude::*;
use bevy::render::view::window::screenshot::{save_to_disk, Screenshot};
use bevy::winit::WinitSettings;

#[derive(Resource, Default)]
struct Frame(u32);

fn main() {
    let cockpit = std::env::var("COCKPIT").map(|v| v != "0").unwrap_or(true);
    let out = std::env::var("OUT").unwrap_or_else(|_| "/tmp/panel_view.png".into());
    let look: f32 = std::env::var("LOOK")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(0.0);
    let pitch: f32 = std::env::var("PITCH")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(0.0);
    let focus = std::env::var("FOCUS")
        .ok()
        .and_then(|v| gauge::PanelKey::from_name(&v));

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "panel_view_shot".into(),
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
            gauge::GaugePlugin,
            weather::WeatherPlugin,
            sky::SkyPlugin,
            map::MapPlugin,
            taxi::TaxiPlugin,
            aerodrome::AerodromePlugin,
            lights::LightsPlugin,
        ))
        .insert_state(app_state::AppState::Menu)
        .add_systems(Startup, start_flying)
        .add_systems(
            OnEnter(app_state::AppState::Flying),
            (view::spawn_flight_aircraft, view::reset_control_rig).chain(),
        )
        .init_resource::<Frame>()
        .insert_resource(Cockpit(cockpit))
        .insert_resource(Out(out))
        .insert_resource(Look(look))
        .insert_resource(Pitch(pitch))
        .insert_resource(ForceFocus(focus))
        .add_systems(Startup, apply)
        .add_systems(Update, apply.before(view::follow_camera))
        .add_systems(Update, shot)
        .run();
}

#[derive(Resource)]
struct Cockpit(bool);

/// Transition after the one-shot startup systems (sky, camera, UI) have run;
/// their command buffers are applied at the end of `Startup`, so a mission's
/// `OnEnter(Flying)` sees `CloudField` and friends.
fn start_flying(mut next: ResMut<NextState<app_state::AppState>>) {
    next.set(app_state::AppState::Flying);
}

#[derive(Resource)]
struct Out(String);

/// Head-look yaw to apply through the real cockpit-look field, in degrees.
#[derive(Resource)]
struct Look(f32);

/// Head-look pitch, in degrees.
#[derive(Resource)]
struct Pitch(f32);

/// Instrument to hold in the close-up, if any.
#[derive(Resource)]
struct ForceFocus(Option<gauge::PanelKey>);

fn apply(
    cockpit: Res<Cockpit>,
    look: Res<Look>,
    pitch: Res<Pitch>,
    forced: Res<ForceFocus>,
    focus: Option<ResMut<gauge::focus::InstrumentFocus>>,
    mut ui: ResMut<view::Ui>,
) {
    ui.cockpit = cockpit.0;
    // Drive the same fields the mouse drives, so this exercises the real path.
    ui.look_yaw = look.0.to_radians();
    ui.look_pitch = pitch.0.to_radians();
    if let (Some(key), Some(mut focus)) = (forced.0, focus) {
        focus.hovered = Some(key);
        focus.active = Some(key);
        focus.blend = 1.0;
    }
}

fn shot(
    mut commands: Commands,
    mut f: ResMut<Frame>,
    mut exit: MessageWriter<AppExit>,
    out: Res<Out>,
) {
    f.0 += 1;
    match f.0 {
        200 => {
            commands
                .spawn(Screenshot::primary_window())
                .observe(save_to_disk(out.0.clone()));
        }
        240 => {
            exit.write(AppExit::Success);
        }
        _ => {}
    }
}
