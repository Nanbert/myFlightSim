// Bevy system signatures are the shape of this crate: they routinely take many
// query parameters, and the query types themselves are long. Both lints are
// silenced crate-wide on purpose; hand-written helpers added here should still
// keep their argument lists short.
#![allow(clippy::too_many_arguments, clippy::type_complexity)]

pub mod aerodrome;
pub mod airfield;
pub mod aircraft_profile;
pub mod app_state;
pub mod approach;
pub mod assist;
pub mod autopilot;
pub mod checklist;
pub mod cloud_camera;
pub mod cockpit;
pub mod combat;
pub mod config;
pub mod controls;
pub mod fx;
pub mod g_strain;
pub mod gauge;
pub mod hop;
pub mod hud;
pub mod ils;
pub mod input;
pub mod jet_go_around;
pub mod jet_landing_lesson;
pub mod jet_takeoff_lesson;
pub mod jet_taxi;
pub mod lesson_flow;
pub mod landing;
pub mod landing_lesson;
pub mod lens;
pub mod lights;
pub mod map;
#[cfg(feature = "shots")]
pub mod mcp_bake;
pub mod mcp_layout;
pub mod mcp_panel;
pub mod mcp_ui;
pub mod menu;
pub mod menu_page;
pub mod mission;
pub mod pause;
pub mod pfd;
pub mod scoring;
pub mod sim;
pub mod sky;
pub mod statue;
pub mod takeoff;
pub mod takeoff_lesson;
pub mod taxi;
pub mod tutorial;
pub mod ui_theme;
pub mod vapor;
pub mod view;
pub mod weather;

use bevy::prelude::*;
use bevy::render::view::window::screenshot::{save_to_disk, Screenshot};
use bevy::window::WindowMode;

#[derive(Resource)]
struct ShotTimer(Timer);

impl Default for ShotTimer {
    fn default() -> Self {
        Self(Timer::from_seconds(3.0, TimerMode::Once))
    }
}

pub fn run() {
    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "myFlightSim — Rust + Bevy".into(),
                // The instrument panel is laid out for a wide screen: three
                // gauges against each edge with the attitude indicator centred.
                // A half-width window would overlap the two trios, so start
                // fullscreen (as the sim is meant to be flown).
                mode: WindowMode::BorderlessFullscreen(MonitorSelection::Primary),
                ..default()
            }),
            ..default()
        }))
        .insert_resource(Time::<Fixed>::from_hz(1.0 / sim::DT))
        .add_plugins(ui_theme::UiThemePlugin)
        .add_plugins((
            sim::SimPlugin,
            autopilot::AutopilotPlugin,
            mcp_ui::McpUiPlugin,
            pfd::PfdPlugin,
            // Before `InputPlugin`: it needs `GameConfig` for the startup
            // aircraft's arcade setting.
            config::ConfigPlugin,
            input::InputPlugin,
            view::ViewPlugin,
            fx::FxPlugin,
            vapor::VaporPlugin,
            gauge::GaugePlugin,
            hud::HudPlugin,
            sky::SkyPlugin,
            weather::WeatherPlugin,
            lens::LensPlugin,
            menu::MenuPlugin,
        ))
        .add_plugins(mcp_panel::McpPanelPlugin)
        .add_plugins(g_strain::GStrainPlugin)
        // After the menu: it builds terrain for whatever mission
        // `start_flight` just launched. (A separate call: Bevy's plugin tuples
        // top out at 15.)
        .add_plugins(map::MapPlugin)
        .add_plugins(statue::StatuePlugin)
        .add_plugins(cockpit::CockpitPlugin)
        .add_plugins(controls::ControlPlugin)
        .add_plugins(assist::AssistPlugin)
        .add_plugins(aerodrome::AerodromePlugin)
        .add_plugins(landing::LandingPlugin)
        .add_plugins(takeoff::TakeoffPlugin)
        .add_plugins(takeoff_lesson::TakeoffLessonPlugin)
        .add_plugins(jet_takeoff_lesson::JetTakeoffLessonPlugin)
        .add_plugins(jet_landing_lesson::JetLandingLessonPlugin)
        .add_plugins(jet_go_around::JetGoAroundLessonPlugin)
        .add_plugins(landing_lesson::LandingLessonPlugin)
        .add_plugins(approach::ApproachPlugin)
        .add_plugins(hop::HopPlugin)
        .add_plugins(checklist::ChecklistPlugin)
        .add_plugins(taxi::TaxiPlugin)
        .add_plugins(jet_taxi::JetTaxiPlugin)
        .add_plugins(lesson_flow::LessonFlowPlugin)
        .add_plugins(mission::MissionPlugin)
        .add_plugins(combat::CombatPlugin)
        .add_plugins(lights::LightsPlugin)
        .add_plugins(pause::PausePlugin)
        .init_resource::<ShotTimer>()
        .add_systems(Update, auto_shot)
        .run();
}

/// Development hook: `SHOT=/tmp/game.png` grabs one frame after a few seconds,
/// so a desktop screenshot pass can inspect the real game view without driving
/// the menu or a window manager.
fn auto_shot(mut commands: Commands, time: Res<Time>, mut timer: ResMut<ShotTimer>) {
    let Ok(path) = std::env::var("SHOT") else {
        return;
    };
    timer.0.tick(time.delta());
    if timer.0.just_finished() {
        commands
            .spawn(Screenshot::primary_window())
            .observe(save_to_disk(path));
    }
}
