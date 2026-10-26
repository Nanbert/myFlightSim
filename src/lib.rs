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
pub mod atc_voice;
pub mod autopilot;
pub mod checklist;
pub mod cloud_camera;
// Volumetric clouds need a custom `Core3d` pass, a 3D density texture and an
// HDR target; the web build keeps the card clouds (`clouds.rs`) instead.
#[cfg(not(target_arch = "wasm32"))]
pub mod cloud_volume;
pub mod clouds;
pub mod cockpit;
pub mod combat;
pub mod config;
pub mod controls;
pub mod engine_audio;
pub mod fx;
pub mod g_strain;
pub mod gauge;
pub mod hop;
pub mod hud;
pub mod ils;
pub mod input;
pub mod jet_go_around;
pub mod jet_landing_lesson;
pub mod jet_rto;
pub mod jet_automation;
pub mod jet_takeoff_lesson;
pub mod jet_taxi;
pub mod lesson_flow;
pub mod landing;
pub mod landing_lesson;
pub mod lens;
pub mod lights;
pub mod island;
pub mod map;
#[cfg(feature = "shots")]
pub mod mcp_bake;
pub mod mcp_layout;
pub mod mcp_panel;
pub mod mcp_ui;
pub mod menu;
pub mod menu_page;
pub mod mission;
pub mod mcp_guide;
pub mod panel_geom;
pub mod pause;
pub mod pedestal_layout;
#[cfg(feature = "shots")]
pub mod pedestal_bake;
pub mod pedestal_ui;
pub mod pfd;
pub mod radio;
pub mod scoring;
pub mod sim;
pub mod sky;
pub mod statue;
pub mod takeoff;
pub mod takeoff_lesson;
pub mod taxi;
pub mod tutorial;
pub mod ui_audio;
pub mod ui_theme;
pub mod vapor;
pub mod view;
pub mod weather;

use bevy::prelude::*;
use bevy::render::view::window::screenshot::{save_to_disk, Screenshot};
use bevy::window::WindowMode;

#[derive(Resource)]
struct ShotTimer {
    delay: Timer,
    /// Cadence between frames of a multi-frame grab (`SHOT_FRAMES`).
    burst: Timer,
    /// `SHOT`'s path plus the total frame count; `None` when no grab is armed.
    path: Option<(String, u32)>,
    frames_left: u32,
}

impl Default for ShotTimer {
    fn default() -> Self {
        let frames = std::env::var("SHOT_FRAMES")
            .ok()
            .and_then(|v| v.parse().ok())
            .unwrap_or(1)
            .max(1);
        let delay = std::env::var("SHOT_DELAY")
            .ok()
            .and_then(|v| v.parse().ok())
            .unwrap_or(3.0);
        Self {
            delay: Timer::from_seconds(delay, TimerMode::Once),
            burst: Timer::from_seconds(0.4, TimerMode::Repeating),
            path: std::env::var("SHOT").ok().map(|p| (p, frames)),
            frames_left: frames,
        }
    }
}

pub fn run() {
    // Web hosts (itch.io-style, nginx `try_files ... /index.html`, …) answer
    // missing `*.meta` fetches with a 200 HTML page instead of a real 404, and
    // the RON meta parse fails for every asset unprocessed assets don't ship
    // meta files for anyway. Skip the meta round-trip on wasm; loaders fall
    // back to their default meta, which is exactly what unprocessed assets
    // need (Bevy issue #13442 / AssetMetaCheck).
    #[cfg(target_arch = "wasm32")]
    let asset_plugin = AssetPlugin {
        meta_check: bevy::asset::AssetMetaCheck::Never,
        ..default()
    };
    #[cfg(not(target_arch = "wasm32"))]
    let asset_plugin = AssetPlugin::default();

    App::new()
        .add_plugins(DefaultPlugins.set(asset_plugin).set(WindowPlugin {
            primary_window: Some(Window {
                title: "myFlightSim — Rust + Bevy".into(),
                // The instrument panel is laid out for a wide screen: three
                // gauges against each edge with the attitude indicator centred.
                // A half-width window would overlap the two trios, so start
                // fullscreen (as the sim is meant to be flown).
                mode: WindowMode::BorderlessFullscreen(MonitorSelection::Primary),
                // Web: size the canvas to its container and follow resizes,
                // instead of freezing at winit's default logical size.
                #[cfg(target_arch = "wasm32")]
                fit_canvas_to_parent: true,
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
        // Separate call: Bevy's plugin tuples top out at 15.
        .add_plugins(ui_audio::UiAudioPlugin)
        .add_plugins(engine_audio::EngineAudioPlugin)
        .add_plugins(atc_voice::AtcVoicePlugin)
        .add_plugins(mcp_guide::McpGuidePlugin)
        .add_plugins(mcp_panel::McpPanelPlugin)
        .add_plugins(pedestal_ui::PedestalUiPlugin)
        .add_plugins(g_strain::GStrainPlugin)
        .add_plugins(radio::RadioPlugin)
        // After the menu: it builds terrain for whatever mission
        // `start_flight` just launched. (A separate call: Bevy's plugin tuples
        // top out at 15.)
        .add_plugins(map::MapPlugin)
        .add_plugins(clouds::CloudPlugin)
        .add_plugins(island::IslandPlugin)
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
        .add_plugins(jet_rto::JetRtoLessonPlugin)
        .add_plugins(jet_automation::JetAutomationLessonPlugin)
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

/// Development hook: `SHOT=/tmp/game.png` grabs one frame after `SHOT_DELAY`
/// seconds (default 3), so a desktop screenshot pass can inspect the real game
/// view without driving the menu or a window manager. `SHOT_FRAMES=N` grabs N
/// frames 0.4 s apart (`/tmp/game.0.png`, `/tmp/game.1.png`, …) instead, to
/// compare flicker across consecutive renders at nearly the same camera pose.
fn auto_shot(mut commands: Commands, time: Res<Time>, mut timer: ResMut<ShotTimer>) {
    let Some((path, total)) = timer.path.clone() else {
        return;
    };
    if timer.frames_left == 0 {
        return;
    }
    timer.delay.tick(time.delta());
    if timer.delay.just_finished() {
        timer.burst.reset();
    } else if timer.delay.elapsed_secs() < timer.delay.duration().as_secs_f32() {
        return;
    } else {
        timer.burst.tick(time.delta());
        if !timer.burst.just_finished() {
            return;
        }
    }
    // `save_to_disk` picks the format from the extension, so burst frames
    // number the file stem and always end in `.png`.
    let name = if total == 1 {
        path
    } else {
        let stem = std::path::Path::new(&path).with_extension("");
        format!("{}.{}.png", stem.display(), total - timer.frames_left)
    };
    commands
        .spawn(Screenshot::primary_window())
        .observe(save_to_disk(name));
    timer.frames_left -= 1;
}
