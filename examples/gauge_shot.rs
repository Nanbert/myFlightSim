//! Drives the C172 instrument cluster through a few flight conditions and
//! screenshots each, for eyeballing the gauges against the reference photos.
//!
//! Run: `cargo run --example gauge_shot`

#[path = "../src/aircraft_profile.rs"]
mod aircraft_profile;
#[path = "../src/config.rs"]
mod config;
#[path = "../src/input.rs"]
mod input;
#[path = "../src/app_state.rs"]
mod app_state;
#[path = "../src/autopilot.rs"]
mod autopilot;
#[path = "../src/sim.rs"]
mod sim;
#[path = "../src/view.rs"]
mod view;
#[path = "../src/weather.rs"]
mod weather;
#[path = "../src/gauge/mod.rs"]
mod gauge;

use bevy::prelude::*;
use bevy::render::view::window::screenshot::{save_to_disk, Screenshot};
use bevy::window::WindowMode;

#[derive(Resource, Default)]
struct Frame(u32);

fn main() {
    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "gauge_shot".into(),
                mode: WindowMode::BorderlessFullscreen(MonitorSelection::Primary),
                ..default()
            }),
            ..default()
        }))
        .insert_resource(Time::<Fixed>::from_hz(1.0 / sim::DT))
        .add_plugins((
            sim::SimPlugin,
            autopilot::AutopilotPlugin,
            config::ConfigPlugin,
            input::InputPlugin,
            view::ViewPlugin,
            weather::WeatherPlugin,
            gauge::GaugePlugin,
        ))
        .insert_state(app_state::AppState::Flying)
        .add_systems(
            OnEnter(app_state::AppState::Flying),
            (view::spawn_flight_aircraft, view::reset_control_rig).chain(),
        )
        .init_resource::<Frame>()
        .add_systems(Update, (set_condition, shot).chain())
        .run();
}

/// Reset the aircraft just before each screenshot, so the cluster is captured
/// at a known reading (minimal drift in the few frames before the shot).
///
/// The attitude/rate/slip ICs are spelled the same way JSBSim spells them, so
/// this works on either backend.
fn set_condition(mut sim: ResMut<sim::Sim>, f: Res<Frame>) {
    match f.0 {
        65 => sim.reset_air(3_827.0, 90.0),
        125 => {
            // A banked, climbing, turning attitude: the horizon displaces and
            // rolls, the heading card swings, the turn symbol banks.
            set_ic(&mut sim, 3_000.0, 100.0, 60.0);
            sim.model.set("ic/phi-deg", 25.0);
            sim.model.set("ic/theta-deg", 8.0);
            sim.model.set("ic/r-rad_sec", 3.0_f64.to_radians());
            sim.model.apply_ic();
        }
        185 => {
            // Wings level but yawing through the air (a sideslip): the turn
            // symbol sits level while the ball rolls off centre.
            set_ic(&mut sim, 3_000.0, 100.0, 0.0);
            sim.model.set("ic/beta-deg", 8.0);
            sim.model.apply_ic();
        }
        _ => {}
    }
}

fn set_ic(sim: &mut sim::Sim, alt_ft: f64, vc_kts: f64, psi_deg: f64) {
    sim.model.set("ic/vc-kts", vc_kts);
    sim.model.set("ic/h-sl-ft", alt_ft);
    sim.model.set("ic/psi-true-deg", psi_deg);
    // Clear the optional attitude/rate/slip ICs so each shot stands alone rather
    // than inheriting the previous branch's attitude.
    for p in [
        "ic/phi-deg",
        "ic/theta-deg",
        "ic/beta-deg",
        "ic/p-rad_sec",
        "ic/q-rad_sec",
        "ic/r-rad_sec",
    ] {
        sim.model.set(p, 0.0);
    }
}

fn shot(mut commands: Commands, mut f: ResMut<Frame>, mut exit: MessageWriter<AppExit>) {
    f.0 += 1;
    for (n, path) in [
        (70u32, "/tmp/gauge_level.png"),
        (130, "/tmp/gauge_bank.png"),
        (190, "/tmp/gauge_slip.png"),
    ] {
        if f.0 == n {
            commands
                .spawn(Screenshot::primary_window())
                .observe(save_to_disk(path));
            info!("screenshot -> {path}");
        }
    }
    if f.0 == 210 {
        exit.write(AppExit::Success);
    }
}
