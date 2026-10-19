//! Flies the F-22 through a few attitudes and screenshots the HUD, for eyeballing
//! the symbology.
//!
//! Run: `AIRCRAFT=f22 cargo run --example hud_shot`

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
#[path = "../src/hud.rs"]
mod hud;
#[path = "../src/input.rs"]
mod input;
#[path = "../src/sim.rs"]
mod sim;
#[path = "../src/view.rs"]
mod view;
#[path = "../src/weather.rs"]
mod weather;

use bevy::prelude::*;
use bevy::render::view::window::screenshot::{save_to_disk, Screenshot};
use bevy::winit::WinitSettings;

#[derive(Resource, Default)]
struct Frame(u32);

fn main() {
    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "hud_shot".into(),
                resolution: (1200, 800).into(),
                ..default()
            }),
            ..default()
        }))
        // Keep stepping while unfocused, so a window manager that tiles the
        // window can't throttle the frame-counted shots.
        .insert_resource(WinitSettings::continuous())
        .insert_resource(Time::<Fixed>::from_hz(1.0 / sim::DT))
        .add_plugins((
            sim::SimPlugin,
            autopilot::AutopilotPlugin,
            config::ConfigPlugin,
            input::InputPlugin,
            view::ViewPlugin,
            weather::WeatherPlugin,
            hud::HudPlugin,
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

/// Rewrite the state just before each capture so the reading is known.
fn set_condition(mut sim: ResMut<sim::Sim>, f: Res<Frame>) {
    match f.0 {
        170 => sim.reset_air(20_000.0, 420.0),
        255 => {
            // Banked and climbing: the ladder and horizon roll, the bank pointer
            // swings, the FPM walks off the boresight.
            set_ic(&mut sim, 18_000.0, 380.0, 120.0);
            sim.model.set("ic/phi-deg", 32.0);
            sim.model.set("ic/theta-deg", 9.0);
            sim.model.apply_ic();
        }
        365 => {
            // Gear down, slow: the approach-angle bar appears beside the FPM.
            set_ic(&mut sim, 3_000.0, 165.0, 0.0);
            sim.model.set("ic/theta-deg", 8.0);
            sim.model.apply_ic();
            sim.model.set("gear/gear-cmd-norm", 1.0);
            sim.model.set("gear/gear-pos-norm", 1.0);
        }
        _ => {}
    }
}

fn set_ic(sim: &mut sim::Sim, alt_ft: f64, vc_kts: f64, psi_deg: f64) {
    sim.model.set("ic/vc-kts", vc_kts);
    sim.model.set("ic/h-sl-ft", alt_ft);
    sim.model.set("ic/psi-true-deg", psi_deg);
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
        (200u32, "/tmp/hud_level.png"),
        (300, "/tmp/hud_bank.png"),
        (440, "/tmp/hud_gear.png"),
    ] {
        if f.0 == n {
            commands
                .spawn(Screenshot::primary_window())
                .observe(save_to_disk(path));
            info!("screenshot -> {path}");
        }
    }
    if f.0 == 480 {
        exit.write(AppExit::Success);
    }
}
