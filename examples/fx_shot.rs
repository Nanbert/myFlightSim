//! Screenshots the F-22's engine effects (afterburner flame and exhaust heat
//! haze) from a rear-quarter chase view, for eyeballing the anchor placement and
//! tuning.
//!
//! Run: `FDM_BACKEND=rust AIRCRAFT=f22 cargo run --example fx_shot`
//!
//! `VIEW=side` swaps the camera to a broadside view of the exhaust.

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
#[path = "../src/fx.rs"]
mod fx;
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
#[path = "../src/lens.rs"]
mod lens;
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

/// `VIEW=side` puts the camera broadside to the exhaust instead of the default
/// rear-quarter view.
#[derive(Resource, Default)]
struct SideView(bool);

fn main() {
    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "fx_shot".into(),
                resolution: (1200, 800).into(),
                ..default()
            }),
            ..default()
        }))
        // Keep stepping while unfocused, so the frame-counted shots fire.
        .insert_resource(WinitSettings::continuous())
        .insert_resource(Time::<Fixed>::from_hz(1.0 / sim::DT))
        .add_plugins((
            sim::SimPlugin,
            autopilot::AutopilotPlugin,
            config::ConfigPlugin,
            input::InputPlugin,
            view::ViewPlugin,
            fx::FxPlugin,
            weather::WeatherPlugin,
        ))
        .insert_state(app_state::AppState::Flying)
        .add_systems(
            OnEnter(app_state::AppState::Flying),
            (view::spawn_flight_aircraft, view::reset_control_rig).chain(),
        )
        .init_resource::<Frame>()
        .init_resource::<SideView>()
        .add_systems(Startup, (launch, read_view_env))
        .add_systems(Update, clear_weather)
        .add_systems(Last, drive)
        .add_systems(Update, shot)
        .add_systems(PostUpdate, chase_cam)
        .run();
}

/// Start airborne so the gear is up and the jet is moving.
fn launch(mut sim: ResMut<sim::Sim>) {
    sim.reset_air(6000.0, 320.0);
}

/// Pin the sky clear: the saved config may have left it raining, and rain on the
/// lens would obscure the effects we're here to look at.
fn clear_weather(mut weather: ResMut<weather::Weather>) {
    weather.rain = 0.0;
}

/// `VIEW=side` switches the camera to a broadside view.
fn read_view_env(mut side: ResMut<SideView>) {
    side.0 = std::env::var("VIEW").is_ok_and(|v| v == "side");
}

/// Full throttle throughout, so the burner stays lit for the shot.
fn drive(mut bindings: ResMut<input::Bindings>) {
    for b in &mut bindings.list {
        b.value = match b.property {
            "fcs/throttle-cmd-norm" => 1.0,
            _ => 0.0,
        };
    }
}

/// Rear-quarter (default) or side-on, slightly above: the nozzles stay in view.
/// `VIEW=side` puts the camera broadside to the exhaust.
fn chase_cam(
    rp: Res<sim::RenderPose>,
    side: Res<SideView>,
    mut q: Query<&mut Transform, With<Camera3d>>,
) {
    let Ok(mut t) = q.single_mut() else { return };
    if !rp.init {
        return;
    }
    let local = if side.0 {
        Vec3::new(44.0, 3.0, 4.0)
    } else {
        Vec3::new(11.0, 6.0, 34.0)
    };
    let pos = rp.curr_pos;
    let yaw = Quat::from_rotation_y(-rp.curr_heading);
    *t = Transform::from_translation(pos + yaw * local).looking_at(pos, Vec3::Y);
}

fn shot(mut commands: Commands, mut f: ResMut<Frame>, mut exit: MessageWriter<AppExit>) {
    f.0 += 1;
    if f.0 % 100 == 0 {
        info!("frame {}", f.0);
    }
    for (n, path) in [
        (60u32, "/tmp/opencode/fx_level.png"),
        (200, "/tmp/opencode/fx_pull.png"),
        (340, "/tmp/opencode/fx_pull2.png"),
    ] {
        if f.0 == n {
            commands
                .spawn(Screenshot::primary_window())
                .observe(save_to_disk(path));
            info!("screenshot -> {path}");
        }
    }
    if f.0 == 380 {
        exit.write(AppExit::Success);
    }
}
