//! Boots one arena mission in flight and screenshots it, for look-dev on the
//! terrain centres of mass: `MAP=GLACIAL cargo run --example map_shot`.
//!
//! Replicates the menu's map-mission start (arcade F-22, seated airborne) and
//! grabs a chase-view shot after the world has settled.
//!
//! Run: `MAP=GLACIAL cargo run --example map_shot`

#[path = "../src/aircraft_profile.rs"]
mod aircraft_profile;
#[path = "../src/aerodrome.rs"]
mod aerodrome;
#[path = "../src/app_state.rs"]
mod app_state;
#[path = "../src/autopilot.rs"]
mod autopilot;
#[path = "../src/cloud_camera.rs"]
mod cloud_camera;
#[path = "../src/config.rs"]
mod config;
#[path = "../src/ils.rs"]
mod ils;
#[path = "../src/hud.rs"]
mod hud;
#[path = "../src/input.rs"]
mod input;
#[path = "../src/lights.rs"]
mod lights;
#[path = "../src/map.rs"]
mod map;
#[path = "../src/mission.rs"]
mod mission;
#[path = "../src/sim.rs"]
mod sim;
#[path = "../src/sky.rs"]
mod sky;
#[path = "../src/statue.rs"]
mod statue;
#[path = "../src/taxi.rs"]
mod taxi;
#[path = "../src/view.rs"]
mod view;
#[path = "../src/weather.rs"]
mod weather;
#[path = "../src/ui_theme.rs"]
mod ui_theme;

use bevy::prelude::*;
use bevy::render::view::window::screenshot::{save_to_disk, Screenshot};
use bevy::winit::WinitSettings;

#[derive(Resource, Default)]
struct Frame(u32);

/// Screenshots must not touch the real config file, so the sink is a no-op and
/// the config starts empty.
struct NullStore;
impl config::ConfigStore for NullStore {
    fn load(&self) -> Option<String> {
        None
    }
    fn save(&self, _: &str) {}
}

fn main() {
    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "map_shot".into(),
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
            taxi::TaxiPlugin,
            lights::LightsPlugin,
            hud::HudPlugin,
            view::ViewPlugin,
            weather::WeatherPlugin,
            sky::SkyPlugin,
            mission::MissionPlugin,
            map::MapPlugin,
            statue::StatuePlugin,
        ))
        .insert_resource(config::GameConfig::default())
        .insert_resource(config::ConfigSink(Box::new(NullStore)))
        // The mission; a plain insert survives the map plugin's own
        // init. Default to the glacial: this harness exists for its look-dev.
        .insert_resource(map::PendingMission(Some(read_map_env())))
        // Register the state (default Menu) and start straight in the flight;
        // the map plugin builds the terrain on the state transition. The
        // switch in Startup runs after setup_sky has inserted `CloudField`
        // (enter_mission reads it).
        .insert_state(app_state::AppState::Menu)
        .add_systems(Startup, enter_flight)
        .add_systems(
            OnEnter(app_state::AppState::Flying),
            (mission_boot, view::spawn_flight_aircraft, view::reset_control_rig).chain(),
        )
        .init_resource::<Frame>()
        .init_resource::<StatueView>()
        .add_systems(Startup, read_statue_env)
        .add_systems(Update, (hold_alt, shot))
        // After the follow camera wrote this frame's pose.
        .add_systems(PostUpdate, statue_camera)
        .run();
}

// `STATUE=1` pins the camera in front of the glacial bust for a close look.
#[derive(Resource, Default)]
struct StatueView(bool);

/// `STATUE=1` frames the summit landmark instead of the chase view.
fn read_statue_env(mut view: ResMut<StatueView>) {
    view.0 = std::env::var("STATUE").is_ok_and(|v| v == "1");
}

/// `MAP=GLACIAL` (default), `MAP=ALPINE`, `MAP=ARCHIPELAGO`.
fn read_map_env() -> map::MapId {
    match std::env::var("MAP").unwrap_or_default().to_ascii_uppercase().as_str() {
        "ARCHIPELAGO" => map::MapId::Archipelago,
        "ALPINE" => map::MapId::Alpine,
        _ => map::MapId::Glacial,
    }
}

/// Trigger the state transition once Startup (and `setup_sky`) has run.
fn enter_flight(mut next: ResMut<NextState<app_state::AppState>>) {
    next.set(app_state::AppState::Flying);
}

/// The menu's map-mission initial conditions, without the menu itself: arcade
/// F-22 seated airborne over the arena, cruise trim solved from frame one.
/// Re-reads the env instead of `PendingMission`: the map plugin's
/// `enter_mission` may consume the pending map before this runs.
fn mission_boot(mut sim: ResMut<sim::Sim>) {
    let map = read_map_env();
    *sim = sim::Sim::new_arcade("f22");
    sim.model.set("fcs/arcade-enable", 1.0);
    sim.reset_air(map.spawn_alt_ft(), map.spawn_kts());
}

/// Pin a level cruise so the shot is a clean bird's-eye of the terrain rather
/// than whatever the arcade trim drifts into.
fn hold_alt(mut bindings: ResMut<input::Bindings>) {
    for b in &mut bindings.list {
        b.value = match b.property {
            "fcs/throttle-cmd-norm" => 0.85,
            "fcs/elevator-cmd-norm" => 0.0,
            "fcs/aileron-cmd-norm" => 0.0,
            "fcs/rudder-cmd-norm" => 0.0,
            _ => 0.0,
        };
    }
}

/// Face the bust from the southern approach: the summit disc is level, so the
/// camera floats 480 m above it (mid-bust) and 1.6 km out.
fn statue_camera(view: Res<StatueView>, mut q: Query<&mut Transform, (With<Camera3d>, Without<cloud_camera::CloudCamera>)>) {
    if !view.0 {
        return;
    }
    let summit = map::ground_height(map::MapId::Glacial, 0.0, 0.0);
    if let Ok(mut t) = q.single_mut() {
        *t = Transform::from_xyz(0.0, summit + 480.0, 1_600.0)
            .looking_at(Vec3::new(0.0, summit + 400.0, 0.0), Vec3::Y);
    }
}

fn shot(mut commands: Commands, mut f: ResMut<Frame>, mut exit: MessageWriter<AppExit>) {
    let out: &'static str = Box::leak(
        std::env::var("OUT")
            .unwrap_or_else(|_| "/tmp/map_shot.png".into())
            .into_boxed_str(),
    );
    f.0 += 1;
    if f.0 == 900 {
        commands
            .spawn(Screenshot::primary_window())
            .observe(save_to_disk(out));
        info!("screenshot");
    }
    if f.0 == 960 {
        exit.write(AppExit::Success);
    }
}
