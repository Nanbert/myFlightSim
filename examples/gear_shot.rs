//! Flies the F-22 airborne with the gear down, then commands it up and
//! screenshots each state from a low front-quarter view that follows the jet
//! (the parked interlock won't retract on the ground, so this has to fly).
//!
//! Run: `FDM_BACKEND=rust AIRCRAFT=f22 cargo run --example gear_shot`

#[path = "../src/cloud_camera.rs"]
mod cloud_camera;
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

use bevy::prelude::*;
use bevy::render::view::window::screenshot::{save_to_disk, Screenshot};

#[derive(Resource, Default)]
struct Frame(u32);

fn main() {
    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "gear_shot".into(),
                resolution: (1200, 800).into(),
                ..default()
            }),
            ..default()
        }))
        .insert_resource(Time::<Fixed>::from_hz(1.0 / sim::DT))
        .add_plugins((
            sim::SimPlugin,
            autopilot::AutopilotPlugin,
            input::InputPlugin,
            view::ViewPlugin,
            weather::WeatherPlugin,
        ))
        // No menu here: start directly in `Flying` and spawn the aircraft
        // ourselves (MenuPlugin would otherwise do it on entering the state).
        .insert_state(app_state::AppState::Flying)
        .add_systems(
            OnEnter(app_state::AppState::Flying),
            (view::spawn_flight_aircraft, view::reset_control_rig).chain(),
        )
        .init_resource::<Frame>()
        .add_systems(Startup, launch)
        .add_systems(Last, drive)
        .add_systems(Update, shot)
        .add_systems(PostUpdate, side_cam)
        .run();
}

/// Start airborne so the gear can leave the wells.
fn launch(mut sim: ResMut<sim::Sim>) {
    sim.reset_air(8000.0, 300.0);
}

/// Fixed chase view: off the jet's left, low, looking slightly up at the belly.
fn side_cam(rp: Res<sim::RenderPose>, mut q: Query<&mut Transform, With<Camera3d>>) {
    let Ok(mut t) = q.single_mut() else { return };
    if !rp.init {
        return;
    }
    let pos = rp.curr_pos;
    let yaw = Quat::from_rotation_y(-rp.curr_heading);
    // 18 m to the side, 3 m below the CG, 4 m ahead.
    let eye = pos + yaw * Vec3::new(18.0, -3.0, -2.0);
    *t = Transform::from_translation(eye).looking_at(pos, Vec3::Y);
}

fn drive(frame: Res<Frame>, mut bindings: ResMut<input::Bindings>, mut sim: ResMut<sim::Sim>) {
    for b in &mut bindings.list {
        b.value = match b.property {
            "fcs/throttle-cmd-norm" => 0.75,
            _ => 0.0,
        };
    }
    if frame.0 == 120 {
        sim.model.set("gear/gear-cmd-norm", 0.0);
        info!("gear up commanded");
    }
}

fn shot(mut commands: Commands, mut f: ResMut<Frame>, mut exit: MessageWriter<AppExit>) {
    f.0 += 1;
    for (n, path) in [
        (100u32, "/tmp/gear_down.png"),
        (120, "/tmp/gear_mid.png"),
        (480, "/tmp/gear_transit.png"),
        (1100, "/tmp/gear_up.png"),
    ] {
        if f.0 == n {
            commands
                .spawn(Screenshot::primary_window())
                .observe(save_to_disk(path));
            info!("screenshot -> {path}");
        }
    }
    if f.0 == 1140 {
        exit.write(AppExit::Success);
    }
}
