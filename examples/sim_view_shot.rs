//! Runs the real sim + view (same plugins as `main.rs`), pins the control
//! inputs to full deflection, and screenshots — for verifying the rigged model
//! is wired in correctly.
//!
//! Run: `cargo run --example sim_view_shot`

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

#[derive(Resource, Default)]
struct Frame(u32);

fn main() {
    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "sim_view_shot".into(),
                resolution: (1200, 800).into(),
                ..default()
            }),
            ..default()
        }))
        .insert_resource(Time::<Fixed>::from_hz(1.0 / sim::DT))
        .add_plugins((
            sim::SimPlugin,
            input::InputPlugin,
            view::ViewPlugin,
            weather::WeatherPlugin,
        ))
        // Manual control, so nothing fights the pinned inputs below.
        .insert_resource(sim::Autopilot {
            enabled: false,
            phase: "manual",
        })
        .init_resource::<Frame>()
        // `Last` so our values survive `read_input`'s ramp-down and are what the
        // next FixedUpdate (advance_fdm) writes into the FDM.
        .add_systems(Last, pin_controls)
        .add_systems(Update, shot)
        // Override the chase cam for a 3/4 view where ailerons, flaps, elevator
        // and rudder all read at once. PostUpdate runs after `follow_camera`.
        .add_systems(PostUpdate, birdseye)
        .run();
}

fn birdseye(mut q: Query<&mut Transform, With<Camera3d>>) {
    if let Ok(mut t) = q.single_mut() {
        *t = Transform::from_xyz(-4.0, 4.0, 13.0).looking_at(Vec3::new(0.0, 1.3, 2.0), Vec3::Y);
    }
}

/// Deflect one surface at a time (brakes on, throttle 0) so each direction can
/// be read unambiguously. Windows: neutral, aileron only, elevator only, rudder
/// only — one screenshot each.
fn pin_controls(frame: Res<Frame>, mut bindings: ResMut<input::Bindings>) {
    let f = frame.0;
    let (ail, ele, rud) = if f < 160 {
        (0.0, 0.0, 0.0)
    } else if f < 240 {
        (1.0, 0.0, 0.0) // +aileron == roll right
    } else if f < 320 {
        (0.0, 1.0, 0.0) // +elevator == nose down
    } else {
        (0.0, 0.0, 1.0) // +rudder == yaw left
    };
    for b in &mut bindings.list {
        b.value = match b.property {
            "fcs/left-brake-cmd-norm" | "fcs/right-brake-cmd-norm" => 1.0,
            "fcs/throttle-cmd-norm" | "fcs/flap-cmd-norm" => 0.0,
            "fcs/aileron-cmd-norm" => ail,
            "fcs/elevator-cmd-norm" => ele,
            "fcs/rudder-cmd-norm" => rud,
            _ => 0.0,
        };
    }
}

fn shot(mut commands: Commands, mut f: ResMut<Frame>, mut exit: MessageWriter<AppExit>) {
    f.0 += 1;
    for (n, path) in [
        (140u32, "/tmp/n_neutral.png"),
        (220, "/tmp/n_aileron.png"),
        (300, "/tmp/n_elevator.png"),
        (380, "/tmp/n_rudder.png"),
    ] {
        if f.0 == n {
            commands
                .spawn(Screenshot::primary_window())
                .observe(save_to_disk(path));
            info!("screenshot -> {path}");
        }
    }
    if f.0 == 440 {
        exit.write(AppExit::Success);
    }
}
