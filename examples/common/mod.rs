//! Shared scaffolding for the screenshot examples.
//!
//! Every example under `shots` boots the same sim: a window, a fixed timestep,
//! the core flight plugin chain, and a direct entry into `AppState::Flying`
//! (no menu). They also count frames to drive a deterministic capture script.
//! This module holds those pieces; an example adds its own look-dev plugins,
//! sets up its scene, and either hands a [`Shots`] list to [`drive_shots`] or
//! writes its own frame-indexed capture system against [`Frame`].
//!
//! `common` is not an example: Cargo only auto-builds `.rs` files directly in
//! `examples/` and subdirectories with a `main.rs`, so this `mod.rs` is only
//! linked by the examples that declare it.

#![allow(dead_code)]

use bevy::prelude::*;
use bevy::render::view::window::screenshot::{save_to_disk, Screenshot};
use bevy::window::{MonitorSelection, WindowMode};
use bevy::winit::WinitSettings;
use my_flight_sim::{app_state, autopilot, config, input, sim, view, weather};

/// Frame counter for the frame-indexed scripts. `Update`-scheduled counters
/// advance once per rendered frame; `First` systems see the previous value.
#[derive(Resource, Default)]
pub struct Frame(pub u32);

/// A fixed capture script: one screenshot at each listed frame, then exit.
#[derive(Resource)]
pub struct Shots {
    pub list: &'static [(u32, &'static str)],
    pub exit_frame: u32,
}

impl Shots {
    pub const fn new(list: &'static [(u32, &'static str)], exit_frame: u32) -> Self {
        Self { list, exit_frame }
    }
}

/// Build the sim testbed. `resolution: Some((w, h))` opens a window; `None`
/// goes borderless fullscreen.
pub fn app(title: &str, resolution: Option<(u32, u32)>) -> App {
    let mode = match resolution {
        Some(_) => WindowMode::Windowed,
        None => WindowMode::BorderlessFullscreen(MonitorSelection::Primary),
    };
    app_with(Window {
        title: title.into(),
        mode,
        resolution: resolution.map(Into::into).unwrap_or_default(),
        ..default()
    })
}

/// [`app`] with a fully-specified window, for examples that need more than a
/// title and resolution (e.g. a custom present mode).
pub fn app_with(window: Window) -> App {
    let mut app = App::new();
    app.add_plugins(DefaultPlugins.set(WindowPlugin {
        primary_window: Some(window),
        ..default()
    }))
    // Keep stepping while the window is unfocused: game mode otherwise
    // throttles a frame-counted script to ~1 Hz when the compositor steals
    // focus.
    .insert_resource(WinitSettings::continuous())
    .insert_resource(Time::<Fixed>::from_hz(1.0 / sim::DT))
    .add_plugins((
        sim::SimPlugin,
        autopilot::AutopilotPlugin,
        // Before `InputPlugin`: it builds the startup bindings from `GameConfig`.
        config::ConfigPlugin,
        input::InputPlugin,
        view::ViewPlugin,
        weather::WeatherPlugin,
    ))
    // No menu: enter `Flying` directly and spawn the aircraft the menu would
    // have spawned.
    .insert_state(app_state::AppState::Flying)
    .add_systems(
        OnEnter(app_state::AppState::Flying),
        (view::spawn_flight_aircraft, view::reset_control_rig).chain(),
    )
    .init_resource::<Frame>();
    app
}

/// Advance [`Frame`], capture each [`Shots`] entry on its frame, then exit.
pub fn drive_shots(
    mut commands: Commands,
    mut f: ResMut<Frame>,
    shots: Res<Shots>,
    mut exit: MessageWriter<AppExit>,
) {
    f.0 += 1;
    for &(n, path) in shots.list {
        if f.0 == n {
            commands
                .spawn(Screenshot::primary_window())
                .observe(save_to_disk(path));
            info!("screenshot -> {path}");
        }
    }
    if f.0 == shots.exit_frame {
        exit.write(AppExit::Success);
    }
}

/// Set the FDM's initial condition to level flight at `vc_kts` and `alt_ft`
/// heading `psi_deg`, clearing any attitude/rate/slip left by a previous
/// condition. The property names are JSBSim's, so this works on either
/// backend.
pub fn set_ic(sim: &mut sim::Sim, alt_ft: f64, vc_kts: f64, psi_deg: f64) {
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
