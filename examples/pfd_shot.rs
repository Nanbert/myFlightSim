//! Screenshot harness for the two 737 PFDs, with representative takeoff data.
//!
//! Run: `cargo run --example pfd_shot` and open `/tmp/pfd_shot.png`.

use bevy::prelude::*;
use bevy::render::view::window::screenshot::{save_to_disk, Screenshot};
use bevy::window::WindowMode;
use bevy::winit::WinitSettings;
use my_flight_sim::{app_state, autopilot, ils, pfd, sim, view};

use autopilot::{AtMode, Autopilot, LatMode, Mcp, SpeedUnit, VertMode};

#[derive(Resource, Default)]
struct Frame(u32);

fn main() {
    std::env::set_var("FDM_BACKEND", "rust");
    let state = sim::AircraftState {
        cas_kts: 145.0,
        alt_ft: 3_000.0,
        vs_fpm: 0.0,
        pitch_deg: 8.0,
        roll_deg: 0.0,
        heading_deg: 77.0,
        gs_kts: 140.0,
        tas_kts: 158.0,
        mach: 0.24,
        radar_alt_ft: 3_000.0,
        flap_deg: 10.0,
        slip: 0.02,
        on_ground: true,
        ..default()
    };
    let mut ap = Autopilot::default();
    ap.engaged = true;
    ap.lat = LatMode::HdgSel;
    ap.vert = VertMode::AltHold;
    ap.at = AtMode::Speed;
    ap.fd_pitch_deg = 7.0;
    ap.fd_bank_deg = 0.0;
    let mcp = Mcp {
        hdg_deg: 77.0,
        alt_ft: 3_000.0,
        vs_fpm: 0.0,
        ias_kts: 145.0,
        mach: 0.78,
        speed_unit: SpeedUnit::Ias,
    };

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "pfd_shot".into(),
                mode: WindowMode::BorderlessFullscreen(MonitorSelection::Primary),
                ..default()
            }),
            ..default()
        }))
        .insert_resource(ClearColor(Color::srgb(0.02, 0.06, 0.10)))
        .insert_resource(WinitSettings::continuous())
        .insert_resource(sim::Sim::new("737"))
        .insert_resource(state)
        .insert_resource(ap)
        .insert_resource(mcp)
        .insert_resource(ils::Ils::default())
        .init_resource::<view::Ui>()
        .add_plugins(pfd::PfdPlugin)
        .insert_state(app_state::AppState::Flying)
        .init_resource::<Frame>()
        .add_systems(Startup, |mut commands: Commands| {
            commands.spawn(Camera2d);
        })
        .add_systems(Update, shot)
        .run();
}

fn shot(mut commands: Commands, mut frame: ResMut<Frame>, mut exit: MessageWriter<AppExit>) {
    frame.0 += 1;
    if frame.0 == 180 {
        commands
            .spawn(Screenshot::primary_window())
            .observe(save_to_disk("/tmp/pfd_shot.png"));
        info!("screenshot -> /tmp/pfd_shot.png");
    }
    if frame.0 >= 220 {
        exit.write(AppExit::Success);
    }
}
