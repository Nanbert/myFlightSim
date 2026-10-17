//! Boeing 737 primary flight displays.
//!
//! Two identical PFDs sit in the bottom corners, flanking the MCP: the
//! captain's and the first officer's. Both are `UiMaterial` nodes sharing one
//! material, so they cannot drift apart — every quantity is packed once per
//! frame from [`AircraftState`] and the autopilot.
//!
//! The display itself (attitude, flight directors, speed/altitude tapes, V/S,
//! heading tape, FMA and the bottom data row) is drawn procedurally by
//! `assets/shaders/pfd.wesl`. A PFD is a live digital screen, so unlike the MCP
//! it cannot be a photo: the reference photos are static and annotated.

use bevy::asset::Asset;
use bevy::prelude::*;
use bevy::render::render_resource::{AsBindGroup, ShaderType};
use bevy::shader::ShaderRef;
use bevy::ui_render::prelude::{MaterialNode, UiMaterial, UiMaterialPlugin};
use bevy::window::PrimaryWindow;

use crate::aircraft_profile;
use crate::app_state::AppState;
use crate::autopilot::{AtMode, Autopilot, LatMode, Mcp, SpeedUnit, VertMode};
use crate::mcp_ui::PANEL_W;
use crate::sim::{AircraftState, Sim};

/// Nominal display size in logical px. The screen is portrait, like the 737's.
const PFD_W: f32 = 400.0;
const PFD_H: f32 = 430.0;
/// Distance from the window's edge, and from the centred MCP panel.
const MARGIN: f32 = 12.0;
const SIDE_GAP: f32 = 12.0;
/// Below this the tape numerals stop being legible; keep it as a floor and let
/// the PFDs tuck closer to the MCP instead.
const MIN_W: f32 = 180.0;

/// Shader parameters. Field order and packing match `PfdParams` in
/// `assets/shaders/pfd.wesl` exactly.
#[derive(Clone, Copy, ShaderType, Default)]
struct PfdParams {
    /// cas_kts, alt_ft, vs_fpm, pitch_deg
    a: Vec4,
    /// roll_deg, heading_deg, selected_speed_kts, selected_alt_ft
    b: Vec4,
    /// selected_heading_deg, flap_position_norm, gs_kts, vr_kts
    c: Vec4,
    /// fd_pitch_deg, fd_bank_deg, baro_hpa, radio_alt_ft
    d: Vec4,
    /// engaged, ap_disc, on_ground, speed_is_mach
    e: Vec4,
    /// at_mode (0 off / 1 SPD / 2 N1), lat_mode (0 off / 1 HDG SEL),
    /// vert_mode (0 off / 1 ALT HOLD / 2 V/S / 3 LVL CHG), mach
    f: Vec4,
    /// time_s, slip, tas_kts, unused
    g: Vec4,
}

#[derive(Asset, TypePath, AsBindGroup, Clone, Default)]
struct PfdMaterial {
    #[uniform(0)]
    params: PfdParams,
}

impl UiMaterial for PfdMaterial {
    fn fragment_shader() -> ShaderRef {
        "shaders/pfd.wesl".into()
    }
}

#[derive(Resource)]
struct PfdMaterialHandle(Handle<PfdMaterial>);

/// Marks one of the two displays.
#[derive(Component)]
struct Pfd;

pub struct PfdPlugin;

impl Plugin for PfdPlugin {
    fn build(&self, app: &mut App) {
        app.add_plugins(UiMaterialPlugin::<PfdMaterial>::default())
            .add_systems(OnEnter(AppState::Flying), spawn)
            .add_systems(
                Update,
                (fit_to_window, update_params).run_if(in_state(AppState::Flying)),
            );
    }
}

/// The PFDs are 737-only: they need an autopilot (and the MCP it drives).
fn spawn(mut commands: Commands, sim: Res<Sim>, mut materials: ResMut<Assets<PfdMaterial>>) {
    if !aircraft_profile::profile(&sim.name).has_autopilot {
        return;
    }
    let material = materials.add(PfdMaterial::default());
    commands.insert_resource(PfdMaterialHandle(material.clone()));

    for left in [true, false] {
        commands.spawn((
            Pfd,
            MaterialNode(material.clone()),
            Node {
                position_type: PositionType::Absolute,
                left: if left { Val::Px(MARGIN) } else { Val::Auto },
                right: if left { Val::Auto } else { Val::Px(MARGIN) },
                bottom: Val::Px(MARGIN),
                width: Val::Px(PFD_W),
                height: Val::Px(PFD_H),
                ..default()
            },
            // Under the MCP/FMA strip, above the world.
            GlobalZIndex(14),
            DespawnOnExit(AppState::Flying),
        ));
    }
}

/// Width that fits between the window edge and the centred MCP panel. On the
/// intended fullscreen layout this is `PFD_W`; on narrower windows the PFD
/// shrinks (aspect held) rather than overlapping the panel.
fn pfd_width(window_w: f32) -> f32 {
    let room = (window_w - PANEL_W) * 0.5 - MARGIN - SIDE_GAP;
    room.clamp(MIN_W, PFD_W)
}

fn fit_to_window(
    windows: Query<&Window, With<PrimaryWindow>>,
    mut q: Query<&mut Node, With<Pfd>>,
) {
    let Ok(window) = windows.single() else {
        return;
    };
    let width = pfd_width(window.width());
    let height = width * (PFD_H / PFD_W);
    for mut node in &mut q {
        if node.width != Val::Px(width) {
            node.width = Val::Px(width);
            node.height = Val::Px(height);
        }
    }
}

fn update_params(
    state: Res<AircraftState>,
    mcp: Res<Mcp>,
    ap: Res<Autopilot>,
    time: Res<Time>,
    handle: Option<Res<PfdMaterialHandle>>,
    mut materials: ResMut<Assets<PfdMaterial>>,
) {
    let Some(handle) = handle else {
        return;
    };
    let Some(mut material) = materials.get_mut(&handle.0) else {
        return;
    };

    let flap_norm = (state.flap_deg / 40.0).clamp(0.0, 1.0);
    let speed_is_mach = mcp.speed_unit == SpeedUnit::Mach;
    material.params = PfdParams {
        a: Vec4::new(
            state.cas_kts as f32,
            state.alt_ft as f32,
            state.vs_fpm as f32,
            state.pitch_deg as f32,
        ),
        b: Vec4::new(
            state.roll_deg as f32,
            state.heading_deg as f32,
            if speed_is_mach { 0.0 } else { mcp.ias_kts as f32 },
            mcp.alt_ft as f32,
        ),
        c: Vec4::new(
            mcp.hdg_deg as f32,
            flap_norm as f32,
            state.gs_kts as f32,
            vr_for_flaps(flap_norm) as f32,
        ),
        d: Vec4::new(
            ap.fd_pitch_deg as f32,
            ap.fd_bank_deg as f32,
            1013.0,
            state.radar_alt_ft as f32,
        ),
        e: Vec4::new(
            ap.engaged as u8 as f32,
            ap.disc_annunciated() as u8 as f32,
            state.on_ground as u8 as f32,
            speed_is_mach as u8 as f32,
        ),
        f: Vec4::new(
            at_mode(ap.at),
            lat_mode(ap.lat),
            vert_mode(ap.vert),
            state.mach as f32,
        ),
        g: Vec4::new(time.elapsed_secs() as f32, state.slip as f32, state.tas_kts as f32, 0.0),
    };
}

/// Rotation speed for the current flap setting, from the takeoff probe in this
/// sim: clean ≈185 kt, flap 0.25 ≈171 kt, flap 0.5 ≈160 kt. Linear in the
/// flap handle between those points.
fn vr_for_flaps(flap_norm: f64) -> f64 {
    (185.0 - 50.0 * flap_norm).clamp(135.0, 185.0)
}

fn at_mode(mode: AtMode) -> f32 {
    match mode {
        AtMode::Off => 0.0,
        AtMode::Speed => 1.0,
        AtMode::N1 => 2.0,
    }
}

fn lat_mode(mode: LatMode) -> f32 {
    match mode {
        LatMode::Off => 0.0,
        LatMode::HdgSel => 1.0,
    }
}

fn vert_mode(mode: VertMode) -> f32 {
    match mode {
        VertMode::Off => 0.0,
        VertMode::AltHold => 1.0,
        VertMode::Vs => 2.0,
        VertMode::LvlChg => 3.0,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn vr_falls_as_flaps_extend() {
        assert!((vr_for_flaps(0.0) - 185.0).abs() < 1e-9);
        assert!((vr_for_flaps(0.25) - 172.5).abs() < 1e-9);
        assert!((vr_for_flaps(0.5) - 160.0).abs() < 1e-9);
        // Clamped at the bottom for full flap.
        assert!((vr_for_flaps(1.0) - 135.0).abs() < 1e-9);
    }

    #[test]
    fn display_shrinks_before_it_overlaps_the_mcp() {
        // Nominal fullscreen: the full 400 px display fits on each side.
        assert_eq!(pfd_width(2048.0), PFD_W);
        // 1900 logical: (1900 - 1100)/2 - 24 = 376.
        assert!((pfd_width(1900.0) - 376.0).abs() < 1e-6);
        // Below the floor the width stops shrinking.
        assert_eq!(pfd_width(1280.0), MIN_W);
        // The right edge never crosses the MCP's left edge.
        for w in [1280.0, 1536.0, 1900.0, 2048.0, 2560.0] {
            let panel_left = (w - PANEL_W) * 0.5;
            assert!(MARGIN + pfd_width(w) <= panel_left.max(MARGIN + MIN_W));
        }
    }
}
