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
use crate::ils;
use crate::mcp_ui::PANEL_W;
use crate::sim::{AircraftState, Sim};
use crate::view::{CockpitModel, Ui};

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
    /// at_mode (0 off / 1 SPD / 2 N1), lat_mode (0 off / 1 HDG SEL / 2 LOC),
    /// vert_mode (0 off / 1 ALT HOLD / 2 V/S / 3 LVL CHG / 4 G/S / 5 FLARE),
    /// mach
    f: Vec4,
    /// time_s, slip, tas_kts, app_armed
    g: Vec4,
    /// speedbrake_pos, reverse_pos, loc_dev_dots, gs_dev_dots
    h: Vec4,
    /// gpws_glideslope, gpws_sink_rate, unused, unused
    i: Vec4,
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

/// The same symbology, on a quad lying over one of the cockpit model's display
/// units. Unlike the UI overlay it is unlit, so the shader's colours land as
/// written, and it is parented to the cockpit model (hidden with it outside
/// the cockpit view).
#[derive(Asset, TypePath, AsBindGroup, Clone, Default)]
struct Pfd3dMaterial {
    #[uniform(0)]
    params: PfdParams,
}

impl Material for Pfd3dMaterial {
    fn fragment_shader() -> ShaderRef {
        "shaders/pfd3d.wesl".into()
    }

    fn alpha_mode(&self) -> AlphaMode {
        AlphaMode::Opaque
    }

    fn enable_prepass() -> bool {
        false
    }

    fn enable_shadows() -> bool {
        false
    }
}

#[derive(Resource)]
struct PfdMaterialHandle(Handle<PfdMaterial>);

#[derive(Resource)]
struct Pfd3dMaterialHandle(Handle<Pfd3dMaterial>);

/// Marks one of the two displays.
#[derive(Component)]
struct Pfd;

/// Marks one of the in-cockpit display quads.
#[derive(Component)]
struct Pfd3d;

/// The two inboard display units (captain's and first officer's PFDs), in the
/// cockpit model's frame (+X left, +Y up, nose +Z), measured from the glTF.
/// The black screen islands are ~0.101 square; the quad sits a couple of
/// millimetres in front of them, facing aft toward the pilots.
const PFD3D_CENTRES: [[f32; 3]; 2] = [[0.1983, 1.8782, 1.455], [-0.1549, 1.8782, 1.455]];
const PFD3D_W: f32 = 0.098;
const PFD3D_H: f32 = 0.105;

pub struct PfdPlugin;

impl Plugin for PfdPlugin {
    fn build(&self, app: &mut App) {
        app.add_plugins((
            UiMaterialPlugin::<PfdMaterial>::default(),
            MaterialPlugin::<Pfd3dMaterial>::default(),
        ))
        .add_systems(
            OnEnter(AppState::Flying),
            spawn.after(crate::sim::StartFlight),
        )
        .add_systems(
            Update,
            (fit_to_window, cockpit_visibility, spawn_3d, update_params)
                .chain()
                .run_if(in_state(AppState::Flying)),
        );
    }
}

/// The PFDs are 737-only: they need an autopilot (and the MCP it drives).
fn spawn(
    mut commands: Commands,
    sim: Res<Sim>,
    mut materials: ResMut<Assets<PfdMaterial>>,
    mut materials_3d: ResMut<Assets<Pfd3dMaterial>>,
) {
    if !aircraft_profile::profile(&sim.name).has_autopilot {
        return;
    }
    let material = materials.add(PfdMaterial::default());
    commands.insert_resource(PfdMaterialHandle(material.clone()));
    commands.insert_resource(Pfd3dMaterialHandle(
        materials_3d.add(Pfd3dMaterial::default()),
    ));

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

fn fit_to_window(windows: Query<&Window, With<PrimaryWindow>>, mut q: Query<&mut Node, With<Pfd>>) {
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

/// The 2D corner PFDs are chase-view instruments; the cockpit has the real
/// screens. Hide the overlays while the pilot's eye is inside.
fn cockpit_visibility(ui: Res<Ui>, mut q: Query<&mut Node, With<Pfd>>) {
    let display = if ui.cockpit {
        Display::None
    } else {
        Display::Flex
    };
    for mut node in &mut q {
        if node.display != display {
            node.display = display;
        }
    }
}

/// Put the PFDs on the 737's two inboard display units once the cockpit model
/// exists. The quads are children of that model, so they inherit its pose and
/// visibility (hidden in the chase view, where the model itself is hidden).
fn spawn_3d(
    mut commands: Commands,
    sim: Res<Sim>,
    cockpit: Query<Entity, With<CockpitModel>>,
    existing: Query<(), With<Pfd3d>>,
    handle: Option<Res<Pfd3dMaterialHandle>>,
    mut meshes: ResMut<Assets<Mesh>>,
) {
    if !matches!(sim.name.as_str(), "737" | "b737") || !existing.is_empty() {
        return;
    }
    let (Some(handle), Ok(model)) = (handle, cockpit.single()) else {
        return;
    };
    let mesh = meshes.add(Rectangle::new(PFD3D_W, PFD3D_H));
    for centre in PFD3D_CENTRES {
        commands.spawn((
            Pfd3d,
            Mesh3d(mesh.clone()),
            MeshMaterial3d(handle.0.clone()),
            Transform {
                translation: Vec3::from_array(centre),
                // The model's screens face aft; a Rectangle's normal is +Z.
                rotation: Quat::from_rotation_y(std::f32::consts::PI),
                ..default()
            },
            ChildOf(model),
        ));
    }
}

fn update_params(
    state: Res<AircraftState>,
    mcp: Res<Mcp>,
    ap: Res<Autopilot>,
    ils: Res<crate::ils::Ils>,
    time: Res<Time>,
    ui_handle: Option<Res<PfdMaterialHandle>>,
    du_handle: Option<Res<Pfd3dMaterialHandle>>,
    mut materials: ResMut<Assets<PfdMaterial>>,
    mut materials_3d: ResMut<Assets<Pfd3dMaterial>>,
) {
    let (Some(ui_handle), Some(du_handle)) = (ui_handle, du_handle) else {
        return;
    };

    let flap_norm = (state.flap_deg / 40.0).clamp(0.0, 1.0);
    let speed_is_mach = mcp.speed_unit == SpeedUnit::Mach;

    // The on-ils view: the scales show while APP is armed (satisfying the
    // armed word too) or while LOC/GS is engaged.
    let on_ils = ap.engaged && (ap.app_armed || ap.lat == LatMode::Loc);
    let info = on_ils && ils.available;

    // GPWS-lite: a beam we are flying and dropping under, or too much sink
    // with the ground closing.
    let gs_below = info
        && matches!(ap.vert, VertMode::Gs | VertMode::Flare)
        && ils.gs_dev_deg < -ils::GS_WARN_DEG;
    let sink_rate = !state.on_ground
        && state.radar_alt_ft < 2_000.0
        && state.vs_fpm < -1_400.0;

    let params = PfdParams {
        a: Vec4::new(
            state.cas_kts as f32,
            state.alt_ft as f32,
            state.vs_fpm as f32,
            state.pitch_deg as f32,
        ),
        b: Vec4::new(
            state.roll_deg as f32,
            state.heading_deg as f32,
            if speed_is_mach {
                0.0
            } else {
                mcp.ias_kts as f32
            },
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
        g: Vec4::new(
            time.elapsed_secs() as f32,
            state.slip as f32,
            state.tas_kts as f32,
            (on_ils && ap.app_armed) as u8 as f32,
        ),
        h: Vec4::new(
            state.speedbrake_pos as f32,
            state.reverse_pos as f32,
            // Deviations in receiver dots while the ILS is live.
            if info { (ils.loc_dev_deg / ils::LOC_DOT_DEG) as f32 } else { 0.0 },
            if info { (ils.gs_dev_deg / ils::GS_DOT_DEG) as f32 } else { 0.0 },
        ),
        i: Vec4::new(
            gs_below as u8 as f32,
            sink_rate as u8 as f32,
            0.0,
            0.0,
        ),
    };
    if let Some(mut material) = materials.get_mut(&ui_handle.0) {
        material.params = params;
    }
    if let Some(mut material) = materials_3d.get_mut(&du_handle.0) {
        material.params = params;
    }
}

/// Rotation speed for the current flap setting, from the takeoff probe in this
/// sim: clean ≈185 kt, flap 0.25 ≈171 kt, flap 0.5 ≈160 kt. Linear in the
/// flap handle between those points. The takeoff report card grades the
/// liftoff speed against this same line.
pub(crate) fn vr_for_flaps(flap_norm: f64) -> f64 {
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
        LatMode::Loc => 2.0,
    }
}

fn vert_mode(mode: VertMode) -> f32 {
    match mode {
        VertMode::Off => 0.0,
        VertMode::AltHold => 1.0,
        VertMode::Vs => 2.0,
        VertMode::LvlChg => 3.0,
        VertMode::Gs => 4.0,
        VertMode::Flare => 5.0,
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
