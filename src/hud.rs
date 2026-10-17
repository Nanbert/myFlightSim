//! F-22A head-up display.
//!
//! A full-screen [`UiMaterial`] overlay drawn procedurally by
//! `assets/shaders/f22_hud.wesl` — heading tape, bank scale, earth-stabilised
//! pitch ladder and horizon, flight path and waterline markers, the KCAS and
//! altitude dials with their fine-scale needles, and the data blocks. The
//! symbology follows the HUD chapter of the TMS F-22A manual; the quantities it
//! needs are published each frame on [`AircraftState`], and this module only
//! packs them into the shader's uniform.
//!
//! The overlay is F-22-only and lives above every other UI layer, so it is
//! hidden in the menu and while flying any other aircraft (`sync_visibility`).

use bevy::asset::Asset;
use bevy::prelude::*;
use bevy::render::render_resource::{AsBindGroup, ShaderType};
use bevy::shader::ShaderRef;
use bevy::ui_render::prelude::{MaterialNode, UiMaterial, UiMaterialPlugin};

use crate::app_state::AppState;
use crate::sim::{AircraftState, Sim};

/// One knot in feet per second.
const FPS_TO_KT: f64 = 0.592_483_801;

/// Uniform for `assets/shaders/f22_hud.wesl`. Field order and packing match the
/// WGSL `HudParams` struct exactly.
#[derive(Clone, Copy, ShaderType, Default)]
struct HudParams {
    /// cas_kts, alt_ft, vs_fpm, pitch_deg
    a: Vec4,
    /// roll_deg, heading_deg, alpha_deg, beta_deg
    b: Vec4,
    /// mach, g, max_g, gs_kts
    c: Vec4,
    /// tas_kts, radar_alt_ft, fpm_x_deg, fpm_y_deg
    d: Vec4,
    /// speed_trend_kt_s, wind_from_deg, wind_kt, gear_pos
    e: Vec4,
}

#[derive(Asset, TypePath, AsBindGroup, Clone, Default)]
struct HudMaterial {
    #[uniform(0)]
    params: HudParams,
}

impl UiMaterial for HudMaterial {
    fn fragment_shader() -> ShaderRef {
        "shaders/f22_hud.wesl".into()
    }
}

#[derive(Component)]
struct HudOverlay;

pub struct HudPlugin;

impl Plugin for HudPlugin {
    fn build(&self, app: &mut App) {
        app.add_plugins(UiMaterialPlugin::<HudMaterial>::default())
            .add_systems(Startup, setup)
            .add_systems(Update, (update, sync_visibility));
    }
}

fn setup(mut commands: Commands, mut materials: ResMut<Assets<HudMaterial>>) {
    let mat = materials.add(HudMaterial::default());
    commands.spawn((
        HudOverlay,
        MaterialNode(mat),
        Node {
            position_type: PositionType::Absolute,
            left: Val::Px(0.0),
            top: Val::Px(0.0),
            width: Val::Percent(100.0),
            height: Val::Percent(100.0),
            // Hidden until we're flying the F-22; see `sync_visibility`.
            display: Display::None,
            ..default()
        },
        // Above the gauges and the in-world UI.
        GlobalZIndex(20),
    ));
}

/// Pack this frame's state into the shader uniform.
fn update(
    sim: Res<Sim>,
    state: Res<AircraftState>,
    mut materials: ResMut<Assets<HudMaterial>>,
    q: Query<&MaterialNode<HudMaterial>>,
) {
    if sim.name != "f22" {
        return;
    }
    let Ok(node) = q.single() else {
        return;
    };
    let Some(mut mat) = materials.get_mut(&node.0) else {
        return;
    };

    let (wind_from, wind_kt) = wind(state.wind_north_fps, state.wind_east_fps);
    mat.params = HudParams {
        a: Vec4::new(
            state.cas_kts as f32,
            state.alt_ft as f32,
            state.vs_fpm as f32,
            state.pitch_deg as f32,
        ),
        b: Vec4::new(
            state.roll_deg as f32,
            state.heading_deg as f32,
            state.alpha_deg as f32,
            state.beta_deg as f32,
        ),
        c: Vec4::new(
            state.mach as f32,
            state.g_norm as f32,
            state.max_g_norm as f32,
            state.gs_kts as f32,
        ),
        d: Vec4::new(
            state.tas_kts as f32,
            // -1 blanks the readout; the shader's field is four digits.
            if state.radar_alt_ft < 10_000.0 {
                state.radar_alt_ft as f32
            } else {
                -1.0
            },
            wrap180(state.track_deg - state.heading_deg) as f32,
            (state.pitch_deg - state.flight_path_deg) as f32,
        ),
        e: Vec4::new(
            state.speed_trend as f32,
            wind_from,
            wind_kt,
            state.gear_pos as f32,
        ),
    };
}

/// HUD only exists for the F-22, and only while flying it.
fn sync_visibility(
    state: Res<State<AppState>>,
    sim: Res<Sim>,
    mut q: Query<&mut Node, With<HudOverlay>>,
) {
    let want = if *state.get() == AppState::Flying && sim.name == "f22" {
        Display::Flex
    } else {
        Display::None
    };
    for mut node in &mut q {
        if node.display != want {
            node.display = want;
        }
    }
}

/// Wrap an angle difference into `[-180, 180)`, so the flight path marker takes
/// the short way across the 0°/360° seam.
fn wrap180(deg: f64) -> f64 {
    (deg + 180.0).rem_euclid(360.0) - 180.0
}

/// Wind direction the air comes *from* (deg true) and its speed (kt), from the
/// FDM's north/east wind velocity components.
fn wind(north_fps: f64, east_fps: f64) -> (f32, f32) {
    let kt = north_fps.hypot(east_fps) * FPS_TO_KT;
    // Calm air has no direction (atan2 would hand back a meaningless 180°).
    if kt < 0.5 {
        return (0.0, 0.0);
    }
    let from = (-east_fps).atan2(-north_fps).to_degrees().rem_euclid(360.0);
    (from as f32, kt as f32)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn wrap180_takes_the_short_way_round() {
        assert!((wrap180(10.0) - 10.0).abs() < 1e-9);
        assert!((wrap180(350.0) - -10.0).abs() < 1e-9);
        assert!((wrap180(-350.0) - 10.0).abs() < 1e-9);
        assert!(wrap180(180.0) >= -180.0 && wrap180(180.0) < 180.0);
    }

    #[test]
    fn wind_is_reported_as_the_direction_it_comes_from() {
        // Air moving toward the north comes from the south (180°).
        let (from, kt) = wind(100.0, 0.0);
        assert!((from - 180.0).abs() < 1e-3, "from {from}");
        // Air moving toward the east comes from the west (270°).
        let (from, _) = wind(0.0, 100.0);
        assert!((from - 270.0).abs() < 1e-3, "from {from}");
        // Calm.
        assert_eq!(wind(0.0, 0.0), (0.0, 0.0));
        // 100 fps ≈ 59 kt.
        assert!((kt - 59.25).abs() < 0.5, "kt {kt}");
    }
}
