//! F-22A head-up display.
//!
//! Symbology drawn procedurally by `assets/shaders/f22_hud_sym.wesl` — heading
//! tape, bank scale, earth-stabilised pitch ladder and horizon, flight path and
//! waterline markers, the KCAS and altitude dials with their fine-scale needles,
//! and the data blocks. It follows the HUD chapter of the TMS F-22A manual; the
//! quantities it needs are published each frame on [`AircraftState`], and this
//! module only packs them into the shader's uniform.
//!
//! Two views consume the same layout:
//!
//! - the chase view uses a full-screen [`UiMaterial`] overlay, above every
//!   other UI layer;
//! - the cockpit view draws it on a quad sitting on the combiner glass of the
//!   F-22 cockpit model, so the symbology belongs to the aircraft instead of
//!   the screen and moves with free-look.
//!
//! Both are F-22-only and hidden in the menu and for other aircraft.

use bevy::asset::Asset;
use bevy::prelude::*;
use bevy::render::render_resource::{AsBindGroup, ShaderType};
use bevy::shader::ShaderRef;
use bevy::ui_render::prelude::{MaterialNode, UiMaterial, UiMaterialPlugin};

use crate::app_state::AppState;
use crate::sim::{AircraftState, Sim};
use crate::view::{CockpitModel, Ui};

/// One knot in feet per second.
const FPS_TO_KT: f64 = 0.592_483_801;

/// Uniform for both HUD materials. Field order and packing match the WGSL
/// `HudParams` struct in `assets/shaders/f22_hud_sym.wesl` exactly.
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
    /// Combiner layout only: xy = virtual screen size, z = downward shift that
    /// centres the layout on the glass, w = unused.
    f: Vec4,
    /// Arcade combat stores: IR missiles, BVR missiles, gun rounds, flares.
    /// A negative IR count hides the block (no combat in this flight).
    g: Vec4,
    /// Kills, selected weapon (0 IR / 1 BVR), unused.
    h: Vec4,
}

/// Arcade combat stores shown in the HUD's weapon block. `combat.rs` fills
/// this each frame; a negative missile count hides the block (no fight).
#[derive(Resource)]
pub(crate) struct HudWeapons {
    /// Sidewinder-class rounds; negative hides the whole stores block.
    pub(crate) ir: i32,
    pub(crate) bvr: i32,
    /// 0 = IR selected, 1 = BVR.
    pub(crate) selected: u32,
    pub(crate) rounds: i32,
    pub(crate) flares: i32,
    pub(crate) kills: u32,
}

impl Default for HudWeapons {
    fn default() -> Self {
        HudWeapons {
            ir: -1,
            bvr: 0,
            selected: 0,
            rounds: 0,
            flares: 0,
            kills: 0,
        }
    }
}

/// Uniform for the AH-64D IHADSS overlay. Field order matches `Ah64Params` in
/// `assets/shaders/ah64_hud_sym.wesl` exactly.
#[derive(Clone, Copy, ShaderType, Default)]
struct Ah64Params {
    /// cas_kts, alt_ft, vs_fpm, radar_alt_ft
    a: Vec4,
    /// heading_deg, pitch_deg, roll_deg, collective_pct
    b: Vec4,
    /// torque_pct, rotor_rpm_pct, fpm_x_deg, fpm_y_deg
    c: Vec4,
    /// gs_kts, g, waypoint_km, waypoint_bearing_deg
    d: Vec4,
    /// arcade, hover_state (0 off/1 takeoff/2 hold/3 land), balance, on_ground
    e: Vec4,
    /// layout (unused by the screen overlay)
    f: Vec4,
}

#[derive(Asset, TypePath, AsBindGroup, Clone, Default)]
struct Ah64Material {
    #[uniform(0)]
    params: Ah64Params,
}

impl UiMaterial for Ah64Material {
    fn fragment_shader() -> ShaderRef {
        "shaders/ah64_hud.wesl".into()
    }
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

/// The same symbology drawn on the cockpit model's combiner glass. Transparent
/// and unlit: the shader writes the phosphor colour directly.
#[derive(Asset, TypePath, AsBindGroup, Clone)]
struct HudGlassMaterial {
    #[uniform(0)]
    params: HudParams,
}

impl Material for HudGlassMaterial {
    fn fragment_shader() -> ShaderRef {
        "shaders/f22_hud_glass.wesl".into()
    }

    fn alpha_mode(&self) -> AlphaMode {
        AlphaMode::Blend
    }

    fn enable_prepass() -> bool {
        false
    }

    fn enable_shadows() -> bool {
        false
    }
}

#[derive(Resource)]
struct GlassMaterial(Handle<HudGlassMaterial>);

#[derive(Component)]
struct HudOverlay;

/// Full-screen AH-64D IHADSS overlay (shown in both chase and cockpit views).
#[derive(Component)]
struct Ah64Overlay;

/// The quad on the combiner glass. A child of the cockpit model, so it lives
/// in that model's frame and hides with it outside the cockpit view.
#[derive(Component)]
struct GlassHud;

/// Live placement of the combiner quad, tunable from the cockpit view:
/// `]`/`[` slide it toward the eye / back toward the glass, `.`/`,` scale it
/// about its centre. Values are printed as they change.
#[derive(Resource)]
struct GlassTune {
    /// Model units (0.1 m) pulled from the glass toward the pilot.
    pull: f32,
    scale: f32,
}

impl Default for GlassTune {
    fn default() -> Self {
        Self {
            pull: 0.0,
            scale: 1.0,
        }
    }
}

const PULL_STEP: f32 = 0.2;
const PULL_MIN: f32 = -0.5;
const PULL_MAX: f32 = 6.0;
const SCALE_STEP: f32 = 0.05;
const SCALE_MIN: f32 = 0.3;
const SCALE_MAX: f32 = 3.0;

/// Combiner glass in the F-22 cockpit model's frame (10 units/m, +X nose,
/// +Y up, +Z right), read off its `hud` mesh: a plate tilted 17° back from
/// vertical. The symbology quad sits a few millimetres in front of it (toward
/// the pilot), sharing its tilt.
const GLASS_CENTRE: Vec3 = Vec3::new(100.438, 11.083, 0.0);
const GLASS_UP: Vec3 = Vec3::new(-0.2959, 0.9552, 0.0);
const GLASS_NORMAL: Vec3 = Vec3::new(-0.9552, -0.2959, 0.0);
const GLASS_SIZE: Vec2 = Vec2::new(1.836, 2.168);
/// Virtual "screen" the symbology is laid out on, in pixel-like units: the
/// shader's line widths and text sizes are fractions of this height. Its aspect
/// matches the glass so the symbology isn't stretched.
const GLASS_LAYOUT: Vec2 = Vec2::new(846.0, 1000.0);
/// The layout's content sits in the upper part of its virtual screen; shifting
/// the sampling down by this fraction of its height centres it on the glass.
const GLASS_SHIFT: f32 = 0.183;

/// Whether the F-22 symbology is drawn. `O` toggles it.
#[derive(Resource)]
struct HudShown(bool);

impl Default for HudShown {
    fn default() -> Self {
        Self(true)
    }
}

pub struct HudPlugin;

impl Plugin for HudPlugin {
    fn build(&self, app: &mut App) {
        app.add_plugins((
            UiMaterialPlugin::<HudMaterial>::default(),
            UiMaterialPlugin::<Ah64Material>::default(),
            MaterialPlugin::<HudGlassMaterial>::default(),
        ))
        .init_resource::<HudShown>()
        .init_resource::<GlassTune>()
        .init_resource::<HudWeapons>()
        .add_systems(Startup, setup)
        .add_systems(
            Update,
            (
                spawn_glass,
                tune_glass,
                update,
                update_ah64,
                toggle_shown.run_if(in_state(AppState::Flying)),
                sync_visibility,
                sync_visibility_ah64,
            ),
        );
    }
}

fn setup(
    mut commands: Commands,
    mut materials: ResMut<Assets<HudMaterial>>,
    mut ah64_materials: ResMut<Assets<Ah64Material>>,
    mut glass: ResMut<Assets<HudGlassMaterial>>,
) {
    let glass = glass.add(HudGlassMaterial {
        params: HudParams {
            f: glass_layout(),
            ..default()
        },
    });
    commands.insert_resource(GlassMaterial(glass));

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

    let ah64 = ah64_materials.add(Ah64Material::default());
    commands.spawn((
        Ah64Overlay,
        MaterialNode(ah64),
        Node {
            position_type: PositionType::Absolute,
            left: Val::Px(0.0),
            top: Val::Px(0.0),
            width: Val::Percent(100.0),
            height: Val::Percent(100.0),
            display: Display::None,
            ..default()
        },
        GlobalZIndex(20),
    ));
}

/// The combiner's virtual layout size plus the shift that centres the content.
fn glass_layout() -> Vec4 {
    Vec4::new(GLASS_LAYOUT.x, GLASS_LAYOUT.y, GLASS_SHIFT, 0.0)
}

/// Pack this frame's state into the shader uniform, for both the screen overlay
/// and the combiner-glass material.
fn update(
    sim: Res<Sim>,
    state: Res<AircraftState>,
    weapons: Res<HudWeapons>,
    mut materials: ResMut<Assets<HudMaterial>>,
    mut glass: ResMut<Assets<HudGlassMaterial>>,
    overlay: Query<&MaterialNode<HudMaterial>>,
    on_glass: Query<&MeshMaterial3d<HudGlassMaterial>>,
) {
    if sim.name != "f22" {
        return;
    }

    let (wind_from, wind_kt) = wind(state.wind_north_fps, state.wind_east_fps);
    let params = HudParams {
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
        f: glass_layout(),
        // Negative IR count hides the stores block outside an arcade fight.
        g: Vec4::new(
            weapons.ir as f32,
            weapons.bvr as f32,
            weapons.rounds as f32,
            weapons.flares as f32,
        ),
        h: Vec4::new(weapons.kills as f32, weapons.selected as f32, 0.0, 0.0),
    };

    if let Ok(node) = overlay.single() {
        if let Some(mut mat) = materials.get_mut(&node.0) {
            mat.params = params;
        }
    }
    for node in &on_glass {
        if let Some(mut mat) = glass.get_mut(&node.0) {
            mat.params = params;
        }
    }
}

/// Spawn the combiner quad once the cockpit model exists. Parenting it to that
/// model carries the model's scale/yaw/offset and its `Visibility`, so the quad
/// appears exactly in the cockpit view and nowhere else.
fn spawn_glass(
    mut commands: Commands,
    sim: Res<Sim>,
    cockpit: Query<Entity, With<CockpitModel>>,
    existing: Query<(), With<GlassHud>>,
    glass: Res<GlassMaterial>,
    mut meshes: ResMut<Assets<Mesh>>,
) {
    if sim.name != "f22" || !existing.is_empty() {
        return;
    }
    let Ok(model) = cockpit.single() else {
        return;
    };
    let rotation = Quat::from_mat3(&Mat3::from_cols(Vec3::Z, GLASS_UP, GLASS_NORMAL));
    commands.spawn((
        GlassHud,
        Mesh3d(meshes.add(Rectangle::new(GLASS_SIZE.x, GLASS_SIZE.y))),
        MeshMaterial3d(glass.0.clone()),
        Transform {
            translation: GLASS_CENTRE,
            rotation,
            ..default()
        },
        ChildOf(model),
    ));
}

/// Live placement of the combiner quad, adjustable from the cockpit view. The
/// quad rides an offset along the glass normal (toward the pilot) and is scaled
/// about its centre, so the symbology can be brought closer and made larger
/// than the physical glass. Prints the current values whenever they change.
fn tune_glass(
    keys: Res<ButtonInput<KeyCode>>,
    ui: Res<Ui>,
    mut tune: ResMut<GlassTune>,
    mut hud: Query<&mut Transform, With<GlassHud>>,
) {
    if ui.cockpit {
        let mut changed = false;
        if keys.just_pressed(KeyCode::BracketRight) {
            tune.pull = (tune.pull + PULL_STEP).min(PULL_MAX);
            changed = true;
        }
        if keys.just_pressed(KeyCode::BracketLeft) {
            tune.pull = (tune.pull - PULL_STEP).max(PULL_MIN);
            changed = true;
        }
        if keys.just_pressed(KeyCode::Period) {
            tune.scale = (tune.scale + SCALE_STEP).min(SCALE_MAX);
            changed = true;
        }
        if keys.just_pressed(KeyCode::Comma) {
            tune.scale = (tune.scale - SCALE_STEP).max(SCALE_MIN);
            changed = true;
        }
        if changed {
            info!("hud glass: pull={:.2} scale={:.2}", tune.pull, tune.scale);
        }
    }
    for mut transform in &mut hud {
        transform.translation = GLASS_CENTRE + GLASS_NORMAL * tune.pull;
        transform.scale = Vec3::splat(tune.scale);
    }
}

fn toggle_shown(keys: Res<ButtonInput<KeyCode>>, mut shown: ResMut<HudShown>) {
    if keys.just_pressed(KeyCode::KeyO) {
        shown.0 = !shown.0;
    }
}

/// The HUD exists only for the F-22, only while flying it, and only while the
/// pilot hasn't toggled it off with `O`. Inside the cockpit the symbology moves
/// onto the combiner glass, so the full-screen overlay is hidden there.
fn sync_visibility(
    state: Res<State<AppState>>,
    sim: Res<Sim>,
    shown: Res<HudShown>,
    ui: Res<Ui>,
    mut overlay: Query<&mut Node, With<HudOverlay>>,
    mut glass: Query<&mut Visibility, With<GlassHud>>,
) {
    let want = if *state.get() == AppState::Flying && sim.name == "f22" && shown.0 && !ui.cockpit {
        Display::Flex
    } else {
        Display::None
    };
    for mut node in &mut overlay {
        if node.display != want {
            node.display = want;
        }
    }
    // The quad's parent (the cockpit model) already hides it outside the
    // cockpit view; this only honours the `O` toggle.
    let vis = if shown.0 {
        Visibility::Inherited
    } else {
        Visibility::Hidden
    };
    for mut v in &mut glass {
        if *v != vis {
            *v = vis;
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

/// Pack the AH-64D state into the IHADSS uniform.
fn update_ah64(
    sim: Res<Sim>,
    state: Res<AircraftState>,
    mut materials: ResMut<Assets<Ah64Material>>,
    overlay: Query<&MaterialNode<Ah64Material>>,
) {
    if sim.name != "ah1s" {
        return;
    }

    // The AH-1S rotor governor runs at 324 rpm; display it as a percentage.
    let nr_pct = if state.main_rotor_rpm > 1.0 {
        (state.main_rotor_rpm / 324.0 * 100.0) as f32
    } else {
        0.0
    };
    // Rotor reaction torque is in lbf·ft; 20,000 ft·lb is the nominal 100%.
    let tq_pct = (state.torque_lbsft / 200.0) as f32;

    let params = Ah64Params {
        a: Vec4::new(
            state.cas_kts as f32,
            state.alt_ft as f32,
            state.vs_fpm as f32,
            if state.radar_alt_ft < 10_000.0 {
                state.radar_alt_ft as f32
            } else {
                -1.0
            },
        ),
        b: Vec4::new(
            state.heading_deg as f32,
            state.pitch_deg as f32,
            state.roll_deg as f32,
            (state.collective * 100.0) as f32,
        ),
        c: Vec4::new(
            tq_pct,
            nr_pct,
            wrap180(state.track_deg - state.heading_deg) as f32,
            (state.pitch_deg - state.flight_path_deg) as f32,
        ),
        d: Vec4::new(state.gs_kts as f32, state.g_norm as f32, 5.2, 0.0),
        e: Vec4::new(
            state.arcade as i32 as f32,
            state.heli_hover_state as f32,
            state.heli_balance as i32 as f32,
            state.on_ground as i32 as f32,
        ),
        f: Vec4::ZERO,
    };

    if let Ok(node) = overlay.single() {
        if let Some(mut mat) = materials.get_mut(&node.0) {
            mat.params = params;
        }
    }
}

/// The AH-64D overlay lives in both the chase and cockpit views (the real
/// aircraft has an IHADSS monocle, not a fixed combiner glass).
fn sync_visibility_ah64(
    state: Res<State<AppState>>,
    sim: Res<Sim>,
    shown: Res<HudShown>,
    mut overlay: Query<&mut Node, With<Ah64Overlay>>,
) {
    let want = if *state.get() == AppState::Flying && sim.name == "ah1s" && shown.0 {
        Display::Flex
    } else {
        Display::None
    };
    for mut node in &mut overlay {
        if node.display != want {
            node.display = want;
        }
    }
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
