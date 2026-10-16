//! F-22 engine effects.
//!
//! Two effects, both gated to the F-22 (`sim.name == "f22"`), because only that
//! model ships the geometry (measured nozzle anchors) and only its FDM publishes
//! the signals they are driven from:
//!
//! - **Afterburner plume** — a stream of billboard puffs per nozzle
//!   ([`FlamePuffMaterial`] + `assets/shaders/flame_puff.wesl`). The burner is a
//!   hard step in the FDM (full reheat above 99 % throttle), so it is latched
//!   off the throttle command rather than eased in with it: dry power shows a
//!   short warm stub, and reheat stretches it out and drives it up the
//!   temperature ramp — a blue-white core at the nozzle, orange through the
//!   body, a dull red tail — with shock diamonds, flaring once as it lights. The
//!   camera's bloom pass (see `view.rs`) turns the >1 hot core into a glow.
//! - **Exhaust heat haze** — a screen-space refraction behind the nozzles
//!   ([`ExhaustHaze`] + `assets/shaders/exhaust_haze.wesl`), riding on Bevy's
//!   `FullscreenMaterial` exactly like the rain-on-lens pass in `lens.rs`.
//!
//! The plume is rigidly attached to the airframe: every puff's position is a
//! downstream offset in the *body* frame, so the plume sweeps with the aircraft
//! and curves correctly through a turn. That is also why it is not the
//! world-space trail the wingtip vapor uses — a plume is only a few metres long,
//! and at 300 m/s a world-space ring would recycle it inside two frames.
//!
//! The plugin is self-contained: it spawns its entities once `AircraftRoot`
//! exists (see [`spawn_fx`]) rather than hooking the flight spawn, so examples
//! that build their own scene (and never add this plugin) are unaffected, and
//! there is no cross-plugin ordering to get wrong. Every entity carries
//! `DespawnOnExit` so a fresh flight starts clean.

use bevy::core_pipeline::fullscreen_material::{FullscreenMaterial, FullscreenMaterialPlugin};
use bevy::prelude::*;
use bevy::render::extract_component::ExtractComponent;
use bevy::render::render_resource::{AsBindGroup, ShaderType};
use bevy::render::RenderApp;
use bevy::shader::ShaderRef;

use crate::app_state::AppState;
use crate::sim::{AircraftState, Sim};
use crate::view::{AircraftRoot, model_to_body};

/// Engine nozzle exits in the F-22's own glTF frame (metres, +X left / +Y up /
/// +Z nose), measured from the model geometry. Left is +X in that frame.
const F22_NOZZLE_L: Vec3 = Vec3::new(0.16, 0.145, -1.28);
const F22_NOZZLE_R: Vec3 = Vec3::new(-0.16, 0.145, -1.28);

/// One plume per nozzle.
const NOZZLES: usize = 2;
/// Puffs per plume. One is emitted every [`STEP`] metres, so this is how many
/// the reheat plume holds at once; it also sets how finely the flame is sampled
/// — enough that consecutive puffs overlap into a continuous column at the
/// nozzle, few enough that the individual puffs are still visible where the
/// plume frays at the tail.
const SAMPLES: usize = 36;

/// Distance (m) between puffs along the axis. With [`AB_REACH`] it fixes the
/// emission rate (`AXIAL / STEP` puffs a second) and keeps the ring a slot
/// short of full at full reheat, so a puff is never recycled while it is still
/// alive and being drawn.
const STEP: f32 = AB_REACH / (SAMPLES as f32 - 1.0);

/// How fast a puff travels aft down the plume (m/s, body frame). Together with
/// [`AB_REACH`] this sets how long a puff lasts, and hence how many are in the
/// air at once.
const AXIAL: f32 = 17.0;

/// Distance (m, body frame) a puff covers before it dies, by regime: a dry
/// nozzle leaves a short stub, the burner stretches it out. Deliberately short —
/// a reheat plume that hugs the tail reads as *power*; a long one reads as a
/// smoke trail.
const MIL_REACH: f32 = 1.0;
const AB_REACH: f32 = 6.0;

/// Puff radius (m) at the nozzle and at the tail, by regime. A reheat plume is
/// broadest right at the nozzle and tapers to a hard point — that taper is what
/// makes it read as a flame rather than as a cloud, and it keeps the bright
/// mass on the nozzle instead of trailing off it. A dry stub instead billows as
/// it dissipates.
const AB_R0: f32 = 0.44;
const AB_R1: f32 = 0.13;
const DRY_R0: f32 = 0.30;
const DRY_R1: f32 = 0.70;

/// How far (m) a puff's centre may be thrown off the axis, at the nozzle and at
/// the tail. Kept small so the plume stays a tight column; the scatter only
/// softens its edge, it does not fan it into a cone.
const SPREAD0: f32 = 0.06;
const SPREAD1: f32 = 0.26;

/// Peak opacity of a single puff, before its per-slot jitter. High enough that
/// a handful of overlapping puffs near the nozzle reach full opacity — that
/// thickness is what makes the core read as a solid hot column rather than as a
/// haze of separate discs.
const PUFF_GAIN: f32 = 0.50;

/// The flame's temperature ramp: cool gas glows dull red, hotter gas orange,
/// then yellow, and the hottest gas runs blue-white. Colour follows temperature
/// along the whole plume — the nozzle end is the hottest and the tail the
/// coolest — so one ramp covers both regimes and a hotter plume simply reaches
/// further up it.
///
/// The camera runs `Tonemapping::None`, so a channel at or above 1.0 clips flat;
/// only the hot end is allowed past 1.0 (on blue), which is what makes the core
/// bloom blue-white. Everything cooler stays under 1.0, or the plume washes out
/// to pale beige.
const COLD: Vec3 = Vec3::new(0.60, 0.075, 0.012);
const WARM: Vec3 = Vec3::new(1.70, 0.50, 0.10);
const HOT: Vec3 = Vec3::new(1.75, 1.25, 0.45);
/// Deliberately blue-dominant rather than near-white: with only blue past 1.0
/// the core clips to blue instead of washing out to a white blob.
const WHITE_HOT: Vec3 = Vec3::new(0.55, 0.85, 2.10);

/// Shock diamonds: the standing bright cells a reheat plume forms just aft of
/// the nozzle. They are pinned to distance, not carried downstream — the shocks
/// are held by the pressure field, so a puff brightens as it passes through one.
const DIAMOND_SPACING: f32 = 1.15;
const DIAMOND_GAIN: f32 = 1.1;
const DIAMOND_DECAY: f32 = 0.30;

/// Throttle command above which the FDM lights the burner — the same `0.99` the
/// engine's augmentation branch tests, so the visual cuts exactly when the
/// thrust does. (`fcs/thrust-norm` can't serve here: it is scaled by the
/// altitude/Mach thrust tables, so reheat reads ~1.4 at sea level but only ~0.7
/// at 30,000 ft, under dry power at sea level.)
const AB_THROTTLE: f64 = 0.99;
/// Seconds for the burner to reach full, so light-up ramps instead of popping.
const AB_RAMP: f32 = 0.12;
/// Seconds of flare after light-up: the plume punches long and bright, then
/// settles. The lurch reads as the nozzle opening.
const IGN_TIME: f32 = 0.30;
const IGN_FLARE: f32 = 0.65;
const IGN_STRETCH: f32 = 0.45;

/// These effects exist for the F-22 alone.
fn is_f22(sim: &Sim) -> bool {
    sim.name == "f22"
}

// --- afterburner plume -----------------------------------------------------

/// Uniform for `assets/shaders/flame_puff.wesl`. Two `vec4`s: a multiple of 16
/// bytes, as the uniform address space requires.
#[derive(Clone, Copy, ShaderType, Default)]
struct FlamePuffParams {
    /// rgb = puff colour (red may exceed 1 for bloom), a = peak intensity.
    color: Vec4,
    /// x = time (s), y = per-puff seed 0..1, z = rim raggedness 0..1,
    /// w = unused.
    anim: Vec4,
}

#[derive(Asset, TypePath, AsBindGroup, Clone)]
struct FlamePuffMaterial {
    #[uniform(0)]
    params: FlamePuffParams,
}

impl Material for FlamePuffMaterial {
    fn fragment_shader() -> ShaderRef {
        "shaders/flame_puff.wesl".into()
    }

    // Translucent, not additive. `AlphaMode::Add` does *not* mean `One/One` on a
    // custom material — it maps to Bevy's `PREMULTIPLIED_ALPHA_BLENDING`, which
    // is an ordinary "over". Having the puffs composite properly is what keeps
    // the plume readable: it is thirty-odd deep, and an additive stack runs off
    // the top of the range before the first puff is drawn.
    fn alpha_mode(&self) -> AlphaMode {
        AlphaMode::Blend
    }

    // A pure additive glow: nothing to write to a depth prepass, no shadows.
    fn enable_prepass() -> bool {
        false
    }

    fn enable_shadows() -> bool {
        false
    }
}

/// One billboard of a nozzle's plume, addressed by its slot in [`Plume`]. Not
/// parented to the aircraft: it is positioned in world space each frame from its
/// body-frame offset, which keeps the billboard maths in one frame.
#[derive(Component, Clone, Copy)]
struct FlamePuff {
    nozzle: usize,
    idx: usize,
}

/// Ring of puff slots, one ring per nozzle, plus the emission cursor that fills
/// them. A slot records how far down the plume its puff has travelled and the
/// power setting stamped on it as it left the nozzle — so a puff born under
/// reheat stays long and orange even after the burner cuts, and the plume then
/// dies out from the tail forward.
///
/// Emission is spaced by *distance*, like the wingtip vapor: one puff every
/// `STEP` metres of gas. Spacing it by time instead would bunch the puffs at low
/// speed and string them out at high speed, and the plume's density would rise
/// and fall with airspeed.
#[derive(Resource)]
struct Plume {
    /// Distance aft of the nozzle (body-frame metres), clamped at the puff's own
    /// reach — which is also how a dead puff is marked, since `f` then reads 1.
    dist: [[f32; SAMPLES]; NOZZLES],
    /// Burner level 0..1, as stamped at emission.
    level: [[f32; SAMPLES]; NOZZLES],
    /// Dry-thrust fraction 0..1, as stamped at emission.
    dry: [[f32; SAMPLES]; NOZZLES],
    /// Light-up stretch, as stamped at emission.
    flare: [[f32; SAMPLES]; NOZZLES],
    /// The slot the next puff goes into.
    cursor: [usize; NOZZLES],
    /// Metres of gas since the last emission.
    acc: [f32; NOZZLES],
}

impl Default for Plume {
    fn default() -> Self {
        // Start every slot "dead" — parked exactly at the reach of an unlit
        // nozzle — so nothing shows until the emission cursor has walked the
        // ring once.
        let dist = [[reach_of(0.0, 0.0, 0.0); SAMPLES]; NOZZLES];
        Plume {
            dist,
            level: [[0.0; SAMPLES]; NOZZLES],
            dry: [[0.0; SAMPLES]; NOZZLES],
            flare: [[0.0; SAMPLES]; NOZZLES],
            cursor: [0; NOZZLES],
            acc: [0.0; NOZZLES],
        }
    }
}

/// Afterburner latch state. The FDM lights the burner as a discrete jump, so
/// the visual is staged off it: `level` eases toward the gate's verdict and
/// `since_light` times the ignition flare.
#[derive(Resource, Default)]
struct Burner {
    /// 0 = dry, 1 = full reheat, eased so light-up and cut are visible.
    level: f32,
    lit: bool,
    /// Seconds since the burner last lit.
    since_light: f32,
}

// --- exhaust heat haze -----------------------------------------------------

/// Per-frame parameters for `assets/shaders/exhaust_haze.wesl`. Plain `f32`s
/// (like [`crate::lens::RainLens`]), four to a row, so the hand-written WGSL
/// struct matches the uniform layout byte for byte.
///
/// Nozzle screen positions and directions are in **aspect-corrected UV**: the
/// UV offset from the screen centre with x multiplied by the aspect ratio, so
/// the shader's lobes stay round on a wide window.
#[derive(Component, Clone, Copy, Default, ShaderType, ExtractComponent)]
#[extract_app(RenderApp)]
struct ExhaustHaze {
    uv0x: f32,
    uv0y: f32,
    dir0x: f32,
    dir0y: f32,
    str0: f32,
    rad0: f32,
    len0: f32,
    pad0: f32,
    uv1x: f32,
    uv1y: f32,
    dir1x: f32,
    dir1y: f32,
    str1: f32,
    rad1: f32,
    len1: f32,
    pad1: f32,
    time: f32,
    intensity: f32,
    /// Exhaust heat 0..1 (dry throttle blended with the burner), which the
    /// shader uses to speed the shimmer up as the plume gets hotter.
    temp: f32,
    pad3: f32,
}

impl FullscreenMaterial for ExhaustHaze {
    fn fragment_shader() -> ShaderRef {
        "shaders/exhaust_haze.wesl".into()
    }
}

// --- plugin ----------------------------------------------------------------

pub struct FxPlugin;

impl Plugin for FxPlugin {
    fn build(&self, app: &mut App) {
        app.add_plugins((
            MaterialPlugin::<FlamePuffMaterial>::default(),
            FullscreenMaterialPlugin::<ExhaustHaze>::default(),
        ))
        .init_resource::<Burner>()
        .init_resource::<Plume>()
        .add_systems(Update, spawn_fx.run_if(in_state(AppState::Flying)))
        // Reads the freshly-written airframe pose, so it must run after
        // `sync_aircraft` placed the aircraft this frame.
        .add_systems(
            Update,
            drive_flames
                .after(crate::view::sync_aircraft)
                .run_if(in_state(AppState::Flying)),
        )
        // The haze needs the final airframe/camera pose, so it runs after the
        // view systems have placed everything.
        .add_systems(
            PostUpdate,
            manage_haze.run_if(in_state(AppState::Flying)),
        );
    }
}

/// Spawn the F-22's plume entities once the flown aircraft exists. Idempotent:
/// it bails as soon as a puff is present, so it fires exactly once per flight.
///
/// The puffs are top-level (world-space) entities — `drive_flames` derives their
/// world position from the body-frame offset each frame.
fn spawn_fx(
    mut commands: Commands,
    sim: Res<Sim>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut mats: ResMut<Assets<FlamePuffMaterial>>,
    roots: Query<Entity, With<AircraftRoot>>,
    existing: Query<(), With<FlamePuff>>,
) {
    if !is_f22(&sim) || !existing.is_empty() || roots.single().is_err() {
        return;
    }

    // One unit quad per slot. `Rectangle` is 2×2, so the transform's scale is
    // the puff radius outright.
    let quad = meshes.add(Rectangle::new(2.0, 2.0));
    for nozzle in 0..NOZZLES {
        for idx in 0..SAMPLES {
            // A material per puff: each has to fade on its own clock.
            let mat = mats.add(FlamePuffMaterial {
                params: FlamePuffParams::default(),
            });
            commands.spawn((
                FlamePuff { nozzle, idx },
                Mesh3d(quad.clone()),
                MeshMaterial3d(mat),
                Transform::default(),
                Visibility::Hidden,
                DespawnOnExit(AppState::Flying),
            ));
        }
    }
}

/// How far (m, body frame) a puff travels before it dies, from the power setting
/// stamped when it left the nozzle: a dry nozzle leaves a short stub, the burner
/// stretches it out, and the overall scale follows `heat` so a low dry setting
/// reaches even less. The light-up flare stretches the plume for the puffs
/// emitted during it.
fn reach_of(level: f32, dry: f32, flare: f32) -> f32 {
    let heat = level.max(dry);
    (MIL_REACH + (AB_REACH - MIL_REACH) * level) * (0.30 + 0.70 * heat) * (1.0 + flare)
}

/// The flame colour at temperature `t` (0 = cold, 1 = white hot), read off the
/// ramp [`COLD`] → [`WARM`] → [`HOT`] → [`WHITE_HOT`]. Both the plume and the
/// ignition flare go through here, so a hotter flame is simply a further-along
/// flame and the light-up reads as the gas heating up.
fn flame_ramp(t: f32) -> Vec3 {
    let t = t.clamp(0.0, 1.0);
    if t < 0.45 {
        COLD.lerp(WARM, t / 0.45)
    } else if t < 0.78 {
        WARM.lerp(HOT, (t - 0.45) / 0.33)
    } else {
        HOT.lerp(WHITE_HOT, (t - 0.78) / 0.22)
    }
}

/// A puff's radius (m) at life fraction `f`, blending the dry billow and the
/// reheat taper by `heat` so the near-nozzle mass tracks the power setting.
fn puff_radius(heat: f32, f: f32) -> f32 {
    let dry = DRY_R0 + (DRY_R1 - DRY_R0) * f;
    let ab = AB_R0 + (AB_R1 - AB_R0) * f;
    dry.lerp(ab, heat)
}

/// Latch the burner off the throttle command, advance every puff down its
/// plume, and refresh each puff's transform, colour and intensity.
fn drive_flames(
    time: Res<Time>,
    sim: Res<Sim>,
    state: Res<AircraftState>,
    mut burner: ResMut<Burner>,
    mut plume: ResMut<Plume>,
    root: Query<&Transform, (With<AircraftRoot>, Without<FlamePuff>)>,
    cameras: Query<&Transform, (With<Camera3d>, Without<FlamePuff>)>,
    mut puffs: Query<(
        &FlamePuff,
        &MeshMaterial3d<FlamePuffMaterial>,
        &mut Transform,
        &mut Visibility,
    )>,
    mut mats: ResMut<Assets<FlamePuffMaterial>>,
) {
    if !is_f22(&sim) {
        return;
    }
    // Clamped: a puff walks `AXIAL · dt` metres down its plume each frame, so a
    // hitched frame would either fling it past the tail or, worse, pile every
    // puff into the last metre (the recycle then always lands in the same narrow
    // band and the plume collapses to a blob). Below this cap the plume is
    // stable; above it, the plume just runs slow for one frame.
    let dt = time.delta_secs().min(1.0 / 30.0);
    let t = time.elapsed_secs();

    // Latch on the same predicate the FDM's engine uses, and ease a level toward
    // it so the hard thrust step becomes a visible light-up; then time a flare
    // off the moment it latches.
    let ab = state.throttle > AB_THROTTLE;
    if ab && !burner.lit {
        burner.lit = true;
        burner.since_light = 0.0;
    } else if !ab {
        burner.lit = false;
    }
    if burner.lit {
        burner.since_light += dt;
    }
    let k = 1.0 - (-dt / AB_RAMP).exp();
    burner.level += (if ab { 1.0 } else { 0.0 } - burner.level) * k;

    // Dry thrust, for the military plume: idle leaves nothing, full mil a blue
    // stub. Ignition flare: 1 at light-up, decaying to 0 over `IGN_TIME`.
    let throttle = state.throttle.clamp(0.0, 1.0) as f32;
    let dry = ((throttle - 0.15) / 0.85).clamp(0.0, 1.0);
    let burst = if burner.lit {
        (1.0 - burner.since_light / IGN_TIME).clamp(0.0, 1.0)
    } else {
        0.0
    };
    // Bake the flare into what a puff *records*, so the puffs emitted during the
    // light-up are the long bright ones and the plume settles as they die out.
    let level = (burner.level * (1.0 + IGN_FLARE * burst)).min(1.0);
    let flare = IGN_STRETCH * burst;

    // Advance every live puff down its plume. A puff that has reached its reach
    // is parked there — `f` then reads 1, the shader gets no opacity, and the
    // hidden test below skips it without needing a separate flag.
    for n in 0..NOZZLES {
        for i in 0..SAMPLES {
            let reach = reach_of(plume.level[n][i], plume.dry[n][i], plume.flare[n][i]);
            let d = plume.dist[n][i];
            if d < reach {
                plume.dist[n][i] = (d + AXIAL * dt).min(reach);
            }
        }
    }

    // Emit into the ring as gas passes, stamping each new puff with the power
    // setting of the moment — that is what lets a plume outlive the burner.
    // Bounded so a hitched frame cannot spin here.
    for n in 0..NOZZLES {
        plume.acc[n] += AXIAL * dt;
        let mut guard = 0;
        while plume.acc[n] >= STEP && guard < SAMPLES {
            plume.acc[n] -= STEP;
            let c = plume.cursor[n];
            plume.dist[n][c] = 0.0;
            plume.level[n][c] = level;
            plume.dry[n][c] = dry;
            plume.flare[n][c] = flare;
            plume.cursor[n] = (c + 1) % SAMPLES;
            guard += 1;
        }
    }

    let Ok(root_tf) = root.single() else {
        return;
    };
    let nozzles = [
        model_to_body(&sim.name, F22_NOZZLE_L),
        model_to_body(&sim.name, F22_NOZZLE_R),
    ];
    let cam = cameras.single().ok().map(|t| t.translation);

    for (puff, mat, mut tf, mut vis) in &mut puffs {
        let (n, i) = (puff.nozzle, puff.idx);
        let (lvl, dry_v, flare_v, d) =
            (plume.level[n][i], plume.dry[n][i], plume.flare[n][i], plume.dist[n][i]);
        let heat = lvl.max(dry_v);
        // How far through its life the puff is. Every scale below keys off this,
        // so the plume's colour, size and spread all track its length.
        let f = (d / reach_of(lvl, dry_v, flare_v)).clamp(0.0, 1.0);

        // Deterministic per-slot jitter, so the plume is never a row of identical
        // puffs: independent hashes for size, brightness and scatter direction.
        // Kept narrow — wide jitter is what made the plume boil.
        let h = n as f32 * 0.37 + i as f32 * 0.618_034;
        let size_j = 0.82 + 0.36 * (h * 1.3).fract();
        let alpha_j = 0.82 + 0.30 * (h * 2.7).fract();

        // Body-frame placement. The AircraftRoot frame has the nose on -Z, so +Z
        // is aft and a puff's downstream offset is a plain +Z translation. The
        // scatter shrinks with `heat` so a hot plume stays a tight column.
        let spread = (SPREAD0 + (SPREAD1 - SPREAD0) * f) * (1.0 - 0.5 * heat);
        let dir = Quat::from_rotation_z((h * 3.1).fract() * std::f32::consts::TAU) * Vec3::X;
        let local = nozzles[n] + Vec3::Z * d + dir * spread * size_j;
        let world = root_tf.translation + root_tf.rotation * local;
        tf.translation = world;
        tf.scale = Vec3::splat(puff_radius(heat, f) * size_j);
        // Billboard about the quad's normal (+Z) so the puff always faces the
        // viewer.
        if let Some(cam) = cam {
            let to_cam = cam - world;
            if to_cam.length_squared() > 1e-6 {
                tf.rotation = Quat::from_rotation_arc(Vec3::Z, to_cam.normalize());
            }
        }

        // Colour, read off the temperature ramp. The gas is hottest as it leaves
        // the nozzle and cools down the plume, so a puff's colour is its power
        // setting's temperature faded by how far it has travelled: the reheat
        // core runs blue-white, through orange, to a dull red tail, and a cool
        // dry stub only ever reaches the warm end. Only the hot end rides past
        // 1.0, which is what lets the bloom catch the core.
        let temp = heat * (1.0 - 0.85 * f);
        let mut col = flame_ramp(temp);

        // Shock diamonds, gaussian so they have no hard edge. Anchored to
        // distance rather than to the puff, so the train stands still in the
        // nozzle's frame while the puffs march through it. A cell is brighter
        // because it is *hotter*, so it climbs the same ramp rather than being
        // scaled channel by channel — lifting green alone on an orange cell
        // makes it green, not yellow.
        let ph = d / DIAMOND_SPACING;
        let dd = (ph - ph.round()).abs();
        let diamond = lvl * (-dd * dd * 14.0).exp() * (-d * DIAMOND_DECAY).exp();
        col = col.lerp(flame_ramp(temp + 0.30), 0.45 * diamond);

        let peak =
            PUFF_GAIN * heat * alpha_j * (1.0 - f).powf(1.4) * (1.0 + DIAMOND_GAIN * diamond);
        if peak < 0.004 {
            if *vis != Visibility::Hidden {
                *vis = Visibility::Hidden;
            }
            continue;
        }
        *vis = Visibility::Inherited;
        if let Some(mut m) = mats.get_mut(&mat.0) {
            m.params = FlamePuffParams {
                color: Vec4::new(col.x, col.y, col.z, peak),
                // The rim frays as the puff travels: `f` is the raggedness, so
                // the plume leaves the nozzle as a clean round column and ends
                // as a torn cloud.
                anim: Vec4::new(t, (h * 5.7).fract(), f, 0.0),
            };
        }
    }
}

/// Refresh the camera's exhaust-haze uniform from the live nozzle screen
/// positions, attaching/detaching the pass with the throttle so a dead engine
/// costs nothing.
fn manage_haze(
    mut commands: Commands,
    time: Res<Time>,
    sim: Res<Sim>,
    state: Res<AircraftState>,
    burner: Res<Burner>,
    root: Query<&Transform, With<AircraftRoot>>,
    mut cameras: Query<(Entity, &Camera, &GlobalTransform, Option<&ExhaustHaze>), With<Camera3d>>,
) {
    let throttle = state.throttle.clamp(0.0, 1.0) as f32;
    // Exhaust heat, the same blend the plume colours itself from: dry throttle,
    // lifted by the burner. The haze rides on it so light-up shows as a jump in
    // the shimmer, not just in the flame.
    let dry = ((throttle - 0.15) / 0.85).clamp(0.0, 1.0);
    let temp = burner.level.max(dry);
    let want = is_f22(&sim) && throttle > 0.02;
    let root_tf = root.single().ok();
    let name = sim.name.as_str();

    for (e, cam, cam_gt, existing) in &mut cameras {
        // Drop the pass (no per-frame cost) when nothing is running or seen.
        let Some(root_tf) = root_tf.filter(|_| want) else {
            if existing.is_some() {
                commands.entity(e).remove::<ExhaustHaze>();
            }
            continue;
        };
        let Some(rect) = cam.logical_viewport_rect() else {
            continue;
        };
        let size = rect.size();
        if size.x < 1.0 || size.y < 1.0 {
            continue;
        }
        let aspect = size.x / size.y;

        // A nozzle's screen UV plus the on-screen direction its exhaust trails
        // toward, both in aspect-corrected UV. `None` if it is behind the camera.
        let nozzle = |model: Vec3| -> Option<(Vec2, Vec2)> {
            let world = root_tf.translation
                + root_tf.rotation * (root_tf.scale * model_to_body(name, model));
            let p = cam.world_to_viewport(cam_gt, world).ok()?;
            // The plume runs along the body's aft axis (+Z); sample a point a
            // few metres downstream to get its screen direction.
            let aft = world + root_tf.rotation * (root_tf.scale * Vec3::Z) * 4.0;
            let p2 = cam.world_to_viewport(cam_gt, aft).ok()?;
            let to_uv = |v: Vec2| ((v - rect.min) / size - Vec2::splat(0.5)) * Vec2::new(aspect, 1.0);
            let a = to_uv(p);
            let b = to_uv(p2);
            // Looking straight down the exhaust the two points nearly coincide;
            // fall back to "heat rises" so there is still a shimmer.
            let off = b - a;
            let dir = if off.length() < 1e-4 {
                Vec2::NEG_Y
            } else {
                off.normalize()
            };
            Some((a, dir))
        };
        let (Some((uv0, dir0)), Some((uv1, dir1))) = (nozzle(F22_NOZZLE_L), nozzle(F22_NOZZLE_R))
        else {
            if existing.is_some() {
                commands.entity(e).remove::<ExhaustHaze>();
            }
            continue;
        };

        // The refraction grows with the exhaust heat, and the lobes stretch a
        // little as it does — a hot plume distorts a longer slipstream.
        let intensity = 0.25 + 0.75 * temp;
        // A subtle shimmer: the offset is a small fraction of the screen, not a
        // full warp. `rad`/`len` are the lobe's half-width and length in
        // aspect-corrected UV.
        let str = 0.011 * intensity;
        let len = 0.070 + 0.070 * temp;
        commands.entity(e).insert(ExhaustHaze {
            uv0x: uv0.x,
            uv0y: uv0.y,
            dir0x: dir0.x,
            dir0y: dir0.y,
            str0: str,
            rad0: 0.035,
            len0: len,
            pad0: 0.0,
            uv1x: uv1.x,
            uv1y: uv1.y,
            dir1x: dir1.x,
            dir1y: dir1.y,
            str1: str,
            rad1: 0.035,
            len1: len,
            pad1: 0.0,
            time: time.elapsed_secs(),
            intensity,
            temp,
            pad3: 0.0,
        });
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn reach_grows_from_a_dry_stub_to_the_reheat_plume() {
        // Idle: nothing lit, nothing to reach.
        assert!(reach_of(0.0, 0.0, 0.0) < 0.5);
        // Full reheat reaches the whole plume.
        assert!((reach_of(1.0, 0.0, 0.0) - AB_REACH).abs() < 1e-4);
        // Full military power is a short stub, well inside the reheat plume;
        // part throttle reaches less again.
        assert!((reach_of(0.0, 1.0, 0.0) - MIL_REACH).abs() < 1e-4);
        let half = reach_of(0.0, 0.5, 0.0);
        assert!(half > 0.0 && half < MIL_REACH, "got {half}");
        // The light-up flare stretches it past nominal.
        assert!(reach_of(1.0, 0.0, IGN_STRETCH) > AB_REACH);
    }

    #[test]
    fn flame_ramp_runs_from_red_up_to_blue_white() {
        // The cool end is red-dominant, the hot end blue-dominant — the whole
        // point of the temperature ramp.
        let cold = flame_ramp(0.0);
        assert!(cold.x > cold.z * 5.0, "cold end should be red, got {cold}");
        let hot = flame_ramp(1.0);
        assert!(hot.z > hot.x, "hot end should be blue, got {hot}");
        // And green stays a middle channel throughout, so no leg of the ramp
        // reads as a pure primary.
        for i in 0..=10 {
            let c = flame_ramp(i as f32 / 10.0);
            assert!(c.y <= c.x.max(c.z) + 1e-4, "green ran hot at {i}: {c}");
        }
    }

    #[test]
    fn reheat_puffs_taper_while_dry_ones_billow() {
        // The reheat plume is broadest at the nozzle and tapers aft; the dry one
        // grows.
        assert!(puff_radius(1.0, 0.0) > puff_radius(1.0, 1.0));
        assert!(puff_radius(0.0, 1.0) > puff_radius(0.0, 0.0));
    }
}
