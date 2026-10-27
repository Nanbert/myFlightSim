//! F-22 condensation vapor: the white mist that blooms over a fighter's wing and
//! body in a hard manoeuvre, and the cone that appears as it pushes through the
//! sound barrier.
//!
//! Both effects are gated to the F-22 (`sim.name == "f22"`), like the engine
//! effects in `fx.rs`: only that model ships the measured wing-tip anchors, and
//! only its FDM publishes the two signals that drive them — angle of attack
//! (`aero/alpha-rad`, both backends) and Mach (`velocities/mach`). Neither is
//! smoke: the air's own water vapor condensing where the flow drops in pressure
//! and temperature, so both track the *flow*, not the engine.
//!
//! - **Vortex trails.** Condensation shows wherever the flow drops in pressure
//!   over a lifting surface, so the jet wears it in several places at once: the
//!   wingtip vortices, the leading edge along the span, and the forebody/chine
//!   over the centerline — the last two being a fighter's strongest vortex
//!   sources at high angle of attack (the [`EMITTERS`] table). Every emitter is
//!   driven by its own *local* angle of attack: pull and they feed, unload and
//!   they stop and dissipate. Roll rate biases each emitter by its side, since
//!   the wing rolling *down* meets the flow at a higher effective angle — so a
//!   roll lights the leading edge on one side, the way it reads in footage. Each
//!   emitter drops soft billboard puffs into **world** space, recycled from a
//!   fixed ring, so the aircraft flies out of them and a hard turn draws long
//!   curved ribbons for free. Samples are spaced by *distance* along the
//!   emitter's path, so a ribbon stays continuous whether the jet is ambling at
//!   200 kt or crossing the barrier.
//! - **Transonic cone.** Near Mach 1 the shock condenses the air into a
//!   translucent cone wrapping the airframe: one cone shell, a child of the
//!   airframe, whose opacity peaks in a band around Mach 1.
//!
//! The plugin is self-contained: it spawns its entities once `AircraftRoot`
//! exists (see [`spawn_vapor`]) rather than hooking the flight spawn, so
//! examples that build their own scene (and never add it) are unaffected. Every
//! entity carries `DespawnOnExit`, so a fresh flight starts clean.

use bevy::asset::RenderAssetUsages;
use bevy::audio::{AudioPlayer, AudioSource, PlaybackSettings, Volume};
use bevy::mesh::{Indices, PrimitiveTopology};
use bevy::prelude::*;
use bevy::render::mesh::ConeAnchor;
use bevy::render::render_resource::{AsBindGroup, ShaderType};
use bevy::shader::ShaderRef;
use bevy::transform::TransformSystems;

use crate::app_state::{unpaused, AppState};
use crate::sim::{AircraftState, Sim};
use crate::ui_theme::blend_material;
use crate::view::{model_to_body, AircraftRoot};
use crate::weather::Weather;

/// One condensation-vapor emitter, anchored in the F-22's own glTF frame
/// (metres, +X left / +Y up / +Z nose). Every emitter runs the same trail
/// machinery — only where it sits and how its feed is shaped differ, since the
/// mist tracks the local *flow*, not the emitter.
struct Emitter {
    /// Anchor on the airframe, at the surface the vortex peels off.
    model: Vec3,
    /// Roll sensitivity: `+1` on the wing a *positive (right) roll* drives
    /// down — the model's −x / right side — and `−1` on the other, so a roll
    /// raises the local angle of attack on the descending wing alone. `0` for a
    /// symmetric emitter, which the roll does not bias.
    roll_side: f32,
    /// Local angle-of-attack window (deg) the vapor ramps in over. Emitters
    /// nearer a vortex origin (the LEX, the forebody) fog sooner, so their
    /// window opens lower.
    aoa0: f32,
    aoa1: f32,
    /// Degrees of effective angle of attack added per deg/s of roll / yaw, at
    /// [`RATE_CAL_KT`]. Roll carries `roll_side`; yaw is symmetric (a yaw feeds
    /// both chines). This is what lets a fast roll light the leading edge at an
    /// angle of attack too low for the wingtips to show.
    roll_gain: f32,
    yaw_gain: f32,
    /// Share of the transonic pickup this emitter takes. Only the tips take any:
    /// near Mach 1 the shock condenses air off them, which is a *different*
    /// effect from the vortex vapor, and letting every emitter draw on the band
    /// would pile all seven into one blob over the wing.
    mach_gain: f32,
    /// Whether this emitter drops a world-space trail. Only the wingtips do;
    /// the wing-surface vapor is an attached sheet instead of a long wake.
    trail: bool,
}

/// The F-22's vortex sources. The two wingtips are the anchors the model was
/// measured around (span ±1.01 model units, tip chord at z ≈ −0.72); the rest
/// were read off the wing and fuselage meshes — the leading edge runs from the
/// tip back to the root, and the forebody/chine sits over the centerline ahead
/// of the canopy. All distances are model units.
const EMITTERS: &[Emitter] = &[
    // Wingtips (left +x, right −x; the roll only biases the ones that move).
    Emitter {
        model: Vec3::new(1.00, 0.14, -0.72),
        roll_side: -1.0,
        aoa0: 5.0,
        aoa1: 16.0,
        roll_gain: 0.000,
        yaw_gain: 0.00,
        mach_gain: 1.0,
        trail: true,
    },
    Emitter {
        model: Vec3::new(-1.00, 0.14, -0.72),
        roll_side: 1.0,
        aoa0: 5.0,
        aoa1: 16.0,
        roll_gain: 0.000,
        yaw_gain: 0.00,
        mach_gain: 1.0,
        trail: true,
    },
    // Leading edge, mid-span (the tip-to-root sweep line).
    Emitter {
        model: Vec3::new(0.66, 0.08, -0.31),
        roll_side: -1.0,
        aoa0: 6.0,
        aoa1: 14.0,
        roll_gain: 0.035,
        yaw_gain: 0.02,
        mach_gain: 0.0,
        trail: false,
    },
    Emitter {
        model: Vec3::new(-0.66, 0.08, -0.31),
        roll_side: 1.0,
        aoa0: 6.0,
        aoa1: 14.0,
        roll_gain: 0.035,
        yaw_gain: 0.02,
        mach_gain: 0.0,
        trail: false,
    },
    // Wing-root leading edge / LEX, where the vortex is strongest.
    Emitter {
        model: Vec3::new(0.34, 0.08, 0.06),
        roll_side: -1.0,
        aoa0: 5.0,
        aoa1: 13.0,
        roll_gain: 0.026,
        yaw_gain: 0.03,
        mach_gain: 0.0,
        trail: false,
    },
    Emitter {
        model: Vec3::new(-0.34, 0.08, 0.06),
        roll_side: 1.0,
        aoa0: 5.0,
        aoa1: 13.0,
        roll_gain: 0.026,
        yaw_gain: 0.03,
        mach_gain: 0.0,
        trail: false,
    },
    // Forebody / chine, over the centerline ahead of the canopy. On the roll axis,
    // so the roll does not bias it — only the yaw does.
    Emitter {
        model: Vec3::new(0.00, 0.22, 0.34),
        roll_side: 0.0,
        aoa0: 7.0,
        aoa1: 20.0,
        roll_gain: 0.000,
        yaw_gain: 0.05,
        mach_gain: 0.0,
        trail: false,
    },
];

/// One trail ring per emitter.
const EMITTER_COUNT: usize = EMITTERS.len();

/// History samples per trail; the ring holds `SAMPLES · STEP` metres of trail.
/// Kept short: the wingtip vortex is a brief ribbon, not a smoke wake.
const SAMPLES: usize = 36;

/// Distance (m) between trail samples along a wingtip's path. Emission is
/// spaced in *distance*, not time: a time-spaced ring would put the samples
/// 5 m apart at Mach 1 and string the trail out into beads, where at 200 kt the
/// same ring merges into a ribbon. Distance spacing holds at any speed.
const STEP: f32 = 1.2;

/// How long a wingtip ribbon lives (s), on its own clock. Above about 160 kt the
/// ring is the limit, so this only binds when the jet is slow: same trail, shorter.
const LIFE: f32 = 0.6;

/// Puff radius (m) at birth and at death; a puff expands as it dissipates. Born
/// wider than half the sample spacing so consecutive puffs merge into a ribbon
/// rather than reading as a row of beads.
const PUFF_R0: f32 = 1.4;
const PUFF_R1: f32 = 2.8;
/// Peak opacity of a fresh puff on a fully-fed trail. Kept low because a dense
/// stream of them accumulates; dialled down from the two-emitter version now
/// that a hard pull lights several streams at once.
const PUFF_ALPHA: f32 = 0.045;
/// How far (m) a puff's centre may wander off the sampled path. Breaks the
/// ribbon up so it reads as vapor rather than a smooth tube, without pulling
/// neighbouring puffs far enough apart to come unstitched.
const PUFF_WANDER: f32 = 0.4;

/// Speed (m/s) below which the vortex trails are suppressed — on rotation the
/// wing is loaded but the aircraft is slow, and a stationary puff pile looks
/// wrong. Fades in over `SPEED0..SPEED1`.
const SPEED0: f32 = 25.0;
const SPEED1: f32 = 60.0;
/// Mach near which the transonic cone is full, and the half-width of its band.
const MACH_C: f32 = 1.02;
const MACH_W: f32 = 0.20;
/// Airspeed (kt) the emitters' rate gains are calibrated at. The manoeuvre bias
/// scales as `RATE_CAL_KT / airspeed`, since the same roll rate swings the local
/// angle of attack further the more slowly the air is moving past — which is why
/// a fighter wears the vapor low and slow. `RATE_MIN_KT` floors the divisor so a
/// slow roll can't blow the bias up.
const RATE_CAL_KT: f32 = 320.0;
const RATE_MIN_KT: f32 = 140.0;

/// Wing-surface burst cloud and its pop sound only light up above this CAS
/// (kt). Kept deliberately high: a fighter's surface condensation fog is a
/// high-speed, high-load phenomenon, so the burst pops rarely rather than on
/// every pull. The wingtip trails are *not* gated by this — see `drive_vapor`.
const BURST_SPEED_KT: f32 = 250.0;
/// Minimum gap between two wing-surface burst pops (s). A hard pull can spike
/// the load every few frames, so without this the one-shot stutters.
const VAPOR_COOLDOWN_S: f32 = 0.8;
/// Fixed level for the vapor pop; deliberately not ducked under the radio.
const VAPOR_VOLUME: f32 = 0.6;

/// Transonic vapor cone, in the model frame: the apex sits just behind the
/// radome, opening aft past the wing root. The F-22's glTF runs z ∈ [-1.34,
/// +1.29] (nose at +1.29), so the shell spans the forward fuselage and the wing
/// root — where the shock that condenses it actually sits — and stays close to
/// the body: `CONE_RADIUS` is a little over the fuselage half-width, not the
/// span.
const CONE_NOSE: Vec3 = Vec3::new(0.0, 0.16, 1.18);
const CONE_AFT: Vec3 = Vec3::new(0.0, 0.16, -0.60);
const CONE_RADIUS: f32 = 0.30;

pub struct VaporPlugin;

impl Plugin for VaporPlugin {
    fn build(&self, app: &mut App) {
        app.add_plugins((
            MaterialPlugin::<PuffMaterial>::default(),
            MaterialPlugin::<ConeMaterial>::default(),
            MaterialPlugin::<SheetMaterial>::default(),
            MaterialPlugin::<VolumeMaterial>::default(),
            MaterialPlugin::<BoxMaterial>::default(),
            MaterialPlugin::<ShadowMaterial>::default(),
        ))
        .init_resource::<Trails>()
        .init_resource::<SheetState>()
        .init_resource::<WingPuffs>()
        .add_systems(Startup, setup_vapor_sfx)
        .add_systems(OnEnter(AppState::Flying), reset_vapor_state)
        .add_systems(Update, spawn_vapor.run_if(unpaused))
        // Reads the freshly-written airframe pose, so it must run after
        // `sync_aircraft` placed the aircraft this frame.
        .add_systems(
            Update,
            drive_vapor
                .after(crate::view::sync_aircraft)
                .run_if(unpaused),
        )
        .add_systems(
            Update,
            play_vapor_sfx
                .after(drive_vapor)
                .run_if(unpaused),
        )
        .add_systems(
            Update,
            drive_sheet
                .after(drive_vapor)
                .run_if(unpaused),
        )
        .add_systems(
            PostUpdate,
            drive_box
                .after(TransformSystems::Propagate)
                .run_if(unpaused),
        )
        // Warmup runs after every visibility writer (the drive systems live in
        // Update, `drive_box` above), so one forced-visible frame wins.
        .add_systems(
            PostUpdate,
            warm_pipelines
                .after(drive_box)
                .after(TransformSystems::Propagate)
                .run_if(unpaused),
        )
        .add_systems(
            Update,
            drive_shadow
                .after(drive_vapor)
                .run_if(unpaused),
        );
    }
}

/// Clear the world-space rings and the sheet envelope when a flight starts, so
/// a new aircraft never inherits the previous one's trail or burst level.
fn reset_vapor_state(
    mut trails: ResMut<Trails>,
    mut sheet: ResMut<SheetState>,
    mut sfx: ResMut<VaporSfx>,
) {
    *trails = Trails::default();
    *sheet = SheetState::default();
    sfx.cooldown = 0.0;
}

/// One-shot wing-surface vapor pop, hard-panned per wing: index 0 (left) is
/// `vapor_pop_L`, index 1 (right) `vapor_pop_R`. `cooldown` gates retriggers
/// so a hard pull doesn't machine-gun the pop into a stutter.
#[derive(Resource)]
struct VaporSfx {
    left: Handle<AudioSource>,
    right: Handle<AudioSource>,
    cooldown: f32,
}

/// Load the two panned pops once; `AssetServer` keeps them hot all session.
fn setup_vapor_sfx(mut commands: Commands, assets: Res<AssetServer>) {
    commands.insert_resource(VaporSfx {
        left: assets.load("audio/f22/vapor_pop_L.wav"),
        right: assets.load("audio/f22/vapor_pop_R.wav"),
        cooldown: 0.0,
    });
}

/// Consume the per-side burst requests `drive_vapor` leaves in
/// [`SheetState::pending_burst`] and play the matching panned pop, respecting
/// the cooldown. Side 0 is the +x / left wing, side 1 the −x / right wing; both
/// firing on the same frame sound together (left + right). Runs after
/// `drive_vapor` so the flags are read the frame they are written.
fn play_vapor_sfx(
    mut commands: Commands,
    time: Res<Time>,
    mut sfx: ResMut<VaporSfx>,
    mut sheet: ResMut<SheetState>,
) {
    let dt = time.delta_secs();
    sfx.cooldown = (sfx.cooldown - dt).max(0.0);
    let fired = sheet.pending_burst;
    sheet.pending_burst = [false; 2];
    if (fired[0] || fired[1]) && sfx.cooldown <= 0.0 {
        if fired[0] {
            commands.spawn((
                AudioPlayer::new(sfx.left.clone()),
                PlaybackSettings::DESPAWN.with_volume(Volume::Linear(VAPOR_VOLUME)),
                DespawnOnExit(AppState::Flying),
            ));
        }
        if fired[1] {
            commands.spawn((
                AudioPlayer::new(sfx.right.clone()),
                PlaybackSettings::DESPAWN.with_volume(Volume::Linear(VAPOR_VOLUME)),
                DespawnOnExit(AppState::Flying),
            ));
        }
        sfx.cooldown = VAPOR_COOLDOWN_S;
    }
}

// --- wingtip trail puffs ---------------------------------------------------

/// Uniform for `assets/shaders/vapor_puff.wesl`.
#[derive(Clone, Copy, ShaderType, Default)]
struct PuffParams {
    /// rgb = vapor colour, a = opacity.
    color: Vec4,
    /// x = soft-edge threshold (0..1) of the round mask; y/z/w unused.
    anim: Vec4,
}

// Vapor is translucent, not emissive: it dims the sky behind it rather than
// adding to it. Each puff owns its material so it can fade on its own.
#[derive(Asset, TypePath, AsBindGroup, Clone)]
struct PuffMaterial {
    #[uniform(0)]
    params: PuffParams,
}

blend_material!(PuffMaterial, "shaders/vapor_puff.wesl", crate::ui_theme::SORT_TOP_BIAS);

/// One quad of a wingtip trail, addressed by its slot in [`Trails`]. Not
/// parented to the aircraft: the puffs hang in world space once dropped.
#[derive(Component)]
struct VaporPuff {
    side: usize,
    idx: usize,
}

/// Ring of history samples, one per emitter. A slot holds where the emitter was,
/// how long ago, and how strong the vapor feed was when it was dropped.
#[derive(Resource)]
struct Trails {
    pos: [[Vec3; SAMPLES]; EMITTER_COUNT],
    age: [[f32; SAMPLES]; EMITTER_COUNT],
    feed: [[f32; SAMPLES]; EMITTER_COUNT],
    cursor: [usize; EMITTER_COUNT],
    /// Distance the tip has covered since its last sample, per side.
    acc: [f32; EMITTER_COUNT],
    /// Where the tip was on the previous frame, per side.
    last: [Vec3; EMITTER_COUNT],
    /// Whether the tip has been sighted yet; the first sample is dropped as soon
    /// as it is, so the trail starts under the wing rather than behind it.
    live: [bool; EMITTER_COUNT],
}

impl Default for Trails {
    fn default() -> Self {
        Trails {
            pos: [[Vec3::ZERO; SAMPLES]; EMITTER_COUNT],
            // Start every slot "old" so an untouched one stays hidden.
            age: [[f32::MAX; SAMPLES]; EMITTER_COUNT],
            feed: [[0.0; SAMPLES]; EMITTER_COUNT],
            cursor: [0; EMITTER_COUNT],
            acc: [0.0; EMITTER_COUNT],
            last: [Vec3::ZERO; EMITTER_COUNT],
            live: [false; EMITTER_COUNT],
        }
    }
}

// --- transonic vapor cone --------------------------------------------------

/// Uniform for `assets/shaders/vapor_cone.wesl`.
#[derive(Clone, Copy, ShaderType, Default)]
struct ConeParams {
    /// x = intensity (0..1), y = time (s), z/w unused.
    anim: Vec4,
    /// rgb = vapor colour, a unused.
    tint: Vec4,
}

#[derive(Asset, TypePath, AsBindGroup, Clone)]
struct ConeMaterial {
    #[uniform(0)]
    params: ConeParams,
}

blend_material!(ConeMaterial, "shaders/vapor_cone.wesl", crate::ui_theme::SORT_TOP_BIAS);

#[derive(Component)]
struct VaporCone;

// --- wing surface sheet ----------------------------------------------------

/// Approximate F-22 wing planform, in the model frame: root LE, tip LE, tip TE,
/// root TE. The sheet sits a little above the upper surface; the shader fades
/// its root/tip and leading/trailing edges, so the exact outline is forgiving.
const WING_SHEET: [(f32, f32, f32); 4] = [
    (0.28, 0.14, 0.06),
    (1.00, 0.18, -0.72),
    (0.98, 0.16, -1.02),
    (0.28, 0.13, -0.60),
];

/// Three stacked sheets per wing: a dense base, a billowing middle and a thin
/// torn top. The vertical offsets give real parallax from the chase camera, so
/// the effect reads as a volume instead of a decal on the wing.
const SHEET_LAYERS: usize = 1;
const SHEET_LAYER_OFFSET: [f32; SHEET_LAYERS] = [0.12];
/// Spanwise/chordwise subdivisions of each sheet. A flat quad reads as frost;
/// a gently billowed grid gives the silhouette parallax and thickness.
const SPAN_SEGS: usize = 6;
const CHORD_SEGS: usize = 6;
/// Static billow amplitude per layer, metres. The edges are windowed to zero so
/// the sheet still starts and ends on the wing.
const SHEET_LAYER_WAVE: [f32; SHEET_LAYERS] = [0.12];

/// The visible surface vapor is a short-lived volume cloud of camera-facing
/// puffs distributed through the wing's boundary layer. The sheet remains as a
/// faint base (disabled here); the puffs are what give it thickness.
const WING_PUFF_COUNT: usize = 384;
const WING_PUFFS_ENABLED: bool = false;
const WING_PUFFS_PER_WING: usize = WING_PUFF_COUNT / 2;
const WING_PUFF_ALPHA: f32 = 0.20;
const WING_PUFF_LIFE_MIN: f32 = 0.18;
const WING_PUFF_LIFE_MAX: f32 = 0.34;
/// Set false to drop the flat sheet entirely; the volume puffs carry the look.
const SHEET_ENABLED: bool = false;
const BOX_VOLUME_ENABLED: bool = true;

/// Uniform for `assets/shaders/vapor_sheet.wesl`.
#[derive(Clone, Copy, ShaderType, Default)]
struct SheetParams {
    /// x = intensity (0..1), y = time (s), z = side, w = layer.
    anim: Vec4,
    /// rgb = vapor colour.
    tint: Vec4,
    /// xyz = direction toward the sun.
    light: Vec4,
}

#[derive(Asset, TypePath, AsBindGroup, Clone)]
struct SheetMaterial {
    #[uniform(0)]
    params: SheetParams,
}

blend_material!(SheetMaterial, "shaders/vapor_sheet.wesl", crate::ui_theme::SORT_TOP_BIAS);

/// One attached sheet per wing. The mesh is authored in the body frame and
/// parented to `AircraftRoot`, so the vapor stays on the wing instead of being
/// left behind as a world-space wake.
#[derive(Component)]
struct VaporSheet {
    side: usize,
    layer: usize,
}

/// Fast-attack / short-release envelope for the surface sheet, plus a one-shot
/// pulse when the load factor spikes.
#[derive(Resource, Default)]
struct SheetState {
    /// Envelope density per side, kept for the disabled sheet path.
    intensity: [f32; 2],
    /// Seconds since the burst started, per side.
    age: [f32; 2],
    /// Trigger strength (0..1), per side.
    strength: [f32; 2],
    /// Tip trails stay off until the mass has had a head start.
    trail_unlock: [bool; 2],
    /// Speed-driven footprint coverage (0.25..1), per side.
    coverage: [f32; 2],
    /// +1 upper surface, -1 lower surface, per side.
    surface: [f32; 2],
    /// Set when a burst fires this frame, consumed by [`play_vapor_sfx`] so the
    /// pop plays once per event instead of once per frame.
    pending_burst: [bool; 2],
    prev_g: f32,
    prev_local_alpha: [f32; 2],
}

fn smoothstep01(a: f32, b: f32, x: f32) -> f32 {
    let t = ((x - a) / (b - a)).clamp(0.0, 1.0);
    t * t * (3.0 - 2.0 * t)
}

/// The 0.45 s wing-mass life: snap open, hold, then shrink back into the wing.
/// Returns (density, scale); both are zero when idle.
fn burst_envelope(age: f32) -> (f32, f32) {
    const LIFE: f32 = 0.45;
    let attack = smoothstep01(0.0, 0.05, age);
    let scale = attack * (1.0 - smoothstep01(0.16, LIFE, age));
    let density = attack * (1.0 - smoothstep01(0.18, LIFE, age));
    (density, scale)
}

// --- wing volume puffs -----------------------------------------------------

/// Uniform for `assets/shaders/vapor_volume.wesl`.
#[derive(Clone, Copy, ShaderType, Default)]
struct VolumeParams {
    /// rgb = vapour colour, a = opacity.
    color: Vec4,
    /// x = seed, y = time (s), zw unused.
    anim: Vec4,
    /// xyz = direction toward the sun.
    light: Vec4,
}

#[derive(Asset, TypePath, AsBindGroup, Clone)]
struct VolumeMaterial {
    #[uniform(0)]
    params: VolumeParams,
}

blend_material!(VolumeMaterial, "shaders/vapor_volume.wesl", crate::ui_theme::SORT_TOP_BIAS);

/// Uniform for `assets/shaders/vapor_box.wesl`.
#[derive(Clone, Copy, ShaderType, Default)]
struct BoxParams {
    /// rgb = vapour colour, a unused.
    color: Vec4,
    /// x = intensity, y = time (s), z = side, w unused.
    anim: Vec4,
    /// xyz = direction toward the sun, w = surface sign.
    light: Vec4,
    /// xyz = body/local drift, w = age fraction.
    drift: Vec4,
    /// World -> box-local (unit cube) transform.
    world_to_local: Mat4,
}

#[derive(Asset, TypePath, AsBindGroup, Clone)]
struct BoxMaterial {
    #[uniform(0)]
    params: BoxParams,
}

blend_material!(BoxMaterial, "shaders/vapor_box.wesl", crate::ui_theme::SORT_TOP_BIAS);

/// Soft dark pool on the wing under a mass. It gives the white cloud a base to
/// sit on and keeps it from reading as a floating oval against the white wing.
#[derive(Clone, Copy, ShaderType, Default)]
struct ShadowParams {
    /// rgb = shadow colour, a unused.
    color: Vec4,
    /// x = opacity, y = seed/time.
    anim: Vec4,
}

#[derive(Asset, TypePath, AsBindGroup, Clone)]
struct ShadowMaterial {
    #[uniform(0)]
    params: ShadowParams,
}

blend_material!(ShadowMaterial, "shaders/vapor_shadow.wesl", crate::ui_theme::SORT_TOP_BIAS);

#[derive(Component)]
struct MassShadow {
    side: usize,
    surface: f32,
    base: Vec3,
}

/// A body-frame box over one wing whose fragment shader raymarches the
/// boundary-layer density. Unlike a flat sheet, the box has a real 3D
/// silhouette, and unlike billboards it has no stacking bands.
#[derive(Component)]
struct VaporBox {
    side: usize,
    surface: f32,
}

/// One camera-facing puff of the wing boundary-layer cloud. Positions are
/// recomputed from the body-frame anchor every frame, so the cloud rides the
/// wing instead of being left behind.
#[derive(Component)]
struct WingPuff {
    idx: usize,
}

#[derive(Resource)]
struct WingPuffs {
    age: [f32; WING_PUFF_COUNT],
    life: [f32; WING_PUFF_COUNT],
    strength: [f32; WING_PUFF_COUNT],
    seed: [f32; WING_PUFF_COUNT],
    anchor: [Vec3; WING_PUFF_COUNT],
    cursor: usize,
    spawn_acc: f32,
}

impl Default for WingPuffs {
    fn default() -> Self {
        Self {
            age: [f32::MAX; WING_PUFF_COUNT],
            life: [0.0; WING_PUFF_COUNT],
            strength: [0.0; WING_PUFF_COUNT],
            seed: [0.0; WING_PUFF_COUNT],
            anchor: [Vec3::ZERO; WING_PUFF_COUNT],
            cursor: 0,
            spawn_acc: 0.0,
        }
    }
}

fn hash01(mut x: f32) -> f32 {
    x = (x * 12.9898 + 78.233).sin() * 43_758.545;
    x.fract().abs()
}

/// A deterministic body-frame anchor in the wing's boundary layer. Root and
/// leading edge are biased denser, and the height spreads the cloud through a
/// few tens of centimetres above the surface.
fn wing_puff_anchor(name: &str, idx: usize) -> Vec3 {
    let side = idx / WING_PUFFS_PER_WING;
    let i = idx % WING_PUFFS_PER_WING;
    let mirror = if side == 0 { 1.0 } else { -1.0 };
    let corners = WING_SHEET.map(|(x, y, z)| Vec3::new(x * mirror, y, z));
    // Root-biased span and leading-edge-biased chord.
    let u = hash01(i as f32 * 0.618_034 + side as f32 * 0.37).powf(1.4);
    let v = hash01(i as f32 * 0.754_878 + side as f32 * 0.61).powf(1.3);
    let h = hash01(i as f32 * 0.569_840 + side as f32 * 0.23) * 0.07;
    let root = corners[0].lerp(corners[3], v);
    let tip = corners[1].lerp(corners[2], v);
    let model_point = root.lerp(tip, u) + Vec3::Y * h;
    model_to_body(name, model_point)
}

/// Build one wing patch in the body frame. `mirror` is +1 for the left wing
/// and −1 for the right; the index order flips with the mirror so both patches
/// face up. The grid is billowed slightly (and differently per layer), so the
/// layers read as a volume rather than a flat decal.
fn wing_sheet_mesh(name: &str, mirror: f32, layer: usize) -> Mesh {
    let corners = WING_SHEET.map(|(x, y, z)| Vec3::new(x * mirror, y, z));
    let mut positions = Vec::with_capacity((SPAN_SEGS + 1) * (CHORD_SEGS + 1));
    let mut normals = Vec::with_capacity(positions.capacity());
    let mut uvs = Vec::with_capacity(positions.capacity());
    for j in 0..=CHORD_SEGS {
        let v = j as f32 / CHORD_SEGS as f32;
        for i in 0..=SPAN_SEGS {
            let u = i as f32 / SPAN_SEGS as f32;
            let root = corners[0].lerp(corners[3], v);
            let tip = corners[1].lerp(corners[2], v);
            let model_point = root.lerp(tip, u);
            let mut body = model_to_body(name, model_point);
            // Window the billow to zero at both edges so the sheet stays seated
            // on the wing. The layer phase makes the three surfaces caress
            // rather than stack as identical planes.
            let window = (std::f32::consts::PI * u).sin() * (std::f32::consts::PI * v).sin();
            let wave = (u * 9.0 + layer as f32 * 1.7).sin() * (v * 6.0 - layer as f32 * 2.1).cos();
            body.y += wave * window * SHEET_LAYER_WAVE[layer];
            positions.push(body.to_array());
            normals.push(Vec3::Y.to_array());
            uvs.push([u, v]);
        }
    }
    let row = (SPAN_SEGS + 1) as u32;
    let mut indices = Vec::with_capacity(SPAN_SEGS * CHORD_SEGS * 6);
    for j in 0..CHORD_SEGS {
        for i in 0..SPAN_SEGS {
            let a = (j as u32) * row + i as u32;
            let b = a + 1;
            let c = a + row;
            let d = c + 1;
            if mirror > 0.0 {
                indices.extend([a, b, d, a, d, c]);
            } else {
                indices.extend([a, d, b, a, c, d]);
            }
        }
    }
    let mut mesh = Mesh::new(
        PrimitiveTopology::TriangleList,
        RenderAssetUsages::default(),
    );
    mesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, positions);
    mesh.insert_attribute(Mesh::ATTRIBUTE_NORMAL, normals);
    mesh.insert_attribute(Mesh::ATTRIBUTE_UV_0, uvs);
    mesh.insert_indices(Indices::U32(indices));
    mesh
}

// --- spawn -----------------------------------------------------------------

/// Spawn the F-22's vapor entities once the flown aircraft exists. Idempotent:
/// it bails as soon as a puff is present, so it fires once per flight.
///
/// The trail puffs are top-level (world-space) entities; the cone is parented to
/// `AircraftRoot` so it rides the airframe.
fn spawn_vapor(
    mut commands: Commands,
    sim: Res<Sim>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut puffs: ResMut<Assets<PuffMaterial>>,
    mut cones: ResMut<Assets<ConeMaterial>>,
    mut sheet_mats: ResMut<Assets<SheetMaterial>>,
    mut volume_mats: ResMut<Assets<VolumeMaterial>>,
    mut box_mats: ResMut<Assets<BoxMaterial>>,
    mut shadow_mats: ResMut<Assets<ShadowMaterial>>,
    mut wing: ResMut<WingPuffs>,
    roots: Query<Entity, With<AircraftRoot>>,
    existing: Query<(), With<VaporPuff>>,
) {
    if sim.name != "f22" || !existing.is_empty() {
        return;
    }
    let Ok(root) = roots.single() else {
        return;
    };

    // Trail puffs: one unit-radius quad per sample slot (Rectangle is 2×2, so
    // the transform's scale is the puff radius).
    let quad = meshes.add(Rectangle::new(2.0, 2.0));
    for side in 0..EMITTER_COUNT {
        for idx in 0..SAMPLES {
            let material = puffs.add(PuffMaterial {
                params: PuffParams {
                    color: Vec4::new(1.0, 1.0, 1.0, 0.0),
                    // `anim.x` is the mask's soft-edge threshold: 0 feathers the
                    // disc all the way from its centre, so nothing in the trail
                    // ever reads as a solid core however many puffs pile up.
                    anim: Vec4::new(0.15, 0.0, 0.0, 0.0),
                },
            });
            commands.spawn((
                VaporPuff { side, idx },
                Mesh3d(quad.clone()),
                MeshMaterial3d(material),
                Transform::default(),
                Visibility::Hidden,
                DespawnOnExit(AppState::Flying),
            ));
        }
    }

    // Transonic cone. The mesh is authored in model units; the cone entity is a
    // child of the body-frame root, so its scale must carry the model's own
    // scale (which `model_to_body` folds into the translation only).
    let cone = meshes.add(
        Cone {
            radius: CONE_RADIUS,
            height: (CONE_NOSE - CONE_AFT).length(),
        }
        .mesh()
        .resolution(48)
        .anchor(ConeAnchor::Tip)
        .build(),
    );
    let cone_mat = cones.add(ConeMaterial {
        params: ConeParams {
            anim: Vec4::new(0.0, 0.0, 0.0, 0.0),
            tint: Vec4::new(1.0, 1.0, 1.0, 0.0),
        },
    });
    // `ConeAnchor::Tip` puts the apex on the origin and the cone along -Y; a
    // -90° X rotation turns that onto the body's +Z (aft), so the apex lands on
    // the nose and the shell opens back over the fuselage.
    commands.spawn((
        VaporCone,
        Mesh3d(cone),
        MeshMaterial3d(cone_mat),
        Transform {
            translation: model_to_body(&sim.name, CONE_NOSE),
            rotation: Quat::from_rotation_x(-std::f32::consts::FRAC_PI_2),
            scale: Vec3::splat(model_scale(&sim.name)),
        },
        DespawnOnExit(AppState::Flying),
        ChildOf(root),
    ));

    // The old flat sheet is kept behind a switch for comparison, but the
    // visible surface vapor is now a volume cloud of wing-anchored puffs.
    if SHEET_ENABLED {
        for side in 0..2 {
            let mirror = if side == 0 { 1.0 } else { -1.0 };
            for (layer, layer_y) in SHEET_LAYER_OFFSET.iter().copied().enumerate() {
                let material = sheet_mats.add(SheetMaterial {
                    params: SheetParams::default(),
                });
                commands.spawn((
                    VaporSheet { side, layer },
                    Mesh3d(meshes.add(wing_sheet_mesh(&sim.name, mirror, layer))),
                    MeshMaterial3d(material),
                    Transform::from_xyz(0.0, layer_y, 0.0),
                    Visibility::Hidden,
                    DespawnOnExit(AppState::Flying),
                    ChildOf(root),
                ));
            }
        }
    }

    if BOX_VOLUME_ENABLED {
        // Two surfaces per side. Only the suction-side copy is shown; the
        // other stays hidden. The shader flips its height axis for the lower.
        let cube = meshes.add(Cuboid::new(1.0, 1.0, 1.0));
        for side in 0..2 {
            let mirror = if side == 0 { 1.0 } else { -1.0 };
            for surface in [1.0, -1.0] {
                let material = box_mats.add(BoxMaterial {
                    params: BoxParams::default(),
                });
                let y = if surface > 0.0 { 0.77 } else { -0.85 };
                commands.spawn((
                    VaporBox { side, surface },
                    Mesh3d(cube.clone()),
                    MeshMaterial3d(material),
                    Transform::from_xyz(-4.35 * mirror, y, 3.25)
                        .with_scale(Vec3::new(5.2, 1.40, 7.6)),
                    Visibility::Hidden,
                    DespawnOnExit(AppState::Flying),
                    ChildOf(root),
                ));

                let shadow = shadow_mats.add(ShadowMaterial {
                    params: ShadowParams::default(),
                });
                let y = if surface > 0.0 { 0.035 } else { -0.18 };
                let scale = if surface > 0.0 {
                    Vec3::splat(1.15)
                } else {
                    Vec3::new(1.15, -1.15, 1.15)
                };
                commands.spawn((
                    MassShadow {
                        side,
                        surface,
                        base: Vec3::new(0.0, y, 0.0),
                    },
                    Mesh3d(meshes.add(wing_sheet_mesh(&sim.name, mirror, 0))),
                    MeshMaterial3d(shadow),
                    Transform::from_xyz(0.0, y, 0.0).with_scale(scale),
                    Visibility::Hidden,
                    DespawnOnExit(AppState::Flying),
                    ChildOf(root),
                ));
            }
        }
    }

    *wing = WingPuffs::default();
    if WING_PUFFS_ENABLED {
        // Randomly oriented quads need both faces; otherwise half of the cloud
        // is back-face culled and it reads as a handful of flat cards.
        let mut volume_quad = Mesh::new(
            PrimitiveTopology::TriangleList,
            RenderAssetUsages::default(),
        );
        volume_quad.insert_attribute(
            Mesh::ATTRIBUTE_POSITION,
            vec![
                [-1.0, -1.0, 0.0],
                [1.0, -1.0, 0.0],
                [1.0, 1.0, 0.0],
                [-1.0, 1.0, 0.0],
            ],
        );
        volume_quad.insert_attribute(Mesh::ATTRIBUTE_NORMAL, vec![[0.0, 0.0, 1.0]; 4]);
        volume_quad.insert_attribute(
            Mesh::ATTRIBUTE_UV_0,
            vec![[0.0, 1.0], [1.0, 1.0], [1.0, 0.0], [1.0, 0.0]],
        );
        volume_quad.insert_indices(Indices::U32(vec![0, 1, 2, 0, 2, 3, 0, 2, 1, 0, 3, 2]));
        let volume_quad = meshes.add(volume_quad);
        for idx in 0..WING_PUFF_COUNT {
            wing.anchor[idx] = wing_puff_anchor(&sim.name, idx);
            let material = volume_mats.add(VolumeMaterial {
                params: VolumeParams::default(),
            });
            commands.spawn((
                WingPuff { idx },
                Mesh3d(volume_quad.clone()),
                MeshMaterial3d(material),
                Transform::default(),
                Visibility::Hidden,
                DespawnOnExit(AppState::Flying),
            ));
        }
    }

    // Every vapor pipeline compiles on this flight's first frames (see
    // `warm_pipelines`), so the cone appears the instant the mach band asks
    // for it instead of seconds into the acceleration.
    commands.insert_resource(PipelineWarmup {
        frames_left: WARMUP_FRAMES,
    });
}

/// For how many frames `warm_pipelines` keeps the vapor entities forced
/// visible after a flight spawns. All the materials sit at alpha 0 until the
/// flight state calls for them, so the window draws nothing — it exists only
/// to push every vapor shader through the pipeline cache before the pilot is
/// moving fast enough to see one.
const WARMUP_FRAMES: u32 = 30;

#[derive(Resource)]
struct PipelineWarmup {
    frames_left: u32,
}

/// Hidden entities are visibility-culled, so a vapor material's GPU pipeline
/// compiles the first frame it is actually shown — on WebGL2 that stall is
/// seconds long, which read as the vapor "popping in" mid-acceleration. For a
/// short window after the flight spawns, force every vapor entity visible
/// (alpha-0 materials draw nothing) so all six pipelines compile while the
/// aircraft is still on the runway; then hand visibility back to the drive
/// systems. Runs in PostUpdate after `drive_box`, the last visibility writer.
fn warm_pipelines(
    mut commands: Commands,
    mut warmup: Option<ResMut<PipelineWarmup>>,
    mut visibles: Query<
        &mut Visibility,
        Or<(
            With<VaporPuff>,
            With<VaporCone>,
            With<VaporSheet>,
            With<VaporBox>,
            With<MassShadow>,
            With<WingPuff>,
        )>,
    >,
) {
    let Some(warm) = warmup.as_deref_mut() else {
        return;
    };
    for mut vis in &mut visibles {
        *vis = Visibility::Inherited;
    }
    warm.frames_left -= 1;
    if warm.frames_left == 0 {
        commands.remove_resource::<PipelineWarmup>();
    }
}

/// The uniform scale `spawn_model_child` applies to `name`'s glTF, recovered
/// from `model_to_body` (which folds the yaw as a rotation, so a scaled unit
/// axis keeps its length).
fn model_scale(name: &str) -> f32 {
    (model_to_body(name, Vec3::X) - model_to_body(name, Vec3::ZERO)).length()
}

// --- drive -----------------------------------------------------------------

/// Vapor "feed" for one emitter (0..1): ramps in with the *local* angle of
/// attack — the aircraft's alpha plus the manoeuvre bias of a roll (side-signed)
/// or yaw — and picks up again in the transonic band, where the shock also
/// condenses air off the wing. Above the band it falls away again: a supersonic
/// jet does not trail vortex vapor.
fn emitter_feed(
    e: &Emitter,
    alpha_deg: f32,
    roll_dps: f32,
    yaw_dps: f32,
    mach: f32,
    airspeed_kt: f32,
) -> f32 {
    let rate_scale = RATE_CAL_KT / airspeed_kt.max(RATE_MIN_KT);
    let bias = (e.roll_gain * roll_dps * e.roll_side + e.yaw_gain * yaw_dps.abs()) * rate_scale;
    let local = alpha_deg + bias;
    let a = ((local - e.aoa0) / (e.aoa1 - e.aoa0)).clamp(0.0, 1.0);
    a.max(cone_band(mach) * e.mach_gain)
}

/// Transonic weighting (0..1): a band centred on Mach 1, used by both the
/// wingtip feed and the cone, so the two appear together as the aircraft
/// crosses the barrier.
fn cone_band(mach: f32) -> f32 {
    let d = (mach - MACH_C) / MACH_W;
    (1.0 - d * d).clamp(0.0, 1.0)
}

/// Build an orthonormal basis for a flow-aligned ribbon: local X across the
/// streak, local Y along the flow, local Z towards the camera. Projecting the
/// flow into the camera plane keeps the ribbon visible from directly astern,
/// where a pure velocity-billboard would collapse to a line.
fn streak_basis(flow: Vec3, to_cam: Vec3) -> (Vec3, Vec3, Vec3) {
    let z = to_cam.normalize_or_zero();
    let f = flow.normalize_or_zero();
    let mut y = f - z * f.dot(z);
    if y.length_squared() < 1.0e-4 {
        y = Vec3::Y - z * Vec3::Y.dot(z);
    }
    if y.length_squared() < 1.0e-4 {
        y = Vec3::X - z * Vec3::X.dot(z);
    }
    let y = y.normalize_or_zero();
    let x = y.cross(z).normalize_or_zero();
    (x, y, z)
}

/// How much of the flow lies across the view axis. 1 = broadside (stretch the
/// ribbon), 0 = looking straight down the trail (keep it round, or it collapses
/// to a vertical blade).
fn streak_alignment(flow: Vec3, to_cam: Vec3) -> f32 {
    let z = to_cam.normalize_or_zero();
    let f = flow.normalize_or_zero();
    (f - z * f.dot(z)).length().clamp(0.0, 1.0)
}

/// Advance both effects: drop fresh puffs at the wingtips, age and place the
/// trail, and refresh the cone's opacity.
fn drive_vapor(
    time: Res<Time>,
    sim: Res<Sim>,
    state: Res<AircraftState>,
    weather: Res<Weather>,
    sun: Query<&GlobalTransform, With<DirectionalLight>>,
    root: Query<
        &Transform,
        (
            With<AircraftRoot>,
            Without<VaporPuff>,
            Without<VaporCone>,
            Without<VaporSheet>,
            Without<WingPuff>,
        ),
    >,
    cameras: Query<
        &Transform,
        (
            With<Camera3d>,
            Without<crate::cloud_camera::CloudCamera>,
            Without<VaporPuff>,
            Without<VaporCone>,
            Without<VaporSheet>,
            Without<WingPuff>,
        ),
    >,
    mut trails: ResMut<Trails>,
    mut puffs: Query<
        (
            &VaporPuff,
            &MeshMaterial3d<PuffMaterial>,
            &mut Transform,
            &mut Visibility,
        ),
        (Without<VaporCone>, Without<VaporSheet>, Without<WingPuff>),
    >,
    mut puff_mats: ResMut<Assets<PuffMaterial>>,
    mut cones: Query<
        (&MeshMaterial3d<ConeMaterial>, &mut Visibility),
        (
            With<VaporCone>,
            Without<VaporPuff>,
            Without<VaporSheet>,
            Without<WingPuff>,
        ),
    >,
    mut cone_mats: ResMut<Assets<ConeMaterial>>,
    mut sheet_state: ResMut<SheetState>,
    mut wing: ResMut<WingPuffs>,
    mut wing_puffs: Query<
        (
            &WingPuff,
            &MeshMaterial3d<VolumeMaterial>,
            &mut Transform,
            &mut Visibility,
        ),
        (
            With<WingPuff>,
            Without<VaporPuff>,
            Without<VaporCone>,
            Without<VaporSheet>,
        ),
    >,
    mut volume_mats: ResMut<Assets<VolumeMaterial>>,
) {
    if sim.name != "f22" {
        return;
    }
    let dt = time.delta_secs();
    let time_s = time.elapsed_secs();

    // Wetter air condenses sooner, so rain makes the vapor a little more eager.
    let humidity = (0.72 + 0.5 * weather.rain).min(1.0);
    let speed = (state.cas_kts as f32 * 0.5144).max(0.0);
    let speed_gate = ((speed - SPEED0) / (SPEED1 - SPEED0)).clamp(0.0, 1.0);
    let alpha = state.alpha_deg as f32;
    let roll = state.roll_rate_dps as f32;
    let yaw = state.yaw_rate_dps as f32;
    let mach = state.mach as f32;
    // Each emitter's feed, from its own local flow.
    let mut feeds: [f32; EMITTER_COUNT] = std::array::from_fn(|i| {
        emitter_feed(&EMITTERS[i], alpha, roll, yaw, mach, state.cas_kts as f32)
            * humidity
            * speed_gate
    });

    // Surface burst: one 0.45 s mass per side, triggered by a load spike. The
    // mass snaps open, holds, then shrinks back into the wing. Tip trails are
    // held off for the first 0.10 s so the eye sees the mass pop first.
    let g = state.g_norm as f32;
    let load = ((g.abs() - 2.0) / 3.0).clamp(0.0, 1.0);
    // Low speed can still condense, but only a small patch in the trigger-prone
    // root/leading-edge region; high speed spreads the burst over more wing.
    let speed_kts = state.cas_kts as f32;
    let vapor_speed = smoothstep01(60.0, 320.0, speed_kts);
    let coverage = 0.30 + 0.70 * vapor_speed;

    // A roll changes the *local* angle of attack: the descending wing sees
    // more, the rising wing less. Delta alpha ~ roll_rate * semi-span / V.
    // Use per-side local alpha instead of a symmetric roll trigger, so a pure
    // roll does not pop the same cloud on both wings.
    let speed_mps = speed.max(25.0);
    let delta_alpha = (roll * 3.5 / speed_mps).clamp(-6.0, 6.0);
    let local_alpha = [alpha - delta_alpha, alpha + delta_alpha];
    let previous_g = sheet_state.prev_g;
    let g_spike = previous_g.abs() > 0.5
        && ((g - previous_g).abs() > 0.5 || (previous_g.abs() < 3.0 && g.abs() >= 3.0));

    for (side, &a) in local_alpha.iter().enumerate() {
        let prev_a = sheet_state.prev_local_alpha[side];
        let alpha_load = ((a.abs() - 8.0) / 8.0).clamp(0.0, 1.0);
        let alpha_rate = (a - prev_a) / dt.max(1.0e-3);
        let maneuver = load
            .max(alpha_load * 0.9)
            .max((alpha_rate.abs() / 80.0).clamp(0.0, 1.0));
        let base = maneuver * humidity * (0.55 + 0.45 * vapor_speed);
        let alpha_spike = (a - prev_a).abs() > 3.0
            || alpha_rate.abs() > 25.0
            || (a.abs() > 12.0 && prev_a.abs() <= 12.0);
        let surface = if g < -0.5 || a < 0.0 { -1.0 } else { 1.0 };
        let spike = g_spike || alpha_spike;
        // Any maneuver spike above the trail floor unlocks the tip trails,
        // independently of the burst cloud — so raising the burst gate to
        // high-speed only never starves the low-speed vortex ribbons.
        if speed_kts >= 60.0 && spike {
            sheet_state.trail_unlock[side] = true;
        }
        // The wing-surface burst cloud is deliberately high-speed only.
        if speed_kts > BURST_SPEED_KT && spike {
            sheet_state.age[side] = 0.0;
            sheet_state.strength[side] = base.clamp(0.15, 1.0);
            // The descending wing also gets the larger low-pressure footprint.
            let side_coverage = coverage * (0.75 + 0.25 * alpha_load);
            sheet_state.coverage[side] = side_coverage.clamp(0.25, 1.0);
            sheet_state.surface[side] = surface;
            // Hand the pop to `play_vapor_sfx` — kept out of `drive_vapor`,
            // which is already at Bevy's 16-parameter system limit.
            sheet_state.pending_burst[side] = true;
        }
    }

    sheet_state.prev_g = g;
    sheet_state.prev_local_alpha = local_alpha;
    for side in 0..2 {
        if sheet_state.strength[side] > 0.0 {
            sheet_state.age[side] += dt;
            if sheet_state.age[side] >= 0.45 {
                sheet_state.age[side] = 0.0;
                sheet_state.strength[side] = 0.0;
                sheet_state.intensity[side] = 0.0;
            } else {
                let (density, _) = burst_envelope(sheet_state.age[side]);
                sheet_state.intensity[side] = sheet_state.strength[side] * density;
                if sheet_state.age[side] > 0.10 {
                    sheet_state.trail_unlock[side] = true;
                }
            }
        } else {
            sheet_state.intensity[side] = 0.0;
        }
    }
    for (i, e) in EMITTERS.iter().enumerate() {
        if e.trail {
            let side = if e.roll_side < 0.0 { 0 } else { 1 };
            if !sheet_state.trail_unlock[side] {
                feeds[i] = 0.0;
            }
        }
    }

    // Age every sample every frame, fed or not, so a trail thins out and dies
    // once the wing unloads.
    for side in 0..EMITTER_COUNT {
        for i in 0..SAMPLES {
            trails.age[side][i] += dt;
        }
    }

    let root_tf = root.single().ok();
    // +Z is aft in the body frame; using it as the ribbon's long axis puts the
    // feathered tail behind the aircraft.
    let flow_dir = root_tf
        .map(|tf| (tf.rotation * Vec3::Z).normalize_or_zero())
        .unwrap_or(Vec3::Z);
    if let Some(root_tf) = root_tf {
        for (side, e) in EMITTERS.iter().enumerate() {
            if !e.trail {
                continue;
            }
            let at = root_tf.translation + root_tf.rotation * model_to_body(&sim.name, e.model);
            // Drop a sample every `STEP` metres, and record the *current*
            // strength with it, so the trail's brightness follows the load.
            if !trails.live[side] {
                trails.live[side] = true;
                trails.last[side] = at;
            }
            trails.acc[side] += (at - trails.last[side]).length();
            trails.last[side] = at;
            while trails.acc[side] >= STEP {
                trails.acc[side] -= STEP;
                let c = trails.cursor[side];
                trails.pos[side][c] = at;
                trails.age[side][c] = 0.0;
                trails.feed[side][c] = feeds[side];
                trails.cursor[side] = (c + 1) % SAMPLES;
            }
        }
    }

    let cam = cameras.single().ok().map(|t| t.translation);
    for (puff, mat, mut tf, mut vis) in &mut puffs {
        let (side, idx) = (puff.side, puff.idx);
        let age = trails.age[side][idx];
        let strength = trails.feed[side][idx];
        if age >= LIFE || strength <= 0.002 {
            if *vis != Visibility::Hidden {
                *vis = Visibility::Hidden;
            }
            continue;
        }
        *vis = Visibility::Inherited;

        // Fade on whichever runs out first: the vapor's own clock, or the slot
        // reaching the end of the ring (its world distance behind the tip). The
        // second term is what keeps the ribbon from ending in a hard cut when
        // the jet is fast enough to recycle the ring inside `LIFE`.
        let back = (trails.cursor[side] + SAMPLES - 1 - idx) % SAMPLES;
        let f = (age / LIFE).max(back as f32 / SAMPLES as f32);
        // Deterministic per-slot wobble, so the ribbon isn't a row of identical
        // beads: some puffs are bigger, some brighter, on independent hashes.
        let h = (idx as f32) * 0.618_034;
        let size_j = 0.8 + 0.4 * h.fract();
        let alpha_j = 0.7 + 0.6 * (h * 1.7).fract();
        // A fixed wobble per slot, in a deterministic random direction.
        let wob = Quat::from_euler(
            EulerRot::ZYX,
            (h * 2.3).fract() * std::f32::consts::TAU,
            (h * 3.1).fract() * std::f32::consts::TAU,
            0.0,
        ) * Vec3::Z;
        tf.translation = trails.pos[side][idx] + wob * PUFF_WANDER * size_j * f;
        // A long, thin sheet along the relative wind instead of a round puff.
        // The length grows with airspeed so the trail reads as a streak rather
        // than a row of beads; the width still expands as the vapor dissipates.
        let radius = (PUFF_R0 + (PUFF_R1 - PUFF_R0) * f) * size_j;
        let base_stretch = (1.4 + speed * 0.006).clamp(1.4, 3.6);
        let mut stretch = base_stretch;
        // Camera-facing, with the long axis projected along the flow. From
        // directly astern the projection vanishes, so fade the stretch back to
        // 1 and keep the puff round instead of snapping it to a vertical blade.
        if let Some(cam) = cam {
            let to_cam = cam - tf.translation;
            if to_cam.length_squared() > 1e-6 {
                let align = streak_alignment(flow_dir, to_cam);
                stretch = 1.0 + (base_stretch - 1.0) * align;
                let (x, y, z) = streak_basis(flow_dir, to_cam);
                tf.rotation = Quat::from_mat3(&Mat3::from_cols(x, y, z));
            }
        }
        tf.scale = Vec3::new(radius, radius * stretch, 1.0);
        if let Some(mut m) = puff_mats.get_mut(&mat.0) {
            m.params.color.w = PUFF_ALPHA * strength * alpha_j * (1.0 - f).powf(1.4);
            // Per-slot seed: without it every ribbon segment repeats the same
            // filament pattern and the trail reads as a periodic texture.
            m.params.anim.y = h.fract();
        }
    }

    // The cone is translucent, not a solid funnel: it wraps the airframe and the
    // aircraft reads through it.
    let cone_i = cone_band(state.mach as f32) * humidity;
    for (mat, mut vis) in &mut cones {
        *vis = if cone_i > 0.02 {
            Visibility::Inherited
        } else {
            Visibility::Hidden
        };
        if let Some(mut m) = cone_mats.get_mut(&mat.0) {
            m.params.anim = Vec4::new(cone_i, time_s, 0.0, 0.0);
        }
    }

    let sun_dir = sun
        .single()
        .map(|t| (t.rotation() * Vec3::Z).normalize_or_zero())
        .unwrap_or(Vec3::Y);

    // Wing boundary-layer volume: while the burst is active, recycle puffs
    // through a deterministic anchor cloud on the wing. Each puff lives a few
    // tenths of a second, so the cloud snaps on and dissipates in place.
    if WING_PUFFS_ENABLED {
        let wing_intensity = sheet_state.intensity[0].max(sheet_state.intensity[1]);
        if wing_intensity > 0.01 && root_tf.is_some() {
            wing.spawn_acc += dt * (200.0 + 1100.0 * wing_intensity);
            while wing.spawn_acc >= 1.0 {
                wing.spawn_acc -= 1.0;
                let idx = wing.cursor % WING_PUFF_COUNT;
                wing.cursor = wing.cursor.wrapping_add(1);
                wing.age[idx] = 0.0;
                wing.life[idx] = WING_PUFF_LIFE_MIN
                    + (WING_PUFF_LIFE_MAX - WING_PUFF_LIFE_MIN)
                        * hash01(idx as f32 * 1.7 + wing.cursor as f32 * 0.3);
                wing.strength[idx] = wing_intensity;
                wing.seed[idx] = hash01(idx as f32 * 0.618 + wing.cursor as f32 * 0.37);
            }
        }
        for age in &mut wing.age {
            *age += dt;
        }
        for (puff, mat, mut tf, mut vis) in &mut wing_puffs {
            let idx = puff.idx;
            let age = wing.age[idx];
            let life = wing.life[idx];
            let strength = wing.strength[idx];
            let Some(root_tf) = root_tf else {
                *vis = Visibility::Hidden;
                continue;
            };
            if age >= life || strength <= 0.01 {
                *vis = Visibility::Hidden;
                continue;
            }
            let f = age / life;
            let side = (idx / WING_PUFFS_PER_WING).min(1);
            let intensity = strength * sheet_state.intensity[side] * (1.0 - f).powf(1.2);
            if intensity <= 0.004 {
                *vis = Visibility::Hidden;
                continue;
            }
            *vis = Visibility::Inherited;

            let seed = wing.seed[idx];
            let jitter = Vec3::new(
                (seed * 17.0 + f * 3.1).sin(),
                (seed * 23.0 + f * 2.3).cos() * 0.5,
                (seed * 29.0 - f * 2.7).cos(),
            ) * (0.10 + 0.18 * f);
            let anchor = wing.anchor[idx] + jitter;
            let world = root_tf.translation + root_tf.rotation * anchor;
            tf.translation = world;
            let size = (0.45 + 0.75 * f) * (0.80 + 0.40 * seed);
            tf.scale = Vec3::splat(size);
            // Random plane orientations fill a volume from every view;
            // parallel camera-facing billboards stack into visible bands.
            tf.rotation = Quat::from_euler(
                EulerRot::XYZ,
                seed * std::f32::consts::TAU,
                (seed * 1.7).fract() * std::f32::consts::TAU,
                (seed * 2.3).fract() * std::f32::consts::TAU,
            );
            if let Some(mut m) = volume_mats.get_mut(&mat.0) {
                m.params.color = Vec4::new(0.96, 0.98, 1.0, WING_PUFF_ALPHA * intensity);
                m.params.anim = Vec4::new(seed, time_s, 0.0, 0.0);
                m.params.light = Vec4::new(sun_dir.x, sun_dir.y, sun_dir.z, 0.0);
            }
        }
    }
}

/// Push the burst envelope and sun direction into the single volumetric wing
/// sheet. Kept separate from `drive_vapor` to stay within Bevy's system-param
/// limit.
fn drive_sheet(
    time: Res<Time>,
    sheet_state: Res<SheetState>,
    sun: Query<&GlobalTransform, With<DirectionalLight>>,
    mut sheets: Query<(&VaporSheet, &MeshMaterial3d<SheetMaterial>, &mut Visibility)>,
    mut sheet_mats: ResMut<Assets<SheetMaterial>>,
) {
    let sun_dir = sun
        .single()
        .map(|t| (t.rotation() * Vec3::Z).normalize_or_zero())
        .unwrap_or(Vec3::Y);
    let t = time.elapsed_secs();
    for (sheet, mat, mut vis) in &mut sheets {
        let intensity = sheet_state.intensity[sheet.side.min(1)];
        *vis = if intensity > 0.004 {
            Visibility::Inherited
        } else {
            Visibility::Hidden
        };
        if let Some(mut m) = sheet_mats.get_mut(&mat.0) {
            m.params.anim = Vec4::new(intensity, t, sheet.side as f32, sheet.layer as f32);
            m.params.tint = Vec4::new(0.95, 0.97, 1.0, 0.0);
            m.params.light = Vec4::new(sun_dir.x, sun_dir.y, sun_dir.z, 0.0);
        }
    }
}

/// Feed the burst envelope, sun and world-to-local transform into the two wing
/// volume boxes. The shader does the raymarch; this just keeps the box glued to
/// the airframe and tells it how dense to be.
fn drive_box(
    time: Res<Time>,
    sheet_state: Res<SheetState>,
    sun: Query<&GlobalTransform, With<DirectionalLight>>,
    mut boxes: Query<(
        &VaporBox,
        &GlobalTransform,
        &MeshMaterial3d<BoxMaterial>,
        &mut Visibility,
    )>,
    mut box_mats: ResMut<Assets<BoxMaterial>>,
) {
    let sun_dir = sun
        .single()
        .map(|t| (t.rotation() * Vec3::Z).normalize_or_zero())
        .unwrap_or(Vec3::Y);
    let t = time.elapsed_secs();
    for (vapor_box, global, mat, mut vis) in &mut boxes {
        let side = vapor_box.side.min(1);
        let surface = sheet_state.surface[side];
        if (vapor_box.surface > 0.0) != (surface > 0.0) {
            *vis = Visibility::Hidden;
            continue;
        }
        let (density, scale) = burst_envelope(sheet_state.age[side]);
        let coverage = sheet_state.coverage[side].clamp(0.25, 1.0);
        let size = 0.55 + 0.45 * coverage;
        let intensity = sheet_state.strength[side] * density * size;
        let scale = scale * size;
        let age = sheet_state.age[side];
        let f = (age / 0.45).clamp(0.0, 1.0);
        let outboard = if side == 0 { -1.0 } else { 1.0 };
        let dx = outboard * 0.6 * f / 5.2;
        let dz = 1.8 * f / 7.6;
        *vis = if intensity > 0.004 && scale > 0.004 {
            Visibility::Inherited
        } else {
            Visibility::Hidden
        };
        if let Some(mut m) = box_mats.get_mut(&mat.0) {
            m.params.color = Vec4::new(0.96, 0.98, 1.0, coverage);
            m.params.anim = Vec4::new(intensity, t, side as f32, scale);
            m.params.light = Vec4::new(sun_dir.x, sun_dir.y, sun_dir.z, surface);
            m.params.drift = Vec4::new(dx, 0.0, dz, f);
            m.params.world_to_local = global.to_matrix().inverse();
        }
    }
}

/// Fade the wing shadow with the same burst envelope as its mass.
fn drive_shadow(
    time: Res<Time>,
    sheet_state: Res<SheetState>,
    mut shadows: Query<(
        &MassShadow,
        &MeshMaterial3d<ShadowMaterial>,
        &mut Transform,
        &mut Visibility,
    )>,
    mut shadow_mats: ResMut<Assets<ShadowMaterial>>,
) {
    let t = time.elapsed_secs();
    for (shadow, mat, mut tf, mut vis) in &mut shadows {
        let side = shadow.side.min(1);
        let surface = sheet_state.surface[side];
        if (shadow.surface > 0.0) != (surface > 0.0) {
            *vis = Visibility::Hidden;
            continue;
        }
        let (density, _) = burst_envelope(sheet_state.age[side]);
        let coverage = sheet_state.coverage[side].clamp(0.25, 1.0);
        let opacity = sheet_state.strength[side] * density;
        let age = sheet_state.age[side];
        let f = (age / 0.45).clamp(0.0, 1.0);
        let outboard = if side == 0 { -1.0 } else { 1.0 };
        tf.translation = shadow.base + Vec3::new(outboard * 0.6 * f * 0.15, 0.0, 1.8 * f * 0.15);
        *vis = if opacity > 0.004 {
            Visibility::Inherited
        } else {
            Visibility::Hidden
        };
        if let Some(mut m) = shadow_mats.get_mut(&mat.0) {
            m.params.color = Vec4::new(0.035, 0.055, 0.085, coverage);
            m.params.anim = Vec4::new(opacity * 1.20, t + side as f32, 0.0, 0.0);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The left wingtip — the plain angle-of-attack emitter.
    fn tip() -> &'static Emitter {
        &EMITTERS[0]
    }

    /// Feed with no manoeuvre, at the calibration airspeed.
    fn still(e: &Emitter, alpha_deg: f32, mach: f32) -> f32 {
        emitter_feed(e, alpha_deg, 0.0, 0.0, mach, RATE_CAL_KT)
    }

    #[test]
    fn tip_feed_is_off_at_cruise_alpha_and_full_when_pulled() {
        // Level cruise: a few degrees of alpha, subsonic — no vapor.
        assert_eq!(still(tip(), 2.0, 0.6), 0.0);
        // A hard pull feeds it fully.
        assert_eq!(still(tip(), 20.0, 0.6), 1.0);
        // Between the thresholds it ramps.
        let mid = still(tip(), (tip().aoa0 + tip().aoa1) / 2.0, 0.6);
        assert!(mid > 0.0 && mid < 1.0, "expected a ramp, got {mid}");
    }

    #[test]
    fn tip_feed_picks_up_near_the_sound_barrier() {
        // Little alpha, but crossing the barrier still condenses off the tips:
        // full at the band's peak, ramping up on the way in and gone once past.
        assert_eq!(still(tip(), 0.0, MACH_C), 1.0);
        let mid = still(tip(), 0.0, MACH_C - MACH_W / 2.0);
        assert!(mid > 0.0 && mid < 1.0, "expected a ramp, got {mid}");
        assert_eq!(still(tip(), 0.0, MACH_C + MACH_W), 0.0);
    }

    #[test]
    fn only_the_tips_pick_up_the_transonic_band() {
        // The leading edge and forebody are vortex emitters: crossing the
        // barrier must not light them, or all seven pile into one blob.
        for e in &EMITTERS[2..] {
            assert_eq!(
                still(e, 0.0, MACH_C),
                0.0,
                "a non-tip emitter drew on the transonic band"
            );
        }
    }

    #[test]
    fn roll_fogs_only_the_descending_wing() {
        // A right roll (p > 0) drives the model −x / right wing down, so at an
        // angle of attack too low for the left leading edge, the right one fogs
        // and the left stays clear. `EMITTERS[2]` is the left LE, `[3]` the
        // right (same anchor, mirrored).
        let (le_l, le_r) = (&EMITTERS[2], &EMITTERS[3]);
        assert_eq!(le_l.model.x, -le_r.model.x, "the LE pair must be mirrored");
        let alpha = 8.0; // inside the window (aoa0 6), so the base feed ramps
        let quiet = still(le_l, alpha, 0.5);
        let left = emitter_feed(le_l, alpha, 90.0, 0.0, 0.5, RATE_CAL_KT);
        let right = emitter_feed(le_r, alpha, 90.0, 0.0, 0.5, RATE_CAL_KT);
        assert!(
            right > quiet && left < quiet,
            "a right roll must raise the right wing's feed and lower the left's: \
             quiet {quiet}, left {left}, right {right}"
        );
    }

    #[test]
    fn the_same_roll_feeds_harder_when_slow() {
        // The manoeuvre bias scales as 1/airspeed: identical roll rate, more
        // vapor at 200 kt than at 400 kt.
        let le = &EMITTERS[3];
        let fast = emitter_feed(le, 4.0, 200.0, 0.0, 0.5, 400.0);
        let slow = emitter_feed(le, 4.0, 200.0, 0.0, 0.5, 200.0);
        assert!(
            slow > fast,
            "a slow roll must feed harder than a fast one: slow {slow}, fast {fast}"
        );
    }

    #[test]
    fn the_centreline_emitter_ignores_roll() {
        // The forebody sits on the roll axis, so a roll must not bias it.
        let fore = EMITTERS.last().unwrap();
        assert_eq!(fore.model.x, 0.0);
        assert_eq!(fore.roll_gain, 0.0);
        let a = emitter_feed(fore, 12.0, 200.0, 0.0, 0.5, RATE_CAL_KT);
        let b = emitter_feed(fore, 12.0, -200.0, 0.0, 0.5, RATE_CAL_KT);
        assert_eq!(a, b, "a roll must not bias an emitter on the centreline");
    }

    #[test]
    fn cone_band_peaks_at_mach_one_and_vanishes_far_from_it() {
        assert!((cone_band(MACH_C) - 1.0).abs() < 1e-6);
        assert_eq!(cone_band(0.4), 0.0);
        assert_eq!(cone_band(1.6), 0.0);
        // Symmetric about the peak.
        assert!((cone_band(MACH_C - MACH_W / 2.0) - cone_band(MACH_C + MACH_W / 2.0)).abs() < 1e-6);
    }

    #[test]
    fn model_scale_recovers_the_f22_gltf_scale() {
        // `model_spec` scales the F-22 by 6.8 (see `view.rs`).
        assert!((model_scale("f22") - 6.8).abs() < 1e-3);
    }

    #[test]
    fn only_the_wingtips_drop_world_space_trails() {
        for (i, e) in EMITTERS.iter().enumerate() {
            assert_eq!(e.trail, i < 2, "emitter {i} has the wrong trail flag");
        }
    }

    #[test]
    fn wing_sheet_mesh_is_a_subdivided_patch() {
        let mesh = wing_sheet_mesh("f22", 1.0, 0);
        let Some(positions) = mesh.attribute(Mesh::ATTRIBUTE_POSITION) else {
            panic!("sheet has no positions");
        };
        assert_eq!(positions.len(), (SPAN_SEGS + 1) * (CHORD_SEGS + 1));
        assert_eq!(
            mesh.indices().expect("indexed").len(),
            SPAN_SEGS * CHORD_SEGS * 6
        );
    }

    #[test]
    fn wing_sheet_first_triangle_faces_up_on_both_wings() {
        for mirror in [1.0f32, -1.0] {
            let p = |i: usize| {
                let (x, y, z) = WING_SHEET[i];
                model_to_body("f22", Vec3::new(x * mirror, y, z))
            };
            // Must match the first triangle in `wing_sheet_mesh`.
            let (a, b, c) = if mirror > 0.0 {
                (p(0), p(1), p(2))
            } else {
                (p(0), p(2), p(1))
            };
            let normal = (b - a).cross(c - a);
            assert!(
                normal.y > 0.0,
                "sheet triangle faces down for mirror {mirror}: {normal:?}"
            );
        }
    }

    #[test]
    fn streak_alignment_is_full_broadside_and_zero_astern() {
        assert!((streak_alignment(Vec3::NEG_Z, Vec3::X) - 1.0).abs() < 1e-4);
        assert!(streak_alignment(Vec3::NEG_Z, Vec3::Z) < 1e-4);
        assert!(streak_alignment(Vec3::NEG_Z, (Vec3::Z + Vec3::X).normalize()) > 0.5);
    }

    #[test]
    fn streak_basis_is_orthonormal_from_side_and_astern() {
        for (flow, to_cam) in [
            // Ordinary side view.
            (Vec3::NEG_Z, Vec3::X),
            // Directly astern: the flow projects to a point, so the up fallback
            // must keep a usable width axis.
            (Vec3::NEG_Z, Vec3::Z),
            // Front quarter.
            (Vec3::NEG_Z, (Vec3::Z + Vec3::X).normalize()),
        ] {
            let (x, y, z) = streak_basis(flow, to_cam);
            assert!((x.length() - 1.0).abs() < 1e-4, "x not unit: {x:?}");
            assert!((y.length() - 1.0).abs() < 1e-4, "y not unit: {y:?}");
            assert!((z.length() - 1.0).abs() < 1e-4, "z not unit: {z:?}");
            assert!(x.dot(y).abs() < 1e-4, "x/y not perpendicular");
            assert!(x.dot(z).abs() < 1e-4, "x/z not perpendicular");
            assert!(y.dot(z).abs() < 1e-4, "y/z not perpendicular");
        }
    }
}
