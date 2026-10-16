//! F-22 condensation vapor: the white mist a fighter drags off its wingtips in a
//! hard manoeuvre, and the cone that blooms as it pushes through the sound
//! barrier.
//!
//! Both effects are gated to the F-22 (`sim.name == "f22"`), like the engine
//! effects in `fx.rs`: only that model ships the measured wing-tip anchors, and
//! only its FDM publishes the two signals that drive them — angle of attack
//! (`aero/alpha-rad`, both backends) and Mach (`velocities/mach`). Neither is
//! smoke: the air's own water vapor condensing where the flow drops in pressure
//! and temperature, so both track the *flow*, not the engine.
//!
//! - **Wingtip trails.** Condensation first shows in the low-pressure core of
//!   the wingtip vortex, which strengthens with the wing's load. So the trails
//!   are driven by angle of attack: pull and they feed, unload and they stop
//!   and dissipate. Each tip drops soft billboard puffs into **world** space,
//!   recycled from a fixed ring, so the aircraft flies out of them and a hard
//!   turn draws long curved ribbons for free. Samples are spaced by *distance*
//!   along the tip's path, so the ribbon stays continuous whether the jet is
//!   ambling at 200 kt or crossing the barrier.
//! - **Transonic cone.** Near Mach 1 the shock condenses the air into a
//!   translucent cone wrapping the airframe: one cone shell, a child of the
//!   airframe, whose opacity peaks in a band around Mach 1.
//!
//! The plugin is self-contained: it spawns its entities once `AircraftRoot`
//! exists (see [`spawn_vapor`]) rather than hooking the flight spawn, so
//! examples that build their own scene (and never add it) are unaffected. Every
//! entity carries `DespawnOnExit`, so a fresh flight starts clean.

use bevy::prelude::*;
use bevy::render::mesh::ConeAnchor;
use bevy::render::render_resource::{AsBindGroup, ShaderType};
use bevy::shader::ShaderRef;

use crate::app_state::AppState;
use crate::sim::{AircraftState, Sim};
use crate::view::{AircraftRoot, model_to_body};
use crate::weather::Weather;

/// Wing-tip emitters in the F-22's own glTF frame (metres, +X left / +Y up /
/// +Z nose), read off the wing mesh (span ±1.01 model units) at the tip chord.
const F22_TIP_L: Vec3 = Vec3::new(1.00, 0.14, -0.72);
const F22_TIP_R: Vec3 = Vec3::new(-1.00, 0.14, -0.72);

/// One trail per wingtip.
const SIDES: usize = 2;

/// History samples per trail; the ring holds `SAMPLES · STEP` metres of trail.
const SAMPLES: usize = 60;

/// Distance (m) between trail samples along a wingtip's path. Emission is
/// spaced in *distance*, not time: a time-spaced ring would put the samples
/// 5 m apart at Mach 1 and string the trail out into beads, where at 200 kt the
/// same ring merges into a ribbon. Distance spacing holds at any speed.
const STEP: f32 = 1.8;

/// How long vapor lives (s), on its own clock. Above about 160 kt the ring is
/// the limit, so this only binds when the jet is slow: same trail, shorter.
const LIFE: f32 = 1.6;

/// Puff radius (m) at birth and at death; a puff expands as it dissipates. Born
/// wider than half the sample spacing so consecutive puffs merge into a ribbon
/// rather than reading as a row of beads.
const PUFF_R0: f32 = 1.6;
const PUFF_R1: f32 = 3.4;
/// Peak opacity of a fresh puff on a fully-fed trail. Kept low because a dense
/// stream of them accumulates.
const PUFF_ALPHA: f32 = 0.13;
/// How far (m) a puff's centre may wander off the sampled path. Breaks the
/// ribbon up so it reads as vapor rather than a smooth tube, without pulling
/// neighbouring puffs far enough apart to come unstitched.
const PUFF_WANDER: f32 = 0.7;

/// Angle of attack (deg) below which no vapor shows, and the level that feeds
/// the trail fully.
const AOA0: f32 = 5.0;
const AOA1: f32 = 16.0;
/// Speed (m/s) below which the wingtip trail is suppressed — on rotation the
/// wing is loaded but the aircraft is slow, and a stationary puff pile looks
/// wrong. Fades in over `SPEED0..SPEED1`.
const SPEED0: f32 = 25.0;
const SPEED1: f32 = 60.0;
/// Mach near which the transonic cone is full, and the half-width of its band.
const MACH_C: f32 = 1.02;
const MACH_W: f32 = 0.20;

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
        ))
        .init_resource::<Trails>()
        .add_systems(
            Update,
            spawn_vapor.run_if(in_state(AppState::Flying)),
        )
        // Reads the freshly-written airframe pose, so it must run after
        // `sync_aircraft` placed the aircraft this frame.
        .add_systems(
            Update,
            drive_vapor
                .after(crate::view::sync_aircraft)
                .run_if(in_state(AppState::Flying)),
        );
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

#[derive(Asset, TypePath, AsBindGroup, Clone)]
struct PuffMaterial {
    #[uniform(0)]
    params: PuffParams,
}

impl Material for PuffMaterial {
    fn fragment_shader() -> ShaderRef {
        "shaders/vapor_puff.wesl".into()
    }

    // Vapor is translucent, not emissive: it dims the sky behind it rather than
    // adding to it. Each puff owns its material so it can fade on its own.
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

/// One quad of a wingtip trail, addressed by its slot in [`Trails`]. Not
/// parented to the aircraft: the puffs hang in world space once dropped.
#[derive(Component)]
struct VaporPuff {
    side: usize,
    idx: usize,
}

/// Ring of history samples, one per wingtip. A slot holds where the tip was,
/// how long ago, and how strong the vapor feed was when it was dropped.
#[derive(Resource)]
struct Trails {
    pos: [[Vec3; SAMPLES]; SIDES],
    age: [[f32; SAMPLES]; SIDES],
    feed: [[f32; SAMPLES]; SIDES],
    cursor: [usize; SIDES],
    /// Distance the tip has covered since its last sample, per side.
    acc: [f32; SIDES],
    /// Where the tip was on the previous frame, per side.
    last: [Vec3; SIDES],
    /// Whether the tip has been sighted yet; the first sample is dropped as soon
    /// as it is, so the trail starts under the wing rather than behind it.
    live: [bool; SIDES],
}

impl Default for Trails {
    fn default() -> Self {
        Trails {
            pos: [[Vec3::ZERO; SAMPLES]; SIDES],
            // Start every slot "old" so an untouched one stays hidden.
            age: [[f32::MAX; SAMPLES]; SIDES],
            feed: [[0.0; SAMPLES]; SIDES],
            cursor: [0; SIDES],
            acc: [0.0; SIDES],
            last: [Vec3::ZERO; SIDES],
            live: [false; SIDES],
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

impl Material for ConeMaterial {
    fn fragment_shader() -> ShaderRef {
        "shaders/vapor_cone.wesl".into()
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

#[derive(Component)]
struct VaporCone;

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
    for side in 0..SIDES {
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
}

/// The uniform scale `spawn_model_child` applies to `name`'s glTF, recovered
/// from `model_to_body` (which folds the yaw as a rotation, so a scaled unit
/// axis keeps its length).
fn model_scale(name: &str) -> f32 {
    (model_to_body(name, Vec3::X) - model_to_body(name, Vec3::ZERO)).length()
}

// --- drive -----------------------------------------------------------------

/// Vapor "feed" from the wing's load (0..1): ramps in with angle of attack, and
/// picks up again in the transonic band, where the shock also condenses air off
/// the tips. Above the band it falls away again — a supersonic jet does not
/// trail wingtip vapor.
fn wing_feed(alpha_deg: f32, mach: f32) -> f32 {
    let a = ((alpha_deg - AOA0) / (AOA1 - AOA0)).clamp(0.0, 1.0);
    a.max(cone_band(mach))
}

/// Transonic weighting (0..1): a band centred on Mach 1, used by both the
/// wingtip feed and the cone, so the two appear together as the aircraft
/// crosses the barrier.
fn cone_band(mach: f32) -> f32 {
    let d = (mach - MACH_C) / MACH_W;
    (1.0 - d * d).clamp(0.0, 1.0)
}

/// Advance both effects: drop fresh puffs at the wingtips, age and place the
/// trail, and refresh the cone's opacity.
fn drive_vapor(
    time: Res<Time>,
    sim: Res<Sim>,
    state: Res<AircraftState>,
    weather: Res<Weather>,
    root: Query<&Transform, (With<AircraftRoot>, Without<VaporPuff>, Without<VaporCone>)>,
    cameras: Query<&Transform, (With<Camera3d>, Without<VaporPuff>, Without<VaporCone>)>,
    mut trails: ResMut<Trails>,
    mut puffs: Query<
        (
            &VaporPuff,
            &MeshMaterial3d<PuffMaterial>,
            &mut Transform,
            &mut Visibility,
        ),
        Without<VaporCone>,
    >,
    mut puff_mats: ResMut<Assets<PuffMaterial>>,
    mut cones: Query<
        (&MeshMaterial3d<ConeMaterial>, &mut Visibility),
        (With<VaporCone>, Without<VaporPuff>),
    >,
    mut cone_mats: ResMut<Assets<ConeMaterial>>,
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
    let feed = wing_feed(state.alpha_deg as f32, state.mach as f32) * humidity * speed_gate;

    // Age every sample every frame, fed or not, so a trail thins out and dies
    // once the wing unloads.
    for side in 0..SIDES {
        for i in 0..SAMPLES {
            trails.age[side][i] += dt;
        }
    }

    if let Ok(root_tf) = root.single() {
        for (side, model) in [F22_TIP_L, F22_TIP_R].iter().enumerate() {
            let tip = root_tf.translation + root_tf.rotation * model_to_body(&sim.name, *model);
            // Drop a sample every `STEP` metres, and record the *current*
            // strength with it, so the trail's brightness follows the load.
            if !trails.live[side] {
                trails.live[side] = true;
                trails.last[side] = tip;
            }
            trails.acc[side] += (tip - trails.last[side]).length();
            trails.last[side] = tip;
            while trails.acc[side] >= STEP {
                trails.acc[side] -= STEP;
                let c = trails.cursor[side];
                trails.pos[side][c] = tip;
                trails.age[side][c] = 0.0;
                trails.feed[side][c] = feed;
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
        tf.scale = Vec3::splat((PUFF_R0 + (PUFF_R1 - PUFF_R0) * f) * size_j);
        // Billboard about the quad's normal (+Z) so the puff always faces the
        // viewer.
        if let Some(cam) = cam {
            let to_cam = cam - tf.translation;
            if to_cam.length_squared() > 1e-6 {
                tf.rotation = Quat::from_rotation_arc(Vec3::Z, to_cam.normalize());
            }
        }
        if let Some(mut m) = puff_mats.get_mut(&mat.0) {
            m.params.color.w = PUFF_ALPHA * strength * alpha_j * (1.0 - f).powf(1.4);
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
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn wing_feed_is_off_at_cruise_alpha_and_full_when_pulled() {
        // Level cruise: a few degrees of alpha, subsonic — no vapor.
        assert_eq!(wing_feed(2.0, 0.6), 0.0);
        // A hard pull feeds it fully.
        assert_eq!(wing_feed(20.0, 0.6), 1.0);
        // Between the thresholds it ramps.
        let mid = wing_feed((AOA0 + AOA1) / 2.0, 0.6);
        assert!(mid > 0.0 && mid < 1.0, "expected a ramp, got {mid}");
    }

    #[test]
    fn wing_feed_picks_up_near_the_sound_barrier() {
        // Little alpha, but crossing the barrier still condenses off the tips:
        // full at the band's peak, ramping up on the way in and gone once past.
        assert_eq!(wing_feed(0.0, MACH_C), 1.0);
        let mid = wing_feed(0.0, MACH_C - MACH_W / 2.0);
        assert!(mid > 0.0 && mid < 1.0, "expected a ramp, got {mid}");
        assert_eq!(wing_feed(0.0, MACH_C + MACH_W), 0.0);
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
}
