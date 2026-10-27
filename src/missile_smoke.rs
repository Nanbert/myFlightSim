//! Missile exhaust smoke: the long white column an AMRAAM-class round drags
//! across the sky, and the shorter one an incoming round leaves behind.
//!
//! `combat.rs` puts a [`Trail`] on the round it launches and nothing else: this
//! module owns emission, motion and fade, so the missile code stays about
//! guidance and the smoke stays in one place. Every puff is its own billboard
//! quad with its own material, exactly like the afterburner plume (`fx.rs`) and
//! the wingtip vapor (`vapor.rs`) — a *stream of overlapping soft discs* rather
//! than a tube, so the column can fray at the edges and every puff boils on its
//! own clock.
//!
//! Three things make the trail read the way the footage does:
//!
//! - **Emission is spaced, not timed.** The round accelerates from an eject to
//!   over a kilometre a second, so a timed emission would leave the head of the
//!   column sparse (fresh, small puffs far apart) and the tail dense. Laying one
//!   puff every [`TrailSpec::step`] metres instead keeps the spacing — and so
//!   the coverage — the same from the rail to the last puff.
//! - **Puffs outlive the motor by design.** A puff lives [`TrailSpec::life`]
//!   seconds (eight for the BVR round), so the column keeps growing after the
//!   grain burns out and hangs in the air long after the missile has gone —
//!   at 900 m/s average that is a six-to-seven kilometre streak.
//! - **The trail is painted, not parented.** A puff is dropped into world space
//!   and only inherits a few per cent of the round's velocity, so the missile
//!   flies out of its own smoke instead of dragging it along.
//!
//! The puffs are not parented to anything and carry `DespawnOnExit`, so a fresh
//! flight starts with a clean sky even if a round was still burning.

use bevy::prelude::*;
use bevy::render::render_resource::{AsBindGroup, ShaderType};
use bevy::shader::ShaderRef;

use crate::app_state::{unpaused, AppState};
use crate::ui_theme::{blend_material, SORT_TOP_BIAS};

/// How one round's trail behaves. The two rounds differ in every dimension that
/// matters to the eye: the AMRAAM's motor runs twice as long and twice as fast,
/// so its column is long and wide; the incoming round's is short and thin.
#[derive(Clone, Copy)]
pub struct TrailSpec {
    /// Seconds after launch before the motor lights (the eject phase). Nothing
    /// is emitted before it, which is what puts the burst at the nozzle rather
    /// than at the rail.
    pub ignite: f32,
    /// Motor burn time (s) — emission stops with the grain.
    pub burn: f32,
    /// Metres of flight between puffs while burning. Spacing, not rate: the
    /// round accelerates four-fold over the burn, and a timed emission would
    /// leave the head of the column sparse and the tail dense.
    pub step: f32,
    /// Puff lifetime (s). This, not the burn, sets how long the column is.
    pub life: f32,
    /// Puff radius (m) at birth and at death. The birth radius has to be at
    /// least half [`step`](Self::step), or the column reads as a dotted line at
    /// the head where the puffs are still small.
    pub r0: f32,
    pub r1: f32,
    /// Peak opacity of a fresh puff. Kept well under 1: a trail is dozens of
    /// puffs deep and they composite as "over".
    pub alpha: f32,
    /// Puffs thrown out at ignition — the cloud that blooms off the rail.
    pub burst: u32,
    /// Smoke tint: sunlit white for both, the threat a touch greyer so an
    /// inbound round reads as dirtier.
    pub color: Vec3,
}

/// Seconds from the trigger to the motor lighting: the AMRAAM falls off the rail
/// on its eject charge first. `combat.rs` runs the round's arm phase for exactly
/// this long, so the plume, the ignition flash and the camera kick all land on
/// the same frame the smoke starts.
pub const BVR_IGNITE_S: f32 = 0.22;

/// The AIM-120's column: ignites a fifth of a second after the eject, burns for
/// six seconds, and every puff lives eight — long enough that the streak spans
/// six to seven kilometres at the round's cruise speed.
pub const BVR_TRAIL: TrailSpec = TrailSpec {
    ignite: BVR_IGNITE_S,
    burn: 6.0,
    step: 34.0,
    life: 8.0,
    r0: 32.0,
    r1: 78.0,
    alpha: 0.55,
    burst: 12,
    color: Vec3::new(0.95, 0.96, 0.99),
};

/// An incoming IR round: three seconds of motor, a puff life of three, so the
/// trail marks where the shot came from without hiding the jet behind a wall of
/// smoke.
pub const THREAT_TRAIL: TrailSpec = TrailSpec {
    ignite: 0.0,
    burn: 3.0,
    step: 24.0,
    life: 3.0,
    r0: 14.0,
    r1: 34.0,
    alpha: 0.34,
    burst: 6,
    color: Vec3::new(0.90, 0.91, 0.94),
};

/// Put a trail on a round. The component rides the missile entity; when the
/// missile dies the trail stops emitting but the smoke it already laid keeps
/// fading on its own.
#[derive(Component)]
pub struct Trail {
    spec: TrailSpec,
    /// Seconds since launch.
    t: f32,
    /// Where the round was when it was last sampled.
    last: Vec3,
    /// Cleared on the first frame, when there is no previous sample to diff.
    seen: bool,
    /// Set once the motor has lit, which is also when the rail burst fires.
    ignited: bool,
    /// Metres of flight owed, carried across frames so the spacing does not
    /// depend on frame time.
    acc: f32,
}

impl Trail {
    pub fn new(spec: TrailSpec) -> Self {
        Self {
            spec,
            t: 0.0,
            last: Vec3::ZERO,
            seen: false,
            ignited: false,
            acc: 0.0,
        }
    }
}

/// One smoke billboard in world space. Not parented to the round: it hangs in
/// the air where it was dropped.
#[derive(Component)]
struct Puff {
    age: f32,
    life: f32,
    r0: f32,
    r1: f32,
    /// Peak opacity, held for the first third of the puff's life so the head of
    /// the column stays solid while the tail is already thinning out.
    alpha: f32,
    /// Drift (m/s): a few per cent of the round's velocity, plus the ignition
    /// burst's own scatter.
    vel: Vec3,
    mat: Handle<SmokeMaterial>,
}

/// Uniform for `assets/shaders/missile_smoke.wesl`.
#[derive(Clone, Copy, ShaderType, Default)]
struct SmokeParams {
    /// rgb = lit smoke colour, a = peak opacity.
    color: Vec4,
    /// x = time (s), y = per-puff seed, z = rim raggedness, w = shading.
    anim: Vec4,
}

/// Smoke is translucent, not emissive: it takes light and hides the sky behind
/// it. A material per puff is what lets each one fade on its own clock.
#[derive(Asset, TypePath, AsBindGroup, Clone)]
struct SmokeMaterial {
    #[uniform(0)]
    params: SmokeParams,
}

// Top draw bias: the column must sit over the sea and the cloud cards, which
// mis-sort against it because their huge footprints sort by a distant centre.
blend_material!(SmokeMaterial, "shaders/missile_smoke.wesl", SORT_TOP_BIAS);

/// The unit quad every puff shares, plus the running seed counter that keeps the
/// puffs from being clones of one another.
#[derive(Resource)]
struct SmokeQuad {
    mesh: Handle<Mesh>,
    seed: u32,
}

impl SmokeQuad {
    /// Deterministic xorshift: the same launch looks the same in a screenshot
    /// run and on the player's machine, and the sequence needs no extra crate.
    fn next(&mut self) -> f32 {
        self.seed ^= self.seed << 13;
        self.seed ^= self.seed >> 17;
        self.seed ^= self.seed << 5;
        (self.seed >> 8) as f32 / 16_777_216.0
    }
}

pub struct MissileSmokePlugin;

impl Plugin for MissileSmokePlugin {
    fn build(&self, app: &mut App) {
        app.add_plugins(MaterialPlugin::<SmokeMaterial>::default())
            .add_systems(Startup, setup_smoke)
            .add_systems(Update, (emit_trails, fade_puffs).chain().run_if(unpaused));
    }
}

fn setup_smoke(mut commands: Commands, mut meshes: ResMut<Assets<Mesh>>) {
    // `Rectangle` is 2×2, so a puff's transform scale *is* its radius.
    commands.insert_resource(SmokeQuad {
        mesh: meshes.add(Rectangle::new(2.0, 2.0)),
        seed: 0x9e37_79b9,
    });
}

/// Light the motor's smoke: fire the rail burst at ignition, then lay a puff
/// every [`TrailSpec::step`] metres until the grain is out.
fn emit_trails(
    mut commands: Commands,
    time: Res<Time>,
    mut rng: ResMut<SmokeQuad>,
    mut mats: ResMut<Assets<SmokeMaterial>>,
    mut trails: Query<(&mut Trail, &Transform)>,
) {
    let dt = time.delta_secs();
    if dt <= 0.0 {
        return;
    }
    for (mut trail, tf) in &mut trails {
        let pos = tf.translation;
        let spec = trail.spec;
        // The round's own velocity, differenced from last frame's sample. One
        // frame of lag is invisible in smoke, and it keeps the missile code from
        // having to publish anything for this.
        let vel = if trail.seen {
            (pos - trail.last) / dt
        } else {
            Vec3::ZERO
        };
        trail.last = pos;
        trail.seen = true;
        trail.t += dt;

        if !trail.ignited {
            if trail.t < spec.ignite {
                continue;
            }
            trail.ignited = true;
            // The rail burst: a hot cloud thrown off the nozzle at light-up.
            for _ in 0..spec.burst {
                let a = rng.next() * std::f32::consts::TAU;
                let up = rng.next() * 1.4 - 0.4;
                let out = Vec3::new(a.cos(), up, a.sin()) * (5.0 + rng.next() * 9.0);
                let seed = rng.next();
                puff(
                    &mut commands,
                    &rng.mesh,
                    &mut mats,
                    pos,
                    vel * 0.04 + out,
                    2.0,
                    5.0,
                    16.0,
                    0.42,
                    spec.color,
                    seed,
                );
            }
        }
        if trail.t > spec.ignite + spec.burn {
            continue;
        }

        // Emission is by distance flown: a fixed spacing keeps the column
        // continuous whether the round is doing 300 m/s or 1100.
        trail.acc += vel.length() * dt;
        let step = spec.step;
        let speed = vel.length();
        let fwd = if speed > 1.0 { vel / speed } else { Vec3::Z };
        while trail.acc >= step {
            trail.acc -= step;
            // Scatter across the axis and jitter the size: a chain of identical
            // discs reads as beads, a chain of ragged ones reads as smoke. The
            // scatter is what lets the column wander and widen instead of
            // stacking on one line.
            let sway = Vec3::new(
                rng.next() - 0.5,
                (rng.next() - 0.5) * 0.7 + 0.25,
                rng.next() - 0.5,
            ) * (step * 0.45);
            let size = 0.78 + rng.next() * 0.5;
            let life = spec.life * (0.85 + rng.next() * 0.3);
            let seed = rng.next();
            puff(
                &mut commands,
                &rng.mesh,
                &mut mats,
                pos - fwd * 2.0 + sway,
                vel * 0.03 + sway * 0.5,
                life,
                spec.r0 * size,
                spec.r1 * size,
                spec.alpha,
                spec.color,
                seed,
            );
        }
    }
}

#[allow(clippy::too_many_arguments)]
fn puff(
    commands: &mut Commands,
    mesh: &Handle<Mesh>,
    mats: &mut Assets<SmokeMaterial>,
    at: Vec3,
    vel: Vec3,
    life: f32,
    r0: f32,
    r1: f32,
    alpha: f32,
    color: Vec3,
    seed: f32,
) {
    let mat = mats.add(SmokeMaterial {
        params: SmokeParams {
            color: Vec4::new(color.x, color.y, color.z, alpha),
            anim: Vec4::new(0.0, seed, 0.2, 1.0),
        },
    });
    commands.spawn((
        Puff {
            age: 0.0,
            life,
            r0,
            r1,
            alpha,
            vel,
            mat: mat.clone(),
        },
        Mesh3d(mesh.clone()),
        MeshMaterial3d(mat),
        Transform::from_translation(at).with_scale(Vec3::splat(r0)),
        DespawnOnExit(AppState::Flying),
    ));
}

/// Grow, drift, fade — and keep the quad facing the camera. The column is left
/// to expand in place: the puffs inherit only a few per cent of the round's
/// velocity and are damped hard, so the jet flies out of its own smoke.
fn fade_puffs(
    mut commands: Commands,
    time: Res<Time>,
    mut mats: ResMut<Assets<SmokeMaterial>>,
    cam: Query<
        &Transform,
        (
            With<Camera3d>,
            Without<crate::cloud_camera::CloudCamera>,
            Without<Puff>,
        ),
    >,
    mut puffs: Query<(Entity, &mut Puff, &mut Transform)>,
) {
    let dt = time.delta_secs();
    let eye = cam.single().ok().map(|t| t.translation);
    for (entity, mut p, mut tf) in &mut puffs {
        p.age += dt;
        let k = p.age / p.life;
        if k >= 1.0 {
            commands.entity(entity).despawn();
            continue;
        }

        tf.translation += p.vel * dt;
        p.vel *= (1.0 - 1.6 * dt).max(0.0);
        // Hot gas rises: a slow buoyancy that opens the column out as it ages.
        p.vel.y += 0.35 * dt;

        // Fast early growth, then a slow billow — the shape a puff of exhaust
        // takes as it loses its momentum to the air.
        let r = p.r0 + (p.r1 - p.r0) * k.powf(0.55);
        tf.scale = Vec3::splat(r);

        if let Some(eye) = eye {
            let to_cam = eye - tf.translation;
            if to_cam.length_squared() > 1e-6 {
                tf.rotation = Quat::from_rotation_arc(Vec3::Z, to_cam.normalize());
            }
        }

        // Solid for the first quarter of the life, then a linear fade: the head
        // of the column stays opaque while the tail is already thinning out.
        // Fade out right against the lens too: the chase camera sits twenty-odd
        // metres behind the jet and flies straight through its own launch smoke,
        // and an unfaded puff there is a white wall across half the frame.
        let near = eye.map_or(1.0, |e| {
            let t = ((e.distance(tf.translation) - 10.0) / 45.0).clamp(0.0, 1.0);
            t * t * (3.0 - 2.0 * t)
        });
        let a = p.alpha * ((1.0 - k) * 1.6).clamp(0.0, 1.0) * near;
        if let Some(mut m) = mats.get_mut(&p.mat) {
            m.params.color.w = a;
            m.params.anim.x = p.age;
            // Raggedness grows with age: tight at the nozzle, torn once it has
            // been in the air a while.
            m.params.anim.z = 0.2 + 0.6 * k;
        }
    }
}
