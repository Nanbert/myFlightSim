//! Landing-gear ground reaction, transcribed from JSBSim's `FGLGear`.
//!
//! Each contact point is a spring-damper along the ground normal plus a tyre
//! friction force in the ground plane. The strut, damping, Pacejka side-force
//! and brake-coefficient formulas and constants are copied verbatim from
//! JSBSim (`src/models/FGLGear.cpp`); the friction is then applied *explicitly*
//! and saturated to the friction cone, where JSBSim instead resolves all
//! contacts jointly with Lagrange multipliers and a projected Gauss-Seidel
//! solve (`FGAccelerations::CalculateFrictionForces`). For a single aircraft on
//! a flat runway the difference is negligible.
//!
//! The reaction is a pure function of the aircraft state (no cross-step
//! memory), so it slots into the per-stage RK4 force closure in `Fdm::advance`.
//! Two JSBSim details are dropped: the runway bumpiness noise (our ground is a
//! perfect plane; the default bumpiness is 0 anyway) and the per-gear wheel
//! spin state (we don't integrate wheel rotational inertia — rolling resistance
//! and braking are applied as a bounded longitudinal force instead).

use crate::state::{add, cross, dot, norm, scale, State, Vec3};

const D2R: f64 = std::f64::consts::PI / 180.0;
const R2D: f64 = 180.0 / std::f64::consts::PI;

/// Feet per second per knot (matches `lib.rs`'s conversion, kept local so the
/// ground reaction stays a self-contained module).
const KT_TO_FPS: f64 = 1.68781;

/// Speed below which friction ramps in smoothly (ft/s), so the explicit model
/// doesn't chatter between + and - around zero at the 120 Hz step. High speed
/// recovers the full Coulomb magnitude; near zero the force fades, which is
/// what lets a braked aircraft actually hold still instead of bang-banging.
const V_EPS: f64 = 1.0;

// Pacejka Magic Formula constants (FGLGear.cpp: Stiffness / Shape / Curvature).
const TIRE_STIFFNESS: f64 = 0.06;
const TIRE_SHAPE: f64 = 2.8;
const TIRE_CURVATURE: f64 = 1.03;

/// How a contact point carries load and friction.
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum ContactType {
    /// A wheel on a strut. The normal force comes from spring compression;
    /// friction is direction-dependent (rolling + Pacejka cornering).
    Bogey,
    /// A rigid point (skid / wingtip): isotropic Coulomb friction, rarely
    /// touching except on a crash or extreme attitude.
    Structure,
}

/// Which brake channel drives this contact's longitudinal friction.
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum BrakeGroup {
    None,
    Left,
    Right,
}

/// Speed-attenuated nosewheel steering, the jets' rule: the wheel keeps the
/// full `steer_max_deg` up to `lo_kt`, then the authority tapers linearly to
/// `floor_deg` at `hi_kt` — rudder-pedal-only steering at taxi speed, as on a
/// real 737 where the tiller is a low-speed tool.
#[derive(Clone, Copy)]
pub struct SteerFade {
    pub lo_kt: f64,
    pub hi_kt: f64,
    pub floor_deg: f64,
}

/// The steering deflection limit at `gs_kt` ground speed: `steer_max_deg`
/// faded by [`SteerFade`] when the gear declares one, full authority otherwise.
fn steer_limit(gear: &Gear, gs_kt: f64) -> f64 {
    match gear.steer_fade {
        None => gear.steer_max_deg,
        Some(f) => {
            let floor = f.floor_deg.min(gear.steer_max_deg);
            let t = ((f.hi_kt - gs_kt) / (f.hi_kt - f.lo_kt)).clamp(0.0, 1.0);
            floor + (gear.steer_max_deg - floor) * t
        }
    }
}

/// One ground contact point: geometry, strut, and friction data. Positions are
/// structural-frame inches from the datum, matching the model's `<contact>`
/// blocks.
pub struct Gear {
    pub pos_in: Vec3,
    /// Strut spring rate (lb/ft).
    pub spring: f64,
    /// Compression damping (lb/ft/s).
    pub damp: f64,
    /// Rebound damping (lb/ft/s); equal to `damp` when the model gives none.
    pub damp_rebound: f64,
    pub static_f: f64,
    pub dyn_f: f64,
    pub roll_f: f64,
    pub contact: ContactType,
    /// Max steering deflection (deg); 0 for a fixed wheel.
    pub steer_max_deg: f64,
    /// Steering-authority fade with speed; `None` keeps full deflection at any
    /// speed (the fixed-gear airframes and the F-22).
    pub steer_fade: Option<SteerFade>,
    pub brake: BrakeGroup,
}

/// Ground reaction summed over all contacts, in body axes about the CG.
#[derive(Clone, Copy, Default)]
pub struct Ground {
    pub wow: bool,
    pub force_body: Vec3,
    pub moment_body: Vec3,
}

/// Sum the ground reaction from every gear at state `s`.
///
/// `arms_ft` are the CG->contact-point arms in body axes (precomputed from
/// [`Gear::pos_in`] via `struct_to_body`), aligned with `gears`. `rudder_cmd`
/// steers the nosewheel; `brake` is `[left, right]` normalized. `dt` bounds the
/// first-contact damping spike.
pub fn ground_force(
    s: &State,
    gears: &[Gear],
    arms_ft: &[Vec3],
    rudder_cmd: f64,
    brake: [f64; 2],
    dt: f64,
) -> Ground {
    // Ground normal ("up") in body axes: NED down is -z, so up is (0,0,-1).
    let up_body = s.ned_to_body([0.0, 0.0, -1.0]);
    // Ground speed (kt) drives the steering-authority fade.
    let v_ned_cg = s.body_to_ned(s.vel);
    let gs_kt = v_ned_cg[0].hypot(v_ned_cg[1]) / KT_TO_FPS;
    let mut out = Ground::default();

    for (gear, &r) in gears.iter().zip(arms_ft) {
        // Contact-point position and penetration. Ground is the NED plane
        // z = 0, so height above ground is -z; below the ground => contact.
        let contact_ned = add(s.pos, s.body_to_ned(r));
        let height = -contact_ned[2];
        if height >= 0.0 {
            continue;
        }
        let pen = -height;

        // Contact-point velocity (body): v_cm + omega x r. Compression speed is
        // its NED-down component — sinking pushes the contact down, i.e.
        // v_ned[2] > 0, matching JSBSim's `-vGroundWhlVel(eZ)`.
        let v_body = add(s.vel, cross(s.omega, r));
        let v_ned = s.body_to_ned(v_body);
        let mut cs = v_ned[2];
        // A stage that first pierces the ground in one step would otherwise see
        // an unbounded compression speed and spike the damper; cap it at what
        // the penetration depth allows over one step.
        let cap = pen / dt;
        if cs.abs() > cap {
            cs = cs.signum() * cap;
        }

        // Vertical strut force, push-only (JSBSim clamps `StrutForce` <= 0, i.e.
        // magnitude >= 0), along the ground normal. Linear damping; the rebound
        // branch uses its own coefficient (equal here).
        let b = if cs >= 0.0 {
            gear.damp
        } else {
            gear.damp_rebound
        };
        let n = (gear.spring * pen + b * cs).max(0.0);

        // Ground-plane basis. `roll` is the wheel heading (the body forward axis
        // projected onto the ground), rotated by the steer angle about vertical;
        // `side` is perpendicular. A positive steer (left rudder) turns the
        // wheel left, which yaws the aircraft left.
        let fwd_h = horizontal([1.0, 0.0, 0.0], up_body);
        let roll = if gear.steer_max_deg != 0.0 {
            let theta = rudder_cmd * steer_limit(gear, gs_kt) * D2R;
            add(
                scale(fwd_h, theta.cos()),
                scale(cross(up_body, fwd_h), theta.sin()),
            )
        } else {
            fwd_h
        };
        let side = cross(up_body, roll);

        let v_roll = dot(v_body, roll);
        let v_side = dot(v_body, side);

        // Longitudinal force: rolling resistance plus brake, bounded by
        // JSBSim's `BrakeFCoeff = rolling + brake*(static - rolling)`.
        // Lateral force: Pacejka cornering for a tyre, isotropic Coulomb for a
        // skid.
        let (f_roll, f_side) = match gear.contact {
            ContactType::Bogey => {
                let brake_pos = match gear.brake {
                    BrakeGroup::None => 0.0,
                    BrakeGroup::Left => brake[0],
                    BrakeGroup::Right => brake[1],
                };
                let brake_f = gear.roll_f + brake_pos * (gear.static_f - gear.roll_f);
                let f_roll = -brake_f * n * (v_roll / V_EPS).tanh();
                let f_side = if v_roll.hypot(v_side) > 1e-3 {
                    let slip = -v_side.atan2(v_roll.abs()) * R2D;
                    let stiff = TIRE_STIFFNESS * slip;
                    let coeff = gear.static_f
                        * (TIRE_SHAPE * (stiff - TIRE_CURVATURE * (stiff - stiff.atan())).atan())
                            .sin();
                    coeff * n
                } else {
                    0.0
                };
                (f_roll, f_side)
            }
            ContactType::Structure => (
                -gear.dyn_f * n * (v_roll / V_EPS).tanh(),
                -gear.dyn_f * n * (v_side / V_EPS).tanh(),
            ),
        };

        // Friction acts in the ground plane; the strut force along the normal.
        // The moment uses the uncompressed arm: strut travel is < 6% of the arm
        // and RK4 already samples trial states, so the offset is immaterial.
        let force = add(
            add(scale(up_body, n), scale(roll, f_roll)),
            scale(side, f_side),
        );
        out.force_body = add(out.force_body, force);
        out.moment_body = add(out.moment_body, cross(r, force));
        out.wow = true;
    }

    out
}

/// Project `v` onto the plane perpendicular to `n`, normalized. Falls back to
/// body-forward if `v` is parallel to `n` (an implausible ground attitude).
fn horizontal(v: Vec3, n: Vec3) -> Vec3 {
    let h = add(v, scale(n, -dot(v, n)));
    let m = norm(h);
    if m < 1e-9 {
        [1.0, 0.0, 0.0]
    } else {
        scale(h, 1.0 / m)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn faded_gear() -> Gear {
        Gear {
            pos_in: [0.0; 3],
            spring: 0.0,
            damp: 0.0,
            damp_rebound: 0.0,
            static_f: 0.8,
            dyn_f: 0.5,
            roll_f: 0.02,
            contact: ContactType::Bogey,
            steer_max_deg: 35.0,
            steer_fade: Some(SteerFade {
                lo_kt: 5.0,
                hi_kt: 40.0,
                floor_deg: 7.0,
            }),
            brake: BrakeGroup::None,
        }
    }

    #[test]
    fn steer_fade_holds_full_deflection_at_a_walk() {
        assert_eq!(steer_limit(&faded_gear(), 0.0), 35.0);
        assert_eq!(steer_limit(&faded_gear(), 5.0), 35.0);
    }

    #[test]
    fn steer_fade_tapers_between_the_knees() {
        // Halfway between 5 and 40 kt the authority is halfway between 35
        // and 7 degrees.
        let mid = steer_limit(&faded_gear(), 22.5);
        assert!((mid - 21.0).abs() < 0.01, "mid fade = {mid}");
    }

    #[test]
    fn steer_fade_floors_at_taxi_speed() {
        assert_eq!(steer_limit(&faded_gear(), 40.0), 7.0);
        assert_eq!(steer_limit(&faded_gear(), 120.0), 7.0);
    }

    #[test]
    fn no_fade_keeps_full_deflection_at_any_speed() {
        let mut g = faded_gear();
        g.steer_fade = None;
        assert_eq!(steer_limit(&g, 120.0), 35.0);
    }
}
