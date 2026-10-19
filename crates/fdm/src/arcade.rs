//! Arcade flight dynamics — the F-22's "game" mode.
//!
//! The realistic path flies the airframe the model describes: aero tables, a
//! rate-command fly-by-wire, stall departures and all. This module replaces that
//! with the handful of rules an arcade jet runs on, so the F-22 handles like an
//! Ace Combat aircraft — point the nose and the jet goes, crisp rates, and no
//! stall or spin to fall into.
//!
//! It is not a separate [`FlightModel`](flight_model::FlightModel): it only hands
//! a different force/moment pair to the same RK4 integrator, so the property bus,
//! the atmosphere, the engine and the gear all carry over untouched. `Fdm` selects
//! it with `fcs/arcade-enable`; the realistic path is unchanged.
//!
//! Two ideas do all the work:
//!
//! - **Rate command as a moment.** Rather than compute an aerodynamic moment from
//!   a stick deflection, ask for the body rate directly and synthesise the moment
//!   that produces it. The equations of motion solve `ω̇ = J⁻¹(M − ω×(Jω))`, so
//!   feeding `M = J·ω̇_des + ω×(Jω)` with `ω̇_des = (ω_cmd − ω)/TAU_ROT` gives
//!   exactly that first-order rate response — and it holds at every RK4 stage,
//!   because the closure recomputes `M` from the trial `ω`.
//! - **Velocity pinned to the nose.** A lateral/normal force drives the body
//!   `(v, w)` velocity components to zero, so the velocity vector tracks the nose
//!   instead of lagging behind it. That is what removes angle of attack, and with
//!   it the stall and the spin: the jet always flies exactly where it points.
//!
//! The only aero kept is the airframe's own *axial* force (drag), so thrust has
//! something to balance and there is a top speed. Everything lateral/normal is
//! dropped — that is the whole trick.

use crate::atmos::G0;
use crate::state::{add, cross, Inertia, State, Vec3};

/// Body rates (rad/s) commanded at full stick, and the time constants (s) the
/// response lags them by. Roll is fast and crisp; pitch is slower nose-down than
/// nose-up, as on the real fly-by-wire; yaw is a light coordinated turn, not a
/// rudder authority the pilot would have to manage.
pub const RATE_P: f64 = 4.0;
pub const RATE_Q_UP: f64 = 1.2;
pub const RATE_Q_DN: f64 = 0.9;
pub const RATE_R: f64 = 0.5;
/// Time constant for the commanded body rates (s). Well inside RK4's stability
/// limit at dt = 1/120 (which is ~0.003 s), so the response is a clean lag.
pub const TAU_ROT: f64 = 0.15;
/// Time constant for pinning the velocity to the nose (s). The "point and go":
/// the flight path swings onto the nose this quickly.
pub const TAU_V: f64 = 0.30;
/// Airspeed (kt) below which the pinning force is off, so it cannot fight the
/// gear or the takeoff roll on the ground.
pub const ALIGN_MIN_KT: f64 = 50.0;
/// Cap on the pinning force, in g, so a wild state can't fling the jet. High
/// enough that a hard pull at 400 kt (which needs ~15 g to swing the flight path
/// onto the nose) still fully aligns; the cap only bites on an already-wild state.
const ALIGN_MAX_G: f64 = 20.0;

/// The per-step arcade inputs that don't come from the trial state. Snapshotted
/// before the integration, since the closure can't borrow the model.
pub struct Cfg {
    pub elevator: f64,
    pub aileron: f64,
    pub rudder: f64,
    pub thrust: f64,
    pub mass: f64,
    /// Whether to pin the velocity to the nose this step — airborne and fast
    /// enough, so the gear and the nosewheel steering own the jet on the ground.
    pub align: bool,
}

/// The body force and moment the arcade rules produce at trial state `s`.
/// `aero_axial` is the airframe's own axial (drag) force, kept so thrust has
/// something to balance; `ground_*` is the landing-gear reaction, passed through
/// untouched so the jet can still taxi, take off and land.
pub fn force_moment(
    cfg: &Cfg,
    s: &State,
    aero_axial: f64,
    inertia: &Inertia,
    ground_force: Vec3,
    ground_moment: Vec3,
) -> (Vec3, Vec3) {
    // Commanded body rates, in the airframe's own sign convention: +aileron rolls
    // right, +elevator pitches nose-down, +rudder yaws nose-left.
    let p_cmd = RATE_P * cfg.aileron;
    let k_q = if cfg.elevator < 0.0 {
        RATE_Q_UP
    } else {
        RATE_Q_DN
    };
    let q_cmd = -k_q * cfg.elevator;
    let r_cmd = -RATE_R * cfg.rudder;

    let wd = [
        (p_cmd - s.omega[0]) / TAU_ROT,
        (q_cmd - s.omega[1]) / TAU_ROT,
        (r_cmd - s.omega[2]) / TAU_ROT,
    ];
    // M = J·ω̇_des + ω×(Jω), so the equations of motion yield ω̇ = ω̇_des exactly.
    let jw = inertia.mul(s.omega);
    let moment = add(inertia.mul(wd), cross(s.omega, jw));

    // Pin the velocity to the nose: drive (v, w) to zero, capped so it can never
    // dominate.
    let gate = if cfg.align { 1.0 } else { 0.0 };
    let cap = ALIGN_MAX_G * cfg.mass * G0;
    let v_force = (-cfg.mass * s.vel[1] * gate / TAU_V).clamp(-cap, cap);
    let w_force = (-cfg.mass * s.vel[2] * gate / TAU_V).clamp(-cap, cap);
    let force = [aero_axial + cfg.thrust, v_force, w_force];

    (add(force, ground_force), add(moment, ground_moment))
}
