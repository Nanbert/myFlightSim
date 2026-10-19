//! Helicopter arcade dynamics.
//!
//! The realistic rotor path is the AH-1S momentum-theory model. Arcade mode
//! keeps the same 6-DoF integrator, engine, gear and property bus, but replaces
//! the rotor/aero force resolution with a small set of game rules:
//!
//! * vertical speed is commanded directly, with a proportional lift controller
//!   (`vz_target` comes from the auto-hover state machine or the collective);
//! * horizontal velocity is damped in the NED frame, so a released stick settles
//!   into a stable hover instead of drifting;
//! * roll/pitch are attitude commands with rate damping, so releasing the cyclic
//!   auto-levels the aircraft while still allowing coordinated manoeuvres;
//! * yaw is a rate command.
//!
//! This is deliberately not a rotor model: no inflow, flapping, stall or tail
//! rotor authority. It is the "point it and it goes, release it and it settles"
//! layer that makes the Apache flyable on a keyboard.

use crate::atmos::G0;
use crate::state::{add, cross, scale, Inertia, State, Vec3};

/// Attitude-loop stiffness (1/s). Higher = snappier levelling.
const ATT_K: f64 = 2.8;
/// Rate loop time constant (s).
const TAU_ROT: f64 = 0.22;
/// Vertical-speed loop time constant (s).
const TAU_V: f64 = 0.75;
/// Horizontal velocity damping time constant (s). Longer = more drift.
const TAU_H: f64 = 1.6;
/// Full-cyclic attitude targets, radians.
const MAX_ROLL: f64 = 0.52;
const MAX_PITCH: f64 = 0.38;
/// Yaw rate at full pedal, rad/s.
const RATE_R: f64 = 0.85;
/// Maximum climb/descent rate commanded by the collective, ft/s.
pub const MAX_MANUAL_VZ: f64 = 12.0;

/// Everything the force closure needs that isn't in the trial [`State`].
pub struct Cfg {
    pub elevator: f64,
    pub aileron: f64,
    pub rudder: f64,
    pub collective: f64,
    pub mass: f64,
    pub balance: bool,
    pub on_ground: bool,
    /// Desired vertical speed, ft/s, positive up. Ignored when `lift` is false.
    pub vz_target: f64,
    /// Whether the vertical controller may produce lift this step. False when
    /// parked with auto-hover off, so gravity + the gear settle the aircraft.
    pub lift: bool,
}

/// Body force and moment for the arcade helicopter at trial state `s`.
pub fn force_moment(
    cfg: &Cfg,
    s: &State,
    inertia: &Inertia,
    ground_force: Vec3,
    ground_moment: Vec3,
) -> (Vec3, Vec3) {
    let (phi, theta, _) = s.euler();
    let [p, q, r] = s.omega;

    let (p_cmd, q_cmd, r_cmd) = if cfg.balance {
        // Attitude command: full stick asks for a bank/pitch, no stick asks for
        // level. The rate loop makes the response smooth and prevents PIO.
        let roll_target = (MAX_ROLL * cfg.aileron).clamp(-MAX_ROLL, MAX_ROLL);
        let pitch_target = (-MAX_PITCH * cfg.elevator).clamp(-MAX_PITCH, MAX_PITCH);
        let p_rate = (ATT_K * (roll_target - phi)).clamp(-2.0, 2.0);
        let q_rate = (ATT_K * (pitch_target - theta)).clamp(-1.60, 1.60);
        let r_rate = (-RATE_R * cfg.rudder).clamp(-RATE_R, RATE_R);
        (p_rate, q_rate, r_rate)
    } else {
        // Direct rate command, still lightly damped by the rate loop.
        (
            (1.25 * cfg.aileron).clamp(-1.6, 1.6),
            (-1.10 * cfg.elevator).clamp(-1.4, 1.4),
            (-RATE_R * cfg.rudder).clamp(-RATE_R, RATE_R),
        )
    };

    let wd = [
        (p_cmd - p) / TAU_ROT,
        (q_cmd - q) / TAU_ROT,
        (r_cmd - r) / TAU_ROT,
    ];
    let jw = inertia.mul(s.omega);
    let moment = add(inertia.mul(wd), cross(s.omega, jw));

    // --- vertical controller -------------------------------------------------
    // NED down is +z, so up is -z. Lift acts along the body's world-up vector,
    // which is why a bank/pitch also produces translation.
    let v_ned = s.body_to_ned(s.vel);
    let hdot_up = -v_ned[2];
    let up_body = s.ned_to_body([0.0, 0.0, -1.0]);
    let lift_accel = (cfg.vz_target - hdot_up) / TAU_V;
    let lift_force = if cfg.lift {
        (cfg.mass * (G0 + lift_accel)).max(0.0)
    } else {
        0.0
    };
    let mut force = scale(up_body, lift_force);

    // --- horizontal damping --------------------------------------------------
    // Damp in NED so the hover is stable even when the fuselage is pitched; the
    // pilot can still translate by holding a small attitude.
    let damp_ned = [
        -cfg.mass * v_ned[0] / TAU_H,
        -cfg.mass * v_ned[1] / TAU_H,
        0.0,
    ];
    force = add(force, s.ned_to_body(damp_ned));

    // Keep the gear reaction: it is what lets the arcade helicopter touch down
    // and settle instead of sinking through the runway.
    let _ = cfg.collective;
    let _ = cfg.on_ground;
    (add(force, ground_force), add(moment, ground_moment))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::state::{Inertia, State};

    fn aircraft() -> (Cfg, State, Inertia) {
        (
            Cfg {
                elevator: 0.0,
                aileron: 0.0,
                rudder: 0.0,
                collective: 0.5,
                mass: 300.0,
                balance: true,
                on_ground: false,
                vz_target: 0.0,
                lift: true,
            },
            State::default(),
            Inertia::new(10_000.0, 30_000.0, 30_000.0, 0.0, 0.0, 0.0),
        )
    }

    #[test]
    fn hover_holds_vertical_speed() {
        let (cfg, s, inertia) = aircraft();
        let (f, m) = force_moment(&cfg, &s, &inertia, [0.0; 3], [0.0; 3]);
        // Body up is -z, so a level hover lifts along body -z at ~mg.
        assert!(f[2] < 0.0, "lift should be up: {f:?}");
        assert!((f[2] + cfg.mass * G0).abs() < 1.0, "{f:?}");
        assert!(m.iter().all(|v| v.abs() < 1e-9), "{m:?}");
    }

    #[test]
    fn banked_attitude_commands_translate_and_return_level() {
        let (mut cfg, mut s, inertia) = aircraft();
        cfg.aileron = 1.0;
        let (_, m) = force_moment(&cfg, &s, &inertia, [0.0; 3], [0.0; 3]);
        assert!(m[0] > 0.0, "right aileron should roll right: {m:?}");

        // Simulate the attitude response, then centre the stick and check the
        // moment pulls the bank back toward zero.
        s.quat = crate::euler_to_quat(0.3, 0.0, 0.0);
        cfg.aileron = 0.0;
        let (_, m2) = force_moment(&cfg, &s, &inertia, [0.0; 3], [0.0; 3]);
        assert!(m2[0] < 0.0, "level command should roll left: {m2:?}");
    }
}
