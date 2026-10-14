//! Minimal closed-loop autopilot for the c172p: ground roll -> rotate -> climb
//! -> level cruise. Drives the FDM's `fcs/*-cmd-norm` channels and reports the
//! current mission phase.
//!
//! Shared by the `takeoff` example and the game, so the 3D view flies the exact
//! same law that was validated headlessly.

use flight_model::FlightModel;

const R2D: f64 = 180.0 / std::f64::consts::PI;

// Mission profile.
const CRUISE_ALT_FT: f64 = 3000.0;
const V_ROTATE_KT: f64 = 55.0; // liftoff / rotate speed
const V_TARGET_KT: f64 = 110.0; // cruise speed
const CLIMB_PITCH_DEG: f64 = 9.0;
const MAX_BANK_DEG: f64 = 15.0;

// Surface sign convention, read off c172p.xml and confirmed by step response:
//   +elevator-cmd -> nose DOWN, +aileron-cmd -> roll RIGHT, +rudder-cmd -> yaw LEFT.
// Every loop below is written against these.

/// Surface commands, normalized to [-1, 1] (throttle in [0, 1]).
pub struct Cmds {
    pub throttle: f64,
    pub elevator: f64,
    pub aileron: f64,
    pub rudder: f64,
    /// Gear selector: down for the roll and rotate, up once airborne. Written as
    /// `gear/gear-cmd-norm` (1 = down, 0 = up); ignored by fixed-gear aircraft.
    pub gear_down: bool,
}

/// Advance the outer loop one step: read the FDM state and return the surface
/// commands plus the current phase ("roll" / "rotate" / "climb" / "cruise").
pub fn step(fdm: &dyn FlightModel) -> (Cmds, &'static str) {
    let alt = fdm.get("position/h-sl-ft");
    let vc = fdm.get("velocities/vc-kts");
    let vs = fdm.get("velocities/h-dot-fps") * 60.0; // fpm
    let theta = fdm.get("attitude/theta-deg");
    let phi = fdm.get("attitude/phi-deg");
    let psi = fdm.get("attitude/psi-deg");
    let q = fdm.get("velocities/q-rad_sec") * R2D; // deg/s
    let p = fdm.get("velocities/p-rad_sec") * R2D;
    let r = fdm.get("velocities/r-rad_sec") * R2D;
    // WOW is the reliable ground cue: the CG sits a few feet above the runway at
    // rest, so AGL alone reads "airborne" while still on the ground.
    let airborne = fdm.get("gear/wow") < 0.5;

    // ---- Outer loop: decide throttle + target pitch ------------------------
    let (throttle, theta_cmd, phase) = if !airborne {
        if vc < V_ROTATE_KT {
            (1.0, 0.0, "roll")
        } else {
            (1.0, 8.0, "rotate")
        }
    } else if alt < CRUISE_ALT_FT - 50.0 {
        (1.0, CLIMB_PITCH_DEG, "climb")
    } else {
        (
            clampf(0.5 + 0.02 * (V_TARGET_KT - vc), 0.0, 1.0),
            // Hold altitude: pitch proportional to altitude error, damped by VS.
            clampf(0.02 * (CRUISE_ALT_FT - alt) - 0.02 * vs, -5.0, 5.0),
            "cruise",
        )
    };

    // ---- Inner loops: surface commands -------------------------------------
    // Elevator: track pitch attitude (nose-up = negative cmd), damped by rate.
    let elevator = clampf(-0.08 * (theta_cmd - theta) + 0.03 * q, -1.0, 1.0);

    // Aileron: bank to hold heading 0 (bank command from heading error).
    let bank_cmd = if airborne {
        clampf(-1.5 * wrap180(psi), -MAX_BANK_DEG, MAX_BANK_DEG)
    } else {
        0.0
    };
    let aileron = clampf(0.08 * (bank_cmd - phi) - 0.02 * p, -1.0, 1.0);

    // Rudder: nosewheel steering on the ground, light yaw damper in air.
    // +rud = yaw left, so a positive heading error (nose right of 0) needs
    // positive rudder; the damper does the same on yaw rate.
    let rudder = if airborne {
        clampf(0.03 * r, -0.3, 0.3)
    } else {
        clampf(0.05 * wrap180(psi), -1.0, 1.0)
    };

    (
        Cmds {
            throttle,
            elevator,
            aileron,
            rudder,
            // Gear down on the ground, up as soon as it lifts off.
            gear_down: !airborne,
        },
        phase,
    )
}

/// Write `cmds` to the FDM's control channels (clears pitch trim).
pub fn apply(fdm: &mut dyn FlightModel, cmds: &Cmds) {
    fdm.set("fcs/throttle-cmd-norm", cmds.throttle);
    fdm.set("fcs/elevator-cmd-norm", cmds.elevator);
    fdm.set("fcs/pitch-trim-cmd-norm", 0.0);
    fdm.set("fcs/aileron-cmd-norm", cmds.aileron);
    fdm.set("fcs/rudder-cmd-norm", cmds.rudder);
    fdm.set("gear/gear-cmd-norm", if cmds.gear_down { 1.0 } else { 0.0 });
}

fn clampf(x: f64, lo: f64, hi: f64) -> f64 {
    x.max(lo).min(hi)
}

/// Wrap an angle in degrees to (-180, 180].
fn wrap180(mut d: f64) -> f64 {
    d = (d + 180.0).rem_euclid(360.0) - 180.0;
    if d <= -180.0 {
        d + 360.0
    } else {
        d
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn wrap180_keeps_in_range_angles_unchanged() {
        assert_eq!(wrap180(0.0), 0.0);
        assert_eq!(wrap180(90.0), 90.0);
        assert_eq!(wrap180(-90.0), -90.0);
    }

    #[test]
    fn wrap180_maps_the_half_turn_to_positive_180() {
        // The range is (-180, 180]: both ±180 collapse onto +180.
        assert_eq!(wrap180(180.0), 180.0);
        assert_eq!(wrap180(-180.0), 180.0);
    }

    #[test]
    fn wrap180_folds_multi_turn_angles() {
        assert_eq!(wrap180(181.0), -179.0);
        assert_eq!(wrap180(-181.0), 179.0);
        assert_eq!(wrap180(370.0), 10.0);
        assert_eq!(wrap180(-370.0), -10.0);
    }

    #[test]
    fn clampf_bounds_both_ends() {
        assert_eq!(clampf(5.0, -1.0, 1.0), 1.0);
        assert_eq!(clampf(-5.0, -1.0, 1.0), -1.0);
        assert_eq!(clampf(0.3, -1.0, 1.0), 0.3);
    }
}
