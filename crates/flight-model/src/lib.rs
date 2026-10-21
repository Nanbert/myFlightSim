//! The seam between the app and whatever computes flight dynamics.
//!
//! A flight model is driven entirely through a *named-property bus* — the same
//! shape JSBSim's property tree has. The app writes control commands into it,
//! steps it, and reads instruments back out; nothing else crosses the boundary.
//! Both backends (the JSBSim C++ model in `jsbsim-sys`, and the pure-Rust one in
//! `fdm`) implement this, so `view`, `input`, the autopilot, and the HUD are
//! identical whichever one is loaded.
//!
//! # Canonical property contract
//!
//! Property *names* are part of the contract: a backend must expose the names
//! below, or the app silently reads zeros. They follow JSBSim's spelling so the
//! JSBSim backend needs no translation.
//!
//! Written by the app:
//! - `ic/vc-kts`, `ic/h-sl-ft`, `ic/psi-true-deg` — initial conditions (before `apply_ic`)
//! - `ic/{phi,theta}-deg`, `ic/{p,q,r}-rad_sec`, `ic/beta-deg` — optional attitude,
//!   body-rate and sideslip ICs (default 0 = level, unaccelerated, slip-free)
//! - `propulsion/set-running` — light the engine after IC
//! - `fcs/{elevator,aileron,rudder}-cmd-norm`, `fcs/throttle-cmd-norm`,
//!   `fcs/pitch-trim-cmd-norm`, `fcs/flap-cmd-norm`,
//!   `fcs/{left,right}-brake-cmd-norm` — pilot commands
//! - `gear/gear-cmd-norm` — gear selector (1 = down); `gear/gear-pos-norm` —
//!   seat the achieved position directly (a reset convenience; a no-op on
//!   fixed-gear models)
//!
//! Read by the app:
//! - `position/{lat-gc-deg,long-gc-deg,h-sl-ft}` — world position
//! - `attitude/{phi,theta,psi}-rad` and `-deg` — body attitude
//! - `velocities/{vc-kts,h-dot-fps,p-rad_sec,q-rad_sec,r-rad_sec}` — airspeed & rates
//! - `accelerations/n-pilot-y-norm` — lateral load factor (0 coordinated);
//!   drives the turn coordinator's slip ball
//! - `gear/wow` — weight-on-wheels
//! - `fcs/flap-pos-deg`, `fcs/{elevator,aileron,rudder}-cmd-norm`,
//!   `fcs/throttle-cmd-norm`, `fcs/pitch-trim-cmd-norm`,
//!   `fcs/{roll,yaw}-trim-cmd-norm` — surface & trim feedback
//! - `propulsion/engine[0]/engine-rpm` — engine rpm
//! - `propulsion/engine[0]/rotor-rpm`, `propulsion/engine[1]/rotor-rpm` — main
//!   and tail rotor rpm (rotorcraft only; the app spins the rotor meshes from these)
//! - `fcs/collective-cmd-norm` — collective lever (rotorcraft only)
//! - `velocities/vg-fps`, `velocities/v-north/v-east-fps` — ground velocity (wind
//!   drift included; the FPM marker needs this, not the airspeed)
//! - `atmosphere/wind-{north,east,down}-fps` — ambient wind, NED, ft/s. Both
//!   directions of the bus: the app writes it to set the weather, and reads it
//!   back for the windsock/HUD. A backend that doesn't model wind can ignore
//!   the write and always answer 0.
//! - `velocities/vtrue-kts`, `velocities/mach` — air-relative true airspeed and
//!   Mach (the pitot never reads the wind, even though the vehicle drifts)
//! - `propulsion/engine[0]/fuel-flow-pph` — lb/hr from the last piston step
//! - `propulsion/tank-{0,1}-contents-lbs` — wing-tank contents (lb); only the
//!   Rust c172p publishes these (it carries a `FuelSpec`). JSBSim's own tanks
//!   live under its native `propulsion/tank[N]/...`
//! - `propulsion/tank-capacity-lbs` — total usable capacity, for gauge scaling
//! - `propulsion/engine[0]/has-fuel` — 1.0 only when the backend publishes the
//!   fuel bookkeeping above (the app gates its fuel UI on this)
//! - `propulsion/ignition-enable` — write: spark present (master + mags folded
//!   together); read: the same, for the status line
//! - `propulsion/engine[0]/starter-cmd-norm` — write: the held starter crank,
//!   0..1. Cranks against the prop drag; fires at ~200 rpm with spark+fuel
//! - `propulsion/set-running` — write: the spawn shortcut that lights and spools
//!   a piston engine (JSBSim's convention). Refused when the tanks are dry
//! - `propulsion/engine[0]/set-stopped` — write: any non-zero shuts the running
//!   engine down (the ENGINE = COLD toggle's mirror action)
//! - `propulsion/engine[0]/running` — read: 1.0 while the engine makes power
//! - `propulsion/refuel-cmd` — write: any non-zero fills the wing tanks to
//!   capacity (the apron pump; never relights a stopped engine)

/// A flight dynamics model driven through a named-property bus.
pub trait FlightModel: Send + Sync {
    /// Advance one fixed timestep. The step size is fixed at construction.
    fn step(&mut self);

    /// Apply the currently-programmed initial conditions (`ic/*`) and settle for
    /// one zero-length frame, so the first `step` starts from a trimmed state.
    fn apply_ic(&mut self);

    /// Read a named signal. Returns `0.0` for names the model doesn't expose.
    fn get(&self, name: &str) -> f64;

    /// Write a named signal. Names the model doesn't expose are ignored.
    fn set(&mut self, name: &str, value: f64);

    /// Solve a wings-level steady trim at `vc_kts` (calibrated airspeed) and
    /// `alt_ft`, then seat the model on it, so a subsequent open-loop `step`
    /// holds the point. Default is a no-op for backends that can't trim.
    fn trim(&mut self, vc_kts: f64, alt_ft: f64) {
        let _ = (vc_kts, alt_ft);
    }

    /// Whether the model implements thrust vectoring (a `fcs/tvc-enable` switch).
    /// Default `false`; the F-22's fly-by-wire overrides it. The app uses this
    /// only to decide whether to show the TVC control on the HUD.
    fn has_tvc(&self) -> bool {
        false
    }

    /// Whether the gear retracts, driven by `gear/gear-cmd-norm` and reported
    /// through `gear/gear-pos-norm` (1 = down, 0 = up). Default `false` (fixed
    /// gear); the app uses this only to decide whether to show the gear state on
    /// the HUD and whether to animate it.
    fn has_retractable_gear(&self) -> bool {
        false
    }

    /// Whether the model is a rotorcraft (has a rotor). Default `false`; the
    /// pure-Rust `fdm` overrides it for the AH-1S. The app uses this to show the
    /// rotor HUD line and to keep the fixed-wing autopilot off.
    fn has_rotor(&self) -> bool {
        false
    }

    /// CG height above the runway to seed the IC at when parked (ft). Default is
    /// the c172p value (reset00.xml); the Rust `fdm` overrides it per aircraft so
    /// e.g. a helicopter parks on its skids instead of burying them.
    fn rest_h_ft(&self) -> f64 {
        4.7
    }
}

/// Wrap an angle in degrees to `(-180, 180]`, the heading-error convention both
/// autopilots use, so the difference crosses zero instead of jumping at the
/// 0°/360° seam.
///
/// Shared by the app's autopilot (`src/autopilot.rs`), the JSBSim autopilot
/// (`jsbsim-sys/src/autopilot.rs`) and the HUD; three copies that agreed on
/// everything except the ±180° seam used to live side by side.
pub fn wrap180(mut deg: f64) -> f64 {
    deg = (deg + 180.0).rem_euclid(360.0) - 180.0;
    if deg <= -180.0 {
        deg + 360.0
    } else {
        deg
    }
}

#[cfg(test)]
mod tests {
    use super::wrap180;

    #[test]
    fn wraps_to_the_semi_open_interval() {
        assert_eq!(wrap180(0.0), 0.0);
        assert_eq!(wrap180(90.0), 90.0);
        assert_eq!(wrap180(-90.0), -90.0);
        assert_eq!(wrap180(180.0), 180.0);
        assert_eq!(wrap180(-180.0), 180.0);
        assert_eq!(wrap180(181.0), -179.0);
        assert_eq!(wrap180(-181.0), 179.0);
        assert_eq!(wrap180(370.0), 10.0);
        assert_eq!(wrap180(-370.0), -10.0);
    }
}
