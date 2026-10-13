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
//! - `propulsion/set-running` — light the engine after IC
//! - `fcs/{elevator,aileron,rudder}-cmd-norm`, `fcs/throttle-cmd-norm`,
//!   `fcs/pitch-trim-cmd-norm`, `fcs/flap-cmd-norm`,
//!   `fcs/{left,right}-brake-cmd-norm` — pilot commands
//!
//! Read by the app:
//! - `position/{lat-gc-deg,long-gc-deg,h-sl-ft}` — world position
//! - `attitude/{phi,theta,psi}-rad` and `-deg` — body attitude
//! - `velocities/{vc-kts,h-dot-fps,p-rad_sec,q-rad_sec,r-rad_sec}` — airspeed & rates
//! - `gear/wow` — weight-on-wheels
//! - `fcs/flap-pos-deg`, `fcs/{elevator,aileron,rudder}-cmd-norm`,
//!   `fcs/throttle-cmd-norm`, `fcs/pitch-trim-cmd-norm` — surface feedback
//! - `propulsion/engine[0]/engine-rpm` — engine rpm

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
}
