//! Pure-Rust flight dynamics model — the wasm-clean replacement for JSBSim.
//!
//! **P0 skeleton.** This implements the [`FlightModel`] seam so the app can
//! select it (`FDM_BACKEND=rust`), but computes nothing: every `get` reads 0.0
//! and `step` is a no-op, so the aircraft simply sits at the origin. The physics
//! lands in later phases — 6-DoF integration and an ISA atmosphere, then
//! per-aircraft aero (`coeff` + evaluator), engines, FCS, and trim.
//!
//! Kept free of Bevy, the filesystem, and threads so it compiles to `wasm32`.

use flight_model::FlightModel;

/// A flight model instance for one aircraft.
pub struct Fdm {
    aircraft: String,
}

impl Fdm {
    /// Construct an (empty) model for `aircraft` (e.g. `"c172p"`).
    pub fn new(aircraft: &str) -> Self {
        Fdm {
            aircraft: aircraft.to_string(),
        }
    }

    /// The aircraft this model was built for.
    pub fn aircraft(&self) -> &str {
        &self.aircraft
    }
}

impl FlightModel for Fdm {
    fn step(&mut self) {}

    fn apply_ic(&mut self) {}

    fn get(&self, _name: &str) -> f64 {
        0.0
    }

    fn set(&mut self, _name: &str, _value: f64) {}
}
