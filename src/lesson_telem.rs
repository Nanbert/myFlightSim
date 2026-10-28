//! One frame's flight readouts, shared by every graded-lesson meter.
//!
//! The meters' `tick` functions are pure and testable; this is the superset of
//! every field any of them reads, sampled once from the live aircraft. Each
//! lesson starts from [`Telem::sample`] and fills in only the fields its own
//! drill adds (the runway test, the ILS deviations, the MCP windows), so the
//! per-lesson mapping is no longer copied nine times.

use bevy::prelude::Vec3;

use crate::autopilot::{AtMode, LatMode, VertMode};
use crate::sim::AircraftState;

/// The raw aircraft readouts every meter may look at, plus the derived and
/// AP/MCP fields a lesson adds. Unused fields stay at [`Default`].
#[derive(Clone, Copy, Debug, Default)]
pub struct Telem {
    pub on_ground: bool,
    pub gs_kts: f64,
    pub cas_kts: f64,
    /// Altitude, feet (MSL for the jet lessons; the field sits at sea level).
    pub alt_ft: f64,
    /// AGL above the field, feet — the landing lesson's gate input.
    pub agl_ft: f64,
    pub vs_fpm: f64,
    pub pitch_deg: f64,
    pub roll_deg: f64,
    pub heading_deg: f64,
    /// World position — the runway/centreline offsets read off it.
    pub pos: Vec3,
    pub throttle: f64,
    pub flap_deg: f64,
    /// Achieved gear position: 1 down, 0 up.
    pub gear_pos: f64,
    pub reverse_pos: f64,
    /// Derived: wheels on the pavement of the lesson's runway.
    pub on_runway: bool,
    /// Autopilot state (the jet callout lessons).
    pub ap: bool,
    pub lat: LatMode,
    pub vert: VertMode,
    pub app_armed: bool,
    pub at: AtMode,
    /// The MCP windows the frame was flown with.
    pub mcp_hdg: f64,
    pub mcp_alt: f64,
    pub mcp_vs: f64,
    pub mcp_ias: f64,
    /// ILS deviations, degrees (+ right / + above); zero outside the volume.
    pub loc_dev_deg: f64,
    pub gs_dev_deg: f64,
    /// Along-course distance to the threshold, metres.
    pub dist_m: f64,
}

impl Telem {
    /// The raw aircraft fields, with everything else at its default. Each
    /// step only adds what its lesson derived (runway, ILS, AP modes, MCP).
    pub fn sample(aircraft: &AircraftState) -> Self {
        Self {
            on_ground: aircraft.on_ground,
            gs_kts: aircraft.gs_kts,
            cas_kts: aircraft.cas_kts,
            alt_ft: aircraft.alt_ft,
            // The airfields sit at sea level, so MSL is AGL here.
            agl_ft: aircraft.alt_ft,
            vs_fpm: aircraft.vs_fpm,
            pitch_deg: aircraft.pitch_deg,
            roll_deg: aircraft.roll_deg,
            heading_deg: aircraft.heading_deg,
            pos: aircraft.pos,
            throttle: aircraft.throttle,
            flap_deg: aircraft.flap_deg,
            gear_pos: aircraft.gear_pos,
            reverse_pos: aircraft.reverse_pos,
            ..Default::default()
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sample_copies_the_raw_aircraft_readouts() {
        let aircraft = AircraftState {
            on_ground: true,
            gs_kts: 12.0,
            cas_kts: 11.0,
            alt_ft: 340.0,
            vs_fpm: -2.0,
            pitch_deg: 3.0,
            roll_deg: 4.0,
            heading_deg: 5.0,
            pos: Vec3::new(1.0, 2.0, 3.0),
            throttle: 0.6,
            flap_deg: 15.0,
            gear_pos: 1.0,
            reverse_pos: 0.2,
            ..Default::default()
        };
        let t = Telem::sample(&aircraft);
        assert!(t.on_ground);
        assert_eq!(
            (t.gs_kts, t.cas_kts, t.alt_ft, t.vs_fpm),
            (12.0, 11.0, 340.0, -2.0)
        );
        assert_eq!((t.pitch_deg, t.roll_deg, t.heading_deg), (3.0, 4.0, 5.0));
        assert_eq!(t.pos, Vec3::new(1.0, 2.0, 3.0));
        assert_eq!(
            (t.throttle, t.flap_deg, t.gear_pos, t.reverse_pos),
            (0.6, 15.0, 1.0, 0.2)
        );
        // The airfields sit at sea level, so the sample's AGL is the MSL alt.
        assert_eq!(t.agl_ft, 340.0);
        // Everything a lesson derives itself stays off until it fills it in.
        assert!(!t.on_runway);
        assert_eq!((t.loc_dev_deg, t.gs_dev_deg, t.dist_m), (0.0, 0.0, 0.0));
        assert_eq!((t.vert, t.at, t.lat), (VertMode::Off, AtMode::Off, LatMode::Off));
    }
}
