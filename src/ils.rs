//! The MYFS instrument-landing service: runway 36/18 localizer and glideslope.
//!
//! The world is a flat plane with the runway paint at `y = 0`, so the ILS is a
//! pure piece of geometry over the world position [`Sim::pose`] publishes:
//!
//! * **Localizer** — the centreline of the selected runway. Deviation is the
//!   lateral offset converted to the angular error the vanilla receiver shows:
//!   half scale is 1.25°, which at the far gate (5 NM) is roughly ±1.2 dots.
//! * **Glideslope** — a 3° beam that meets the ground exactly at the threshold,
//!   where the approach lights are. Eight-tenths of a degree of "below beam"
//!   is where the amber GLIDESLOPE annunciator lights (see `pfd_sym.wesl`).
//!
//! Course 36 approaches from the south (world +Z), course 18 from the north,
//! and each is only valid inside its approach volume — the beam does not
//! illuminate behind the antenna. `loc_dev_deg`/`gs_dev_deg` are signed:
//! positive localizer is *right of course*, positive glideslope is *above the
//! beam*, which is exactly what the capture laws and the PFD want.
//!
//! The autopilot consumes this once per FDM step, before `advance`.

use bevy::prelude::*;

use crate::airfield::AirfieldSpec;
use crate::sim::Sim;

/// Glideslope angle shared by both runways.
pub const GS_ANGLE_DEG: f64 = 3.0;
/// Up to this far out the beams are radiated; 9300 m ≈ 5 NM.
const GATE_M: f64 = 9300.0;
/// And the beams start at this distance from the threshold (the grass just
/// south of the numbers), so taxiing across the threshold does not slam the
/// deviation to ±90°.
const GATE_IN_M: f64 = 150.0;

/// Half-scale (one dot): the receiver's standard 1.25° deviation width.
pub const LOC_DOT_DEG: f64 = 1.25;
pub const GS_DOT_DEG: f64 = 0.35;

/// The MYFS ILS beams, recomputed each FDM step inside `advance_fdm` and
/// consumed by the autopilot's LOC/GS capture laws.
#[derive(Resource, Debug, Clone, Copy)]
pub struct Ils {
    /// True while the aircraft is inside one of the two approach volumes.
    pub available: bool,
    /// Angular localizer deviation, degrees; + right of course.
    pub loc_dev_deg: f64,
    /// Angular glideslope deviation, degrees; + above the beam.
    pub gs_dev_deg: f64,
    /// Along-course distance to the threshold, metres.
    pub dist_m: f64,
    /// True course of the volume the aircraft is inside (0 = 36, 180 = 18).
    pub course_deg: f64,
}

/// GLIDESLOPE annunciator: this far *below* the flown beam lights the amber
/// word (the real warning band starts around 0.7 dot).
pub const GS_WARN_DEG: f64 = 0.55;

impl Ils {
    pub fn none() -> Self {
        Ils {
            available: false,
            loc_dev_deg: 0.0,
            gs_dev_deg: 0.0,
            dist_m: 0.0,
            course_deg: 0.0,
        }
    }
}

impl Default for Ils {
    fn default() -> Self {
        Self::none()
    }
}

/// World position -> deviation, using the runway geometry in the spec.
/// `z_north = -z_world` north convention comes from [`Sim::pose`].
pub fn compute(sim: &Sim, spec: &AirfieldSpec) -> Ils {
    let (pos, _, _) = sim.pose();
    let x_east = pos.x as f64;
    let z_world = pos.z as f64;
    let h_m = pos.y as f64;

    // The pose is metres; the threshold constants are metres already.
    let gs_rise = GS_ANGLE_DEG.to_radians().tan();

    // Runway 36: antenna at the 18 end, beam lays over the threshold toward the
    // approach gate at world +Z. On course = north = up the runway.
    let d36 = z_world - spec.threshold_z as f64;
    if d36 > 0.0 {
        let dev = beam(x_east, h_m, d36, gs_rise, 0.0);
        if let Some(ils) = dev {
            return ils;
        }
    }

    // Runway 18: approached from the north, antenna at the 36 end. On course =
    // south, so "right of course" is west (−east).
    let d18 = spec.rwy_end_z as f64 - z_world;
    if d18 > 0.0 {
        let dev = beam(-x_east, h_m, d18, gs_rise, 180.0);
        if let Some(ils) = dev {
            return ils;
        }
    }

    Ils::none()
}

/// Localizer + glideslope geometry for one approach. `right_offset` should be
/// the aircraft's offset to the *right of course* in metres and `d` the
/// along-course threshold distance; both courses funnel through here with the
/// sign already fixed.
fn beam(right_offset: f64, h_m: f64, d: f64, gs_rise: f64, course_deg: f64) -> Option<Ils> {
    if !(GATE_IN_M..GATE_M).contains(&d) {
        return None;
    }
    let loc_dev_deg = (right_offset.atan2(d) * (180.0 / std::f64::consts::PI))
        .clamp(-LOC_DOT_DEG * 3.0, LOC_DOT_DEG * 3.0);
    // Height of the beam centreline above the runway plane at this range.
    let beam_m = gs_rise * d;
    let gs_dev_deg = (((h_m - beam_m) / d).atan() * (180.0 / std::f64::consts::PI))
        .clamp(-GS_DOT_DEG * 4.0, GS_DOT_DEG * 4.0);
    Some(Ils {
        available: true,
        loc_dev_deg,
        gs_dev_deg,
        dist_m: d,
        course_deg,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn beam_at_the_gate_is_on_scale() {
        // 3° slope: at 3000 m the beam is ~157 m up. Fly 60 m to the right and
        // 10 m low and check both deviations read sensibly.
        let ils = beam(60.0, 147.0, 3000.0, 0.052408, 0.0).unwrap();
        assert!(ils.available);
        // 60 m / 3000 m ≈ 1.15° Right.
        assert!(
            ils.loc_dev_deg > 1.0 && ils.loc_dev_deg < 1.3,
            "{:?}",
            ils.loc_dev_deg
        );
        // 10 m in 3000 m ≈ 0.19° low (negative).
        assert!(
            ils.gs_dev_deg < -0.15 && ils.gs_dev_deg > -0.25,
            "{:?}",
            ils.gs_dev_deg
        );
        assert_eq!(ils.dist_m, 3000.0);
    }

    #[test]
    fn behind_the_antenna_and_beyond_the_gate_are_invalid() {
        assert!(
            beam(0.0, 100.0, 100.0, 0.052408, 0.0).is_none(),
            "too close"
        );
        assert!(
            beam(0.0, 100.0, 10_000.0, 0.052408, 0.0).is_none(),
            "too far"
        );
        assert!(beam(0.0, 100.0, -500.0, 0.052408, 0.0).is_none(), "behind");
    }
}
