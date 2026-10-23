//! Data-driven airfield specs. The `Airfield` resource holds the active
//! airport's geometry so scoring, ILS, and taxi modules query it at runtime
//! instead of reading MYFS constants from `aerodrome`.
//!
//! Coordinate convention: +X = east, -Z = north, psi = 0 is north. Runway 36
//! threshold sits at +Z, the 18 at -Z, the roll runs north (-Z).

use bevy::prelude::*;

/// A taxiway connector leaving the runway's east edge for the parallel way.
#[derive(Clone, Debug)]
pub struct Connector {
    /// Connector centre-line z.
    pub z: f32,
    /// A rapid exit (chicane) rather than a threshold stub.
    pub rapid: bool,
}

/// An apron slab.
#[derive(Clone, Copy, Debug)]
pub struct ApronSlab {
    pub x: f32,
    pub z: f32,
    pub width: f32,
    pub depth: f32,
}

/// A parking stand.
#[derive(Clone, Copy, Debug)]
pub struct Stand {
    pub x: f32,
    pub z: f32,
    /// Heading at the stand, degrees (270 = nose west toward the taxiway).
    pub heading: f32,
}

/// A procedural airport, described once and consumed by both the scenery
/// spawner and the geometry-query modules (landing, ILS, taxi, lessons).
#[derive(Clone, Debug)]
pub struct AirfieldSpec {
    pub icao: &'static str,
    pub name: &'static str,
    pub class: &'static str,
    /// Primary runway number (northbound, "36").
    pub rwy_id: &'static str,
    /// Reciprocal runway number (southbound, "18").
    pub rwy_id_rec: &'static str,
    /// Runway dimensions, metres.
    pub rwy_length: f32,
    pub rwy_width: f32,
    /// Z of the 36 threshold (north end of the paved surface in world +Z).
    pub threshold_z: f32,
    /// Z of the 18 threshold.
    pub rwy_end_z: f32,
    /// Parallel taxiway centre-line and width.
    pub taxi_x: f32,
    pub taxi_width: f32,
    /// Parallel taxiway z extents (south first, north second).
    pub taxi_z0: f32,
    pub taxi_z1: f32,
    /// Connectors from runway to parallel taxiway.
    pub connectors: Vec<Connector>,
    /// Half-depth of each connector stub (metres). Total width = 2 × this.
    pub connector_half_depth: f32,
    pub apron: Option<ApronSlab>,
    pub stand: Option<Stand>,
    /// Runway holding-position markings (FAA-ordered: solid on the holding
    /// side, dashed over the runway edge).
    pub hold_dashed_xs: Vec<f32>,
    pub hold_solid_xs: Vec<f32>,
    /// The solid pair's taxiway-side line: the hold-short grader's zero mark.
    pub hold_line_x: f32,
    /// Refuelling pump position and capture radius, if any.
    pub pump: Option<(Vec3, f64)>,
    /// Whether this field radiates an ILS (both courses).
    pub has_ils: bool,
}

impl AirfieldSpec {
    /// Centre of the runway along its length.
    pub fn rwy_center_z(&self) -> f32 {
        (self.threshold_z + self.rwy_end_z) * 0.5
    }

    /// ICAO Annex 14 proportioning: the marking set scales with runway width
    /// off the 150 ft (45.72 m) reference.
    pub fn mark_scale(&self) -> f32 {
        self.rwy_width / 45.72
    }

    /// Whether a world point is on the paved runway surface (with tolerance).
    pub fn on_runway(&self, x: f32, z: f32) -> bool {
        self.runway_lateral(x) <= self.rwy_width * 0.5 + 1.0
            && (self.rwy_end_z - 1.0..=self.threshold_z + 1.0).contains(&z)
    }

    /// Distance from the runway centre-line, metres.
    pub fn runway_lateral(&self, x: f32) -> f32 {
        x.abs()
    }

    /// Distance along the roll from the active threshold, metres.
    pub fn runway_along(&self, heading_deg: f32, z: f32) -> f32 {
        if heading_deg.to_radians().cos() >= 0.0 {
            self.threshold_z - z
        } else {
            z - self.rwy_end_z
        }
    }

    /// Whether a world point is on any paved surface: runway, parallel
    /// taxiway, connectors, apron.
    pub fn on_pavement(&self, x: f32, z: f32) -> bool {
        if self.on_runway(x, z) {
            return true;
        }
        let on_taxiway = (self.taxi_x - self.taxi_width * 0.5
            ..=self.taxi_x + self.taxi_width * 0.5)
            .contains(&x)
            && (self.taxi_z1..=self.taxi_z0).contains(&z);
        if on_taxiway {
            return true;
        }
        let on_connector = self
            .connectors
            .iter()
            .any(|c| (self.rwy_width * 0.5..=self.taxi_x).contains(&x)
                && (z - c.z).abs() <= self.connector_half_depth);
        if on_connector {
            return true;
        }
        if let Some(ref apron) = self.apron {
            if (apron.x - apron.width * 0.5..=apron.x + apron.width * 0.5).contains(&x)
                && (apron.z - apron.depth * 0.5..=apron.z + apron.depth * 0.5).contains(&z)
            {
                return true;
            }
        }
        false
    }

    /// Signed offset from the nearest connector band's centre-line, `None` off
    /// the connector pavement.
    pub fn connector_offset(&self, x: f32, z: f32) -> Option<f32> {
        let half = self.connector_half_depth;
        let band_x = self.rwy_width * 0.5..=(self.taxi_x - half);
        self.connectors.iter().find_map(|c| {
            ((z - c.z).abs() < half && band_x.contains(&x)).then_some(z - c.z)
        })
    }

    // ------------------------------------------------------------------
    // Spec constructors
    // ------------------------------------------------------------------

    /// MYFS: the C172's home airfield. Values match the `aerodrome` constants.
    pub fn myfs() -> Self {
        let threshold_z = 140.0;
        let rwy_end_z = -1060.0;
        AirfieldSpec {
            icao: "MYFS",
            name: "My Flight School",
            class: "3C",
            rwy_id: "36",
            rwy_id_rec: "18",
            rwy_length: 1200.0,
            rwy_width: 30.0,
            threshold_z,
            rwy_end_z,
            taxi_x: 45.0,
            taxi_width: 12.0,
            taxi_z0: 134.0,
            taxi_z1: -1024.0,
            connectors: vec![
                Connector { z: 134.0, rapid: false },
                Connector { z: -1024.0, rapid: false },
                Connector { z: -150.0, rapid: false },
            ],
            connector_half_depth: 6.0,
            apron: Some(ApronSlab {
                x: 111.0,
                z: -150.0,
                width: 120.0,
                depth: 160.0,
            }),
            stand: Some(Stand {
                x: 75.0,
                z: -150.0,
                heading: 270.0,
            }),
            hold_dashed_xs: vec![18.0, 19.5],
            hold_solid_xs: vec![22.0, 23.5],
            hold_line_x: 23.5,
            pump: Some((Vec3::new(90.0, 1.0, -190.0), 12.0)),
            has_ils: true,
        }
    }

    /// ZSNM 石家庄正定国际机场: ICAO 4E, runway 36/18, 3400 m × 45 m.
    /// Parallel taxiway at x=130 (107.5 m from runway edge), 18 m-wide
    /// connectors. Apron starts immediately east of the taxiway.
    pub fn zsnm() -> Self {
        let threshold_z = 140.0;
        let rwy_end_z = threshold_z - 3400.0; // = -3260.0
        let apron_z = -1500.0;
        AirfieldSpec {
            icao: "ZSNM",
            name: "石家庄正定国际机场",
            class: "4E",
            rwy_id: "36",
            rwy_id_rec: "18",
            rwy_length: 3400.0,
            rwy_width: 45.0,
            threshold_z,
            rwy_end_z,
            taxi_x: 130.0,
            taxi_width: 23.0,
            taxi_z0: threshold_z - 6.0,   // = 134.0
            taxi_z1: rwy_end_z + 36.0,    // = -3224.0
            connectors: vec![
                Connector { z: threshold_z - 6.0, rapid: false },      // 134
                Connector { z: threshold_z - 500.0, rapid: true },     // -360
                Connector { z: apron_z, rapid: false },                  // -1500
                Connector { z: threshold_z - 2000.0, rapid: true },    // -1860
                Connector { z: rwy_end_z + 36.0, rapid: false },      // -3224
            ],
            connector_half_depth: 9.0,
            apron: Some(ApronSlab {
                x: 292.0,  // 141.5 (taxiway east edge) + 150.5 (half width)
                z: apron_z,
                width: 300.0,
                depth: 200.0,
            }),
            stand: Some(Stand {
                x: 252.0,  // On the apron, west side
                z: apron_z,
                heading: 270.0,
            }),
            hold_dashed_xs: vec![30.0, 32.0],
            hold_solid_xs: vec![35.0, 37.0],
            hold_line_x: 37.0,
            pump: None,
            has_ils: true,
        }
    }
}

/// The active airfield spec, `None` when the flight runs on a mission map
/// or with an aircraft that has no home field.
#[derive(Resource, Default)]
pub struct Airfield(pub Option<AirfieldSpec>);

/// Resolve the airfield spec for a given aircraft profile name.
pub fn spec_for(name: &str) -> AirfieldSpec {
    match crate::aircraft_profile::normalized(name) {
        "737" => AirfieldSpec::zsnm(),
        _ => AirfieldSpec::myfs(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn myfs_spec_matches_aerodrome_constants() {
        use crate::aerodrome::{
            APRON_Z, CONNECTOR_ZS, HOLD_DASHED_XS, HOLD_LINE_X, HOLD_SOLID_XS, RUNWAY_END_Z,
            RUNWAY_WIDTH, STAND_X, STAND_Z, TAXI_X, TAXI_Z0, TAXI_Z1, THRESHOLD_Z,
        };
        let s = AirfieldSpec::myfs();
        assert_eq!(s.rwy_width, RUNWAY_WIDTH);
        assert_eq!(s.threshold_z, THRESHOLD_Z);
        assert_eq!(s.rwy_end_z, RUNWAY_END_Z);
        assert_eq!(s.taxi_x, TAXI_X);
        assert_eq!(s.taxi_z0, TAXI_Z0);
        assert_eq!(s.taxi_z1, TAXI_Z1);
        assert_eq!(s.hold_line_x, HOLD_LINE_X);
        assert_eq!(s.hold_dashed_xs.as_slice(), HOLD_DASHED_XS);
        assert_eq!(s.hold_solid_xs.as_slice(), HOLD_SOLID_XS);
        assert_eq!(s.apron.unwrap().z, APRON_Z);
        assert_eq!(s.stand.unwrap().x, STAND_X);
        assert_eq!(s.stand.unwrap().z, STAND_Z);
        for (i, cz) in CONNECTOR_ZS.iter().enumerate() {
            assert_eq!(s.connectors[i].z, *cz);
        }
    }

    #[test]
    fn myfs_geometry_queries_match_free_functions() {
        use crate::aerodrome::{
            connector_offset, on_pavement, on_runway, runway_along, runway_lateral,
        };
        let s = AirfieldSpec::myfs();
        for &(x, z) in &[(0.0, 0.0), (14.0, -300.0), (20.0, -300.0), (45.0, 0.0), (111.0, -150.0), (75.0, -150.0), (-100.0, -150.0), (46.0, 500.0)] {
            assert_eq!(
                s.on_runway(x, z),
                on_runway(x, z),
                "on_runway({x},{z})"
            );
            assert_eq!(
                s.on_pavement(x, z),
                on_pavement(x, z),
                "on_pavement({x},{z})"
            );
            assert_eq!(
                s.runway_lateral(x),
                runway_lateral(x),
                "runway_lateral({x})"
            );
            assert_eq!(
                s.connector_offset(x, z),
                connector_offset(x, z),
                "connector_offset({x},{z})"
            );
        }
        for &(hdg, z) in &[(0.0, 140.0), (0.0, -160.0), (180.0, -1060.0), (180.0, -760.0)] {
            assert!(
                (s.runway_along(hdg, z) - runway_along(hdg, z)).abs() < 1e-3,
                "runway_along({hdg},{z})"
            );
        }
    }

    #[test]
    fn zsnm_spec_is_geometrically_sound() {
        let s = AirfieldSpec::zsnm();
        assert_eq!(s.icao, "ZSNM");
        assert_eq!(s.class, "4E");
        assert!((s.rwy_length - 3400.0).abs() < 1e-3);
        assert!((s.rwy_width - 45.0).abs() < 1e-3);
        assert!((s.rwy_end_z - (140.0 - 3400.0)).abs() < 1e-3);
        assert!(s.connectors.len() >= 4);
        assert!(s.has_ils);
        // A point on the runway is on the runway.
        assert!(s.on_runway(0.0, 0.0));
        // Beyond the runway end is not.
        assert!(!s.on_runway(0.0, s.rwy_end_z - 10.0));
        // Past the threshold is not.
        assert!(!s.on_runway(0.0, s.threshold_z + 10.0));
    }
}
