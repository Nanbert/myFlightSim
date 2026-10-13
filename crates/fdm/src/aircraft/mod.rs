//! Per-aircraft data. Everything that distinguishes one airframe from another
//! lives in a `const Aircraft` (see [`c172`]); the shared evaluator in
//! `crate::aero`, the ground model in `crate::gear`, and the integrator in
//! `crate::state` never change. This is the seam that makes a turbofan 737 or a
//! thrust-vectoring F-22 a new data file, not new code — their engines, control
//! laws, and gear layouts land as further structs here.

pub mod c172;

use crate::aero::Aerodynamics;
use crate::engine::{PistonSpec, PropSpec};
use crate::gear::Gear;
use crate::state::{Inertia, Vec3};

/// Pound-mass -> slug (JSBSim's `lbtoslug`).
pub const LB_TO_SLUG: f64 = 1.0 / 32.174049;
pub const INCH_TO_FT: f64 = 1.0 / 12.0;

/// A discrete mass at a structural-frame position (inches from the datum).
pub struct PointMass {
    pub weight_lb: f64,
    pub pos_in: Vec3,
}

/// Mass & balance. `empty_inertia` and `cg_in` come straight from the model's
/// `<mass_balance>`; `points` (pilot, fuel, ...) are added about the CG at load.
pub struct MassSpec {
    pub empty_weight_lb: f64,
    /// `[ixx, iyy, izz, ixy, ixz, iyz]`, slug·ft².
    pub empty_inertia: [f64; 6],
    pub cg_in: Vec3,
    pub points: &'static [PointMass],
}

/// Flight-control-surface geometry: gains from a normalized command to radians,
/// and deflection limits. Mirrors the `<flight_control>` block.
pub struct Fcs {
    pub elevator_gain: f64,
    pub elevator_lim_deg: (f64, f64),
    pub aileron_gain: f64,
    pub aileron_lim_deg: (f64, f64),
    pub rudder_gain: f64,
    pub rudder_lim_deg: (f64, f64),
    pub flap_max_deg: f64,
    /// Seconds for the flaps to travel from 0 to full.
    pub flap_time_s: f64,
}

/// A complete airframe definition — data only.
pub struct Aircraft {
    pub name: &'static str,
    pub wing_area_sqft: f64,
    pub wing_span_ft: f64,
    pub wing_chord_ft: f64,
    pub mass: MassSpec,
    pub aero: Aerodynamics,
    pub fcs: Fcs,
    pub piston: PistonSpec,
    pub prop: PropSpec,
    /// Landing-gear contact points (from `<ground_reactions>`).
    pub gear: &'static [Gear],
}

impl Aircraft {
    /// Total weight (lb): empty plus every point mass.
    pub fn total_weight_lb(&self) -> f64 {
        self.mass.empty_weight_lb + self.mass.points.iter().map(|p| p.weight_lb).sum::<f64>()
    }

    /// Total mass in slugs.
    pub fn mass_slug(&self) -> f64 {
        self.total_weight_lb() * LB_TO_SLUG
    }

    /// CG of the loaded aircraft (structural-frame inches): the empty CG plus
    /// each point mass, weight-weighted. This is what the moments and inertia
    /// are taken about, not the empty CG.
    pub fn loaded_cg_in(&self) -> Vec3 {
        let mut w = self.mass.empty_weight_lb;
        let mut m = [
            w * self.mass.cg_in[0],
            w * self.mass.cg_in[1],
            w * self.mass.cg_in[2],
        ];
        for p in self.mass.points {
            w += p.weight_lb;
            m[0] += p.weight_lb * p.pos_in[0];
            m[1] += p.weight_lb * p.pos_in[1];
            m[2] += p.weight_lb * p.pos_in[2];
        }
        [m[0] / w, m[1] / w, m[2] / w]
    }

    /// Inertia about the loaded CG: the empty inertia plus each point mass's
    /// parallel-axis contribution. Products of inertia are taken from the empty
    /// tensor (the c172's loading is near symmetric about the roll/pitch planes).
    pub fn inertia(&self) -> Inertia {
        let e = self.mass.empty_inertia;
        let cg = self.loaded_cg_in();
        let (mut ixx, mut iyy, mut izz) = (e[0], e[1], e[2]);
        for p in self.mass.points {
            let m = p.weight_lb * LB_TO_SLUG;
            let dx = (p.pos_in[0] - cg[0]) * INCH_TO_FT;
            let dy = (p.pos_in[1] - cg[1]) * INCH_TO_FT;
            let dz = (p.pos_in[2] - cg[2]) * INCH_TO_FT;
            ixx += m * (dy * dy + dz * dz);
            iyy += m * (dx * dx + dz * dz);
            izz += m * (dx * dx + dy * dy);
        }
        Inertia::new(ixx, iyy, izz, e[3], e[4], e[5])
    }
}
