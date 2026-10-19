//! Per-aircraft data. Everything that distinguishes one airframe from another
//! lives in a `const Aircraft` (see [`c172`]); the shared evaluator in
//! `crate::aero`, the ground model in `crate::gear`, and the integrator in
//! `crate::state` never change. This is the seam that makes a turbofan 737 or a
//! thrust-vectoring F-22 a new data file, not new code — their engines, control
//! laws, and gear layouts land as further structs here.

pub mod ah1s;
pub mod b737;
pub mod c172;
pub mod f22;

use crate::aero::Aerodynamics;
use crate::engine::PowerplantSpec;
use crate::fcs::ControlSystem;
use crate::gear::Gear;
use crate::rotor::RotorSpec;
use crate::state::{Inertia, Vec3};

/// Pound-mass -> slug (JSBSim's `lbtoslug`).
pub const LB_TO_SLUG: f64 = 1.0 / 32.174049;
pub const INCH_TO_FT: f64 = 1.0 / 12.0;

/// A discrete mass at a structural-frame position (inches from the datum).
pub struct PointMass {
    pub weight_lb: f64,
    pub pos_in: Vec3,
}

/// Fuel in the wing tanks. The tank point masses in [`MassSpec::points`] get
/// their `weight_lb` overridden by the current contents, so burning fuel
/// genuinely moves the CG and the inertia. Only airframes that declare a
/// `FuelSpec` carry consumable fuel (`propulsion/tank-*-contents-lbs`).
pub struct FuelSpec {
    /// Indexes into `MassSpec::points` — the left and right wing tanks.
    pub tank_points: [usize; 2],
    /// Tank capacity, per tank (lb) — the model's `<capacity>`.
    pub capacity_lb: f64,
    /// Contents at spawn, per tank (lb) — the model's `<contents>`.
    pub initial_lb: f64,
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

/// A complete airframe definition — data only.
pub struct Aircraft {
    pub name: &'static str,
    pub wing_area_sqft: f64,
    pub wing_span_ft: f64,
    pub wing_chord_ft: f64,
    pub mass: MassSpec,
    pub aero: Aerodynamics,
    /// The control system: direct gains (c172p) or a rate-command FBW (F-22).
    pub control: ControlSystem,
    pub power: PowerplantSpec,
    /// Landing-gear contact points (from `<ground_reactions>`).
    pub gear: &'static [Gear],
    /// Whether the gear retracts in flight. Fixed-gear aircraft (c172p) ignore
    /// the gear command and are always down; retractable ones (F-22) drive
    /// `gear/gear-pos-norm` from the pilot's `gear/gear-cmd-norm`.
    pub gear_retractable: bool,
    /// Rotors (main first, then tail). Empty for fixed-wing aircraft.
    pub rotors: &'static [RotorSpec],
    /// CG height above the runway to seed the IC at when parked (ft). Seeding at
    /// the wrong height buries the gear/skids and the ground springs fling the
    /// aircraft, so this is per-aircraft (e.g. the c172p reset height, or a
    /// helicopter resting on its skids).
    pub rest_h_ft: f64,
    /// Whether the airframe has flight spoilers (the 737's speedbrake): the
    /// `fcs/speedbrake-cmd-norm` command is otherwise ignored.
    pub has_speedbrake: bool,
    /// Maximum reverse thrust **per engine** (lbf), applied only while the
    /// wheels carry weight. 0 = no thrust reverse.
    pub max_reverse_lbs: f64,
    /// Consumable wing-tank fuel, if the airframe models it (c172p). Turbines
    /// and tankless airframes leave this `None` and fly at the spec mass.
    pub fuel: Option<FuelSpec>,
}

impl Aircraft {
    /// Point weights after an optional fuel override: the fuel-spec tank points
    /// carry the given contents (lb) instead of the model's static `<contents>`.
    pub fn point_weights(&self, fuel: Option<&[f64; 2]>) -> Vec<f64> {
        let mut w: Vec<f64> = self.mass.points.iter().map(|p| p.weight_lb).collect();
        if let (Some(spec), Some(contents)) = (self.fuel.as_ref(), fuel) {
            for (i, lb) in spec.tank_points.iter().zip(contents) {
                if *i < w.len() {
                    w[*i] = *lb;
                }
            }
        }
        w
    }

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
        self.loaded_mass(None).cg_in
    }

    /// Everything the dynamics need about the loaded airframe: total mass, CG
    /// and the inertia about that CG — with the fuel tanks at `fuel` contents
    /// (or the spec defaults when `None`).
    pub fn loaded_mass(&self, fuel: Option<&[f64; 2]>) -> LoadedMass {
        let weights = self.point_weights(fuel);
        let mut w = self.mass.empty_weight_lb;
        let mut cg = [
            w * self.mass.cg_in[0],
            w * self.mass.cg_in[1],
            w * self.mass.cg_in[2],
        ];
        for (p, lb) in self.mass.points.iter().zip(&weights) {
            w += lb;
            cg[0] += lb * p.pos_in[0];
            cg[1] += lb * p.pos_in[1];
            cg[2] += lb * p.pos_in[2];
        }
        let cg = [cg[0] / w, cg[1] / w, cg[2] / w];

        // Inertia about the loaded CG: the empty tensor plus each point mass's
        // parallel-axis contribution. Products of inertia are taken from the
        // empty tensor (the c172's loading is near symmetric about the
        // roll/pitch planes).
        let e = self.mass.empty_inertia;
        let (mut ixx, mut iyy, mut izz) = (e[0], e[1], e[2]);
        for (p, lb) in self.mass.points.iter().zip(&weights) {
            let m = lb * LB_TO_SLUG;
            let dx = (p.pos_in[0] - cg[0]) * INCH_TO_FT;
            let dy = (p.pos_in[1] - cg[1]) * INCH_TO_FT;
            let dz = (p.pos_in[2] - cg[2]) * INCH_TO_FT;
            ixx += m * (dy * dy + dz * dz);
            iyy += m * (dx * dx + dz * dz);
            izz += m * (dx * dx + dy * dy);
        }
        LoadedMass {
            mass_slug: w * LB_TO_SLUG,
            cg_in: cg,
            inertia: Inertia::new(ixx, iyy, izz, e[3], e[4], e[5]),
        }
    }

    /// Inertia about the loaded CG: the empty inertia plus each point mass's
    /// parallel-axis contribution. Products of inertia are taken from the empty
    /// tensor (the c172's loading is near symmetric about the roll/pitch planes).
    pub fn inertia(&self) -> Inertia {
        self.loaded_mass(None).inertia
    }
}

/// A loaded mass snapshot: what `Fdm` caches between fuel-state changes.
pub struct LoadedMass {
    pub mass_slug: f64,
    pub cg_in: Vec3,
    pub inertia: Inertia,
}
