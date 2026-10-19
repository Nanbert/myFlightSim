//! U.S. Standard Atmosphere 1976, matching JSBSim's `FGStandardAtmosphere` for
//! a zero-bias standard day. Units are imperial (feet, slugs, pounds, Rankine)
//! to line up with the JSBSim property values the app already speaks; the view
//! layer converts to SI at the render boundary.
//!
//! Only the standard atmosphere is modelled — no temperature bias or
//! humidity, and the wind lives on the property bus (`atmosphere/wind-*-fps`),
//! not inside this module. That is all `c172p` needs, and it keeps the Rust
//! model honest about what it does.

use crate::table::Table1;

// --- Physical constants (JSBSim's values, so trajectories compare cleanly) ---

const FTTOTM: f64 = 0.3048;
const KGTOSLUG: f64 = 0.06852168;
/// Sea-level gravity used by the *definition* of the standard atmosphere.
pub const G0: f64 = 9.80665 / FTTOTM; // ~32.174 ft/s^2
/// Earth radius used for the geometric <-> geopotential conversion.
const EARTH_RADIUS_FT: f64 = 6356766.0 / FTTOTM;
/// Specific gas constant for dry air, ft*lbf/(slug*R).
const RDRY: f64 = RSTAR / MAIR;
const RSTAR: f64 = 8.31432 * KGTOSLUG / (FTTOTM * FTTOTM * 1.8); // ft*lbf/(R/mol)
const MAIR: f64 = 28.9645 * KGTOSLUG / 1000.0; // slug/mol
/// Ratio of specific heats for air.
const SH_RATIO: f64 = 1.4;

pub const SL_PRESSURE_PSF: f64 = 2116.228;

/// Geopotential altitude (ft) at each table breakpoint.
const GEO_POT_FT: [f64; 9] = [
    0.0,
    36089.2388,
    65616.7979,
    104986.8766,
    154199.4751,
    167322.8346,
    232939.6325,
    278385.8268,
    298556.4304,
];
/// Temperature (deg R) at each breakpoint.
const TEMP_R: [f64; 9] = [
    518.67, 389.97, 389.97, 411.57, 487.17, 487.17, 386.37, 336.5028, 336.5028,
];

const LNG: usize = 8; // number of layers

/// Precomputed lapse rates and pressure breakpoints for the standard day.
#[derive(Clone, Copy, Debug)]
pub struct Isa {
    lapse: [f64; LNG],
    p_break: [f64; LNG],
    t_table: Table1,
}

impl Default for Isa {
    fn default() -> Self {
        Self::new()
    }
}

impl Isa {
    pub fn new() -> Self {
        let mut lapse = [0.0; LNG];
        for b in 0..LNG {
            lapse[b] = (TEMP_R[b + 1] - TEMP_R[b]) / (GEO_POT_FT[b + 1] - GEO_POT_FT[b]);
        }

        let mut p_break = [0.0; LNG];
        p_break[0] = SL_PRESSURE_PSF;
        for b in 0..LNG - 1 {
            let tmb = TEMP_R[b];
            let dh = GEO_POT_FT[b + 1] - GEO_POT_FT[b];
            p_break[b + 1] = if lapse[b] != 0.0 {
                let exp = G0 / (RDRY * lapse[b]);
                let factor = tmb / (tmb + lapse[b] * dh);
                p_break[b] * factor.powf(exp)
            } else {
                p_break[b] * (-G0 * dh / (RDRY * tmb)).exp()
            };
        }

        Isa {
            lapse,
            p_break,
            t_table: Table1::new(&GEO_POT_FT, &TEMP_R),
        }
    }

    /// Layer index whose geopotential span contains `hpot`, clamped to the top.
    fn layer(hpot: f64) -> usize {
        for b in 0..LNG {
            if hpot < GEO_POT_FT[b + 1] {
                return b;
            }
        }
        LNG - 1
    }

    /// Temperature (deg R) at a geometric altitude (ft).
    pub fn temperature(&self, geometric_alt_ft: f64) -> f64 {
        let hpot = geopotential(geometric_alt_ft);
        if hpot >= 0.0 {
            self.t_table.lookup(hpot)
        } else {
            TEMP_R[0] + hpot * self.lapse[0]
        }
    }

    /// Static pressure (psf) at a geometric altitude (ft).
    pub fn pressure(&self, geometric_alt_ft: f64) -> f64 {
        let hpot = geopotential(geometric_alt_ft);
        if hpot < GEO_POT_FT[0] {
            // Below sea level: extrapolate the first layer.
            let tmb = TEMP_R[0];
            let lmb = self.lapse[0];
            let dh = hpot - GEO_POT_FT[0];
            return if lmb != 0.0 {
                let exp = G0 / (RDRY * lmb);
                let factor = tmb / (tmb + lmb * dh);
                self.p_break[0] * factor.powf(exp)
            } else {
                self.p_break[0] * (-G0 * dh / (RDRY * tmb)).exp()
            };
        }
        let b = Self::layer(hpot);
        let tmb = TEMP_R[b];
        let lmb = self.lapse[b];
        let dh = hpot - GEO_POT_FT[b];
        if lmb != 0.0 {
            let exp = G0 / (RDRY * lmb);
            let factor = tmb / (tmb + lmb * dh);
            self.p_break[b] * factor.powf(exp)
        } else {
            self.p_break[b] * (-G0 * dh / (RDRY * tmb)).exp()
        }
    }

    /// Density (slug/ft^3) at a geometric altitude (ft).
    pub fn density(&self, geometric_alt_ft: f64) -> f64 {
        self.pressure(geometric_alt_ft) / (RDRY * self.temperature(geometric_alt_ft))
    }

    /// Speed of sound (ft/s) at a geometric altitude (ft).
    pub fn sound_speed(&self, geometric_alt_ft: f64) -> f64 {
        (SH_RATIO * RDRY * self.temperature(geometric_alt_ft)).sqrt()
    }

    /// Geometric altitude (ft) at which the standard atmosphere has density
    /// `rho`, by bisection on the monotone density profile (JSBSim's
    /// `GetDensityAltitude` — which likewise returns *geometric* altitude). The
    /// turbine thrust tables are indexed by `atmosphere/density-altitude`.
    pub fn density_altitude(&self, rho: f64) -> f64 {
        let (mut lo, mut hi) = (-1000.0_f64, 300_000.0_f64);
        if rho >= self.density(lo) {
            return lo;
        }
        for _ in 0..64 {
            let mid = 0.5 * (lo + hi);
            if self.density(mid) > rho {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        0.5 * (lo + hi)
    }

    /// Calibrated -> true airspeed (ft/s) at a geometric altitude, via the
    /// subsonic pitot relation (JSBSim's `SetVcalibratedKtsIC`). This is what
    /// turns a programmed `ic/vc-kts` into the model's actual TAS.
    pub fn cas_to_tas(&self, vcas_fps: f64, alt_ft: f64) -> f64 {
        let p = self.pressure(alt_ft);
        let a = self.sound_speed(alt_ft);
        let a0 = self.sound_speed(0.0);
        let qc = pitot_total_pressure(vcas_fps / a0, SL_PRESSURE_PSF) - SL_PRESSURE_PSF;
        mach_from_impact_pressure(qc, p) * a
    }

    /// True -> calibrated airspeed (ft/s) at a geometric altitude (JSBSim's
    /// `VcalibratedFromMach`), the inverse of [`Isa::cas_to_tas`].
    pub fn tas_to_cas(&self, vt_fps: f64, alt_ft: f64) -> f64 {
        let p = self.pressure(alt_ft);
        let a = self.sound_speed(alt_ft);
        let a0 = self.sound_speed(0.0);
        let qc = pitot_total_pressure(vt_fps / a, p) - p;
        a0 * mach_from_impact_pressure(qc, SL_PRESSURE_PSF)
    }
}

/// Pitot (stagnation) pressure (psf) at `mach` in static pressure `p` (psf) —
/// the subsonic isentropic branch. JSBSim's `FGAuxiliary::PitotTotalPressure`.
fn pitot_total_pressure(mach: f64, p: f64) -> f64 {
    p * (1.0 + 0.5 * (SH_RATIO - 1.0) * mach * mach).powf(SH_RATIO / (SH_RATIO - 1.0))
}

/// Mach from impact pressure `qc` at static pressure `p` (psf), subsonic branch.
/// JSBSim's `FGAuxiliary::MachFromImpactPressure`.
fn mach_from_impact_pressure(qc: f64, p: f64) -> f64 {
    let a = 2.0 / (SH_RATIO - 1.0);
    let b = (SH_RATIO - 1.0) / SH_RATIO;
    (a * ((qc / p + 1.0).powf(b) - 1.0)).max(0.0).sqrt()
}

/// Convert a geometric altitude to geopotential altitude (both ft).
fn geopotential(geometric_alt_ft: f64) -> f64 {
    (geometric_alt_ft * EARTH_RADIUS_FT) / (EARTH_RADIUS_FT + geometric_alt_ft)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sea_level_matches_standard_day() {
        let isa = Isa::new();
        assert!((isa.temperature(0.0) - 518.67).abs() < 1e-6);
        assert!((isa.pressure(0.0) - 2116.228).abs() < 1e-3);
        // Standard sea-level density ~0.0023769 slug/ft^3.
        assert!((isa.density(0.0) - 0.0023769).abs() < 1e-6);
        // Speed of sound ~1116.45 ft/s.
        assert!((isa.sound_speed(0.0) - 1116.45).abs() < 0.5);
    }

    #[test]
    fn tropopause_temperature_is_isothermal() {
        let isa = Isa::new();
        // 36089 ft geopotential is the tropopause; both sides near 389.97 R.
        let t_below = isa.temperature(36000.0);
        let t_above = isa.temperature(39000.0);
        assert!((t_below - 389.97).abs() < 3.0);
        assert!((t_above - 389.97).abs() < 3.0);
        // Temperature strictly decreases up to the tropopause.
        assert!(isa.temperature(0.0) > isa.temperature(18000.0));
    }

    #[test]
    fn pressure_and_density_decrease_with_altitude() {
        let isa = Isa::new();
        let p0 = isa.pressure(0.0);
        let p1 = isa.pressure(10000.0);
        let p2 = isa.pressure(20000.0);
        assert!(p0 > p1 && p1 > p2);
        assert!(isa.density(0.0) > isa.density(10000.0));
    }

    #[test]
    fn density_altitude_inverts_density() {
        let isa = Isa::new();
        for alt in [0.0, 5000.0, 20000.0, 40000.0] {
            let back = isa.density_altitude(isa.density(alt));
            assert!((back - alt).abs() < 1.0, "alt {alt} -> {back}");
        }
    }
}
