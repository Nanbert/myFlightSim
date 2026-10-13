//! Reciprocating (piston) engine and fixed-pitch propeller, following JSBSim's
//! `FGPiston` + `FGPropeller` closely enough to reproduce the c172p powerplant.
//! Units stay imperial on the output side (thrust in lbf, torque in lbf·ft) but
//! the gas equations run in SI internally, exactly as JSBSim does, so the
//! manifold-pressure math needs no translation.
//!
//! A handful of JSBSim refinements are deliberately omitted for now and noted
//! where they occur: ram-air pressure rise, supercharging/boost, the mixture
//! lean-out schedule (full rich is assumed), and the propeller's gyroscopic
//! angular-momentum term. Each is small for a normally-aspirated c172 in the
//! cruise regime this model is first validated in.

use crate::state::Vec3;
use crate::table::Table1;

/// ft·lbf per second in one horsepower.
const HP_TO_FTLBS: f64 = 550.0;
/// psf -> Pa.
const PSF_TO_PA: f64 = 47.88;
/// inHg -> Pa.
const INHG_TO_PA: f64 = 3386.38;
/// in^3 -> m^3.
const IN3_TO_M3: f64 = 1.639e-5;
/// Specific gas constant of air, J/(kg·K).
const R_AIR: f64 = 287.3;
/// Standard sea-level pressure, Pa.
const STD_PRESSURE: f64 = 101325.0;

/// Engine configuration (a physical engine, not a running instance).
#[derive(Clone, Copy)]
pub struct PistonSpec {
    pub displacement_in3: f64,
    pub max_hp: f64,
    pub cycles: f64,
    pub idle_rpm: f64,
    pub max_rpm: f64,
    pub min_map_inhg: f64,
    pub max_map_inhg: f64,
    pub stroke_in: f64,
    pub compression_ratio: f64,
    pub volumetric_eff: f64,
    pub peak_mps_fps: f64,
    pub fmep_dyn: f64,
    pub fmep_static: f64,
    pub static_friction_hp: f64,
    pub manifold_lag: f64,
    pub min_throttle: f64,
    pub max_throttle: f64,
    /// Brake specific fuel consumption (lb/hp/hr). A value > 0 is used as-is,
    /// matching JSBSim's `<bsfc>` override; otherwise ISFC is estimated from the
    /// engine geometry.
    pub bsfc: f64,
}

/// Propeller configuration.
#[derive(Clone, Copy)]
pub struct PropSpec {
    pub diameter_ft: f64,
    pub ixx: f64,
    /// Thrust coefficient vs advance ratio J.
    pub c_thrust: Table1,
    /// Power coefficient vs advance ratio J.
    pub c_power: Table1,
    /// Mach correction on thrust, indexed by helical tip Mach.
    pub ct_mach: Table1,
    /// Mach correction on power, indexed by helical tip Mach.
    pub cp_mach: Table1,
    /// Thruster location in the structural frame (inches from the datum, X aft
    /// / Y right / Z up), exactly as the model file gives it.
    pub location: Vec3,
}

/// What the powerplant produces each step.
#[derive(Clone, Copy)]
pub struct EngineOutput {
    /// Thrust along the body x-axis (lbf).
    pub thrust: f64,
    /// Reaction torque about the body x-axis (lbf·ft) — the roll you feel from
    /// the engine turning the prop.
    pub torque_x: f64,
    /// Slipstream velocity added to the elevator/rudder in `qbar-induced`.
    pub induced_velocity: f64,
    pub power_hp: f64,
    pub rpm: f64,
}

/// Running powerplant state.
pub struct Powerplant {
    piston: PistonSpec,
    prop: PropSpec,
    // Derived at init:
    z_airbox: f64,
    z_throttle: f64,
    isfc: f64,
    // Running state:
    pub rpm: f64,
    tmap: f64,
    pub running: bool,
}

impl Powerplant {
    pub fn new(piston: PistonSpec, prop: PropSpec) -> Self {
        let max_map = piston.max_map_inhg.min(29.9) * INHG_TO_PA;
        let min_map = piston.min_map_inhg * INHG_TO_PA;
        let rated_mps = (piston.max_rpm * piston.stroke_in) / 360.0;
        let ze = piston.peak_mps_fps / rated_mps;
        let z_airbox = (STD_PRESSURE * ze / max_map) - ze;
        let idle_mps = (piston.idle_rpm * piston.stroke_in) / 360.0;
        let z_throttle = (piston.peak_mps_fps / idle_mps) * (STD_PRESSURE / min_map - 1.0)
            - z_airbox;

        // ISFC: the engine file's `<bsfc>` wins; otherwise estimate from geometry
        // (JSBSim computes it only when the file gives none).
        let isfc = if piston.bsfc > 0.0 {
            piston.bsfc
        } else {
            let disp_si = piston.displacement_in3 * IN3_TO_M3;
            let pmep = (29.92 - piston.max_map_inhg) * INHG_TO_PA * piston.volumetric_eff;
            let fmep = piston.fmep_dyn * rated_mps * 0.3048 + piston.fmep_static;
            let hp_loss =
                ((pmep + fmep) * disp_si * piston.max_rpm) / (piston.cycles * 22371.0);
            (1.1 * piston.displacement_in3 * piston.max_rpm * piston.volumetric_eff
                * (piston.max_map_inhg / 29.92))
                / (9411.0 * (piston.max_hp + hp_loss - piston.static_friction_hp))
        };

        Powerplant {
            piston,
            prop,
            z_airbox,
            z_throttle,
            isfc,
            rpm: 0.0,
            tmap: STD_PRESSURE,
            running: false,
        }
    }

    /// Light the engine and time-march it to a steady state at full throttle,
    /// mirroring JSBSim's `FGPropulsion::GetSteadyState` (which `set-running`
    /// triggers): big 0.5 s steps until the thrust stops changing. Without this
    /// the engine starts at a raw idle and needs ~1.3 s to spool up, which is a
    /// visible transient the reference model doesn't have.
    pub fn start(
        &mut self,
        rho: f64,
        pressure_psf: f64,
        temperature_r: f64,
        vel_x: f64,
        sound_speed: f64,
    ) {
        self.light();
        self.spool(1.0, rho, pressure_psf, temperature_r, vel_x, sound_speed);
    }

    /// Light the engine at twice idle rpm with a fresh manifold pressure — the
    /// starting point JSBSim's `FGPiston::InitRunning` uses.
    fn light(&mut self) {
        self.rpm = 2.0 * self.piston.idle_rpm;
        self.running = true;
        self.tmap = STD_PRESSURE;
    }

    /// Time-march the (already lit) engine at big 0.5 s steps until the thrust
    /// stops changing, mirroring JSBSim's `FGPropulsion::GetSteadyState`. Spools
    /// from whatever rpm the engine already carries, so the caller sets the
    /// starting point; leaves the engine sitting at that steady state and returns
    /// its output.
    #[allow(clippy::too_many_arguments)]
    pub fn spool(
        &mut self,
        throttle: f64,
        rho: f64,
        pressure_psf: f64,
        temperature_r: f64,
        vel_x: f64,
        sound_speed: f64,
    ) -> EngineOutput {
        let mut last_thrust = -1.0;
        let mut steady = 0;
        let mut out = EngineOutput {
            thrust: 0.0,
            torque_x: 0.0,
            induced_velocity: 0.0,
            power_hp: 0.0,
            rpm: self.rpm,
        };
        for _ in 0..6000 {
            out = self.step(0.5, throttle, rho, pressure_psf, temperature_r, vel_x, sound_speed);
            let thrust = out.thrust;
            if (last_thrust - thrust).abs() < 0.0001 {
                steady += 1;
                if steady > 120 {
                    break;
                }
            } else {
                steady = 0;
            }
            last_thrust = thrust;
        }
        out
    }

    /// Bring *this* engine to the steady state at `throttle` and the given flight
    /// condition, leaving it running there. Used to seat the live powerplant on
    /// the trim point.
    #[allow(clippy::too_many_arguments)]
    pub fn steady_at(
        &mut self,
        throttle: f64,
        rho: f64,
        pressure_psf: f64,
        temperature_r: f64,
        vel_x: f64,
        sound_speed: f64,
    ) -> EngineOutput {
        self.light();
        self.spool(throttle, rho, pressure_psf, temperature_r, vel_x, sound_speed)
    }

    /// Steady output at `throttle` and a flight condition, computed on a fresh
    /// scratch engine so this one's running state is neither read nor disturbed.
    /// The trim solver evaluates it at many trial conditions.
    #[allow(clippy::too_many_arguments)]
    pub fn steady_output(
        &self,
        throttle: f64,
        rho: f64,
        pressure_psf: f64,
        temperature_r: f64,
        vel_x: f64,
        sound_speed: f64,
    ) -> EngineOutput {
        let mut scratch = Powerplant::new(self.piston, self.prop);
        scratch.steady_at(throttle, rho, pressure_psf, temperature_r, vel_x, sound_speed)
    }

    /// Advance the powerplant one step. `throttle` is the normalized command,
    /// `rho`/`pressure`/`temperature` come from the atmosphere, `vel_x` is the
    /// body-forward airspeed at the propeller, and `sound_speed` sets the tip
    /// Mach correction. Returns thrust, torque, and the induced slipstream.
    #[allow(clippy::too_many_arguments)]
    pub fn step(
        &mut self,
        dt: f64,
        throttle: f64,
        rho: f64,
        pressure_psf: f64,
        temperature_r: f64,
        vel_x: f64,
        sound_speed: f64,
    ) -> EngineOutput {
        let throttle = throttle.clamp(self.piston.min_throttle, self.piston.max_throttle);
        let p_amb = pressure_psf * PSF_TO_PA;
        let t_amb = temperature_r / 1.8;
        let hp = self.engine_power(dt, throttle, p_amb, t_amb);

        let engine_power = hp * HP_TO_FTLBS; // ft·lbf/s
        self.spin_up(dt, engine_power, rho, vel_x, sound_speed);
        let (thrust, induced, _) = self.prop_forces(rho, vel_x, sound_speed);

        let omega = (self.rpm / 60.0) * 2.0 * std::f64::consts::PI;
        let torque_x = if omega > 0.01 {
            -engine_power / omega
        } else {
            0.0
        };

        EngineOutput {
            thrust,
            torque_x,
            induced_velocity: induced,
            power_hp: hp,
            rpm: self.rpm,
        }
    }

    /// Piston power (hp) — JSBSim's `doMAP`/`doAirFlow`/`doFuelFlow`/`doEnginePower`.
    fn engine_power(&mut self, dt: f64, throttle: f64, p_amb: f64, t_amb: f64) -> f64 {
        let rpm = self.rpm.max(0.0);
        let mean_mps = (rpm * self.piston.stroke_in) / 360.0;

        // --- manifold pressure (throttle impedance model) ---
        let zt = (1.0 - throttle).powi(2) * self.z_throttle;
        let ze = if mean_mps > 0.0 {
            self.piston.peak_mps_fps / mean_mps
        } else {
            999999.0
        };
        let map_coeff = ze / (ze + self.z_airbox + zt);
        let mut dmap = self.tmap - p_amb * map_coeff;
        if self.piston.manifold_lag > dt {
            dmap *= dt / self.piston.manifold_lag;
        }
        self.tmap -= dmap;
        let map = self.tmap;
        let pmep = (map - p_amb) * self.piston.volumetric_eff;

        if !self.running {
            return 0.0;
        }

        // --- air flow ---
        let gamma = 1.3;
        let cr = self.piston.compression_ratio;
        let mut mratio = if map < 1.0 { cr } else { p_amb / map };
        if mratio > cr {
            mratio = cr;
        }
        let ve = (gamma - 1.0) / gamma + (cr - mratio) / (gamma * (cr - 1.0));
        let swept_volume = (self.piston.displacement_in3 * IN3_TO_M3 * (rpm / 60.0)) / 2.0;
        let v_dot_air = swept_volume * self.piston.volumetric_eff * ve;
        let rho_manifold = map / (R_AIR * t_amb);
        let m_dot_air = v_dot_air * rho_manifold;

        // --- fuel flow (full rich) ---
        let equivalence_ratio = 1.3 * STD_PRESSURE / p_amb;
        let m_dot_fuel = m_dot_air * equivalence_ratio / 14.7;
        let fuel_flow_pph = m_dot_fuel * 2.2046 * 3600.0;

        // --- brake power ---
        let me = mixture_efficiency(m_dot_fuel / m_dot_air.max(1e-9));
        let indicated = (fuel_flow_pph / self.isfc) * me - self.piston.static_friction_hp;

        let fmep = -(self.piston.fmep_dyn * mean_mps * 0.3048 + self.piston.fmep_static);
        let pumping_hp = ((pmep + fmep) * self.piston.displacement_in3 * IN3_TO_M3 * rpm)
            / (self.piston.cycles * 22371.0);

        (indicated + pumping_hp).max(0.0)
    }

    /// Propeller coefficients at the current rpm: thrust (lbf), induced
    /// velocity (ft/s), and the power the prop absorbs (ft·lbf/s). Pure — no
    /// state changes — so the integrator can call it at each trial state.
    pub fn prop_forces(&self, rho: f64, vel: f64, sound_speed: f64) -> (f64, f64, f64) {
        let d = self.prop.diameter_ft;
        let d4 = d.powi(4);
        let d5 = d * d4;
        let rps = self.rpm / 60.0;

        let v_tip = rps * d * std::f64::consts::PI;
        let tip_mach = (v_tip * v_tip + vel * vel).sqrt() / sound_speed.max(1.0);

        let j = if rps > 0.01 { vel / (d * rps) } else { vel / d };

        let local_rps = rps.max(0.01);
        let cp = self.prop.c_power.lookup(j) * self.prop.cp_mach.lookup(tip_mach);
        let power_required = cp * local_rps.powi(3) * d5 * rho;

        let ct = self.prop.c_thrust.lookup(j) * self.prop.ct_mach.lookup(tip_mach);
        let thrust = ct * rps * rps * d4 * rho;

        let area = 0.25 * d * d * std::f64::consts::PI;
        let vel2sum = vel * vel.abs() + 2.0 * thrust / (rho * area).max(1e-9);
        let induced = if vel2sum > 0.0 {
            0.5 * (-vel + vel2sum.sqrt())
        } else {
            0.5 * (-vel - (-vel2sum).sqrt())
        };

        (thrust, induced, power_required)
    }

    /// Spin the propeller toward power balance (called once per FDM step).
    fn spin_up(&mut self, dt: f64, engine_power: f64, rho: f64, vel: f64, sound_speed: f64) {
        let (_, _, power_required) = self.prop_forces(rho, vel, sound_speed);
        let rps = self.rpm / 60.0;
        let omega = rps * 2.0 * std::f64::consts::PI;
        let excess_torque = if omega > 0.01 {
            (engine_power - power_required) / omega
        } else {
            engine_power - power_required
        };
        self.rpm =
            (rps + ((excess_torque / self.prop.ixx) / (2.0 * std::f64::consts::PI)) * dt) * 60.0;
        if self.rpm < 0.0 {
            self.rpm = 0.0;
        }
    }
}

/// Combustion efficiency vs fuel/air ratio (JSBSim's default correlation).
fn mixture_efficiency(far: f64) -> f64 {
    const XS: [f64; 15] = [
        0.05000, 0.05137, 0.05179, 0.05430, 0.05842, 0.06312, 0.06942, 0.07786, 0.08845,
        0.09270, 0.10120, 0.11455, 0.12158, 0.12435, 0.12500,
    ];
    const YS: [f64; 15] = [
        0.00000, 0.00862, 0.21552, 0.48276, 0.70690, 0.83621, 0.93103, 1.00000, 1.00000,
        0.98276, 0.93103, 0.72414, 0.45690, 0.23276, 0.00000,
    ];
    Table1::new(&XS, &YS).lookup(far)
}

#[cfg(test)]
mod tests {
    use super::*;

    // Sea-level standard-day conditions.
    const SL_RHO: f64 = 0.0023769;
    const SL_PRESSURE: f64 = 2116.228;
    const SL_TEMP_R: f64 = 518.67;
    const SL_SOUND: f64 = 1116.45;

    fn io320() -> PistonSpec {
        PistonSpec {
            displacement_in3: 320.0,
            max_hp: 160.0,
            cycles: 4.0,
            idle_rpm: 550.0,
            max_rpm: 2700.0,
            min_map_inhg: 10.0,
            max_map_inhg: 28.5,
            stroke_in: 4.375,
            compression_ratio: 8.5,
            volumetric_eff: 0.85,
            peak_mps_fps: 100.0,
            fmep_dyn: 18400.0,
            fmep_static: 46500.0,
            static_friction_hp: 1.5,
            manifold_lag: 1.0,
            min_throttle: 0.1,
            max_throttle: 1.0,
            bsfc: 0.32,
        }
    }

    fn prop() -> PropSpec {
        PropSpec {
            diameter_ft: 6.25,
            ixx: 1.67,
            c_thrust: Table1::new(
                &[0.0, 0.4, 0.8, 1.2, 1.6, 2.3],
                &[0.073, 0.069, 0.045, -0.006, -0.045, -0.073],
            ),
            c_power: Table1::new(
                &[0.0, 0.4, 0.8, 1.2, 1.6, 2.4],
                &[0.0660, 0.0600, 0.0426, 0.0155, 0.0426, 0.0504],
            ),
            ct_mach: Table1::new(&[0.85, 1.05], &[1.0, 0.8]),
            cp_mach: Table1::new(&[0.85, 1.05, 2.0], &[1.0, 1.8, 1.4]),
            location: [-37.7 / 12.0, 0.0, 26.6 / 12.0],
        }
    }

    #[test]
    fn full_throttle_produces_power_near_rated() {
        let mut pp = Powerplant::new(io320(), prop());
        // `start` spools to a steady state, so it should already be at cruise.
        pp.start(SL_RHO, SL_PRESSURE, SL_TEMP_R, 0.0, SL_SOUND);
        let out = pp_step(&mut pp, 1.0);
        // Sea-level *static* full throttle settles near 2500 rpm (the prop's
        // zero-advance-ratio power demand is highest there).
        assert!(out.rpm > 2400.0, "expected near-static cruise rpm, got {}", out.rpm);
        assert!(out.power_hp > 100.0, "expected real power, got {}", out.power_hp);
        // Rated 160 hp is a nominal sea-level rating; the power curve can carry
        // it a bit past that at full throttle, matching JSBSim.
        assert!(out.power_hp < 220.0, "power way past rating: {}", out.power_hp);
    }

    #[test]
    fn idle_throttle_produces_little_thrust() {
        let mut pp = Powerplant::new(io320(), prop());
        pp.start(SL_RHO, SL_PRESSURE, SL_TEMP_R, 0.0, SL_SOUND);
        for _ in 0..600 {
            pp_step(&mut pp, 0.1);
        }
        let out = pp_step(&mut pp, 0.1);
        assert!(out.power_hp < 20.0, "idle power too high: {}", out.power_hp);
    }

    fn pp_step(pp: &mut Powerplant, throttle: f64) -> EngineOutput {
        pp.step(1.0 / 120.0, throttle, SL_RHO, SL_PRESSURE, SL_TEMP_R, 0.0, SL_SOUND)
    }
}
