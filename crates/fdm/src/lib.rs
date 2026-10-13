//! Pure-Rust flight dynamics model — the wasm-clean replacement for JSBSim.
//!
//! This wires the building blocks together and exposes the [`FlightModel`]
//! named-property bus, so `FDM_BACKEND=rust` flies with no C++ in the loop:
//!
//! * [`table`] / [`aero`] — JSBSim's `<table>` and `<function>` coefficient
//!   evaluator; [`aircraft::c172`] supplies the data.
//! * [`atmos`] — U.S. Standard Atmosphere 1976.
//! * [`state`] — 6-DoF rigid-body state and an RK4 integrator (flat-earth NED).
//! * [`engine`] — the reciprocating engine + fixed-pitch propeller.
//! * [`gear`] — landing-gear ground reaction (strut spring/damper, tyre
//!   friction, nosewheel steering).
//!
//! Each `step` advances one fixed timestep at 120 Hz: the atmosphere is sampled
//! at the CG, the powerplant spins up (quasi-steady over the step), then RK4
//! integrates the aircraft with aero, thrust, ground reaction, and gravity
//! recomputed at every stage from the trial state. [`FlightModel::trim`] solves
//! a wings-level steady trim at a target airspeed and altitude and seats the
//! model on it; the lateral correction is parked in the roll/yaw trim tabs so it
//! survives the pilot keys. Deliberate omissions for this first cut (noted in
//! the modules that own them): no wind and `alphadot` fixed at 0.
//!
//! Kept free of Bevy, the filesystem, and threads so it compiles to `wasm32`.

mod aero;
mod aircraft;
mod atmos;
mod engine;
mod gear;
mod state;
mod table;

use crate::aero::{AeroResult, PropMap};
use crate::aircraft::{c172::C172, Aircraft, INCH_TO_FT};
use crate::atmos::{Isa, G0};
use crate::engine::{EngineOutput, Powerplant};
use crate::gear::{ground_force, Ground};
use crate::state::{rk4, State, Vec3};
use flight_model::FlightModel;

const R2D: f64 = 180.0 / std::f64::consts::PI;
const D2R: f64 = std::f64::consts::PI / 180.0;
/// Knots <-> ft/s.
const KT_TO_FPS: f64 = 1.68781;
const FPS_TO_KT: f64 = 1.0 / KT_TO_FPS;
/// Feet per degree of latitude, chosen so the app's NED->world round-trip is
/// exact for its `EARTH_RADIUS_M = 6_378_137 m`: `(pi/180)*R/0.3048`.
const FT_PER_DEG_LAT: f64 = 365_221.79;

/// A per-step snapshot of the aero/propulsion force and moment split, published
/// through the property bus so the validation harness can diff it against
/// JSBSim term by term.
#[derive(Clone, Copy, Default)]
struct Diag {
    alpha: f64,
    qbar: f64,
    force_aero: Vec3,
    moment_aero: Vec3,
    moment_prop: Vec3,
    thrust: f64,
}

/// Resolved control-surface positions for one step (radians, except flap in deg).
#[derive(Clone, Copy, Default)]
struct Surfaces {
    elevator_rad: f64,
    mag_elevator_rad: f64,
    aileron_rad: f64,
    rudder_rad: f64,
    flap_deg: f64,
}

/// A flight model instance for one aircraft.
pub struct Fdm {
    ac: &'static Aircraft,
    state: State,
    isa: Isa,
    power: Powerplant,
    props: PropMap,
    // Mass properties are fixed (the point masses don't move), so cache them.
    mass: f64,
    inertia: state::Inertia,
    /// Propeller position minus the loaded CG, body axes (ft) — the arm for the
    /// off-axis thrust moment.
    prop_arm_ft: Vec3,
    /// Aerodynamic reference point minus the loaded CG, body axes (ft).
    rp_offset_ft: Vec3,
    /// CG -> each gear contact point, body axes (ft), aligned with `ac.gear`.
    gear_arms_ft: Vec<Vec3>,
    // Pilot commands (normalized), written through `set`.
    elevator_cmd: f64,
    aileron_cmd: f64,
    rudder_cmd: f64,
    throttle_cmd: f64,
    pitch_trim_cmd: f64,
    left_brake_cmd: f64,
    right_brake_cmd: f64,
    // Lateral trim tabs, summed onto the pilot commands (JSBSim's roll/yaw trim
    // summers). The trim solver writes these; the pilot keys don't, so a trim
    // survives the per-step command writes.
    roll_trim_cmd: f64,
    yaw_trim_cmd: f64,
    flap_cmd: f64,
    flap_pos_deg: f64,
    // Programmed initial conditions (read by `apply_ic`).
    ic_vc_kts: f64,
    ic_alt_ft: f64,
    ic_psi_deg: f64,
    // Last engine outputs the app can read back.
    engine_rpm: f64,
    engine_power_hp: f64,
    // True when any gear is in contact with the ground on the last step.
    wow: bool,
    // Last step's force/moment breakdown, for frame-by-frame comparison against
    // JSBSim (`forces/*`, `moments/*`).
    diag: Diag,
    // Flat-earth origin, so NED can be reported as lat/lon.
    origin_lat: f64,
    origin_lon: f64,
    dt: f64,
}

impl Fdm {
    /// Construct a model for `aircraft` (e.g. `"c172p"`).
    pub fn new(aircraft: &str) -> Self {
        let ac: &'static Aircraft = match aircraft {
            "c172p" | "c172" => &C172,
            _ => &C172,
        };
        let cg_in = ac.loaded_cg_in();
        let prop_arm_ft = struct_to_body(ac.prop.location, cg_in);
        let rp_offset_ft = struct_to_body(ac.aero.aerorp_in, cg_in);
        let gear_arms_ft = ac.gear.iter().map(|g| struct_to_body(g.pos_in, cg_in)).collect();
        Fdm {
            ac,
            state: State::default(),
            isa: Isa::new(),
            power: Powerplant::new(ac.piston, ac.prop),
            props: PropMap::new(),
            mass: ac.mass_slug(),
            inertia: ac.inertia(),
            prop_arm_ft,
            rp_offset_ft,
            gear_arms_ft,
            elevator_cmd: 0.0,
            aileron_cmd: 0.0,
            rudder_cmd: 0.0,
            throttle_cmd: 0.0,
            pitch_trim_cmd: 0.0,
            left_brake_cmd: 0.0,
            right_brake_cmd: 0.0,
            roll_trim_cmd: 0.0,
            yaw_trim_cmd: 0.0,
            flap_cmd: 0.0,
            flap_pos_deg: 0.0,
            ic_vc_kts: 0.0,
            ic_alt_ft: 0.0,
            ic_psi_deg: 0.0,
            engine_rpm: 0.0,
            engine_power_hp: 0.0,
            wow: false,
            diag: Diag::default(),
            origin_lat: 0.0,
            origin_lon: 0.0,
            dt: 1.0 / 120.0,
        }
    }

    /// The aircraft this model was built for.
    pub fn aircraft(&self) -> &str {
        self.ac.name
    }

    /// Height above the ground (ft), from the NED down-axis.
    fn alt_ft(&self) -> f64 {
        -self.state.pos[2]
    }

    /// Resolve the control commands into surface deflections. Mirrors the c172p
    /// `<flight_control>` channels: the pitch trim is summed into the elevator
    /// command and clipped to ±1 (`summer`), then a single `aerosurface_scale`
    /// maps that onto the elevator deg range (`zero_centered`, so sign picks max
    /// vs min) and the gain converts to radians.
    fn surfaces(&self) -> Surfaces {
        let fcs = &self.ac.fcs;
        let elevator_rad =
            scale(self.elevator_cmd + self.pitch_trim_cmd, fcs.elevator_lim_deg) * fcs.elevator_gain;
        let aileron_rad =
            scale(self.aileron_cmd + self.roll_trim_cmd, fcs.aileron_lim_deg) * fcs.aileron_gain;
        let rudder_rad =
            scale(self.rudder_cmd + self.yaw_trim_cmd, fcs.rudder_lim_deg) * fcs.rudder_gain;
        Surfaces {
            elevator_rad,
            mag_elevator_rad: elevator_rad.abs(),
            aileron_rad,
            rudder_rad,
            flap_deg: self.flap_pos_deg,
        }
    }

    /// Roll the flaps toward the commanded deflection at the model's flap rate.
    fn step_flap(&mut self, dt: f64) {
        let fcs = &self.ac.fcs;
        let target = self.flap_cmd * fcs.flap_max_deg;
        let rate = fcs.flap_max_deg / fcs.flap_time_s;
        let max_move = rate * dt;
        let err = target - self.flap_pos_deg;
        self.flap_pos_deg = if err.abs() <= max_move {
            target
        } else {
            self.flap_pos_deg + max_move * err.signum()
        };
    }

    /// Advance the whole model one fixed timestep.
    fn advance(&mut self, dt: f64) {
        let alt = self.alt_ft();
        let rho = self.isa.density(alt);
        let pressure = self.isa.pressure(alt);
        let temp = self.isa.temperature(alt);
        let sound = self.isa.sound_speed(alt);

        // Powerplant spins up once per step (quasi-steady); its thrust, induced
        // slipstream, and reaction torque then hold constant over the step, the
        // same once-per-frame cadence JSBSim's propulsion module uses.
        let out = self.power.step(
            dt,
            self.throttle_cmd,
            rho,
            pressure,
            temp,
            self.state.vel[0],
            sound,
        );
        self.engine_rpm = out.rpm;
        self.engine_power_hp = out.power_hp;
        let thrust = out.thrust;
        let torque_x = out.torque_x;
        let induced = out.induced_velocity;
        // Propeller moment about the CG: the engine's reaction torque about the
        // roll axis, plus the thrust's off-axis arm (M = r x F, F = [thrust,0,0]).
        let arm = self.prop_arm_ft;
        let mprop = [torque_x, arm[2] * thrust, -arm[1] * thrust];

        let surf = self.surfaces();
        self.step_flap(dt);

        let ac = self.ac;
        let mass = self.mass;
        let inertia = self.inertia;
        let state = self.state;
        let rp_offset = self.rp_offset_ft;
        // Ground-reaction inputs: the gear layout, its precomputed arms, the
        // nosewheel steer angle, and the brake channels — held constant over the
        // step (the steer angle comes from the rudder command).
        let gears = ac.gear;
        let arms = &self.gear_arms_ft;
        let rudder = self.rudder_cmd;
        let brake = [self.left_brake_cmd, self.right_brake_cmd];
        // Reuse the scratch map's allocation across steps; take it out of `self`
        // so the aero closure can hold it mutably without borrowing all of `self`.
        let mut props = std::mem::take(&mut self.props);

        // Aero is recomputed at each RK4 stage from the trial state; thrust and
        // its moment are constant over the step. The snapshot keeps the last
        // stage's aero (nearest the end-of-step state) for the property bus.
        let mut diag = Diag::default();
        // Ground reaction is also recomputed per stage; keep the last stage's.
        let mut ground = Ground::default();
        let next = rk4(&state, dt, mass, &inertia, |s: &State| {
            fill_props(&mut props, ac, s, &surf, induced, rho);
            let aero = ac.aero.evaluate(&mut props, rp_offset);
            ground = ground_force(s, gears, arms, rudder, brake, dt);
            diag = Diag {
                alpha: props.get("aero/alpha-rad"),
                qbar: props.get("aero/qbar-psf"),
                force_aero: aero.force_body,
                moment_aero: aero.moment_body,
                moment_prop: mprop,
                thrust,
            };
            (
                [
                    aero.force_body[0] + thrust + ground.force_body[0],
                    aero.force_body[1] + ground.force_body[1],
                    aero.force_body[2] + ground.force_body[2],
                ],
                [
                    aero.moment_body[0] + torque_x + ground.moment_body[0],
                    aero.moment_body[1] + mprop[1] + ground.moment_body[1],
                    aero.moment_body[2] + mprop[2] + ground.moment_body[2],
                ],
            )
        });
        self.props = props;
        self.diag = diag;
        self.wow = ground.wow;
        self.state = next;
    }

    /// Vertical speed (ft/s), positive up: `-z_dot` in NED.
    fn h_dot_fps(&self) -> f64 {
        -self.state.body_to_ned(self.state.vel)[2]
    }

    /// Evaluate the aero forces/moments and the (steady) powerplant at a trial
    /// trim point: bank angle `phi`, angle of attack `alpha`, the four surface
    /// commands, and throttle. Returns the aero result, the engine output, and
    /// gravity in body axes. Shared by the longitudinal and lateral trim
    /// residuals. `phi` tilts gravity (and so the lateral force) — the bank that
    /// balances side force, matching JSBSim's `tFull` trim.
    #[allow(clippy::too_many_arguments)]
    fn trim_eval(
        &self,
        alpha: f64,
        phi: f64,
        aileron_cmd: f64,
        rudder_cmd: f64,
        trim_cmd: f64,
        throttle: f64,
        vt: f64,
        alt: f64,
    ) -> (AeroResult, EngineOutput, Vec3) {
        let rho = self.isa.density(alt);
        let pressure = self.isa.pressure(alt);
        let temp = self.isa.temperature(alt);
        let sound = self.isa.sound_speed(alt);

        // The prop sees the body-forward airspeed u = V cos(alpha).
        let u = vt * alpha.cos();
        let eng = self
            .power
            .steady_output(throttle, rho, pressure, temp, u, sound);

        let fcs = &self.ac.fcs;
        let elevator_rad = scale(trim_cmd, fcs.elevator_lim_deg) * fcs.elevator_gain;
        let surf = Surfaces {
            elevator_rad,
            mag_elevator_rad: elevator_rad.abs(),
            aileron_rad: scale(aileron_cmd, fcs.aileron_lim_deg) * fcs.aileron_gain,
            rudder_rad: scale(rudder_cmd, fcs.rudder_lim_deg) * fcs.rudder_gain,
            flap_deg: 0.0,
        };
        // Flight-path-level, sideslip-free: pitch attitude equals alpha, bank is
        // `phi` (body velocity stays [u, 0, w], so alpha is unaffected by bank).
        let state = State {
            pos: [0.0, 0.0, -alt],
            vel: [u, 0.0, vt * alpha.sin()],
            quat: euler_to_quat(phi, alpha, 0.0),
            omega: [0.0; 3],
        };
        let mut props = PropMap::new();
        fill_props(&mut props, self.ac, &state, &surf, eng.induced_velocity, rho);
        let aero = self.ac.aero.evaluate(&mut props, self.rp_offset_ft);
        let grav = state.ned_to_body([0.0, 0.0, G0]);
        (aero, eng, grav)
    }

    /// Longitudinal residual `[Fx, Fz, My]` (weight-normalized), zero at trim.
    /// `x = [alpha_rad, pitch_trim_cmd, throttle]`.
    fn trim_long_residual(&self, x: [f64; 3], vt: f64, alt: f64) -> [f64; 3] {
        let (aero, eng, grav) = self.trim_eval(x[0], 0.0, 0.0, 0.0, x[1], x[2], vt, alt);
        let weight = self.mass * G0;
        let fx = aero.force_body[0] + eng.thrust + grav[0] * self.mass;
        let fz = aero.force_body[2] + grav[2] * self.mass;
        let my = aero.moment_body[1] + self.prop_arm_ft[2] * eng.thrust;
        [fx / weight, fz / weight, my / (weight * self.ac.wing_chord_ft)]
    }

    /// Lateral residual `[Fy, Mx, Mz]` (weight- and weight-span-normalized), zero
    /// at trim, with the longitudinal unknowns held at their solved values.
    /// `x = [phi_bank, aileron_cmd, rudder_cmd]`. The bank balances side force;
    /// the aileron/rudder null the roll and yaw moments, the roll moment
    /// absorbing the engine's reaction torque (a real c172 counters it with
    /// aileron trim).
    fn trim_lat_residual(&self, x: [f64; 3], long: [f64; 3], vt: f64, alt: f64) -> [f64; 3] {
        let (aero, eng, grav) =
            self.trim_eval(long[0], x[0], x[1], x[2], long[1], long[2], vt, alt);
        let weight = self.mass * G0;
        let scale = weight * self.ac.wing_span_ft;
        let fy = aero.force_body[1] + grav[1] * self.mass;
        [
            fy / weight,
            (aero.moment_body[0] + eng.torque_x) / scale,
            aero.moment_body[2] / scale,
        ]
    }
}

impl FlightModel for Fdm {
    /// Solve a wings-level, steady trim at `vc_kts` (calibrated) and `alt_ft`,
    /// then seat the model on it. Longitudinal unknowns — angle of attack,
    /// pitch-trim command, throttle — null the axial/vertical force and pitch
    /// moment; lateral unknowns — aileron and rudder — go to the roll/yaw trim
    /// tabs, nulling the roll and yaw moment (the last absorbing the engine
    /// reaction torque). Each is a Newton iteration with a finite-difference
    /// Jacobian. The trimmed state is then a fixed point, so the aircraft holds
    /// the point open-loop.
    fn trim(&mut self, vc_kts: f64, alt_ft: f64) {
        let vt = self.isa.cas_to_tas(vc_kts * KT_TO_FPS, alt_ft);
        const H: f64 = 1e-4;

        // Longitudinal: alpha, pitch-trim command, throttle.
        let mut xl = [0.05, 0.0, 0.6];
        let n3 = |r: [f64; 3]| (r[0] * r[0] + r[1] * r[1] + r[2] * r[2]).sqrt();
        for _ in 0..60 {
            let r = self.trim_long_residual(xl, vt, alt_ft);
            let rnorm = n3(r);
            if rnorm < 1e-7 {
                break;
            }
            let mut j = [[0.0; 3]; 3];
            for k in 0..3 {
                let mut xp = xl;
                xp[k] += H;
                let rp = self.trim_long_residual(xp, vt, alt_ft);
                for i in 0..3 {
                    j[i][k] = (rp[i] - r[i]) / H;
                }
            }
            let Some(dx) = solve_n(j, [-r[0], -r[1], -r[2]]) else {
                break;
            };
            // Backtracking line search: a raw Newton step overshoots here and can
            // push throttle through the engine's clamp, which zeroes that Jacobian
            // column and stalls the solve. Halve the step until the residual drops.
            let mut step = 1.0;
            let mut xnew = xl;
            for _ in 0..24 {
                for k in 0..3 {
                    xnew[k] = xl[k] + step * dx[k];
                }
                clamp_long(&mut xnew);
                if n3(self.trim_long_residual(xnew, vt, alt_ft)) < rnorm {
                    break;
                }
                step *= 0.5;
            }
            xl = xnew;
        }

        // Lateral: bank (side force), aileron (roll), rudder (yaw), with the
        // longitudinal values held. This is JSBSim's `tFull` lateral triad.
        let mut xa = [0.0, 0.0, 0.0];
        let n3lat = |r: [f64; 3]| (r[0] * r[0] + r[1] * r[1] + r[2] * r[2]).sqrt();
        for _ in 0..40 {
            let r = self.trim_lat_residual(xa, xl, vt, alt_ft);
            let rnorm = n3lat(r);
            if rnorm < 1e-8 {
                break;
            }
            let mut j = [[0.0; 3]; 3];
            for k in 0..3 {
                let mut xp = xa;
                xp[k] += H;
                let rp = self.trim_lat_residual(xp, xl, vt, alt_ft);
                for i in 0..3 {
                    j[i][k] = (rp[i] - r[i]) / H;
                }
            }
            let Some(dx) = solve_n(j, [-r[0], -r[1], -r[2]]) else {
                break;
            };
            let mut step = 1.0;
            let mut xnew = xa;
            for _ in 0..24 {
                for k in 0..3 {
                    xnew[k] = (xa[k] + step * dx[k]).clamp(-0.35, 0.35);
                }
                if n3lat(self.trim_lat_residual(xnew, xl, vt, alt_ft)) < rnorm {
                    break;
                }
                step *= 0.5;
            }
            xa = xnew;
        }

        let alpha = xl[0];
        self.elevator_cmd = 0.0;
        self.pitch_trim_cmd = xl[1].clamp(-1.0, 1.0);
        self.throttle_cmd = xl[2].clamp(0.0, 1.0);
        // The solved aileron/rudder go to the *trim tabs*, not the pilot command:
        // the app's key bindings spring the commands back to zero every step, so a
        // trim parked in the command channel would be wiped. Total deflection is
        // unchanged (command 0 + tab).
        self.aileron_cmd = 0.0;
        self.rudder_cmd = 0.0;
        self.roll_trim_cmd = xa[1].clamp(-1.0, 1.0);
        self.yaw_trim_cmd = xa[2].clamp(-1.0, 1.0);
        // Keep the flat-earth horizontal position: re-seating altitude only means
        // an in-flight `trim` doesn't teleport the aircraft back to the origin.
        let [px, py, _] = self.state.pos;
        self.state = State {
            pos: [px, py, -alt_ft],
            vel: [vt * alpha.cos(), 0.0, vt * alpha.sin()],
            quat: euler_to_quat(xa[0], alpha, self.ic_psi_deg * D2R),
            omega: [0.0; 3],
        };
        self.flap_pos_deg = 0.0;
        self.ic_vc_kts = vc_kts;
        self.ic_alt_ft = alt_ft;

        // Seat the live powerplant on the trim point so the first step is already
        // consistent (otherwise it would start cold and re-spool).
        let rho = self.isa.density(alt_ft);
        let pressure = self.isa.pressure(alt_ft);
        let temp = self.isa.temperature(alt_ft);
        let sound = self.isa.sound_speed(alt_ft);
        let (throttle, u) = (self.throttle_cmd, vt * alpha.cos());
        let eng = self.power.steady_at(throttle, rho, pressure, temp, u, sound);
        self.engine_rpm = eng.rpm;
        self.engine_power_hp = eng.power_hp;
    }

    fn step(&mut self) {
        self.advance(self.dt);
    }

    fn apply_ic(&mut self) {
        // Airborne ICs only: level body axes, heading given. `ic/vc-kts` is a
        // *calibrated* airspeed, so convert it to the true airspeed the model
        // integrates (JSBSim's `SetVcalibratedKtsIC`).
        let tas = self
            .isa
            .cas_to_tas(self.ic_vc_kts * KT_TO_FPS, self.ic_alt_ft);
        self.state = State {
            pos: [0.0, 0.0, -self.ic_alt_ft],
            vel: [tas, 0.0, 0.0],
            quat: euler_to_quat(0.0, 0.0, self.ic_psi_deg * D2R),
            omega: [0.0; 3],
        };
        self.flap_pos_deg = self.flap_cmd * self.ac.fcs.flap_max_deg;
    }

    fn get(&self, name: &str) -> f64 {
        let (phi, theta, psi) = self.state.euler();
        match name {
            "position/h-sl-ft" => self.alt_ft(),
            "position/lat-gc-deg" => {
                self.origin_lat + self.state.pos[0] / FT_PER_DEG_LAT
            }
            "position/long-gc-deg" => {
                let cos_lat = (self.origin_lat * D2R).cos().max(1e-6);
                self.origin_lon + self.state.pos[1] / (FT_PER_DEG_LAT * cos_lat)
            }
            "attitude/phi-rad" => phi,
            "attitude/theta-rad" => theta,
            "attitude/psi-rad" => psi,
            "attitude/phi-deg" => phi * R2D,
            "attitude/theta-deg" => theta * R2D,
            "attitude/psi-deg" => psi * R2D,
            // Reported as *calibrated* airspeed, matching JSBSim's property.
            "velocities/vc-kts" => {
                self.isa.tas_to_cas(self.state.airspeed(), self.alt_ft()) * FPS_TO_KT
            }
            "velocities/h-dot-fps" => self.h_dot_fps(),
            "velocities/p-rad_sec" => self.state.omega[0],
            "velocities/q-rad_sec" => self.state.omega[1],
            "velocities/r-rad_sec" => self.state.omega[2],
            // True when any landing-gear contact point is on the ground.
            "gear/wow" => {
                if self.wow {
                    1.0
                } else {
                    0.0
                }
            }
            "fcs/flap-pos-deg" => self.flap_pos_deg,
            "fcs/elevator-cmd-norm" => self.elevator_cmd,
            "fcs/aileron-cmd-norm" => self.aileron_cmd,
            "fcs/rudder-cmd-norm" => self.rudder_cmd,
            "fcs/throttle-cmd-norm" => self.throttle_cmd,
            "fcs/pitch-trim-cmd-norm" => self.pitch_trim_cmd,
            "fcs/roll-trim-cmd-norm" => self.roll_trim_cmd,
            "fcs/yaw-trim-cmd-norm" => self.yaw_trim_cmd,
            "propulsion/engine[0]/engine-rpm" => self.engine_rpm,
            "propulsion/engine[0]/power-hp" => self.engine_power_hp,
            "propulsion/engine[0]/thrust-lbs" => self.diag.thrust,
            // Last-step aero/propulsion split (same spelling JSBSim uses), so
            // the validation harness can diff forces and moments frame by frame.
            "aero/alpha-rad" => self.diag.alpha,
            "aero/qbar-psf" => self.diag.qbar,
            "forces/fbx-aero-lbs" => self.diag.force_aero[0],
            "forces/fby-aero-lbs" => self.diag.force_aero[1],
            "forces/fbz-aero-lbs" => self.diag.force_aero[2],
            "moments/l-aero-lbsft" => self.diag.moment_aero[0],
            "moments/m-aero-lbsft" => self.diag.moment_aero[1],
            "moments/n-aero-lbsft" => self.diag.moment_aero[2],
            "moments/l-prop-lbsft" => self.diag.moment_prop[0],
            "moments/m-prop-lbsft" => self.diag.moment_prop[1],
            "moments/n-prop-lbsft" => self.diag.moment_prop[2],
            _ => 0.0,
        }
    }

    fn set(&mut self, name: &str, value: f64) {
        match name {
            "ic/vc-kts" => self.ic_vc_kts = value,
            "ic/h-sl-ft" => self.ic_alt_ft = value,
            "ic/psi-true-deg" => self.ic_psi_deg = value,
            // Any non-zero `set-running` lights the engine and spools it to a
            // steady state at the current flight condition (JSBSim's convention).
            "propulsion/set-running" => {
                if value != 0.0 {
                    let alt = self.alt_ft();
                    self.power.start(
                        self.isa.density(alt),
                        self.isa.pressure(alt),
                        self.isa.temperature(alt),
                        self.state.vel[0],
                        self.isa.sound_speed(alt),
                    );
                }
            }
            "fcs/elevator-cmd-norm" => self.elevator_cmd = value,
            "fcs/aileron-cmd-norm" => self.aileron_cmd = value,
            "fcs/rudder-cmd-norm" => self.rudder_cmd = value,
            "fcs/throttle-cmd-norm" => self.throttle_cmd = value,
            "fcs/pitch-trim-cmd-norm" => self.pitch_trim_cmd = value,
            "fcs/roll-trim-cmd-norm" => self.roll_trim_cmd = value,
            "fcs/yaw-trim-cmd-norm" => self.yaw_trim_cmd = value,
            "fcs/flap-cmd-norm" => self.flap_cmd = value,
            "fcs/left-brake-cmd-norm" => self.left_brake_cmd = value,
            "fcs/right-brake-cmd-norm" => self.right_brake_cmd = value,
            _ => {}
        }
    }
}

/// Fill the named-property scratch map the aero evaluator reads.
fn fill_props(
    props: &mut PropMap,
    ac: &Aircraft,
    s: &State,
    surf: &Surfaces,
    induced_fps: f64,
    rho: f64,
) {
    let (u, v, w) = (s.vel[0], s.vel[1], s.vel[2]);
    let vt = (u * u + v * v + w * w).sqrt().max(1e-6);
    let alpha = w.atan2(u);
    let beta = v.atan2((u * u + w * w).sqrt());
    let qbar = 0.5 * rho * vt * vt;

    props.set("metrics/Sw-sqft", ac.wing_area_sqft);
    props.set("metrics/bw-ft", ac.wing_span_ft);
    props.set("metrics/cbarw-ft", ac.wing_chord_ft);
    props.set("aero/qbar-psf", qbar);
    props.set("aero/alpha-rad", alpha);
    props.set("aero/beta-rad", beta);
    props.set("aero/mag-beta-rad", beta.abs());
    props.set("aero/bi2vel", ac.wing_span_ft / (2.0 * vt));
    props.set("aero/ci2vel", ac.wing_chord_ft / (2.0 * vt));
    props.set("aero/h_b-mac-ft", -s.pos[2] / ac.wing_chord_ft);
    props.set("aero/alphadot-rad_sec", 0.0);
    props.set("aero/stall-hyst-norm", 0.0);
    props.set("velocities/p-aero-rad_sec", s.omega[0]);
    props.set("velocities/q-aero-rad_sec", s.omega[1]);
    props.set("velocities/r-aero-rad_sec", s.omega[2]);
    props.set("velocities/u-aero-fps", u);
    props.set("atmosphere/rho-slugs_ft3", rho);
    props.set("propulsion/engine/prop-induced-velocity_fps", induced_fps);
    props.set("fcs/elevator-pos-rad", surf.elevator_rad);
    props.set("fcs/mag-elevator-pos-rad", surf.mag_elevator_rad);
    props.set("fcs/left-aileron-pos-rad", surf.aileron_rad);
    props.set("fcs/rudder-pos-rad", surf.rudder_rad);
    props.set("fcs/flap-pos-deg", surf.flap_deg);
}

/// Structural frame (X aft, Y right, Z **up**; inches from the datum) to body
/// frame (X forward, Y right, Z down; feet from the CG) — JSBSim's
/// `StructuralToBody`, which subtracts the CG and then rotates 180° about Y.
fn struct_to_body(r_in: Vec3, cg_in: Vec3) -> Vec3 {
    [
        -(r_in[0] - cg_in[0]) * INCH_TO_FT,
        (r_in[1] - cg_in[1]) * INCH_TO_FT,
        -(r_in[2] - cg_in[2]) * INCH_TO_FT,
    ]
}

/// A zero-centered `aerosurface_scale`: a positive command maps onto `lim.1`
/// (max), a negative one onto `lim.0` (min). Input is clamped to [-1, 1].
fn scale(cmd: f64, lim: (f64, f64)) -> f64 {
    let cmd = cmd.clamp(-1.0, 1.0);
    if cmd >= 0.0 {
        cmd * lim.1
    } else {
        -cmd * lim.0
    }
}

/// Keep the longitudinal trim unknowns `[alpha_rad, pitch_trim_cmd, throttle]`
/// in a smooth, physical range: angle of attack within +/-20 deg, the trim
/// command within its normalized authority, and throttle clear of the engine's
/// hard clamps (where its derivative vanishes and the Jacobian goes singular).
fn clamp_long(x: &mut [f64; 3]) {
    x[0] = x[0].clamp(-0.35, 0.35);
    x[1] = x[1].clamp(-1.0, 1.0);
    x[2] = x[2].clamp(0.02, 0.995);
}

/// Solve the `N x N` system `A x = b` by Gaussian elimination with partial
/// pivoting. `None` if `A` is (near-)singular.
#[allow(clippy::needless_range_loop)] // two rows of `a` are indexed in step
fn solve_n<const N: usize>(mut a: [[f64; N]; N], mut b: [f64; N]) -> Option<[f64; N]> {
    for c in 0..N {
        let mut p = c;
        for r in c + 1..N {
            if a[r][c].abs() > a[p][c].abs() {
                p = r;
            }
        }
        if a[p][c].abs() < 1e-12 {
            return None;
        }
        a.swap(p, c);
        b.swap(p, c);
        for r in c + 1..N {
            let f = a[r][c] / a[c][c];
            for k in c..N {
                a[r][k] -= f * a[c][k];
            }
            b[r] -= f * b[c];
        }
    }
    let mut x = [0.0; N];
    for i in (0..N).rev() {
        let mut s = b[i];
        for k in i + 1..N {
            s -= a[i][k] * x[k];
        }
        x[i] = s / a[i][i];
    }
    Some(x)
}

/// Body->NED quaternion `[w, x, y, z]` from ZYX Euler angles (rad).
fn euler_to_quat(phi: f64, theta: f64, psi: f64) -> [f64; 4] {
    let (cp, sp) = ((phi / 2.0).cos(), (phi / 2.0).sin());
    let (ct, st) = ((theta / 2.0).cos(), (theta / 2.0).sin());
    let (cy, sy) = ((psi / 2.0).cos(), (psi / 2.0).sin());
    // q = qz * qy * qx
    [
        cy * ct * cp + sy * st * sp,
        cy * ct * sp - sy * st * cp,
        cy * st * cp + sy * ct * sp,
        sy * ct * cp - cy * st * sp,
    ]
}

#[cfg(test)]
mod tests {
    use super::*;

    fn fly(steps: usize) -> Fdm {
        let mut m = Fdm::new("c172p");
        m.set("ic/vc-kts", 120.0);
        m.set("ic/h-sl-ft", 3000.0);
        m.set("ic/psi-true-deg", 0.0);
        m.apply_ic();
        m.set("propulsion/set-running", -1.0);
        m.set("fcs/throttle-cmd-norm", 1.0);
        for _ in 0..steps {
            m.step();
        }
        m
    }

    /// Parked on the runway at the threshold, engine running — the state the
    /// app's `reset_ground` programs.
    fn parked() -> Fdm {
        let mut m = Fdm::new("c172p");
        m.set("ic/vc-kts", 0.0);
        m.set("ic/h-sl-ft", 4.7);
        m.set("ic/psi-true-deg", 0.0);
        m.apply_ic();
        m.set("propulsion/set-running", -1.0);
        m
    }

    #[test]
    fn parks_and_rests_on_gear() {
        // The gear must hold the aircraft up: from the 4.7 ft IC it settles onto
        // the struts and stays there, instead of sinking through the ground.
        let mut m = parked();
        for _ in 0..120 * 5 {
            m.step();
        }
        let h = m.get("position/h-sl-ft");
        assert!(h > 0.0, "the aircraft sank through the ground: h = {h}");
        assert!((3.0..6.0).contains(&h), "unexpected settled height: {h}");
        assert!(m.get("gear/wow") > 0.5, "should report weight-on-wheels");
        assert!(m.get("velocities/h-dot-fps").abs() < 1.0, "should be at rest");
    }

    #[test]
    fn wow_true_on_ground_false_airborne() {
        let mut m = parked();
        for _ in 0..120 {
            m.step();
        }
        assert!(m.get("gear/wow") > 0.5, "a parked aircraft is on the wheels");
        m.set("ic/vc-kts", 120.0);
        m.set("ic/h-sl-ft", 3000.0);
        m.apply_ic();
        m.set("propulsion/set-running", -1.0);
        m.step();
        m.step();
        assert!(m.get("gear/wow") < 0.5, "an airborne aircraft is off the wheels");
    }

    #[test]
    fn full_throttle_rolls_forward() {
        // From a standstill, full throttle should accelerate the aircraft down
        // the runway (heading north) without it sinking or veering.
        let mut m = parked();
        m.set("fcs/throttle-cmd-norm", 1.0);
        let mut min_h = f64::INFINITY;
        for _ in 0..120 * 8 {
            m.step();
            min_h = min_h.min(m.get("position/h-sl-ft"));
        }
        assert!(min_h > 0.0, "sank through the ground during the roll: {min_h}");
        // Heading 0 = north, so lat-gc rises with the northward displacement.
        assert!(m.get("position/lat-gc-deg") > 0.0, "should have rolled forward");
        assert!(m.get("velocities/vc-kts") > 10.0, "should have accelerated");
    }

    #[test]
    fn brakes_stop_the_roll() {
        let mut m = parked();
        m.set("fcs/throttle-cmd-norm", 1.0);
        for _ in 0..120 * 4 {
            m.step();
        }
        let vc_peak = m.get("velocities/vc-kts");
        assert!(vc_peak > 5.0, "should build speed before braking, got {vc_peak}");
        m.set("fcs/throttle-cmd-norm", 0.0);
        m.set("fcs/left-brake-cmd-norm", 1.0);
        m.set("fcs/right-brake-cmd-norm", 1.0);
        for _ in 0..120 * 4 {
            m.step();
        }
        let vc = m.get("velocities/vc-kts");
        assert!(vc < 0.5 * vc_peak, "brakes should slow the aircraft: {vc_peak} -> {vc}");
    }

    #[test]
    fn nosewheel_steers_left_with_rudder() {
        // With speed built up, a held left-rudder input should turn the aircraft
        // left (heading decreases). The rudder drives the nosewheel on the ground.
        let mut m = parked();
        m.set("fcs/throttle-cmd-norm", 1.0);
        for _ in 0..120 * 3 {
            m.step();
        }
        let psi0 = m.get("attitude/psi-deg");
        m.set("fcs/rudder-cmd-norm", 1.0);
        for _ in 0..120 * 3 {
            m.step();
        }
        let dpsi = m.get("attitude/psi-deg") - psi0;
        assert!(dpsi < -5.0, "left rudder should turn the nose left, dpsi = {dpsi}");
    }

    #[test]
    fn apply_ic_sets_level_attitude_at_programmed_state() {
        let mut m = Fdm::new("c172p");
        m.set("ic/vc-kts", 100.0);
        m.set("ic/h-sl-ft", 5000.0);
        m.set("ic/psi-true-deg", 90.0);
        m.apply_ic();
        assert!((m.get("position/h-sl-ft") - 5000.0).abs() < 1e-6);
        assert!((m.get("velocities/vc-kts") - 100.0).abs() < 1e-6);
        assert!((m.get("attitude/theta-deg")).abs() < 1e-9);
        assert!((m.get("attitude/psi-deg") - 90.0).abs() < 1e-9);
    }

    #[test]
    fn throttle_up_produces_climb() {
        // Mirrors JSBSim's `c172p_climbs_after_throttle_up` (jsbsim-sys): from
        // 120 kt at 3000 ft, full throttle for 2 s climbs and holds airspeed.
        // This is the shared acceptance check the two backends are compared on;
        // long-run stability needs the trim solver (task #25).
        let m = fly(240);
        let h = m.get("position/h-sl-ft");
        let vc = m.get("velocities/vc-kts");
        assert!(h > 3000.0, "expected climb, got h = {h}");
        assert!(vc > 90.0, "expected airspeed above 90 kt, got {vc}");
    }

    #[test]
    fn idle_throttle_descends() {
        let mut m = Fdm::new("c172p");
        m.set("ic/vc-kts", 100.0);
        m.set("ic/h-sl-ft", 3000.0);
        m.apply_ic();
        m.set("propulsion/set-running", -1.0);
        m.set("fcs/throttle-cmd-norm", 0.0);
        for _ in 0..120 * 20 {
            m.step();
        }
        assert!(m.get("position/h-sl-ft") < 3000.0, "should sink at idle");
    }

    #[test]
    fn aircraft_name_is_reported() {
        assert_eq!(Fdm::new("c172p").aircraft(), "c172p");
    }

    #[test]
    fn trim_holds_altitude_and_airspeed() {
        // A real trim is an exact fixed point of the integrator, so 30 s flown
        // open-loop should barely move.
        let mut m = Fdm::new("c172p");
        m.trim(120.0, 3000.0);
        let (h0, vc0) = (m.get("position/h-sl-ft"), m.get("velocities/vc-kts"));
        for _ in 0..120 * 30 {
            m.step();
        }
        let h = m.get("position/h-sl-ft");
        let vc = m.get("velocities/vc-kts");
        assert!((h - h0).abs() < 15.0, "trim drifted in altitude: {h0} -> {h}");
        assert!((vc - vc0).abs() < 1.0, "trim drifted in airspeed: {vc0} -> {vc}");
    }

    #[test]
    fn trim_converges_across_speeds() {
        // The solver should find a holding trim over the normal cruise band, not
        // just at one lucky speed.
        for vc in [85.0, 105.0, 120.0] {
            let mut m = Fdm::new("c172p");
            m.trim(vc, 5000.0);
            let h0 = m.get("position/h-sl-ft");
            for _ in 0..120 * 20 {
                m.step();
            }
            let dh = m.get("position/h-sl-ft") - h0;
            assert!(dh.abs() < 25.0, "trim at {vc} kt drifted {dh:.1} ft over 20 s");
        }
    }

    #[test]
    fn trim_nets_out_forces_and_moment() {
        // The solve should land on a genuine equilibrium: essentially wings-level
        // (a tiny bank balances side force), part throttle (cruise, not climb),
        // and no residual body rate.
        let mut m = Fdm::new("c172p");
        m.trim(120.0, 3000.0);
        assert!(
            m.get("attitude/phi-deg").abs() < 0.2,
            "trim should be near wings-level, phi = {}",
            m.get("attitude/phi-deg")
        );
        assert!(m.get("fcs/throttle-cmd-norm") < 1.0, "cruise trim should be part-throttle");
        assert!(m.get("velocities/q-rad_sec").abs() < 1e-6, "trim should have no pitch rate");
        // One step from the trim point leaves the attitude essentially unchanged.
        let theta0 = m.get("attitude/theta-deg");
        m.step();
        assert!((m.get("attitude/theta-deg") - theta0).abs() < 0.05);
    }

    #[test]
    fn trim_reachable_through_trait_object() {
        // The app holds a `Box<dyn FlightModel>`, so trim must be reachable
        // through the trait, not just as an inherent method.
        let mut m: Box<dyn FlightModel> = Box::new(Fdm::new("c172p"));
        m.trim(120.0, 3000.0);
        assert!((m.get("velocities/vc-kts") - 120.0).abs() < 1.0);
        assert!((m.get("position/h-sl-ft") - 3000.0).abs() < 1.0);
    }

    #[test]
    fn trim_tabs_hold_level_when_pilot_commands_spring_to_zero() {
        // The trim parks the lateral correction in the roll/yaw trim tabs, so the
        // pilot aileron/rudder commands springing back to zero every step (as the
        // app's springs do) no longer wipes it — otherwise the engine's reaction
        // torque rolls the aircraft away. The c172 is mildly spirally unstable, so
        // it still drifts slowly with the tabs; the tabs must make a big difference.
        let fly = |keep_tabs: bool| {
            let mut m = Fdm::new("c172p");
            m.trim(120.0, 3000.0);
            if !keep_tabs {
                m.set("fcs/roll-trim-cmd-norm", 0.0);
                m.set("fcs/yaw-trim-cmd-norm", 0.0);
            }
            let h0 = m.get("position/h-sl-ft");
            for _ in 0..120 * 30 {
                // Re-zero the pilot commands each step, as the spring keys do.
                m.set("fcs/aileron-cmd-norm", 0.0);
                m.set("fcs/rudder-cmd-norm", 0.0);
                m.step();
            }
            (m.get("attitude/phi-deg"), m.get("position/h-sl-ft") - h0)
        };

        let (phi_tabs, dh) = fly(true);
        let (phi_no_tabs, _) = fly(false);
        assert!(phi_tabs.abs() < 8.0, "with tabs phi drifted to {phi_tabs}");
        assert!(dh.abs() < 20.0, "with tabs altitude drifted {dh} ft");
        assert!(
            phi_no_tabs.abs() > 20.0,
            "without tabs the roll should be far worse, got {phi_no_tabs}"
        );
    }
}
