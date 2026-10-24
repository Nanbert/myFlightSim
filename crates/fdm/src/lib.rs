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
//! the modules that own them): ambient wind only via the property bus
//! (`atmosphere/wind-*-fps`, default calm) and `alphadot` fixed at 0.
//!
//! Kept free of Bevy, the filesystem, and threads so it compiles to `wasm32`.

mod aero;
mod aircraft;
mod arcade;
mod atmos;
mod engine;
mod fcs;
mod gear;
mod heli_arcade;
mod rotor;
mod state;
mod table;

use crate::aero::{AeroResult, PropMap};
use crate::aircraft::{ah1s::AH1S, b737::B737, c172::C172, f22::F22, Aircraft, INCH_TO_FT};
use crate::atmos::{Isa, G0};
use crate::engine::{EngineOutput, Powerplant};
use crate::fcs::{scale, ControlSystem, Fbw, FcsInputs, Surfaces};
use crate::gear::{ground_force, Gear, Ground};
use crate::rotor::{Rotor, RotorControlMap};
use crate::state::{add, norm, quat_rotate, rk4, sub, State, Vec3};
use flight_model::FlightModel;

const R2D: f64 = 180.0 / std::f64::consts::PI;
const D2R: f64 = std::f64::consts::PI / 180.0;
/// Knots <-> ft/s.
const KT_TO_FPS: f64 = 1.68781;
const FPS_TO_KT: f64 = 1.0 / KT_TO_FPS;
/// Feet per degree of latitude, chosen so the app's NED->world round-trip is
/// exact for its `EARTH_RADIUS_M = 6_378_137 m`: `(pi/180)*R/0.3048`.
/// `pub` because the app's `reset_final` seats its own approach-gate IC with
/// the same conversion the FDM publishes back.
pub const FT_PER_DEG_LAT: f64 = 365_221.79;
/// Landing-gear full travel time (s), down <-> up. Matches the gear kinematic in
/// the JSBSim models (both the F-22 and 737 traversals are 5 s).
const GEAR_CYCLE_S: f64 = 5.0;
/// Full defection *deploy* time of the 737's cockpit speedbrake handle (the
/// model's `<kinematic>` traverse: 0 over 0 s, 1 over 0.6 s).
const SPEEDBRAKE_TRAVEL_S: f64 = 0.6;
/// Thrust-reverse lever throw (the app's reverse toggle ramps over this throw).
const REVERSE_TRAVEL_S: f64 = 1.2;

/// A per-step snapshot of the aero/propulsion force and moment split, published
/// through the property bus so the validation harness can diff it against
/// JSBSim term by term.
#[derive(Clone, Copy, Default)]
struct Diag {
    alpha: f64,
    qbar: f64,
    cl: f64,
    force_aero: Vec3,
    moment_aero: Vec3,
    moment_prop: Vec3,
    thrust: f64,
}

/// Auto-hover / vertical-takeoff state machine for the arcade helicopter.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
enum HoverState {
    #[default]
    Off = 0,
    Takeoff = 1,
    Hold = 2,
    Land = 3,
}

/// A flight model instance for one aircraft.
pub struct Fdm {
    ac: &'static Aircraft,
    state: State,
    isa: Isa,
    power: Powerplant,
    /// Running fly-by-wire integrators. Unused (stays zero) for direct-FCS
    /// aircraft like the c172p.
    fbw: Fbw,
    /// Previous frame's lift coefficient squared, fed to `aero/cl-squared` for
    /// the induced-drag term (JSBSim's one-frame feedback).
    cl_sq: f64,
    /// The surfaces resolved on the last step, for read-back (`fcs/*-pos-rad`).
    surf: Surfaces,
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
    /// The rotors, main first then tail. Empty for fixed-wing aircraft.
    rotors: Vec<Rotor>,
    /// CG -> each rotor hub, body axes (ft), aligned with `ac.rotors`.
    rotor_arms_ft: Vec<Vec3>,
    // Pilot commands (normalized), written through `set`.
    elevator_cmd: f64,
    aileron_cmd: f64,
    rudder_cmd: f64,
    throttle_cmd: f64,
    collective_cmd: f64,
    collective_trim_cmd: f64,
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
    // Flight spoilers (the 737's speedbrake) and thrust reverse. Both are
    // command/position pairs: the position rolls toward the command at the
    // model's traverse rate. Spoilers add drag (CDsb) and cost wing lift
    // (kCLsb); reverse points the thrust backward, but only while the wheels
    // carry weight — airborne it is driven to stowed.
    speedbrake_cmd: f64,
    speedbrake_pos: f64,
    reverse_cmd: f64,
    reverse_pos: f64,
    // Programmed initial conditions (read by `apply_ic`).
    ic_vc_kts: f64,
    ic_alt_ft: f64,
    ic_phi_deg: f64,
    ic_theta_deg: f64,
    ic_psi_deg: f64,
    ic_beta_deg: f64,
    ic_p_rps: f64,
    ic_q_rps: f64,
    ic_r_rps: f64,
    // Last engine outputs the app can read back.
    engine_rpm: f64,
    engine_power_hp: f64,
    // True when any gear is in contact with the ground on the last step.
    wow: bool,
    // The previous step's wow / TVC-master / arcade-switch state, for the FBW
    // integrator reset at regime boundaries (see `poll_fbw_transitions`).
    fbw_prev_wow: bool,
    fbw_prev_tvc: bool,
    fbw_prev_arcade: bool,
    // Pilot master switch for thrust vectoring (`fcs/tvc-enable`). On by
    // default; only the F-22's FBW uses it.
    tvc_enable: bool,
    // Arcade mode (`fcs/arcade-enable`). Off by default; when on, `advance`
    // swaps the aero/FBW force resolution for the simplified [`arcade`] rules.
    arcade: bool,
    // Helicopter arcade helpers. The two pilot switches are `fcs/heli-hover-enable`
    // (vertical takeoff / hover / land) and `fcs/heli-balance-enable` (auto-level).
    heli_hover_cmd: bool,
    heli_hover_prev: bool,
    heli_balance: bool,
    heli_hover_state: HoverState,
    heli_hover_target_ft: f64,
    // Landing gear. `gear_cmd` is the pilot's selector (true = down); `gear_pos`
    // is the achieved position (1 = down, 0 = up) rolled toward the command at
    // the gear-cycle rate. Fixed-gear aircraft stay at 1.
    gear_cmd: bool,
    gear_pos: f64,
    // Hard altitude floor (ft above sea level) enforced while airborne, if set.
    // Missions use it as a sea-surface stop: with the gear up the model has no
    // belly contact, so nothing else prevents a dive through the water.
    floor_ft: Option<f64>,
    // Ambient wind, NED components (ft/s), written through the property bus
    // (`atmosphere/wind-{north,east,down}-fps`, JSBSim's names). The aero
    // evaluation and every airspeed readback see this subtracted from the
    // ground velocity; gravity, the integrator and the tire forces stay
    // ground-based, so parked friction and the runway contact are unaffected.
    wind_ned_fps: Vec3,
    // Fuel in the two wing tanks (lb), when the airframe declares a
    // `FuelSpec`. The engine burns it every step; whenever the contents have
    // moved ~0.2 lb the cached mass properties/tank arms are rebuilt (see
    // `rebuild_mass`), so the CG genuinely wanders as the tanks empty.
    // `fuel_built` is the snapshot these caches were built from.
    fuel: [f64; 2],
    fuel_built: [f64; 2],
    // Cold-start inputs the app feeds in: spark available
    // (`propulsion/ignition-enable`) and the held starter crank
    // (`propulsion/engine[0]/starter-cmd-norm`). A running engine needs neither.
    ignition: bool,
    starter_cmd: f64,
    // Last step's force/moment breakdown, for frame-by-frame comparison against
    // JSBSim (`forces/*`, `moments/*`).
    diag: Diag,
    // Flat-earth origin, so NED can be reported as lat/lon.
    origin_lat: f64,
    origin_lon: f64,
    // IC offsets from that origin (ft), set through `ic/lat-gc-deg` /
    // `ic/long-gc-deg`. Default 0 = start over the origin (the runway
    // threshold), as every parked reset does.
    ic_north_ft: f64,
    ic_east_ft: f64,
    dt: f64,
}

impl Fdm {
    /// Construct a model for `aircraft` (e.g. `"c172p"`, `"f22"`).
    pub fn new(aircraft: &str) -> Self {
        let ac: &'static Aircraft = match aircraft {
            "f22" => &F22,
            "ah1s" | "ah-1s" => &AH1S,
            "737" | "b737" => &B737,
            "c172p" | "c172" => &C172,
            _ => &C172,
        };
        let dt = 1.0 / 120.0;
        let cg_in = ac.loaded_cg_in();
        let prop_arm_ft = struct_to_body(ac.power.location_in(), cg_in);
        let rp_offset_ft = struct_to_body(ac.aero.aerorp_in, cg_in);
        let gear_arms_ft = ac
            .gear
            .iter()
            .map(|g| struct_to_body(g.pos_in, cg_in))
            .collect();
        let rotors: Vec<Rotor> = ac.rotors.iter().map(|s| Rotor::new(s, dt)).collect();
        let rotor_arms_ft = ac
            .rotors
            .iter()
            .map(|s| struct_to_body(s.location_in, cg_in))
            .collect();
        // Tanks start at the model's `<contents>`, which by convention equals
        // the static POINTS weights the arms above were built from — no rebuild
        // needed until the first burn.
        let fuel0 = ac.fuel.as_ref().map(|f| f.initial_lb).unwrap_or(0.0);
        let fuel = [fuel0, fuel0];
        Fdm {
            ac,
            state: State::default(),
            isa: Isa::new(),
            power: Powerplant::new(ac.power),
            fbw: Fbw::default(),
            cl_sq: 0.0,
            surf: Surfaces::default(),
            props: PropMap::new(),
            mass: ac.mass_slug(),
            inertia: ac.inertia(),
            prop_arm_ft,
            rp_offset_ft,
            gear_arms_ft,
            rotors,
            rotor_arms_ft,
            elevator_cmd: 0.0,
            aileron_cmd: 0.0,
            rudder_cmd: 0.0,
            throttle_cmd: 0.0,
            collective_cmd: 0.0,
            collective_trim_cmd: 0.0,
            pitch_trim_cmd: 0.0,
            left_brake_cmd: 0.0,
            right_brake_cmd: 0.0,
            roll_trim_cmd: 0.0,
            yaw_trim_cmd: 0.0,
            flap_cmd: 0.0,
            flap_pos_deg: 0.0,
            speedbrake_cmd: 0.0,
            speedbrake_pos: 0.0,
            reverse_cmd: 0.0,
            reverse_pos: 0.0,
            ic_vc_kts: 0.0,
            ic_alt_ft: 0.0,
            ic_phi_deg: 0.0,
            ic_theta_deg: 0.0,
            ic_psi_deg: 0.0,
            ic_beta_deg: 0.0,
            ic_p_rps: 0.0,
            ic_q_rps: 0.0,
            ic_r_rps: 0.0,
            engine_rpm: 0.0,
            engine_power_hp: 0.0,
            wow: false,
            fbw_prev_wow: false,
            fbw_prev_tvc: true,
            fbw_prev_arcade: false,
            tvc_enable: true,
            arcade: false,
            heli_hover_cmd: false,
            heli_hover_prev: false,
            heli_balance: false,
            heli_hover_state: HoverState::Off,
            heli_hover_target_ft: 0.0,
            gear_cmd: true,
            gear_pos: 1.0,
            floor_ft: None,
            wind_ned_fps: [0.0; 3],
            fuel,
            fuel_built: fuel,
            ignition: true,
            starter_cmd: 0.0,
            diag: Diag::default(),
            origin_lat: 0.0,
            origin_lon: 0.0,
            ic_north_ft: 0.0,
            ic_east_ft: 0.0,
            dt,
        }
    }

    /// The aircraft this model was built for.
    pub fn aircraft(&self) -> &str {
        self.ac.name
    }

    /// Recompute the cached mass properties and every CG-dependent arm from the
    /// current fuel contents. Fuel is the only thing that moves the mass
    /// points, so this runs whenever the tanks have moved enough since the last
    /// build — the edit is small (sub-slug) but keeps the CG drift and the
    /// inertia truthful as the tanks empty.
    fn rebuild_mass(&mut self) {
        let ac = self.ac;
        let sel = if ac.fuel.is_some() { Some(&self.fuel) } else { None };
        let loaded = ac.loaded_mass(sel);
        let cg = loaded.cg_in;
        self.prop_arm_ft = struct_to_body(ac.power.location_in(), cg);
        self.rp_offset_ft = struct_to_body(ac.aero.aerorp_in, cg);
        self.gear_arms_ft = ac
            .gear
            .iter()
            .map(|g| struct_to_body(g.pos_in, cg))
            .collect();
        self.rotor_arms_ft = ac
            .rotors
            .iter()
            .map(|s| struct_to_body(s.location_in, cg))
            .collect();
        self.mass = loaded.mass_slug;
        self.inertia = loaded.inertia;
    }

    /// Fill the wing tanks to capacity — the apron pump at MYFS. Purely a fuel
    /// action: a running engine doesn't care, and an empty one still needs the
    /// starter flow afterwards (refuelling never relights an engine by itself).
    pub fn refuel(&mut self) {
        let Some(spec) = self.ac.fuel.as_ref() else {
            return;
        };
        let cap = spec.capacity_lb;
        self.fuel = [cap, cap];
        self.fuel_built = self.fuel;
        self.rebuild_mass();
    }

    /// Height above the ground (ft), from the NED down-axis.
    fn alt_ft(&self) -> f64 {
        -self.state.pos[2]
    }

    /// Resolve the pilot commands (and the sensed body rates) into surface
    /// deflections. The `Direct` path is the c172p's command-to-surface mapping;
    /// the `Fbw` path is the F-22's rate-command loop, which advances its
    /// integrators by `dt`. Called once per step, so the surfaces are frozen
    /// across the RK4 stages — the same once-per-frame cadence JSBSim's FCS
    /// module runs at.
    fn resolve_surfaces(
        &mut self,
        mach: f64,
        rho: f64,
        pressure_psf: f64,
        thrust_lbs: f64,
        dt: f64,
    ) -> Surfaces {
        let ac = self.ac;
        let (u, v, w) = (self.state.vel[0], self.state.vel[1], self.state.vel[2]);
        let vt = (u * u + v * v + w * w).sqrt().max(1e-6);
        let (alpha, _) = air_angles(u, v, w);
        let qbar = 0.5 * rho * vt * vt;
        let speedbrake_pos = if ac.has_speedbrake {
            self.speedbrake_pos
        } else {
            0.0
        };
        let inputs = FcsInputs {
            p: self.state.omega[0],
            q: self.state.omega[1],
            r: self.state.omega[2],
            alpha,
            mach,
            qbar,
            pressure_psf,
            wow: self.wow,
            gear_down: self.gear_pos > 0.5,
            tvc_enable: self.tvc_enable,
            elevator_cmd: self.elevator_cmd,
            aileron_cmd: self.aileron_cmd,
            rudder_cmd: self.rudder_cmd,
            pitch_trim_cmd: self.pitch_trim_cmd,
            roll_trim_cmd: self.roll_trim_cmd,
            yaw_trim_cmd: self.yaw_trim_cmd,
            flap_pos_deg: self.flap_pos_deg,
            thrust_lbs,
            engine_count: ac.power.engine_count(),
            collective_cmd: self.collective_cmd,
            collective_trim_cmd: self.collective_trim_cmd,
        };
        let mut surf = ac.control.resolve(&mut self.fbw, &inputs, dt);
        // The resolved position drives the CDsb drag / kCLsb lift factors. The
        // direct and FBW laws never set this field, so it would otherwise read
        // the struct's zero default and the boards would never bite.
        surf.speedbrake_norm = speedbrake_pos;
        surf
    }

    /// Roll the flaps toward the commanded deflection at the model's flap rate.
    /// Fly-by-wire aircraft (F-22) have no trailing-edge flaps in this model.
    fn step_flap(&mut self, dt: f64) {
        let ControlSystem::Direct(fcs) = &self.ac.control else {
            return;
        };
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

    /// Roll the landing gear toward its commanded position. Fixed-gear aircraft
    /// stay down; retractable ones move at the gear-cycle rate, and retraction
    /// is inhibited on the ground (the weight-on-wheels interlock), so the gear
    /// only stows in flight — as on the real jet.
    fn step_gear(&mut self, dt: f64) {
        let down = !self.ac.gear_retractable || (self.wow && !self.gear_cmd) || self.gear_cmd;
        let target = if down { 1.0 } else { 0.0 };
        let step = dt / GEAR_CYCLE_S;
        let err = target - self.gear_pos;
        self.gear_pos = if err.abs() <= step {
            target
        } else {
            self.gear_pos + step * err.signum()
        };
    }

    /// Drive the flight spoilers (the 737's speedbrake) and the thrust reverse
    /// through their command/position pairs.
    ///
    /// Spoilers take 0.6 s of full travel, straight off the model's kinematic
    /// traverse. Reverse is **ground-interlocked**: the position rolls toward
    /// the command only while weight rests on the wheels, and an airborne
    /// command is driven to stowed — so a reverse left half-way through a
    /// touch-and-go fails closed instead of spooling the engines backwards in
    /// the flare. Reverse travel takes 1.2 s (the real lever throw).
    fn step_spoilers(&mut self, dt: f64) {
        if self.ac.has_speedbrake {
            let step = dt / SPEEDBRAKE_TRAVEL_S;
            let err = self.speedbrake_cmd.clamp(0.0, 1.0) - self.speedbrake_pos;
            self.speedbrake_pos = if err.abs() <= step {
                self.speedbrake_pos + err
            } else {
                self.speedbrake_pos + step * err.signum()
            };
        }
        let rev_target = if self.wow {
            self.reverse_cmd.clamp(0.0, 1.0)
        } else {
            0.0
        };
        let rev_step = dt / REVERSE_TRAVEL_S;
        let rev_err = rev_target - self.reverse_pos;
        self.reverse_pos = if rev_err.abs() <= rev_step {
            self.reverse_pos + rev_err
        } else {
            self.reverse_pos + rev_step * rev_err.signum()
        };
    }

    /// Drive the arcade helicopter's vertical state machine. Returns
    /// `(vz_target_fps, lift_enabled)`.
    fn step_heli_hover(&mut self, alt_ft: f64) -> (f64, bool) {
        // `wow` is only valid after a step; the first hover press can arrive
        // while the model is still seated at its parked IC, so also treat the
        // parked height as ground.
        let on_ground = self.wow || alt_ft <= self.ac.rest_h_ft + 2.0;
        if self.heli_hover_cmd != self.heli_hover_prev {
            if self.heli_hover_cmd {
                if on_ground || self.heli_hover_state == HoverState::Land {
                    self.heli_hover_state = HoverState::Takeoff;
                    self.heli_hover_target_ft = alt_ft + 15.0;
                } else {
                    self.heli_hover_state = HoverState::Hold;
                    self.heli_hover_target_ft = alt_ft;
                }
            } else if self.heli_hover_state != HoverState::Off {
                self.heli_hover_state = HoverState::Land;
            }
            self.heli_hover_prev = self.heli_hover_cmd;
        }

        if self.heli_hover_state == HoverState::Takeoff && alt_ft >= self.heli_hover_target_ft - 0.8
        {
            self.heli_hover_state = HoverState::Hold;
        }
        if self.heli_hover_state == HoverState::Land && (self.wow || alt_ft <= 0.2) {
            self.heli_hover_state = HoverState::Off;
            self.heli_hover_cmd = false;
            self.heli_hover_prev = false;
        }

        let vz = match self.heli_hover_state {
            HoverState::Takeoff => ((self.heli_hover_target_ft - alt_ft) * 0.9).clamp(1.0, 8.0),
            HoverState::Hold => ((self.heli_hover_target_ft - alt_ft) * 0.8).clamp(-4.0, 4.0),
            HoverState::Land => {
                if alt_ft < 6.0 {
                    ((-alt_ft) * 0.55).clamp(-2.0, 0.5)
                } else {
                    ((-alt_ft) * 0.45).clamp(-5.0, -0.5)
                }
            }
            HoverState::Off => (self.collective_cmd - 0.5) * 2.0 * heli_arcade::MAX_MANUAL_VZ,
        };
        let lift =
            !on_ground || self.heli_hover_state != HoverState::Off || self.collective_cmd > 0.52;
        (vz, lift)
    }

    /// Wipe the fly-by-wire integrators when the FDM crosses a regime boundary:
    /// TVC toggled, weight transitions on/off the wheels, or the arcade switch
    /// flipped. The integrators are one-frame controller state, not physical
    /// state — a value wound up under the old regime (e.g. the pitch integral
    /// that was holding the jet level on the takeoff roll) would fire the
    /// surfaces the instant the new regime starts, so each regime starts from
    /// scratch. Non-FBW aircraft keep zeros; the poll is a no-op for them.
    fn poll_fbw_transitions(&mut self) {
        if matches!(self.ac.control, ControlSystem::Fbw(_))
            && (self.wow != self.fbw_prev_wow
                || self.tvc_enable != self.fbw_prev_tvc
                || self.arcade != self.fbw_prev_arcade)
        {
            self.fbw = Fbw::default();
        }
        self.fbw_prev_wow = self.wow;
        self.fbw_prev_tvc = self.tvc_enable;
        self.fbw_prev_arcade = self.arcade;
    }

    /// Advance the whole model one fixed timestep.
    fn advance(&mut self, dt: f64) {
        // Detect any FBW regime boundary this step first, so the loop below
        // runs on a clean control-law state and the boundary itself is recorded
        // before anything else reads it.
        self.poll_fbw_transitions();
        // Gear travels toward the commanded position first, so this step's LEF
        // schedule, aero drag and ground reaction all see the same position.
        self.step_gear(dt);
        self.step_spoilers(dt);
        let alt = self.alt_ft();
        let heli_mode = self.arcade && !self.rotors.is_empty();
        let (heli_vz_target, heli_lift) = if heli_mode {
            self.step_heli_hover(alt)
        } else {
            (0.0, false)
        };
        let rho = self.isa.density(alt);
        let pressure = self.isa.pressure(alt);
        let temp = self.isa.temperature(alt);
        let sound = self.isa.sound_speed(alt);

        // Airspeeds are air-relative: the propeller and the Mach tables see the
        // vehicle's motion through the air mass, not over the ground.
        let air = self.air_vel_body();
        let vt = norm(air).max(1e-6);
        let mach = vt / sound.max(1.0);

        // Powerplant spins up once per step (quasi-steady); its thrust, induced
        // slipstream, and reaction torque then hold constant over the step, the
        // same once-per-frame cadence JSBSim's propulsion module uses. The
        // turbofan's thrust tables are indexed by Mach, so it needs the step's
        // Mach number too.
        // Fuel state first: push the ignition/starter commands to the engine
        // and starve it once the tanks run dry, so this step's solve sees the
        // correct fuel situation.
        if self.ac.fuel.is_some() {
            let starved = self.fuel[0] + self.fuel[1] <= 0.0;
            self.power.set_starved(starved);
            self.power.set_ignition(self.ignition);
            self.power.set_starter(self.starter_cmd);
        }
        let out = self.power.step(
            dt,
            self.throttle_cmd,
            rho,
            pressure,
            temp,
            air[0],
            sound,
            mach,
        );
        self.engine_rpm = out.rpm;
        self.engine_power_hp = out.power_hp;
        // Deduct this step's burn symmetrically from the two tanks and rebuild
        // the mass caches once the contents have moved ~0.2 lb (roughly every
        // ten seconds at full power — cheap machinery, honest physics).
        if self.ac.fuel.is_some() {
            let burn_lb = dt * self.power.fuel_flow_pph() / 3600.0;
            for f in &mut self.fuel {
                *f = (*f - 0.5 * burn_lb).max(0.0);
            }
            let moved = (self.fuel[0] - self.fuel_built[0])
                .abs()
                .max((self.fuel[1] - self.fuel_built[1]).abs());
            if moved > 0.2 {
                self.fuel_built = self.fuel;
                self.rebuild_mass();
            }
        }
        // Thrust reverse: back up the exhaust while the wheels carry weight. The
        // honest cell for that force is exactly where forward thrust lands —
        // net body-axis force and its moment arm both take the difference, so
        // the RK4 stages, trim ctx and diag all see one consistent value.
        // Airborne (reverse already stepped to stowed) it is simply zero.
        let reverse_thrust = if self.ac.max_reverse_lbs > 0.0 {
            self.ac.max_reverse_lbs
                * self.ac.power.engine_count() as f64
                * self.reverse_pos
        } else {
            0.0
        };
        let thrust = out.thrust - reverse_thrust;
        let torque_x = out.torque_x;
        let induced = out.induced_velocity;
        // Thruster moment about the CG: the propeller's reaction torque about
        // the roll axis, plus the thrust's off-axis arm (M = r x F, F = [T,0,0]).
        let arm = self.prop_arm_ft;
        let mprop = [torque_x, arm[2] * thrust, -arm[1] * thrust];

        // Resolve the surfaces once per step (the FBW advances its integrators
        // here), so every RK4 stage flies the same frozen deflection.
        let surf = self.resolve_surfaces(mach, rho, pressure, thrust, dt);
        self.step_flap(dt);

        // Rotors: governor-lock the main rotor (the tail rotor follows through
        // the gear ratio), write this step's controls, and run one
        // momentum-theory solve. Like thrust, the resulting body force/moment is
        // frozen across the RK4 stages; only the inflow ratio `nu` integrates,
        // and it advances once per step here.
        let (rotor_force, rotor_moment) = self.step_rotors(&surf, alt, rho, dt);

        let ac = self.ac;
        let mass = self.mass;
        let inertia = self.inertia;
        let state = self.state;
        let rp_offset = self.rp_offset_ft;
        // Ground-reaction inputs: the gear layout, its precomputed arms, the
        // nosewheel steer angle, and the brake channels — held constant over the
        // step (the steer angle comes from the rudder command). With the gear
        // retracted there is nothing to touch the ground, so no contact is
        // summed (a gear-up descent is simply unsupported).
        let gear_pos = self.gear_pos;
        let gears: &[Gear] = if gear_pos > 0.99 { ac.gear } else { &[] };
        let arms = &self.gear_arms_ft;
        let rudder = self.rudder_cmd;
        let brake = [self.left_brake_cmd, self.right_brake_cmd];
        // Arcade ("game") mode: snapshot its inputs here, since the closure below
        // can't borrow `self`. The pinning gate is decided from this step's own
        // airspeed and weight-on-wheels — off on the ground, so the gear and
        // steering own the jet on the takeoff roll.
        let arcade = self.arcade;
        let arcade_cfg = arcade::Cfg {
            elevator: self.elevator_cmd,
            aileron: self.aileron_cmd,
            rudder: self.rudder_cmd,
            thrust,
            mass,
            align: arcade && !self.wow && vt > arcade::ALIGN_MIN_KT * KT_TO_FPS,
        };
        let heli_cfg = if heli_mode {
            Some(heli_arcade::Cfg {
                elevator: self.elevator_cmd,
                aileron: self.aileron_cmd,
                rudder: self.rudder_cmd,
                collective: self.collective_cmd,
                mass,
                balance: self.heli_balance,
                on_ground: self.wow,
                vz_target: heli_vz_target,
                lift: heli_lift,
            })
        } else {
            None
        };
        // Reuse the scratch map's allocation across steps; take it out of `self`
        // so the aero closure can hold it mutably without borrowing all of `self`.
        let mut props = std::mem::take(&mut self.props);
        // The induced-drag term reads last frame's lift coefficient (JSBSim's
        // one-frame feedback, avoiding a same-frame circular dependency).
        props.set("aero/cl-squared", self.cl_sq);
        // The rotor advance ratio (JSBSim aliases `propulsion/engine/advance-ratio`
        // as `aero/mu`). It drives the AH-1S tail pitch stiffness/damping curves;
        // frozen with the rotor force for this step, like every other rotor output.
        props.set("aero/mu", self.rotor_get(0, |r| r.advance_ratio()));

        // Aero is recomputed at each RK4 stage from the trial state; thrust and
        // its moment are constant over the step. The snapshot keeps the last
        // stage's aero (nearest the end-of-step state) for the property bus.
        let mut diag = Diag::default();
        // Ground reaction is also recomputed per stage; keep the last stage's.
        let mut ground = Ground::default();
        let wind_ned = self.wind_ned_fps;
        let next = rk4(&state, dt, mass, &inertia, |s: &State| {
            fill_props(
                &mut props,
                ac,
                s,
                &surf,
                induced,
                rho,
                sound,
                s.ned_to_body(wind_ned),
            );
            props.set("gear/gear-pos-norm", gear_pos);
            let aero = ac.aero.evaluate(&mut props, rp_offset);
            ground = ground_force(s, gears, arms, rudder, brake, dt);
            diag = Diag {
                alpha: props.get("aero/alpha-rad"),
                qbar: props.get("aero/qbar-psf"),
                cl: aero.cl,
                force_aero: aero.force_body,
                moment_aero: aero.moment_body,
                moment_prop: mprop,
                thrust,
            };
            if let Some(hcfg) = &heli_cfg {
                return heli_arcade::force_moment(
                    hcfg,
                    s,
                    &inertia,
                    ground.force_body,
                    ground.moment_body,
                );
            }
            if arcade {
                // Only the airframe's axial (drag) force survives; the arcade
                // rules supply the rate-command moment and the pinning force.
                return arcade::force_moment(
                    &arcade_cfg,
                    s,
                    aero.force_body[0],
                    &inertia,
                    ground.force_body,
                    ground.moment_body,
                );
            }
            (
                [
                    aero.force_body[0] + thrust + ground.force_body[0] + rotor_force[0],
                    aero.force_body[1] + ground.force_body[1] + rotor_force[1],
                    aero.force_body[2] + ground.force_body[2] + rotor_force[2],
                ],
                [
                    aero.moment_body[0] + torque_x + ground.moment_body[0] + rotor_moment[0],
                    aero.moment_body[1] + mprop[1] + ground.moment_body[1] + rotor_moment[1],
                    aero.moment_body[2] + mprop[2] + ground.moment_body[2] + rotor_moment[2],
                ],
            )
        });
        self.props = props;
        self.cl_sq = diag.cl * diag.cl;
        self.diag = diag;
        self.surf = surf;
        self.wow = ground.wow;
        self.state = next;

        // Hard floor above sea level, a catch for a dive the mission assist
        // could not recover. Skipped with the gear down so runway landings still
        // settle onto the wheels; the descending NED rate is killed at the stop
        // so a pinned dive does not keep integrating toward the sea.
        if let Some(floor) = self.floor_ft {
            if self.gear_pos < 0.99 && self.alt_ft() < floor {
                self.state.pos[2] = -floor;
                let mut v = self.state.body_to_ned(self.state.vel);
                if v[2] > 0.0 {
                    v[2] = 0.0;
                    self.state.vel = self.state.ned_to_body(v);
                }
            }
        }
    }

    /// Vertical speed (ft/s), positive up: `-z_dot` in NED.
    fn h_dot_fps(&self) -> f64 {
        -self.state.body_to_ned(self.state.vel)[2]
    }

    /// Air-relative velocity in body axes (ft/s): the ground velocity less the
    /// ambient wind rotated into the body frame. The aero evaluation, the
    /// propeller inflow and the rotors all see this.
    fn air_vel_body(&self) -> Vec3 {
        sub(self.state.vel, self.state.ned_to_body(self.wind_ned_fps))
    }

    /// Air-relative true airspeed (ft/s).
    fn air_airspeed_fps(&self) -> f64 {
        norm(self.air_vel_body())
    }

    /// Air-relative sideslip (rad), same convention as `aero/beta-deg`. Held
    /// at zero near a standstill (see `air_angles`).
    fn air_beta_rad(&self) -> f64 {
        let air = self.air_vel_body();
        if norm(air) < AIR_ANGLES_MIN_FPS {
            0.0
        } else {
            air[1].atan2(air[0])
        }
    }

    /// Re-seat the vehicle *velocity* for a wind-aware world: the trims and ICs
    /// solve and express velocities in the air frame, but the integrator works
    /// on ground velocity, so the ambient wind is added back at seat time. A
    /// no-op in still air (the default).
    fn seat_air_frame(&mut self) {
        let quat = self.state.quat;
        let air_ned = quat_rotate(quat, self.state.vel);
        self.state.vel = self.state.ned_to_body(add(air_ned, self.wind_ned_fps));
    }

    /// Read a rotor output by index, or 0.0 when there's no such rotor
    /// (fixed-wing aircraft carry none).
    fn rotor_get(&self, i: usize, f: impl Fn(&Rotor) -> f64) -> f64 {
        self.rotors.get(i).map_or(0.0, f)
    }

    /// Spin the rotors one step and recompute their body force and moment.
    ///
    /// The governor holds the main rotor at the engine's RPM; the tail rotor
    /// rides the same shaft through the gear ratios
    /// (`rpm_tail = rpm_main · gear_main / gear_tail`). Each rotor gets this
    /// step's control angles from the resolved surfaces, then a single
    /// momentum-theory solve. The summed force and moment (the hub-arm moment
    /// already folded in by the rotor) are frozen across the RK4 stages, exactly
    /// like thrust. A no-op for fixed-wing aircraft (no rotors).
    fn step_rotors(&mut self, surf: &Surfaces, h_agl: f64, rho: f64, dt: f64) -> (Vec3, Vec3) {
        if self.rotors.is_empty() {
            return ([0.0; 3], [0.0; 3]);
        }
        // The rotors see the air-relative velocity too (the AH-1S's advance
        // ratio and flapping time-constant all key on it).
        let uvw = self.air_vel_body();
        let pqr = self.state.omega;
        let rpm_main = self.engine_rpm.max(0.0);
        let main_gear = self.rotors[0].spec().gear_ratio;
        let mut force = [0.0; 3];
        let mut moment = [0.0; 3];
        for (i, rotor) in self.rotors.iter_mut().enumerate() {
            let (rpm, col, lon, lat) = match rotor.spec().control_map {
                RotorControlMap::Main => (
                    rpm_main,
                    surf.collective_rad,
                    surf.longitudinal_cyc_rad,
                    surf.lateral_cyc_rad,
                ),
                RotorControlMap::Tail => (
                    rpm_main * (main_gear / rotor.spec().gear_ratio),
                    surf.antitorque_rad,
                    0.0,
                    0.0,
                ),
            };
            rotor.set_rpm(rpm);
            rotor.set_controls(col, lon, lat);
            rotor.calculate(uvw, pqr, self.rotor_arms_ft[i], rho, h_agl, dt);
            force = add(force, rotor.force_body());
            moment = add(moment, rotor.moment_body());
        }
        (force, moment)
    }

    /// Evaluate the aero forces/moments and the (steady) powerplant at a trial
    /// trim point: angle of attack `alpha`, bank `phi`, a resolved `surf`, and
    /// throttle. Returns the aero result, the engine output, and gravity in body
    /// axes. `cl_sq` seeds the induced-drag feedback (0 for a first pass).
    /// Shared by the longitudinal and lateral trim residuals. `phi` tilts
    /// gravity (and so the lateral force) — the bank that balances side force,
    /// matching JSBSim's `tFull` trim.
    #[allow(clippy::too_many_arguments)]
    fn trim_eval_surf(
        &self,
        alpha: f64,
        phi: f64,
        surf: &Surfaces,
        throttle: f64,
        vt: f64,
        alt: f64,
        cl_sq: f64,
    ) -> (AeroResult, EngineOutput, Vec3) {
        let rho = self.isa.density(alt);
        let pressure = self.isa.pressure(alt);
        let temp = self.isa.temperature(alt);
        let sound = self.isa.sound_speed(alt);

        // The prop sees the body-forward airspeed u = V cos(alpha).
        let u = vt * alpha.cos();
        let mach = vt / sound.max(1.0);
        let eng = self
            .power
            .steady_output(throttle, rho, pressure, temp, u, sound, mach);

        // Flight-path-level, sideslip-free: pitch attitude equals alpha, bank is
        // `phi` (body velocity stays [u, 0, w], so alpha is unaffected by bank).
        let state = State {
            pos: [0.0, 0.0, -alt],
            vel: [u, 0.0, vt * alpha.sin()],
            quat: euler_to_quat(phi, alpha, 0.0),
            omega: [0.0; 3],
        };
        let mut props = PropMap::new();
        props.set("aero/cl-squared", cl_sq);
        // Trim the *live* configuration, so a trim seat is a true fixed point
        // (gear drag and the LEF schedule depend on the gear position).
        fill_props(
            &mut props,
            self.ac,
            &state,
            surf,
            eng.induced_velocity,
            rho,
            sound,
            // The trim frame *is* the air frame: `vt` here is an airspeed, so
            // the wind must not be subtracted again.
            [0.0; 3],
        );
        props.set("gear/gear-pos-norm", self.gear_pos);
        let aero = self.ac.aero.evaluate(&mut props, self.rp_offset_ft);
        let grav = state.ned_to_body([0.0, 0.0, G0]);
        (aero, eng, grav)
    }

    /// Direct-FCS trim evaluation (c172p/737): build the surfaces from the pilot
    /// commands the way the live loop does, then evaluate. `cl_sq` seeds the
    /// induced-drag feedback (0 for a first pass).
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
        cl_sq: f64,
    ) -> (AeroResult, EngineOutput, Vec3) {
        let ControlSystem::Direct(fcs) = &self.ac.control else {
            unreachable!("trim_eval (direct) called on a fly-by-wire aircraft")
        };
        let elevator_rad = scale(trim_cmd, fcs.elevator_lim_deg) * fcs.elevator_gain;
        let aileron_rad = scale(aileron_cmd, fcs.aileron_lim_deg) * fcs.aileron_gain;
        // The flap rides the live achieved position, the same live-config
        // rule `trim_eval_surf` uses for the gear: a seat solved with the
        // flaps retracted while the wing flies flapped is not a trim point.
        let flap_norm = if fcs.flap_max_deg > 0.0 {
            self.flap_pos_deg / fcs.flap_max_deg
        } else {
            0.0
        };
        let surf = Surfaces {
            elevator_rad,
            mag_elevator_rad: elevator_rad.abs(),
            aileron_rad,
            right_aileron_rad: -aileron_rad,
            rudder_rad: scale(rudder_cmd, fcs.rudder_lim_deg) * fcs.rudder_gain,
            flap_deg: self.flap_pos_deg,
            flap_norm,
            ..Default::default()
        };
        self.trim_eval_surf(alpha, phi, &surf, throttle, vt, alt, cl_sq)
    }

    /// Longitudinal residual `[Fx, Fz, My]` (weight-normalized), zero at trim.
    /// `x = [alpha_rad, pitch_trim_cmd, throttle]`.
    fn trim_long_residual(&self, x: [f64; 3], vt: f64, alt: f64) -> [f64; 3] {
        // Bootstrap the induced-drag feedback: the CDi term keys on CL², unknown
        // until alpha is evaluated, so evaluate once for CL and reseed (the same
        // one-pass trick the FBW path uses). Without it the trim omits induced
        // drag and the seat drifts — visible on the 737, whose CDi is real.
        let (aero0, _, _) = self.trim_eval(x[0], 0.0, 0.0, 0.0, x[1], x[2], vt, alt, 0.0);
        let (aero, eng, grav) = self.trim_eval(
            x[0],
            0.0,
            0.0,
            0.0,
            x[1],
            x[2],
            vt,
            alt,
            aero0.cl * aero0.cl,
        );
        let weight = self.mass * G0;
        let fx = aero.force_body[0] + eng.thrust + grav[0] * self.mass;
        let fz = aero.force_body[2] + grav[2] * self.mass;
        let my = aero.moment_body[1] + self.prop_arm_ft[2] * eng.thrust;
        [
            fx / weight,
            fz / weight,
            my / (weight * self.ac.wing_chord_ft),
        ]
    }

    /// Lateral residual `[Fy, Mx, Mz]` (weight- and weight-span-normalized), zero
    /// at trim, with the longitudinal unknowns held at their solved values.
    /// `x = [phi_bank, aileron_cmd, rudder_cmd]`. The bank balances side force;
    /// the aileron/rudder null the roll and yaw moments, the roll moment
    /// absorbing the engine's reaction torque (a real c172 counters it with
    /// aileron trim).
    fn trim_lat_residual(&self, x: [f64; 3], long: [f64; 3], vt: f64, alt: f64) -> [f64; 3] {
        let (aero0, _, _) =
            self.trim_eval(long[0], x[0], x[1], x[2], long[1], long[2], vt, alt, 0.0);
        let (aero, eng, grav) = self.trim_eval(
            long[0],
            x[0],
            x[1],
            x[2],
            long[1],
            long[2],
            vt,
            alt,
            aero0.cl * aero0.cl,
        );
        let weight = self.mass * G0;
        let scale = weight * self.ac.wing_span_ft;
        let fy = aero.force_body[1] + grav[1] * self.mass;
        [
            fy / weight,
            (aero.moment_body[0] + eng.torque_x) / scale,
            aero.moment_body[2] / scale,
        ]
    }

    /// Longitudinal residual for a fly-by-wire aircraft, in the same
    /// weight-normalized form as the direct path but with the elevator *surface
    /// angle* as the direct unknown (the rate-command loop has no trim-command
    /// channel to solve). `x = [alpha_rad, elevator_rad, throttle]`. The
    /// induced-drag feedback is bootstrapped in one pass (evaluate, then reseed
    /// with the resulting CL²) since the trim solves alpha and drag together.
    fn fbw_long_residual(&self, x: [f64; 3], vt: f64, alt: f64) -> [f64; 3] {
        let spec = match self.ac.control {
            ControlSystem::Fbw(s) => s,
            ControlSystem::Direct(_) | ControlSystem::Heli(_) => {
                unreachable!("fbw_long_residual called on a non-FBW aircraft")
            }
        };
        let (alpha, elevator_rad, throttle) = (x[0], x[1], x[2]);
        let rho = self.isa.density(alt);
        let pressure = self.isa.pressure(alt);
        let sound = self.isa.sound_speed(alt);
        let mach = vt / sound.max(1.0);
        let qbar = 0.5 * rho * vt * vt;
        let surf = Surfaces {
            elevator_rad,
            mag_elevator_rad: elevator_rad.abs(),
            lef_rad: spec.lef_rad(alpha, qbar, pressure, false, mach),
            ..Default::default()
        };
        let (aero0, eng, grav) = self.trim_eval_surf(alpha, 0.0, &surf, throttle, vt, alt, 0.0);
        let (aero, _, _) =
            self.trim_eval_surf(alpha, 0.0, &surf, throttle, vt, alt, aero0.cl * aero0.cl);
        let weight = self.mass * G0;
        let fx = aero.force_body[0] + eng.thrust + grav[0] * self.mass;
        let fz = aero.force_body[2] + grav[2] * self.mass;
        let my = aero.moment_body[1] + self.prop_arm_ft[2] * eng.thrust;
        [
            fx / weight,
            fz / weight,
            my / (weight * self.ac.wing_chord_ft),
        ]
    }

    /// Residual for the helicopter trim, weight-normalized and zero at trim: the
    /// six body force/moment components `[Fx, Fy, Fz, Mx, My, Mz]` at the target
    /// flight condition. `x` is `[collective, longitudinal, lateral, pedal, phi,
    /// theta]` — the four pilot commands plus the pitch/roll attitude. `v_ned` is
    /// the target velocity in NED (zero = hover); the trial attitude carries it
    /// into the body frame, so the rotor and airframe see the right airspeed and
    /// angle of attack. Aero vanishes at hover (qbar → 0), so this reduces to the
    /// old hover residual there.
    fn trim_heli_residual(&mut self, x: [f64; 6], alt: f64, v_ned: [f64; 3]) -> [f64; 6] {
        self.collective_cmd = x[0];
        self.elevator_cmd = x[1];
        self.aileron_cmd = x[2];
        self.rudder_cmd = x[3];
        // Heading doesn't enter the body-frame balance, but write the caller's so
        // the trim seats on it (and the final inflow-settle passes don't clobber
        // it with 0).
        self.state.quat = euler_to_quat(x[4], x[5], self.ic_psi_deg * D2R);
        self.state.vel = self.state.ned_to_body(v_ned);
        self.state.omega = [0.0; 3];

        let rho = self.isa.density(alt);
        let pressure = self.isa.pressure(alt);
        let sound = self.isa.sound_speed(alt);
        let mach = self.state.airspeed() / sound.max(1.0);
        let surf = self.resolve_surfaces(mach, rho, pressure, 0.0, self.dt);
        let (rf, rm) = self.step_rotors(&surf, alt, rho, self.dt);

        // Airframe aero: the fuselage drag and fin weathercock (this model has no
        // wing lift — the rotor carries the weight).
        let mut props = PropMap::new();
        props.set("aero/cl-squared", self.cl_sq);
        props.set("aero/mu", self.rotor_get(0, |r| r.advance_ratio()));
        fill_props(
            &mut props,
            self.ac,
            &self.state,
            &surf,
            0.0,
            rho,
            sound,
            self.state.ned_to_body(self.wind_ned_fps),
        );
        props.set("gear/gear-pos-norm", self.gear_pos);
        let aero = self.ac.aero.evaluate(&mut props, self.rp_offset_ft);

        let grav = self.state.ned_to_body([0.0, 0.0, G0]);
        let mass = self.mass;
        let weight = mass * G0;
        [
            (aero.force_body[0] + rf[0] + grav[0] * mass) / weight,
            (aero.force_body[1] + rf[1] + grav[1] * mass) / weight,
            (aero.force_body[2] + rf[2] + grav[2] * mass) / weight,
            (aero.moment_body[0] + rm[0]) / (weight * self.ac.wing_span_ft),
            (aero.moment_body[1] + rm[1]) / (weight * self.ac.wing_chord_ft),
            (aero.moment_body[2] + rm[2]) / (weight * self.ac.wing_span_ft),
        ]
    }

    /// Trim a helicopter at the commanded airspeed. Newton-solve the four pilot
    /// commands — collective, longitudinal cyclic, lateral cyclic, pedal —
    /// together with the pitch and roll attitude, so the six body force/moment
    /// components vanish, then seat the model on the solved point. `vc_kts = 0`
    /// is a hover; a positive value is level flight at that airspeed, along the
    /// current heading (`ic_psi_deg`). The result is in place: the current ground
    /// track is preserved, and the trimmed velocity keeps the commanded airspeed
    /// rather than being reset to zero.
    fn trim_heli(&mut self, vc_kts: f64, alt_ft: f64) {
        let rho = self.isa.density(alt_ft);
        // Light the rotor drive (the governor then holds nominal RPM) and seed
        // the engine outputs, so the rotor spins throughout the solve.
        let eng = self.power.steady_at(
            1.0,
            rho,
            self.isa.pressure(alt_ft),
            self.isa.temperature(alt_ft),
            0.0,
            self.isa.sound_speed(alt_ft),
            0.0,
        );
        self.engine_rpm = eng.rpm;
        self.engine_power_hp = eng.power_hp;

        // Target velocity: level flight along the current heading at the
        // commanded (calibrated) airspeed. Zero keeps it a hover.
        let tas = self.isa.cas_to_tas(vc_kts * KT_TO_FPS, alt_ft);
        let psi = self.ic_psi_deg * D2R;
        let v_ned = [tas * psi.cos(), tas * psi.sin(), 0.0];

        // Keep the current ground track (x, y) so a trim taken airborne doesn't
        // teleport the aircraft back over the origin.
        let [px, py, _] = self.state.pos;

        // Seat the state at the target altitude with the heading; the residual
        // writes the trial attitude and body velocity into it each pass.
        self.state = State {
            pos: [px, py, -alt_ft],
            vel: [0.0; 3],
            quat: euler_to_quat(0.0, 0.0, psi),
            omega: [0.0; 3],
        };

        const H: f64 = 1e-4;
        let nrm = |r: [f64; 6]| (r.iter().map(|v| v * v).sum::<f64>()).sqrt();
        let mut x = [0.5, 0.0, 0.0, 0.0, 0.0, 0.0];
        for _ in 0..60 {
            let r = self.trim_heli_residual(x, alt_ft, v_ned);
            let rnorm = nrm(r);
            if rnorm < 1e-8 {
                break;
            }
            let mut j = [[0.0; 6]; 6];
            for k in 0..6 {
                let mut xp = x;
                xp[k] += H;
                let rp = self.trim_heli_residual(xp, alt_ft, v_ned);
                for i in 0..6 {
                    j[i][k] = (rp[i] - r[i]) / H;
                }
            }
            let Some(dx) = solve_n(j, [-r[0], -r[1], -r[2], -r[3], -r[4], -r[5]]) else {
                break;
            };
            let mut step = 1.0;
            let mut xnew = x;
            for _ in 0..24 {
                for k in 0..6 {
                    xnew[k] = x[k] + step * dx[k];
                }
                clamp_heli(&mut xnew);
                if nrm(self.trim_heli_residual(xnew, alt_ft, v_ned)) < rnorm {
                    break;
                }
                step *= 0.5;
            }
            x = xnew;
        }

        self.collective_cmd = x[0].clamp(0.0, 1.0);
        self.elevator_cmd = x[1];
        self.aileron_cmd = x[2];
        self.rudder_cmd = x[3];
        self.collective_trim_cmd = 0.0;
        self.pitch_trim_cmd = 0.0;
        self.roll_trim_cmd = 0.0;
        self.yaw_trim_cmd = 0.0;
        self.throttle_cmd = 1.0;
        self.flap_pos_deg = 0.0;
        self.ic_vc_kts = vc_kts;
        self.ic_alt_ft = alt_ft;
        self.state = State {
            pos: [px, py, -alt_ft],
            vel: [0.0; 3],
            quat: euler_to_quat(x[4], x[5], psi),
            omega: [0.0; 3],
        };
        self.state.vel = self.state.ned_to_body(v_ned);

        // Settle the rotor inflow at the solved point (a couple of passes), so
        // the first `step` starts from the converged inflow.
        self.trim_heli_residual(x, alt_ft, v_ned);
        self.trim_heli_residual(x, alt_ft, v_ned);
    }

    /// Seat a fly-by-wire aircraft on a wings-level steady trim at `vc_kts` and
    /// `alt_ft`. Because the loop commands *rates*, the equilibrium is an
    /// (alpha, elevator_rad, throttle) that nulls the longitudinal residual with
    /// the rate error zero; the solved elevator is pre-loaded into the pitch
    /// integrator (`elevator = -Ki·I` at zero rate error) so the loop reproduces
    /// it and the trimmed state is a true fixed point. Lateral stays neutral.
    fn trim_fbw(&mut self, vc_kts: f64, alt_ft: f64) {
        let spec = match self.ac.control {
            ControlSystem::Fbw(s) => s,
            ControlSystem::Direct(_) | ControlSystem::Heli(_) => {
                unreachable!("trim_fbw on a non-FBW aircraft")
            }
        };
        let vt = self.isa.cas_to_tas(vc_kts * KT_TO_FPS, alt_ft);
        const H: f64 = 1e-4;
        let n3 = |r: [f64; 3]| (r[0] * r[0] + r[1] * r[1] + r[2] * r[2]).sqrt();

        let mut xl = [0.05, 0.0, 0.5];
        for _ in 0..60 {
            let r = self.fbw_long_residual(xl, vt, alt_ft);
            let rnorm = n3(r);
            if rnorm < 1e-7 {
                break;
            }
            let mut j = [[0.0; 3]; 3];
            for k in 0..3 {
                let mut xp = xl;
                xp[k] += H;
                let rp = self.fbw_long_residual(xp, vt, alt_ft);
                for i in 0..3 {
                    j[i][k] = (rp[i] - r[i]) / H;
                }
            }
            let Some(dx) = solve_n(j, [-r[0], -r[1], -r[2]]) else {
                break;
            };
            let mut step = 1.0;
            let mut xnew = xl;
            for _ in 0..24 {
                for k in 0..3 {
                    xnew[k] = xl[k] + step * dx[k];
                }
                clamp_fbw_long(&mut xnew, spec.elevator_lim_rad);
                if n3(self.fbw_long_residual(xnew, vt, alt_ft)) < rnorm {
                    break;
                }
                step *= 0.5;
            }
            xl = xnew;
        }

        let (alpha, elevator_rad) = (xl[0], xl[1]);
        // Level-flight seat at the solved alpha; lateral neutral (symmetric
        // airframe, sideslip-free).
        let [px, py, _] = self.state.pos;
        self.state = State {
            pos: [px, py, -alt_ft],
            vel: [vt * alpha.cos(), 0.0, vt * alpha.sin()],
            quat: euler_to_quat(0.0, alpha, self.ic_psi_deg * D2R),
            omega: [0.0; 3],
        };
        self.seat_air_frame();
        self.elevator_cmd = 0.0;
        self.aileron_cmd = 0.0;
        self.rudder_cmd = 0.0;
        self.pitch_trim_cmd = 0.0;
        self.roll_trim_cmd = 0.0;
        self.yaw_trim_cmd = 0.0;
        self.throttle_cmd = xl[2].clamp(0.0, 1.0);
        self.flap_pos_deg = 0.0;
        self.ic_vc_kts = vc_kts;
        self.ic_alt_ft = alt_ft;

        // Pre-load the integrators so the loop's output equals the solved
        // elevator at zero rate error: elevator = -(Kp·0 + Ki·I) => I = -elev/Ki.
        self.fbw.pitch_int =
            (-elevator_rad / spec.pitch_ki).clamp(-spec.pitch_int_limit, spec.pitch_int_limit);
        self.fbw.roll_int = 0.0;
        self.fbw.yaw_int = 0.0;

        // Seed the induced-drag feedback so the first step's drag is consistent.
        let rho = self.isa.density(alt_ft);
        let pressure = self.isa.pressure(alt_ft);
        let temp = self.isa.temperature(alt_ft);
        let sound = self.isa.sound_speed(alt_ft);
        let mach = vt / sound.max(1.0);
        let u = vt * alpha.cos();
        let qbar = 0.5 * rho * vt * vt;
        let surf = Surfaces {
            elevator_rad,
            mag_elevator_rad: elevator_rad.abs(),
            lef_rad: spec.lef_rad(alpha, qbar, pressure, false, mach),
            ..Default::default()
        };
        let (aero, _, _) =
            self.trim_eval_surf(alpha, 0.0, &surf, self.throttle_cmd, vt, alt_ft, 0.0);
        self.cl_sq = aero.cl * aero.cl;

        // Seat the live powerplant on the trim point so the first step is
        // already consistent (otherwise it would start cold and re-spool).
        let eng = self
            .power
            .steady_at(self.throttle_cmd, rho, pressure, temp, u, sound, mach);
        self.engine_rpm = eng.rpm;
        self.engine_power_hp = eng.power_hp;
    }
}

impl FlightModel for Fdm {
    /// The F-22's FBW drives thrust vectoring; direct-FCS aircraft don't.
    fn has_tvc(&self) -> bool {
        matches!(self.ac.control, ControlSystem::Fbw(_))
    }

    /// Whether the gear retracts (F-22) or is fixed (c172p).
    fn has_retractable_gear(&self) -> bool {
        self.ac.gear_retractable
    }

    /// Rotorcraft (the AH-1S) carry rotors; fixed-wing aircraft don't.
    fn has_rotor(&self) -> bool {
        !self.rotors.is_empty()
    }

    /// Parked CG height above the runway. Each airframe carries its own value:
    /// the c172p/F-22 reset heights, a helicopter's ski-sole height, or the
    /// 737's `altitude 49 IN` reset. Seeding at the wrong height buries the
    /// gear and the ground springs launch the aircraft.
    fn rest_h_ft(&self) -> f64 {
        self.ac.rest_h_ft
    }

    /// Solve a wings-level, steady trim at `vc_kts` (calibrated) and `alt_ft`,
    /// then seat the model on it. Longitudinal unknowns — angle of attack,
    /// pitch-trim command, throttle — null the axial/vertical force and pitch
    /// moment; lateral unknowns — aileron and rudder — go to the roll/yaw trim
    /// tabs, nulling the roll and yaw moment (the last absorbing the engine
    /// reaction torque). Each is a Newton iteration with a finite-difference
    /// Jacobian. The trimmed state is then a fixed point, so the aircraft holds
    /// the point open-loop.
    ///
    /// A fly-by-wire aircraft (F-22) has no trim-command channel to solve, so it
    /// takes the [`Fdm::trim_fbw`] path instead: solve the equilibrium elevator
    /// angle and pre-load it into the rate loop's integrator.
    fn trim(&mut self, vc_kts: f64, alt_ft: f64) {
        // Arcade mode has no aero to trim against: the FBW solve would seat an
        // elevator the arcade rules ignore. `apply_ic` already leaves a level,
        // unaccelerated state at the target speed, which is enough.
        if self.arcade {
            return;
        }
        if let ControlSystem::Fbw(_) = self.ac.control {
            self.trim_fbw(vc_kts, alt_ft);
            return;
        }
        if matches!(self.ac.control, ControlSystem::Heli(_)) {
            self.trim_heli(vc_kts, alt_ft);
            return;
        }
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
        self.seat_air_frame();
        self.ic_vc_kts = vc_kts;
        self.ic_alt_ft = alt_ft;

        // Seat the live powerplant on the trim point so the first step is already
        // consistent (otherwise it would start cold and re-spool).
        let rho = self.isa.density(alt_ft);
        let pressure = self.isa.pressure(alt_ft);
        let temp = self.isa.temperature(alt_ft);
        let sound = self.isa.sound_speed(alt_ft);
        let mach = vt / sound.max(1.0);
        let (throttle, u) = (self.throttle_cmd, vt * alpha.cos());
        let eng = self
            .power
            .steady_at(throttle, rho, pressure, temp, u, sound, mach);
        self.engine_rpm = eng.rpm;
        self.engine_power_hp = eng.power_hp;
    }

    fn step(&mut self) {
        self.advance(self.dt);
    }

    fn apply_ic(&mut self) {
        // Airborne ICs: attitude (`ic/{phi,theta,psi}-deg`), body rates
        // (`ic/{p,q,r}-rad_sec`) and sideslip (`ic/beta-deg`), all optional and
        // defaulting to a level, unaccelerated, slip-free trim. `ic/vc-kts` is a
        // *calibrated* airspeed, so convert it to the true airspeed the model
        // integrates (JSBSim's `SetVcalibratedKtsIC`). Sideslip carries the
        // velocity off the body x-axis by `beta` (v = V·sinβ), so the aero model
        // sees the programmed β.
        let tas = self
            .isa
            .cas_to_tas(self.ic_vc_kts * KT_TO_FPS, self.ic_alt_ft);
        let (sb, cb) = (self.ic_beta_deg * D2R).sin_cos();
        self.state = State {
            pos: [self.ic_north_ft, self.ic_east_ft, -self.ic_alt_ft],
            vel: [tas * cb, tas * sb, 0.0],
            quat: euler_to_quat(
                self.ic_phi_deg * D2R,
                self.ic_theta_deg * D2R,
                self.ic_psi_deg * D2R,
            ),
            omega: [self.ic_p_rps, self.ic_q_rps, self.ic_r_rps],
        };
        // Parked resets (vc-kts 0) must not drift with the wind: the wheels
        // hold the vehicle, so a zero airspeed stays a zero ground speed.
        // Airborne ICs carry the air -> ground conversion.
        if self.ic_vc_kts > 0.0 {
            self.seat_air_frame();
        }
        self.flap_pos_deg = self.flap_cmd * self.ac.control.flap_max_deg();
        // Every reset starts parked or trimmed, never mid-auto-hover.
        self.heli_hover_cmd = false;
        self.heli_hover_prev = false;
        self.heli_hover_state = HoverState::Off;
        self.heli_hover_target_ft = 0.0;
        self.heli_balance = self.arcade && !self.rotors.is_empty();
    }

    fn get(&self, name: &str) -> f64 {
        let (phi, theta, psi) = self.state.euler();
        match name {
            "position/h-sl-ft" => self.alt_ft(),
            "position/lat-gc-deg" => self.origin_lat + self.state.pos[0] / FT_PER_DEG_LAT,
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
            // Airspeeds are air-relative: the ambient wind never inflates a
            // pitot reading even though it moves the vehicle over the ground.
            "velocities/vc-kts" => {
                self.isa.tas_to_cas(self.air_airspeed_fps(), self.alt_ft()) * FPS_TO_KT
            }
            "velocities/vtrue-kts" => self.air_airspeed_fps() * FPS_TO_KT,
            // NED ground velocity components (the wind is *not* subtracted
            // here; it moves the vehicle and carries the HUD's FPM marker).
            "velocities/v-north-fps" => self.state.body_to_ned(self.state.vel)[0],
            "velocities/v-east-fps" => self.state.body_to_ned(self.state.vel)[1],
            "velocities/vg-fps" => {
                let v = self.state.body_to_ned(self.state.vel);
                (v[0] * v[0] + v[1] * v[1]).sqrt()
            }
            "velocities/h-dot-fps" => self.h_dot_fps(),
            // Flat-earth ground plane at sea level.
            "position/h-agl-ft" => self.alt_ft(),
            // Normal load factor: the aero force along the body z axis over
            // weight, so 1.0 in level flight (JSBSim's `forces/load-factor`).
            "forces/load-factor" => -self.diag.force_aero[2] / (self.mass * G0),
            // Sideslip: body lateral over forward velocity, positive when the
            // relative wind comes from the right.
            "aero/beta-deg" => self.air_beta_rad() * R2D,
            "velocities/mach" => {
                self.air_airspeed_fps() / self.isa.sound_speed(self.alt_ft()).max(1.0)
            }
            // Ambient wind readback (NED, fps), same scales as JSBSim's bus.
            "atmosphere/wind-north-fps" => self.wind_ned_fps[0],
            "atmosphere/wind-east-fps" => self.wind_ned_fps[1],
            "atmosphere/wind-down-fps" => self.wind_ned_fps[2],
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
            // Achieved gear position (1 = down, 0 = up) and the selector.
            "gear/gear-pos-norm" => self.gear_pos,
            "gear/gear-cmd-norm" => self.gear_cmd as i32 as f64,
            "fcs/flap-pos-deg" => self.flap_pos_deg,
            // SpoILER/reverse readback (positions, not commands).
            "fcs/speedbrake-pos-norm" => self.speedbrake_pos,
            "fcs/speedbrake-cmd-norm" => self.speedbrake_cmd,
            "fcs/reverse-pos-norm" => self.reverse_pos,
            "fcs/reverse-cmd-norm" => self.reverse_cmd,
            // Last-step resolved surface positions (radians) and the normalized
            // thrust the aero model saw.
            "fcs/elevator-pos-rad" => self.surf.elevator_rad,
            "fcs/left-aileron-pos-rad" => self.surf.aileron_rad,
            "fcs/right-aileron-pos-rad" => self.surf.right_aileron_rad,
            "fcs/rudder-pos-rad" => self.surf.rudder_rad,
            "fcs/lef-pos-rad" => self.surf.lef_rad,
            "fcs/tvc-pos-rad" => self.surf.tvc_rad,
            "fcs/tvc-enable" => self.tvc_enable as i32 as f64,
            "fcs/arcade-enable" => self.arcade as i32 as f64,
            // Arcade-helicopter switches and state (no-op for other aircraft).
            "fcs/heli-hover-enable" => self.heli_hover_cmd as i32 as f64,
            "fcs/heli-balance-enable" => self.heli_balance as i32 as f64,
            "fcs/heli-hover-state" => self.heli_hover_state as i32 as f64,
            "fcs/thrust-norm" => self.surf.thrust_norm,
            "fcs/elevator-cmd-norm" => self.elevator_cmd,
            "fcs/aileron-cmd-norm" => self.aileron_cmd,
            "fcs/rudder-cmd-norm" => self.rudder_cmd,
            "fcs/throttle-cmd-norm" => self.throttle_cmd,
            "fcs/left-brake-cmd-norm" => self.left_brake_cmd,
            "fcs/right-brake-cmd-norm" => self.right_brake_cmd,
            "fcs/collective-cmd-norm" => self.collective_cmd,
            "fcs/pitch-trim-cmd-norm" => self.pitch_trim_cmd,
            "fcs/roll-trim-cmd-norm" => self.roll_trim_cmd,
            "fcs/yaw-trim-cmd-norm" => self.yaw_trim_cmd,
            // Rotor state, main rotor at index 0 and tail at 1 (JSBSim's
            // `propulsion/engine[N]/...` rotor properties). Zero for fixed-wing.
            "propulsion/engine[0]/rotor-rpm" => self.rotor_get(0, |r| r.rpm()),
            "propulsion/engine[1]/rotor-rpm" => self.rotor_get(1, |r| r.rpm()),
            "propulsion/engine[0]/a0-rad" => self.rotor_get(0, |r| r.coning_angle()),
            "propulsion/engine[0]/a1-rad" => self.rotor_get(0, |r| r.longitudinal_flap()),
            "propulsion/engine[0]/b1-rad" => self.rotor_get(0, |r| r.lateral_flap()),
            "propulsion/engine[0]/thrust-coefficient" => {
                self.rotor_get(0, |r| r.thrust_coefficient())
            }
            "propulsion/engine[0]/torque-lbsft" => self.rotor_get(0, |r| r.torque()),
            "propulsion/engine[0]/vi-fps" => self.rotor_get(0, |r| r.induced_velocity()),
            "propulsion/engine[0]/induced-inflow-ratio" => self.rotor_get(0, |r| r.inflow_ratio()),
            "propulsion/engine[0]/engine-rpm" => self.engine_rpm,
            "propulsion/engine[0]/power-hp" => self.engine_power_hp,
            // Fuel readback (c172p only: the model declares a FuelSpec).
            "propulsion/engine[0]/fuel-flow-pph" => self.power.fuel_flow_pph(),
            "propulsion/engine[0]/has-fuel" => if self.ac.fuel.is_some() { 1.0 } else { 0.0 },
            "propulsion/engine[0]/running" => if self.engine_rpm > 0.0 && self.engine_power_hp > 0.0
            {
                1.0
            } else {
                0.0
            },
            // Cold-start state readback (the status line's MASTER field).
            "propulsion/ignition-enable" => if self.ignition { 1.0 } else { 0.0 },
            "propulsion/tank-0-contents-lbs" => self.fuel[0],
            "propulsion/tank-1-contents-lbs" => self.fuel[1],
            // Total usable capacity, for gauge scaling.
            "propulsion/tank-capacity-lbs" => {
                self.ac.fuel.as_ref().map(|f| 2.0 * f.capacity_lb).unwrap_or(0.0)
            }
            // Propeller/fan thrust for fixed-wing aircraft; the main rotor's
            // thrust for a helicopter (JSBSim ties this to the thruster's
            // `GetThrust`, which for an `FGRotor` is the rotor thrust).
            "propulsion/engine[0]/thrust-lbs" => {
                self.diag.thrust + self.rotor_get(0, |r| r.thrust())
            }
            "propulsion/engine[1]/thrust-lbs" => self.rotor_get(1, |r| r.thrust()),
            // Last-step aero/propulsion split (same spelling JSBSim uses), so
            // the validation harness can diff forces and moments frame by frame.
            "aero/alpha-rad" => self.diag.alpha,
            "aero/alpha-deg" => self.diag.alpha * R2D,
            "aero/qbar-psf" => self.diag.qbar,
            "aero/cl-squared" => self.cl_sq,
            "forces/fbx-aero-lbs" => self.diag.force_aero[0],
            "forces/fby-aero-lbs" => self.diag.force_aero[1],
            "forces/fbz-aero-lbs" => self.diag.force_aero[2],
            // Lateral load factor at the pilot, weight-normalised (0 in
            // coordinated flight). Drives the turn coordinator's slip ball;
            // the prop's side force is negligible for these aircraft.
            "accelerations/n-pilot-y-norm" => self.diag.force_aero[1] / (self.mass * G0),
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
            "ic/phi-deg" => self.ic_phi_deg = value,
            "ic/theta-deg" => self.ic_theta_deg = value,
            "ic/psi-true-deg" => self.ic_psi_deg = value,
            "ic/beta-deg" => self.ic_beta_deg = value,
            "ic/p-rad_sec" => self.ic_p_rps = value,
            "ic/q-rad_sec" => self.ic_q_rps = value,
            "ic/r-rad_sec" => self.ic_r_rps = value,
            // Flat-earth position over the origin: degrees -> feet offsets so
            // apply_ic can seat the IC away from the threshold (the approach
            // gate).
            "ic/lat-gc-deg" => self.ic_north_ft = (value - self.origin_lat) * FT_PER_DEG_LAT,
            "ic/long-gc-deg" => {
                let cos_lat = (self.origin_lat * D2R).cos().max(1e-6);
                self.ic_east_ft = (value - self.origin_lon) * FT_PER_DEG_LAT * cos_lat;
            }
            // Any non-zero `set-running` lights the engine and spools it to a
            // steady state at the current flight condition (JSBSim's convention).
            // A dry tank refuses the light: there is nothing to burn.
            "propulsion/set-running" => {
                let dry = self.ac.fuel.is_some() && self.fuel[0] + self.fuel[1] <= 0.0;
                if value != 0.0 && !dry {
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
            "fcs/collective-cmd-norm" => self.collective_cmd = value,
            // Ambient wind (NED components, ft/s). Zero everywhere by default,
            // so models without a wind stage fly exactly as they did before.
            "atmosphere/wind-north-fps" => self.wind_ned_fps[0] = value,
            "atmosphere/wind-east-fps" => self.wind_ned_fps[1] = value,
            "atmosphere/wind-down-fps" => self.wind_ned_fps[2] = value,
            // Cold start: spark available (master + magnetos folded together by
            // the app) and the held starter crank. Set-running is the shortcut
            // the spawner uses; these are the pilot's path.
            "propulsion/ignition-enable" => self.ignition = value != 0.0,
            "propulsion/engine[0]/starter-cmd-norm" => self.starter_cmd = value.clamp(0.0, 1.0),
            // Shutdown (`ENGINE = COLD` selector): kills the running engine —
            // the mirror of `set-running` for the settings toggle.
            "propulsion/engine[0]/set-stopped" => {
                if value != 0.0 {
                    self.power.stop();
                }
            }
            // The apron pump filling the wing tanks to capacity.
            "propulsion/refuel-cmd" => {
                if value != 0.0 {
                    self.refuel();
                }
            }
            "fcs/pitch-trim-cmd-norm" => self.pitch_trim_cmd = value,
            "fcs/roll-trim-cmd-norm" => self.roll_trim_cmd = value,
            "fcs/yaw-trim-cmd-norm" => self.yaw_trim_cmd = value,
            "fcs/flap-cmd-norm" => self.flap_cmd = value,
            // Seat the achieved flap position, the reset twin of the gear
            // seat below: teleporting into an approach with the flaps still
            // travelling would leave the wing clean for seconds of approach
            // while the actuator catches up. Not part of JSBSim's writable
            // set; on that bench the native FCS input takes over.
            "fcs/flap-pos-norm" => {
                self.flap_pos_deg =
                    value.clamp(0.0, 1.0) * self.ac.control.flap_max_deg();
            }
            // Flight spoilers / thrust reverse. Both are 0..1 commands; the
            // positions roll toward them in `step_spoilers`. Airframes that
            // lack the channels (gear-fixed singles, the F-22) store the
            // command but its position never deploys and the thrust term is
            // zeroed by `max_reverse_lbs`.
            "fcs/speedbrake-cmd-norm" => self.speedbrake_cmd = value,
            "fcs/reverse-cmd-norm" => self.reverse_cmd = value,
            "fcs/tvc-enable" => self.tvc_enable = value != 0.0,
            // Arcade ("game") mode: swaps the aero/FBW force resolution for the
            // simplified rules in `arcade`. Off by default. On a rotorcraft it
            // selects the helicopter variant and arms auto-balance by default.
            "fcs/arcade-enable" => {
                let on = value != 0.0;
                if on && !self.arcade && !self.rotors.is_empty() {
                    self.heli_balance = true;
                }
                if !on {
                    self.heli_hover_cmd = false;
                    self.heli_hover_prev = false;
                    self.heli_hover_state = HoverState::Off;
                    self.heli_hover_target_ft = 0.0;
                    self.heli_balance = false;
                }
                self.arcade = on;
            }
            "fcs/heli-hover-enable" => self.heli_hover_cmd = value != 0.0,
            "fcs/heli-balance-enable" => self.heli_balance = value != 0.0,
            // Gear selector: 1 = down, 0 = up (JSBSim's convention). The
            // achieved position rolls toward it; fixed-gear aircraft ignore it.
            "gear/gear-cmd-norm" => self.gear_cmd = value > 0.5,
            // Seat the achieved position directly (skipping the gear travel).
            // Used by the app's resets: teleporting to the runway with the gear
            // still retracting would leave no wheels to touch down on. Not part
            // of JSBSim's writable set, but harmless on the fixed-gear paths
            // (`step_gear` overwrites it next step).
            "gear/gear-pos-norm" => self.gear_pos = value.clamp(0.0, 1.0),
            // Sea-surface stop (ft above sea level); zero or negative disables.
            "environment/floor-ft" => {
                self.floor_ft = if value.is_finite() && value > 0.0 {
                    Some(value)
                } else {
                    None
                };
            }
            "fcs/left-brake-cmd-norm" => self.left_brake_cmd = value,
            "fcs/right-brake-cmd-norm" => self.right_brake_cmd = value,
            _ => {}
        }
    }
}

/// Fill the named-property scratch map the aero evaluator reads. The gear
/// position (`gear/gear-pos-norm`, driving the CDgear drag term) is set by the
/// caller, since it's a whole-aircraft state rather than a resolved surface.
const AIR_ANGLES_MIN_FPS: f64 = 2.0;

/// Angle-of-attack and sideslip from an air-relative velocity, held at an
/// honest zero below a walking pace (`AIR_ANGLES_MIN_FPS`). Near a standstill
/// that velocity is settling and creep noise — a few millimetres per second
/// whose direction sweeps as the plane creeps — so the `atan2`s land anywhere:
/// park noise reads ±90°/±180° and drifts through the stall band, and every
/// consumer of the reading (the HUD's data stack and FPM AoA chevron, the
/// slip ball's beta, the camera's speed-feel, the FCS schedules) breathes with
/// it. The gate cannot hurt physics: below it the dynamic pressure — and so
/// every force carrying alpha — is all but zero.
fn air_angles(u: f64, v: f64, w: f64) -> (f64, f64) {
    let air = (u * u + v * v + w * w).sqrt();
    if air < AIR_ANGLES_MIN_FPS {
        (0.0, 0.0)
    } else {
        (w.atan2(u), v.atan2((u * u + w * w).sqrt()))
    }
}

/// Fill the named-property scratch map the aero evaluator reads. The gear
/// position (`gear/gear-pos-norm`, driving the CDgear drag term) is set by the
/// caller, since it's a whole-aircraft state rather than a resolved surface.
#[allow(clippy::too_many_arguments)] // the full per-airframe aero inputs, each used independently
fn fill_props(
    props: &mut PropMap,
    ac: &Aircraft,
    s: &State,
    surf: &Surfaces,
    induced_fps: f64,
    rho: f64,
    sound_speed: f64,
    wind_body: Vec3,
) {
    // Aerodynamic inputs are *air-relative*: the state velocity is ground
    // velocity in the presence of wind, so the ambient wind comes off before
    // any alpha/beta/qbar is derived.
    let (u, v, w) = (
        s.vel[0] - wind_body[0],
        s.vel[1] - wind_body[1],
        s.vel[2] - wind_body[2],
    );
    let vt = (u * u + v * v + w * w).sqrt().max(1e-6);
    let (alpha, beta) = air_angles(u, v, w);
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
    props.set("aero/betadot-rad_sec", 0.0);
    props.set("aero/stall-hyst-norm", 0.0);
    props.set("velocities/mach", vt / sound_speed.max(1.0));
    props.set("velocities/p-aero-rad_sec", s.omega[0]);
    props.set("velocities/q-aero-rad_sec", s.omega[1]);
    props.set("velocities/r-aero-rad_sec", s.omega[2]);
    props.set("velocities/u-aero-fps", u);
    props.set("velocities/v-aero-fps", v);
    props.set("velocities/w-aero-fps", w);
    // Trig atoms the AH-1S wing/fuselage build-up multiplies (the coefficient
    // framework has no sin/cos of its own — see `aero::Factor`).
    let (sa2, ca2) = (alpha.sin(), alpha.cos());
    let (sb2, cb2) = (beta.sin(), beta.cos());
    props.set("aero/sin-alpha", sa2);
    props.set("aero/cos-alpha", ca2);
    props.set("aero/sin2-alpha", sa2 * sa2);
    props.set("aero/cos2-alpha", ca2 * ca2);
    props.set("aero/abs-sin-alpha", sa2.abs());
    props.set("aero/sin-beta", sb2);
    props.set("aero/cos-beta", cb2);
    props.set("aero/sin2-beta", sb2 * sb2);
    // The main-rotor longitudinal cyclic, the AH-1S stabilator (horizontal-tail
    // incidence) schedule's independent variable.
    props.set(
        "propulsion/engine/longitudinal-ctrl-rad",
        surf.longitudinal_cyc_rad,
    );
    props.set("atmosphere/rho-slugs_ft3", rho);
    props.set("propulsion/engine/prop-induced-velocity_fps", induced_fps);
    props.set("fcs/elevator-pos-rad", surf.elevator_rad);
    props.set("fcs/mag-elevator-pos-rad", surf.mag_elevator_rad);
    props.set("fcs/left-aileron-pos-rad", surf.aileron_rad);
    props.set("fcs/right-aileron-pos-rad", surf.right_aileron_rad);
    props.set("fcs/rudder-pos-rad", surf.rudder_rad);
    props.set("fcs/flap-pos-deg", surf.flap_deg);
    props.set("fcs/flap-pos-norm", surf.flap_norm);
    props.set("fcs/lef-pos-rad", surf.lef_rad);
    props.set("fcs/speedbrake-pos-norm", surf.speedbrake_norm);
    props.set("fcs/tvc-pos-rad", surf.tvc_rad);
    props.set("fcs/thrust-norm", surf.thrust_norm);
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

/// Keep the longitudinal trim unknowns `[alpha_rad, pitch_trim_cmd, throttle]`
/// in a smooth, physical range: angle of attack within +/-20 deg, the trim
/// command within its normalized authority, and throttle clear of the engine's
/// hard clamps (where its derivative vanishes and the Jacobian goes singular).
fn clamp_long(x: &mut [f64; 3]) {
    x[0] = x[0].clamp(-0.35, 0.35);
    x[1] = x[1].clamp(-1.0, 1.0);
    x[2] = x[2].clamp(0.02, 0.995);
}

/// Keep the FBW trim unknowns `[alpha_rad, elevator_rad, throttle]` physical:
/// alpha within +/-20 deg, the elevator within its surface limit, and throttle
/// clear of the engine's clamps (same reason as `clamp_long`).
fn clamp_fbw_long(x: &mut [f64; 3], elevator_lim_rad: f64) {
    x[0] = x[0].clamp(-0.35, 0.35);
    x[1] = x[1].clamp(-elevator_lim_rad, elevator_lim_rad);
    x[2] = x[2].clamp(0.02, 0.995);
}

/// Keep the helicopter hover-trim unknowns `[collective, longitudinal, lateral,
/// pedal, phi, theta]` physical: the commands within their normalized ranges
/// and the hover attitude within a few degrees of level.
fn clamp_heli(x: &mut [f64; 6]) {
    x[0] = x[0].clamp(0.0, 1.0);
    x[1] = x[1].clamp(-1.0, 1.0);
    x[2] = x[2].clamp(-1.0, 1.0);
    x[3] = x[3].clamp(-1.0, 1.0);
    x[4] = x[4].clamp(-0.35, 0.35);
    x[5] = x[5].clamp(-0.35, 0.35);
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
pub(crate) fn euler_to_quat(phi: f64, theta: f64, psi: f64) -> [f64; 4] {
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

    #[test]
    fn wind_round_trips_through_the_property_bus() {
        let mut m = Fdm::new("c172p");
        m.set("atmosphere/wind-north-fps", 10.0);
        m.set("atmosphere/wind-east-fps", 20.0);
        m.set("atmosphere/wind-down-fps", 1.0);
        assert_eq!(m.get("atmosphere/wind-north-fps"), 10.0);
        assert_eq!(m.get("atmosphere/wind-east-fps"), 20.0);
        assert_eq!(m.get("atmosphere/wind-down-fps"), 1.0);
    }

    /// Airspeeds are air-relative: in a 10 kt tailwind the pitot still reads the
    /// programmed 30 kt, while the vehicle sits 10 kt faster over the ground.
    #[test]
    fn pitot_readings_ignore_the_wind() {
        let mut m = Fdm::new("c172p");
        m.set("ic/vc-kts", 30.0);
        m.set("ic/h-sl-ft", 1000.0);
        m.set("ic/psi-true-deg", 0.0);
        // 10 kt of wind blowing *toward* the north (a tailwind for a heading-0
        // flight), so the vehicle has to cover the same air distance at the
        // same airspeed and drift downwind over the ground.
        m.set("atmosphere/wind-north-fps", 10.0 * KT_TO_FPS);
        m.apply_ic();
        // vc-kts is calibrated; at sea level TAS == CAS within roundoff.
        let vc = m.get("velocities/vc-kts");
        assert!((vc - 30.0).abs() < 0.5, "vc = {vc}");
        // Ground velocity = air velocity + wind. `ic/vc-kts` is calibrated, and
        // at 1000 ft the converted TAS is a little over 30 kt, so the vehicle
        // sits between 40 and 41 kt over the ground.
        let north = m.get("velocities/v-north-fps");
        let min = 40.0 * KT_TO_FPS;
        let max = 41.0 * KT_TO_FPS;
        assert!(
            north > min && north < max,
            "v-north = {north}, expected within [{} .. {}]",
            min,
            max
        );
        // ...and the reported Mach is sane while the wind echoes back.
        let mach = m.get("velocities/mach");
        assert_eq!(m.get("atmosphere/wind-north-fps"), 10.0 * KT_TO_FPS);
        assert!(mach > 0.0, "mach = {mach}");
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
        assert!(
            m.get("velocities/h-dot-fps").abs() < 1.0,
            "should be at rest"
        );
    }

    #[test]
    fn parked_angles_settle_to_zero() {
        // Near a standstill alpha/beta are `atan2` of drift noise: the settle
        // and the centimetres-a-second creep sweep the readings around
        // ±90°/±180° and through the stall band, and the HUD, the slip ball
        // and the chase camera all breathe with it. The bus must hold them at
        // an honest zero until there is real airflow. Brakes on, exactly as
        // the app's parking-brake hook programs a cold spawn.
        let mut m = parked();
        m.set("fcs/left-brake-cmd-norm", 1.0);
        m.set("fcs/right-brake-cmd-norm", 1.0);
        for _ in 0..120 * 5 {
            m.step();
        }
        assert_eq!(m.get("aero/alpha-deg"), 0.0);
        assert_eq!(m.get("aero/beta-deg"), 0.0);
    }

    #[test]
    fn cruise_alpha_never_reads_as_the_rest_noise() {
        // The gate covers the standstill only. A trimmed flight stalls in
        // alpha ~4-6 deg — never the ±90 garbage the old raw reading showed.
        let mut m = Fdm::new("c172p");
        m.trim(120.0, 3000.0);
        for _ in 0..120 * 5 {
            m.step();
        }
        let alpha = m.get("aero/alpha-deg");
        assert!(
            alpha.abs() < 15.0,
            "cruise alpha must be a real wing reading: {alpha}"
        );
    }

    /// Full brakes hold a parked aircraft against idle thrust plus a 15 kt
    /// quartering headwind, so a windy spawn does not creep down the runway
    /// (the app's parking-brake hook relies on this).
    #[test]
    fn full_brakes_hold_against_wind_and_idle_thrust() {
        let mut m = parked();
        // 15 kt from 300 deg: NED roughly (-0.5, +0.87) * 15 kt in fps.
        let kts = 15.0 * KT_TO_FPS;
        m.set("atmosphere/wind-north-fps", -0.5 * kts);
        m.set("atmosphere/wind-east-fps", 0.866 * kts);
        m.set("fcs/left-brake-cmd-norm", 1.0);
        m.set("fcs/right-brake-cmd-norm", 1.0);
        for _ in 0..120 * 10 {
            m.step();
        }
        let drift = (m.get("velocities/v-north-fps").powi(2)
            + m.get("velocities/v-east-fps").powi(2))
        .sqrt();
        assert!(drift < 0.5, "the parked aircraft crept: {drift} fps");
        assert!(m.get("gear/wow") > 0.5);
    }

    #[test]
    fn hard_floor_stops_a_gear_up_dive() {
        let mut m = Fdm::new("f22");
        m.set("gear/gear-cmd-norm", 0.0);
        m.set("gear/gear-pos-norm", 0.0);
        m.set("environment/floor-ft", 40.0);
        // 5 ft above the sea, diving at 60 ft/s: one step would put it in the
        // water, so the floor must catch it.
        m.state.pos = [0.0, 0.0, -5.0];
        m.state.vel = [400.0, 0.0, 60.0];
        m.step();
        assert_eq!(m.get("position/h-sl-ft"), 40.0);
        assert!(
            m.get("velocities/h-dot-fps").abs() < 1e-6,
            "the stop must kill the descent: {}",
            m.get("velocities/h-dot-fps")
        );
    }

    #[test]
    fn wow_true_on_ground_false_airborne() {
        let mut m = parked();
        for _ in 0..120 {
            m.step();
        }
        assert!(
            m.get("gear/wow") > 0.5,
            "a parked aircraft is on the wheels"
        );
        m.set("ic/vc-kts", 120.0);
        m.set("ic/h-sl-ft", 3000.0);
        m.apply_ic();
        m.set("propulsion/set-running", -1.0);
        m.step();
        m.step();
        assert!(
            m.get("gear/wow") < 0.5,
            "an airborne aircraft is off the wheels"
        );
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
        assert!(
            min_h > 0.0,
            "sank through the ground during the roll: {min_h}"
        );
        // Heading 0 = north, so lat-gc rises with the northward displacement.
        assert!(
            m.get("position/lat-gc-deg") > 0.0,
            "should have rolled forward"
        );
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
        assert!(
            vc_peak > 5.0,
            "should build speed before braking, got {vc_peak}"
        );
        m.set("fcs/throttle-cmd-norm", 0.0);
        m.set("fcs/left-brake-cmd-norm", 1.0);
        m.set("fcs/right-brake-cmd-norm", 1.0);
        for _ in 0..120 * 4 {
            m.step();
        }
        let vc = m.get("velocities/vc-kts");
        assert!(
            vc < 0.5 * vc_peak,
            "brakes should slow the aircraft: {vc_peak} -> {vc}"
        );
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
        assert!(
            dpsi < -5.0,
            "left rudder should turn the nose left, dpsi = {dpsi}"
        );
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
    fn apply_ic_honours_attitude_rate_and_sideslip() {
        let mut m = Fdm::new("c172p");
        m.set("ic/vc-kts", 100.0);
        m.set("ic/h-sl-ft", 3000.0);
        m.set("ic/psi-true-deg", 60.0);
        m.set("ic/phi-deg", 25.0);
        m.set("ic/theta-deg", 8.0);
        m.set("ic/beta-deg", 8.0);
        m.set("ic/r-rad_sec", 3.0_f64.to_radians());
        m.apply_ic();
        assert!((m.get("attitude/phi-deg") - 25.0).abs() < 1e-9);
        assert!((m.get("attitude/theta-deg") - 8.0).abs() < 1e-9);
        assert!((m.get("attitude/psi-deg") - 60.0).abs() < 1e-9);
        assert!((m.get("velocities/r-rad_sec") - 3.0_f64.to_radians()).abs() < 1e-12);
        // Sideslip rides in the body velocity, so the aero model sees β: at 100 kt
        // with β = 8° the lateral component is roughly 24 fps.
        let w = m.state.vel[1];
        assert!(w > 15.0, "expected lateral velocity from beta, got {w}");
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
        assert!(
            (h - h0).abs() < 15.0,
            "trim drifted in altitude: {h0} -> {h}"
        );
        assert!(
            (vc - vc0).abs() < 1.0,
            "trim drifted in airspeed: {vc0} -> {vc}"
        );
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
            assert!(
                dh.abs() < 25.0,
                "trim at {vc} kt drifted {dh:.1} ft over 20 s"
            );
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
        assert!(
            m.get("fcs/throttle-cmd-norm") < 1.0,
            "cruise trim should be part-throttle"
        );
        assert!(
            m.get("velocities/q-rad_sec").abs() < 1e-6,
            "trim should have no pitch rate"
        );
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

    // ---------------------------- F-22 Raptor ------------------------------

    #[test]
    fn f22_constructs_and_is_named() {
        let m = Fdm::new("f22");
        assert_eq!(m.aircraft(), "f22");
    }

    #[test]
    fn f22_full_throttle_accelerates() {
        // Two F119s at full afterburner should produce far more thrust than the
        // c172's single piston engine, and the trim solver should seat the jet in
        // the air with a sensible (sub-unity) cruise throttle.
        let mut m = Fdm::new("f22");
        m.trim(350.0, 15000.0);
        println!(
            "trim: h={} vc={} theta={} throttle={} alpha={} cl2={}",
            m.get("position/h-sl-ft"),
            m.get("velocities/vc-kts"),
            m.get("attitude/theta-deg"),
            m.get("fcs/throttle-cmd-norm"),
            m.get("aero/alpha-deg"),
            m.get("aero/cl-squared"),
        );
        assert!(
            m.get("fcs/throttle-cmd-norm") < 1.0,
            "cruise should not need full throttle: {}",
            m.get("fcs/throttle-cmd-norm")
        );
    }

    #[test]
    fn f22_fbw_holds_level() {
        // The rate-command FBW makes the statically-unstable airframe hold a
        // steady trim: flown open-loop for 30 s, altitude and airspeed barely
        // move and the attitude stays bounded.
        let mut m = Fdm::new("f22");
        m.trim(350.0, 15000.0);
        let (h0, vc0) = (m.get("position/h-sl-ft"), m.get("velocities/vc-kts"));
        let mut max_q = 0.0_f64;
        for _ in 0..120 * 30 {
            m.step();
            max_q = max_q.max(m.get("velocities/q-rad_sec").abs());
        }
        let (h, vc) = (m.get("position/h-sl-ft"), m.get("velocities/vc-kts"));
        println!(
            "30s: h {h0:.0} -> {h:.0}, vc {vc0:.1} -> {vc:.1}, theta {} phi {} max|q| {max_q:.4}",
            m.get("attitude/theta-deg"),
            m.get("attitude/phi-deg"),
        );
        assert!((h - h0).abs() < 100.0, "altitude drifted {h0} -> {h}");
        assert!((vc - vc0).abs() < 10.0, "airspeed drifted {vc0} -> {vc}");
        assert!(max_q < 1.0, "pitch rate diverged: max|q| = {max_q}");
    }

    #[test]
    fn f22_fbw_tracks_pitch_rate_command_and_damps() {
        // The statically-unstable airframe only flies because the rate loop
        // closes it. A held nose-up command should drive a roughly-constant
        // pitch rate; releasing it should let the rate fall back to ~0 without
        // diverging.
        let mut m = Fdm::new("f22");
        m.trim(350.0, 15000.0);
        // +elevator-cmd is nose-down, so -0.3 commands nose-up.
        m.set("fcs/elevator-cmd-norm", -0.3);
        for _ in 0..120 {
            m.step();
        }
        let q_held = m.get("velocities/q-rad_sec");
        m.set("fcs/elevator-cmd-norm", 0.0);
        for _ in 0..120 * 5 {
            m.step();
        }
        let q_after = m.get("velocities/q-rad_sec");
        let theta = m.get("attitude/theta-deg");
        println!("q held {q_held:.4}, q after release {q_after:.4}, theta {theta:.1}");
        assert!(
            q_held > 0.1,
            "should pitch nose-up under the command: q = {q_held}"
        );
        assert!(
            q_after.abs() < 0.05,
            "pitch rate should damp back to ~0: {q_after}"
        );
        assert!(theta.abs() < 45.0, "attitude should stay bounded: {theta}");
    }

    #[test]
    fn f22_gear_carries_the_weight() {
        // Parked on the runway with the engine running, the F-22 should settle
        // onto its three gear struts rather than sinking through the ground.
        let mut m = Fdm::new("f22");
        m.set("ic/vc-kts", 0.0);
        m.set("ic/h-sl-ft", 8.0);
        m.set("ic/psi-true-deg", 0.0);
        m.apply_ic();
        m.set("propulsion/set-running", -1.0);
        for _ in 0..120 * 5 {
            m.step();
        }
        let h = m.get("position/h-sl-ft");
        println!("f22 parked: h={h} wow={}", m.get("gear/wow"));
        assert!(h > 0.0, "the aircraft sank through the ground: h = {h}");
        assert!(m.get("gear/wow") > 0.5, "should report weight-on-wheels");
    }

    #[test]
    fn f22_gear_cmd_roundtrips() {
        let mut m = Fdm::new("f22");
        assert!(m.get("gear/gear-cmd-norm") > 0.5, "selector defaults down");
        m.set("gear/gear-cmd-norm", 0.0);
        assert!(m.get("gear/gear-cmd-norm") < 0.5);
    }

    #[test]
    fn f22_gear_pos_seats_directly_for_resets() {
        // The app's resets seat the achieved position (skipping the gear
        // travel), so a teleport to the runway lands on deployed gear.
        let mut m = f22_cruise(false);
        m.set("gear/gear-cmd-norm", 0.0);
        for _ in 0..120 * 8 {
            m.step();
        }
        assert!(m.get("gear/gear-pos-norm") < 0.01, "up in flight");
        m.set("gear/gear-cmd-norm", 1.0);
        m.set("gear/gear-pos-norm", 1.0);
        assert!(m.get("gear/gear-pos-norm") > 0.99, "seated down at once");
    }

    #[test]
    fn f22_gear_retracts_in_flight_and_redeploys() {
        // A gear cycle is 5 s; give each direction 8 s to complete.
        let mut m = f22_cruise(false);
        assert!(m.get("gear/gear-pos-norm") > 0.99, "starts down");
        m.set("gear/gear-cmd-norm", 0.0);
        for _ in 0..120 * 8 {
            m.step();
        }
        assert!(
            m.get("gear/gear-pos-norm") < 0.01,
            "gear should be fully up in flight: {}",
            m.get("gear/gear-pos-norm")
        );
        m.set("gear/gear-cmd-norm", 1.0);
        for _ in 0..120 * 8 {
            m.step();
        }
        assert!(
            m.get("gear/gear-pos-norm") > 0.99,
            "gear should redeploy: {}",
            m.get("gear/gear-pos-norm")
        );
    }

    #[test]
    fn f22_gear_retraction_inhibited_on_the_ground() {
        // On the wheels the gear cannot be raised (weight-on-wheels interlock),
        // so a gear-up command while parked is ignored.
        let mut m = Fdm::new("f22");
        m.set("ic/vc-kts", 0.0);
        m.set("ic/h-sl-ft", 6.0);
        m.set("ic/psi-true-deg", 0.0);
        m.apply_ic();
        m.set("propulsion/set-running", -1.0);
        for _ in 0..120 {
            m.step();
        }
        assert!(m.get("gear/wow") > 0.5, "should be on the wheels");
        m.set("gear/gear-cmd-norm", 0.0);
        for _ in 0..120 * 8 {
            m.step();
        }
        assert!(
            m.get("gear/gear-pos-norm") > 0.99,
            "gear must stay down on the ground: {}",
            m.get("gear/gear-pos-norm")
        );
        assert!(m.get("gear/wow") > 0.5, "and keep carrying the weight");
    }

    #[test]
    fn c172_gear_is_fixed() {
        // The 172 has welded spring-steel gear: the command is a no-op.
        let mut m = Fdm::new("c172p");
        m.set("ic/vc-kts", 0.0);
        m.set("ic/h-sl-ft", 4.7);
        m.apply_ic();
        for _ in 0..120 {
            m.step();
        }
        m.set("gear/gear-cmd-norm", 0.0);
        for _ in 0..120 * 8 {
            m.step();
        }
        assert!(
            m.get("gear/gear-pos-norm") > 0.99,
            "the 172's gear is welded down"
        );
    }

    #[test]
    fn retractable_gear_is_reported_per_aircraft() {
        assert!(Fdm::new("f22").has_retractable_gear());
        assert!(!Fdm::new("c172p").has_retractable_gear());
    }

    #[test]
    fn rotor_and_rest_height_are_reported_per_aircraft() {
        // The AH-1S carries rotors and parks on its skids; the fixed-wing models
        // do neither.
        let heli = Fdm::new("ah1s");
        assert!(heli.has_rotor());
        assert!((heli.rest_h_ft() - 6.0).abs() < 1e-9);
        assert!(!Fdm::new("c172p").has_rotor());
        assert!((Fdm::new("c172p").rest_h_ft() - 4.7).abs() < 1e-9);
        // The 737's reset is `altitude 49 IN` — gear-sole height above the runway.
        assert!((Fdm::new("737").rest_h_ft() - 49.0 / 12.0).abs() < 1e-9);
    }

    #[test]
    fn f22_tvc_enable_roundtrips() {
        let mut m = Fdm::new("f22");
        assert!(m.get("fcs/tvc-enable") > 0.5, "TVC should default on");
        m.set("fcs/tvc-enable", 0.0);
        assert!(m.get("fcs/tvc-enable") < 0.5);
        m.set("fcs/tvc-enable", 1.0);
        assert!(m.get("fcs/tvc-enable") > 0.5);
    }

    /// Cruise-seeded F-22 with thrust vectoring switched as requested, ready for
    /// a pitch input.
    fn f22_cruise(tvc: bool) -> Fdm {
        let mut m = Fdm::new("f22");
        m.set("ic/vc-kts", 350.0);
        m.set("ic/h-sl-ft", 15000.0);
        m.set("ic/psi-true-deg", 0.0);
        m.apply_ic();
        m.set("propulsion/set-running", -1.0);
        m.set("fcs/tvc-enable", if tvc { 1.0 } else { 0.0 });
        m.trim(350.0, 15000.0);
        m
    }

    /// Fly a hard pull to high alpha (1.5 s), then release, and return the peak
    /// angle of attack seen over the next 20 s — the Cobra-like maneuver a pilot
    /// actually flies, as opposed to a sustained max-stick hold.
    fn f22_pull_then_release(tvc: bool) -> f64 {
        let mut m = f22_cruise(tvc);
        let mut peak_alpha = 0.0_f64;
        for i in 0..=120 * 20 {
            if i > 0 {
                m.step();
            }
            m.set("fcs/elevator-cmd-norm", if i <= 180 { -1.0 } else { 0.0 });
            peak_alpha = peak_alpha.max(m.get("aero/alpha-deg"));
        }
        peak_alpha
    }

    #[test]
    fn f22_tvc_on_holds_high_alpha() {
        // With the nozzles backing up the tail, the hard pull stays inside the
        // (raised) envelope: alpha peaks in the high-alpha band and never runs
        // past vertical into a tumble.
        let peak = f22_pull_then_release(true);
        println!("TVC ON peak alpha = {peak:.1}");
        assert!(
            peak < 90.0,
            "TVC on should hold alpha below 90: peak = {peak:.1}"
        );
    }

    #[test]
    fn f22_tvc_off_departs() {
        // Without vectoring the stabilator is authority-limited at high alpha,
        // so the same pull departs into a tumble (alpha runs past inverted).
        let peak = f22_pull_then_release(false);
        println!("TVC OFF peak alpha = {peak:.1}");
        assert!(
            peak > 120.0,
            "TVC off should depart into a tumble: peak = {peak:.1}"
        );
    }

    #[test]
    fn f22_tvc_inhibited_on_ground() {
        // The nozzles are gated off while the gear is down (`fcs/tvc-inhibit`),
        // so on the runway a nose-up command never deflects them.
        let mut m = Fdm::new("f22");
        m.set("ic/vc-kts", 0.0);
        m.set("ic/h-sl-ft", 6.0);
        m.set("ic/psi-true-deg", 0.0);
        m.apply_ic();
        m.set("propulsion/set-running", -1.0);
        m.set("fcs/tvc-enable", 1.0);
        for _ in 0..120 {
            m.set("fcs/elevator-cmd-norm", -1.0);
            m.step();
        }
        assert!(m.get("gear/wow") > 0.5, "should still be on the wheels");
        assert_eq!(
            m.get("fcs/tvc-pos-rad"),
            0.0,
            "TVC must be inhibited on the ground"
        );
    }

    /// A level, cruising F-22 with arcade mode on. No trim is applied: the FBW
    /// trim is bypassed in arcade mode, and the velocity-pinning term holds the
    /// flight path on the nose without it.
    fn f22_arcade() -> Fdm {
        let mut m = Fdm::new("f22");
        m.set("ic/vc-kts", 350.0);
        m.set("ic/h-sl-ft", 15000.0);
        m.set("ic/psi-true-deg", 0.0);
        m.apply_ic();
        m.set("propulsion/set-running", -1.0);
        m.set("fcs/arcade-enable", 1.0);
        m
    }

    #[test]
    fn arcade_is_off_by_default() {
        // The realistic F-22 must be the default; arcade is opt-in through the
        // property bus (which the menu sets from the saved config).
        let mut m = Fdm::new("f22");
        assert_eq!(m.get("fcs/arcade-enable"), 0.0, "arcade must be opt-in");
        m.set("fcs/arcade-enable", 1.0);
        assert_eq!(m.get("fcs/arcade-enable"), 1.0, "the flag must round-trip");
    }

    #[test]
    fn arcade_tracks_a_pitch_rate_command() {
        // Elevator is nose-down positive, so half nose-up stick commands
        // +0.5·RATE_Q_UP of pitch rate. The synthesised moment should drive the
        // body rate straight onto that command with the TAU_ROT lag.
        let mut m = f22_arcade();
        for _ in 0..120 {
            m.set("fcs/elevator-cmd-norm", -0.5);
            m.step();
        }
        let q = m.get("velocities/q-rad_sec");
        let want = 0.5 * arcade::RATE_Q_UP;
        println!("arcade q = {q:.3} rad/s (want {want:.3})");
        assert!(
            (q - want).abs() < 0.15 * want,
            "pitch rate should reach the command: q = {q:.3}, want {want:.3}"
        );
        assert!(
            m.get("attitude/theta-deg") > 5.0,
            "the nose should have come up"
        );
    }

    #[test]
    fn arcade_points_the_velocity_at_the_nose() {
        // The realistic F-22 departs into a tumble under a hard sustained pull
        // (see `f22_tvc_off_departs`). Arcade drops the aero normal force and
        // pins the velocity to the nose, so alpha stays small — no stall, no
        // spin — while the nose keeps coming up.
        let mut m = f22_arcade();
        let mut peak_alpha = 0.0_f64;
        for i in 0..120 * 3 {
            m.set("fcs/elevator-cmd-norm", -1.0);
            m.step();
            if i % 12 == 0 {
                peak_alpha = peak_alpha.max(m.get("aero/alpha-deg"));
            }
        }
        println!("arcade peak alpha under a hard pull = {peak_alpha:.1} deg");
        assert!(
            peak_alpha < 45.0,
            "arcade should hold alpha well below departure: peak = {peak_alpha:.1}"
        );
    }

    #[test]
    fn arcade_roll_follows_the_aileron() {
        // Full right stick should command RATE_P rad/s of roll and bank the jet.
        let mut m = f22_arcade();
        for _ in 0..60 {
            m.set("fcs/aileron-cmd-norm", 1.0);
            m.step();
        }
        let p = m.get("velocities/p-rad_sec");
        println!("arcade p = {p:.3} rad/s");
        assert!(
            p > 0.5 * arcade::RATE_P,
            "full aileron should roll right fast: p = {p:.3}"
        );
        assert!(
            m.get("attitude/phi-deg") > 10.0,
            "the roll should have banked the jet"
        );
    }

    #[test]
    fn arcade_trim_leaves_the_jet_level() {
        // The FBW trim seats an elevator the arcade rules ignore, so `trim` is a
        // no-op there: it must not move the stick, and a level jet stays level.
        let mut m = f22_arcade();
        m.trim(350.0, 15000.0);
        assert_eq!(
            m.get("fcs/elevator-cmd-norm"),
            0.0,
            "arcade trim must not move the stick"
        );
        for _ in 0..12 {
            m.step();
        }
        assert!(
            m.get("velocities/q-rad_sec").abs() < 1e-3,
            "a level arcade jet should not pitch on its own"
        );
    }

    // ---------------------------- Boeing 737 -------------------------------

    /// Parked on the runway at the 737's reset height (49 in), engine running.
    fn parked_737() -> Fdm {
        let mut m = Fdm::new("737");
        m.set("ic/vc-kts", 0.0);
        m.set("ic/h-sl-ft", 49.0 / 12.0);
        m.set("ic/psi-true-deg", 0.0);
        m.apply_ic();
        m.set("propulsion/set-running", -1.0);
        m
    }

    #[test]
    fn the_jet_coasts_gently_at_idle() {
        // Idle thrust (~1,670 lbf) nearly balances the 0.012 tyre's rolling
        // resistance: from 10 kt at idle she bleeds off slowly, not halts
        // dead. The generic 0.02 tyre made idle a net 1,800 lbf brake.
        let mut m = Fdm::new("737");
        m.set("ic/vc-kts", 10.0);
        m.set("ic/h-sl-ft", 49.0 / 12.0);
        m.set("ic/psi-true-deg", 0.0);
        m.apply_ic();
        m.set("propulsion/set-running", -1.0);
        m.set("fcs/throttle-cmd-norm", 0.0);
        for _ in 0..120 * 10 {
            m.step();
        }
        let kt = m.get("velocities/vg-fps") / KT_TO_FPS;
        assert!(kt >= 9.0, "idle bleeds off gently, got {kt:.2} kt");
        assert!(kt <= 10.1, "and never accelerates on its own: {kt:.2} kt");
    }

    #[test]
    fn a_small_bump_breaks_the_jet_away() {
        // 30% thrust against the rolling tyre moves 172,600 lb: the
        // break-away bump of the taxi drill.
        let mut m = parked_737();
        m.set("fcs/throttle-cmd-norm", 0.30);
        for _ in 0..120 * 30 {
            m.step();
        }
        let kt = m.get("velocities/vg-fps") / KT_TO_FPS;
        assert!(kt > 3.0, "a 30% bump gets her rolling, got {kt:.2} kt");
    }

    /// A 737 at a cruise condition, trimmed and holding. Installs the engines
    /// first so the trim solver sees their thrust.
    fn cruise_737(vc: f64, alt: f64) -> Fdm {
        let mut m = Fdm::new("737");
        m.set("ic/vc-kts", vc);
        m.set("ic/h-sl-ft", alt);
        m.set("ic/psi-true-deg", 0.0);
        m.apply_ic();
        m.set("propulsion/set-running", -1.0);
        m.trim(vc, alt);
        m
    }

    #[test]
    fn position_ics_seat_off_the_origin() {
        // The approach gate: 5140 m south (negative lat), parked altitude.
        let mut m = Fdm::new("737");
        m.set("ic/lat-gc-deg", -0.046_206);
        m.set("ic/h-sl-ft", 1300.0);
        m.set("ic/vc-kts", 155.0);
        m.apply_ic();
        let lat = m.get("position/lat-gc-deg");
        let north_m = (lat * D2R) * 6_378_137.0;
        assert!(
            (north_m + 5140.0).abs() < 5.0,
            "should be 5140 m south, got {north_m}"
        );
        // And a parked reset at the default puts it back over the origin.
        let mut m = Fdm::new("737");
        m.set("ic/vc-kts", 0.0);
        m.apply_ic();
        assert!((m.get("position/lat-gc-deg")).abs() < 1e-9);
    }

    #[test]
    fn b737_constructs_and_is_named() {
        let m = Fdm::new("737");
        assert_eq!(m.aircraft(), "737");
        assert!(!m.has_tvc(), "the 737 has no thrust vectoring");
        assert!(!m.has_rotor(), "the 737 has no rotor");
        assert!(m.has_retractable_gear(), "the 737 retracts its gear");
    }

    #[test]
    fn b737_parks_on_gear() {
        // The gear carries the ~107 000 lb loaded weight: from the reset height
        // the aircraft settles onto the struts and stays at rest.
        let mut m = parked_737();
        for _ in 0..120 * 5 {
            m.step();
        }
        let h = m.get("position/h-sl-ft");
        assert!(h > 0.0, "the aircraft sank through the ground: h = {h}");
        assert!((3.0..6.0).contains(&h), "unexpected settled height: {h}");
        assert!(m.get("gear/wow") > 0.5, "should report weight-on-wheels");
        assert!(
            m.get("velocities/h-dot-fps").abs() < 1.0,
            "should be at rest"
        );
    }

    #[test]
    fn b737_full_throttle_rolls_forward() {
        // Two CFM56s at full throttle should roll the airliner forward down the
        // runway (heading north) without it sinking or veering.
        let mut m = parked_737();
        m.set("fcs/throttle-cmd-norm", 1.0);
        let mut min_h = f64::INFINITY;
        for _ in 0..120 * 8 {
            m.step();
            min_h = min_h.min(m.get("position/h-sl-ft"));
        }
        assert!(
            min_h > 0.0,
            "sank through the ground during the roll: {min_h}"
        );
        assert!(
            m.get("position/lat-gc-deg") > 0.0,
            "should have rolled forward"
        );
        assert!(m.get("velocities/vc-kts") > 10.0, "should have accelerated");
    }

    #[test]
    fn b737_brakes_stop_the_roll() {
        let mut m = parked_737();
        m.set("fcs/throttle-cmd-norm", 1.0);
        for _ in 0..120 * 4 {
            m.step();
        }
        let vc_peak = m.get("velocities/vc-kts");
        assert!(
            vc_peak > 5.0,
            "should build speed before braking, got {vc_peak}"
        );
        m.set("fcs/throttle-cmd-norm", 0.0);
        m.set("fcs/left-brake-cmd-norm", 1.0);
        m.set("fcs/right-brake-cmd-norm", 1.0);
        for _ in 0..120 * 6 {
            m.step();
        }
        let vc = m.get("velocities/vc-kts");
        assert!(
            vc < 0.5 * vc_peak,
            "brakes should slow the aircraft: {vc_peak} -> {vc}"
        );
    }

    #[test]
    fn b737_nosewheel_steers_left_with_rudder() {
        // The rudder drives the steerable nosewheel (35° throw) on the ground, so
        // a held left-rudder input turns the nose left.
        let mut m = parked_737();
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
        assert!(
            dpsi < -5.0,
            "left rudder should turn the nose left, dpsi = {dpsi}"
        );
    }

    #[test]
    fn b737_speedbrake_deploys_slow_and_costs_lift() {
        // Command the boards out and confirm the 0.6 s kinematic traverse, then
        // hold wings-level at the same speed and confirm the aerodynamic lift
        // coefficient falls (the kCLsb factor) versus the clean wing.
        let mut m = cruise_737(160.0, 3_000.0);
        // Flush the one-frame induced-drag feedback into a live weight.
        for _ in 0..2 {
            m.step();
        }
        let cl_clean = m.get("aero/cl-squared");
        m.set("fcs/speedbrake-cmd-norm", 1.0);
        for _ in 0..120 {
            m.step();
        }
        let pos = m.get("fcs/speedbrake-pos-norm");
        assert!(pos > 0.9, "boards should be out after 1 s, got {pos}");
        let cl_open = m.get("aero/cl-squared");
        assert!(
            cl_open < cl_clean * 0.95,
            "spoilers should shed lift: {cl_clean} -> {cl_open}"
        );
        // Stow back down; the position drives back to zero.
        m.set("fcs/speedbrake-cmd-norm", 0.0);
        for _ in 0..120 {
            m.step();
        }
        assert!(
            m.get("fcs/speedbrake-pos-norm") < 0.05,
            "boards should stow"
        );
    }

    #[test]
    fn b737_reverse_bites_on_the_ground_and_stows_airborne() {
        // Parked: reverse the thrust and confirm `engine-thrust-lbs` goes
        // negative (a net backward force) at full reverse.
        let mut m = parked_737();
        m.set("fcs/reverse-cmd-norm", 1.0);
        for _ in 0..120 * 2 {
            m.step();
        }
        assert!(m.get("fcs/reverse-pos-norm") > 0.9, "levers should be back");
        let t = m.get("propulsion/engine[0]/thrust-lbs");
        assert!(t < -1000.0, "full reverse should push backward, got {t}");

        // Airborne: the interlock drives the levers stowed and the thrust term
        // returns to forward-only.
        let mut m = cruise_737(250.0, 10_000.0);
        m.set("fcs/reverse-cmd-norm", 1.0);
        for _ in 0..120 * 2 {
            m.step();
        }
        assert!(
            m.get("fcs/reverse-pos-norm") < 0.05,
            "reverse must not deploy in the air"
        );
        assert!(
            m.get("propulsion/engine[0]/thrust-lbs") > 0.0,
            "thrust stays forward airborne"
        );
    }

    #[test]
    fn b737_trims_to_cruise() {
        // The airliner's cruise trim should seat with a sensible part throttle
        // (not full power) and a small positive angle of attack.
        let m = cruise_737(285.0, 30000.0);
        let thr = m.get("fcs/throttle-cmd-norm");
        let alpha = m.get("aero/alpha-deg");
        println!(
            "737 cruise trim: theta={} alpha={alpha:.2} throttle={thr:.3} h={} vc={}",
            m.get("attitude/theta-deg"),
            m.get("position/h-sl-ft"),
            m.get("velocities/vc-kts"),
        );
        assert!(thr < 1.0, "cruise should not need full throttle: {thr}");
        assert!(thr > 0.05, "trim throttle collapsed to idle: {thr}");
        assert!(
            (0.0..10.0).contains(&alpha),
            "unexpected cruise alpha: {alpha}"
        );
    }

    #[test]
    fn b737_cruise_trim_holds() {
        // A real trim is a fixed point of the integrator: 30 s flown open-loop
        // should barely move.
        let mut m = cruise_737(285.0, 30000.0);
        let (h0, vc0) = (m.get("position/h-sl-ft"), m.get("velocities/vc-kts"));
        for _ in 0..120 * 30 {
            m.step();
        }
        let h = m.get("position/h-sl-ft");
        let vc = m.get("velocities/vc-kts");
        println!("737 cruise 30 s: h {h0:.0} -> {h:.0}, vc {vc0:.1} -> {vc:.1}");
        assert!(
            (h - h0).abs() < 50.0,
            "trim drifted in altitude: {h0} -> {h}"
        );
        assert!(
            (vc - vc0).abs() < 2.0,
            "trim drifted in airspeed: {vc0} -> {vc}"
        );
    }

    #[test]
    fn b737_roll_input_banks_and_turns() {
        // No steady-turn trim API, so this is a weaker check: a lateral input
        // banks the airliner and the bank develops a turn (heading changes).
        let mut m = cruise_737(285.0, 30000.0);
        for _ in 0..120 {
            m.set("fcs/aileron-cmd-norm", 0.3);
            m.step();
        }
        m.set("fcs/aileron-cmd-norm", 0.0);
        let phi0 = m.get("attitude/phi-deg");
        let psi0 = m.get("attitude/psi-deg");
        assert!(
            phi0.abs() > 3.0,
            "aileron should bank the aircraft: phi = {phi0}"
        );
        for _ in 0..120 * 20 {
            m.step();
        }
        let dpsi = m.get("attitude/psi-deg") - psi0;
        println!(
            "737 roll-into-turn: phi {phi0:.1} -> {}, dpsi {dpsi:.1}",
            m.get("attitude/phi-deg")
        );
        assert!(
            dpsi.abs() > 5.0,
            "a banked aircraft should turn: dpsi = {dpsi}"
        );
    }

    /// An AH-1S hovering at `alt` ft: still and level, engine lit (the governor
    /// then holds the rotor at nominal RPM), trimmed to hover.
    fn hovering(alt: f64) -> Fdm {
        let mut m = Fdm::new("ah1s");
        m.set("ic/vc-kts", 0.0);
        m.set("ic/h-sl-ft", alt);
        m.set("ic/psi-true-deg", 0.0);
        m.apply_ic();
        m.set("propulsion/set-running", -1.0);
        m.trim(0.0, alt);
        m
    }

    #[test]
    fn helicopter_hovers() {
        // The hover trim must be a true fixed point: the model holds altitude,
        // stays put horizontally, and holds heading, with the rotor thrust
        // carrying the 8500 lb weight.
        let mut m = hovering(2000.0);
        for _ in 0..120 * 10 {
            m.step();
        }
        assert!(
            (m.get("position/h-sl-ft") - 2000.0).abs() < 20.0,
            "altitude drifted: {}",
            m.get("position/h-sl-ft")
        );
        assert!(
            m.get("velocities/h-dot-fps").abs() < 2.0,
            "not settled vertically: {}",
            m.get("velocities/h-dot-fps")
        );
        assert!(m.get("velocities/vc-kts") < 2.0, "drifting horizontally");
        assert!(
            m.get("velocities/r-rad_sec").abs() < 0.05,
            "yawing at hover: {}",
            m.get("velocities/r-rad_sec")
        );
        let t = m.get("propulsion/engine[0]/thrust-lbs");
        assert!(
            (t - 8500.0).abs() < 250.0,
            "hover thrust {t} should match the 8500 lb weight"
        );
    }

    #[test]
    fn helicopter_trims_in_forward_flight() {
        // The trim honors the commanded airspeed: seat level flight across a
        // speed range and confirm each is a fixed point (speed and altitude
        // hold). Zero reduces to the hover case.
        for vc in [0.0, 40.0, 80.0, 120.0] {
            let mut m = Fdm::new("ah1s");
            m.set("ic/vc-kts", 0.0);
            m.set("ic/h-sl-ft", 2000.0);
            m.set("ic/psi-true-deg", 0.0);
            m.apply_ic();
            m.set("propulsion/set-running", -1.0);
            m.trim(vc, 2000.0);

            let vc0 = m.get("velocities/vc-kts");
            let alt0 = m.get("position/h-sl-ft");
            for _ in 0..120 * 10 {
                m.step();
            }
            assert!(
                (m.get("velocities/vc-kts") - vc0).abs() < 5.0,
                "speed drifted from {vc} kt: {vc0} -> {}",
                m.get("velocities/vc-kts")
            );
            assert!(
                (m.get("position/h-sl-ft") - alt0).abs() < 30.0,
                "altitude drifted at {vc} kt: {alt0} -> {}",
                m.get("position/h-sl-ft")
            );
        }
    }

    #[test]
    fn hover_trim_keeps_position_and_heading() {
        // A trim taken airborne must hold the current ground track and heading,
        // not teleport the aircraft back over the origin (the FDM's lat/lon
        // origin). Fly east a couple of seconds, then re-trim.
        let mut m = hovering(2000.0);
        m.set("ic/psi-true-deg", 90.0);
        m.apply_ic();
        m.set("propulsion/set-running", -1.0);
        m.trim(0.0, 2000.0);
        m.set("fcs/elevator-cmd-norm", 0.5); // nose down -> accelerate forward
        for _ in 0..120 * 6 {
            m.step();
        }

        let lat0 = m.get("position/lat-gc-deg");
        let lon0 = m.get("position/long-gc-deg");
        let psi0 = m.get("attitude/psi-deg");
        assert!(lon0.abs() > 1e-4, "should have flown east: lon = {lon0}");

        // Mimic the app's re-trim: seat the current heading, then trim.
        m.set("ic/psi-true-deg", psi0);
        m.trim(m.get("velocities/vc-kts"), m.get("position/h-sl-ft"));

        assert!(
            (m.get("position/lat-gc-deg") - lat0).abs() < 1e-6,
            "trim moved latitude: {lat0} -> {}",
            m.get("position/lat-gc-deg")
        );
        assert!(
            (m.get("position/long-gc-deg") - lon0).abs() < 1e-6,
            "trim moved longitude: {lon0} -> {}",
            m.get("position/long-gc-deg")
        );
        assert!(
            (m.get("attitude/psi-deg") - psi0).abs() < 1e-3,
            "trim changed heading: {psi0} -> {}",
            m.get("attitude/psi-deg")
        );
    }

    #[test]
    fn collective_commands_a_climb() {
        // Raising collective above the hover setting adds thrust and climbs.
        let mut m = hovering(2000.0);
        m.set("fcs/collective-cmd-norm", 1.0);
        for _ in 0..120 * 5 {
            m.step();
        }
        assert!(
            m.get("position/h-sl-ft") > 2020.0,
            "raising collective should climb: h = {}",
            m.get("position/h-sl-ft")
        );
        assert!(
            m.get("velocities/h-dot-fps") > 1.0,
            "should be climbing: hdot = {}",
            m.get("velocities/h-dot-fps")
        );
    }

    #[test]
    fn tail_rotor_counters_main_torque() {
        // Trimmed, the pedal's tail-rotor thrust cancels the main rotor's
        // reaction torque, so the hover holds heading. Drop the pedal to neutral
        // and the torque yaws the airframe.
        let mut m = hovering(2000.0);
        for _ in 0..120 * 5 {
            m.step();
        }
        assert!(
            m.get("velocities/r-rad_sec").abs() < 0.05,
            "trimmed hover should hold heading: r = {}",
            m.get("velocities/r-rad_sec")
        );
        m.set("fcs/rudder-cmd-norm", 0.0);
        for _ in 0..120 * 3 {
            m.step();
        }
        assert!(
            m.get("velocities/r-rad_sec").abs() > 0.05,
            "unbalanced main-rotor torque should yaw the airframe: r = {}",
            m.get("velocities/r-rad_sec")
        );
    }

    #[test]
    fn ah1s_parks_on_skids() {
        // Idle on the ground: the skids hold the weight, the aircraft settles at
        // a stable height and stays at rest, without sinking through.
        let mut m = Fdm::new("ah1s");
        m.set("ic/vc-kts", 0.0);
        m.set("ic/h-sl-ft", 6.0);
        m.set("ic/psi-true-deg", 0.0);
        m.apply_ic();
        m.set("propulsion/set-running", -1.0);
        for _ in 0..120 * 8 {
            m.step();
        }
        let h = m.get("position/h-sl-ft");
        assert!(h > 4.0, "sank through the skids: h = {h}");
        assert!((5.5..7.0).contains(&h), "unexpected settled height: {h}");
        assert!(m.get("gear/wow") > 0.5, "should rest on the skids");
        assert!(
            m.get("velocities/h-dot-fps").abs() < 1.0,
            "should be at rest"
        );
    }

    #[test]
    fn ah1s_cyclic_commands_pitch_and_roll() {
        // Longitudinal cyclic tilts the rotor disk forward: from a hover a
        // nose-down command should pitch the nose down and accelerate forward;
        // lateral cyclic rolls the airframe.
        let mut m = hovering(2000.0);
        m.set("fcs/elevator-cmd-norm", 0.5);
        for _ in 0..120 * 2 {
            m.step();
        }
        assert!(
            m.get("attitude/theta-rad") < -0.01,
            "longitudinal cyclic should pitch the nose down: theta = {}",
            m.get("attitude/theta-rad")
        );
        assert!(
            m.get("velocities/vc-kts") > 1.0,
            "should accelerate forward"
        );

        let mut m = hovering(2000.0);
        m.set("fcs/aileron-cmd-norm", 0.5);
        for _ in 0..120 * 2 {
            m.step();
        }
        assert!(
            m.get("attitude/phi-rad").abs() > 0.01,
            "lateral cyclic should roll the airframe: phi = {}",
            m.get("attitude/phi-rad")
        );
    }

    #[test]
    fn ah1s_pitch_disturbance_is_bounded() {
        // The horizontal tail carries the airframe's pitch stability, through
        // three moments: the tail-lift term (`Ma_ht`, from `CLa_ht`) and the
        // stiffness/damping pair (`CMw_ht`/`CMq_ht`). Without them a cyclic
        // disturbance diverges (nose keeps dropping, model pitches over); with
        // them the response is a bounded, lightly-damped phugoid. Trim at cruise,
        // kick the cyclic, release, and confirm the pitch never runs away.
        let mut m = Fdm::new("ah1s");
        m.set("ic/vc-kts", 0.0);
        m.set("ic/h-sl-ft", 2000.0);
        m.set("ic/psi-true-deg", 0.0);
        m.apply_ic();
        m.set("propulsion/set-running", -1.0);
        m.trim(80.0, 2000.0);

        // 0.5 s nose-down pulse, then release back to the trim seat (zero cyclic).
        for _ in 0..120 / 2 {
            m.set("fcs/elevator-cmd-norm", 0.15);
            m.step();
        }

        let mut max_theta: f64 = 0.0;
        for _ in 0..120 * 30 {
            m.step();
            max_theta = max_theta.max(m.get("attitude/theta-rad").abs());
        }
        assert!(
            max_theta < 0.5,
            "pitch ran away after a disturbance: max |theta| = {max_theta:.3} rad"
        );
        assert!(
            m.get("velocities/vc-kts") > 20.0,
            "speed collapsed: {} kt",
            m.get("velocities/vc-kts")
        );
    }

    #[test]
    fn ah1s_wing_lift_builds_up_with_speed() {
        // The wing and fuselage lift/drag/side build-up is speed-dependent: in
        // hover the dynamic pressure is zero and the rotor carries the whole
        // weight (no airframe aero force), while at cruise the airframe makes a
        // real upward lift and backward drag of its own.
        let mut m = Fdm::new("ah1s");
        m.set("ic/vc-kts", 0.0);
        m.set("ic/h-sl-ft", 2000.0);
        m.set("ic/psi-true-deg", 0.0);
        m.apply_ic();
        m.set("propulsion/set-running", -1.0);
        m.trim(0.0, 2000.0);
        m.step();
        assert!(
            m.get("forces/fbz-aero-lbs").abs() < 1.0,
            "hover should have no airframe aero force: {}",
            m.get("forces/fbz-aero-lbs")
        );

        m.trim(100.0, 2000.0);
        m.step();
        assert!(
            m.get("forces/fbz-aero-lbs") < -100.0,
            "the wing should carry lift at cruise: {} lbs",
            m.get("forces/fbz-aero-lbs")
        );
        assert!(
            m.get("forces/fbx-aero-lbs") < -200.0,
            "the airframe should make drag at cruise: {} lbs",
            m.get("forces/fbx-aero-lbs")
        );
    }
}

#[cfg(test)]
mod heli_arcade_tests {
    use super::*;

    fn parked_heli() -> Fdm {
        let mut m = Fdm::new("ah1s");
        m.set("ic/vc-kts", 0.0);
        m.set("ic/h-sl-ft", m.rest_h_ft());
        m.set("ic/psi-true-deg", 0.0);
        m.apply_ic();
        m.set("gear/gear-cmd-norm", 1.0);
        m.set("gear/gear-pos-norm", 1.0);
        m.set("propulsion/set-running", -1.0);
        m.set("fcs/arcade-enable", 1.0);
        for _ in 0..240 {
            m.step();
        }
        m
    }

    #[test]
    fn heli_arcade_hover_from_the_parked_ic() {
        // The player can press J before the first ground step; the parked
        // height must still count as "ground" so it takes off vertically.
        let mut m = Fdm::new("ah1s");
        m.set("ic/vc-kts", 0.0);
        m.set("ic/h-sl-ft", m.rest_h_ft());
        m.set("ic/psi-true-deg", 0.0);
        m.apply_ic();
        m.set("gear/gear-cmd-norm", 1.0);
        m.set("gear/gear-pos-norm", 1.0);
        m.set("propulsion/set-running", -1.0);
        m.set("fcs/arcade-enable", 1.0);
        m.set("fcs/heli-hover-enable", 1.0);
        for _ in 0..120 * 4 {
            m.step();
        }
        assert!(m.get("position/h-sl-ft") > m.rest_h_ft() + 5.0);
        assert!(m.get("gear/wow") < 0.5);
        assert!((m.get("fcs/heli-hover-state") - 2.0).abs() < 0.1);
    }

    #[test]
    fn heli_arcade_takes_off_hovers_and_lands() {
        let mut m = parked_heli();
        let ground = m.get("position/h-sl-ft");
        assert!(m.get("gear/wow") > 0.5, "should be parked on the gear");

        m.set("fcs/heli-hover-enable", 1.0);
        for _ in 0..120 * 8 {
            m.step();
        }
        let hover = m.get("position/h-sl-ft");
        assert!(hover > ground + 5.0, "hover altitude {hover} vs {ground}");
        assert!(m.get("gear/wow") < 0.5, "should be airborne");
        assert!(
            (m.get("fcs/heli-hover-state") - 2.0).abs() < 0.1,
            "state should be Hold"
        );

        m.set("fcs/heli-hover-enable", 0.0);
        for _ in 0..120 * 15 {
            m.step();
        }
        let landed = m.get("position/h-sl-ft");
        assert!(landed < hover - 5.0, "should descend: {landed} vs {hover}");
        assert!(m.get("gear/wow") > 0.5, "should touch down");
        assert!(
            m.get("fcs/heli-hover-enable") < 0.5,
            "auto-hover should switch off on landing"
        );
    }

    #[test]
    fn heli_arcade_auto_balance_levels_after_a_bank() {
        let mut m = parked_heli();
        m.set("ic/vc-kts", 0.0);
        m.set("ic/h-sl-ft", 60.0);
        m.apply_ic();
        m.set("fcs/arcade-enable", 1.0);
        m.set("fcs/heli-balance-enable", 1.0);
        // Hold an aileron input long enough to establish a bank.
        m.set("fcs/aileron-cmd-norm", 1.0);
        for _ in 0..120 * 2 {
            m.step();
        }
        let banked = m.get("attitude/phi-deg");
        assert!(banked > 10.0, "should bank right, got {banked}");
        // Release: the attitude loop should bring it back toward level.
        m.set("fcs/aileron-cmd-norm", 0.0);
        for _ in 0..120 * 5 {
            m.step();
        }
        let level = m.get("attitude/phi-deg");
        assert!(level.abs() < 5.0, "should return to level, got {level}");
    }
}

#[cfg(test)]
mod fuel_tests {
    use super::*;

    /// Cold and parked at the threshold, like the app's ENGINE = COLD spawn.
    fn parked_cold() -> Fdm {
        let mut m = Fdm::new("c172p");
        m.set("ic/vc-kts", 0.0);
        m.set("ic/h-sl-ft", 4.7);
        m.set("ic/psi-true-deg", 0.0);
        m.apply_ic();
        m.set("propulsion/ignition-enable", 0.0);
        m
    }

    /// Parked with the engine running (the app's default spawn).
    fn parked_running() -> Fdm {
        let mut m = parked_cold();
        m.set("propulsion/set-running", 1.0);
        m
    }

    #[test]
    fn engine_burns_fuel_and_reports_it() {
        let mut m = parked_running();
        let before = m.get("propulsion/tank-0-contents-lbs");
        // Full throttle on the ramp: the flow should be plausibly C172-shaped
        // (the engine manages ~50-60 pph at high power) and the left tank must
        // tick down as the engine burns.
        m.set("fcs/throttle-cmd-norm", 1.0);
        for _ in 0..120 * 10 {
            m.step();
        }
        let flow = m.get("propulsion/engine[0]/fuel-flow-pph");
        assert!(
            (30.0..80.0).contains(&flow),
            "fuel flow out of piston range: {flow} pph"
        );
        let after = m.get("propulsion/tank-0-contents-lbs");
        assert!(after < before - 0.1, "tank should burn: {before} -> {after}");
    }

    #[test]
    fn dry_tank_flames_out_and_refuses_the_light() {
        let mut m = parked_running();
        // A cup of fuel per tank: high power burns it in a second or two.
        m.fuel = [0.02, 0.02];
        m.set("fcs/throttle-cmd-norm", 1.0);
        for _ in 0..120 * 6 {
            m.step();
        }
        assert!(m.fuel[0] == 0.0 && m.fuel[1] == 0.0, "tanks should run dry");
        assert!(m.engine_power_hp < 1.0, "a starved engine makes no power");
        assert!(m.engine_rpm < 1200.0, "rpm should be spinning down, was {}", m.engine_rpm);
        // The spawn shortcut can't relight it: there is nothing to burn.
        m.set("propulsion/set-running", 1.0);
        for _ in 0..120 {
            m.step();
        }
        assert!(
            m.engine_power_hp < 1.0,
            "refusing the light with an empty tank"
        );
        // ...but the pump can. Refuelling alone must not relight the engine.
        m.refuel();
        assert!(m.fuel[0] > 100.0, "the pump fills the tanks");
        assert!(m.engine_power_hp < 1.0, "refuelling must not relight");
        // Spark + starter do.
        m.set("propulsion/ignition-enable", 1.0);
        m.set("propulsion/engine[0]/starter-cmd-norm", 1.0);
        for _ in 0..120 * 5 {
            m.step();
        }
        assert!(
            m.engine_rpm > 900.0,
            "a cold engine cranks up and fires, rpm = {}",
            m.engine_rpm
        );
    }

    #[test]
    fn cold_start_route_lights_only_with_spark_and_crank() {
        let mut m = parked_cold();
        // Cranking without spark spins the prop but never fires.
        m.set("propulsion/engine[0]/starter-cmd-norm", 1.0);
        for _ in 0..120 * 5 {
            m.step();
        }
        assert!(m.engine_power_hp < 1.0, "no spark: no fire");
        let crank_rpm = m.engine_rpm;
        assert!(crank_rpm > 100.0, "the starter should crank, rpm = {crank_rpm}");
        // Spark arrives mid-crank: it should catch.
        m.set("propulsion/ignition-enable", 1.0);
        for _ in 0..120 * 4 {
            m.step();
        }
        assert!(
            m.engine_power_hp > 5.0 && m.engine_rpm > 500.0,
            "fires once spark arrives: rpm = {}, hp = {}",
            m.engine_rpm,
            m.engine_power_hp
        );
        // Release the starter: the engine keeps running on its own.
        m.set("propulsion/engine[0]/starter-cmd-norm", 0.0);
        for _ in 0..120 * 5 {
            m.step();
        }
        assert!(
            m.engine_rpm > 500.0,
            "running engine must not stall with the starter released, rpm = {}",
            m.engine_rpm
        );
    }

    #[test]
    fn burning_fuel_makes_the_mass_properties_follow() {
        let mut m = parked_running();
        let w0 = m.mass * 32.174049; // slug back to lb for readability
        // Drain the tanks by hand, rebuild, and check the totals.
        m.fuel = [0.0, 0.0];
        m.fuel_built = [-1.0, -1.0]; // force the rebuild threshold
        m.rebuild_mass();
        let w_dry = m.mass * 32.174049;
        assert!(
            (w_dry - 1680.0).abs() < 2.0,
            "empty tanks: empty + pilot = 1680 lb, got {w_dry}"
        );
        assert!(w_dry < w0 - 190.0, "burning fuel lightens the aircraft");
        // Full tanks: 1500 + 180 + 2×185.
        m.refuel();
        let w_full = m.mass * 32.174049;
        assert!(
            (w_full - 2050.0).abs() < 2.0,
            "full tanks: 2050 lb, got {w_full}"
        );
    }
}
