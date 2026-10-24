//! Bridges a flight dynamics model into the Bevy world: drives the FDM at a
//! fixed rate and converts its geodetic state into Bevy-space position and
//! orientation.
//!
//! The model sits behind [`flight_model::FlightModel`], a named-property bus, so
//! this file (and everything above it — `input`, `view`, the HUD, the autopilot)
//! is identical whichever backend is loaded. The pure-Rust engine in `fdm` is
//! the default (it flies, can be trimmed, and implements arcade mode);
//! `FDM_BACKEND=jsbsim` selects the JSBSim model in `jsbsim-sys`, whose `trim`
//! is a no-op.

use bevy::prelude::*;
use flight_model::FlightModel;

use crate::app_state::unpaused;
use crate::autopilot::{Autopilot, Mcp};

/// The menu rebuilds [`Sim`] for the selected aircraft when flying starts.
/// Per-flight spawners that branch on `sim.name` must run `.after` this set,
/// or they may read the previous flight's aircraft.
#[derive(SystemSet, Debug, Clone, PartialEq, Eq, Hash)]
pub struct StartFlight;

/// FDM step. Matches the rate JSBSim was validated against.
pub const DT: f64 = 1.0 / 120.0;

/// One knot in feet per second.
pub const FPS_TO_KT: f64 = 0.592_483_801;

/// Default cruise point for the airborne trimmed start.
#[cfg(test)]
const CRUISE_ALT_FT: f64 = 3000.0;
#[cfg(test)]
const CRUISE_VC_KT: f64 = 110.0;

const FT_TO_M: f64 = 0.3048;
const EARTH_RADIUS_M: f64 = 6_378_137.0;

/// NED -> world basis (east, up, south) folded with the body -> mesh basis.
/// JSBSim body axes are (x fwd, y right, z down); the mesh's local axes are
/// (x right, y up, -z fwd). Both maps share this matrix, so the aircraft's
/// world rotation is `S · R_nb · Sᵀ` (R_nb = body -> NED). Transposing R_nb
/// here mirrors the aircraft: heading renders reversed and pitch/roll invert.
fn axis_map() -> Mat3 {
    Mat3::from_cols(
        Vec3::new(0.0, 0.0, -1.0),
        Vec3::new(1.0, 0.0, 0.0),
        Vec3::new(0.0, -1.0, 0.0),
    )
}

#[derive(Resource)]
pub struct Sim {
    /// The loaded flight model, behind the neutral property bus.
    pub model: Box<dyn FlightModel>,
    /// Human-readable "<backend> <aircraft>" for the HUD title.
    pub label: String,
    /// Aircraft id ("c172p" / "f22"), for choosing the visual model.
    pub name: String,
    origin_lat: f64,
    origin_lon: f64,
    /// Highest load factor (`forces/load-factor`) since the last reset, for the
    /// F-22 HUD's max-G readout.
    max_g: f64,
    /// Smoothed rate of change of KCAS (kt/s), for the HUD's accel/decel caret.
    speed_trend: f64,
    last_cas_kts: f64,
    /// Cold-and-dark state (c172p only): resets hand the pilot a shut-down
    /// aircraft — no spark, prop stopped — and the start flow is on them.
    pub engine_cold: bool,
}

/// The aircraft the app starts with: `AIRCRAFT=f22` overrides the c172 default.
/// This is only the *initial* selection — the menu can pick any aircraft after
/// launch — so the env read lives here rather than inside [`Sim::new`].
pub fn startup_aircraft() -> String {
    std::env::var("AIRCRAFT").unwrap_or_else(|_| "c172p".to_owned())
}

impl Sim {
    pub fn new(name: &str) -> Self {
        // Backend selection. The pure-Rust engine is the default;
        // `FDM_BACKEND=jsbsim` opts back into the C++ model.
        let rust = std::env::var("FDM_BACKEND").as_deref() != Ok("jsbsim");
        Self::build(name, rust)
    }

    /// Arcade missions always run the Rust engine: `fcs/arcade-enable` is
    /// implemented there, while the JSBSim aircraft models ignore it.
    pub fn new_arcade(name: &str) -> Self {
        Self::build(name, true)
    }

    fn build(name: &str, rust: bool) -> Self {
        let (model, backend): (Box<dyn FlightModel>, &str) = if rust {
            (Box::new(fdm::Fdm::new(name)), "Rust")
        } else {
            (Box::new(jsbsim_sys::Fdm::load(name, DT)), "JSBSim")
        };
        let display = if name == "ah1s" { "AH-64D" } else { name };
        let label = format!("{backend} {display}");
        let mut sim = Sim {
            model,
            label,
            name: name.to_owned(),
            origin_lat: 0.0,
            origin_lon: 0.0,
            max_g: 0.0,
            speed_trend: 0.0,
            last_cas_kts: 0.0,
            engine_cold: false,
        };
        // Hard sea floor for gear-up flight: the model has no belly contact, so
        // without it a dive can pass through the water plane. It sits 500 m
        // BELOW sea level (a positive value would arm a floor above the field
        // and snap every gear-up aircraft up to it — the U-key "teleport").
        // Enforced by the FDM only while the gear is retracted, so landings
        // still settle.
        sim.model.set(
            "environment/floor-ft",
            -(crate::map::SEA_FLOOR_M as f64) / FT_TO_M,
        );
        sim.reset_ground();
        sim
    }

    /// Parked on the apron stand, at rest, nose west along the lead-in line.
    /// The M1 TAXI ramp spawn: taxi school starts here, with
    /// the yellow paint (and at night the blue edge lights) leading out to the
    /// runway. Engine state follows the lesson: cold for the taxi drill, warm
    /// for any other spawn.
    ///
    /// Like `reset_final` this keeps the aerodrome's own anchor (app origin 0)
    /// — the stand coordinates are scenery-frame positions, and re-anchoring
    /// would carry the runway along with us.
    pub fn reset_stand(&mut self) {
        self.model.set("ic/vc-kts", 0.0);
        self.model.set("ic/h-sl-ft", self.model.rest_h_ft());
        let spec = crate::airfield::spec_for(&self.name);
        let stand = spec.stand.expect("airfield has a stand");
        self.model.set("ic/psi-true-deg", stand.heading as f64);
        let north_m = stand.z as f64;
        let east_m = stand.x as f64;
        self.model
            .set("ic/lat-gc-deg", -(north_m / 0.3048) / fdm::FT_PER_DEG_LAT);
        self.model
            .set("ic/long-gc-deg", (east_m / 0.3048) / fdm::FT_PER_DEG_LAT);
        self.model.apply_ic();
        // Gear down on the ground — snapped, not commanded (see reset_ground).
        self.model.set("gear/gear-cmd-norm", 1.0);
        self.model.set("gear/gear-pos-norm", 1.0);
        self.light_engine();
        self.settle_gear();
        self.reset_hud_memory();
        self.origin_lat = 0.0;
        self.origin_lon = 0.0;
    }

    /// Parked on the runway at the threshold, at rest, pointing north.
    pub fn reset_ground(&mut self) {
        self.model.set("ic/vc-kts", 0.0);
        // Per-aircraft parked height: seeding the IC at 0 (or the wrong height)
        // buries the gear/skids and the ground springs fling the aircraft.
        self.model.set("ic/h-sl-ft", self.model.rest_h_ft());
        self.model.set("ic/psi-true-deg", 0.0);
        self.model.apply_ic();
        // Gear down on the ground — snapped, not commanded: teleporting onto the
        // runway with the gear mid-retraction would leave nothing to land on.
        self.model.set("gear/gear-cmd-norm", 1.0);
        self.model.set("gear/gear-pos-norm", 1.0);
        // Light the engine; without this the throttle commands produce no thrust.
        self.light_engine();
        self.settle_gear();
        self.reset_hud_memory();
        self.origin_lat = self.model.get("position/lat-gc-deg");
        self.origin_lon = self.model.get("position/long-gc-deg");
    }

    /// Lined up on the numbers: the aircraft's own home field, runway 36
    /// threshold, at rest, pointing north. The 737 M2 spawn — an explicit IC
    /// seat like `reset_stand`, so the aerodrome frame stays anchored and the
    /// nose sits on the piano keys (`threshold_z`), not wherever the previous
    /// flight's IC offsets happened to point.
    pub fn reset_threshold(&mut self) {
        self.model.set("ic/vc-kts", 0.0);
        self.model.set("ic/h-sl-ft", self.model.rest_h_ft());
        self.model.set("ic/psi-true-deg", 0.0);
        let spec = crate::airfield::spec_for(&self.name);
        let north_m = spec.threshold_z as f64;
        self.model
            .set("ic/lat-gc-deg", -(north_m / 0.3048) / fdm::FT_PER_DEG_LAT);
        self.model.set("ic/long-gc-deg", 0.0);
        self.model.apply_ic();
        self.model.set("gear/gear-cmd-norm", 1.0);
        self.model.set("gear/gear-pos-norm", 1.0);
        self.light_engine();
        self.settle_gear();
        self.reset_hud_memory();
        self.origin_lat = 0.0;
        self.origin_lon = 0.0;
    }

    /// March the FDM a few seconds with the throttle pinned at idle so the
    /// gear springs take the weight and the attitude stops ringing *before*
    /// the first rendered frame. Seeding the IC at the rest height with a
    /// level attitude means the airframe starts nose-level and sags onto the
    /// wheels over ~0.2 s, then sways on the (lightly damped) springs for
    /// seconds — the parked-but-visible camera sway. Idempotent for the
    /// rigid-gear backends, which settle instantly.
    fn settle_gear(&mut self) {
        let throttle = self.model.get("fcs/throttle-cmd-norm");
        self.model.set("fcs/throttle-cmd-norm", 0.0);
        for _ in 0..360 {
            self.model.step();
        }
        // Hand the lever back: the app writes its own 60% taxi setting right
        // after the reset returns.
        self.model.set("fcs/throttle-cmd-norm", throttle);
    }

    /// Start airborne, already trimmed for level cruise at `alt_ft` / `vc_kts`.
    /// Trim is a no-op on backends that can't solve it (JSBSim), which then just
    /// start airborne at the given condition.
    pub fn reset_air(&mut self, alt_ft: f64, vc_kts: f64) {
        self.model.set("ic/vc-kts", vc_kts);
        self.model.set("ic/h-sl-ft", alt_ft);
        self.model.set("ic/psi-true-deg", 0.0);
        self.model.apply_ic();
        // Cruise is a clean config: gear up. Seat it before the trim below, or
        // the solver seats the (much draggier) gear-down jet.
        self.model.set("gear/gear-cmd-norm", 0.0);
        self.model.set("gear/gear-pos-norm", 0.0);
        self.light_engine();
        self.model.trim(vc_kts, alt_ft);
        self.reset_hud_memory();
        self.origin_lat = self.model.get("position/lat-gc-deg");
        self.origin_lon = self.model.get("position/long-gc-deg");
    }

    /// Clear the per-flight HUD history (peak G, speed trend) after a reset.
    fn reset_hud_memory(&mut self) {
        self.max_g = 0.0;
        self.speed_trend = 0.0;
        self.last_cas_kts = self.model.get("velocities/vc-kts");
    }

    /// Airborne trimmed start at the lesson/flight-test altitude, kept for
    /// the reset tests as an airborne setup helper.
    #[cfg(test)]
    pub fn reset_cruise(&mut self) {
        self.reset_air(CRUISE_ALT_FT, CRUISE_VC_KT);
    }

    /// Re-trim for the current airspeed and altitude, keeping heading and the
    /// flat-earth position (so it doesn't teleport the aircraft).
    pub fn trim_now(&mut self) {
        let vc = self.model.get("velocities/vc-kts");
        let alt = self.model.get("position/h-sl-ft");
        self.model
            .set("ic/psi-true-deg", self.model.get("attitude/psi-deg"));
        self.model.trim(vc, alt);
    }

    /// Seat on a 3° final for runway 36, just inside the ILS gate: ~5 km south
    /// of the threshold at 1300 ft, gear down, flaps coming to approach, in a
    /// clean trim. The autopilot's APP mode takes it from there.
    ///
    /// Unlike every other reset this keeps the world origin anchored at the
    /// aerodrome (the scenery is static); re-anchoring here would carry the
    /// runway along with the aircraft.
    pub fn reset_final(&mut self) {
        // 5140 m south of the origin: z_world = 140 (threshold) + 5000.
        self.reset_final_at(5140.0, 1300.0, 155.0, 0.33);
    }

    /// The C172 landing lesson's seat: on a 3° final for runway 36, 2 NM out
    /// (~3.7 km) at 640 ft and 70 kt, flaps 20 — the numbers the landing
    /// briefing asks for (65-70 kt on final). Anchored to the aerodrome like
    /// `reset_final`, and well inside the ILS gate for the CDI needle.
    pub fn reset_final_c172(&mut self) {
        // 3704 m south of the origin: z_world = 140 (threshold) + 3704 (2 NM).
        self.reset_final_at(3704.0, 640.0, 70.0, 0.67);
    }

    /// Seat the aircraft on final at `gate_m` south of the threshold, `alt_ft`
    /// / `vc_kts`, with flaps `flap` and gear down. Keeps the world origin
    /// anchored at the aerodrome (the scenery is static), like `reset_final`.
    fn reset_final_at(&mut self, gate_m: f64, alt_ft: f64, vc_kts: f64, flap: f64) {
        let dlat = -(gate_m / 0.3048) / fdm::FT_PER_DEG_LAT;
        self.model.set("ic/lat-gc-deg", dlat);
        self.model.set("ic/long-gc-deg", 0.0);
        self.model.set("ic/vc-kts", vc_kts);
        self.model.set("ic/h-sl-ft", alt_ft);
        self.model.set("ic/psi-true-deg", 0.0);
        self.model.set("gear/gear-cmd-norm", 1.0);
        self.model.apply_ic();
        // IC resets forget pre-set commands, so re-command the approach
        // config after them. The flap *position* is seated directly — the
        // reset twin of the gear seat — so the trim below judges the flapped
        // wing and the surface is on top from the first frame instead of
        // slewing through the opening seconds of the approach.
        self.model.set("fcs/flap-cmd-norm", flap);
        self.model.set("fcs/flap-pos-norm", flap);
        self.model.set("gear/gear-pos-norm", 1.0);
        self.light_engine();
        self.model.trim(vc_kts, alt_ft);
        self.reset_hud_memory();
        self.origin_lat = 0.0;
        self.origin_lon = 0.0;
    }

    /// The 737 M3 seat: a mid-range final for ZSNM's ILS 36 — 9.2 km out ON
    /// the 3° beam (1,565 ft), 160 kt, flaps 15, gear UP: arriving already
    /// in approach configuration, one rung from landing flap. The seat's own
    /// speed IS the ladder's first target, so the gate (1,000 ft, ~45 s in)
    /// crosses at ~Vref+6 with the beam's own sink — both criteria with
    /// margin, which no faster seat allowed. ON the beam — the GS capture's
    /// angle gate shrinks with distance, so capturing from above at this
    /// range never fires; seated on it, LOC and GS take at once. Trimmed on
    /// the seat's own wing so it holds from the first frame.
    pub fn reset_final_jet(&mut self) {
        let dlat = -(9200.0 / 0.3048) / fdm::FT_PER_DEG_LAT;
        self.model.set("ic/lat-gc-deg", dlat);
        self.model.set("ic/long-gc-deg", 0.0);
        self.model.set("ic/vc-kts", 160.0);
        self.model.set("ic/h-sl-ft", 1565.0);
        self.model.set("ic/psi-true-deg", 0.0);
        self.model.set("gear/gear-cmd-norm", 0.0);
        self.model.apply_ic();
        self.model.set("fcs/flap-cmd-norm", 15.0 / 40.0);
        self.model.set("fcs/flap-pos-norm", 15.0 / 40.0);
        self.model.set("gear/gear-pos-norm", 0.0);
        self.light_engine();
        self.model.trim(160.0, 1565.0);
        self.reset_hud_memory();
        self.origin_lat = 0.0;
        self.origin_lon = 0.0;
    }

    /// Spawn the engine per the flight's engine plan: running (the tutorial
    /// flow's 60 %-throttle hands-on start) or cold and dark for the cold
    /// lessons — spark off, prop stopped, and the start flow is the pilot's.
    pub fn light_engine(&mut self) {
        if self.engine_cold {
            self.model.set("propulsion/ignition-enable", 0.0);
        } else {
            self.model.set("propulsion/set-running", -1.0);
        }
    }

    /// Set the cold-and-dark state of this flight. Only the piston C172
    /// carries the cold-start flow; other aircraft ignore the switch. The
    /// plan comes from the lesson (M1 starts cold), not a persisted setting.
    /// Turning COLD on while the engine is already running shuts it down now —
    /// the switch is an instruction to start flying from the checklist.
    pub fn set_engine_cold(&mut self, cold: bool) {
        let piston = crate::aircraft_profile::normalized(&self.name) == "c172p"
            && self.model.get("propulsion/engine[0]/has-fuel") > 0.5;
        self.engine_cold = cold && piston;
        if self.engine_cold {
            self.model.set("propulsion/engine[0]/set-stopped", 1.0);
        }
    }

    /// World-space position (meters) and orientation for the aircraft mesh.
    pub fn pose(&self) -> (Vec3, Quat, f32) {
        let lat = self.model.get("position/lat-gc-deg");
        let lon = self.model.get("position/long-gc-deg");
        let alt_m = self.model.get("position/h-sl-ft") * FT_TO_M;
        let north = (lat - self.origin_lat).to_radians() * EARTH_RADIUS_M;
        let east = (lon - self.origin_lon).to_radians()
            * self.origin_lat.to_radians().cos()
            * EARTH_RADIUS_M;
        let pos = Vec3::new(east as f32, alt_m as f32, -(north as f32));

        let phi = self.model.get("attitude/phi-rad") as f32;
        let theta = self.model.get("attitude/theta-rad") as f32;
        let psi = self.model.get("attitude/psi-rad") as f32;

        let r_nb =
            Mat3::from_rotation_z(psi) * Mat3::from_rotation_y(theta) * Mat3::from_rotation_x(phi);
        let s = axis_map();
        let rot = Quat::from_mat3(&(s * r_nb * s.transpose()));
        (pos, rot, psi)
    }
}

/// Latest flight state, published for the view layer each frame.
#[derive(Resource, Default)]
pub struct AircraftState {
    pub pos: Vec3,
    pub alt_ft: f64,
    pub cas_kts: f64,
    /// Vertical speed (`velocities/h-dot-fps` × 60), ft/min.
    pub vs_fpm: f64,
    /// Angle of attack (`aero/alpha-rad`), degrees — drives the F-22's wingtip
    /// vapor trails (`vapor.rs`), which feed with the wing's load.
    ///
    /// The FDM holds the reading at an honest zero below a walking pace, where
    /// the raw `atan2` is only drift noise (see `fdm::air_angles`); the chase
    /// camera's airflow gate (`buffet_weight`) is a second line of defence
    /// around the same stair.
    pub alpha_deg: f64,
    /// Mach number (`velocities/mach`) — drives the transonic vapor cone.
    pub mach: f64,
    pub heading_deg: f64,
    pub pitch_deg: f64,
    pub roll_deg: f64,
    /// Rate of heading change (`psi-dot`), deg/s — drives the turn coordinator.
    pub turn_rate_dps: f64,
    /// Lateral load factor at the pilot (`accelerations/n-pilot-y-norm`), 0 when
    /// coordinated — drives the turn coordinator's slip ball.
    pub slip: f64,
    /// Roll rate (`velocities/p-rad_sec`), deg/s, +right — biases the F-22's
    /// leading-edge vapor toward the wing rolling down.
    pub roll_rate_dps: f64,
    /// Yaw rate (`velocities/r-rad_sec`), deg/s, +right — feeds the F-22's
    /// forebody vapor in a yaw, where the flow off the chine is strongest.
    pub yaw_rate_dps: f64,
    pub throttle: f64,
    /// Normalised thrust per engine (`fcs/thrust-norm`), which the FDM scales by
    /// its altitude/Mach thrust tables: ~1.4 at full reheat on the deck, ~0.7 at
    /// 30,000 ft. The F-22's exhaust plume keys its size off this.
    pub thrust_norm: f64,
    pub elevator: f64,
    /// Actual elevator surface angle (`fcs/elevator-pos-rad`), radians. Unlike
    /// `elevator`, this includes the pitch-trim offset, so the 3D surface can
    /// stay deflected after the yoke is centred.
    pub elevator_pos_rad: f64,
    pub aileron: f64,
    pub rudder: f64,
    pub elevator_trim: f64,
    pub flap_deg: f64,
    /// Left/right toe-brake commands (`fcs/{left,right}-brake-cmd-norm`).
    pub brake_left: f64,
    pub brake_right: f64,
    pub rpm: f64,
    /// Flight-spoiler position (`fcs/speedbrake-pos-norm`), 0..1. Zeros for
    /// aircraft without the boards (737 only for now).
    pub speedbrake_pos: f64,
    /// Thrust-reverse lever position (`fcs/reverse-pos-norm`), 0..1. The FDM
    /// only extends it while the wheels carry weight.
    pub reverse_pos: f64,
    /// Whether the loaded aircraft has an autopilot at all (737 only).
    pub autopilot_available: bool,
    /// Thrust vectoring switched on (`fcs/tvc-enable`).
    pub tvc_on: bool,
    /// Whether the loaded model supports TVC at all (F-22 only).
    pub tvc_available: bool,
    /// Achieved landing-gear position (`gear/gear-pos-norm`): 1 = down, 0 = up.
    pub gear_pos: f64,
    /// Weight-on-wheels (`gear/wow` > 0.5) — drives the PFD's Vr bug and
    /// ROTATE callout.
    pub on_ground: bool,
    /// Whether the loaded model's gear retracts at all (F-22 only).
    pub gear_available: bool,
    /// Main-rotor rpm (`propulsion/engine[0]/rotor-rpm`); drives the rotor spin.
    pub main_rotor_rpm: f64,
    /// Tail-rotor rpm (`propulsion/engine[1]/rotor-rpm`).
    pub tail_rotor_rpm: f64,
    /// Collective lever position (`fcs/collective-cmd-norm`), 0..1.
    pub collective: f64,
    /// Main-rotor reaction torque (`propulsion/engine[0]/torque-lbsft`).
    pub torque_lbsft: f64,
    /// Whether the loaded model is a rotorcraft (AH-64D / AH-1S).
    pub rotor_available: bool,
    /// Whether arcade dynamics are active (`fcs/arcade-enable`).
    pub arcade: bool,
    /// Helicopter auto-hover state: 0 off, 1 takeoff, 2 hold, 3 land.
    pub heli_hover_state: f64,
    /// Helicopter auto-balance switch (`fcs/heli-balance-enable`).
    pub heli_balance: bool,

    // --- F-22 HUD quantities ---
    /// True airspeed (`velocities/vtrue-kts`).
    pub tas_kts: f64,
    /// Ground speed (`velocities/vg-fps`, converted to knots).
    pub gs_kts: f64,
    /// Ground track (`atan2(v-east, v-north)`), deg true. With the flight-path
    /// angle, places the HUD's flight path marker — wind drift included.
    pub track_deg: f64,
    /// Flight path angle (`asin(h-dot / ground-speed)`), deg, + climbing.
    pub flight_path_deg: f64,
    /// Load factor (`forces/load-factor`), 1.0 in level flight.
    pub g_norm: f64,
    /// Peak load factor reached since the last reset.
    pub max_g_norm: f64,
    /// Smoothed rate of change of KCAS, kt/s.
    pub speed_trend: f64,
    /// Height above the ground (`position/h-agl-ft`).
    pub radar_alt_ft: f64,
    /// Angle of sideslip (`aero/beta-deg`). Also held at zero near a
    /// standstill, same gate as the alpha reading.
    pub beta_deg: f64,
    /// Wind velocity components (`atmosphere/wind-{north,east}-fps`), fps.
    pub wind_north_fps: f64,
    pub wind_east_fps: f64,

    // --- fuel & cold start (c172p, Rust backend with a FuelSpec) ---
    /// Wing-tank contents per tank (lb). Zeros on backends without a fuel model.
    pub fuel_left_lb: f64,
    pub fuel_right_lb: f64,
    /// Total usable capacity (lb), for gauge scaling.
    pub fuel_capacity_lb: f64,
    /// Spark available (`propulsion/ignition-enable`): the master/mags state.
    pub master_on: bool,
    /// Engine making power (`propulsion/engine[0]/running`).
    pub engine_running: bool,
}

/// The two most recent *fixed-step* poses. The FDM advances at 120 Hz while the
/// renderer runs uncapped (~160 fps here), and 160 isn't a multiple of 120, so
/// sampling the FDM directly each render frame makes the aircraft hold for a
/// frame then jump a whole step — visible as jitter. Keeping prev+curr lets the
/// view interpolate by `Time<Fixed>::overstep_fraction()`.
#[derive(Resource, Default)]
pub struct RenderPose {
    pub prev_pos: Vec3,
    pub prev_rot: Quat,
    pub prev_heading: f32,
    pub curr_pos: Vec3,
    pub curr_rot: Quat,
    pub curr_heading: f32,
    pub init: bool,
}

pub struct SimPlugin;

impl Plugin for SimPlugin {
    fn build(&self, app: &mut App) {
        let sim = Sim::new(&startup_aircraft());
        app.insert_resource(sim)
            .init_resource::<AircraftState>()
            .init_resource::<RenderPose>()
            // MYFS beams, recomputed inside `advance_fdm` each step.
            .init_resource::<crate::ils::Ils>()
            .add_systems(
                FixedUpdate,
                (advance_fdm, capture_render_pose).chain().run_if(unpaused),
            )
            .add_systems(Update, publish_state.run_if(unpaused));
    }
}

fn advance_fdm(
    mut sim: ResMut<Sim>,
    bindings: Res<crate::input::Bindings>,
    mcp: Res<Mcp>,
    mut ap: ResMut<Autopilot>,
    mut ils_res: ResMut<crate::ils::Ils>,
    airfield: Res<crate::airfield::Airfield>,
    map: Res<crate::map::ActiveMap>,
    weather: Res<crate::weather::Weather>,
) {
    // Ambient wind from the weather settings, pushed through the same property
    // JSBSim exposes. The Rust model treats it as the air-mass velocity; on the
    // JSBSim backend this is a native set.
    let (wind_n, wind_e) = weather.wind_ned_fps();
    sim.model.set("atmosphere/wind-north-fps", wind_n);
    sim.model.set("atmosphere/wind-east-fps", wind_e);
    // Pilot bindings drive every property the loaded model exposes. This runs
    // unconditionally so flaps and brakes stay live even under the autopilot.
    // `effective()` folds in the trim offset (a rotorcraft's solved cyclic).
    for b in &bindings.list {
        sim.model.set(b.property, b.effective());
    }

    // Parking brake: a stopped aircraft with cold throttle and untouched
    // brakes is held in place (brakes full), so a windy spawn does not creep
    // down the runway. Any throttle or brake input releases it for the step.
    let braked = sim.model.get("fcs/left-brake-cmd-norm").abs()
        + sim.model.get("fcs/right-brake-cmd-norm").abs();
    if sim.model.get("gear/wow") > 0.5
        && sim.model.get("velocities/vg-fps") < 2.0
        && sim.model.get("fcs/throttle-cmd-norm") < 0.02
        && braked < 1e-3
    {
        sim.model.set("fcs/left-brake-cmd-norm", 1.0);
        sim.model.set("fcs/right-brake-cmd-norm", 1.0);
    }

    // The autothrottle and the engaged modes write their own channels after
    // the bindings, so the automation wins the step.
    let default_spec = crate::airfield::AirfieldSpec::myfs();
    let spec = airfield.0.as_ref().unwrap_or(&default_spec);
    let ils = crate::ils::compute(&sim, spec);
    *ils_res = ils;
    crate::autopilot::advance(&mut *sim.model, &mcp, &mut ap, DT, &ils);

    // Mission assist last: it may add nose-up or bank after the pilot's
    // commands, so terrain and the arena edge always win the step.
    if let Some(map) = map.0 {
        crate::map::assist(&mut sim, map);
    }

    sim.model.step();
}

/// Shift the fixed-step pose buffer forward each FDM step (runs right after
/// `advance_fdm`), so the view always has prev+curr to interpolate between.
fn capture_render_pose(sim: Res<Sim>, mut rp: ResMut<RenderPose>) {
    let (pos, rot, heading) = sim.pose();
    if rp.init {
        rp.prev_pos = rp.curr_pos;
        rp.prev_rot = rp.curr_rot;
        rp.prev_heading = rp.curr_heading;
    } else {
        rp.prev_pos = pos;
        rp.prev_rot = rot;
        rp.prev_heading = heading;
        rp.init = true;
    }
    rp.curr_pos = pos;
    rp.curr_rot = rot;
    rp.curr_heading = heading;
}

/// Publishes the FDM readouts onto [`AircraftState`] every unpaused frame.
/// `pub(crate)`: the lesson-flow watcher orders against this — during a
/// frozen popup nothing writes `AircraftState`, so the last write lingers at
/// the failure spot and a stale read would re-arm that same FAIL (see
/// `lesson_flow::watch`).
pub(crate) fn publish_state(
    mut sim: ResMut<Sim>,
    time: Res<Time>,
    mut state: ResMut<AircraftState>,
) {
    let (pos, _, _) = sim.pose();
    // Split the borrow: the FDM is read through `m` while the HUD history
    // fields are written below, without either aliasing the other.
    let Sim {
        model,
        name,
        max_g,
        speed_trend,
        last_cas_kts,
        ..
    } = &mut *sim;
    let m = &**model;
    // Rate of heading change, from the body rates and attitude:
    // psi-dot = (q sinφ + r cosφ) / cosθ (rad/s → deg/s). This is the turn
    // coordinator's rate-of-turn input, and it's exact kinematic heading rate.
    let (phi, theta) = (m.get("attitude/phi-rad"), m.get("attitude/theta-rad"));
    let turn_rate_dps = ((m.get("velocities/q-rad_sec") * phi.sin()
        + m.get("velocities/r-rad_sec") * phi.cos())
        / theta.cos().max(1e-3))
    .to_degrees();

    // Ground-referenced velocity, for the HUD's flight path marker and wind.
    let gs_fps = m.get("velocities/vg-fps");
    let hdot_fps = m.get("velocities/h-dot-fps");
    let vn_fps = m.get("velocities/v-north-fps");
    let ve_fps = m.get("velocities/v-east-fps");
    let track_deg = ve_fps.atan2(vn_fps).to_degrees().rem_euclid(360.0);
    let flight_path_deg = if gs_fps > 1.0 {
        (hdot_fps / gs_fps).clamp(-1.0, 1.0).asin().to_degrees()
    } else {
        0.0
    };
    let cas_kts = m.get("velocities/vc-kts");
    let g_norm = m.get("forces/load-factor");

    // HUD history: peak G for the flight, and a lightly smoothed speed trend
    // (a raw per-frame derivative is far too noisy to read).
    let dt = time.delta_secs_f64().clamp(1e-4, 0.25);
    if g_norm > *max_g {
        *max_g = g_norm;
    }
    let trend_raw = (cas_kts - *last_cas_kts) / dt;
    let blend = 1.0 - (-dt / 0.35).exp();
    *speed_trend += (trend_raw - *speed_trend) * blend;
    *last_cas_kts = cas_kts;
    let max_g_norm = *max_g;
    let speed_trend = (*speed_trend).clamp(-99.0, 99.0);

    *state = AircraftState {
        pos,
        alt_ft: m.get("position/h-sl-ft"),
        cas_kts,
        vs_fpm: hdot_fps * 60.0,
        alpha_deg: m.get("aero/alpha-rad").to_degrees(),
        mach: m.get("velocities/mach"),
        heading_deg: m.get("attitude/psi-deg"),
        pitch_deg: m.get("attitude/theta-deg"),
        roll_deg: m.get("attitude/phi-deg"),
        turn_rate_dps,
        slip: m.get("accelerations/n-pilot-y-norm"),
        roll_rate_dps: m.get("velocities/p-rad_sec").to_degrees(),
        yaw_rate_dps: m.get("velocities/r-rad_sec").to_degrees(),
        // Report what is actually driving the surfaces, auto or manual.
        throttle: m.get("fcs/throttle-cmd-norm"),
        thrust_norm: m.get("fcs/thrust-norm"),
        elevator: m.get("fcs/elevator-cmd-norm"),
        elevator_pos_rad: m.get("fcs/elevator-pos-rad"),
        aileron: m.get("fcs/aileron-cmd-norm"),
        rudder: m.get("fcs/rudder-cmd-norm"),
        elevator_trim: m.get("fcs/pitch-trim-cmd-norm"),
        flap_deg: m.get("fcs/flap-pos-deg"),
        brake_left: m.get("fcs/left-brake-cmd-norm"),
        brake_right: m.get("fcs/right-brake-cmd-norm"),
        rpm: m.get("propulsion/engine[0]/engine-rpm"),
        speedbrake_pos: m.get("fcs/speedbrake-pos-norm"),
        reverse_pos: m.get("fcs/reverse-pos-norm"),
        autopilot_available: crate::aircraft_profile::profile(name).has_autopilot,
        tvc_on: m.get("fcs/tvc-enable") > 0.5,
        tvc_available: m.has_tvc(),
        gear_pos: m.get("gear/gear-pos-norm"),
        on_ground: m.get("gear/wow") > 0.5,
        gear_available: m.has_retractable_gear(),
        main_rotor_rpm: m.get("propulsion/engine[0]/rotor-rpm"),
        tail_rotor_rpm: m.get("propulsion/engine[1]/rotor-rpm"),
        collective: m.get("fcs/collective-cmd-norm"),
        torque_lbsft: m.get("propulsion/engine[0]/torque-lbsft"),
        rotor_available: m.has_rotor(),
        arcade: m.get("fcs/arcade-enable") > 0.5,
        heli_hover_state: m.get("fcs/heli-hover-state"),
        heli_balance: m.get("fcs/heli-balance-enable") > 0.5,
        tas_kts: m.get("velocities/vtrue-kts"),
        gs_kts: gs_fps * FPS_TO_KT,
        track_deg,
        flight_path_deg,
        g_norm,
        max_g_norm,
        speed_trend,
        radar_alt_ft: m.get("position/h-agl-ft"),
        beta_deg: m.get("aero/beta-deg"),
        wind_north_fps: m.get("atmosphere/wind-north-fps"),
        wind_east_fps: m.get("atmosphere/wind-east-fps"),
        fuel_left_lb: m.get("propulsion/tank-0-contents-lbs"),
        fuel_right_lb: m.get("propulsion/tank-1-contents-lbs"),
        fuel_capacity_lb: m.get("propulsion/tank-capacity-lbs"),
        master_on: m.get("propulsion/ignition-enable") > 0.5,
        engine_running: m.get("propulsion/engine[0]/running") > 0.5,
    };
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The pure-Rust backend is the one that can trim; every entry point here
    /// reaches the solver through the `Box<dyn FlightModel>` trait object.
    #[test]
    fn a_cold_parked_c172_does_not_rock_or_spin_down_visibly() {
        // The menu's Sim::new lights the engine warm; switching ENGINE = COLD
        // at PLAY used to leave it winding down from idle rpm — a spinning
        // prop rocking the parked airframe on its gear (visible camera sway).
        // The cold spawn must enter the scene already stopped, rpm at zero.
        let mut sim = rust_sim();
        sim.set_engine_cold(true);
        sim.reset_ground();
        assert_eq!(sim.model.get("propulsion/engine[0]/engine-rpm"), 0.0);
        // The gear is already settled: seated pitch from frame zero, and no
        // attitude ringing afterwards.
        let theta0 = sim.model.get("attitude/theta-deg");
        assert!(
            theta0 > 2.0,
            "parked C172 should sit nose-up on its gear: {theta0}"
        );
        for _ in 0..120 * 5 {
            sim.model.step();
        }
        assert_eq!(sim.model.get("propulsion/engine[0]/engine-rpm"), 0.0);
        assert!(
            (sim.model.get("attitude/theta-deg") - theta0).abs() < 0.05,
            "parked airframe still ringing: {theta0} -> {}",
            sim.model.get("attitude/theta-deg")
        );
    }

    fn rust_sim() -> Sim {
        std::env::set_var("FDM_BACKEND", "rust");
        Sim::new("c172p")
    }

    /// Retractable-gear variant (the F-22) for the reset tests.
    fn rust_f22() -> Sim {
        std::env::set_var("FDM_BACKEND", "rust");
        Sim::new("f22")
    }

    #[test]
    fn reset_final_seats_inside_the_ils_gate() {
        std::env::set_var("FDM_BACKEND", "rust");
        let mut sim = Sim::new("737");
        sim.reset_final();
        let (pos, _, _) = sim.pose();
        let spec = crate::airfield::AirfieldSpec::zsnm();
        let ils = crate::ils::compute(&sim, &spec);
        assert!(
            ils.available,
            "should be inside the approach gate {ils:?} pos={pos:?}"
        );
        assert!(ils.loc_dev_deg.abs() < 1.0, "{:?}", ils.loc_dev_deg);
        // 1300 ft against a beam 5 km out (~850 ft): spawned above the beam.
        assert!(ils.gs_dev_deg > 0.6, "{:?}", ils.gs_dev_deg);
        assert!(
            ils.dist_m > 4900.0 && ils.dist_m < 5400.0,
            "{:?}",
            ils.dist_m
        );
        // And the runway reset is *behind* both antennas, not on scale.
        sim.reset_ground();
        assert!(!crate::ils::compute(&sim, &spec).available);
    }

    #[test]
    fn reset_final_c172_seats_inside_the_ils_gate() {
        std::env::set_var("FDM_BACKEND", "rust");
        let mut sim = Sim::new("c172p");
        sim.reset_final_c172();
        let spec = crate::airfield::AirfieldSpec::myfs();
        let ils = crate::ils::compute(&sim, &spec);
        assert!(ils.available, "should be inside the approach gate {ils:?}");
        assert!(ils.loc_dev_deg.abs() < 1.0, "{:?}", ils.loc_dev_deg);
        // 640 ft against a beam 2 NM out (~640 ft): riding the beam.
        assert!(ils.gs_dev_deg.abs() < 0.3, "{:?}", ils.gs_dev_deg);
        assert!(
            ils.dist_m > 3500.0 && ils.dist_m < 3900.0,
            "{:?}",
            ils.dist_m
        );
        // The lesson's approach speed and config, both seated by the trim
        // solver: flaps 20 lands on the wing (the seat writes the flap
        // position directly after IC — see `reset_final_at`), and the trim
        // point was solved with the flapped wing.
        let flap_deg = sim.model.get("fcs/flap-pos-deg");
        assert!((flap_deg - 20.0).abs() < 0.5, "flaps seated: {flap_deg}");
        assert!(
            (sim.model.get("velocities/vc-kts") - 70.0).abs() < 6.0,
            "final speed {}",
            sim.model.get("velocities/vc-kts")
        );
        // Hands-off, the seat is a live trim point: one second of the
        // app's per-step binding writes holds speed and flap position.
        let mut bindings = crate::input::Bindings::for_aircraft("c172p");
        if let Some(b) = bindings
            .list
            .iter_mut()
            .find(|b| b.property == "fcs/flap-cmd-norm")
        {
            b.value = 0.67;
        }
        for _ in 0..120 {
            for b in &bindings.list {
                sim.model.set(b.property, b.effective());
            }
            sim.model.step();
        }
        assert!(
            (sim.model.get("fcs/flap-pos-deg") - 20.0).abs() < 0.5,
            "flap seat holds: {}",
            sim.model.get("fcs/flap-pos-deg")
        );
        assert!(
            (sim.model.get("velocities/vc-kts") - 70.0).abs() < 3.0,
            "hands-off drift: {}",
            sim.model.get("velocities/vc-kts")
        );
        // And a cold lesson at the stand keeps the world anchored: the runway
        // is scenery, so the origin must not re-seat.
        sim.reset_stand();
        assert_eq!(sim.origin_lat, 0.0);
        assert_eq!(sim.origin_lon, 0.0);
    }

    #[test]
    fn reset_cruise_starts_trimmed_at_the_cruise_point() {
        let mut sim = rust_sim();
        sim.reset_cruise();
        assert!((sim.model.get("position/h-sl-ft") - CRUISE_ALT_FT).abs() < 1.0);
        assert!((sim.model.get("velocities/vc-kts") - CRUISE_VC_KT).abs() < 2.0);
        assert!(sim.model.get("attitude/phi-deg").abs() < 0.5);
        assert!(sim.model.get("velocities/q-rad_sec").abs() < 1e-6);
    }

    #[test]
    fn trim_now_seats_the_current_condition() {
        let mut sim = rust_sim();
        sim.reset_cruise();
        // Fly off-trim for a second (a touch of nose-up elevator), then re-trim.
        sim.model.set("fcs/elevator-cmd-norm", -0.2);
        for _ in 0..120 {
            sim.model.step();
        }
        let (vc, alt) = (
            sim.model.get("velocities/vc-kts"),
            sim.model.get("position/h-sl-ft"),
        );
        sim.trim_now();
        assert!((sim.model.get("velocities/vc-kts") - vc).abs() < 2.0);
        assert!((sim.model.get("position/h-sl-ft") - alt).abs() < 1.0);
        assert!(sim.model.get("velocities/q-rad_sec").abs() < 1e-6);
    }

    #[test]
    fn reset_air_retracts_the_gear_before_trimming() {
        let mut sim = rust_f22();
        sim.reset_ground();
        assert!(
            sim.model.get("gear/gear-pos-norm") > 0.99,
            "parked gear down"
        );
        sim.reset_air(15_000.0, 350.0);
        assert!(sim.model.get("gear/gear-pos-norm") < 0.01, "cruise gear up");
        // Trim ran on the gear-up jet, so the seated point must hold. Seating a
        // gear-down trim (much more drag) then flying gear-up would sag away.
        let vc0 = sim.model.get("velocities/vc-kts");
        for _ in 0..240 {
            sim.model.step();
        }
        let vc1 = sim.model.get("velocities/vc-kts");
        assert!(
            (vc1 - vc0).abs() < 4.0,
            "gear-up trim must hold: {vc0} -> {vc1}"
        );
    }

    #[test]
    fn reset_ground_from_a_gear_up_cruise_does_not_fall_through() {
        let mut sim = rust_f22();
        sim.reset_cruise(); // aircraft airborne, gear up
        assert!(sim.model.get("gear/gear-pos-norm") < 0.01);
        sim.reset_ground();
        for _ in 0..240 {
            sim.model.step();
        }
        assert!(
            sim.model.get("position/h-sl-ft") > 0.0,
            "must rest on the runway, not sink through it"
        );
        assert!(sim.model.get("gear/wow") > 0.5, "back on the wheels");
    }
}

#[cfg(test)]
mod m3_probe {
    use super::*;
    use crate::aircraft_profile::ToggleAction;
    use crate::airfield::AirfieldSpec;
    use crate::autopilot::{Autopilot, Mcp, advance};
    use crate::ils;

    /// One M3 recipe run, headless: seat, G-5-3-6 (or a subset), a scripted
    /// config ladder, and a log line every 10 s plus the 1,000 ft crossing.
    fn run(tag: &str, at: bool, window: bool) {
        std::env::set_var("FDM_BACKEND", "rust");
        let mut sim = Sim::new("737");
        sim.reset_final_jet();
        // A breath before the keys, as a human takes: the first steps recompute
        // the stale weight-on-wheels flag the ground spawn left behind.
        for _ in 0..120 {
            sim.model.set("fcs/throttle-cmd-norm", 0.25);
            sim.model.step();
        }
        let spec = AirfieldSpec::zsnm();
        let mut ap = Autopilot::default();
        let mut mcp = Mcp::default();
        mcp.ias_kts = 180.0;
        mcp.vs_fpm = -700.0;
        ap.toggle(&*sim.model);
        if at {
            ap.press_mode(ToggleAction::AutoThrottle, &*sim.model);
        }
        ap.press_mode(ToggleAction::VsMode, &*sim.model);
        ap.press_mode(ToggleAction::Approach, &*sim.model);
        eprintln!(
            "[{tag}] setup: engaged={} vert={:?} at={:?} armed={} wow={} alt={:.0}",
            ap.engaged,
            ap.vert,
            ap.at,
            ap.app_armed,
            sim.model.get("gear/wow"),
            sim.model.get("position/h-sl-ft"),
        );
        let dt = 1.0 / 120.0;
        let mut t = 0.0f64;
        let mut last_log = -1.0;
        for i in 0..120 * 240 {
            sim.model.set("fcs/throttle-cmd-norm", 0.25);
            let beams = ils::compute(&sim, &spec);
            advance(&mut *sim.model, &mcp, &mut ap, dt, &beams);
            sim.model.step();
            t += dt;
            if t > 2.0 {
                sim.model.set("fcs/flap-cmd-norm", 15.0 / 40.0);
            }
            if t > 12.0 {
                sim.model.set("gear/gear-cmd-norm", 1.0);
            }
            if t > 25.0 {
                sim.model.set("fcs/flap-cmd-norm", 0.75);
            }
            // The SOP variant dials the window down with the ladder.
            if window {
                mcp.ias_kts = if t < 25.0 { 160.0 } else { 131.0 };
            }
            let alt = sim.model.get("position/h-sl-ft");
            if t - last_log >= 10.0 {
                last_log = t;
                eprintln!(
                    "[{tag}] t={t:5.0} alt={alt:6.0} cas={:6.1} vs={:6.0} pitch={:5.1} flap={:4.0} gear={:3.0} vert={:?} gs={:+.2}",
                    sim.model.get("velocities/vc-kts"),
                    sim.model.get("velocities/h-dot-fps") * 60.0,
                    sim.model.get("attitude/theta-deg"),
                    sim.model.get("fcs/flap-pos-deg"),
                    sim.model.get("gear/gear-pos-norm") * 30.0,
                    ap.vert,
                    beams.gs_dev_deg,
                );
            }
            if alt < 1000.0 && t > 5.0 {
                eprintln!(
                    "[{tag}] GATE t={t:5.0} alt={alt:6.0} cas={:6.1} vs={:6.0} flap={:4.0} gear={:3.0} vert={:?}",
                    sim.model.get("velocities/vc-kts"),
                    sim.model.get("velocities/h-dot-fps") * 60.0,
                    sim.model.get("fcs/flap-pos-deg"),
                    sim.model.get("gear/gear-pos-norm") * 30.0,
                    ap.vert,
                );
                return;
            }
        }
        eprintln!("[{tag}] never reached the gate");
    }

    #[test]
    fn the_m3_recipe_passes_the_gate() {
        // The coached flow, headless: G-5-3-6, the ladder-paced config, and
        // the A/T window dialled down with the ladder (the [ key's work).
        // The 1,000 ft gate's four criteria must all be green.
        std::env::set_var("FDM_BACKEND", "rust");
        let mut sim = Sim::new("737");
        sim.reset_final_jet();
        for _ in 0..120 {
            sim.model.set("fcs/throttle-cmd-norm", 0.25);
            sim.model.step();
        }
        let spec = AirfieldSpec::zsnm();
        let mut ap = Autopilot::default();
        let mut mcp = Mcp::default();
        mcp.ias_kts = 160.0;
        mcp.vs_fpm = -950.0;
        ap.toggle(&mut *sim.model);
        ap.press_mode(ToggleAction::AutoThrottle, &mut *sim.model);
        ap.press_mode(ToggleAction::VsMode, &mut *sim.model);
        ap.press_mode(ToggleAction::Approach, &mut *sim.model);
        assert!(ap.engaged, "the seat must not refuse the A/P");
        let dt = 1.0 / 120.0;
        let mut t = 0.0f64;
        // The lesson's gate reads a two-second trend of the sink, not one
        // frame of the approach's phugoid: mirror the meter's filter here.
        let mut vs_trend = 0.0f64;
        for i in 0..120 * 240 {
            sim.model.set("fcs/throttle-cmd-norm", 0.25);
            let beams = ils::compute(&sim, &spec);
            advance(&mut *sim.model, &mcp, &mut ap, dt, &beams);
            sim.model.step();
            t += dt;
            let vs_raw = sim.model.get("velocities/h-dot-fps") * 60.0;
            vs_trend += (vs_raw - vs_trend) * (dt / 2.0);
            if t > 3.0 {
                sim.model.set("gear/gear-cmd-norm", 1.0);
            }
            if t > 8.0 {
                sim.model.set("fcs/flap-cmd-norm", 0.75);
                mcp.ias_kts = 131.0;
            }
            let alt = sim.model.get("position/h-sl-ft");
            if (5.0..90.0).contains(&t) && alt < 1000.0 {
                let cas = sim.model.get("velocities/vc-kts");
                let flap = sim.model.get("fcs/flap-pos-deg");
                let gear = sim.model.get("gear/gear-pos-norm");
                let vs = vs_trend;
                eprintln!("gate t={t:.0} cas={cas:.0} vs={vs:.0} flap={flap:.0} gear={gear:.0}");
                assert!(gear > 0.95, "gear down at the gate: {gear}");
                assert!(flap >= 14.0, "flaps 15 at the gate: {flap}");
                assert!(
                    cas <= crate::landing::vref(0.75) + 25.0,
                    "speed at the gate: {cas}"
                );
                // The lesson re-uses approach::criteria's shared budget.
                assert!(vs.abs() <= 1_000.0, "sink at the gate: {vs}");
                return;
            }
        }
        panic!("the recipe never reached the gate");
    }
}

#[cfg(test)]
mod trim_probe {
    use super::*;

    #[test]
    fn probe_seat_trims() {
        std::env::set_var("FDM_BACKEND", "rust");
        for (flap, gear, tag) in [
            (5.0 / 40.0, 0.0, "flaps5 gear-up"),
            (15.0 / 40.0, 0.0, "flaps15 gear-up"),
            (5.0 / 40.0, 1.0, "flaps5 gear-dn"),
            (0.33, 1.0, "reset_final cfg"),
        ] {
            let mut sim = Sim::new("737");
            sim.reset_final_jet();
            sim.model.set("fcs/flap-cmd-norm", flap);
            sim.model.set("fcs/flap-pos-norm", flap);
            sim.model.set("gear/gear-cmd-norm", gear);
            sim.model.set("gear/gear-pos-norm", gear);
            sim.model.trim(180.0, 1550.0);
            eprintln!(
                "[{tag}] trim={:+.3} elev={:+.3} theta={:+.2}",
                sim.model.get("fcs/pitch-trim-cmd-norm"),
                sim.model.get("fcs/elevator-cmd-norm"),
                sim.model.get("attitude/theta-deg"),
            );
        }
    }
}
