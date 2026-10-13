//! Bridges the JSBSim 6-DoF model into the Bevy world: drives the FDM at a
//! fixed rate and converts its geodetic state into Bevy-space position and
//! orientation.

use bevy::prelude::*;
use jsbsim_sys::{autopilot, default_jsbsim_src, Fdm};

/// FDM step. Matches the rate JSBSim was validated against.
pub const DT: f64 = 1.0 / 120.0;

/// Height of the CG above the runway when parked (from c172p/reset00.xml).
/// Seeding the IC at 0 buries the gear and the ground springs fling the aircraft.
const REST_H_FT: f64 = 4.7;

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
    pub fdm: Fdm,
    pub model: String,
    root: String,
    origin_lat: f64,
    origin_lon: f64,
}

impl Sim {
    pub fn new(model: &str) -> Self {
        let root = default_jsbsim_src();
        let mut fdm = Fdm::new();
        fdm.set_root(&root);
        fdm.set_aircraft_path("aircraft");
        fdm.set_engine_path("engine");
        fdm.set_systems_path("systems");
        fdm.set_dt(DT);
        if !fdm.load_model(model) {
            panic!("failed to load JSBSim model '{model}' from {root}");
        }
        let mut sim = Sim {
            fdm,
            model: model.to_string(),
            root,
            origin_lat: 0.0,
            origin_lon: 0.0,
        };
        sim.reset_ground();
        sim
    }

    /// Parked on the runway at the threshold, at rest, pointing north.
    pub fn reset_ground(&mut self) {
        self.fdm.ic_speed_kts(0.0);
        self.fdm.ic_altitude_ft(REST_H_FT);
        self.fdm.ic_heading_deg(0.0);
        self.apply_ic();
    }

    fn apply_ic(&mut self) {
        if !self.fdm.run_ic() {
            panic!("RunIC failed for model '{}' in {}", self.model, self.root);
        }
        // Light the engine; without this the throttle commands produce no thrust.
        self.fdm.set_property("propulsion/set-running", -1.0);
        self.origin_lat = self.fdm.get_property("position/lat-gc-deg");
        self.origin_lon = self.fdm.get_property("position/long-gc-deg");
    }

    /// World-space position (meters) and orientation for the aircraft mesh.
    pub(crate) fn pose(&self) -> (Vec3, Quat, f32) {
        let lat = self.fdm.get_property("position/lat-gc-deg");
        let lon = self.fdm.get_property("position/long-gc-deg");
        let alt_m = self.fdm.get_property("position/h-sl-ft") * FT_TO_M;
        let north = (lat - self.origin_lat).to_radians() * EARTH_RADIUS_M;
        let east = (lon - self.origin_lon).to_radians()
            * self.origin_lat.to_radians().cos()
            * EARTH_RADIUS_M;
        let pos = Vec3::new(east as f32, alt_m as f32, -(north as f32));

        let phi = self.fdm.get_property("attitude/phi-rad") as f32;
        let theta = self.fdm.get_property("attitude/theta-rad") as f32;
        let psi = self.fdm.get_property("attitude/psi-rad") as f32;

        let r_nb = Mat3::from_rotation_z(psi)
            * Mat3::from_rotation_y(theta)
            * Mat3::from_rotation_x(phi);
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
    pub heading_deg: f64,
    pub pitch_deg: f64,
    pub roll_deg: f64,
    pub throttle: f64,
    pub elevator: f64,
    pub aileron: f64,
    pub rudder: f64,
    pub elevator_trim: f64,
    pub flap_deg: f64,
    pub rpm: f64,
    pub autopilot: bool,
    pub phase: &'static str,
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

/// Autopilot switch. When enabled it overrides the keyboard and flies the
/// ground-takeoff -> cruise mission in `jsbsim_sys::autopilot`.
#[derive(Resource)]
pub struct Autopilot {
    pub enabled: bool,
    pub phase: &'static str,
}

pub struct SimPlugin;

impl Plugin for SimPlugin {
    fn build(&self, app: &mut App) {
        app.insert_resource(Sim::new("c172p"))
            .init_resource::<AircraftState>()
            .init_resource::<RenderPose>()
            .insert_resource(Autopilot {
                enabled: true,
                phase: "roll",
            })
            .add_systems(FixedUpdate, (advance_fdm, capture_render_pose).chain())
            .add_systems(Update, publish_state);
    }
}

fn advance_fdm(mut sim: ResMut<Sim>, bindings: Res<crate::input::Bindings>, mut ap: ResMut<Autopilot>) {
    let fdm = &mut sim.fdm;

    // Pilot bindings drive every property the loaded model exposes. This runs
    // unconditionally so flaps and brakes stay live even under the autopilot.
    for b in &bindings.list {
        fdm.set_property(b.property, b.value);
    }

    // The autopilot, when engaged, takes over the channels it owns (elevator,
    // aileron, rudder, throttle, trim) by writing them after the bindings.
    if ap.enabled {
        let (cmds, phase) = autopilot::step(fdm);
        ap.phase = phase;
        autopilot::apply(fdm, &cmds);
    } else {
        ap.phase = "manual";
    }

    fdm.run();
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

fn publish_state(sim: Res<Sim>, ap: Res<Autopilot>, mut state: ResMut<AircraftState>) {
    let (pos, _, _) = sim.pose();
    let fdm = &sim.fdm;
    *state = AircraftState {
        pos,
        alt_ft: fdm.get_property("position/h-sl-ft"),
        cas_kts: fdm.get_property("velocities/vc-kts"),
        heading_deg: fdm.get_property("attitude/psi-deg"),
        pitch_deg: fdm.get_property("attitude/theta-deg"),
        roll_deg: fdm.get_property("attitude/phi-deg"),
        // Report what is actually driving the surfaces, auto or manual.
        throttle: fdm.get_property("fcs/throttle-cmd-norm"),
        elevator: fdm.get_property("fcs/elevator-cmd-norm"),
        aileron: fdm.get_property("fcs/aileron-cmd-norm"),
        rudder: fdm.get_property("fcs/rudder-cmd-norm"),
        elevator_trim: fdm.get_property("fcs/pitch-trim-cmd-norm"),
        flap_deg: fdm.get_property("fcs/flap-pos-deg"),
        rpm: fdm.get_property("propulsion/engine[0]/engine-rpm"),
        autopilot: ap.enabled,
        phase: ap.phase,
    };
}
