//! Boeing 737 (2× CFM56-5 turbofans, direct controls) — transcribed from
//! JSBSim's `aircraft/737/737.xml` and `engine/CFM56.xml`. The aerodynamic
//! coefficients are copied verbatim so the Rust model can be compared against
//! JSBSim frame by frame; the `<product>`-of-factors shape the evaluator
//! already understands is preserved.
//!
//! Against the other airframes:
//!
//! * **Direct FCS, not FBW.** The 737's control law is a plain command →
//!   surface `aerosurface_scale`: full stick is ±0.3 rad of elevator, ±0.35 rad
//!   of aileron and rudder. This is the same [`crate::fcs::DirectFcs`] the
//!   c172p uses, so no new control code is needed.
//! * **Two turbofans.** The CFM56 is non-afterburning, so idle + mil thrust
//!   tables keyed on (mach, density altitude) cover the whole envelope.
//! * **The inertia cross-product is sign-flipped**: the model sets
//!   `negated_crossproduct_inertia="true"`, so JSBSim builds `J_xz = +ixz`.
//!   `crate::state::Inertia::new` uses `J_xz = -ixz`, so the transcribed
//!   `empty_inertia[4]` is the *negated* model value: `-8000`.
//!
//! The 737 also carries flight- and ground-spoiler terms (`CDsb`/`CDsp`,
//! `kCLsb`/`kCLsp`); they are omitted here — the `DirectFcs` has no speedbrake
//! channel. The effect is a slightly low drag/lift without the boards out.
//!
//! Values are imperial: areas ft², arms ft, moments slug·ft², positions inches.

use super::{Aircraft, MassSpec, PointMass};
use crate::aero::{Aerodynamics, Axis, Factor, Function, NamedFn};
use crate::engine::{PowerplantSpec, TurbineSpec};
use crate::fcs::{ControlSystem, DirectFcs};
use crate::gear::{BrakeGroup, ContactType, Gear};
use crate::table::{Table1, Table2};

const R2D: f64 = 180.0 / std::f64::consts::PI;
const D2R: f64 = std::f64::consts::PI / 180.0;

// --- shorthands for the property names the coefficients multiply ---

const QBAR: Factor = Factor::Prop("aero/qbar-psf");
const SW: Factor = Factor::Prop("metrics/Sw-sqft");
const BW: Factor = Factor::Prop("metrics/bw-ft");
const CBAR: Factor = Factor::Prop("metrics/cbarw-ft");
const ALPHA: Factor = Factor::Prop("aero/alpha-rad");
const BETA: Factor = Factor::Prop("aero/beta-rad");
const BI2VEL: Factor = Factor::Prop("aero/bi2vel");
const CI2VEL: Factor = Factor::Prop("aero/ci2vel");
const P_AERO: Factor = Factor::Prop("velocities/p-aero-rad_sec");
const Q_AERO: Factor = Factor::Prop("velocities/q-aero-rad_sec");
const R_AERO: Factor = Factor::Prop("velocities/r-aero-rad_sec");
const ALPHADOT: Factor = Factor::Prop("aero/alphadot-rad_sec");
const CL2: Factor = Factor::Prop("aero/cl-squared");
const KCDGE: Factor = Factor::Prop("aero/function/kCDge");
const KCLGE: Factor = Factor::Prop("aero/function/kCLge");
const ELEV: Factor = Factor::Prop("fcs/elevator-pos-rad");
const ELEV_MAG: Factor = Factor::Prop("fcs/mag-elevator-pos-rad");
const AIL: Factor = Factor::Prop("fcs/left-aileron-pos-rad");
const RUD: Factor = Factor::Prop("fcs/rudder-pos-rad");
const FLAPN: Factor = Factor::Prop("fcs/flap-pos-norm");
const GEARN: Factor = Factor::Prop("gear/gear-pos-norm");

/// A literal factor.
macro_rules! val {
    ($x:expr) => {
        Factor::Value($x)
    };
}

/// A `<product>` function from a factor list.
macro_rules! prod {
    ($($f:expr),* $(,)?) => {
        Function::product(&[$($f),*])
    };
}

// --- ground-effect functions (<function name="aero/function/...">) ---------

const T_KCDGE: Table1 = Table1::new(
    &[0.0, 0.1, 0.15, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8],
    &[0.0480, 0.5150, 0.6290, 0.7090, 0.8150, 0.8820, 0.9280, 0.9620, 0.9880, 1.0000],
);
const T_KCLGE: Table1 = Table1::new(
    &[0.0, 0.1, 0.15, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 1.1],
    &[
        1.2030, 1.1270, 1.0900, 1.0730, 1.0460, 1.0280, 1.0190, 1.0130, 1.0080, 1.0060,
        1.0030, 1.0020, 1.0000,
    ],
);

// --- 1-D tables -----------------------------------------------------------

const T_CD0: Table1 = Table1::new(
    &[-1.57, -0.26, 0.0, 0.26, 1.57],
    &[1.5000, 0.0420, 0.0210, 0.0420, 1.5000],
);
const T_CDMACH: Table1 = Table1::new(&[0.0, 0.79, 1.10, 1.80], &[0.0, 0.0, 0.0230, 0.0150]);
const T_CDBETA: Table1 = Table1::new(
    &[-1.57, -0.26, 0.0, 0.26, 1.57],
    &[1.2300, 0.0500, 0.0000, 0.0500, 1.2300],
);
const T_CLALPHA: Table1 = Table1::new(&[-0.20, 0.0, 0.23, 0.46], &[-0.68, 0.20, 1.20, 0.20]);

const T_CLDA_M: Table1 = Table1::new(&[0.0, 2.0], &[0.100, 0.033]);
const T_CMDE_M: Table1 = Table1::new(&[0.0, 2.0], &[-1.20, -0.30]);

// --- the six axes ---------------------------------------------------------

/// Force axes are DRAG, SIDE, LIFT (wind frame).
const DRAG: Axis = Axis::new(&[
    // CD0 — drag at zero lift
    prod!(QBAR, SW, Factor::Table1 { var: "aero/alpha-rad", table: T_CD0 }),
    // CDi — induced drag (needs the previous frame's CL²)
    prod!(QBAR, SW, CL2, KCDGE, val!(0.043)),
    // CDmach — drag rise with Mach
    prod!(QBAR, SW, Factor::Table1 { var: "velocities/mach", table: T_CDMACH }),
    // CDflap — trailing-edge flaps
    prod!(QBAR, SW, FLAPN, val!(0.059)),
    // CDgear — landing gear
    prod!(QBAR, SW, GEARN, val!(0.015)),
    // CDbeta — sideslip
    prod!(QBAR, SW, Factor::Table1 { var: "aero/beta-rad", table: T_CDBETA }),
    // CDde — elevator deflection
    prod!(QBAR, SW, ELEV_MAG, val!(0.059)),
]);

const SIDE: Axis = Axis::new(&[
    // CYb — side force due to beta
    prod!(QBAR, SW, BETA, val!(-1.0)),
]);

const LIFT: Axis = Axis::new(&[
    // CLalpha — lift due to alpha
    prod!(QBAR, SW, KCLGE, Factor::Table1 { var: "aero/alpha-rad", table: T_CLALPHA }),
    // dCLflap — trailing-edge flaps
    prod!(QBAR, SW, FLAPN, KCLGE, val!(0.9)),
    // CLde — elevator
    prod!(QBAR, SW, ELEV, val!(0.2)),
]);

/// Moment axes are ROLL, PITCH, YAW (body frame).
const ROLL: Axis = Axis::new(&[
    // Clb — roll due to beta
    prod!(QBAR, SW, BW, BETA, val!(-0.09)),
    // Clp — roll damping
    prod!(QBAR, SW, BW, BI2VEL, P_AERO, val!(-0.4)),
    // Clr — roll due to yaw rate
    prod!(QBAR, SW, BW, BI2VEL, R_AERO, val!(0.09)),
    // Clda — aileron
    prod!(QBAR, SW, BW, AIL, Factor::Table1 { var: "velocities/mach", table: T_CLDA_M }),
    // Cldr — rudder
    prod!(QBAR, SW, BW, RUD, val!(0.01)),
]);

const PITCH: Axis = Axis::new(&[
    // Cmalpha — pitch stiffness
    prod!(QBAR, SW, CBAR, ALPHA, val!(-0.6)),
    // Cmde — elevator
    prod!(QBAR, SW, CBAR, ELEV, Factor::Table1 { var: "velocities/mach", table: T_CMDE_M }),
    // Cmq — pitch damping
    prod!(QBAR, SW, CBAR, CI2VEL, Q_AERO, val!(-27.0)),
    // Cmadot — pitch due to alpha rate
    prod!(QBAR, SW, CBAR, CI2VEL, ALPHADOT, val!(-16.0)),
]);

const YAW: Axis = Axis::new(&[
    // Cnb — weathercock stability
    prod!(QBAR, SW, BW, BETA, val!(0.26)),
    // Cnr — yaw damping
    prod!(QBAR, SW, BW, BI2VEL, R_AERO, val!(-0.35)),
    // Cndr — rudder
    prod!(QBAR, SW, BW, RUD, val!(-0.20)),
]);

/// File-level functions computed before the axes.
const PRE: &[NamedFn] = &[
    NamedFn {
        name: "aero/function/kCDge",
        func: Function::product(&[Factor::Table1 { var: "aero/h_b-mac-ft", table: T_KCDGE }]),
    },
    NamedFn {
        name: "aero/function/kCLge",
        func: Function::product(&[Factor::Table1 { var: "aero/h_b-mac-ft", table: T_KCLGE }]),
    },
];

// --- mass & balance -------------------------------------------------------

/// Three fuel tanks: full left/right wing tanks (10000 lb each) and a part-full
/// center tank (4000 lb). Positions are structural-frame inches, matching the
/// `<tank>` locations.
const POINTS: &[PointMass] = &[
    PointMass { weight_lb: 10000.0, pos_in: [520.0, -80.0, -18.0] },
    PointMass { weight_lb: 10000.0, pos_in: [520.0, 80.0, -18.0] },
    PointMass { weight_lb: 4000.0, pos_in: [480.0, 0.0, -18.0] },
];

const MASS: MassSpec = MassSpec {
    empty_weight_lb: 83000.0,
    // `negated_crossproduct_inertia` in the model file flips the sign of ixz
    // that `Inertia::new` expects, hence -8000.
    empty_inertia: [562000.0, 1.473e6, 1.894e6, 0.0, -8000.0, 0.0],
    cg_in: [639.0, 0.0, -40.0],
    points: POINTS,
};

// --- engine (CFM56-5) -----------------------------------------------------

/// Idle-thrust fraction, row = Mach, col = density altitude (ft).
const IDLE_THRUST: Table2 = Table2::new(
    &[0.0, 0.2, 0.4, 0.6, 0.8, 1.0],
    &[-10000.0, 0.0, 10000.0, 20000.0, 30000.0, 40000.0, 50000.0, 60000.0],
    &[
        0.0420, 0.0436, 0.0528, 0.0694, 0.0899, 0.1183, 0.1467, 0.0, //
        0.0500, 0.0501, 0.0335, 0.0544, 0.0797, 0.1049, 0.1342, 0.0, //
        0.0040, 0.0047, 0.0020, 0.0272, 0.0595, 0.0891, 0.1203, 0.0, //
        0.0, 0.0, 0.0, 0.0, 0.0276, 0.0718, 0.1073, 0.0, //
        0.0, 0.0, 0.0, 0.0, 0.0174, 0.0468, 0.0900, 0.0, //
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0422, 0.0700, 0.0, //
    ],
);

/// Military-thrust fraction, row = Mach, col = density altitude (ft). The 1.2
/// row is all-zero (the 737's CFM56 has no thrust there).
const MIL_THRUST: Table2 = Table2::new(
    &[0.0, 0.2, 0.4, 0.6, 0.8, 1.0, 1.2],
    &[-10000.0, 0.0, 10000.0, 20000.0, 30000.0, 40000.0, 50000.0, 60000.0],
    &[
        1.2600, 1.0000, 0.7400, 0.5340, 0.3720, 0.2410, 0.1490, 0.0, //
        1.1710, 0.9340, 0.6970, 0.5060, 0.3550, 0.2310, 0.1430, 0.0, //
        1.1500, 0.9210, 0.6920, 0.5060, 0.3570, 0.2330, 0.1450, 0.0, //
        1.1810, 0.9510, 0.7210, 0.5320, 0.3780, 0.2480, 0.1540, 0.0, //
        1.2580, 1.0200, 0.7820, 0.5820, 0.4170, 0.2750, 0.1700, 0.0, //
        1.3690, 1.1200, 0.8710, 0.6510, 0.4750, 0.3150, 0.1950, 0.0, //
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, //
    ],
);

/// Unused: the CFM56 is not afterburning (a 1×1 zero table keeps the field).
const AUG_THRUST: Table2 = Table2::new(&[0.0], &[0.0], &[0.0]);

/// Two CFM56-5s. Modelled as one thrust at the mean thruster location (y = 0),
/// so the symmetric pair produces no yaw moment (the model's nozzles sit at
/// y = ∓193 in).
const CFM56: TurbineSpec = TurbineSpec {
    milthrust: 20000.0,
    maxthrust: 20000.0,
    augmented: false,
    bleed: 0.04,
    idle_n2: 60.0,
    max_n2: 100.0,
    count: 2,
    location: [540.0, 0.0, -40.0],
    idle_thrust: IDLE_THRUST,
    mil_thrust: MIL_THRUST,
    aug_thrust: AUG_THRUST,
};

// --- flight-control law (direct command → surface) ------------------------

/// A plain `aerosurface_scale` law: full stick is ±0.3 rad elevator, ±0.35 rad
/// aileron and rudder. Limits are stored in degrees (the pipeline multiplies by
/// the rad/deg `gain`), matching the c172p's convention.
const FCS: DirectFcs = DirectFcs {
    elevator_gain: D2R,
    elevator_lim_deg: (-0.3 * R2D, 0.3 * R2D),
    aileron_gain: D2R,
    aileron_lim_deg: (-0.35 * R2D, 0.35 * R2D),
    rudder_gain: D2R,
    rudder_lim_deg: (-0.35 * R2D, 0.35 * R2D),
    // 40 deg of flaps, run over the sum of the model's 9-segment kinematic
    // schedule (5+4+3+2+2+2+2+2 = 22 s) collapsed onto one rate.
    flap_max_deg: 40.0,
    flap_time_s: 22.0,
};

// --- landing gear (<ground_reactions>) ------------------------------------

const fn bogey(
    pos_in: [f64; 3],
    spring: f64,
    damp: f64,
    damp_rebound: f64,
    steer_max_deg: f64,
    brake: BrakeGroup,
) -> Gear {
    Gear {
        pos_in,
        spring,
        damp,
        damp_rebound,
        static_f: 0.8,
        dyn_f: 0.5,
        roll_f: 0.02,
        contact: ContactType::Bogey,
        steer_max_deg,
        brake,
    }
}

/// Nosewheel steers (max 35 deg); the mains carry the brakes.
const GEAR: &[Gear] = &[
    bogey([158.0, 0.0, -84.0], 90000.0, 4000.0, 8000.0, 35.0, BrakeGroup::None),
    bogey([648.0, -100.0, -84.0], 120000.0, 10000.0, 20000.0, 0.0, BrakeGroup::Left),
    bogey([648.0, 100.0, -84.0], 120000.0, 10000.0, 20000.0, 0.0, BrakeGroup::Right),
];

/// The Boeing 737. `AERORP` (625, 0, 24) inches; the FDM shifts moments from
/// there to the loaded CG. `rest_h_ft` is the model's `altitude 49 IN` reset.
pub const B737: Aircraft = Aircraft {
    name: "737",
    wing_area_sqft: 1171.0,
    wing_span_ft: 94.70,
    wing_chord_ft: 12.31,
    mass: MASS,
    aero: Aerodynamics {
        pre: PRE,
        drag: DRAG,
        side: SIDE,
        lift: LIFT,
        roll: ROLL,
        pitch: PITCH,
        yaw: YAW,
        aerorp_in: [625.0, 0.0, 24.0],
    },
    control: ControlSystem::Direct(FCS),
    power: PowerplantSpec::Turbine(CFM56),
    gear: GEAR,
    gear_retractable: true,
    rotors: &[],
    rest_h_ft: 49.0 / 12.0,
};
