//! Cessna 172P (Lycoming IO-320, fixed-pitch prop) — transcribed from JSBSim's
//! `aircraft/c172p/c172p.xml`. The aerodynamic coefficients are copied
//! verbatim so the Rust model can be compared against JSBSim frame by frame;
//! the shape of each `<function>` (product of factors) and `<table>` is
//! preserved.
//!
//! Values are imperial: areas ft², arms ft, moments slug·ft², positions inches.

use super::{bogey, Aircraft, FuelSpec, MassSpec, PointMass};
use crate::aero::{Aerodynamics, Axis, Factor, Function, NamedFn};
use crate::engine::{PistonSpec, PowerplantSpec, PropSpec};
use crate::fcs::{ControlSystem, DirectFcs};
use crate::gear::{BrakeGroup, ContactType, Gear};
use crate::table::{Table1, Table2};

// --- shorthands for the property names the coefficients multiply ---

const QBAR: Factor = Factor::Prop("aero/qbar-psf");
const QBAR_IND: Factor = Factor::Prop("aero/function/qbar-induced-psf");
const SW: Factor = Factor::Prop("metrics/Sw-sqft");
const BW: Factor = Factor::Prop("metrics/bw-ft");
const CBAR: Factor = Factor::Prop("metrics/cbarw-ft");
const KCDGE: Factor = Factor::Prop("aero/function/kCDge");
const KCLGE: Factor = Factor::Prop("aero/function/kCLge");
const ALPHA: Factor = Factor::Prop("aero/alpha-rad");
const MAGBETA: Factor = Factor::Prop("aero/mag-beta-rad");
const BI2VEL: Factor = Factor::Prop("aero/bi2vel");
const CI2VEL: Factor = Factor::Prop("aero/ci2vel");
const P_AERO: Factor = Factor::Prop("velocities/p-aero-rad_sec");
const Q_AERO: Factor = Factor::Prop("velocities/q-aero-rad_sec");
const R_AERO: Factor = Factor::Prop("velocities/r-aero-rad_sec");
const ALPHADOT: Factor = Factor::Prop("aero/alphadot-rad_sec");
const ELEV: Factor = Factor::Prop("fcs/elevator-pos-rad");
const ELEV_MAG: Factor = Factor::Prop("fcs/mag-elevator-pos-rad");
const AIL: Factor = Factor::Prop("fcs/left-aileron-pos-rad");
const RUD: Factor = Factor::Prop("fcs/rudder-pos-rad");

// `val!` / `prod!` are shared from `super` (both defined in aircraft/mod.rs).

// --- 1-D tables -----------------------------------------------------------

/// Delta drag vs flap deflection.
const T_CDDF: Table1 = Table1::new(&[0.0, 10.0, 20.0, 30.0], &[0.0, 0.0070, 0.0120, 0.0180]);
/// Delta lift vs flap deflection.
const T_CLDF: Table1 = Table1::new(&[0.0, 10.0, 20.0, 30.0], &[0.0, 0.2000, 0.3000, 0.3500]);
/// Delta pitching moment vs flap deflection.
const T_CMDF: Table1 = Table1::new(&[0.0, 10.0, 20.0, 30.0], &[0.0, -0.0654, -0.0981, -0.1140]);
/// Roll moment vs sideslip.
const T_CLB: Table1 = Table1::new(&[-0.3490, 0.0, 0.3490], &[0.0322, 0.0, -0.0322]);
/// Yaw moment vs sideslip.
const T_CNB: Table1 = Table1::new(&[-0.3490, 0.0, 0.3490], &[-0.0205, 0.0, 0.0205]);
/// Drag change from ground effect, vs height / MAC.
const T_KCDGE: Table1 = Table1::new(
    &[
        0.0, 0.1, 0.15, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 1.1,
    ],
    &[
        0.4800, 0.5150, 0.6290, 0.7090, 0.8150, 0.8820, 0.9280, 0.9620, 0.9880, 1.0, 1.0, 1.0, 1.0,
    ],
);
/// Lift change from ground effect, vs height / MAC.
const T_KCLGE: Table1 = Table1::new(
    &[
        0.0, 0.1, 0.15, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 1.1,
    ],
    &[
        1.2030, 1.1270, 1.0900, 1.0730, 1.0460, 1.0550, 1.0190, 1.0130, 1.0080, 1.0060, 1.0030,
        1.0020, 1.0,
    ],
);

// --- 2-D tables (row-major: ys[row*ncol + col]) ---------------------------

/// DRAG_due_to_alpha: rows = alpha, cols = flap.
const T_CDWBH: Table2 = Table2::new(
    &[
        -0.0873, -0.0698, -0.0524, -0.0349, -0.0175, 0.0000, 0.0175, 0.0349, 0.0524, 0.0698,
        0.0873, 0.1047, 0.1222, 0.1396, 0.1571, 0.1745, 0.1920, 0.2094, 0.2269, 0.2443, 0.2618,
        0.2793, 0.2967, 0.3142, 0.3316, 0.3491,
    ],
    &[0.0, 10.0, 20.0, 30.0],
    &[
        0.0041, 0.0000, 0.0005, 0.0014, //
        0.0013, 0.0004, 0.0025, 0.0041, //
        0.0001, 0.0023, 0.0059, 0.0084, //
        0.0003, 0.0057, 0.0108, 0.0141, //
        0.0020, 0.0105, 0.0172, 0.0212, //
        0.0052, 0.0168, 0.0251, 0.0299, //
        0.0099, 0.0248, 0.0346, 0.0402, //
        0.0162, 0.0342, 0.0457, 0.0521, //
        0.0240, 0.0452, 0.0583, 0.0655, //
        0.0334, 0.0577, 0.0724, 0.0804, //
        0.0442, 0.0718, 0.0881, 0.0968, //
        0.0566, 0.0874, 0.1053, 0.1148, //
        0.0706, 0.1045, 0.1240, 0.1343, //
        0.0860, 0.1232, 0.1442, 0.1554, //
        0.0962, 0.1353, 0.1573, 0.1690, //
        0.1069, 0.1479, 0.1708, 0.1830, //
        0.1180, 0.1610, 0.1849, 0.1975, //
        0.1298, 0.1746, 0.1995, 0.2126, //
        0.1424, 0.1892, 0.2151, 0.2286, //
        0.1565, 0.2054, 0.2323, 0.2464, //
        0.1727, 0.2240, 0.2521, 0.2667, //
        0.1782, 0.2302, 0.2587, 0.2735, //
        0.1716, 0.2227, 0.2507, 0.2653, //
        0.1618, 0.2115, 0.2388, 0.2531, //
        0.1475, 0.1951, 0.2214, 0.2351, //
        0.1097, 0.1512, 0.1744, 0.1866, //
    ],
);

/// SIDE_force_due_to_beta: rows = beta, cols = flap.
const T_CYB: Table2 = Table2::new(
    &[-0.3490, 0.0, 0.3490],
    &[0.0, 30.0],
    &[0.1370, 0.1060, 0.0, 0.0, -0.1370, -0.1060],
);

/// Side force due to roll rate: rows = alpha, cols = flap.
const T_CYP: Table2 = Table2::new(
    &[0.0, 0.0940],
    &[0.0, 30.0],
    &[-0.0750, -0.1610, -0.1450, -0.2310],
);

/// Side force due to yaw rate: rows = alpha, cols = flap.
const T_CYR: Table2 = Table2::new(
    &[0.0, 0.0940],
    &[0.0, 30.0],
    &[0.2140, 0.1620, 0.2670, 0.2150],
);

/// LIFT_due_to_alpha: rows = alpha, cols = stall hysteresis.
const T_CLWBH: Table2 = Table2::new(
    &[
        -0.0900, 0.0000, 0.0900, 0.1000, 0.1200, 0.1400, 0.1600, 0.1700, 0.1900, 0.2100, 0.2400,
        0.2600, 0.2800, 0.3000, 0.3200, 0.3400, 0.3600,
    ],
    &[0.0, 1.0],
    &[
        -0.2200, -0.2200, //
        0.2500, 0.2500, //
        0.7300, 0.7300, //
        0.8300, 0.7800, //
        0.9200, 0.7900, //
        1.0200, 0.8100, //
        1.0800, 0.8200, //
        1.1300, 0.8300, //
        1.1900, 0.8500, //
        1.2500, 0.8600, //
        1.3500, 0.8800, //
        1.4400, 0.9000, //
        1.4700, 0.9200, //
        1.4300, 0.9500, //
        1.3800, 0.9900, //
        1.3000, 1.0500, //
        1.1500, 1.1500, //
    ],
);

/// Roll moment due to yaw rate: rows = alpha, cols = flap.
const T_CLR: Table2 = Table2::new(
    &[0.0, 0.0940],
    &[0.0, 30.0],
    &[0.0798, 0.1246, 0.1869, 0.2317],
);

// --- the six axes ---------------------------------------------------------

/// Force axes are DRAG, SIDE, LIFT (wind frame).
const DRAG: Axis = Axis::new(&[
    // CDo — drag at zero lift
    prod!(QBAR, SW, val!(0.027)),
    // CDDf — delta drag from flaps
    prod!(
        QBAR,
        SW,
        KCDGE,
        Factor::Table1 {
            var: "fcs/flap-pos-deg",
            table: T_CDDF
        }
    ),
    // CDwbh — drag due to alpha
    prod!(
        QBAR,
        SW,
        KCDGE,
        Factor::Table2 {
            rvar: "aero/alpha-rad",
            cvar: "fcs/flap-pos-deg",
            table: T_CDWBH
        }
    ),
    // CDDe — drag due to elevator
    prod!(QBAR, SW, ELEV_MAG, val!(0.06)),
    // CDbeta — drag due to sideslip
    prod!(QBAR, SW, MAGBETA, val!(0.17)),
]);

const SIDE: Axis = Axis::new(&[
    // CYb — side force due to beta
    prod!(
        QBAR,
        SW,
        Factor::Table2 {
            rvar: "aero/beta-rad",
            cvar: "fcs/flap-pos-deg",
            table: T_CYB
        }
    ),
    // CYdr — side force due to rudder
    prod!(QBAR, SW, RUD, val!(0.187)),
    // CYp — side force due to roll rate
    prod!(
        QBAR,
        SW,
        BI2VEL,
        P_AERO,
        Factor::Table2 {
            rvar: "aero/alpha-rad",
            cvar: "fcs/flap-pos-deg",
            table: T_CYP
        }
    ),
    // CYr — side force due to yaw rate
    prod!(
        QBAR,
        SW,
        BI2VEL,
        R_AERO,
        Factor::Table2 {
            rvar: "aero/alpha-rad",
            cvar: "fcs/flap-pos-deg",
            table: T_CYR
        }
    ),
]);

const LIFT: Axis = Axis::new(&[
    // CLwbh — lift due to alpha (with ground effect and stall hysteresis)
    prod!(
        QBAR,
        SW,
        KCLGE,
        Factor::Table2 {
            rvar: "aero/alpha-rad",
            cvar: "aero/stall-hyst-norm",
            table: T_CLWBH
        }
    ),
    // CLDf — delta lift from flaps
    prod!(
        QBAR,
        SW,
        KCLGE,
        Factor::Table1 {
            var: "fcs/flap-pos-deg",
            table: T_CLDF
        }
    ),
    // CLDe — lift due to elevator
    prod!(QBAR, SW, ELEV, val!(0.43)),
    // CLadot — lift due to alpha rate
    prod!(QBAR, SW, ALPHADOT, CI2VEL, val!(1.7)),
    // CLq — lift due to pitch rate
    prod!(QBAR, SW, Q_AERO, CI2VEL, val!(3.9)),
]);

/// Moment axes are ROLL, PITCH, YAW (body frame).
const ROLL: Axis = Axis::new(&[
    // Clb — roll due to beta
    prod!(
        QBAR,
        SW,
        BW,
        Factor::Table1 {
            var: "aero/beta-rad",
            table: T_CLB
        }
    ),
    // Clp — roll damping
    prod!(QBAR, SW, BW, BI2VEL, P_AERO, val!(-0.484)),
    // Clr — roll due to yaw rate
    prod!(
        QBAR,
        SW,
        BW,
        BI2VEL,
        R_AERO,
        Factor::Table2 {
            rvar: "aero/alpha-rad",
            cvar: "fcs/flap-pos-deg",
            table: T_CLR
        }
    ),
    // ClDa — roll due to aileron
    prod!(QBAR, SW, BW, AIL, val!(0.229)),
    // Cldr — roll due to rudder
    prod!(QBAR, SW, BW, RUD, val!(0.0147)),
]);

const PITCH: Axis = Axis::new(&[
    // Cmo — pitching moment at zero alpha
    prod!(QBAR, SW, CBAR, val!(0.1)),
    // Cmalpha — pitch stiffness
    prod!(QBAR, SW, CBAR, ALPHA, val!(-1.8)),
    // Cmq — pitch damping
    prod!(QBAR, SW, CBAR, CI2VEL, Q_AERO, val!(-12.4)),
    // Cmadot — pitch due to alpha rate
    prod!(QBAR, SW, CBAR, CI2VEL, ALPHADOT, val!(-7.27)),
    // Cmde — pitch due to elevator (uses prop-wash-augmented qbar)
    prod!(QBAR_IND, SW, CBAR, ELEV, val!(-1.122)),
    // Cmdf — delta pitch from flaps
    prod!(
        QBAR,
        SW,
        CBAR,
        Factor::Table1 {
            var: "fcs/flap-pos-deg",
            table: T_CMDF
        }
    ),
]);

const YAW: Axis = Axis::new(&[
    // Cnb — weathercock stability
    prod!(
        QBAR,
        SW,
        BW,
        Factor::Table1 {
            var: "aero/beta-rad",
            table: T_CNB
        }
    ),
    // Cnp — yaw due to roll rate
    prod!(QBAR, SW, BW, BI2VEL, P_AERO, val!(-0.0278)),
    // Cnr — yaw damping
    prod!(QBAR, SW, BW, BI2VEL, R_AERO, val!(-0.0937)),
    // Cnda — adverse yaw from aileron
    prod!(QBAR, SW, BW, AIL, val!(-0.0053)),
    // Cndr — yaw due to rudder (prop-wash-augmented)
    prod!(QBAR_IND, SW, BW, RUD, val!(-0.043)),
]);

/// File-level functions computed before the axes.
const GE_CD: NamedFn = ground_effect!("aero/function/kCDge", T_KCDGE);
const GE_CL: NamedFn = ground_effect!("aero/function/kCLge", T_KCLGE);
const PRE: &[NamedFn] = &[
    GE_CD,
    GE_CL,
    NamedFn {
        // velocity-induced = u_aero + 2 * prop-induced
        name: "aero/function/velocity-induced-fps",
        func: Function::sum(&[
            Factor::Prop("velocities/u-aero-fps"),
            Factor::Prop("propulsion/engine/prop-induced-velocity_fps"),
            Factor::Prop("propulsion/engine/prop-induced-velocity_fps"),
        ]),
    },
    NamedFn {
        // qbar-induced = 0.5 * rho * velocity-induced²
        name: "aero/function/qbar-induced-psf",
        func: Function::product(&[
            Factor::Prop("aero/function/velocity-induced-fps"),
            Factor::Prop("aero/function/velocity-induced-fps"),
            Factor::Prop("atmosphere/rho-slugs_ft3"),
            Factor::Value(0.5),
        ]),
    },
];

// --- mass & balance -------------------------------------------------------

/// Pilot (180 lb) plus two 100-lb fuel tanks. Positions are structural-frame
/// inches, matching `<mass_balance>`.
const POINTS: &[PointMass] = &[
    PointMass {
        weight_lb: 180.0,
        pos_in: [36.0, -14.0, 24.0],
    },
    PointMass {
        weight_lb: 100.0,
        pos_in: [56.0, -112.0, 59.4],
    },
    PointMass {
        weight_lb: 100.0,
        pos_in: [56.0, 112.0, 59.4],
    },
];

const MASS: MassSpec = MassSpec {
    empty_weight_lb: 1500.0,
    empty_inertia: [948.0, 1346.0, 1967.0, -0.0, -0.0, -0.0],
    cg_in: [41.0, 0.0, 36.5],
    points: POINTS,
};

// --- engine & prop --------------------------------------------------------

const PISTON: PistonSpec = PistonSpec {
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
};

const PROP: PropSpec = PropSpec {
    diameter_ft: 75.0 / 12.0,
    ixx: 1.67,
    c_thrust: Table1::new(
        &[
            0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 1.1, 1.2, 1.3, 1.4, 1.5, 1.6,
            1.7, 1.8, 1.9, 2.0, 2.1, 2.2, 2.3, 5.0,
        ],
        &[
            0.073, 0.073, 0.072, 0.071, 0.069, 0.066, 0.062, 0.055, 0.045, 0.034, 0.024, 0.013,
            -0.006, -0.013, -0.024, -0.034, -0.045, -0.055, -0.062, -0.066, -0.069, -0.071, -0.072,
            -0.073, -0.073,
        ],
    ),
    c_power: Table1::new(
        &[
            0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 1.1, 1.2, 1.3, 1.4, 1.5, 1.6,
            1.7, 1.8, 1.9, 2.0, 2.1, 2.2, 2.3, 2.4, 5.0,
        ],
        &[
            0.0660, 0.0700, 0.0700, 0.0660, 0.0600, 0.0530, 0.0501, 0.0469, 0.0426, 0.0360, 0.0282,
            0.0191, 0.0155, 0.0191, 0.0282, 0.0360, 0.0426, 0.0469, 0.0501, 0.0516, 0.0525, 0.0525,
            0.0522, 0.0511, 0.0504, 0.0493,
        ],
    ),
    ct_mach: Table1::new(&[0.85, 1.05], &[1.0, 0.8]),
    cp_mach: Table1::new(&[0.85, 1.05, 2.0], &[1.0, 1.8, 1.4]),
    location: [-37.7, 0.0, 26.6],
};

const FCS: DirectFcs = DirectFcs {
    elevator_gain: 0.01745,
    elevator_lim_deg: (-28.0, 23.0),
    aileron_gain: 0.01745,
    aileron_lim_deg: (-20.0, 15.0),
    rudder_gain: 0.01745,
    rudder_lim_deg: (-16.0, 16.0),
    yaw_damper: None,
    flap_max_deg: 30.0,
    // Full travel in ~6 s: a real C172's flap actuator moves the wings through
    // their range in a handclap, not the half minute the first draft had —
    // which made flaps 20 a 20-second wait inside the M3 approach window.
    flap_time_s: 6.0,
};

// --- landing gear (<ground_reactions>) ------------------------------------
// `bogey` is the shared tyre constructor in `super` (JSBSim's default
// 0.8/0.5/0.02 friction, rebound damping equal to the compress damping, which
// is what the c172p file implies).

/// A rigid skid / tip: isotropic friction, only touches on a crash.
const fn skid(pos_in: [f64; 3]) -> Gear {
    Gear {
        pos_in,
        spring: 20_000.0,
        damp: 2_000.0,
        damp_rebound: 2_000.0,
        static_f: 0.2,
        dyn_f: 0.2,
        roll_f: 0.2,
        contact: ContactType::Structure,
        steer_max_deg: 0.0,
        brake: BrakeGroup::None,
    }
}

/// Nosewheel steers with the rudder (max 10 deg); the mains carry the brakes.
const GEAR: &[Gear] = &[
    bogey(
        [-6.8, 0.0, -19.5],
        1800.0,
        600.0,
        600.0,
        10.0,
        BrakeGroup::None,
    ),
    bogey(
        [58.2, -43.0, -15.5],
        5400.0,
        1600.0,
        1600.0,
        0.0,
        BrakeGroup::Left,
    ),
    bogey(
        [58.2, 43.0, -15.5],
        5400.0,
        1600.0,
        1600.0,
        0.0,
        BrakeGroup::Right,
    ),
    skid([-37.7, 0.0, 26.6]),
    skid([188.0, 0.0, 8.0]),
    skid([43.2, -214.8, 59.4]),
    skid([43.2, 214.8, 59.4]),
];

/// The Cessna 172P. `AERORP` (43.2, 0, 59.4) inches; the FDM shifts moments
/// from there to the loaded CG.
pub const C172: Aircraft = Aircraft {
    name: "c172p",
    wing_area_sqft: 174.0,
    wing_span_ft: 35.8,
    wing_chord_ft: 4.9,
    mass: MASS,
    aero: Aerodynamics {
        pre: PRE,
        drag: DRAG,
        side: SIDE,
        lift: LIFT,
        roll: ROLL,
        pitch: PITCH,
        yaw: YAW,
        aerorp_in: [43.2, 0.0, 59.4],
    },
    control: ControlSystem::Direct(FCS),
    power: PowerplantSpec::Piston {
        engine: PISTON,
        prop: PROP,
    },
    gear: GEAR,
    // The 172 has fixed (welded) spring-steel gear.
    gear_retractable: false,
    rotors: &[],
    rest_h_ft: 4.7,
    has_speedbrake: false,
    max_reverse_lbs: 0.0,
    // Two wing tanks, matching `<mass_balance>` points 1 and 2 (see `POINTS`).
    // Capacity/contents are the model's 185/100 lb.
    fuel: Some(FuelSpec {
        tank_points: [1, 2],
        capacity_lb: 185.0,
        initial_lb: 100.0,
    }),
};
