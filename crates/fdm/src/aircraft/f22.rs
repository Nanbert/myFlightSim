//! F-22A Raptor (2× Pratt & Whitney F119-PW-100 turbofans, thrust-vectoring
//! FBW) — transcribed from JSBSim's `aircraft/f22/f22.xml` and
//! `engine/F119-PW-1.xml`. The aerodynamic coefficients are copied verbatim so
//! the Rust model can be compared against JSBSim frame by frame; the same
//! `<product>`-of-factors shape the evaluator already understands is preserved.
//!
//! Two things set the F-22 apart from the c172p data:
//!
//! * **The airframe is statically unstable subsonic** (`Cma_M = +0.097` at
//!   M0.8, the comment in the model file calls it "10% unstable at subsonic").
//!   It flies only because of the three-axis rate-command fly-by-wire, so the
//!   [`crate::fcs::Fbw`] control law is load-bearing, not decoration.
//! * **The inertia cross-product is sign-flipped**: the model sets
//!   `negated_crossproduct_inertia="true"`, so JSBSim builds `J_xz = +ixz`.
//!   `crate::state::Inertia::new` uses `J_xz = -ixz`, so the transcribed
//!   `empty_inertia[4]` is the *negated* model value: `-713`.
//!
//! Values are imperial: areas ft², arms ft, moments slug·ft², positions inches.

// Transcribed constants land on π fractions by coincidence (0.5236 = 30°,
// 0.7854 = 45°, 1.5707 = 90°); they are the model's angle breakpoints and
// limits, not approximations of the math constants clippy is looking for.
#![allow(clippy::approx_constant)]

use super::{Aircraft, MassSpec, PointMass};
use crate::aero::{Aerodynamics, Axis, Factor, Function, NamedFn};
use crate::engine::{PowerplantSpec, TurbineSpec};
use crate::fcs::{ControlSystem, FbwSpec};
use crate::gear::{BrakeGroup, ContactType, Gear};
use crate::table::{Table1, Table2};

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
const BETADOT: Factor = Factor::Prop("aero/betadot-rad_sec");
const CL2: Factor = Factor::Prop("aero/cl-squared");
const KCDGE: Factor = Factor::Prop("aero/function/kCDge");
const KCLGE: Factor = Factor::Prop("aero/function/kCLge");
const ELEV: Factor = Factor::Prop("fcs/elevator-pos-rad");
const AIL: Factor = Factor::Prop("fcs/left-aileron-pos-rad");
const RUD: Factor = Factor::Prop("fcs/rudder-pos-rad");
const LEF: Factor = Factor::Prop("fcs/lef-pos-rad");
const FLAPN: Factor = Factor::Prop("fcs/flap-pos-norm");
const SBN: Factor = Factor::Prop("fcs/speedbrake-pos-norm");
const GEARN: Factor = Factor::Prop("gear/gear-pos-norm");
const THRUSTN: Factor = Factor::Prop("fcs/thrust-norm");

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

const T_KCLGE: Table1 = Table1::new(
    &[0.0, 0.1, 0.15, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 1.1],
    &[
        1.3893, 1.2108, 1.1972, 1.2108, 1.1785, 1.0697, 1.0578, 1.0323, 1.0136, 1.0051,
        1.0017, 1.0, 1.0,
    ],
);
const T_KCDGE: Table1 = Table1::new(
    &[0.0, 0.1, 0.15, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 1.1],
    &[
        0.5235, 0.4759, 0.5250, 0.5388, 0.5924, 0.6471, 0.6941, 0.7624, 0.8253, 0.8812,
        0.9412, 1.0, 1.0,
    ],
);

// --- 1-D tables -----------------------------------------------------------

const T_CD0: Table1 = Table1::new(
    &[-1.5700, -0.2600, 0.0000, 0.2600, 1.5700],
    &[1.5000, 0.0480, 0.00820, 0.0480, 1.5000],
);
const T_CDMACH: Table1 = Table1::new(&[0.0, 0.81, 1.10, 1.80, 2.40], &[0.0, 0.0, 0.0220, 0.0140, 0.020]);
const T_CDBETA: Table1 = Table1::new(
    &[-1.5700, -0.2600, 0.0000, 0.2600, 1.5700],
    &[1.2300, 0.0500, 0.0000, 0.0500, 1.2300],
);
const T_CDDE: Table1 = Table1::new(
    &[-0.5236, -0.218, 0.0, 0.218, 0.5236],
    &[0.075, 0.0375, 0.010, 0.0375, 0.077],
);
/// The standard 12-point alpha breakpoint list reused by the LEF/q-coefficients.
const ALPHA_XS: [f64; 12] = [
    -0.175, -0.087, 0.0, 0.087, 0.175, 0.262, 0.349, 0.436, 0.524, 0.611, 0.698, 0.785,
];
/// The standard 16-point alpha breakpoint list used by the aileron/rudder terms.
const ALPHA_XS16: [f64; 16] = [
    -0.1745, -0.0873, 0.0, 0.0873, 0.1745, 0.2618, 0.3491, 0.4363, 0.5236, 0.6109, 0.6981,
    0.7854, 0.8727, 1.0472, 1.2217, 1.3963,
];

const T_CDDLEF: Table1 = Table1::new(
    &ALPHA_XS,
    &[0.003, 0.001, 0.000, 0.001, 0.002, 0.004, 0.007, 0.011, 0.015, 0.019, 0.023, 0.024],
);
const T_CDQ: Table1 = Table1::new(
    &ALPHA_XS,
    &[
        -1.265, -2.139, -0.308, 1.402, 3.369, 5.135, 6.880, 8.060, 11.201, 13.872,
        19.217, 22.105,
    ],
);
const T_CDQ_DLEF: Table1 = Table1::new(
    &ALPHA_XS,
    &[0.067, 0.034, 0.028, 0.029, 0.033, 0.059, 0.061, 0.027, 0.036, 0.047, 0.029, 0.015],
);

const T_CYB_M: Table1 = Table1::new(&[0.4, 1.2, 1.6], &[0.0, 0.0573, -0.1719]);

const T_CLALPHA: Table1 = Table1::new(&[-0.1745, 0.7854, 1.3963], &[-0.4279, 2.3000, 1.0726]);
const T_CLDLEF: Table1 = Table1::new(
    &[
        -0.175, -0.087, 0.0, 0.087, 0.175, 0.262, 0.349, 0.436, 0.524, 0.611, 0.698, 0.785,
        0.8727, 1.0472, 1.2217, 1.3963,
    ],
    &[
        -0.012, -0.004, 0.002, 0.007, 0.012, 0.018, 0.022, 0.025, 0.026, 0.028, 0.030,
        0.032, 0.030, 0.028, 0.025, 0.020,
    ],
);
const T_CLQ: Table1 = Table1::new(
    &ALPHA_XS,
    &[
        8.7127, 25.7114, 28.9000, 31.3973, 31.0872, 30.4071, 28.9735, 28.4242, 27.8647,
        27.2654, 30.5158, 27.8165,
    ],
);

const T_CLB: Table1 = Table1::new(
    &[-0.0873, 0.0, 0.0873, 0.1745, 0.2618, 0.3491, 0.4363, 0.5236, 0.6109],
    &[0.0189, 0.0176, 0.0119, 0.0050, 0.0049, 0.0028, 0.0084, 0.0171, 0.0044],
);
const T_CLDA_M: Table1 = Table1::new(&[0.6, 1.2, 1.6], &[0.0, -0.063, -0.063]);
const T_CLDR_M: Table1 = Table1::new(&[0.6, 1.6], &[0.0, -0.0201]);
const T_CLP: Table1 = Table1::new(&[0.0, 0.3490], &[-0.4000, -0.3100]);
const T_CLR: Table1 = Table1::new(
    &ALPHA_XS,
    &[
        -0.126, -0.026, 0.063, 0.113, 0.208, 0.230, 0.319, 0.437, 0.680, 0.100, 0.447,
        -0.330,
    ],
);
const T_CLDA: Table1 = Table1::new(
    &ALPHA_XS16,
    &[
        0.045, 0.052, 0.060, 0.057, 0.052, 0.047, 0.040, 0.034, 0.029, 0.023, 0.019,
        0.015, 0.012, 0.011, 0.009, 0.008,
    ],
);
const T_CLDR: Table1 = Table1::new(
    &ALPHA_XS16,
    &[
        0.0078, 0.0070, 0.0059, 0.0043, 0.0023, 0.0007, -0.0008, 0.0004, 0.0019, 0.0019,
        0.0017, 0.0009, 0.0003, 0.0, 0.0, 0.0,
    ],
);

const T_CMA_M: Table1 = Table1::new(
    &[0.6, 0.8, 0.9, 1.0, 1.1, 1.2],
    &[0.0, 0.0974, -0.3323, -0.9626, -0.8480, -0.7907],
);
const T_CMQ: Table1 = Table1::new(
    &ALPHA_XS,
    &[
        -7.21, -5.40, -5.23, -5.26, -6.11, -6.64, -5.69, -6.00, -6.20, -6.40, -6.60,
        -6.00,
    ],
);
const T_CMADOT: Table1 = Table1::new(&[0.257, 1.8], &[-1.5611, -2.05]);

const T_CNB: Table1 = Table1::new(&[-0.349, 0.0, 0.873], &[0.0745, 0.1780, 0.0086]);
const T_CNB_M: Table1 = Table1::new(
    &[0.6, 0.7, 0.8, 0.9, 1.0, 1.2, 1.4, 1.6],
    &[0.0, -0.0688, -0.0172, 0.0229, 0.0, -0.0688, 0.0057, 0.0688],
);
const T_CNR: Table1 = Table1::new(
    &ALPHA_XS,
    &[
        -0.380, -0.363, -0.378, -0.386, -0.370, -0.453, -0.550, -0.582, -0.595, -0.637,
        -1.020, -0.840,
    ],
);
const T_CNDR: Table1 = Table1::new(
    &ALPHA_XS16,
    &[
        -0.048, -0.045, -0.045, -0.045, -0.044, -0.045, -0.047, -0.048, -0.040, -0.033,
        -0.025, -0.017, -0.010, -0.0075, -0.0075, -0.0075,
    ],
);
const T_CNDR_M: Table1 = Table1::new(&[0.6, 1.2, 1.6], &[0.0, 0.0401, 0.0716]);
const T_CNDA: Table1 = Table1::new(
    &ALPHA_XS16,
    &[
        0.010, 0.009, 0.010, 0.009, 0.008, 0.006, -0.002, -0.003, -0.005, -0.008, -0.006,
        -0.006, -0.008, -0.010, -0.011, -0.013,
    ],
);

// --- 2-D tables (row-major: ys[row*ncol + col]) ---------------------------

/// CmDh: rows = alpha, cols = elevator. The stabilator's own pitching moment —
/// the primary source of pitch stability at low Mach, where Cma is neutral.
const T_CMDH: Table2 = Table2::new(
    &[
        -0.0873, 0.0, 0.0873, 0.1745, 0.2618, 0.3491, 0.4363, 0.5236, 0.6109, 0.6981,
        0.7854, 0.8727, 1.0472, 1.2217,
    ],
    &[-0.5236, 0.0, 0.5236],
    &[
        0.3222, -0.0546, -0.4911, //
        0.3482, 0.0000, -0.4729, //
        0.3897, 0.0286, -0.4782, //
        0.3689, 0.0156, -0.4988, //
        0.3975, 0.0546, -0.4287, //
        0.3819, 0.0442, -0.2722, //
        0.4131, 0.0831, -0.1637, //
        0.4599, 0.1377, -0.0546, //
        0.4443, 0.1325, -0.0530, //
        0.3819, 0.1013, -0.0650, //
        0.3585, 0.0909, -0.0624, //
        0.3430, 0.1299, -0.0364, //
        0.0208, -0.1117, -0.1975, //
        0.3248, -0.4521, -0.5144, //
    ],
);

/// CmDtv: rows = alpha, cols = thrust-vectoring nozzle. Zero for the simplified
/// FCS (which leaves `fcs/tvc-pos-rad` at 0), kept so the term is present.
const T_CMDTV: Table2 = Table2::new(
    &[
        -0.0873, 0.0, 0.0873, 0.1745, 0.2618, 0.3491, 0.4363, 0.5236, 0.6109, 0.6981,
        0.7854, 0.8727, 1.0472, 1.2217,
    ],
    &[-0.3491, 0.0, 0.3491],
    &[
        0.4417, 0.0, -0.7924, //
        0.4650, 0.0, -0.8366, //
        0.4448, 0.0, -0.8808, //
        0.4677, 0.0, -0.8886, //
        0.4755, 0.0, -0.8600, //
        0.4625, 0.0, -0.7534, //
        0.4963, 0.0, -0.6901, //
        0.4936, 0.0, -0.6408, //
        0.4781, 0.0, -0.6000, //
        0.5025, 0.0, -0.5454, //
        0.5015, 0.0, -0.5143, //
        0.4209, 0.0, -0.5284, //
        0.4417, 0.0, -0.5047, //
        0.2988, 0.0, -0.4883, //
    ],
);

// --- the six axes ---------------------------------------------------------

/// Force axes are DRAG, SIDE, LIFT (wind frame).
const DRAG: Axis = Axis::new(&[
    // CD0 — drag at zero lift
    prod!(QBAR, SW, KCDGE, Factor::Table1 { var: "aero/alpha-rad", table: T_CD0 }),
    // CDi — induced drag (needs the previous frame's CL²)
    prod!(QBAR, SW, CL2, KCDGE, val!(0.0700)),
    // CDmach — drag rise with Mach
    prod!(QBAR, SW, Factor::Table1 { var: "velocities/mach", table: T_CDMACH }),
    // CDflap — trailing-edge flaps
    prod!(QBAR, SW, FLAPN, KCDGE, val!(0.0750)),
    // CDgear — landing gear
    prod!(QBAR, SW, GEARN, val!(0.0200)),
    // CDsb — speedbrakes
    prod!(QBAR, SW, SBN, val!(0.0240)),
    // CDbeta — sideslip
    prod!(QBAR, SW, KCDGE, Factor::Table1 { var: "aero/beta-rad", table: T_CDBETA }),
    // CDde — elevator deflection
    prod!(QBAR, SW, KCDGE, Factor::Table1 { var: "fcs/elevator-pos-rad", table: T_CDDE }),
    // CDDlef — leading-edge flaps
    prod!(QBAR, SW, KCDGE, LEF, Factor::Table1 { var: "aero/alpha-rad", table: T_CDDLEF }),
    // CDq — pitch rate
    prod!(QBAR, SW, Q_AERO, CI2VEL, Factor::Table1 { var: "aero/alpha-rad", table: T_CDQ }),
    // CDq_Dlef — pitch rate × leading-edge flaps
    prod!(QBAR, SW, Q_AERO, CI2VEL, LEF, Factor::Table1 {
        var: "aero/alpha-rad", table: T_CDQ_DLEF }),
]);

const SIDE: Axis = Axis::new(&[
    // CYb — side force due to beta
    prod!(QBAR, SW, BETA, val!(-1.7875)),
    // CYb_M — beta × Mach
    prod!(QBAR, SW, BETA, Factor::Table1 { var: "velocities/mach", table: T_CYB_M }),
    // CYbdot — beta rate
    prod!(QBAR, SW, BETADOT, val!(-0.0458)),
    // CYDa — aileron
    prod!(QBAR, SW, AIL, val!(-1.1516e-3)),
    // CYDr — rudder
    prod!(QBAR, SW, RUD, val!(0.0619)),
    // CYp — roll rate
    prod!(QBAR, SW, BI2VEL, P_AERO, val!(0.0399)),
    // CYr — yaw rate
    prod!(QBAR, SW, BI2VEL, R_AERO, val!(0.1218)),
]);

const LIFT: Axis = Axis::new(&[
    // CLalpha — lift due to alpha
    prod!(QBAR, SW, Factor::Table1 { var: "aero/alpha-rad", table: T_CLALPHA }),
    // CLDlef — leading-edge flaps
    prod!(QBAR, SW, LEF, KCLGE, Factor::Table1 { var: "aero/alpha-rad", table: T_CLDLEF }),
    // CLadot — alpha rate
    prod!(QBAR, SW, ALPHADOT, CI2VEL, val!(2.5602)),
    // CLq — pitch rate
    prod!(QBAR, SW, Q_AERO, CI2VEL, KCLGE, Factor::Table1 {
        var: "aero/alpha-rad", table: T_CLQ }),
    // dCLflap — trailing-edge flaps
    prod!(QBAR, SW, FLAPN, KCLGE, val!(0.3500)),
    // dCLsb — speedbrakes
    prod!(QBAR, SW, SBN, KCLGE, val!(0.3500)),
    // CLde — elevator
    prod!(QBAR, SW, ELEV, KCLGE, val!(0.190)),
]);

/// Moment axes are ROLL, PITCH, YAW (body frame).
const ROLL: Axis = Axis::new(&[
    // Clb — roll due to beta
    prod!(QBAR, SW, BW, BETA, Factor::Table1 { var: "aero/alpha-rad", table: T_CLB }),
    // Clda_M — aileron × alpha × Mach
    prod!(QBAR, SW, BW, ALPHA, AIL, Factor::Table1 { var: "velocities/mach", table: T_CLDA_M }),
    // Cldr_M — rudder × alpha × Mach
    prod!(QBAR, SW, BW, ALPHA, RUD, Factor::Table1 { var: "velocities/mach", table: T_CLDR_M }),
    // Clbdot — beta rate
    prod!(QBAR, SW, BW, BI2VEL, BETADOT, val!(-0.0037)),
    // Clp — roll damping
    prod!(QBAR, SW, BW, BI2VEL, P_AERO, Factor::Table1 { var: "aero/alpha-rad", table: T_CLP }),
    // Clr — roll due to yaw rate
    prod!(QBAR, SW, BW, BI2VEL, R_AERO, Factor::Table1 { var: "aero/alpha-rad", table: T_CLR }),
    // Clda — aileron
    prod!(val!(4.0), QBAR, SW, BW, AIL, Factor::Table1 { var: "aero/alpha-rad", table: T_CLDA }),
    // ClDr — rudder
    prod!(val!(2.0), QBAR, SW, BW, RUD, Factor::Table1 { var: "aero/alpha-rad", table: T_CLDR }),
]);

const PITCH: Axis = Axis::new(&[
    // CmDh — stabilator pitching moment
    prod!(QBAR, SW, CBAR, Factor::Table2 {
        rvar: "aero/alpha-rad", cvar: "fcs/elevator-pos-rad", table: T_CMDH }, val!(1.5)),
    // CmDtv — thrust vectoring (zero while TVC is off)
    prod!(val!(1.25), QBAR, SW, CBAR, THRUSTN, Factor::Table2 {
        rvar: "aero/alpha-rad", cvar: "fcs/tvc-pos-rad", table: T_CMDTV }),
    // Cma_M — alpha × Mach (positive subsonic: unstable)
    prod!(QBAR, SW, CBAR, ALPHA, Factor::Table1 { var: "velocities/mach", table: T_CMA_M }),
    // Cmq — pitch damping
    prod!(QBAR, SW, CBAR, CI2VEL, Q_AERO, Factor::Table1 { var: "aero/alpha-rad", table: T_CMQ }),
    // Cmadot — alpha rate
    prod!(QBAR, SW, CBAR, CI2VEL, ALPHADOT, Factor::Table1 {
        var: "velocities/mach", table: T_CMADOT }),
]);

const YAW: Axis = Axis::new(&[
    // Cnb — weathercock stability
    prod!(QBAR, SW, BW, BETA, Factor::Table1 { var: "aero/alpha-rad", table: T_CNB }),
    // Cnb_M — beta × Mach
    prod!(QBAR, SW, BW, BETA, Factor::Table1 { var: "velocities/mach", table: T_CNB_M }),
    // Cnbdot — beta rate
    prod!(QBAR, SW, BW, BI2VEL, BETADOT, val!(-0.0115)),
    // Cnp — yaw due to roll rate
    prod!(QBAR, SW, BW, BI2VEL, P_AERO, val!(-0.0035)),
    // Cnr — yaw damping
    prod!(QBAR, SW, BW, BI2VEL, R_AERO, Factor::Table1 { var: "aero/alpha-rad", table: T_CNR }),
    // CnDr — rudder
    prod!(val!(5.0), QBAR, SW, BW, RUD, Factor::Table1 { var: "aero/alpha-rad", table: T_CNDR }),
    // Cndr_M — rudder × alpha × Mach
    prod!(QBAR, SW, BW, ALPHA, RUD, Factor::Table1 { var: "velocities/mach", table: T_CNDR_M }),
    // Cnda — adverse yaw from aileron
    prod!(val!(2.0), QBAR, SW, BW, AIL, Factor::Table1 { var: "aero/alpha-rad", table: T_CNDA }),
]);

/// File-level functions computed before the axes.
const PRE: &[NamedFn] = &[
    NamedFn {
        name: "aero/function/kCLge",
        func: Function::product(&[Factor::Table1 { var: "aero/h_b-mac-ft", table: T_KCLGE }]),
    },
    NamedFn {
        name: "aero/function/kCDge",
        func: Function::product(&[Factor::Table1 { var: "aero/h_b-mac-ft", table: T_KCDGE }]),
    },
];

// --- mass & balance -------------------------------------------------------

/// Pilot (230 lb) plus two full 9350-lb fuel tanks. Positions are structural
/// frame inches, matching `<mass_balance>` and the `<tank>` locations.
const POINTS: &[PointMass] = &[
    PointMass { weight_lb: 230.0, pos_in: [148.8, 0.0, 20.0] },
    PointMass { weight_lb: 9350.0, pos_in: [446.478, -36.0, -18.6033] },
    PointMass { weight_lb: 9350.0, pos_in: [446.478, 36.0, -18.6033] },
];

const MASS: MassSpec = MassSpec {
    empty_weight_lb: 43430.0,
    // `negated_crossproduct_inertia` in the model file flips the sign of ixz
    // that `Inertia::new` expects, hence -713.
    empty_inertia: [56005.0, 280766.0, 354090.0, 0.0, -713.0, 0.0],
    cg_in: [445.7, 0.0, -18.6],
    points: POINTS,
};

// --- engine (F119-PW-100) -------------------------------------------------

/// Idle-thrust fraction, row = Mach, col = density altitude (ft).
const IDLE_THRUST: Table2 = Table2::new(
    &[0.0, 0.2, 0.4, 0.6, 0.8, 1.0],
    &[-10000.0, 0.0, 10000.0, 20000.0, 30000.0, 40000.0, 50000.0, 60000.0],
    &[
        0.0430, 0.0488, 0.0528, 0.0694, 0.0899, 0.1183, 0.1467, 0.0, //
        0.0500, 0.0501, 0.0335, 0.0544, 0.0797, 0.1049, 0.1342, 0.0, //
        0.0040, 0.0047, 0.0020, 0.0272, 0.0595, 0.0891, 0.1203, 0.0, //
        0.0, 0.0, 0.0, 0.0, 0.0276, 0.0718, 0.1073, 0.0, //
        0.0, 0.0, 0.0, 0.0, 0.0474, 0.0868, 0.0900, 0.0, //
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0552, 0.0800, 0.0, //
    ],
);

/// Military-thrust fraction, row = Mach, col = density altitude (ft).
const MIL_THRUST: Table2 = Table2::new(
    &[0.0, 0.2, 0.4, 0.6, 0.8, 1.0, 1.2, 1.4, 1.6, 1.8, 2.0, 2.2],
    &[-10000.0, 0.0, 10000.0, 20000.0, 30000.0, 40000.0, 50000.0, 60000.0, 70000.0, 80000.0],
    &[
        1.2600, 1.0000, 0.7400, 0.5340, 0.3720, 0.2410, 0.1490, 0.0800, 0.040, 0.0, //
        1.1710, 0.9340, 0.6970, 0.5060, 0.3550, 0.2310, 0.1430, 0.0780, 0.040, 0.0, //
        1.1500, 0.9210, 0.6920, 0.5060, 0.3570, 0.2330, 0.1450, 0.0790, 0.040, 0.0, //
        1.1810, 0.9510, 0.7210, 0.5320, 0.3780, 0.2480, 0.1540, 0.0850, 0.045, 0.0, //
        1.2580, 1.0200, 0.7820, 0.5820, 0.4170, 0.2750, 0.1700, 0.0935, 0.045, 0.0, //
        1.3690, 1.1760, 0.9135, 0.6836, 0.4988, 0.3308, 0.2048, 0.1127, 0.052, 0.0, //
        1.4850, 1.2915, 1.0238, 0.7812, 0.5723, 0.3780, 0.2363, 0.1300, 0.065, 0.0, //
        1.5941, 1.4070, 1.1403, 0.8873, 0.6595, 0.4452, 0.2762, 0.1520, 0.076, 0.0, //
        1.6211, 1.5250, 1.2450, 0.9890, 0.7774, 0.5260, 0.3211, 0.1767, 0.087, 0.0, //
        1.6211, 1.6290, 1.3474, 1.0950, 0.8774, 0.6160, 0.3811, 0.2096, 0.104, 0.0, //
        1.6211, 1.6000, 1.4300, 1.1850, 0.9500, 0.7050, 0.4435, 0.2420, 0.121, 0.0, //
        1.6211, 1.6000, 1.3600, 1.1000, 0.8600, 0.6167, 0.4000, 0.2050, 0.100, 0.0, //
    ],
);

/// Augmented (afterburning) thrust fraction, row = Mach, col = density altitude.
const AUG_THRUST: Table2 = Table2::new(
    &[0.0, 0.2, 0.4, 0.6, 0.8, 1.0, 1.2, 1.4, 1.6, 1.8, 2.0, 2.2, 2.4, 2.6],
    &[-10000.0, 0.0, 10000.0, 20000.0, 30000.0, 40000.0, 50000.0, 60000.0, 70000.0, 80000.0],
    &[
        1.1816, 1.0000, 0.8184, 0.6627, 0.5280, 0.3756, 0.2327, 0.1279, 0.063, 0.0, //
        1.1308, 0.9599, 0.7890, 0.6406, 0.5116, 0.3645, 0.2258, 0.1242, 0.062, 0.0, //
        1.1150, 0.9474, 0.7798, 0.6340, 0.5070, 0.3615, 0.2240, 0.1232, 0.061, 0.0, //
        1.1284, 0.9589, 0.7894, 0.6420, 0.5134, 0.3661, 0.2268, 0.1240, 0.062, 0.0, //
        1.1707, 0.9990, 0.8177, 0.6647, 0.5309, 0.3784, 0.2345, 0.1290, 0.065, 0.0, //
        1.2411, 1.1056, 0.9080, 0.7367, 0.5876, 0.4182, 0.2590, 0.1425, 0.071, 0.0, //
        1.3287, 1.1817, 0.9682, 0.7835, 0.6233, 0.4430, 0.2745, 0.1513, 0.075, 0.0, //
        1.4365, 1.2756, 1.0430, 0.8422, 0.6678, 0.4735, 0.2933, 0.1613, 0.081, 0.0, //
        1.5711, 1.3923, 1.1349, 0.9135, 0.7218, 0.5103, 0.3162, 0.1739, 0.085, 0.0, //
        1.7301, 1.5307, 1.2449, 0.9987, 0.7870, 0.5553, 0.3441, 0.1893, 0.095, 0.0, //
        1.8314, 1.6485, 1.3740, 1.1000, 0.8627, 0.6708, 0.3764, 0.2070, 0.103, 0.0, //
        1.9700, 1.7745, 1.4805, 1.3020, 0.9555, 0.6676, 0.4137, 0.2275, 0.115, 0.0, //
        2.0700, 1.8900, 1.6065, 1.4070, 1.0500, 0.7560, 0.4830, 0.2657, 0.138, 0.0, //
        2.2000, 2.0160, 1.7220, 1.5120, 1.1550, 0.8400, 0.5460, 0.3003, 0.150, 0.0, //
    ],
);

/// Two F119-PW-100s (afterburning, augmethod 1). Modelled as one thrust at the
/// mean thruster location (y = 0).
const F119: TurbineSpec = TurbineSpec {
    milthrust: 26950.0,
    maxthrust: 37000.0,
    augmented: true,
    idle_n2: 53.0,
    max_n2: 100.0,
    count: 2,
    location: [684.131, 0.0, 0.0],
    idle_thrust: IDLE_THRUST,
    mil_thrust: MIL_THRUST,
    aug_thrust: AUG_THRUST,
};

// --- flight-control law (rate-command FBW) --------------------------------

/// Fixed gains for the three-axis rate-command loop. See [`crate::fcs`] for the
/// structure and the fidelity note.
static F22_FBW: FbwSpec = FbwSpec {
    roll_rate_max: 3.0,
    roll_kp: 0.5,
    roll_ki: 1.2,
    roll_int_limit: 1.0,
    aileron_lim_rad: 0.436,

    pitch_rate_max: 0.87,
    pitch_kp: 0.9,
    pitch_ki: 1.6,
    pitch_int_limit: 0.8,
    elevator_lim_rad: 0.5236,
    pitch_down_lim: 0.5236,
    pitch_up_lim: 0.8727,
    alpha_limit_rad: 0.48,
    alpha_limit_tvc_rad: 0.7,
    tvc_gain: 0.6,
    tvc_lim_rad: 0.349,

    yaw_rate_max: 0.436,
    yaw_kp: 0.6,
    yaw_ki: 1.0,
    yaw_int_limit: 0.44,
    rudder_lim_rad: 0.5236,

    lef_alpha_gain: 1.38,
    lef_dyn_gain: -9.05,
    lef_offset: 1.45,
    lef_lim_rad: 0.6108,
};

// --- landing gear (<ground_reactions>) ------------------------------------

const fn bogey(
    pos_in: [f64; 3],
    spring: f64,
    damp: f64,
    steer_max_deg: f64,
    brake: BrakeGroup,
) -> Gear {
    Gear {
        pos_in,
        spring,
        damp,
        damp_rebound: damp,
        static_f: 0.8,
        dyn_f: 0.5,
        roll_f: 0.02,
        contact: ContactType::Bogey,
        steer_max_deg,
        brake,
    }
}

/// Nosewheel steers (max 80 deg); the mains carry the brakes.
const GEAR: &[Gear] = &[
    bogey([96.7, 0.0, -89.3], 18603.4, 6201.1, 80.0, BrakeGroup::None),
    bogey([468.8, -48.0, -89.3], 82011.2, 22402.2, 0.0, BrakeGroup::Left),
    bogey([468.8, 48.0, -89.3], 82011.2, 22402.2, 0.0, BrakeGroup::Right),
];

/// The F-22A Raptor. `AERORP` (417.6, 0, 0) inches; the FDM shifts moments from
/// there to the loaded CG.
pub const F22: Aircraft = Aircraft {
    name: "f22",
    wing_area_sqft: 840.0,
    wing_span_ft: 44.49,
    wing_chord_ft: 23.06,
    mass: MASS,
    aero: Aerodynamics {
        pre: PRE,
        drag: DRAG,
        side: SIDE,
        lift: LIFT,
        roll: ROLL,
        pitch: PITCH,
        yaw: YAW,
        aerorp_in: [417.6, 0.0, 0.0],
    },
    control: ControlSystem::Fbw(&F22_FBW),
    power: PowerplantSpec::Turbine(F119),
    gear: GEAR,
    gear_retractable: true,
};
