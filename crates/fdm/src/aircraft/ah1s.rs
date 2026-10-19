//! AH-1S Cobra (Bell 209, two-blade main + tail rotor) — transcribed from
//! JSBSim's `aircraft/ah1s/ah1s.xml` and its `Engines/*.xml` rotor data.
//!
//! The rotor is where the physics lives; the airframe aerodynamics are the full
//! JSBSim port: the wing and fuselage lift/drag/side build-up, the vertical-fin
//! weathercock plus the momentum yaw damping (`CNb_vt`/`CNv_vt`/`CNr_vt` and the
//! `Nr_limiter`), and the horizontal-tail pitch terms — the tail-lift moment
//! (`CLa_ht`) with its stabilator incidence schedule, on top of the `CMw_ht`
//! stiffness / `CMq_ht` damping that give the airframe its pitch stability.
//! Every coefficient is copied verbatim.
//!
//! The one deferred piece is the rotor-downwash coupling (JSBSim's
//! `aero/setup/downwash-enable`): it is transcribed as **disabled**, so the
//! `*-dw` windows evaluate to their "not immersed" branch and the wing/tail see
//! the free-stream qbar and alpha. The rotor carries the weight, so this is
//! secondary.
//!
//! The coefficient framework has no sin/cos/abs of its own (see `aero::Factor`),
//! so the trig atoms the coefficients multiply are published as properties in
//! `fill_props`, and the few composite sub-expressions are file-level functions.
//!
//! Values are imperial: areas ft², arms ft, moments slug·ft², positions inches.

// Transcribed angle breakpoints land on π fractions by coincidence (1.57 = 90°,
// 3.14 = 180°); they are the model's stall landmarks copied verbatim from
// `ah1s.xml`, not approximations of the math constants clippy is looking for.
#![allow(clippy::approx_constant)]

use super::{Aircraft, MassSpec};
use crate::aero::{Aerodynamics, Axis, Factor, Function, NamedFn};
use crate::engine::{PowerplantSpec, RotorDriveSpec};
use crate::fcs::{ControlSystem, HeliFcs, COLLECTIVE_CURVE};
use crate::gear::{BrakeGroup, ContactType, Gear};
use crate::rotor::{RotorControlMap, RotorSpec};
use crate::table::Table1;

// --- shorthands for the property names the coefficients multiply ---

const QBAR: Factor = Factor::Prop("aero/qbar-psf");
const SW: Factor = Factor::Prop("metrics/Sw-sqft");
const RHO: Factor = Factor::Prop("atmosphere/rho-slugs_ft3");
const ALPHA: Factor = Factor::Prop("aero/alpha-rad");
const BETA: Factor = Factor::Prop("aero/beta-rad");
const SIN_A: Factor = Factor::Prop("aero/sin-alpha");
const COS_A: Factor = Factor::Prop("aero/cos-alpha");
const SIN_B: Factor = Factor::Prop("aero/sin-beta");
const COS_B: Factor = Factor::Prop("aero/cos-beta");
const V_AERO: Factor = Factor::Prop("velocities/v-aero-fps");
const W_AERO: Factor = Factor::Prop("velocities/w-aero-fps");
const Q_AERO: Factor = Factor::Prop("velocities/q-aero-rad_sec");
const R_AERO: Factor = Factor::Prop("velocities/r-aero-rad_sec");
const HIHAT: Factor = Factor::Prop("aero/HIhat-ps");
const HWHAT: Factor = Factor::Prop("aero/HWhat-psft");

/// `aero/ref/area-ft2` = solidity · disk area — the fuselage's reference area,
/// distinct from the wing planform `metrics/Sw-sqft`.
const AERO_REF_AREA: f64 = 99.0;
const REF_AREA: Factor = Factor::Value(AERO_REF_AREA);

macro_rules! val {
    ($x:expr) => {
        Factor::Value($x)
    };
}

macro_rules! prod {
    ($($f:expr),* $(,)?) => {
        Function::product(&[$($f),*])
    };
}

// The coefficient sub-expressions and composite sub-terms the axes read, each
// published as a file-level function in `PRE` (the framework can't nest sums or
// call trig itself, so they're precomputed properties).
const CLA_WING: Factor = Factor::Prop("aero/coefficient/CLa_wing");
const CD0A_WING: Factor = Factor::Prop("aero/coefficient/CD0a_wing");
const CLA_HT: Factor = Factor::Prop("aero/coefficient/CLa_ht");
const FUS_LA: Factor = Factor::Prop("aero/fus-la-factor");
const ONE_SIN2B: Factor = Factor::Prop("aero/one-plus-sin2-beta");
const ABS_SIN_A_SIN2: Factor = Factor::Prop("aero/abs-sin-alpha-sin2");

// --- aerodynamics ----------------------------------------------------------

/// `metrics/iw-rad` — wing incidence, 8.5° (chord points over the nose).
const IW_RAD: f64 = 8.5 * std::f64::consts::PI / 180.0;

/// Fuselage drag coefficients, `aero/coefficient/CD{0,a90,b90}_fus` (verbatim).
const CD0_FUS: f64 = 0.105;
const CDA90_FUS: f64 = 0.600;
const CDB90_FUS: f64 = 1.826;
/// `CDa90_fus - CD0_fus`, the swing in the alpha-dependent fuselage drag.
const CDA90_MINUS_CD0: f64 = CDA90_FUS - CD0_FUS;

/// Horizontal-tail area and arm (`metrics/Sh-sqft`, `metrics/lh-ft`).
const SH_SQFT: f64 = 12.0;
const LH_FT: f64 = 16.50;

/// `metrics/Sv-sqft * metrics/lv-ft` = 17.85 · 25.0 — the fin's area·arm, folded
/// into the weathercock term.
const SV_LV: f64 = 17.85 * 25.0;

/// `aero/setup/Nr_limiter` — the hover yaw-rate limiter's gain (0 = pure model,
/// 1 = strongest; the model ships 0.05).
const NR_LIMITER: f64 = 0.05;

/// `aero/ref/vtip-fps` = main-rotor tip speed Ω·R (constant, governed at 324 rpm).
const VTIP_FPS: f64 = 746.44;
/// Main-rotor radius (44 ft diameter).
const ROTOR_R_FT: f64 = 22.0;

/// Horizontal-tail dynamic momentum `HIhat = rho·solidity·Area·(ΩR)·R`
/// (`aero/HIhat-ps`). Multiplied by `CMw_ht` and `w` it is the tail's pitch
/// stiffness moment, and by `CNv_vt` and `v` the yaw damping.
const HIHAT_CONST: f64 = AERO_REF_AREA * VTIP_FPS * ROTOR_R_FT;
/// Horizontal-tail dynamic angular momentum `HWhat = rho·solidity·Area·(ΩR)·R²`
/// (`aero/HWhat-psft`), paired with `CMq_ht`/`q` for pitch damping and
/// `CNr_vt`/`r` for yaw damping.
const HWHAT_CONST: f64 = HIHAT_CONST * ROTOR_R_FT;

/// `aero/coefficient/CLa_wing` — wing lift vs `alpha-iw` (a=5.5, CL_max=1.92),
/// a stall-style curve, verbatim from `ah1s.xml`.
const T_CLA_WING: Table1 = Table1::new(
    &[
        -3.49, -3.14, -2.79, -2.74, -1.83, -1.57, -1.31, -0.40, -0.35, 0.00, 0.35, 0.40, 1.31,
        1.57, 1.83, 2.74, 2.79, 3.14, 3.49,
    ],
    &[
        -1.92, 0.00, 1.92, 1.92, 0.96, 0.00, -0.96, -1.92, -1.92, 0.00, 1.92, 1.92, 0.96, 0.00,
        -0.96, -1.92, -1.92, 0.00, 1.92,
    ],
);

/// `aero/coefficient/CLa_ht` — horizontal-tail lift vs `alpha-iht`
/// (a=3.5, CL_max=1.83), verbatim.
const T_CLA_HT: Table1 = Table1::new(
    &[
        -3.67, -3.14, -2.62, -2.57, -1.83, -1.57, -1.31, -0.58, -0.52, 0.00, 0.52, 0.58, 1.31,
        1.57, 1.83, 2.57, 2.62, 3.14, 3.67,
    ],
    &[
        -1.83, 0.00, 1.83, 1.83, 0.92, 0.00, -0.92, -1.83, -1.83, 0.00, 1.83, 1.83, 0.92, 0.00,
        -0.92, -1.83, -1.83, 0.00, 1.83,
    ],
);

/// `aero/coefficient/CD0a_wing` — wing drag at zero lift vs `alpha-iw`.
const T_CD0A_WING: Table1 = Table1::new(
    &[-3.14, -1.57, -0.26, 0.00, 0.26, 1.57, 3.14],
    &[0.03, 1.500, 0.031, 0.024, 0.031, 1.500, 0.03],
);

/// `aero/coefficient/CNb_vt` — vertical-fin side force vs sideslip (a=3.5,
/// CL_max=1.83).
const T_CNB_VT: Table1 = Table1::new(
    &[-1.57, -1.31, -0.58, -0.52, 0.0, 0.52, 0.58, 1.31, 1.57],
    &[0.0, -0.92, -1.83, -1.83, 0.0, 1.83, 1.83, 0.92, 0.0],
);

/// `CMw_ht` — horizontal-tail pitch stiffness vs advance ratio. Negative →
/// restoring (a downward `w`, i.e. positive alpha, gives a nose-down moment).
const T_CMW_HT: Table1 = Table1::new(&[0.0, 0.45], &[0.0, -0.07]);
/// `CMq_ht` — horizontal-tail pitch damping vs advance ratio.
const T_CMQ_HT: Table1 = Table1::new(&[0.0, 0.45], &[0.0, -0.09]);

/// `CNv_vt` — vertical-tail yaw vs advance ratio (guessed from `CMw_ht`).
const T_CNV_VT: Table1 = Table1::new(&[0.0, 0.45], &[0.0, 0.07]);
/// `CNr_vt` — vertical-tail yaw damping vs advance ratio (guessed from `CMq_ht`).
const T_CNR_VT: Table1 = Table1::new(&[0.0, 0.45], &[0.0, -0.09]);

/// The `Nr_limiter` window: full gain in hover, fading out by mu = 0.15.
const T_NR_LIM: Table1 = Table1::new(&[0.0, 0.08, 0.15], &[1.0, 0.9, 0.0]);

/// `fcs/var-incidence-ht-rad` — stabilator incidence vs longitudinal cyclic
/// (guessed from UH-1 data, positive = nose-up, verbatim).
const T_VAR_INC: Table1 = Table1::new(
    &[-0.04, -0.01, 0.00, 0.02, 0.09, 0.15],
    &[0.21, 0.00, -0.01, -0.01, 0.06, 0.12],
);

// --- axes ------------------------------------------------------------------

const LIFT: Axis = Axis::new(&[
    // La_wing — wing lift; `CLa_wing` already carries its cos²β factor.
    prod!(QBAR, SW, CLA_WING),
    // L0_fus — fuselage lift at zero incidence.
    prod!(QBAR, REF_AREA, val!(-0.05), val!(CDA90_FUS), COS_A, COS_A),
    // La_fus — fuselage lift due to alpha.
    prod!(QBAR, REF_AREA, val!(CDA90_FUS), SIN_A, COS_A, FUS_LA),
]);

const DRAG: Axis = Axis::new(&[
    // D0_wing — wing drag at zero lift.
    prod!(QBAR, SW, CD0A_WING),
    // Di_wing — induced drag (∝ CLa_wing²).
    prod!(QBAR, SW, CLA_WING, CLA_WING, val!(0.06)),
    // D0_fus — fuselage parasite drag.
    prod!(QBAR, REF_AREA, val!(CD0_FUS)),
    // Da_fus — fuselage drag due to alpha.
    prod!(QBAR, REF_AREA, val!(CDA90_MINUS_CD0), ABS_SIN_A_SIN2),
    // Db_fus — fuselage drag due to beta.
    prod!(QBAR, REF_AREA, val!(CDB90_FUS), SIN_B, SIN_B),
]);

const SIDE: Axis = Axis::new(&[
    // Yb_wing — side force on the wing due to beta.
    prod!(QBAR, SW, BETA, val!(-1.0), val!(2.0), SIN_B, COS_B),
    // Yb_fus — side force on the fuselage due to beta.
    prod!(
        QBAR,
        REF_AREA,
        val!(CDB90_FUS),
        val!(-0.6),
        SIN_B,
        COS_B,
        ONE_SIN2B
    ),
]);

const ROLL: Axis = Axis::new(&[]); // no roll contributions in the model

const PITCH: Axis = Axis::new(&[
    // Ma_ht — tailplane lift moment (the stabilator is what trims the aircraft).
    prod!(QBAR, val!(SH_SQFT), val!(LH_FT), val!(-1.0), CLA_HT),
    // Mw_ht — pitch moment due to w (horizontal-tail stiffness, restoring).
    prod!(
        HIHAT,
        Factor::Table1 {
            var: "aero/mu",
            table: T_CMW_HT
        },
        W_AERO
    ),
    // Mq_ht — pitch moment due to q (horizontal-tail damping).
    prod!(
        HWHAT,
        Factor::Table1 {
            var: "aero/mu",
            table: T_CMQ_HT
        },
        Q_AERO
    ),
]);

const YAW: Axis = Axis::new(&[
    // Nb_vt — vertical-fin weathercock: qbar·Sv·lv·CNb_vt(beta).
    prod!(
        QBAR,
        val!(SV_LV),
        Factor::Table1 {
            var: "aero/beta-rad",
            table: T_CNB_VT
        }
    ),
    // Nv_vt — yaw moment due to v (momentum theory).
    prod!(
        HIHAT,
        Factor::Table1 {
            var: "aero/mu",
            table: T_CNV_VT
        },
        V_AERO
    ),
    // Nr_vt — yaw moment due to r.
    prod!(
        HWHAT,
        Factor::Table1 {
            var: "aero/mu",
            table: T_CNR_VT
        },
        R_AERO
    ),
    // Nr_limiter — extra yaw damping to tame hover, fading with advance ratio.
    prod!(
        val!(-0.1),
        HWHAT,
        R_AERO,
        val!(NR_LIMITER),
        Factor::Table1 {
            var: "aero/mu",
            table: T_NR_LIM
        }
    ),
]);

// --- file-level functions, evaluated before the axes -----------------------

/// The stabilator incidence, the alpha/incidence sums and the composite
/// sub-terms the coefficients multiply, then the two tail dynamic-momentum
/// scalars. Order matters: each entry may read the ones above it.
///
/// The `alpha*-rad` sums skip JSBSim's ±π atan/tan wrap — alpha ∈ (-π/2, π/2),
/// so `alpha + incidence` (|incidence| ≤ 0.21 rad, 8.5° wing) never leaves the
/// principal range, and the normalization would be a no-op.
const PRE: &[NamedFn] = &[
    NamedFn {
        name: "fcs/var-incidence-ht-rad",
        func: Function::product(&[Factor::Table1 {
            var: "propulsion/engine/longitudinal-ctrl-rad",
            table: T_VAR_INC,
        }]),
    },
    NamedFn {
        name: "aero/alpha-iw-rad",
        func: Function::sum(&[ALPHA, val!(IW_RAD)]),
    },
    NamedFn {
        name: "aero/alpha-iht-rad",
        func: Function::sum(&[ALPHA, Factor::Prop("fcs/var-incidence-ht-rad")]),
    },
    NamedFn {
        name: "aero/fus-la-factor",
        func: Function::sum(&[val!(0.2), Factor::Prop("aero/sin2-alpha")]),
    },
    NamedFn {
        name: "aero/one-plus-sin2-beta",
        func: Function::sum(&[val!(1.0), Factor::Prop("aero/sin2-beta")]),
    },
    NamedFn {
        name: "aero/abs-sin-alpha-sin2",
        func: prod!(
            Factor::Prop("aero/abs-sin-alpha"),
            Factor::Prop("aero/sin2-alpha")
        ),
    },
    // Coefficients that depend on the sums above (downwash windows are disabled,
    // so their "compl" factor is 1.0 and drops out).
    NamedFn {
        name: "aero/coefficient/CLa_wing",
        func: prod!(
            Factor::Table1 {
                var: "aero/alpha-iw-rad",
                table: T_CLA_WING
            },
            COS_B,
            COS_B
        ),
    },
    NamedFn {
        name: "aero/coefficient/CD0a_wing",
        func: Function::product(&[Factor::Table1 {
            var: "aero/alpha-iw-rad",
            table: T_CD0A_WING,
        }]),
    },
    NamedFn {
        name: "aero/coefficient/CLa_ht",
        func: prod!(
            Factor::Table1 {
                var: "aero/alpha-iht-rad",
                table: T_CLA_HT
            },
            COS_B,
            COS_B
        ),
    },
    NamedFn {
        name: "aero/HIhat-ps",
        func: prod!(RHO, val!(HIHAT_CONST)),
    },
    NamedFn {
        name: "aero/HWhat-psft",
        func: prod!(RHO, val!(HWHAT_CONST)),
    },
];

// --- mass & balance (<mass_balance>) ---------------------------------------

/// Empty AH-1S. Fuel and crew point masses are deferred (JSBSim loads them
/// separately), so the flying weight is the 8500 lb empty weight.
const MASS: MassSpec = MassSpec {
    empty_weight_lb: 8500.0,
    empty_inertia: [2593.0, 14320.0, 12330.0, 0.0, 0.0, 0.0],
    cg_in: [172.0, 0.0, 75.0],
    points: &[],
};

// --- landing gear: two skids (<ground_reactions>) --------------------------

/// A skid shoe: stiff, no steer, no brake, isotropic friction (the model's
/// `0.85 / 0.65 / 0.65`).
const fn skid_shoe(pos_in: [f64; 3], spring: f64, damp: f64) -> Gear {
    Gear {
        pos_in,
        spring,
        damp,
        damp_rebound: damp,
        static_f: 0.85,
        dyn_f: 0.65,
        roll_f: 0.65,
        contact: ContactType::Bogey,
        steer_max_deg: 0.0,
        brake: BrakeGroup::None,
    }
}

const GEAR: &[Gear] = &[
    skid_shoe([115.90, -42.0, -4.0], 6375.0, 1275.0),
    skid_shoe([115.90, 42.0, -4.0], 6375.0, 1275.0),
    skid_shoe([235.0, -42.0, -5.0], 6375.0, 1275.0),
    skid_shoe([235.0, 42.0, -5.0], 6375.0, 1275.0),
    // Rear bumper — only touches on a tail-down strike.
    Gear {
        pos_in: [490.0, 0.0, 40.0],
        spring: 6000.0,
        damp: 2000.0,
        damp_rebound: 2000.0,
        static_f: 0.85,
        dyn_f: 0.65,
        roll_f: 0.65,
        contact: ContactType::Structure,
        steer_max_deg: 0.0,
        brake: BrakeGroup::None,
    },
];

// --- rotors (<propulsion>/<thruster> + Engines/*.xml) ----------------------

/// Main rotor: 44 ft diameter, two blades, 324 rpm, CCW seen from above.
const MAIN_ROTOR: RotorSpec = RotorSpec {
    diameter_ft: 44.0,
    num_blades: 2,
    gear_ratio: 20.37,
    nominal_rpm: 324.0,
    chord_ft: 2.25,
    lift_curve_slope: 6.0,
    twist_rad: -0.175,
    hinge_offset_ft: 3.30,
    flapping_moment: 1382.0,
    mass_moment: 85.0,
    inflow_lag_s: 0.09,
    tip_loss_factor: 1.0,
    sense: 1.0,
    ground_effect_exp: 0.19,
    ground_effect_shift_ft: 10.0,
    location_in: [176.0, 0.0, 153.0],
    orient_deg: [0.0, 90.0, 0.0],
    control_map: RotorControlMap::Main,
};

/// Tail rotor: 8.5 ft diameter, two blades, 1660 rpm, driven off the main rotor
/// (shared engine shaft via the gear ratios). Mounted on the left of the fin.
const TAIL_ROTOR: RotorSpec = RotorSpec {
    diameter_ft: 8.5,
    num_blades: 2,
    gear_ratio: 3.9759,
    nominal_rpm: 1660.0,
    chord_ft: 0.7,
    lift_curve_slope: 6.0,
    twist_rad: 0.0,
    hinge_offset_ft: 0.015,
    flapping_moment: 1.45,
    mass_moment: 0.46,
    inflow_lag_s: 0.09,
    tip_loss_factor: 1.0,
    sense: 1.0,
    ground_effect_exp: 0.0,
    ground_effect_shift_ft: 0.0,
    location_in: [496.67, 16.0, 119.0],
    orient_deg: [0.0, 0.0, 90.0],
    control_map: RotorControlMap::Tail,
};

const ROTORS: &[RotorSpec] = &[MAIN_ROTOR, TAIL_ROTOR];

/// Engine -> rotor drive: 1500 hp, governor-locked at 324 rpm. The 20.37:1 main
/// gear ratio lives on the rotor (see `MAIN_ROTOR`), where it links the tail
/// rotor's speed to the main rotor's.
const ROTOR_DRIVE: RotorDriveSpec = RotorDriveSpec {
    nominal_rpm: 324.0,
    max_power_hp: 1500.0,
};

// --- control system (Systems/rotor_control.xml) ----------------------------

const HELI_FCS: HeliFcs = HeliFcs {
    collective_gain: 0.22,
    collective_bias: 0.14,
    collective_profile: 1.0,
    center_sensitivity: 1.5,
    longitudinal_gain: 0.125,
    longitudinal_bias: 0.020,
    lateral_gain: 0.05,
    lateral_bias: 0.0,
    lateral_col_mix_gain: -0.1,
    lateral_col_mix_bias: 0.01,
    pedal_gain: 0.18,
    pedal_bias: 0.0,
    pedal_col_mix_gain: 0.65,
    pedal_col_mix_bias: -0.1,
    collective_curve: COLLECTIVE_CURVE,
};

/// The AH-1S Cobra. `AERORP` (176, 0, 55) inches.
pub const AH1S: Aircraft = Aircraft {
    name: "ah1s",
    // `metrics/Sw-sqft` — the wing planform, 16.60 ft². (The fuselage terms use
    // `aero/ref/area-ft2` = 99, the rotor blade area, folded in as `REF_AREA`.)
    wing_area_sqft: 16.60,
    wing_span_ft: 10.75,
    wing_chord_ft: 1.54,
    mass: MASS,
    aero: Aerodynamics {
        pre: PRE,
        drag: DRAG,
        side: SIDE,
        lift: LIFT,
        roll: ROLL,
        pitch: PITCH,
        yaw: YAW,
        aerorp_in: [176.0, 0.0, 55.0],
    },
    control: ControlSystem::Heli(HELI_FCS),
    power: PowerplantSpec::Rotor(ROTOR_DRIVE),
    gear: GEAR,
    gear_retractable: false,
    rotors: ROTORS,
    rest_h_ft: 6.0,
    has_speedbrake: false,
    max_reverse_lbs: 0.0,
    fuel: None,
};
