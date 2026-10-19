//! Helicopter rotor — a port of JSBSim's `FGRotor` (T. Kreitler, 2010-12).
//!
//! Mid-fidelity, momentum-theory model: blade-element inflow with a first-order
//! lag, a coning angle, longitudinal/lateral flapping, the H/J in-plane hub
//! forces, hinge-offset hub moments, and blade-drag torque. The formulas track
//! the SH79 (NASA TM-73254) / TA77 references cited in `FGRotor.cpp`; the port
//! keeps the algebra verbatim and drops only the transmission (clutch, brake,
//! free-wheeling unit) and the LQR control-law scheduling — the rotor itself is
//! faithful.
//!
//! Units are JSBSim's: feet, slugs, radians, lbf, seconds.
//!
//! # Where it plugs in
//!
//! A rotor is *propulsion-like*: it is stepped once per frame and, like
//! `thrust`/`mprop`, its force is frozen across the RK4 sub-stages (see
//! `lib.rs`). The only integrated state is the induced inflow ratio `nu`, a
//! first-order lag; everything else (thrust, `a0`, `a1`, `b1`, torque, H/J) is
//! an algebraic function of the current velocity.
//!
//! # Axes
//!
//! Forces and moments are built in the rotor's *control axes* (hub-shaft-rotor,
//! HSR) then rotated to the body frame. JSBSim does this in two hops: the rotor
//! exposes a custom transform `mT` (from `<orient>`) and the hub forces are
//! pre-rotated by `HsrToTbo`. The two are exact inverses when `<orient>` equals
//! the shaft's body alignment, so the net is identity for a well-formed rotor —
//! which is why the main rotor's hover thrust lands on body −z (up).

use crate::state::{add, cross, Vec3};

const D2R: f64 = std::f64::consts::PI / 180.0;

/// A row-major 3×3 matrix.
type Mat3 = [[f64; 3]; 3];

/// `m * v` for a row-major matrix and a column vector.
fn mat3_mul(m: Mat3, v: Vec3) -> Vec3 {
    [
        m[0][0] * v[0] + m[0][1] * v[1] + m[0][2] * v[2],
        m[1][0] * v[0] + m[1][1] * v[1] + m[1][2] * v[2],
        m[2][0] * v[0] + m[2][1] * v[1] + m[2][2] * v[2],
    ]
}

/// Body -> hub-shaft-rotor. JSBSim's `TboToHsr`, whose initializer lists the
/// elements in visual row order (see the `FGMatrix33` 9-arg constructor).
const TBO_TO_HSR: Mat3 = [[0.0, 0.0, 1.0], [0.0, 1.0, 0.0], [-1.0, 0.0, 0.0]];
/// HSR -> body, the transpose of [`TBO_TO_HSR`].
const HSR_TO_TBO: Mat3 = [[0.0, 0.0, -1.0], [0.0, 1.0, 0.0], [1.0, 0.0, 0.0]];

/// Which controls a rotor answers to (JSBSim's `<controlmap>`).
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum RotorControlMap {
    /// Cyclic + collective + pedals (the main rotor).
    Main,
    /// Antitorque pedals only (the tail rotor).
    Tail,
}

/// Static rotor data, straight from the engine XML (`<rotor>` block). Data
/// only — no state — so it can live in a `const Aircraft`.
pub struct RotorSpec {
    pub diameter_ft: f64,
    pub num_blades: u32,
    /// Rotor turns per engine-shaft turn; links the tail rotor's RPM to the
    /// main rotor's.
    pub gear_ratio: f64,
    pub nominal_rpm: f64,
    pub chord_ft: f64,
    pub lift_curve_slope: f64,
    /// Blade twist, root to tip (rad); negative for wash-out.
    pub twist_rad: f64,
    pub hinge_offset_ft: f64,
    pub flapping_moment: f64,
    pub mass_moment: f64,
    pub inflow_lag_s: f64,
    pub tip_loss_factor: f64,
    /// Rotor sense seen from above: +1 CCW, −1 CW, 0 coaxial.
    pub sense: f64,
    pub ground_effect_exp: f64,
    pub ground_effect_shift_ft: f64,
    /// Hub position in the structural frame (inches from the datum).
    pub location_in: Vec3,
    /// Rotor orientation `(roll, pitch, yaw)` in degrees (JSBSim `<orient>`).
    pub orient_deg: Vec3,
    pub control_map: RotorControlMap,
}

/// First-order Tustin filter (JSBSim's `Filter`), smoothing the above-ground
/// level that feeds ground effect.
struct Lag {
    prev_in: f64,
    prev_out: f64,
    ca: f64,
    cb: f64,
}

impl Lag {
    fn new(coeff: f64, dt: f64) -> Self {
        let denom = 2.0 + coeff * dt;
        Lag {
            prev_in: 0.0,
            prev_out: 0.0,
            ca: coeff * dt / denom,
            cb: (2.0 - coeff * dt) / denom,
        }
    }
    fn execute(&mut self, input: f64) -> f64 {
        let out = (input + self.prev_in) * self.ca + self.prev_out * self.cb;
        self.prev_in = input;
        self.prev_out = out;
        out
    }
}

/// The running rotor: static spec, derived constants, and per-step state.
pub struct Rotor {
    spec: &'static RotorSpec,

    // --- derived constants (from Configure) ---
    radius_ft: f64,
    solidity: f64,
    lock_number_by_rho: f64,
    /// Powers of the radius, `R[0..5]`.
    r_pow: [f64; 5],
    /// Powers of the tip-loss factor, `B[0..5]`.
    b_pow: [f64; 5],
    min_rpm: f64,
    max_rpm: f64,

    // --- transforms ---
    /// Custom -> body (`mT`), from `<orient>`.
    m_t: Mat3,
    /// Body -> custom (`mT^T`), JSBSim's `InvTransform`.
    inv_transform: Mat3,

    // --- per-step environment ---
    rho: f64,
    /// Hub position relative to the CG, in body axes (ft).
    hub_body_ft: Vec3,

    // --- dynamic state / outputs ---
    rpm: f64,
    omega: f64,
    beta_orient: f64,
    a0: f64,
    a1: f64,
    b1: f64,
    a_dw: f64,
    a1s: f64,
    b1s: f64,
    h_drag: f64,
    j_side: f64,
    torque: f64,
    ct: f64,
    lambda: f64,
    mu: f64,
    nu: f64,
    v_induced: f64,
    thrust: f64,

    // --- controls (rad) ---
    collective_rad: f64,
    longitudinal_rad: f64,
    lateral_rad: f64,

    damp_hagl: Lag,

    // --- body-frame output for the last step ---
    force_body: Vec3,
    moment_body: Vec3,
}

impl Rotor {
    pub fn new(spec: &'static RotorSpec, dt: f64) -> Self {
        let radius = 0.5 * spec.diameter_ft;
        let sol = spec.num_blades as f64 * spec.chord_ft / (std::f64::consts::PI * radius);
        let lock = spec.lift_curve_slope * spec.chord_ft * radius.powi(4) / spec.flapping_moment;

        let mut r_pow = [1.0; 5];
        let mut b_pow = [1.0; 5];
        for i in 1..5 {
            r_pow[i] = r_pow[i - 1] * radius;
            b_pow[i] = b_pow[i - 1] * spec.tip_loss_factor;
        }

        let m_t = orient_matrix(spec.orient_deg);
        let inv_transform = transpose(m_t);

        Rotor {
            spec,
            radius_ft: radius,
            solidity: sol,
            lock_number_by_rho: lock,
            r_pow,
            b_pow,
            min_rpm: 1.0,
            max_rpm: 2.0 * spec.nominal_rpm,
            m_t,
            inv_transform,
            rho: 0.0,
            hub_body_ft: [0.0; 3],
            rpm: 0.0,
            omega: 0.0,
            beta_orient: 0.0,
            a0: 0.0,
            a1: 0.0,
            b1: 0.0,
            a_dw: 0.0,
            a1s: 0.0,
            b1s: 0.0,
            h_drag: 0.0,
            j_side: 0.0,
            torque: 0.0,
            ct: 0.0,
            lambda: -0.005,
            mu: 0.0,
            nu: 0.005,
            v_induced: 0.0,
            thrust: 0.0,
            collective_rad: 0.0,
            longitudinal_rad: 0.0,
            lateral_rad: 0.0,
            damp_hagl: Lag::new(1.0, dt),
            force_body: [0.0; 3],
            moment_body: [0.0; 3],
        }
    }

    pub fn spec(&self) -> &'static RotorSpec {
        self.spec
    }

    /// Set the rotor speed (rpm). The governor holds the main rotor at nominal;
    /// the tail rotor is driven from it via the gear ratios (see `lib.rs`).
    pub fn set_rpm(&mut self, rpm: f64) {
        self.rpm = rpm.clamp(self.min_rpm, self.max_rpm);
    }

    /// Write the control angles for this step (radians): collective pitch, then
    /// longitudinal and lateral cyclic. The tail rotor ignores the latter two.
    pub fn set_controls(&mut self, collective_rad: f64, longitudinal_rad: f64, lateral_rad: f64) {
        self.collective_rad = collective_rad;
        self.longitudinal_rad = longitudinal_rad;
        self.lateral_rad = lateral_rad;
    }

    /// Advance the rotor one step and recompute its body force and moment.
    ///
    /// `uvw`/`pqr` are the body velocity (ft/s) and rate (rad/s); `hub_body_ft`
    /// is the hub's position relative to the CG in body axes (ft), for the
    /// moment arm; `rho` is air density (slugs/ft³); `h_agl` the height above
    /// ground (ft); `dt` the frame step (s). Mirrors `FGRotor::CalcRotorState`.
    pub fn calculate(
        &mut self,
        uvw: Vec3,
        pqr: Vec3,
        hub_body_ft: Vec3,
        rho: f64,
        h_agl: f64,
        dt: f64,
    ) {
        self.rho = rho.max(1e-9);
        self.hub_body_ft = hub_body_ft;
        self.omega = (self.rpm / 60.0) * 2.0 * std::f64::consts::PI;

        // Lateral (roll) and longitudinal (pitch) control, and collective.
        let a_ic = self.lateral_rad;
        let b_ic = self.longitudinal_rad;
        let theta_col = self.collective_rad;

        // Optional ground effect: ge_factor 1.0 = none, 0.5 = max influence.
        // The lagged height avoids jumps in h_agl feeding jumps into thrust.
        let mut ge_factor = 1.0;
        if self.spec.ground_effect_exp > 1e-5 {
            let h = h_agl.max(0.0);
            let filtered = self.damp_hagl.execute(h) + self.spec.ground_effect_shift_ft;
            ge_factor -= (-filtered * self.spec.ground_effect_exp).exp()
                * (self.rpm / self.spec.nominal_rpm);
            ge_factor = ge_factor.clamp(0.5, 1.0);
        }

        let v_hub = self.hub_vel_body2ca(uvw, pqr, a_ic, b_ic);
        let av_fus = self.fus_angvel_body2ca(pqr);

        self.calc_flow_and_thrust(theta_col, v_hub[0], v_hub[2], ge_factor, dt);
        self.calc_coning_angle(theta_col);
        self.calc_flapping_angles(theta_col, av_fus);
        self.calc_drag_and_side_forces(theta_col);
        self.calc_torque();

        let f_s = self.body_forces(a_ic, b_ic);
        let m_s = self.body_moments(a_ic, b_ic);

        // FGForce::GetBodyForces: vFb = mT * vFn; vM = vMn + hub×vFb.
        let f_body = mat3_mul(self.m_t, mat3_mul(HSR_TO_TBO, f_s));
        let m_hinge = mat3_mul(self.m_t, mat3_mul(HSR_TO_TBO, m_s));
        self.force_body = f_body;
        self.moment_body = add(cross(hub_body_ft, f_body), m_hinge);
    }

    /// Shear velocity at the hub, in the control axes (SH79 eqns 19-22).
    fn hub_vel_body2ca(&mut self, uvw: Vec3, pqr: Vec3, a_ic: f64, b_ic: f64) -> Vec3 {
        let v_r = add(uvw, cross(pqr, self.hub_body_ft));
        let v_shaft = mat3_mul(TBO_TO_HSR, mat3_mul(self.inv_transform, v_r));

        self.beta_orient = v_shaft[1].atan2(v_shaft[0]);

        let (sb, cb) = self.beta_orient.sin_cos();
        [
            v_shaft[0] * cb + v_shaft[1] * sb,
            0.0,
            v_shaft[2] - b_ic * v_shaft[0] - a_ic * v_shaft[1],
        ]
    }

    /// Fuselage angular velocity in the control axes (SH79 eqns 30, 31).
    fn fus_angvel_body2ca(&self, pqr: Vec3) -> Vec3 {
        let av_s = mat3_mul(TBO_TO_HSR, mat3_mul(self.inv_transform, pqr));
        let (sb, cb) = self.beta_orient.sin_cos();
        [
            av_s[0] * cb + av_s[1] * sb,
            -av_s[0] * sb + av_s[1] * cb,
            av_s[2],
        ]
    }

    /// Momentum theory with the inflow lag (SH79 eqns 24-27, TA77 eqn 10a).
    /// `uw`/`ww` are the in-plane and axial hub velocities; `flow_scale` is the
    /// ground-effect factor.
    fn calc_flow_and_thrust(&mut self, theta_0: f64, uw: f64, ww: f64, flow_scale: f64, dt: f64) {
        let omega_r = self.omega * self.radius_ft;

        self.mu = uw / omega_r;
        if self.mu > 0.7 {
            self.mu = 0.7;
        }
        let mu2 = self.mu * self.mu;

        let ct_t0 = (self.b_pow[3] / 3.0 + 0.5 * self.spec.tip_loss_factor * mu2
            - 4.0 / (9.0 * std::f64::consts::PI) * self.mu * mu2)
            * theta_0;
        let ct_t1 = (0.25 * self.b_pow[4] + 0.25 * self.b_pow[2] * mu2) * self.spec.twist_rad;

        let mut ct_l = (0.5 * self.b_pow[2] + 0.25 * mu2) * self.lambda;

        let mut c0 = 0.5 * self.spec.lift_curve_slope * (ct_l + ct_t0 + ct_t1) * self.solidity;
        c0 /= 2.0 * (self.mu * self.mu + self.lambda * self.lambda).sqrt() + 1e-15;

        // dnu/dt = (1/tau)(Ct/(2·sqrt(mu²+λ²)) − ν), integrated with mu, lambda
        // held constant over the step (the replacement for eqn 26).
        self.nu = flow_scale * ((self.nu - c0) * (-dt / self.spec.inflow_lag_s).exp() + c0);

        self.lambda = ww / omega_r - self.nu; // eqn 25

        ct_l = (0.5 * self.b_pow[2] + 0.25 * mu2) * self.lambda;

        let ct_over_sigma = 0.5 * self.spec.lift_curve_slope * (ct_l + ct_t0 + ct_t1); // eqn 27

        self.thrust = self.spec.num_blades as f64
            * self.spec.chord_ft
            * self.radius_ft
            * self.rho
            * omega_r
            * omega_r
            * ct_over_sigma;
        self.ct = ct_over_sigma * self.solidity;
        self.v_induced = self.nu * omega_r;
    }

    /// Coning angle (SH79 eqn 29).
    fn calc_coning_angle(&mut self, theta_0: f64) {
        let lock_gamma = self.lock_number_by_rho * self.rho;
        let a0_l = (1.0 / 6.0 + 0.04 * self.mu * self.mu * self.mu) * self.lambda;
        let a0_t0 = (1.0 / 8.0 + 0.125 * self.mu * self.mu) * theta_0;
        let a0_t1 = (1.0 / 10.0 + self.mu * self.mu / 12.0) * self.spec.twist_rad;
        self.a0 = lock_gamma * (a0_l + a0_t0 + a0_t1);
    }

    /// Longitudinal/lateral flapping (SH79 eqn 32).
    fn calc_flapping_angles(&mut self, theta_0: f64, pqr_w: Vec3) {
        let lock_gamma = self.lock_number_by_rho * self.rho;
        let mu2_2 = self.mu * self.mu / 2.0;
        let t075 = theta_0 + 0.75 * self.spec.twist_rad;

        self.a1 = 1.0 / (1.0 - mu2_2)
            * ((2.0 * self.lambda + (8.0 / 3.0) * t075) * self.mu + pqr_w[0] / self.omega
                - 16.0 * pqr_w[1] / (lock_gamma * self.omega));

        self.b1 = 1.0 / (1.0 + mu2_2)
            * ((4.0 / 3.0) * self.mu * self.a0
                - pqr_w[1] / self.omega
                - 16.0 * pqr_w[0] / (lock_gamma * self.omega));

        // The last term divides by C_T/sigma, used verbatim (as in JSBSim): it
        // stays O(1) through normal flight and would only misbehave if the rotor
        // were at zero thrust.
        let ct_over_sigma = self.ct / self.solidity;
        self.a_dw = 1.0 / (1.0 - mu2_2)
            * ((2.0 * self.lambda + (8.0 / 3.0) * t075) * self.mu
                - 24.0 * pqr_w[1] / (lock_gamma * self.omega)
                    * (1.0 - 0.29 * t075 / ct_over_sigma));
    }

    /// In-plane hub forces H (drag) and J (side) (SH79 eqns 38, 39).
    fn calc_drag_and_side_forces(&mut self, theta_0: f64) {
        let t075 = theta_0 + 0.75 * self.spec.twist_rad;
        self.h_drag = self.thrust * self.a_dw;

        let mu = self.mu;
        let cy_over_sigma = 0.75 * self.b1 * self.lambda - 1.5 * self.a0 * mu * self.lambda
            + 0.25 * self.a1 * self.b1 * mu
            - self.a0 * self.a1 * mu * mu
            + (1.0 / 6.0) * self.a0 * self.a1
            - (0.75 * mu * self.a0 - (1.0 / 3.0) * self.b1 - 0.5 * mu * mu * self.b1) * t075;
        let cy_over_sigma = cy_over_sigma * self.spec.lift_curve_slope / 2.0;

        let omega_r = self.omega * self.radius_ft;
        self.j_side = self.spec.num_blades as f64
            * self.spec.chord_ft
            * self.radius_ft
            * self.rho
            * omega_r
            * omega_r
            * cy_over_sigma;
    }

    /// Rotor torque (simplified SH79 eqn 36; Bramwell eqns 3.43, 3.44).
    fn calc_torque(&mut self) {
        let delta_dr =
            0.009 + 0.3 * (6.0 * self.ct / (self.spec.lift_curve_slope * self.solidity)).powi(2);
        let omega_r = self.omega * self.radius_ft;
        self.torque = self.rho
            * self.spec.num_blades as f64
            * self.spec.chord_ft
            * delta_dr
            * omega_r
            * omega_r
            * self.r_pow[2]
            * (1.0 + 4.5 * self.mu * self.mu)
            / 8.0
            - (self.thrust * self.lambda + self.h_drag * self.mu) * self.radius_ft;
    }

    /// Rotor force in the control axes (SH79 eqns 40, 41).
    fn body_forces(&self, a_ic: f64, b_ic: f64) -> Vec3 {
        let (sb, cb) = self.beta_orient.sin_cos();
        [
            -self.h_drag * cb - self.j_side * sb + self.thrust * b_ic,
            -self.h_drag * sb + self.j_side * cb + self.thrust * a_ic,
            -self.thrust,
        ]
    }

    /// Hub moments (hinge offset + torque) in the control axes (SH79 eqn 43).
    fn body_moments(&mut self, a_ic: f64, b_ic: f64) -> Vec3 {
        let (sb, cb) = self.beta_orient.sin_cos();
        self.a1s = self.a1 * cb + self.b1 * sb - b_ic;
        self.b1s = self.b1 * cb - self.a1 * sb + a_ic;

        let mf = 0.5
            * self.spec.hinge_offset_ft
            * self.spec.num_blades as f64
            * self.omega
            * self.omega
            * self.spec.mass_moment;
        [mf * self.b1s, mf * self.a1s, self.torque * self.spec.sense]
    }

    pub fn force_body(&self) -> Vec3 {
        self.force_body
    }

    pub fn moment_body(&self) -> Vec3 {
        self.moment_body
    }

    pub fn thrust(&self) -> f64 {
        self.thrust
    }

    pub fn torque(&self) -> f64 {
        self.torque
    }

    pub fn rpm(&self) -> f64 {
        self.rpm
    }

    pub fn coning_angle(&self) -> f64 {
        self.a0
    }

    pub fn longitudinal_flap(&self) -> f64 {
        self.a1
    }

    pub fn lateral_flap(&self) -> f64 {
        self.b1
    }

    pub fn thrust_coefficient(&self) -> f64 {
        self.ct
    }

    pub fn induced_velocity(&self) -> f64 {
        self.v_induced
    }

    pub fn inflow_ratio(&self) -> f64 {
        self.nu
    }

    /// Advance ratio `mu = V_perp / (Omega·R)` (JSBSim's
    /// `propulsion/engine/advance-ratio`, aliased as `aero/mu`).
    pub fn advance_ratio(&self) -> f64 {
        self.mu
    }
}

/// Build the custom transform `mT` from JSBSim's `UpdateCustomTransformMatrix`,
/// given `(roll, pitch, yaw)` in degrees.
fn orient_matrix(orient_deg: Vec3) -> Mat3 {
    let (sr, cr) = (orient_deg[0] * D2R).sin_cos();
    let (sp, cp) = (orient_deg[1] * D2R).sin_cos();
    let (sy, cy) = (orient_deg[2] * D2R).sin_cos();
    let srsp = sr * sp;
    let crcy = cr * cy;
    let crsy = cr * sy;
    [
        [cp * cy, srsp * cy - crsy, crcy * sp + sr * sy],
        [cp * sy, srsp * sy + crcy, crsy * sp - sr * cy],
        [-sp, sr * cp, cr * cp],
    ]
}

fn transpose(m: Mat3) -> Mat3 {
    [
        [m[0][0], m[1][0], m[2][0]],
        [m[0][1], m[1][1], m[2][1]],
        [m[0][2], m[1][2], m[2][2]],
    ]
}
