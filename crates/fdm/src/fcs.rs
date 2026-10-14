//! Control systems — the seam between pilot commands and surface deflections.
//!
//! [`DirectFcs`] is the c172p's command-to-surface mapping (a `summer` of
//! command and trim, scaled onto a degree range, times a rad/deg gain).
//! [`Fbw`] is the F-22's three-axis **rate-command fly-by-wire**: the bare
//! airframe is statically unstable subsonic (`Cma_M = +0.097` at M0.8), so the
//! control law — not the airframe — is what makes it fly. Both produce the same
//! [`Surfaces`] bundle, so the FDM never branches on which is in use.
//!
//! The FBW here is JSBSim's flight_control *structure* (rate command from the
//! stick, error against the sensed body rates, integrated and clipped into a
//! surface) with simplified, fixed gains rather than the model's full LQR
//! gain schedules indexed by airspeed, altitude and alpha. See the fidelity
//! note in the project memory: keep the load-bearing loop, drop the peripheral
//! scheduling.

const R2D: f64 = 180.0 / std::f64::consts::PI;
const D2R: f64 = std::f64::consts::PI / 180.0;

/// Resolved control-surface positions for one step (radians, except `*_deg`
/// and the normalized fields). The c172p fills the elevator/aileron/rudder/flap
/// entries and leaves the rest zero; the F-22 also drives the leading-edge
/// flaps and reports normalized thrust.
#[derive(Clone, Copy, Default)]
pub struct Surfaces {
    pub elevator_rad: f64,
    pub mag_elevator_rad: f64,
    /// Left aileron.
    pub aileron_rad: f64,
    pub right_aileron_rad: f64,
    pub rudder_rad: f64,
    pub flap_deg: f64,
    /// Flap position, normalized 0..1 (F-22 aero).
    pub flap_norm: f64,
    /// Leading-edge flap deflection (rad).
    pub lef_rad: f64,
    pub speedbrake_norm: f64,
    /// Thrust-vectoring nozzle deflection (rad). Driven by the F-22's FBW
    /// (gear-up, when enabled); 0 for direct-FCS aircraft.
    pub tvc_rad: f64,
    /// Thrust normalized by military thrust (F-22 aero).
    pub thrust_norm: f64,
    /// Main-rotor collective pitch (rad). Helicopters only.
    pub collective_rad: f64,
    /// Main-rotor longitudinal cyclic (rad).
    pub longitudinal_cyc_rad: f64,
    /// Main-rotor lateral cyclic (rad).
    pub lateral_cyc_rad: f64,
    /// Tail-rotor collective / antitorque pedal (rad).
    pub antitorque_rad: f64,
}

/// The control system an aircraft's data file selects.
pub enum ControlSystem {
    /// Direct command-to-surface gains (c172p).
    Direct(DirectFcs),
    /// Three-axis rate-command fly-by-wire with the given gains (F-22).
    Fbw(&'static FbwSpec),
    /// Helicopter rotor controls with collective mixing (AH-1S).
    Heli(HeliFcs),
}

impl ControlSystem {
    /// Resolve the pilot commands (and sensed rates) into surface deflections,
    /// advancing the FBW integrators by `dt` when the FBW is active.
    pub fn resolve(&self, fbw: &mut Fbw, inputs: &FcsInputs, dt: f64) -> Surfaces {
        match self {
            ControlSystem::Direct(fcs) => fcs.resolve(inputs),
            ControlSystem::Fbw(spec) => fbw.resolve(spec, inputs, dt),
            ControlSystem::Heli(fcs) => fcs.resolve(inputs),
        }
    }

    /// Max trailing-edge flap deflection (deg), used by the c172p's `apply_ic`.
    pub fn flap_max_deg(&self) -> f64 {
        match self {
            ControlSystem::Direct(fcs) => fcs.flap_max_deg,
            ControlSystem::Fbw(_) => 0.0,
            ControlSystem::Heli(_) => 0.0,
        }
    }
}

/// Direct command -> surface mapping (c172p): each axis is a `summer` of the
/// pilot command and its trim tab, clipped and scaled onto a degree range, then
/// multiplied by a rad/deg gain.
pub struct DirectFcs {
    pub elevator_gain: f64,
    pub elevator_lim_deg: (f64, f64),
    pub aileron_gain: f64,
    pub aileron_lim_deg: (f64, f64),
    pub rudder_gain: f64,
    pub rudder_lim_deg: (f64, f64),
    pub flap_max_deg: f64,
    pub flap_time_s: f64,
}

impl DirectFcs {
    fn resolve(&self, i: &FcsInputs) -> Surfaces {
        let elevator_rad = scale(
            i.elevator_cmd + i.pitch_trim_cmd,
            self.elevator_lim_deg,
        ) * self.elevator_gain;
        let aileron_rad =
            scale(i.aileron_cmd + i.roll_trim_cmd, self.aileron_lim_deg) * self.aileron_gain;
        let rudder_rad =
            scale(i.rudder_cmd + i.yaw_trim_cmd, self.rudder_lim_deg) * self.rudder_gain;
        Surfaces {
            elevator_rad,
            mag_elevator_rad: elevator_rad.abs(),
            aileron_rad,
            right_aileron_rad: -aileron_rad,
            rudder_rad,
            flap_deg: i.flap_pos_deg,
            ..Default::default()
        }
    }
}

/// A 5th-order S-curve (JSBSim's `fcs/misc/collective-cmd-norm-polymod`): the
/// collective input is remapped to give more resolution in the mid range. Knotted
/// every 0.05 of command; interpolated linearly between knots.
pub const COLLECTIVE_CURVE: &[(f64, f64)] = &[
    (0.00, 0.000000),
    (0.05, 0.127377),
    (0.10, 0.218080),
    (0.15, 0.282983),
    (0.20, 0.330560),
    (0.25, 0.367188),
    (0.30, 0.397440),
    (0.35, 0.424393),
    (0.40, 0.449920),
    (0.45, 0.474997),
    (0.50, 0.500000),
    (0.55, 0.525002),
    (0.60, 0.550080),
    (0.65, 0.575608),
    (0.70, 0.602560),
    (0.75, 0.632813),
    (0.80, 0.669440),
    (0.85, 0.717018),
    (0.90, 0.781920),
    (0.95, 0.872623),
    (1.00, 1.000000),
];

/// Helicopter rotor controls — a port of the AH-1S `rotor_control.xml` system.
///
/// Four pilot inputs (collective, longitudinal/lateral cyclic, antitorque pedal)
/// map onto four rotor control angles, each a `input * gain + bias`. The
/// load-bearing part is **collective mixing**: part of the collective command is
/// summed into the lateral and pedal channels so a collective change does not
/// need the pilot to re-trim the other axes (and to give the proper counter
/// moments as torque rises). Without it the machine is nearly unflyable.
#[derive(Clone, Copy)]
pub struct HeliFcs {
    pub collective_gain: f64,
    pub collective_bias: f64,
    /// Input-profile blend, 0 = raw command, 1 = full S-curve. (`fcs/adj/collective-profile`)
    pub collective_profile: f64,
    /// Center sensitivity exponent for cyclic/pedal (`sgn(x)*|x|^n`).
    pub center_sensitivity: f64,

    pub longitudinal_gain: f64,
    pub longitudinal_bias: f64,

    pub lateral_gain: f64,
    pub lateral_bias: f64,
    pub lateral_col_mix_gain: f64,
    pub lateral_col_mix_bias: f64,

    pub pedal_gain: f64,
    pub pedal_bias: f64,
    pub pedal_col_mix_gain: f64,
    pub pedal_col_mix_bias: f64,

    /// Collective profile knots.
    pub collective_curve: &'static [(f64, f64)],
}

impl HeliFcs {
    fn resolve(&self, i: &FcsInputs) -> Surfaces {
        let sens = self.center_sensitivity;

        // --- collective: S-curve-profile, trimmed, gain + bias ---
        let coll_cmd = i.collective_cmd.clamp(-1.0, 1.0);
        let shaped = coll_cmd * (1.0 - self.collective_profile)
            + self.collective_profile * interp(self.collective_curve, coll_cmd);
        let coll_sum = (shaped + i.collective_trim_cmd).clamp(-1.0, 1.0);
        let coll_rad = self.collective_gain * coll_sum + self.collective_bias;
        // `fcs/collective-ctrl-rad` (pre-lag, unclipped) feeds the mixing; the
        // lagged output is clipped into [0, 0.7] for the rotor itself.
        let coll_mix_src = coll_rad;
        let collective_rad = coll_rad.clamp(0.0, 0.7);

        // --- longitudinal cyclic ---
        let long_sum = (exmod(i.elevator_cmd, sens) + i.pitch_trim_cmd).clamp(-1.0, 1.0);
        let longitudinal_cyc_rad =
            (self.longitudinal_gain * long_sum + self.longitudinal_bias).clamp(-0.7, 0.7);

        // --- lateral cyclic (with collective compensation) ---
        let lat_sum = (exmod(i.aileron_cmd, sens) + i.roll_trim_cmd).clamp(-1.0, 1.0);
        let lat_clipped = (self.lateral_gain * lat_sum + self.lateral_bias).clamp(-0.7, 0.7);
        let lateral_cyc_rad =
            lat_clipped + (coll_mix_src * self.lateral_col_mix_gain + self.lateral_col_mix_bias);

        // --- pedal / antitorque (with collective compensation) ---
        let ped_sum = (exmod(i.rudder_cmd, sens) + i.yaw_trim_cmd).clamp(-1.0, 1.0);
        let ped_pre = self.pedal_gain * ped_sum + self.pedal_bias;
        let antitorque_rad =
            ped_pre + (coll_mix_src * self.pedal_col_mix_gain + self.pedal_col_mix_bias);

        Surfaces {
            collective_rad,
            longitudinal_cyc_rad,
            lateral_cyc_rad,
            antitorque_rad,
            ..Default::default()
        }
    }
}

/// `sgn(x)*|x|^sens` — the center-sensitivity shaping (JSBSim `*_cmd_norm_exmod`).
fn exmod(x: f64, sens: f64) -> f64 {
    if x >= 0.0 {
        x.powf(sens)
    } else {
        -(-x).powf(sens)
    }
}

/// Linear interpolation over a `(x, y)` knot table, clamped at the ends.
fn interp(table: &[(f64, f64)], x: f64) -> f64 {
    if x <= table[0].0 {
        return table[0].1;
    }
    let last = table[table.len() - 1];
    if x >= last.0 {
        return last.1;
    }
    for w in table.windows(2) {
        let (x0, y0) = w[0];
        let (x1, y1) = w[1];
        if x <= x1 {
            let t = (x - x0) / (x1 - x0);
            return y0 + t * (y1 - y0);
        }
    }
    last.1
}

/// The fixed gains of an FBW control law. All static data — the running
/// integrators live in [`Fbw`].
pub struct FbwSpec {
    // --- roll (aileron) ---
    /// Full-stick roll-rate command (rad/s).
    pub roll_rate_max: f64,
    pub roll_kp: f64,
    pub roll_ki: f64,
    pub roll_int_limit: f64,
    pub aileron_lim_rad: f64,

    // --- pitch (elevator / stabilator) ---
    /// Rate command per unit stick (rad/s); `+elevator-cmd` commands nose-down.
    pub pitch_rate_max: f64,
    pub pitch_kp: f64,
    pub pitch_ki: f64,
    pub pitch_int_limit: f64,
    pub elevator_lim_rad: f64,
    /// Nose-down / nose-up rate-command clips (rad/s).
    pub pitch_down_lim: f64,
    pub pitch_up_lim: f64,
    /// Angle of attack above which alpha protection engages (rad).
    pub alpha_limit_rad: f64,
    /// Higher alpha limit used while thrust vectoring is active: the nozzles
    /// supply the pitch authority the stabilator loses at high alpha, so the
    /// usable envelope opens up.
    pub alpha_limit_tvc_rad: f64,
    /// Nozzle deflection per unit pitch-rate integral (rad). TVC shares the
    /// pitch channel's integrator, mirroring the model's `fcs/tvc-pitch-cmd`.
    pub tvc_gain: f64,
    /// Nozzle deflection limit (rad); the F-22's is ±0.349 (±20°).
    pub tvc_lim_rad: f64,

    // --- yaw (rudder) ---
    /// Yaw-rate command per unit pedal (rad/s); `+rudder-cmd` yaws nose-left.
    pub yaw_rate_max: f64,
    pub yaw_kp: f64,
    pub yaw_ki: f64,
    pub yaw_int_limit: f64,
    pub rudder_lim_rad: f64,

    // --- leading-edge flaps ---
    /// `deg = alpha_deg*gain + (qbar/P)*dyn_gain + offset`, clipped to [0, lim].
    pub lef_alpha_gain: f64,
    pub lef_dyn_gain: f64,
    pub lef_offset: f64,
    pub lef_lim_rad: f64,
}

impl FbwSpec {
    /// Leading-edge flap deflection (rad) for the current condition — the model's
    /// `fcs/lef-pos` schedule: `deg = 1.38·alpha_deg + (-9.05)·(qbar/P) + 1.45`,
    /// clipped to [0, lim]. Active only with the gear up (`fcs/lef-pos` gates on
    /// `gear/gear-pos-norm == 0`) and below M0.9, so `inhibit` is set on the
    /// ground or whenever the gear is out. Shared by the live loop and the trim
    /// solver so a trim seat is a true fixed point.
    pub fn lef_rad(&self, alpha: f64, qbar: f64, pressure_psf: f64, inhibit: bool, mach: f64) -> f64 {
        if inhibit || mach > 0.9 {
            return 0.0;
        }
        let deg = self.lef_alpha_gain * (alpha * R2D)
            + self.lef_dyn_gain * (qbar / pressure_psf.max(1.0))
            + self.lef_offset;
        (deg * D2R).clamp(0.0, self.lef_lim_rad)
    }
}

/// Running FBW state: one integrator per axis.
pub struct Fbw {
    pub roll_int: f64,
    pub pitch_int: f64,
    pub yaw_int: f64,
}

impl Default for Fbw {
    fn default() -> Self {
        Fbw {
            roll_int: 0.0,
            pitch_int: 0.0,
            yaw_int: 0.0,
        }
    }
}

impl Fbw {
    fn resolve(&mut self, s: &FbwSpec, i: &FcsInputs, dt: f64) -> Surfaces {
        // --- roll: +aileron-cmd rolls right (+p) ---
        let aileron_in = (i.aileron_cmd + i.roll_trim_cmd).clamp(-1.0, 1.0);
        let p_cmd = (s.roll_rate_max * aileron_in).clamp(-4.0, 4.0);
        let e_p = p_cmd - i.p;
        self.roll_int = (self.roll_int + e_p * dt).clamp(-s.roll_int_limit, s.roll_int_limit);
        let aileron_rad = (s.roll_kp * e_p + s.roll_ki * self.roll_int)
            .clamp(-s.aileron_lim_rad, s.aileron_lim_rad);

        // --- pitch: +elevator-cmd commands nose-down (-q) ---
        // With TVC on, the nozzles back up the tail, so alpha may be pushed
        // further before protection engages.
        let alpha_lim = if i.tvc_enable {
            s.alpha_limit_tvc_rad
        } else {
            s.alpha_limit_rad
        };
        let elev_in = (i.elevator_cmd + i.pitch_trim_cmd).clamp(-1.0, 0.6);
        let mut q_cmd = (-s.pitch_rate_max * elev_in).clamp(-s.pitch_down_lim, s.pitch_up_lim);
        if i.alpha > alpha_lim && q_cmd > 0.0 {
            q_cmd = 0.0; // alpha limiter: stop commanding more nose-up
        }
        let e_q = q_cmd - i.q;
        self.pitch_int = (self.pitch_int + e_q * dt).clamp(-s.pitch_int_limit, s.pitch_int_limit);
        let elevator_rad = (-(s.pitch_kp * e_q + s.pitch_ki * self.pitch_int))
            .clamp(-s.elevator_lim_rad, s.elevator_lim_rad);

        // Thrust-vectoring nozzles: same pitch integral as the stabilator, so
        // they add authority in the same sense — crucially at high alpha, where
        // the tail is blanked. Off on the ground (the model's `fcs/tvc-inhibit`).
        let tvc_rad = if i.tvc_enable && !i.wow {
            (-s.tvc_gain * self.pitch_int).clamp(-s.tvc_lim_rad, s.tvc_lim_rad)
        } else {
            0.0
        };

        // --- yaw: +rudder-cmd yaws nose-left (-r) ---
        let rudder_in = (i.rudder_cmd + i.yaw_trim_cmd).clamp(-1.0, 1.0);
        let r_cmd = -s.yaw_rate_max * rudder_in;
        let e_r = r_cmd - i.r;
        self.yaw_int = (self.yaw_int + e_r * dt).clamp(-s.yaw_int_limit, s.yaw_int_limit);
        let rudder_rad = (-(s.yaw_kp * e_r + s.yaw_ki * self.yaw_int))
            .clamp(-s.rudder_lim_rad, s.rudder_lim_rad);

        // --- leading-edge flaps: alpha- and dynamic-pressure-scheduled ---
        let lef_rad = s.lef_rad(i.alpha, i.qbar, i.pressure_psf, i.wow || i.gear_down, i.mach);

        let thrust_norm =
            (i.thrust_lbs / i.engine_count.max(1) as f64 / 26550.0).clamp(0.0, 1.5);

        Surfaces {
            elevator_rad,
            mag_elevator_rad: elevator_rad.abs(),
            aileron_rad,
            right_aileron_rad: -aileron_rad,
            rudder_rad,
            flap_deg: 0.0,
            flap_norm: 0.0,
            lef_rad,
            speedbrake_norm: 0.0,
            tvc_rad,
            thrust_norm,
            ..Default::default()
        }
    }
}

/// Everything the control system reads to resolve a step: the pilot commands,
/// the sensed body rates, and the flight condition (for the FBW's alpha limiter
/// and leading-edge-flap schedule).
pub struct FcsInputs {
    pub p: f64,
    pub q: f64,
    pub r: f64,
    pub alpha: f64,
    pub mach: f64,
    pub qbar: f64,
    pub pressure_psf: f64,
    pub wow: bool,
    /// Gear deployed (any wheel down). The F-22's LEF schedule is inhibited with
    /// the gear out (`fcs/lef-pos` gates on `gear/gear-pos-norm == 0`).
    pub gear_down: bool,
    /// Pilot/gear-gated TVC master switch (see `fcs/tvc-enable`). Only the
    /// F-22's FBW reads it.
    pub tvc_enable: bool,
    pub elevator_cmd: f64,
    pub aileron_cmd: f64,
    pub rudder_cmd: f64,
    pub pitch_trim_cmd: f64,
    pub roll_trim_cmd: f64,
    pub yaw_trim_cmd: f64,
    pub flap_pos_deg: f64,
    pub thrust_lbs: f64,
    pub engine_count: usize,
    /// Collective lever, −1 … 1 (helicopters; unused by fixed-wing).
    pub collective_cmd: f64,
    pub collective_trim_cmd: f64,
}

/// A zero-centered `aerosurface_scale`: a positive command maps onto `lim.1`
/// (max), a negative one onto `lim.0` (min). Input is clamped to [-1, 1].
pub(crate) fn scale(cmd: f64, lim: (f64, f64)) -> f64 {
    let cmd = cmd.clamp(-1.0, 1.0);
    if cmd >= 0.0 {
        cmd * lim.1
    } else {
        -cmd * lim.0
    }
}
