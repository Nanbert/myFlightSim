//! Mode-based Boeing 737 autopilot: MCP setpoints plus HDG SEL, ALT HOLD, V/S
//! and LVL CHG modes, with an independent autothrottle (SPD or N1).
//!
//! The laws drive the same normalized `fcs/*-cmd-norm` channels the pilot's
//! keys do, through the backend-neutral `FlightModel` property bus, so the
//! autopilot works on both the JSBSim and pure-Rust FDM backends.
//!
//! Mode semantics follow the real 737 closely enough to teach the panel:
//! lateral/vertical buttons only select a mode while the A/P is engaged;
//! ALT HOLD captures the altitude at selection; V/S and LVL CHG capture the
//! MCP altitude window on reaching it; LVL CHG flies speed on pitch with the
//! autothrottle in N1, handing over to SPD when the altitude is captured.

use bevy::prelude::*;
use flight_model::FlightModel;

use crate::aircraft_profile::{self, ToggleAction};
use crate::app_state::AppState;
use crate::sim::Sim;

const R2D: f64 = 180.0 / std::f64::consts::PI;

// MCP nudge steps and repeat timing.
const HDG_STEP_DEG: f64 = 5.0;
const ALT_STEP_FT: f64 = 1000.0;
const VS_STEP_FPM: f64 = 100.0;
const IAS_STEP_KT: f64 = 5.0;
const MACH_STEP: f64 = 0.01;
const REPEAT_DELAY_S: f64 = 0.40;
const REPEAT_INTERVAL_S: f64 = 0.05;

// Lateral law: heading error -> bank command -> aileron.
const HDG_TO_BANK_GAIN: f64 = 1.5;
const MAX_BANK_DEG: f64 = 25.0;
const ROLL_KP: f64 = 0.08;
const ROLL_KD: f64 = 0.02;

// Vertical law: altitude/V-S/speed -> pitch command -> elevator. Both outer
// loops integrate: a proportional-only loop holds the demanded climb only by
// keeping a steady error, which is exactly the offset a real autopilot trims
// out.
const ALT_VS_GAIN: f64 = 3.0;
const VS_COMMAND_MAX_FPM: f64 = 1500.0;
const VS_TO_PITCH_GAIN: f64 = 0.012;
const VS_PITCH_KI: f64 = 0.002;
const SPD_TO_PITCH_GAIN: f64 = 0.35;
const SPD_PITCH_KI: f64 = 0.05;
const PITCH_INTEGRAL_LIMIT_DEG: f64 = 8.0;
const PITCH_CMD_MAX_DEG: f64 = 12.0;
const PITCH_KP: f64 = 0.02;
const PITCH_KD: f64 = 0.01;
/// Inner-loop elevator trim. The 737's elevator is weak enough that the
/// proportional term alone cannot hold the commanded attitude under thrust.
const PITCH_KI: f64 = 0.004;
const PITCH_INT_LIMIT: f64 = 0.6;
const ALT_CAPTURE_FT: f64 = 150.0;

// Autothrottle.
const AT_BASE: f64 = 0.55;
const AT_KP: f64 = 0.015;
const AT_KI: f64 = 0.010;
const AT_INTEGRAL_LIMIT: f64 = 60.0;
const N1_DEADBAND_FT: f64 = 100.0;
/// A Mach error is scaled into "equivalent knots" so the SPD loop can use one
/// set of gains for both units (0.01 Mach ≈ 3 kt-equivalent).
const MACH_TO_KT_EQUIV: f64 = 300.0;

/// How long the AP DISC annunciation stays up after a disconnect.
const DISC_ANNUN_S: f64 = 2.0;

/// Which unit the MCP speed window is showing.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SpeedUnit {
    Ias,
    Mach,
}

/// The four setpoint windows of the Mode Control Panel.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum McpWindow {
    Speed,
    Heading,
    Altitude,
    VerticalSpeed,
}

/// Mode Control Panel setpoints.
#[derive(Resource, Debug)]
pub struct Mcp {
    pub hdg_deg: f64,
    pub alt_ft: f64,
    pub vs_fpm: f64,
    pub ias_kts: f64,
    pub mach: f64,
    pub speed_unit: SpeedUnit,
}

impl Default for Mcp {
    fn default() -> Self {
        Mcp {
            hdg_deg: 0.0,
            alt_ft: 10_000.0,
            vs_fpm: 0.0,
            ias_kts: 250.0,
            mach: 0.78,
            speed_unit: SpeedUnit::Ias,
        }
    }
}

impl Mcp {
    pub fn reset(&mut self) {
        *self = Self::default();
    }

    /// Heading as shown on the MCP: north reads 360, not 000.
    pub fn heading_display(&self) -> f64 {
        let h = self.hdg_deg.rem_euclid(360.0);
        if h < 0.5 {
            360.0
        } else {
            h
        }
    }

    pub fn toggle_speed_unit(&mut self) {
        self.speed_unit = match self.speed_unit {
            SpeedUnit::Ias => SpeedUnit::Mach,
            SpeedUnit::Mach => SpeedUnit::Ias,
        };
    }

    /// One detent on a window. `fine` is the inner knob (1°/100 ft/1 kt); the
    /// outer knob uses the same steps as the keyboard shortcuts.
    pub fn nudge(&mut self, window: McpWindow, dir: f64, fine: bool) {
        match window {
            McpWindow::Heading => {
                let step = if fine { 1.0 } else { HDG_STEP_DEG };
                self.hdg_deg = (self.hdg_deg + dir * step).rem_euclid(360.0);
            }
            McpWindow::Altitude => {
                let step = if fine { ALT_STEP_FT / 10.0 } else { ALT_STEP_FT };
                self.alt_ft = (self.alt_ft + dir * step).clamp(0.0, 50_000.0);
            }
            McpWindow::VerticalSpeed => {
                self.vs_fpm = (self.vs_fpm + dir * VS_STEP_FPM).clamp(-6000.0, 6000.0);
            }
            McpWindow::Speed => match self.speed_unit {
                SpeedUnit::Ias => {
                    let step = if fine { 1.0 } else { IAS_STEP_KT };
                    self.ias_kts = (self.ias_kts + dir * step).clamp(100.0, 340.0);
                }
                SpeedUnit::Mach => {
                    self.mach = (self.mach + dir * MACH_STEP).clamp(0.50, 0.90);
                }
            },
        }
    }

    /// Speed error in kt-equivalent, in whichever unit the window shows.
    fn speed_error(&self, s: &Sensors) -> f64 {
        match self.speed_unit {
            SpeedUnit::Ias => self.ias_kts - s.ias_kts,
            SpeedUnit::Mach => (self.mach - s.mach) * MACH_TO_KT_EQUIV,
        }
    }
}

/// Lateral modes.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LatMode {
    Off,
    HdgSel,
}

/// Vertical modes.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum VertMode {
    Off,
    AltHold,
    Vs,
    LvlChg,
}

/// Autothrottle modes.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AtMode {
    Off,
    Speed,
    N1,
}

/// The engaged autopilot: mode state, captured altitude, and loop memory.
#[derive(Resource, Debug)]
pub struct Autopilot {
    pub engaged: bool,
    pub lat: LatMode,
    pub vert: VertMode,
    pub at: AtMode,
    /// The altitude ALT HOLD is holding (captured on engagement or selection).
    pub alt_hold_ft: f64,
    at_integral: f64,
    /// Pitch-command integrators for the vertical-speed and speed-on-pitch
    /// modes (degrees of pitch).
    vs_int: f64,
    spd_int: f64,
    /// Elevator trim integrated by the inner pitch loop (normalized command).
    pitch_int: f64,
    disc_timer: f64,
    /// Attitude the flight director is commanding, for the PFD's magenta bars.
    pub fd_pitch_deg: f64,
    pub fd_bank_deg: f64,
}

impl Default for Autopilot {
    fn default() -> Self {
        Autopilot {
            engaged: false,
            lat: LatMode::Off,
            vert: VertMode::Off,
            at: AtMode::Off,
            alt_hold_ft: 0.0,
            at_integral: 0.0,
            vs_int: 0.0,
            spd_int: 0.0,
            pitch_int: 0.0,
            disc_timer: 0.0,
            fd_pitch_deg: 0.0,
            fd_bank_deg: 0.0,
        }
    }
}

impl Autopilot {
    pub fn reset(&mut self) {
        *self = Self::default();
    }

    /// A/P master switch. Engagement is airborne-only and captures the current
    /// altitude; disengagement clears every mode and the autothrottle.
    pub fn toggle(&mut self, model: &dyn FlightModel) {
        if self.engaged {
            self.disengage();
            return;
        }
        if model.get("gear/wow") > 0.5 {
            return;
        }
        self.engaged = true;
        self.lat = LatMode::HdgSel;
        self.vert = VertMode::AltHold;
        self.alt_hold_ft = model.get("position/h-sl-ft");
    }

    /// Drop the A/P (and the A/T, so the pilot has the throttles) and flash
    /// AP DISC.
    pub fn disengage(&mut self) {
        if self.engaged {
            self.disc_timer = DISC_ANNUN_S;
        }
        self.engaged = false;
        self.lat = LatMode::Off;
        self.vert = VertMode::Off;
        self.at = AtMode::Off;
        self.at_integral = 0.0;
        self.vs_int = 0.0;
        self.spd_int = 0.0;
        self.pitch_int = 0.0;
        self.fd_pitch_deg = 0.0;
        self.fd_bank_deg = 0.0;
    }

    /// Whether the amber AP DISC annunciation is still showing.
    pub fn disc_annunciated(&self) -> bool {
        self.disc_timer > 0.0
    }

    /// A mode button. Lateral/vertical modes only select while the A/P is
    /// engaged; the autothrottle is independent and can be used hand-flying.
    pub fn press_mode(&mut self, action: ToggleAction, model: &dyn FlightModel) {
        // A mode change starts its pitch loop from a clean integrator.
        self.vs_int = 0.0;
        self.spd_int = 0.0;
        self.pitch_int = 0.0;
        match action {
            ToggleAction::HdgSel if self.engaged => self.lat = LatMode::HdgSel,
            ToggleAction::AltHold if self.engaged => {
                self.vert = VertMode::AltHold;
                self.alt_hold_ft = model.get("position/h-sl-ft");
                if self.at == AtMode::N1 {
                    self.at = AtMode::Speed;
                }
            }
            ToggleAction::VsMode if self.engaged => self.vert = VertMode::Vs,
            ToggleAction::LvlChg if self.engaged => {
                self.vert = VertMode::LvlChg;
                self.at = AtMode::N1;
                self.at_integral = 0.0;
            }
            ToggleAction::AutoThrottle => {
                self.at = match self.at {
                    AtMode::Off => AtMode::Speed,
                    _ => AtMode::Off,
                };
                self.at_integral = 0.0;
            }
            _ => {}
        }
    }

    /// The pilot moved the throttles: drop the A/T without touching the A/P.
    pub fn disconnect_autothrottle(&mut self) {
        self.at = AtMode::Off;
        self.at_integral = 0.0;
    }

    /// Flight-mode annunciation, e.g. `CMD  HDG SEL  ALT HOLD  A/T SPD`.
    pub fn fma(&self) -> String {
        if self.disc_timer > 0.0 {
            return "AP DISC".to_owned();
        }
        let at = match self.at {
            AtMode::Off => "",
            AtMode::Speed => "A/T SPD",
            AtMode::N1 => "A/T N1",
        };
        if !self.engaged {
            return if at.is_empty() {
                "MANUAL".to_owned()
            } else {
                at.to_owned()
            };
        }
        let lat = match self.lat {
            LatMode::Off => "---",
            LatMode::HdgSel => "HDG SEL",
        };
        let vert = match self.vert {
            VertMode::Off => "---",
            VertMode::AltHold => "ALT HOLD",
            VertMode::Vs => "V/S",
            VertMode::LvlChg => "LVL CHG",
        };
        if at.is_empty() {
            format!("CMD  {lat}  {vert}")
        } else {
            format!("CMD  {lat}  {vert}  {at}")
        }
    }
}

/// The flight condition the loops read each step.
struct Sensors {
    alt_ft: f64,
    vs_fpm: f64,
    ias_kts: f64,
    mach: f64,
    theta_deg: f64,
    phi_deg: f64,
    psi_deg: f64,
    p_dps: f64,
    q_dps: f64,
}

impl Sensors {
    fn read(m: &dyn FlightModel) -> Self {
        Sensors {
            alt_ft: m.get("position/h-sl-ft"),
            vs_fpm: m.get("velocities/h-dot-fps") * 60.0,
            ias_kts: m.get("velocities/vc-kts"),
            mach: m.get("velocities/mach"),
            theta_deg: m.get("attitude/theta-deg"),
            phi_deg: m.get("attitude/phi-deg"),
            psi_deg: m.get("attitude/psi-deg"),
            p_dps: m.get("velocities/p-rad_sec") * R2D,
            q_dps: m.get("velocities/q-rad_sec") * R2D,
        }
    }
}

/// Advance the autopilot one FDM step. Writes throttle when the A/T is on, and
/// elevator/aileron when the A/P is engaged. Everything else (flaps, gear,
/// trim, rudder) stays with the pilot.
pub fn advance(model: &mut dyn FlightModel, mcp: &Mcp, ap: &mut Autopilot, dt: f64) {
    if ap.disc_timer > 0.0 {
        ap.disc_timer = (ap.disc_timer - dt).max(0.0);
    }

    let s = Sensors::read(model);

    if ap.at != AtMode::Off {
        let throttle = match ap.at {
            AtMode::Speed => speed_throttle(mcp.speed_error(&s), &mut ap.at_integral, dt),
            AtMode::N1 => n1_throttle(mcp.alt_ft, s.alt_ft),
            AtMode::Off => unreachable!(),
        };
        model.set("fcs/throttle-cmd-norm", throttle);
    } else {
        ap.at_integral = 0.0;
    }

    if !ap.engaged {
        return;
    }

    // ALT ACQ: on reaching the MCP window, V/S and LVL CHG hand over to ALT
    // HOLD. LVL CHG also hands the autothrottle from N1 to SPD.
    if matches!(ap.vert, VertMode::Vs | VertMode::LvlChg)
        && alt_capture(&s, mcp.alt_ft)
    {
        ap.vert = VertMode::AltHold;
        ap.alt_hold_ft = mcp.alt_ft;
        if ap.at == AtMode::N1 {
            ap.at = AtMode::Speed;
            ap.at_integral = 0.0;
        }
    }

    let (aileron, fd_bank) = match ap.lat {
        LatMode::HdgSel => heading_aileron(mcp.hdg_deg, &s),
        LatMode::Off => (0.0, 0.0),
    };

    let theta_cmd = match ap.vert {
        VertMode::AltHold => {
            let vs_cmd = (ALT_VS_GAIN * (ap.alt_hold_ft - s.alt_ft))
                .clamp(-VS_COMMAND_MAX_FPM, VS_COMMAND_MAX_FPM);
            ap.spd_int = 0.0;
            vs_pitch(vs_cmd, &s, &mut ap.vs_int, dt)
        }
        VertMode::Vs => {
            ap.spd_int = 0.0;
            vs_pitch(mcp.vs_fpm, &s, &mut ap.vs_int, dt)
        }
        // LVL CHG: too slow -> nose down, too fast -> nose up.
        VertMode::LvlChg => {
            ap.vs_int = 0.0;
            let err = mcp.speed_error(&s);
            let p = -err * SPD_TO_PITCH_GAIN;
            integrate(&mut ap.spd_int, -SPD_PITCH_KI * err, p, dt);
            (p + ap.spd_int).clamp(-PITCH_CMD_MAX_DEG, PITCH_CMD_MAX_DEG)
        }
        VertMode::Off => {
            ap.vs_int = 0.0;
            ap.spd_int = 0.0;
            ap.pitch_int = 0.0;
            s.theta_deg
        }
    };

    ap.fd_pitch_deg = theta_cmd;
    ap.fd_bank_deg = fd_bank;

    let theta_err = theta_cmd - s.theta_deg;
    let p = -PITCH_KP * theta_err;
    let saturated =
        (p + ap.pitch_int >= 1.0 && theta_err < 0.0) || (p + ap.pitch_int <= -1.0 && theta_err > 0.0);
    if !saturated {
        ap.pitch_int = (ap.pitch_int - PITCH_KI * theta_err * dt)
            .clamp(-PITCH_INT_LIMIT, PITCH_INT_LIMIT);
    }
    let elevator = (p + ap.pitch_int + PITCH_KD * s.q_dps).clamp(-1.0, 1.0);

    model.set("fcs/aileron-cmd-norm", aileron);
    model.set("fcs/elevator-cmd-norm", elevator);
}

/// Vertical-speed -> pitch command, with integral trim for the demanded rate.
/// The integrator freezes while the command is saturated, so a large initial
/// error cannot wind it up into an overshoot.
fn vs_pitch(vs_cmd: f64, s: &Sensors, integral: &mut f64, dt: f64) -> f64 {
    let err = vs_cmd - s.vs_fpm;
    let p = VS_TO_PITCH_GAIN * err;
    integrate(&mut *integral, VS_PITCH_KI * err, p, dt);
    (p + *integral).clamp(-PITCH_CMD_MAX_DEG, PITCH_CMD_MAX_DEG)
}

/// Clamped integrator with conditional integration (anti-windup): stop
/// accumulating while `p + integral` is saturated and the error pushes further.
fn integrate(integral: &mut f64, rate: f64, p: f64, dt: f64) {
    let saturated = (p + *integral >= PITCH_CMD_MAX_DEG && rate > 0.0)
        || (p + *integral <= -PITCH_CMD_MAX_DEG && rate < 0.0);
    if !saturated {
        *integral = (*integral + rate * dt)
            .clamp(-PITCH_INTEGRAL_LIMIT_DEG, PITCH_INTEGRAL_LIMIT_DEG);
    }
}

/// Bank onto the selected heading, with a 25° bank limit and roll-rate damping.
/// Returns the aileron command and the bank the flight director is asking for.
fn heading_aileron(hdg_sel: f64, s: &Sensors) -> (f64, f64) {
    let err = wrap180(hdg_sel - s.psi_deg);
    let bank_cmd = (HDG_TO_BANK_GAIN * err).clamp(-MAX_BANK_DEG, MAX_BANK_DEG);
    (
        (ROLL_KP * (bank_cmd - s.phi_deg) - ROLL_KD * s.p_dps).clamp(-1.0, 1.0),
        bank_cmd,
    )
}

/// ALT ACQ: inside the capture window and not moving away from it.
fn alt_capture(s: &Sensors, alt_sel: f64) -> bool {
    let err = alt_sel - s.alt_ft;
    if err.abs() > ALT_CAPTURE_FT {
        return false;
    }
    err * s.vs_fpm >= 0.0 || s.vs_fpm.abs() < 50.0
}

/// Speed-on-throttle PI loop, error in kt-equivalent.
fn speed_throttle(speed_err: f64, integral: &mut f64, dt: f64) -> f64 {
    *integral = (*integral + speed_err * dt).clamp(-AT_INTEGRAL_LIMIT, AT_INTEGRAL_LIMIT);
    (AT_BASE + AT_KP * speed_err + AT_KI * *integral).clamp(0.0, 1.0)
}

/// LVL CHG thrust: climb thrust to go up, idle to go down.
fn n1_throttle(alt_sel: f64, alt: f64) -> f64 {
    if alt_sel > alt + N1_DEADBAND_FT {
        1.0
    } else if alt_sel < alt - N1_DEADBAND_FT {
        0.0
    } else {
        0.5
    }
}

/// Wrap an angle in degrees to (-180, 180].
fn wrap180(mut d: f64) -> f64 {
    d = (d + 180.0).rem_euclid(360.0) - 180.0;
    if d <= -180.0 {
        d + 360.0
    } else {
        d
    }
}

// --- MCP key input ----------------------------------------------------------

/// Per-knob repeat timers (seconds until the next nudge while a key is held).
#[derive(Resource, Default)]
struct McpRepeat {
    hdg: f64,
    alt: f64,
    vs: f64,
    speed: f64,
}

/// Arrow keys nudge the MCP when they are held: one step immediately, then a
/// fast repeat. Only aircraft with an autopilot show/use these.
fn mcp_input(
    keys: Res<ButtonInput<KeyCode>>,
    time: Res<Time>,
    sim: Res<Sim>,
    mut mcp: ResMut<Mcp>,
    mut repeat: ResMut<McpRepeat>,
) {
    if !aircraft_profile::profile(&sim.name).has_autopilot {
        return;
    }
    let dt = time.delta_secs_f64();

    let hdg = held_step(
        &keys,
        KeyCode::ArrowRight,
        KeyCode::ArrowLeft,
        &mut repeat.hdg,
        dt,
    );
    if hdg != 0.0 {
        mcp.nudge(McpWindow::Heading, hdg, false);
    }

    let alt = held_step(
        &keys,
        KeyCode::ArrowUp,
        KeyCode::ArrowDown,
        &mut repeat.alt,
        dt,
    );
    if alt != 0.0 {
        mcp.nudge(McpWindow::Altitude, alt, false);
    }

    let vs = held_step(
        &keys,
        KeyCode::PageUp,
        KeyCode::PageDown,
        &mut repeat.vs,
        dt,
    );
    if vs != 0.0 {
        mcp.nudge(McpWindow::VerticalSpeed, vs, false);
    }

    let speed = held_step(
        &keys,
        KeyCode::BracketRight,
        KeyCode::BracketLeft,
        &mut repeat.speed,
        dt,
    );
    if speed != 0.0 {
        mcp.nudge(McpWindow::Speed, speed, false);
    }
}

/// +1/-1 one step per press and then every repeat interval while held.
fn held_step(
    keys: &ButtonInput<KeyCode>,
    pos: KeyCode,
    neg: KeyCode,
    timer: &mut f64,
    dt: f64,
) -> f64 {
    let dir = keys.pressed(pos) as i32 - keys.pressed(neg) as i32;
    if dir == 0 {
        *timer = 0.0;
        return 0.0;
    }
    let fire = if keys.just_pressed(pos) || keys.just_pressed(neg) {
        *timer = REPEAT_DELAY_S;
        true
    } else {
        *timer -= dt;
        if *timer <= 0.0 {
            *timer += REPEAT_INTERVAL_S;
            true
        } else {
            false
        }
    };
    if fire {
        dir as f64
    } else {
        0.0
    }
}

pub struct AutopilotPlugin;

impl Plugin for AutopilotPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<Mcp>()
            .init_resource::<Autopilot>()
            .init_resource::<McpRepeat>()
            .add_systems(Update, mcp_input.run_if(in_state(AppState::Flying)));
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sim::DT;
    use flight_model::FlightModel;

    fn rust_737(alt: f64, kt: f64) -> Box<dyn FlightModel> {
        let mut m: Box<dyn FlightModel> = Box::new(fdm::Fdm::new("737"));
        setup(&mut *m, alt, kt);
        m
    }

    fn jsbsim_737(alt: f64, kt: f64) -> Box<dyn FlightModel> {
        let mut m: Box<dyn FlightModel> = Box::new(jsbsim_sys::Fdm::load("737", DT));
        setup(&mut *m, alt, kt);
        m
    }

    /// Same airborne start the game uses, minus the sea-level origin bookkeeping.
    fn setup(m: &mut dyn FlightModel, alt: f64, kt: f64) {
        m.set("ic/vc-kts", kt);
        m.set("ic/h-sl-ft", alt);
        m.set("ic/psi-true-deg", 0.0);
        m.apply_ic();
        m.set("gear/gear-cmd-norm", 0.0);
        m.set("gear/gear-pos-norm", 0.0);
        m.set("propulsion/set-running", -1.0);
        m.trim(kt, alt);
    }

    fn run(m: &mut dyn FlightModel, mcp: &Mcp, ap: &mut Autopilot, secs: f64) {
        for _ in 0..(secs / DT).round() as usize {
            advance(m, mcp, ap, DT);
            m.step();
        }
    }

    #[test]
    fn mcp_knobs_move_in_expected_steps_and_clamp() {
        let mut mcp = Mcp::default();
        mcp.nudge(McpWindow::Heading, 1.0, false);
        assert_eq!(mcp.hdg_deg, 5.0);
        mcp.nudge(McpWindow::Heading, -1.0, true);
        assert_eq!(mcp.hdg_deg, 4.0);
        mcp.nudge(McpWindow::Heading, -5.0, false);
        assert_eq!(mcp.hdg_deg, 339.0);

        mcp.nudge(McpWindow::Altitude, -1.0, false);
        assert_eq!(mcp.alt_ft, 9_000.0);
        mcp.nudge(McpWindow::Altitude, 1.0, true);
        assert_eq!(mcp.alt_ft, 9_100.0);
        mcp.alt_ft = 50_000.0;
        mcp.nudge(McpWindow::Altitude, 1.0, false);
        assert_eq!(mcp.alt_ft, 50_000.0);

        mcp.nudge(McpWindow::VerticalSpeed, 1.0, false);
        assert_eq!(mcp.vs_fpm, 100.0);
        mcp.vs_fpm = 6_000.0;
        mcp.nudge(McpWindow::VerticalSpeed, 1.0, true);
        assert_eq!(mcp.vs_fpm, 6_000.0);

        mcp.nudge(McpWindow::Speed, 1.0, false);
        assert_eq!(mcp.ias_kts, 255.0);
        mcp.nudge(McpWindow::Speed, 1.0, true);
        assert_eq!(mcp.ias_kts, 256.0);
        mcp.toggle_speed_unit();
        mcp.nudge(McpWindow::Speed, 1.0, true);
        assert!((mcp.mach - 0.79).abs() < 1e-9);
    }

    #[test]
    fn north_displays_as_360_and_wrapping_holds() {
        let mut mcp = Mcp::default();
        assert_eq!(mcp.heading_display(), 360.0);
        mcp.hdg_deg = 359.0;
        mcp.nudge(McpWindow::Heading, 1.0, false);
        assert!((mcp.hdg_deg - 4.0).abs() < 1e-9);
        assert!((wrap180(mcp.hdg_deg) - 4.0).abs() < 1e-9);
    }

    #[test]
    fn mode_buttons_are_inert_until_the_autopilot_is_on() {
        let m = rust_737(10_000.0, 250.0);
        let mut ap = Autopilot::default();
        ap.press_mode(ToggleAction::HdgSel, &*m);
        ap.press_mode(ToggleAction::LvlChg, &*m);
        assert_eq!(ap.lat, LatMode::Off);
        assert_eq!(ap.vert, VertMode::Off);
        assert_eq!(ap.at, AtMode::Off);
        // The autothrottle is independent.
        ap.press_mode(ToggleAction::AutoThrottle, &*m);
        assert_eq!(ap.at, AtMode::Speed);
    }

    #[test]
    fn altitude_hold_holds_the_captured_altitude() {
        let mut m = rust_737(10_000.0, 250.0);
        let mcp = Mcp::default();
        let mut ap = Autopilot::default();
        ap.toggle(&*m);
        assert!(ap.engaged);
        assert_eq!(ap.vert, VertMode::AltHold);
        assert!((ap.alt_hold_ft - 10_000.0).abs() < 50.0);
        ap.press_mode(ToggleAction::AutoThrottle, &*m);

        run(&mut *m, &mcp, &mut ap, 40.0);
        let h0 = m.get("position/h-sl-ft");
        run(&mut *m, &mcp, &mut ap, 20.0);
        let h1 = m.get("position/h-sl-ft");
        assert!((h0 - 10_000.0).abs() < 250.0, "altitude {h0}");
        assert!((h1 - h0).abs() < 60.0, "drifted {h0} -> {h1}");
    }

    #[test]
    fn heading_select_turns_onto_the_bug_and_levels() {
        let mut m = rust_737(10_000.0, 250.0);
        let mut mcp = Mcp::default();
        mcp.hdg_deg = 90.0;
        let mut ap = Autopilot::default();
        ap.toggle(&*m);
        ap.press_mode(ToggleAction::AutoThrottle, &*m);

        run(&mut *m, &mcp, &mut ap, 90.0);
        let hdg = m.get("attitude/psi-deg");
        assert!(wrap180(hdg - 90.0).abs() < 5.0, "heading {hdg}");
        let bank = m.get("attitude/phi-deg");
        assert!(bank.abs() < 8.0, "bank {bank}");
    }

    #[test]
    fn vs_climbs_and_captures_the_mcp_altitude() {
        let mut m = rust_737(10_000.0, 250.0);
        let mut mcp = Mcp::default();
        mcp.alt_ft = 12_000.0;
        mcp.vs_fpm = 1_000.0;
        let mut ap = Autopilot::default();
        ap.toggle(&*m);
        ap.press_mode(ToggleAction::AutoThrottle, &*m);
        ap.press_mode(ToggleAction::VsMode, &*m);
        assert_eq!(ap.vert, VertMode::Vs);

        run(&mut *m, &mcp, &mut ap, 20.0);
        let vs = m.get("velocities/h-dot-fps") * 60.0;
        assert!((vs - 1_000.0).abs() < 300.0, "VS {vs}");

        run(&mut *m, &mcp, &mut ap, 140.0);
        let alt = m.get("position/h-sl-ft");
        assert!((alt - 12_000.0).abs() < 200.0, "altitude {alt}");
        assert_eq!(ap.vert, VertMode::AltHold, "should capture ALT HOLD");
    }

    #[test]
    fn autothrottle_holds_the_mcp_speed() {
        let mut m = rust_737(10_000.0, 250.0);
        let mut mcp = Mcp::default();
        mcp.ias_kts = 230.0;
        let mut ap = Autopilot::default();
        ap.toggle(&*m);
        ap.press_mode(ToggleAction::AutoThrottle, &*m);

        run(&mut *m, &mcp, &mut ap, 60.0);
        let v = m.get("velocities/vc-kts");
        assert!((v - 230.0).abs() < 10.0, "speed {v}");
    }

    #[test]
    fn level_change_climbs_to_the_window_and_hands_over_to_spd() {
        let mut m = rust_737(10_000.0, 250.0);
        let mut mcp = Mcp::default();
        mcp.alt_ft = 14_000.0;
        mcp.ias_kts = 250.0;
        let mut ap = Autopilot::default();
        ap.toggle(&*m);
        ap.press_mode(ToggleAction::LvlChg, &*m);
        assert_eq!(ap.vert, VertMode::LvlChg);
        assert_eq!(ap.at, AtMode::N1);

        run(&mut *m, &mcp, &mut ap, 200.0);
        let alt = m.get("position/h-sl-ft");
        let v = m.get("velocities/vc-kts");
        assert!((alt - 14_000.0).abs() < 250.0, "altitude {alt}");
        assert!((v - 250.0).abs() < 25.0, "speed {v}");
        assert_eq!(ap.vert, VertMode::AltHold);
        assert_eq!(ap.at, AtMode::Speed, "capture should hand over to SPD");
    }

    #[test]
    fn jsbsim_backend_flies_altitude_and_speed_hold() {
        let mut m = jsbsim_737(10_000.0, 250.0);
        let mcp = Mcp::default();
        let mut ap = Autopilot::default();
        ap.toggle(&*m);
        ap.press_mode(ToggleAction::AutoThrottle, &*m);

        run(&mut *m, &mcp, &mut ap, 60.0);
        let alt = m.get("position/h-sl-ft");
        let v = m.get("velocities/vc-kts");
        assert!((alt - 10_000.0).abs() < 350.0, "altitude {alt}");
        assert!((v - 250.0).abs() < 15.0, "speed {v}");
    }
}
