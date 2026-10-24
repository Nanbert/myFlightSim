//! Keyboard -> JSBSim normalized control commands, built from the active
//! aircraft's [`crate::aircraft_profile`]. Each aircraft has its own control
//! set: axis bindings (held keys, ramped) and toggles (edge actions). There is
//! no shared state between aircraft — adding a control is adding a row to a
//! profile, not writing a system here.
//!
//! Sign convention comes from the c172p aero coefficients: `+elevator-cmd-norm`
//! pitches the nose DOWN (`aero/coefficient/Cmde` = -1.1220), `+aileron-cmd-norm`
//! rolls RIGHT (`Clda` = +0.2290) and `+rudder-cmd-norm` yaws LEFT
//! (`Cndr` = -0.0430). Each key is mapped to the sign that gives the intuitive
//! response, so the commands are the negative of the key's "positive" sense:
//! W pitches the nose up, S down; D rolls right, A left; Q yaws left, E right.
//! (This assumes the render is un-mirrored — see `sim::pose`.)

use bevy::prelude::*;

use crate::aircraft_profile::{self, Action, Axis, ToggleAction, ToggleSpec};
use crate::app_state::unpaused;
use crate::autopilot::{Autopilot, Mcp};
use crate::config::GameConfig;

/// How fast the arcade throttle spools up while its key is held, in normalized
/// units per second. Faster than the realistic lever (0.6/s) so a tap of the
/// boost key snaps the burner to full.
const ARCADE_SPOOL_RATE: f64 = 2.5;

/// Summed pitch + yaw body rate (rad/s) that counts as a full-drain manoeuvre
/// for the arcade throttle bleed — roughly the arcade envelope's own rate caps.
const ARCADE_BLEED_FULL: f64 = 1.2;

/// How fast a full-drain manoeuvre pulls the arcade throttle down, in normalized
/// units per second. A sustained hard turn therefore drains full reheat back to
/// cruise in under a second.
const ARCADE_BLEED_RATE: f64 = 0.5;

/// How fast the arcade auto-throttle spools back up when the jet is left alone,
/// in normalized units per second. Slower than the drain, so a hard turn costs
/// energy you earn back over about a second — the "auto-throttle" feel.
const ARCADE_RECOVER_RATE: f64 = 0.35;

/// Axis update at 120 Hz internally, so a frame at 30 fps and one at 144 fps
/// produce the same amount of control travel (and the same tap step).
const INPUT_HZ: f64 = 120.0;

/// Keyboard feel for one aircraft.
///
/// `LEGACY` is the original constant-rate ramp every aircraft except the
/// C172 and the 737 still uses. `C172` adds the keyboard-friendly behaviour
/// the position indicators were designed around: a fixed nudge on a tap, a
/// short hold before the ramp starts, a slow, lingering spring-back, and a
/// fine mode. `JET` is the 737's same shape at transport-class slower rates:
/// no tap (the profile's true rates), a short linger, and a gentle
/// spring-back — the hydraulic feel, not the cable feel.
#[derive(Clone, Copy, Debug, PartialEq)]
struct Feel {
    /// Fixed normalized nudge when a direction key is first pressed.
    tap_step: f64,
    /// Seconds a key must be held before the continuous ramp starts.
    hold_delay: f64,
    /// Seconds the axis lingers at its current value after release.
    release_hold: f64,
    /// Return-to-centre ramp as a fraction of the press rate: a real airframe
    /// doesn't snap its surface back the instant the force is gone.
    release_scale: f64,
    /// Scale applied to steps and rates while the fine key is held.
    fine_scale: f64,
    /// Whether the fine key does anything (C172 only for now).
    fine_enabled: bool,
    /// The FEEL page's touch knobs as multipliers over the class baseline,
    /// all `1.0` at the code's defaults (see `config::FeelSetting`).
    tap_mult: f64,
    linger_mult: f64,
    release_mult: f64,
    fine_mult: f64,
}

impl Feel {
    const LEGACY: Self = Self {
        tap_step: 0.0,
        hold_delay: 0.0,
        release_hold: 0.0,
        release_scale: 1.0,
        fine_scale: 1.0,
        fine_enabled: false,
        tap_mult: 1.0,
        linger_mult: 1.0,
        release_mult: 1.0,
        fine_mult: 1.0,
    };

    const C172: Self = Self {
        tap_step: 0.04,
        hold_delay: 0.12,
        release_hold: 0.35,
        release_scale: 0.8,
        fine_scale: 0.25,
        fine_enabled: true,
        tap_mult: 1.0,
        linger_mult: 1.0,
        release_mult: 1.0,
        fine_mult: 1.0,
    };

    const JET: Self = Self {
        tap_step: 0.0,
        hold_delay: 0.0,
        release_hold: 0.2,
        release_scale: 0.8,
        fine_scale: 1.0,
        fine_enabled: false,
        tap_mult: 1.0,
        linger_mult: 1.0,
        release_mult: 1.0,
        fine_mult: 1.0,
    };

    /// Per-axis tap sizes: rudder and trim are deliberately finer than pitch
    /// on the baseline, times the FEEL page's tap multiplier.
    fn step_for(self, action: Action) -> f64 {
        let axis = match action {
            Action::Rudder => 0.75,
            Action::Throttle => 0.5,
            Action::PitchTrim => 0.125,
            Action::Flap | Action::BrakeLeft | Action::BrakeRight | Action::Collective => return 0.0,
            _ => 1.0,
        };
        self.tap_step * axis * self.tap_mult
    }

    /// Rudder keeps its nudge a little longer, because it is the twitchiest.
    /// The linger applies before the spring-back, times the FEEL page's
    /// linger multiplier.
    fn release_for(self, action: Action) -> f64 {
        let axis = match action {
            Action::Rudder => 1.5,
            // Brakes should let go as soon as the key does.
            Action::BrakeLeft | Action::BrakeRight => 0.0,
            _ => 1.0,
        };
        self.release_hold * axis * self.linger_mult
    }

    /// Per-axis press rates the feel owns; `None` keeps the profile's rate.
    /// This is where the "keyboard full travel" seconds live: the C172's
    /// elevator is deliberately the slowest stick (a rotation is a ~1 s
    /// pull, not a twitch), the roll a touch quicker, and the 737's
    /// hydraulic surfaces the slowest of all.
    fn rate_for(self, action: Action) -> Option<f64> {
        match self {
            Self::C172 => match action {
                Action::Elevator => Some(1.0),  // full travel ≈ 1.0 s
                Action::Aileron => Some(2.0),   // full travel ≈ 0.6 s
                Action::Rudder => Some(1.5),    // full travel ≈ 0.8 s
                Action::Throttle => Some(0.5),
                Action::PitchTrim => Some(0.15),
                _ => None,
            },
            Self::JET => match action {
                Action::Elevator => Some(0.85), // full travel ≈ 1.2 s
                Action::Aileron => Some(1.0),   // full travel ≈ 1.0 s
                Action::Rudder => Some(1.5),
                _ => None,
            },
            _ => None,
        }
    }

    /// Axes without a tap step (flaps, brakes, collective) keep the original
    /// immediate ramp; otherwise a short tap would do nothing for 120 ms.
    fn hold_for(self, action: Action) -> f64 {
        if self.step_for(action) == 0.0 {
            0.0
        } else {
            self.hold_delay
        }
    }
}

/// One key pair driving one normalized JSBSim property.
pub struct Binding {
    /// Semantic id (for the HUD / future rebind UI).
    pub action: Action,
    /// Property written to the FDM each step, e.g. `fcs/elevator-cmd-norm`.
    pub property: &'static str,
    pub pos: KeyCode,
    pub neg: Option<KeyCode>,
    rate: f64,
    axis: Axis,
    min: f64,
    max: f64,
    /// Detent stops for a lever with fixed positions (the C172's flap lever),
    /// ascending `min..max` values. Empty = the axis slews continuously.
    detents: &'static [f64],
    /// The axis's starting setting, e.g. cruise throttle. Also the cruise anchor
    /// the arcade auto-throttle falls back to under a hard manoeuvre.
    initial: f64,
    /// Current normalized value in `[min, max]`.
    pub value: f64,
    /// Trim offset added on top of `value`. Zero for every axis the FDM trims
    /// itself; used only for a rotorcraft's cyclic, which has no FDM trim
    /// channel — see [`Binding::effective`] and `adopt_trim`.
    pub trim: f64,
    /// Assist contribution (bank hold / turn coordination), added on top of the
    /// pilot's value. Kept separate so the 2D discs can show white = pilot and
    /// cyan = assist.
    pub assist: f64,
    /// Seconds the current direction has been held (reset on release / flip).
    held_for: f64,
    /// Seconds since the last input stopped, for the release linger.
    since_input: f64,
    /// Previous frame's direction, so a tap step fires once per press.
    last_dir: f64,
}

impl Binding {
    /// What actually drives the FDM: the stick value plus the trim offset and
    /// any assist contribution, clamped to the axis range. For a trimmed
    /// rotorcraft a centered stick (value 0) then holds the solved cyclic.
    pub fn effective(&self) -> f64 {
        (self.value + self.trim + self.assist).clamp(self.min, self.max)
    }

    /// +1 / -1 / 0 from which of the bound keys are down.
    fn direction(&self, keys: &ButtonInput<KeyCode>) -> f64 {
        let pos = keys.pressed(self.pos) as i32;
        let neg = self.neg.map_or(0, |k| keys.pressed(k) as i32);
        (pos - neg) as f64
    }

    fn update(&mut self, keys: &ButtonInput<KeyCode>, dt: f64, feel: Feel, fine: bool) {
        let dir = self.direction(keys);
        // Split render frames into 120 Hz-ish substeps so 30 fps and 144 fps
        // produce the same travel; otherwise a hitch becomes a big stick jump.
        let steps = ((dt * INPUT_HZ).round() as usize).clamp(1, 24);
        let sub = dt / steps as f64;
        for _ in 0..steps {
            self.update_step(dir, sub, feel, fine);
        }
    }

    fn update_step(&mut self, dir: f64, dt: f64, feel: Feel, fine: bool) {
        let scale = if fine {
            feel.fine_scale * feel.fine_mult
        } else {
            1.0
        };
        match self.axis {
            Axis::Spring => {
                if dir != 0.0 {
                    if self.last_dir != dir {
                        self.value = (self.value + dir * feel.step_for(self.action) * scale)
                            .clamp(self.min, self.max);
                        self.held_for = 0.0;
                    }
                    self.held_for += dt;
                    self.since_input = 0.0;
                    if self.held_for >= feel.hold_for(self.action) {
                        let target = if dir > 0.0 { self.max } else { self.min };
                        ramp(&mut self.value, target, self.rate * scale * dt);
                    }
                } else {
                    self.held_for = 0.0;
                    self.since_input += dt;
                    if self.since_input >= feel.release_for(self.action) {
                        // The spring-back is gentler than the press — the
                        // surface has inertia, and the keyboard player needs
                        // the same grace after a tap. The FEEL page's return
                        // multiplier rides on top.
                        ramp(
                            &mut self.value,
                            0.0,
                            self.rate * feel.release_scale * feel.release_mult * dt,
                        );
                    }
                }
                self.last_dir = dir;
            }
            Axis::Hold => {
                if dir != 0.0 {
                    if self.last_dir != dir {
                        if self.detents.is_empty() {
                            self.value = (self.value + dir * feel.step_for(self.action) * scale)
                                .clamp(self.min, self.max);
                        } else {
                            self.value = self.next_detent(dir);
                        }
                        self.held_for = 0.0;
                    }
                    self.held_for += dt;
                    self.since_input = 0.0;
                    if self.detents.is_empty() && self.held_for >= feel.hold_for(self.action) {
                        let target = if dir > 0.0 { self.max } else { self.min };
                        ramp(&mut self.value, target, self.rate * scale * dt);
                    }
                } else {
                    self.held_for = 0.0;
                    self.since_input += dt;
                }
                self.last_dir = dir;
            }
        }
    }

    /// The detent a press of `dir` (+1 forward / −1 back) selects, relative to
    /// the current command. Already at the last stop in that direction? Stay
    /// there and wait for the surface to catch up.
    fn next_detent(&self, dir: f64) -> f64 {
        let eps = 1e-4;
        let mut picked = self.value;
        if dir > 0.0 {
            for stop in self.detents {
                if *stop > self.value + eps {
                    picked = *stop;
                    break;
                }
            }
        } else {
            for stop in self.detents.iter().rev() {
                if *stop < self.value - eps {
                    picked = *stop;
                    break;
                }
            }
        }
        picked.clamp(self.min, self.max)
    }
}

/// The knob baselines the FEEL settings page shows and multiplexes onto the
/// bindings — the code's defaults for one aircraft class, plus which knobs the
/// class offers at all.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FeelBaseline {
    /// Full-travel seconds for the four sweep axes.
    pub travel_elevator: f64,
    pub travel_aileron: f64,
    pub travel_rudder: f64,
    pub travel_throttle: f64,
    /// Normalized tap nudge (fraction of travel).
    pub tap_step: f64,
    /// Release linger seconds.
    pub release_hold: f64,
    /// Spring-back speed as a fraction of press rate.
    pub release_scale: f64,
    /// Fine-mode step scale (fraction of travel).
    pub fine_scale: f64,
    /// Whether the class offers tap/fine knobs at all.
    pub tap_enabled: bool,
    pub fine_enabled: bool,
}

/// The pilot's control set for one aircraft. Iterated each frame to drive the
/// FDM.
#[derive(Resource)]
pub struct Bindings {
    pub list: Vec<Binding>,
    pub toggles: Vec<ToggleSpec>,
    /// Keyboard feel for this aircraft. Copied out before iteration so the
    /// update loop can mutate the bindings.
    feel: Feel,
}

impl Bindings {
    /// Build the runtime control set for `id` from its profile's fixed bindings,
    /// then ride the persisted FEEL multipliers onto it (travel times divide
    /// the ramp rates; the touch knobs multiply the Feel fields).
    pub fn for_aircraft_with(id: &str, feel: &crate::config::FeelSetting) -> Self {
        let mut b = Self::for_aircraft(id);
        b.apply_feel(feel);
        b
    }

    /// Scale the persisted feel onto a built set: `for_aircraft` alone stays
    /// the pure defaults, so tests can control their inputs exactly.
    pub fn apply_feel(&mut self, fs: &crate::config::FeelSetting) {
        for x in &mut self.list {
            // Travel-time multipliers: 1.2 s of travel means 1/1.2 the rate.
            let time = match x.action {
                Action::Elevator => fs.travel_elevator,
                Action::Aileron => fs.travel_aileron,
                Action::Rudder => fs.travel_rudder,
                Action::Throttle => fs.travel_throttle,
                _ => continue,
            };
            if time > 0.01 {
                x.rate /= time;
            }
        }
        let f = &mut self.feel;
        f.tap_mult = fs.tap;
        f.linger_mult = fs.linger;
        f.release_mult = fs.release;
        f.fine_mult = fs.fine;
    }

    /// The knob baselines a settings page shows (the code's defaults for this
    /// aircraft class), plus whether the aircraft offers tap/fine rows at all.
    pub fn feel_baseline(id: &str) -> FeelBaseline {
        let feel = match aircraft_profile::normalized(id) {
            "c172p" => Feel::C172,
            "737" => Feel::JET,
            _ => Feel::LEGACY,
        };
        let base = Self::for_aircraft(id);
        // "Travel" seconds as the page shows them: neutral-to-stop on a
        // spring axis (its ramp spans `max` units from the centre), lever
        // sweep on a hold axis. `for_aircraft` already merges the feel-owned
        // rates over the profile's, so the ramp rate is `x.rate`.
        let travel = |action: Action| -> f64 {
            let Some(x) = base.list.iter().find(|b| b.action == action) else {
                return 0.0;
            };
            if x.rate <= 0.0 {
                return 0.0;
            }
            let range = match x.axis {
                Axis::Spring => x.max.abs(),
                _ => x.max - x.min,
            };
            range / x.rate
        };
        FeelBaseline {
            travel_elevator: travel(Action::Elevator),
            travel_aileron: travel(Action::Aileron),
            travel_rudder: travel(Action::Rudder),
            travel_throttle: travel(Action::Throttle),
            tap_step: feel.tap_step,
            release_hold: feel.release_hold,
            release_scale: feel.release_scale,
            fine_scale: feel.fine_scale,
            tap_enabled: feel.tap_step > 0.0,
            fine_enabled: feel.fine_enabled,
        }
    }

    /// Build the runtime control set for `id` from its profile's fixed bindings.
    pub fn for_aircraft(id: &str) -> Self {
        let p = aircraft_profile::profile(id);
        let feel = match aircraft_profile::normalized(id) {
            "c172p" => Feel::C172,
            "737" => Feel::JET,
            _ => Feel::LEGACY,
        };
        let list = p
            .bindings
            .iter()
            .map(|s| {
                let (pos, neg) = (s.pos, s.neg);
                Binding {
                    action: s.action,
                    property: s.property,
                    pos,
                    neg,
                    rate: feel.rate_for(s.action).unwrap_or(s.rate),
                    axis: s.axis,
                    min: s.min,
                    max: s.max,
                    detents: s.detents,
                    initial: s.initial,
                    value: s.initial,
                    trim: 0.0,
                    assist: 0.0,
                    held_for: 0.0,
                    since_input: 0.0,
                    last_dir: 0.0,
                }
            })
            .collect();
        let toggles = p
            .toggles
            .iter()
            .map(|t| ToggleSpec {
                action: t.action,
                key: t.key,
            })
            .collect();
        Bindings {
            list,
            toggles,
            feel,
        }
    }

    /// Give the throttle the arcade boost feel for aircraft with an arcade
    /// dynamics path (the F-22 in missions, the AH-64D when selected): a tap
    /// spools it to full and it *stays* there, so the burner doesn't need a key
    /// held down. The spool just runs faster than the realistic lever. A no-op
    /// on every other aircraft.
    pub fn enable_arcade(&mut self, id: &str) {
        if !aircraft_profile::profile(id).has_arcade {
            return;
        }
        for b in &mut self.list {
            if b.action == Action::Throttle {
                b.rate = ARCADE_SPOOL_RATE;
            }
        }
    }
}

pub struct InputPlugin;

impl Plugin for InputPlugin {
    fn build(&self, app: &mut App) {
        // `Sim` is inserted by `SimPlugin`, which is registered first, so the
        // startup aircraft is already known here.
        let name = app.world().resource::<crate::sim::Sim>().name.clone();
        let arcade = app.world().resource::<GameConfig>().arcade(&name);
        let mut bindings = Bindings::for_aircraft(&name);
        if arcade {
            bindings.enable_arcade(&name);
        }
        app.insert_resource(bindings)
            .add_systems(Update, read_input.run_if(unpaused));
    }
}

fn read_input(
    keys: Res<ButtonInput<KeyCode>>,
    time: Res<Time>,
    mut bindings: ResMut<Bindings>,
    mut sim: ResMut<crate::sim::Sim>,
    mut ap: ResMut<Autopilot>,
    mut mcp: ResMut<Mcp>,
    config: Res<GameConfig>,
    map: Res<crate::map::ActiveMap>,
    weather: Res<crate::weather::Weather>,
    mut lights: ResMut<crate::lights::AircraftLights>,
) {
    let dt = time.delta_secs_f64();
    // Fine control: C172 only for now. `Space` is the throttle's own key —
    // it must NOT arm fine mode while you feed power on a final (every tap
    // elsewhere would quietly quarter itself, the mushy-then-snappy mess a
    // line-up dies of). Shift and Alt are free everywhere and sit under the
    // left hand anyway.
    let fine = bindings.feel.fine_enabled
        && (keys.pressed(KeyCode::ShiftLeft)
            || keys.pressed(KeyCode::AltLeft)
            || keys.pressed(KeyCode::AltRight));
    let feel = bindings.feel;
    for b in &mut bindings.list {
        b.update(&keys, dt, feel, fine);
    }

    // The arcade throttle is automatic in the air: left alone in straight flight
    // it spools itself up to full reheat, and a hard pull or yaw drains it back
    // toward cruise, where the burner dies. Space tops it up on demand. On the
    // ground it stays a plain lever, so the jet still taxis and lands. Roll is
    // ignored — a pure roll doesn't swing the flight path.
    // Missions force arcade dynamics. `config.arcade` only reports aircraft
    // that still offer the switch; the F-22's arcade path is mission-only.
    let arcade = map.0.is_some() || config.arcade(&sim.name);
    if arcade && sim.model.get("gear/wow") < 0.5 {
        let q = sim.model.get("velocities/q-rad_sec");
        let r = sim.model.get("velocities/r-rad_sec");
        let manoeuvre = manoeuvre_intensity(q, r);
        if let Some(b) = bindings
            .list
            .iter_mut()
            .find(|b| b.action == Action::Throttle)
        {
            // Don't fight a pilot actively working the throttle keys.
            let pilot = keys.pressed(b.pos) || b.neg.is_some_and(|k| keys.pressed(k));
            if !pilot {
                b.value = arcade_throttle(b.value, b.initial, b.max, manoeuvre, dt);
            }
        }
    }

    // Pilot stick or throttle input overrides the automation, as it does in the
    // real aircraft: moving the column or the levers drops the A/P or the A/T.
    for b in &bindings.list {
        if b.direction(&keys) == 0.0 {
            continue;
        }
        match b.action {
            Action::Elevator | Action::Aileron | Action::Rudder => ap.disengage(),
            Action::Throttle => ap.disconnect_autothrottle(),
            _ => {}
        }
    }

    // Edge toggles declared by the active profile. Collect first so the borrow
    // of `bindings.toggles` ends before `adopt_trim` takes `&mut bindings`.
    let fired: Vec<ToggleAction> = bindings
        .toggles
        .iter()
        .filter(|t| keys.just_pressed(t.key))
        .map(|t| t.action)
        .collect();
    for action in fired {
        match action {
            ToggleAction::Autopilot => ap.toggle(&*sim.model),
            ToggleAction::HdgSel
            | ToggleAction::AltHold
            | ToggleAction::VsMode
            | ToggleAction::LvlChg
            | ToggleAction::AutoThrottle => ap.press_mode(action, &*sim.model),
            ToggleAction::SpeedMach => mcp.toggle_speed_unit(),
            // The MCP speed window, one nudge per press (the A/T holds it).
            ToggleAction::SpeedDown => {
                mcp.nudge(crate::autopilot::McpWindow::Speed, -1.0, false)
            }
            ToggleAction::SpeedUp => {
                mcp.nudge(crate::autopilot::McpWindow::Speed, 1.0, false)
            }
            // The 737's speedbrake handle and reverse thrust. Reverse drops
            // the autothrottle so it cannot fight the back-up thrust (the pilot
            // retard the throttles to engage reverse on the real jet anyway).
            ToggleAction::SpeedBrake => toggle_prop(&mut sim, "fcs/speedbrake-cmd-norm"),
            ToggleAction::Reverse => {
                toggle_prop(&mut sim, "fcs/reverse-cmd-norm");
                ap.disconnect_autothrottle();
            }
            ToggleAction::Approach => ap.press_mode(ToggleAction::Approach, &*sim.model),
            // A no-op on models without these (the FDM ignores the property).
            ToggleAction::Tvc => toggle_prop(&mut sim, "fcs/tvc-enable"),
            ToggleAction::Gear => toggle_prop(&mut sim, "gear/gear-cmd-norm"),
            // Re-trim for the current condition; hand control to the pilot and
            // adopt the trimmed commands so the per-step writes don't undo it.
            ToggleAction::Trim => {
                sim.trim_now();
                ap.disengage();
                adopt_trim(&mut bindings, &sim);
            }
            ToggleAction::RotorTrim => {
                sim.trim_now();
                adopt_trim(&mut bindings, &sim);
            }
            // C172 exterior lights pack (nav + beacon + strobe + landing
            // light). Manual `L` overrides the time-of-day default; the pack
            // itself reacts in `lights.rs`.
            ToggleAction::Lights => {
                lights.toggle(&weather);
            }
            // AH-64D arcade helpers. Both are no-ops on models that don't
            // expose the properties; the realistic helicopter ignores them.
            ToggleAction::HeliHover => toggle_prop(&mut sim, "fcs/heli-hover-enable"),
            ToggleAction::HeliBalance => toggle_prop(&mut sim, "fcs/heli-balance-enable"),
            // C172 cold start: the master switch. Spark alone never fires an
            // engine — the starter crank (hold I) does the rest of the job.
            ToggleAction::Master => toggle_prop(&mut sim, "propulsion/ignition-enable"),
            // Arcade combat keys are edge/held inputs consumed by `combat.rs`
            // itself (it reads the same bindings, so rebinding still works).
            ToggleAction::CycleTarget
            | ToggleAction::CycleWeapon
            | ToggleAction::RadarRange
            | ToggleAction::FireMissile
            | ToggleAction::FireGun
            | ToggleAction::Flares => {}
        }
    }

    // Spawns live in the menu (lesson rows / map missions); there are no
    // fly-time relocate keys.
}

/// Flip a 0/1 property (`1 - current`), used by the gear and TVC toggles.
fn toggle_prop(sim: &mut crate::sim::Sim, property: &str) {
    let on = sim.model.get(property) > 0.5;
    sim.model.set(property, if on { 0.0 } else { 1.0 });
}

/// Adopt the model's trimmed commands into the matching bindings. The app
/// writes every binding to the model each step, so a binding left at its default
/// would clobber the trim solver's result; seeding the binding with the readback
/// keeps the two consistent.
///
/// Persistent axes (throttle, collective, pitch trim) adopt the solved value
/// directly. A rotorcraft's cyclic has no FDM trim channel and the app's sticks
/// spring back to neutral, so its solved cyclic is seated as a spring offset
/// ([`Binding::trim`]) — a released stick then holds the hover. Fixed-wing keeps
/// its cyclic neutral; the FDM's own pitch-trim channel carries the elevator.
fn adopt_trim(bindings: &mut Bindings, sim: &crate::sim::Sim) {
    let rotor = sim.model.has_rotor();
    for b in &mut bindings.list {
        match b.property {
            "fcs/throttle-cmd-norm" | "fcs/collective-cmd-norm" | "fcs/pitch-trim-cmd-norm" => {
                b.value = sim.model.get(b.property)
            }
            "fcs/elevator-cmd-norm" | "fcs/aileron-cmd-norm" | "fcs/rudder-cmd-norm" if rotor => {
                b.trim = sim.model.get(b.property)
            }
            _ => {}
        }
    }
}

/// How hard the jet is manoeuvring, 0 (flying straight) .. 1 (a full-rate
/// pitch, or a pitch and yaw that sum to one). Reads the actual body rates, so
/// it responds to the nose actually moving, not to the stick being pressed.
fn manoeuvre_intensity(q: f64, r: f64) -> f64 {
    ((q.abs() + r.abs()) / ARCADE_BLEED_FULL).min(1.0)
}

/// The arcade auto-throttle. `manoeuvre` sets a target between full reheat
/// (straight) and cruise (a hard pull); the throttle moves toward it — draining
/// fast under a hard pull, spooling back up at [`ARCADE_RECOVER_RATE`] when the
/// jet is left alone — so the burner relights itself once the fight stops.
fn arcade_throttle(value: f64, cruise: f64, full: f64, manoeuvre: f64, dt: f64) -> f64 {
    let target = full - (full - cruise) * manoeuvre;
    if value > target {
        (value - ARCADE_BLEED_RATE * manoeuvre * dt).max(target)
    } else {
        (value + ARCADE_RECOVER_RATE * dt).min(target)
    }
}

fn ramp(value: &mut f64, target: f64, step: f64) {
    if (*value - target).abs() <= step {
        *value = target;
    } else if *value < target {
        *value += step;
    } else {
        *value -= step;
    }
}

#[cfg(test)]
mod tests {
    use super::{adopt_trim, arcade_throttle, manoeuvre_intensity, ramp, Binding, Bindings, Feel};
    use crate::aircraft_profile::{Action, Axis};
    use crate::config::FeelSetting;
    use bevy::prelude::{ButtonInput, KeyCode};

    fn cyclic() -> Binding {
        Binding {
            action: Action::Elevator,
            property: "fcs/elevator-cmd-norm",
            pos: KeyCode::KeyS,
            neg: Some(KeyCode::KeyW),
            rate: 3.0,
            axis: Axis::Spring,
            min: -1.0,
            max: 1.0,
            detents: &[],
            initial: 0.0,
            value: 0.0,
            trim: 0.0,
            assist: 0.0,
            held_for: 0.0,
            since_input: 0.0,
            last_dir: 0.0,
        }
    }

    #[test]
    fn hover_trim_adopted_into_bindings_holds_the_hover() {
        // End-to-end for the rotorcraft trim: trim it, adopt the solved commands
        // into the bindings, then drive the model the way `advance_fdm` does
        // (write `effective()` every step). The hover must hold with sticks
        // centered.
        std::env::set_var("FDM_BACKEND", "rust");
        let mut sim = crate::sim::Sim::new("ah1s");
        // Airborne at the altitude the FDM's own hover test validates (2000 ft);
        // `trim_heli` seats a zero-airspeed hover there.
        sim.reset_air(2000.0, 0.0);

        let mut bindings = Bindings::for_aircraft("ah1s");
        sim.trim_now();
        adopt_trim(&mut bindings, &sim);

        let alt0 = sim.model.get("position/h-sl-ft");
        for _ in 0..120 * 10 {
            for b in &bindings.list {
                sim.model.set(b.property, b.effective());
            }
            sim.model.step();
        }
        let alt1 = sim.model.get("position/h-sl-ft");
        assert!((alt1 - alt0).abs() < 25.0, "hover sagged: {alt0} -> {alt1}");
        assert!(
            sim.model.get("velocities/vc-kts") < 2.0,
            "drifted horizontally: {}",
            sim.model.get("velocities/vc-kts")
        );
    }

    #[test]
    fn hover_trim_preserves_the_current_airspeed() {
        // The complaint this fixes: the trim used to zero the velocity, so it
        // felt like a reset. It must now keep the current airspeed.
        std::env::set_var("FDM_BACKEND", "rust");
        let mut sim = crate::sim::Sim::new("ah1s");
        sim.reset_air(2000.0, 0.0);

        // Fly forward (nose-down cyclic accelerates it), then trim.
        sim.model.set("fcs/elevator-cmd-norm", 0.5);
        for _ in 0..120 * 5 {
            sim.model.step();
        }
        let vc_before = sim.model.get("velocities/vc-kts");
        assert!(vc_before > 5.0, "should be moving: {vc_before} kt");

        let mut bindings = Bindings::for_aircraft("ah1s");
        sim.trim_now(); // what the RotorTrim toggle does
        adopt_trim(&mut bindings, &sim);
        let vc_after = sim.model.get("velocities/vc-kts");
        assert!(
            (vc_after - vc_before).abs() < 3.0,
            "trim reset the airspeed: {vc_before} -> {vc_after} kt"
        );

        // And it holds hands-free for ten seconds.
        for _ in 0..120 * 10 {
            for b in &bindings.list {
                sim.model.set(b.property, b.effective());
            }
            sim.model.step();
        }
        assert!(
            (sim.model.get("velocities/vc-kts") - vc_after).abs() < 6.0,
            "forward trim drifted: {vc_after} -> {} kt",
            sim.model.get("velocities/vc-kts")
        );
    }

    #[test]
    fn holding_forward_cyclic_accelerates_forward() {
        // How you fly forward after trimming: hold the cyclic forward (key S,
        // which pitches the nose down), and the helicopter accelerates and moves
        // forward. (It is also pitch-unstable — see the FDM's airframe aero —
        // which is why it's hard to keep in trim by hand.)
        use bevy::prelude::ButtonInput;
        std::env::set_var("FDM_BACKEND", "rust");
        let mut sim = crate::sim::Sim::new("ah1s");
        sim.reset_air(2000.0, 0.0);
        let mut bindings = Bindings::for_aircraft("ah1s");
        sim.trim_now();
        adopt_trim(&mut bindings, &sim);

        let dt = 1.0 / 120.0;
        let mut held = ButtonInput::<KeyCode>::default();
        held.press(KeyCode::KeyS);

        for _ in 0..120 * 3 {
            for b in &mut bindings.list {
                b.update(&held, dt, Feel::LEGACY, false);
            }
            for b in &bindings.list {
                sim.model.set(b.property, b.effective());
            }
            sim.model.step();
        }
        assert!(
            sim.model.get("velocities/vc-kts") > 10.0,
            "holding forward cyclic should accelerate: {} kt",
            sim.model.get("velocities/vc-kts")
        );
        assert!(
            sim.model.get("position/lat-gc-deg") > 1e-4,
            "should have moved north: {} deg",
            sim.model.get("position/lat-gc-deg")
        );
    }

    #[test]
    fn effective_folds_in_trim_and_clamps() {
        let mut b = cyclic();
        // A centered stick holds the trim offset (the hover-trim case).
        b.trim = 0.2;
        assert_eq!(b.effective(), 0.2);
        // Stick input adds on top of the offset.
        b.value = -0.05;
        assert!((b.effective() - 0.15).abs() < 1e-12);
        // The sum is clamped to the axis range.
        b.value = 1.0;
        assert_eq!(b.effective(), 1.0);
    }

    #[test]
    fn ramp_moves_toward_target_by_one_step() {
        let mut v = 0.0;
        ramp(&mut v, 1.0, 0.3);
        assert_eq!(v, 0.3);
        ramp(&mut v, 1.0, 0.3);
        assert_eq!(v, 0.6);
    }

    #[test]
    fn ramp_moves_down_for_a_negative_target() {
        let mut v = 0.0;
        ramp(&mut v, -1.0, 0.25);
        assert_eq!(v, -0.25);
    }

    #[test]
    fn ramp_snaps_and_never_overshoots_within_a_step() {
        let mut v = 0.9;
        ramp(&mut v, 1.0, 0.3);
        assert_eq!(v, 1.0);
        ramp(&mut v, 1.0, 0.3);
        assert_eq!(v, 1.0);
    }

    #[test]
    fn ramp_snaps_when_already_close() {
        let mut v = 0.5;
        ramp(&mut v, 0.5, 0.3);
        assert_eq!(v, 0.5);
    }

    /// The throttle binding's current value, for the arcade tests below.
    fn throttle_value(bindings: &Bindings) -> f64 {
        bindings
            .list
            .iter()
            .find(|b| b.action == Action::Throttle)
            .unwrap()
            .value
    }

    #[test]
    fn arcade_throttle_latches_at_full_and_stays() {
        use bevy::prelude::ButtonInput;
        let mut bindings = Bindings::for_aircraft("f22");
        bindings.enable_arcade("f22");
        let throttle = bindings
            .list
            .iter()
            .find(|b| b.action == Action::Throttle)
            .unwrap();
        assert_eq!(throttle.axis, Axis::Hold);
        assert_eq!(throttle.pos, KeyCode::Space);

        let dt = 1.0 / 120.0;
        let mut keys = ButtonInput::<KeyCode>::default();

        // A tap of the boost: it spools to the top.
        keys.press(KeyCode::Space);
        for _ in 0..120 {
            for b in &mut bindings.list {
                b.update(&keys, dt, Feel::LEGACY, false);
            }
        }
        assert!(throttle_value(&bindings) > 0.99, "boost never reached full");

        // Release: it holds at full — no key needed to keep the thrust.
        keys.release(KeyCode::Space);
        for _ in 0..240 {
            for b in &mut bindings.list {
                b.update(&keys, dt, Feel::LEGACY, false);
            }
        }
        assert!(
            throttle_value(&bindings) > 0.99,
            "the latched throttle drifted on release: {}",
            throttle_value(&bindings)
        );
    }

    #[test]
    fn arcade_throttle_is_auto_between_cruise_and_full() {
        let dt = 1.0 / 120.0;
        let step = |mut v: f64, manoeuvre: f64| {
            for _ in 0..600 {
                v = arcade_throttle(v, 0.6, 1.0, manoeuvre, dt);
            }
            v
        };

        // Roll alone is not a manoeuvre.
        assert_eq!(manoeuvre_intensity(0.0, 0.0), 0.0);

        // Left alone in straight flight it spools itself back up to full reheat.
        assert!(
            (step(0.6, 0.0) - 1.0).abs() < 1e-6,
            "straight flight did not recharge the throttle"
        );

        // A hard pull drains full reheat to cruise and holds there.
        assert!(
            (step(1.0, 1.0) - 0.6).abs() < 1e-6,
            "a hard pull did not drain to cruise"
        );

        // A half-strength manoeuvre settles halfway between the two.
        assert!(
            (step(1.0, 0.5) - 0.8).abs() < 1e-6,
            "a half pull did not settle mid-range"
        );
    }

    #[test]
    fn arcade_throttle_is_left_alone_on_aircraft_without_it() {
        let mut bindings = Bindings::for_aircraft("c172p");
        bindings.enable_arcade("c172p");
        let throttle = bindings
            .list
            .iter()
            .find(|b| b.action == Action::Throttle)
            .unwrap();
        assert_eq!(throttle.axis, Axis::Hold);
        assert_eq!(throttle.pos, KeyCode::ShiftLeft);
    }

    fn axis_value(bindings: &Bindings, action: Action) -> f64 {
        bindings
            .list
            .iter()
            .find(|b| b.action == action)
            .unwrap()
            .value
    }

    /// One held ramp, at 60 fps, telling how far the axis traveled.
    fn hold_travel(action: Action, secs: f64) -> f64 {
        let mut bindings = Bindings::for_aircraft("c172p");
        let feel = bindings.feel;
        let mut keys = ButtonInput::<KeyCode>::default();
        let key = KeyCode::KeyW; // elevator's negative key: nose up
        if action != Action::Elevator {
            panic!("hold_travel sanity: only the elevator is wired here");
        }
        keys.press(key);
        let steps = (secs * 60.0) as usize;
        for _ in 0..steps {
            for b in &mut bindings.list {
                b.update(&keys, 1.0 / 60.0, feel, false);
            }
        }
        axis_value(&bindings, action).abs()
    }

    #[test]
    fn feel_travel_multiplier_rescales_the_ramp() {
        // Stock: 1 s of held ramp from neutral (tap 0.04, then ~0.88 s of ramp).
        let stock = hold_travel(Action::Elevator, 1.0);
        assert!(stock > 0.85 && stock < 0.98, "{stock}");

        // Double the travel *time*: the same held second goes half as far.
        let slow = {
            let mut bindings = Bindings::for_aircraft("c172p");
            bindings.apply_feel(&FeelSetting {
                travel_elevator: 2.0,
                ..FeelSetting::default()
            });
            let feel = bindings.feel;
            let mut keys = ButtonInput::<KeyCode>::default();
            keys.press(KeyCode::KeyW);
            for _ in 0..60 {
                for b in &mut bindings.list {
                    b.update(&keys, 1.0 / 60.0, feel, false);
                }
            }
            axis_value(&bindings, Action::Elevator).abs()
        };
        assert!(
            (slow - stock / 2.0).abs() < 0.12,
            "2x travel time should halve the held ramp: {stock} vs {slow}"
        );

        // And a *faster* travel beats stock the other way.
        let quick = {
            let mut bindings = Bindings::for_aircraft("c172p");
            bindings.apply_feel(&FeelSetting {
                travel_elevator: 0.5,
                ..FeelSetting::default()
            });
            let feel = bindings.feel;
            let mut keys = ButtonInput::<KeyCode>::default();
            keys.press(KeyCode::KeyW);
            for _ in 0..60 {
                for b in &mut bindings.list {
                    b.update(&keys, 1.0 / 60.0, feel, false);
                }
            }
            axis_value(&bindings, Action::Elevator).abs()
        };
        assert!(quick > stock, "0.5x time should beat stock: {stock} vs {quick}");
    }

    #[test]
    fn feel_touch_knobs_scale_taps_and_release() {
        // Tap nudge doubles.
        let mut bindings = Bindings::for_aircraft("c172p");
        bindings.apply_feel(&FeelSetting {
            tap: 2.0,
            ..FeelSetting::default()
        });
        let feel = bindings.feel;
        let mut keys = ButtonInput::<KeyCode>::default();
        keys.press(KeyCode::KeyW);
        for b in &mut bindings.list {
            b.update(&keys, 1.0 / 60.0, feel, false);
        }
        keys.release(KeyCode::KeyW);
        let nudged = axis_value(&bindings, Action::Elevator).abs();
        assert!(
            (nudged - 0.08).abs() < 1e-9,
            "2x tap should nudge 8 %: {nudged}"
        );

        // Linger: stock C172 holds 0.35 s before spring-back; 2x holds 0.7 s.
        let mut bindings = Bindings::for_aircraft("c172p");
        bindings.apply_feel(&FeelSetting {
            linger: 2.0,
            ..FeelSetting::default()
        });
        let feel = bindings.feel;
        let mut keys = ButtonInput::<KeyCode>::default();
        keys.press(KeyCode::KeyW); // first a fresh tap sets the surface
        for b in &mut bindings.list {
            b.update(&keys, 1.0 / 60.0, feel, false);
        }
        keys.release(KeyCode::KeyW);
        for _ in 0..(0.5 * 60.0) as usize {
            for b in &mut bindings.list {
                b.update(&keys, 1.0 / 60.0, feel, false);
            }
        }
        // 0.5 s of idle: with 2x linger the surface has NOT started back yet.
        let rested = axis_value(&bindings, Action::Elevator).abs();
        assert!(
            (rested - 0.04).abs() < 1e-9,
            "2x linger should still be resting at 0.5 s: {rested}"
        );

        // Return + fine: the direct knob arithmetic on the Feel profile, the
        // same quantities update_step consumes.
        let mut feel = Feel::C172;
        assert_eq!(feel.release_for(Action::Elevator), 0.35);
        feel.release_mult = 2.0;
        // The return multiplier does NOT touch the linger.
        assert_eq!(feel.release_for(Action::Elevator), 0.35);
        feel.linger_mult = 2.0;
        assert_eq!(feel.release_for(Action::Elevator), 0.70);
        // The 737's linger scales from its own, shorter baseline.
        let mut jet = Feel::JET;
        jet.linger_mult = 2.0;
        let jet_linger = jet.release_for(Action::Rudder);
        assert!(
            (jet_linger - 0.6).abs() < 1e-9,
            "737 rudder linger x2: {jet_linger}"
        );

        // Fine rate: a second tap with fine held is twice the stock fine nudge
        // (stock 4 % x 0.25 = 1 %; 2x fine = 2 %).
        let mut stock = Bindings::for_aircraft("c172p");
        let stock_feel = stock.feel;
        let mut doubled = Bindings::for_aircraft("c172p");
        doubled.apply_feel(&FeelSetting {
            fine: 2.0,
            ..FeelSetting::default()
        });
        let doubled_feel = doubled.feel;
        let mut keys = ButtonInput::<KeyCode>::default();
        keys.press(KeyCode::KeyW);
        for b in &mut stock.list {
            b.update(&keys, 1.0 / 60.0, stock_feel, true);
        }
        for b in &mut doubled.list {
            b.update(&keys, 1.0 / 60.0, doubled_feel, true);
        }
        let stock_fine = axis_value(&stock, Action::Elevator).abs();
        let doubled_fine = axis_value(&doubled, Action::Elevator).abs();
        assert!(
            (stock_fine - 0.01).abs() < 1e-9,
            "stock fine tap should be 1 %: {stock_fine}"
        );
        assert!(
            (doubled_fine - 0.02).abs() < 1e-9,
            "2x fine tap should be 2 %: {doubled_fine}"
        );
    }

    #[test]
    fn c172_tap_is_a_small_fixed_step_at_any_frame_rate() {
        for fps in [30.0, 60.0, 144.0] {
            let mut bindings = Bindings::for_aircraft("c172p");
            let feel = bindings.feel;
            let mut keys = ButtonInput::<KeyCode>::default();

            // W is the elevator's negative key: nose up.
            keys.press(KeyCode::KeyW);
            for b in &mut bindings.list {
                b.update(&keys, 1.0 / fps, feel, false);
            }
            keys.release(KeyCode::KeyW);

            let value = axis_value(&bindings, Action::Elevator);
            assert!(
                (value + 0.04).abs() < 1e-9,
                "{fps} fps tap should be -0.04, got {value}"
            );
        }
    }

    #[test]
    fn c172_release_lingers_before_springing_back() {
        let mut bindings = Bindings::for_aircraft("c172p");
        let feel = bindings.feel;
        let mut keys = ButtonInput::<KeyCode>::default();
        let dt = 1.0 / 60.0;

        keys.press(KeyCode::KeyW);
        for b in &mut bindings.list {
            b.update(&keys, dt, feel, false);
        }
        keys.release(KeyCode::KeyW);

        // Inside the 350 ms linger the nudge is still there.
        for _ in 0..3 {
            for b in &mut bindings.list {
                b.update(&keys, dt, feel, false);
            }
        }
        let held = axis_value(&bindings, Action::Elevator);
        assert!(
            (held + 0.04).abs() < 1e-9,
            "release hold moved early: {held}"
        );

        // After it, the axis springs back to centre.
        for _ in 0..30 {
            for b in &mut bindings.list {
                b.update(&keys, dt, feel, false);
            }
        }
        let released = axis_value(&bindings, Action::Elevator);
        assert!(released.abs() < 1e-6, "axis did not return: {released}");
    }

    #[test]
    fn c172_fine_mode_scales_the_tap_step() {
        let mut bindings = Bindings::for_aircraft("c172p");
        let feel = bindings.feel;
        let mut keys = ButtonInput::<KeyCode>::default();

        keys.press(KeyCode::Space);
        keys.press(KeyCode::KeyW);
        for b in &mut bindings.list {
            b.update(&keys, 1.0 / 60.0, feel, true);
        }

        let value = axis_value(&bindings, Action::Elevator);
        assert!(
            (value + 0.01).abs() < 1e-9,
            "fine tap should be -0.01, got {value}"
        );
    }

    #[test]
    fn c172_holding_a_key_ramps_to_full() {
        let mut bindings = Bindings::for_aircraft("c172p");
        let feel = bindings.feel;
        let mut keys = ButtonInput::<KeyCode>::default();
        let dt = 1.0 / 60.0;

        keys.press(KeyCode::KeyW);
        for _ in 0..72 {
            for b in &mut bindings.list {
                b.update(&keys, dt, feel, false);
            }
        }

        let value = axis_value(&bindings, Action::Elevator);
        assert!(value < -0.9, "holding W should reach full up: {value}");
    }

    #[test]
    fn jet_feel_is_transport_slow_on_the_stick_and_lingers_after_release() {
        let mut bindings = Bindings::for_aircraft("737");
        let feel = bindings.feel;
        assert_eq!(feel.tap_step, 0.0, "no tap nudge on the jet");
        let mut keys = ButtonInput::<KeyCode>::default();
        let dt = 1.0 / 60.0;

        // W pitches up. One second of holding at 0.85/s: close to full, but
        // far from the old 3/s twitch (0.33 s).
        keys.press(KeyCode::KeyW);
        for _ in 0..60 {
            for b in &mut bindings.list {
                b.update(&keys, dt, feel, false);
            }
        }
        let held = axis_value(&bindings, Action::Elevator);
        assert!(
            (held + 0.85).abs() < 0.06,
            "one second should be ~85% travel: {held}"
        );

        // The release: a short linger at the attained attitude, then a slow
        // spring-back — no instant snap.
        keys.release(KeyCode::KeyW);
        for _ in 0..6 {
            for b in &mut bindings.list {
                b.update(&keys, dt, feel, false);
            }
        }
        let lingering = axis_value(&bindings, Action::Elevator);
        assert!(
            (lingering - held).abs() < 0.02,
            "the 200 ms linger should hold the deflect: {lingering}"
        );

        // Then it decays gently back to centre.
        for _ in 0..150 {
            for b in &mut bindings.list {
                b.update(&keys, dt, feel, false);
            }
        }
        let released = axis_value(&bindings, Action::Elevator);
        assert!(released.abs() < 1e-6, "axis did not return: {released}");
    }

    #[test]
    fn c172_brake_tap_still_bites_immediately() {
        let mut bindings = Bindings::for_aircraft("c172p");
        let feel = bindings.feel;
        let mut keys = ButtonInput::<KeyCode>::default();

        // Brakes have no tap step, so they must keep the legacy immediate ramp
        // rather than waiting out the pitch/roll hold delay.
        keys.press(KeyCode::KeyB);
        for b in &mut bindings.list {
            b.update(&keys, 1.0 / 60.0, feel, false);
        }

        let value = axis_value(&bindings, Action::BrakeLeft);
        assert!(value > 0.05, "a brake tap did nothing: {value}");
    }

    #[test]
    fn feel_axes_without_a_tap_step_skip_the_hold_delay() {
        assert_eq!(Feel::C172.hold_for(Action::BrakeLeft), 0.0);
        assert_eq!(Feel::C172.hold_for(Action::Flap), 0.0);
        assert_eq!(Feel::C172.hold_for(Action::Elevator), Feel::C172.hold_delay);
    }

    #[test]
    fn c172_flap_keys_step_between_detents() {
        let mut bindings = Bindings::for_aircraft("c172p");
        let feel = bindings.feel;
        let mut keys = ButtonInput::<KeyCode>::default();
        let flap_value = |b: &Bindings| axis_value(b, Action::Flap);

        // One tap of F: UP -> 10°.
        keys.press(KeyCode::KeyF);
        for b in &mut bindings.list {
            b.update(&keys, 1.0 / 60.0, feel, false);
        }
        assert!((flap_value(&bindings) - 10.0 / 30.0).abs() < 1e-9);

        // Holding the key must not keep slewing past the stop.
        for _ in 0..120 {
            for b in &mut bindings.list {
                b.update(&keys, 1.0 / 60.0, feel, false);
            }
        }
        assert!((flap_value(&bindings) - 10.0 / 30.0).abs() < 1e-9);

        // Two more taps: 10 -> 20 -> 30.
        keys.release(KeyCode::KeyF);
        for b in &mut bindings.list {
            b.update(&keys, 1.0 / 60.0, feel, false);
        }
        keys.press(KeyCode::KeyF);
        for b in &mut bindings.list {
            b.update(&keys, 1.0 / 60.0, feel, false);
        }
        keys.release(KeyCode::KeyF);
        for _ in 0..3 {
            for b in &mut bindings.list {
                b.update(&keys, 1.0 / 60.0, feel, false);
            }
        }
        keys.press(KeyCode::KeyF);
        for b in &mut bindings.list {
            b.update(&keys, 1.0 / 60.0, feel, false);
        }
        assert!((flap_value(&bindings) - 1.0).abs() < 1e-9, "full detent");

        // Past the last stop, F stays at FULL...
        keys.release(KeyCode::KeyF);
        for _ in 0..3 {
            for b in &mut bindings.list {
                b.update(&keys, 1.0 / 60.0, feel, false);
            }
        }
        keys.press(KeyCode::KeyF);
        for b in &mut bindings.list {
            b.update(&keys, 1.0 / 60.0, feel, false);
        }
        assert!((flap_value(&bindings) - 1.0).abs() < 1e-9);

        // ...and V steps back down to 20°.
        keys.release(KeyCode::KeyF);
        for _ in 0..3 {
            for b in &mut bindings.list {
                b.update(&keys, 1.0 / 60.0, feel, false);
            }
        }
        keys.press(KeyCode::KeyV);
        for b in &mut bindings.list {
            b.update(&keys, 1.0 / 60.0, feel, false);
        }
        assert!((flap_value(&bindings) - 20.0 / 30.0).abs() < 1e-9);
    }
}
