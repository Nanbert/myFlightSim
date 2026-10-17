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
use crate::app_state::AppState;
use crate::autopilot::{Autopilot, Mcp};
use crate::config::{GameConfig, Keymap};

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
    /// The axis's starting setting, e.g. cruise throttle. Also the cruise anchor
    /// the arcade auto-throttle falls back to under a hard manoeuvre.
    initial: f64,
    /// Current normalized value in `[min, max]`.
    pub value: f64,
    /// Trim offset added on top of `value`. Zero for every axis the FDM trims
    /// itself; used only for a rotorcraft's cyclic, which has no FDM trim
    /// channel — see [`Binding::effective`] and `adopt_trim`.
    pub trim: f64,
}

impl Binding {
    /// What actually drives the FDM: the stick value plus the trim offset,
    /// clamped to the axis range. For a trimmed rotorcraft a centered stick
    /// (value 0) then holds the solved cyclic.
    pub fn effective(&self) -> f64 {
        (self.value + self.trim).clamp(self.min, self.max)
    }

    /// +1 / -1 / 0 from which of the bound keys are down.
    fn direction(&self, keys: &ButtonInput<KeyCode>) -> f64 {
        let pos = keys.pressed(self.pos) as i32;
        let neg = self.neg.map_or(0, |k| keys.pressed(k) as i32);
        (pos - neg) as f64
    }

    fn update(&mut self, keys: &ButtonInput<KeyCode>, dt: f64) {
        let dir = self.direction(keys);
        match self.axis {
            Axis::Spring => ramp(&mut self.value, dir, self.rate * dt),
            Axis::Hold => {
                self.value = (self.value + dir * self.rate * dt).clamp(self.min, self.max)
            }
        }
    }
}

/// The pilot's control set for one aircraft. Iterated each frame to drive the
/// FDM.
#[derive(Resource)]
pub struct Bindings {
    pub list: Vec<Binding>,
    pub toggles: Vec<ToggleSpec>,
}

impl Bindings {
    /// Build the runtime control set for `id` from its profile, applying the
    /// pilot's saved key overrides (empty `keymap` = pure defaults).
    pub fn for_aircraft(id: &str, keymap: &Keymap) -> Self {
        let p = aircraft_profile::profile(id);
        let list = p
            .bindings
            .iter()
            .map(|s| {
                let (pos, neg) = keymap.axis(s.action).unwrap_or((s.pos, s.neg));
                Binding {
                    action: s.action,
                    property: s.property,
                    pos,
                    neg,
                    rate: s.rate,
                    axis: s.axis,
                    min: s.min,
                    max: s.max,
                    initial: s.initial,
                    value: s.initial,
                    trim: 0.0,
                }
            })
            .collect();
        let toggles = p
            .toggles
            .iter()
            .map(|t| ToggleSpec {
                action: t.action,
                key: keymap.toggle(t.action).unwrap_or(t.key),
            })
            .collect();
        Bindings { list, toggles }
    }

    /// Give the throttle the arcade boost feel for aircraft whose profile offers
    /// arcade mode (the F-22): a tap spools it to full and it *stays* there, so
    /// the burner doesn't need a key held down. The spool just runs faster than
    /// the realistic lever. A no-op on every other aircraft.
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
        let keymap = app.world().resource::<GameConfig>().keymap(&name);
        let arcade = app.world().resource::<GameConfig>().arcade(&name);
        let mut bindings = Bindings::for_aircraft(&name, &keymap);
        if arcade {
            bindings.enable_arcade(&name);
        }
        app.insert_resource(bindings)
            .add_systems(Update, read_input.run_if(in_state(AppState::Flying)));
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
) {
    let dt = time.delta_secs_f64();
    for b in &mut bindings.list {
        b.update(&keys, dt);
    }

    // The arcade throttle is automatic in the air: left alone in straight flight
    // it spools itself up to full reheat, and a hard pull or yaw drains it back
    // toward cruise, where the burner dies. Space tops it up on demand. On the
    // ground it stays a plain lever, so the jet still taxis and lands. Roll is
    // ignored — a pure roll doesn't swing the flight path.
    if config.arcade(&sim.name) && sim.model.get("gear/wow") < 0.5 {
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
            ToggleAction::Cruise => {
                sim.reset_cruise();
                ap.disengage();
                adopt_trim(&mut bindings, &sim);
            }
            ToggleAction::RotorTrim => {
                sim.trim_now();
                adopt_trim(&mut bindings, &sim);
            }
        }
    }

    // Global (not per-aircraft): R parks on the runway with fresh controls.
    if keys.just_pressed(KeyCode::KeyR) {
        sim.reset_ground();
        let mut rebuilt = Bindings::for_aircraft(&sim.name, &config.keymap(&sim.name));
        if config.arcade(&sim.name) {
            rebuilt.enable_arcade(&sim.name);
        }
        *bindings = rebuilt;
        ap.reset();
        mcp.reset();
    }
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
            "fcs/throttle-cmd-norm"
            | "fcs/collective-cmd-norm"
            | "fcs/pitch-trim-cmd-norm" => b.value = sim.model.get(b.property),
            "fcs/elevator-cmd-norm"
            | "fcs/aileron-cmd-norm"
            | "fcs/rudder-cmd-norm"
                if rotor =>
            {
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
    use super::{Binding, Bindings, adopt_trim, arcade_throttle, manoeuvre_intensity, ramp};
    use crate::aircraft_profile::{Action, Axis};
    use crate::config::Keymap;
    use bevy::prelude::KeyCode;

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
            initial: 0.0,
            value: 0.0,
            trim: 0.0,
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

        let mut bindings = Bindings::for_aircraft("ah1s", &Keymap::default());
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

        let mut bindings = Bindings::for_aircraft("ah1s", &Keymap::default());
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
        let mut bindings = Bindings::for_aircraft("ah1s", &Keymap::default());
        sim.trim_now();
        adopt_trim(&mut bindings, &sim);

        let dt = 1.0 / 120.0;
        let mut held = ButtonInput::<KeyCode>::default();
        held.press(KeyCode::KeyS);

        for _ in 0..120 * 3 {
            for b in &mut bindings.list {
                b.update(&held, dt);
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
        let mut bindings = Bindings::for_aircraft("f22", &Keymap::default());
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
                b.update(&keys, dt);
            }
        }
        assert!(throttle_value(&bindings) > 0.99, "boost never reached full");

        // Release: it holds at full — no key needed to keep the thrust.
        keys.release(KeyCode::Space);
        for _ in 0..240 {
            for b in &mut bindings.list {
                b.update(&keys, dt);
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
        let mut bindings = Bindings::for_aircraft("c172p", &Keymap::default());
        bindings.enable_arcade("c172p");
        let throttle = bindings
            .list
            .iter()
            .find(|b| b.action == Action::Throttle)
            .unwrap();
        assert_eq!(throttle.axis, Axis::Hold);
        assert_eq!(throttle.pos, KeyCode::ShiftLeft);
    }
}
