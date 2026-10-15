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
use crate::config::{GameConfig, Keymap};

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
}

pub struct InputPlugin;

impl Plugin for InputPlugin {
    fn build(&self, app: &mut App) {
        // `Sim` is inserted by `SimPlugin`, which is registered first, so the
        // startup aircraft is already known here.
        let name = app.world().resource::<crate::sim::Sim>().name.clone();
        let keymap = app.world().resource::<GameConfig>().keymap(&name);
        app.insert_resource(Bindings::for_aircraft(&name, &keymap))
            .add_systems(Update, read_input.run_if(in_state(AppState::Flying)));
    }
}

fn read_input(
    keys: Res<ButtonInput<KeyCode>>,
    time: Res<Time>,
    mut bindings: ResMut<Bindings>,
    mut sim: ResMut<crate::sim::Sim>,
    mut ap: ResMut<crate::sim::Autopilot>,
    config: Res<GameConfig>,
) {
    let dt = time.delta_secs_f64();
    for b in &mut bindings.list {
        b.update(&keys, dt);
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
            ToggleAction::Autopilot => ap.enabled = !ap.enabled,
            // A no-op on models without these (the FDM ignores the property).
            ToggleAction::Tvc => toggle_prop(&mut sim, "fcs/tvc-enable"),
            ToggleAction::Gear => toggle_prop(&mut sim, "gear/gear-cmd-norm"),
            // Re-trim for the current condition; hand control to the pilot and
            // adopt the trimmed commands so the per-step writes don't undo it.
            ToggleAction::Trim => {
                sim.trim_now();
                ap.enabled = false;
                adopt_trim(&mut bindings, &sim);
            }
            ToggleAction::Cruise => {
                sim.reset_cruise();
                ap.enabled = false;
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
        *bindings = Bindings::for_aircraft(&sim.name, &config.keymap(&sim.name));
        ap.enabled = aircraft_profile::profile(&sim.name).has_autopilot;
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
    use super::{Binding, Bindings, adopt_trim, ramp};
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
}
