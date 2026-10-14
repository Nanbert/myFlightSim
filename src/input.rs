//! Keyboard -> JSBSim normalized control commands, via a data-driven *binding
//! table*. Each row maps a key pair to one JSBSim property; adding a control is
//! adding a row, not writing code. That is how we track whatever the loaded
//! model exposes (flaps and brakes today, mixture/magnetos if the model ever
//! defines them).
//!
//! Sign convention comes from the c172p aero coefficients: `+elevator-cmd-norm`
//! pitches the nose DOWN (`aero/coefficient/Cmde` = -1.1220), `+aileron-cmd-norm`
//! rolls RIGHT (`Clda` = +0.2290) and `+rudder-cmd-norm` yaws LEFT
//! (`Cndr` = -0.0430). Each key is mapped to the sign that gives the intuitive
//! response, so the commands are the negative of the key's "positive" sense:
//! W pitches the nose up, S down; D rolls right, A left; Q yaws left, E right.
//! (This assumes the render is un-mirrored — see `sim::pose`.)

use bevy::prelude::*;

const SURFACE_RATE: f64 = 3.0; // full deflection per second
const THROTTLE_RATE: f64 = 0.6;
const TRIM_RATE: f64 = 0.3; // full trim range per ~3.3 s
const FLAP_RATE: f64 = 0.5; // full flap command per 2 s (the model rate-limits travel)
const BRAKE_RATE: f64 = 4.0; // brakes bite quickly

/// How a binding responds to its keys.
#[derive(Clone, Copy)]
enum Axis {
    /// Ramp toward the key direction; returns to neutral on release.
    /// Elevator, aileron, rudder, toe brakes.
    Spring,
    /// Accumulate while a key is held; hold position on release.
    /// Throttle, pitch trim, flaps.
    Hold,
}

/// One key pair driving one normalized JSBSim property.
pub struct Binding {
    /// Property written to the FDM each step, e.g. `fcs/elevator-cmd-norm`.
    pub property: &'static str,
    pos: KeyCode,
    neg: Option<KeyCode>,
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

/// The pilot's control set. Iterated each frame to drive the FDM.
#[derive(Resource)]
pub struct Bindings {
    pub list: Vec<Binding>,
}

impl Default for Bindings {
    fn default() -> Self {
        use KeyCode::*;
        // Spring axes span [-1, 1] and center at 0; Hold axes keep their value.
        let spring = |property, pos, neg, rate| Binding {
            property,
            pos,
            neg,
            rate,
            axis: Axis::Spring,
            min: -1.0,
            max: 1.0,
            value: 0.0,
            trim: 0.0,
        };
        let hold = |property, pos, neg, rate, min, max, value| Binding {
            property,
            pos,
            neg,
            rate,
            axis: Axis::Hold,
            min,
            max,
            value,
            trim: 0.0,
        };
        Bindings {
            list: vec![
                spring("fcs/elevator-cmd-norm", KeyS, Some(KeyW), SURFACE_RATE),
                spring("fcs/aileron-cmd-norm", KeyD, Some(KeyA), SURFACE_RATE),
                spring("fcs/rudder-cmd-norm", KeyQ, Some(KeyE), SURFACE_RATE),
                hold("fcs/throttle-cmd-norm", ShiftLeft, Some(ControlLeft), THROTTLE_RATE, 0.0, 1.0, 0.6),
                // Collective, on the same Shift/Ctrl keys: a helicopter's power
                // lever *is* the collective (its throttle axis is meaningless —
                // the governor holds rpm). Both properties are written; each
                // aircraft reads the one it cares about. Initial 0 matches the
                // FDM's ground-idle default, so it sits until you raise it.
                hold("fcs/collective-cmd-norm", ShiftLeft, Some(ControlLeft), THROTTLE_RATE, 0.0, 1.0, 0.0),
                hold("fcs/pitch-trim-cmd-norm", KeyZ, Some(KeyX), TRIM_RATE, -1.0, 1.0, 0.0),
                // Flaps: 0..1, the model rate-limits travel (~4 s full down).
                hold("fcs/flap-cmd-norm", KeyF, Some(KeyV), FLAP_RATE, 0.0, 1.0, 0.0),
                // Toe brakes: both wheels, spring-loaded back to off. Range is 0..1.
                Binding { property: "fcs/left-brake-cmd-norm", pos: KeyB, neg: None, rate: BRAKE_RATE, axis: Axis::Spring, min: 0.0, max: 1.0, value: 0.0, trim: 0.0 },
                Binding { property: "fcs/right-brake-cmd-norm", pos: KeyB, neg: None, rate: BRAKE_RATE, axis: Axis::Spring, min: 0.0, max: 1.0, value: 0.0, trim: 0.0 },
            ],
        }
    }
}

pub struct InputPlugin;

impl Plugin for InputPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<Bindings>()
            .add_systems(Update, read_input);
    }
}

fn read_input(
    keys: Res<ButtonInput<KeyCode>>,
    time: Res<Time>,
    mut bindings: ResMut<Bindings>,
    mut sim: ResMut<crate::sim::Sim>,
    mut ap: ResMut<crate::sim::Autopilot>,
) {
    let dt = time.delta_secs_f64();
    for b in &mut bindings.list {
        b.update(&keys, dt);
    }

    // G hands control to/from the autopilot; R parks back on the runway in auto.
    if keys.just_pressed(KeyCode::KeyG) {
        ap.enabled = !ap.enabled;
    }
    if keys.just_pressed(KeyCode::KeyR) {
        sim.reset_ground();
        *bindings = Bindings::default();
        // A helicopter has no fixed-wing autopilot to hand control back to.
        ap.enabled = !sim.model.has_rotor();
    }
    // J toggles thrust vectoring. A no-op on models without it (the c172
    // ignores the property); the F-22's FBW reads it.
    if keys.just_pressed(KeyCode::KeyJ) {
        let on = sim.model.get("fcs/tvc-enable") > 0.5;
        sim.model.set("fcs/tvc-enable", if on { 0.0 } else { 1.0 });
    }
    // U raises/lowers the gear (`gear/gear-cmd-norm`: 1 = down, 0 = up). A
    // no-op on fixed-gear aircraft; the F-22 ignores a "gear up" command while
    // on the wheels (weight-on-wheels interlock).
    if keys.just_pressed(KeyCode::KeyU) {
        let down = sim.model.get("gear/gear-cmd-norm") > 0.5;
        sim.model.set("gear/gear-cmd-norm", if down { 0.0 } else { 1.0 });
    }
    // T re-trims for the current condition; Y restarts airborne in a trimmed
    // cruise. Both hand control to the pilot and adopt the trimmed throttle/trim
    // into the Hold bindings, so the per-step command writes don't undo the trim.
    if keys.just_pressed(KeyCode::KeyT) {
        sim.trim_now();
        ap.enabled = false;
        adopt_trim(&mut bindings, &sim);
    }
    if keys.just_pressed(KeyCode::KeyY) {
        sim.reset_cruise();
        ap.enabled = false;
        adopt_trim(&mut bindings, &sim);
    }
    // Space trims a rotorcraft to hands-free level flight at the current
    // airspeed (a hover when stationary) and adopts the solved commands, so
    // releasing the sticks holds it. (A helicopter is hard to hold by hand; this
    // is the trim for it.) Fixed-wing uses T/Y above instead.
    if keys.just_pressed(KeyCode::Space) && sim.model.has_rotor() {
        sim.trim_now();
        adopt_trim(&mut bindings, &sim);
    }
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
    use super::{Axis, Binding, Bindings, adopt_trim, ramp};
    use bevy::prelude::KeyCode;

    fn cyclic() -> Binding {
        Binding {
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
        // End-to-end for the Space handler: trim the rotorcraft, adopt the
        // solved commands into the bindings, then drive the model the way
        // `advance_fdm` does (write `effective()` every step). The hover must
        // hold with the sticks centered.
        std::env::set_var("FDM_BACKEND", "rust");
        let mut sim = crate::sim::Sim::new("ah1s");
        // Airborne at the altitude the FDM's own hover test validates (2000 ft);
        // `trim_heli` seats a zero-airspeed hover there.
        sim.reset_air(2000.0, 0.0);

        let mut bindings = Bindings::default();
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

        let mut bindings = Bindings::default();
        sim.trim_now(); // what the Space handler does
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
        let mut bindings = Bindings::default();
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
        // Distance to target (0.1) is within one step (0.3): land exactly on it.
        let mut v = 0.9;
        ramp(&mut v, 1.0, 0.3);
        assert_eq!(v, 1.0);

        // Landing on target is idempotent.
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
