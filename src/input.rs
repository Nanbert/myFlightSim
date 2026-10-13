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
}

impl Binding {
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
        };
        Bindings {
            list: vec![
                spring("fcs/elevator-cmd-norm", KeyS, Some(KeyW), SURFACE_RATE),
                spring("fcs/aileron-cmd-norm", KeyD, Some(KeyA), SURFACE_RATE),
                spring("fcs/rudder-cmd-norm", KeyQ, Some(KeyE), SURFACE_RATE),
                hold("fcs/throttle-cmd-norm", ShiftLeft, Some(ControlLeft), THROTTLE_RATE, 0.0, 1.0, 0.6),
                hold("fcs/pitch-trim-cmd-norm", KeyZ, Some(KeyX), TRIM_RATE, -1.0, 1.0, 0.0),
                // Flaps: 0..1, the model rate-limits travel (~4 s full down).
                hold("fcs/flap-cmd-norm", KeyF, Some(KeyV), FLAP_RATE, 0.0, 1.0, 0.0),
                // Toe brakes: both wheels, spring-loaded back to off. Range is 0..1.
                Binding { property: "fcs/left-brake-cmd-norm", pos: KeyB, neg: None, rate: BRAKE_RATE, axis: Axis::Spring, min: 0.0, max: 1.0, value: 0.0 },
                Binding { property: "fcs/right-brake-cmd-norm", pos: KeyB, neg: None, rate: BRAKE_RATE, axis: Axis::Spring, min: 0.0, max: 1.0, value: 0.0 },
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
        ap.enabled = true;
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
    use super::ramp;

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
