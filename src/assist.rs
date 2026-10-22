//! Optional keyboard assists for the C172: bank hold and turn coordination.
//!
//! Both are deliberate assists, not real C172 systems. They are off by default
//! and exposed on the aircraft's SETTINGS page. The controllers write a
//! separate `Binding::assist` term so the 2D discs can keep showing white =
//! pilot input and cyan = assist contribution, while the FDM sees the sum.
//!
//! Bank hold captures the bank angle after the pilot lets go of the ailerons,
//! then holds it with a P-D loop. Turn coordination nulls lateral
//! acceleration with a damped loop on the slip ratio and its smoothed rate,
//! constrained by a deadband and a surface slew. Both run only while the
//! pilot is not flying that axis; a deliberate input always wins.

use bevy::prelude::*;

use crate::aircraft_profile::{self, Action};
use crate::app_state::{unpaused, AppState};
use crate::config::GameConfig;
use crate::input::Bindings;
use crate::sim::Sim;

/// Runtime state of the assists, also read by the control-position discs.
#[derive(Resource, Default)]
pub struct AssistState {
    /// Bank hold is holding a captured target.
    pub bank_active: bool,
    /// Turn coordination is currently adding rudder.
    pub coord_active: bool,
    /// Current assist contributions, normalized.
    pub aileron: f64,
    pub rudder: f64,
    /// Captured bank target, for documentation/debug and the HUD.
    pub bank_target_deg: f64,
    bank_target: Option<f64>,
    capture: f64,
    /// The last slip reading, for the coordination loop's slope. `None` while
    /// the loop is silent.
    slip_prev: Option<f64>,
    /// The smoothed slip rate — the coordination loop's damping term.
    slip_rate: f64,
    /// The rudder term the last step served the FDM, the slew's anchor.
    rudder_prev: f64,
}

pub struct AssistPlugin;

impl Plugin for AssistPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<AssistState>()
            .add_systems(OnEnter(AppState::Flying), reset_assists)
            // Fixed step, ahead of the FDM tick that consumes the assist
            // term: the P-D loops assume a constant dt, and a render-rate
            // loop makes the damping term change with frame rate.
            .add_systems(FixedPreUpdate, update_assists.run_if(unpaused));
    }
}

fn reset_assists(mut state: ResMut<AssistState>) {
    *state = AssistState::default();
}

/// P-D bank hold: positive aileron rolls right.
fn bank_command(target: f64, bank: f64, roll_rate_dps: f64) -> f64 {
    (0.008 * (target - bank) - 0.012 * roll_rate_dps).clamp(-0.45, 0.45)
}

/// P-D turn coordination: positive slip -> step left (positive rudder),
/// damped by the slip's own smoothed rate. The wide deadband, the rate
/// damping and the surface slew in `update_assists` are the three legs it
/// stands on: hands off, the loop idles inside the band, and on a real
/// slip it steps once instead of ringing the rudder back and forth.
fn coord_command(slip: f64, slip_rate: f64) -> f64 {
    let slip = if slip.abs() < 0.01 { 0.0 } else { slip };
    (slip * 3.0 + slip_rate * 2.0).clamp(-0.3, 0.3)
}

fn update_assists(
    time: Res<Time<Fixed>>,
    sim: Res<Sim>,
    config: Res<GameConfig>,
    keys: Res<ButtonInput<KeyCode>>,
    mut bindings: ResMut<Bindings>,
    mut state: ResMut<AssistState>,
) {
    let dt = time.delta_secs_f64();
    let c172 = aircraft_profile::normalized(&sim.name) == "c172p";
    let cfg = config.assists(&sim.name);

    let aileron_idx = bindings
        .list
        .iter()
        .position(|b| b.action == Action::Aileron);
    let rudder_idx = bindings
        .list
        .iter()
        .position(|b| b.action == Action::Rudder);
    let pilot_aileron = aileron_idx.map_or(0.0, |i| bindings.list[i].value);
    let pilot_rudder = rudder_idx.map_or(0.0, |i| bindings.list[i].value);
    // Key state, not just the value: a fine-mode tap is only 1% of travel, so
    // a value threshold would let bank hold fight a deliberate small input.
    let aileron_key = aileron_idx.is_some_and(|i| {
        let b = &bindings.list[i];
        keys.pressed(b.pos) || b.neg.is_some_and(|k| keys.pressed(k))
    });
    let rudder_key = rudder_idx.is_some_and(|i| {
        let b = &bindings.list[i];
        keys.pressed(b.pos) || b.neg.is_some_and(|k| keys.pressed(k))
    });

    let wow = sim.model.get("gear/wow") > 0.5;
    let cas = sim.model.get("velocities/vc-kts");
    let bank = sim.model.get("attitude/phi-deg");
    let roll_rate = sim.model.get("velocities/p-rad_sec").to_degrees();
    let slip = sim.model.get("accelerations/n-pilot-y-norm");
    let airborne = !wow && cas > 40.0;

    // --- bank hold -------------------------------------------------------
    let mut aileron_target = 0.0;
    let mut bank_active = false;
    if c172 && cfg.bank_hold && airborne && bank.abs() < 60.0 {
        if pilot_aileron.abs() > 0.03 || aileron_key {
            // The pilot is flying: drop the target and let them have the roll
            // axis. It is re-captured after they let go.
            state.bank_target = None;
            state.capture = 0.0;
        } else if let Some(target) = state.bank_target {
            // Positive aileron rolls right. Error = target - bank, damped by
            // the actual roll rate so the capture does not overshoot.
            aileron_target = bank_command(target, bank, roll_rate);
            bank_active = true;
        } else {
            // Let the roll rate settle, then remember where the wings are.
            state.capture += dt;
            if state.capture >= 0.20 && roll_rate.abs() < 3.0 {
                state.bank_target = Some(if bank.abs() < 2.0 { 0.0 } else { bank });
                bank_active = true;
            }
        }
    } else {
        state.bank_target = None;
        state.capture = 0.0;
    }

    // --- turn coordination ----------------------------------------------
    // The damping term is a lightly smoothed slip slope. The fold keeps a
    // slip history only while the loop lives — a re-engage never rides a
    // stale slope.
    let mut rudder_target = 0.0;
    let mut coord_active = false;
    if c172 && cfg.turn_coord && airborne && !rudder_key && pilot_rudder.abs() < 0.03 {
        let raw_rate = state.slip_prev.map_or(0.0, |prev| (slip - prev) / dt);
        let slip_rate = state.slip_rate + (raw_rate - state.slip_rate) * 0.25;
        state.slip_prev = Some(slip);
        state.slip_rate = slip_rate;
        // `n-pilot-y-norm` is positive when the ball rolls left. Positive
        // rudder yaws left, so the proportional term has the same sign.
        rudder_target = coord_command(slip, slip_rate);
        coord_active = rudder_target.abs() > 0.005;
    } else {
        state.slip_prev = None;
        state.slip_rate = 0.0;
    }

    // Smooth engage/disengage so the surfaces do not snap.
    let blend = 1.0 - (-dt / 0.12).exp();
    state.aileron += (aileron_target - state.aileron) * blend;
    state.rudder += (rudder_target - state.rudder) * blend;
    // The coordination surface also has a hard slew: the loop may not jump
    // its rudder back and forth even transiently. 0.6/s is slower than the
    // blend on ramps and much stiffer on reversals.
    let slew = 0.6 * dt;
    state.rudder = state
        .rudder
        .clamp(state.rudder_prev - slew, state.rudder_prev + slew);
    state.rudder_prev = state.rudder;
    if !c172 || !cfg.bank_hold {
        state.aileron = 0.0;
        bank_active = false;
        state.bank_target = None;
        state.capture = 0.0;
    }
    if !c172 || !cfg.turn_coord {
        state.rudder = 0.0;
        state.rudder_prev = 0.0;
        state.slip_prev = None;
        state.slip_rate = 0.0;
        coord_active = false;
    }
    state.bank_active = bank_active;
    state.coord_active = coord_active;
    state.bank_target_deg = state.bank_target.unwrap_or(0.0);

    if let Some(i) = aileron_idx {
        bindings.list[i].assist = state.aileron;
    }
    if let Some(i) = rudder_idx {
        bindings.list[i].assist = state.rudder;
    }
}

#[cfg(test)]
mod tests {
    use super::{bank_command, coord_command};

    #[test]
    fn bank_hold_asks_for_the_side_it_needs() {
        // Below target: roll right (positive aileron).
        assert!(bank_command(20.0, 10.0, 0.0) > 0.0);
        // Above target: roll left.
        assert!(bank_command(10.0, 20.0, 0.0) < 0.0);
        // A right roll rate damps a right-roll command.
        assert!(bank_command(20.0, 10.0, 30.0) < bank_command(20.0, 10.0, 0.0));
        // And it clamps.
        assert!(bank_command(90.0, -90.0, 0.0) <= 0.45);
        assert!(bank_command(-90.0, 90.0, 0.0) >= -0.45);
    }

    #[test]
    fn turn_coordination_steps_on_the_ball() {
        // Positive slip rolls the ball left, so the assist adds left rudder
        // (positive command).
        assert!(coord_command(0.02, 0.0) > 0.0);
        assert!(coord_command(-0.02, 0.0) < 0.0);
        // Inside the (deliberately wide) band the loop stays silent.
        assert_eq!(coord_command(0.005, 0.0), 0.0);
        // Standard PD: a slip still rising feeds the command more; the plant
        // then opposes the motion, which is the damping leg.
        assert!(coord_command(0.02, 0.05) > coord_command(0.02, 0.0));
        assert!(coord_command(0.02, -0.05) < coord_command(0.02, 0.0));
        // And it clamps, softer than the old ±0.4.
        assert!(coord_command(1.0, 0.0) <= 0.3);
        assert!(coord_command(-1.0, 0.0) >= -0.3);
    }
}
