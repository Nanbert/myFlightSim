//! Gamified checklists: the paper procedures the tutorial teaches tick
//! themselves off as you actually fly them.
//!
//! One active phase at a time, shown in a small panel at the screen's bottom
//! left. Items are judged straight off the sim state each frame — nothing to
//! click through, the panel is purely a feedback device.
//!
//! **C172 flow:**
//!
//! - `COLD START` (ENGINE = COLD): master on, starter crank (hold I), engine
//!   running. Inactive unless the cold-start flow is engaged, so the default
//!   running spawn never nags about a master switch.
//! - `BEFORE TAKEOFF`: flaps 10 (the lever's second detent), throttle full,
//!   then airborne. Fires on every ground start — warm or cold — and hides
//!   once the wheels leave.
//!
//! A mid-air flame-out restarts COLD START on its own, because the phases are
//! derived from live state, not a script: that is exactly the right checklist
//! to be looking at when the prop stops.
//!
//! **737 flow** (runway spawn, engines already turning):
//!
//! - `BEFORE TAKEOFF`: flaps 5 (of the wing's 40), throttle full.
//! - `AFTER TAKEOFF`: positive rate, gear up, flaps up. The panels hand over
//!   on the same derived-state transitions the real flow uses, so a
//!   touch-and-go rolls the takeoff checklist back open by itself.
//!
//! Checkbox glyphs are plain ASCII (`[x]` / `[ ]`) — the default UI font has
//! no check mark, and the terminal look fits the status block.

use bevy::prelude::*;

use crate::app_state::{unpaused, AppState};
use crate::sim::{AircraftState, Sim};

#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
pub enum Phase {
    /// Nothing to show (no aircraft flow applies).
    #[default]
    Off,
    ColdStart,
    BeforeTakeoff,
    /// The 737's clean-up card after the wheels leave.
    AfterTakeoff,
    /// Completed; the panel hides until the next flight resets the flow.
    Done,
}

/// A phase's items in display order, evaluated against one frame's readouts.
pub struct Item {
    pub label: &'static str,
    pub done: bool,
}

/// The state one evaluation needs, gathered in `update_cold_start`'s caller.
/// Kept plain so the phase machine is unit-testable without an App.
#[derive(Default)]
pub struct Readouts {
    pub engine_cold: bool,
    pub master_on: bool,
    pub engine_running: bool,
    /// Starter motor engaged (`propulsion/engine[0]/starter-cmd-norm`).
    pub starter: bool,
    /// Commanded flap angle (`fcs/flap-cmd-norm` × 30 deg).
    pub flap_cmd_norm: f64,
    /// Actual flap surface deflection, degrees (`aircraft.flap_deg`). The 737
    /// items judge the trailing edge itself — the continuous lever has no
    /// detents to aim for.
    pub flap_deg: f64,
    pub throttle: f64,
    pub on_ground: bool,
    pub gs_kts: f64,
    /// Airspeed (CAS, kt). The 737's clean-up items are speed-aware.
    pub cas_kts: f64,
    /// Vertical speed, ft/min.
    pub vs_fpm: f64,
    /// Achieved landing-gear position: 1 = down, 0 = up.
    pub gear_pos: f64,
    /// True for the C172 itself.
    pub piston_c172: bool,
    /// True for the 737.
    pub jet_737: bool,
}

/// Advance the phase machine and evaluate the active phase's items.
/// Phase transitions happen even while items are unticked: the panel always
/// shows *something helpful* for the situation at hand.
pub fn evaluate(phase: Phase, r: &Readouts) -> (Phase, Vec<Item>) {
    if r.jet_737 {
        return evaluate_jet(phase, r);
    }
    if !r.piston_c172 {
        return (Phase::Off, Vec::new());
    }
    match phase {
        Phase::Off => {
            if r.engine_cold && !r.engine_running {
                cold_start(r)
            } else {
                before_takeoff(r)
            }
        }
        Phase::ColdStart => {
            if r.engine_running {
                before_takeoff(r)
            } else {
                cold_start(r)
            }
        }
        Phase::BeforeTakeoff | Phase::AfterTakeoff | Phase::Done => {
            // Started and away? The pre-flight flow is finished; back to
            // nothing until the next see-off restarts it.
            if !r.on_ground && r.gs_kts > 20.0 {
                return (Phase::Done, Vec::new());
            }
            // Cold and stopped again (a flame-out, or a shutdown between
            // flights): the start checklist is the right paper to hold.
            if r.engine_cold && !r.engine_running {
                cold_start(r)
            } else {
                before_takeoff(r)
            }
        }
    }
}

fn cold_start(r: &Readouts) -> (Phase, Vec<Item>) {
    (
        Phase::ColdStart,
        vec![
            Item {
                label: "MASTER ON",
                done: r.master_on,
            },
            Item {
                label: "CRANK STARTER (HOLD I)",
                done: r.starter || r.engine_running,
            },
            Item {
                label: "ENGINE RUNNING",
                done: r.engine_running,
            },
        ],
    )
}

fn before_takeoff(r: &Readouts) -> (Phase, Vec<Item>) {
    (
        Phase::BeforeTakeoff,
        vec![
            Item {
                label: "FLAPS 10 DEG",
                done: (r.flap_cmd_norm - 1.0 / 3.0).abs() < 0.02,
            },
            Item {
                label: "THROTTLE FULL",
                done: r.throttle > 0.95,
            },
        ],
    )
}

// --- 737 ----------------------------------------------------------------------

/// The runway card: flaps 5 is the takeoff book setting the tutorial preaches.
/// The jet's lever is continuous (0..40 deg), so the tick reads the trailing
/// edge itself with a hand-wide tolerance.
fn jet_before_takeoff(r: &Readouts) -> (Phase, Vec<Item>) {
    (
        Phase::BeforeTakeoff,
        vec![
            Item {
                label: "FLAPS 5 DEG",
                done: (r.flap_deg - 5.0).abs() < 1.5,
            },
            Item {
                label: "THROTTLE FULL",
                done: r.throttle > 0.95,
            },
        ],
    )
}

/// The clean-up card: positive rate, gear up, flaps up. Speed-pressure items
/// come later with the flap placard work; these three are unambiguous.
fn jet_after_takeoff(r: &Readouts) -> (Phase, Vec<Item>) {
    (
        Phase::AfterTakeoff,
        vec![
            Item {
                label: "POSITIVE RATE",
                done: r.vs_fpm > 100.0,
            },
            Item {
                label: "GEAR UP",
                done: r.gear_pos < 0.05,
            },
            Item {
                label: "FLAPS UP",
                done: r.flap_deg < 1.0,
            },
        ],
    )
}

/// The 737 machine. `Off`/`BeforeTakeoff`/`ColdStart` all resolve off the live
/// state — the jet never cold-starts in this sim, so any ground moment is the
/// takeoff card, and a touch-and-go rolls it back open by itself. `Done` stays
/// quiet in the air (approach flap is not a clean-up item) and lets the next
/// ground moment restart the flow.
fn evaluate_jet(phase: Phase, r: &Readouts) -> (Phase, Vec<Item>) {
    match phase {
        Phase::AfterTakeoff => {
            if r.on_ground {
                jet_before_takeoff(r)
            } else {
                let (phase, items) = jet_after_takeoff(r);
                if items.iter().all(|i| i.done) {
                    (Phase::Done, Vec::new())
                } else {
                    (phase, items)
                }
            }
        }
        Phase::Done => {
            if r.on_ground {
                jet_before_takeoff(r)
            } else {
                (Phase::Done, Vec::new())
            }
        }
        _ => {
            if !r.on_ground && r.cas_kts > 60.0 {
                jet_after_takeoff(r)
            } else {
                jet_before_takeoff(r)
            }
        }
    }
}


pub struct ChecklistPlugin;

/// Panel entity marker.
#[derive(Component)]
struct ChecklistPanel;

#[derive(Resource, Default)]
struct ChecklistState {
    phase: Phase,
}

impl Plugin for ChecklistPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<ChecklistState>()
            .add_systems(Startup, spawn_panel)
            .add_systems(OnEnter(AppState::Flying), reset_checklist)
            .add_systems(Update, update.run_if(unpaused));
    }
}

/// A fresh flight starts with no checklist showing; `evaluate` re-derives the
/// right phase (cold spawn -> COLD START, running spawn -> BEFORE TAKEOFF)
/// from the live state on the first frame.
fn reset_checklist(mut state: ResMut<ChecklistState>) {
    state.phase = Phase::Off;
}

fn spawn_panel(mut commands: Commands) {
    commands.spawn((
        ChecklistPanel,
        Text::new(""),
        TextFont {
            font_size: FontSize::Px(14.0),
            ..default()
        },
        TextColor(Color::srgb(0.92, 0.94, 0.90)),
        BackgroundColor(Color::srgba(0.02, 0.03, 0.04, 0.80)),
        Node {
            position_type: PositionType::Absolute,
            left: Val::Px(18.0),
            bottom: Val::Px(370.0),
            padding: UiRect::all(Val::Px(10.0)),
            border: UiRect::all(Val::Px(1.0)),
            display: Display::None,
            ..default()
        },
        BorderColor::all(Color::srgba(0.7, 0.75, 0.7, 0.35)),
        GlobalZIndex(14),
        Pickable::IGNORE,
    ));
}

fn update(
    sim: Res<Sim>,
    aircraft: Res<AircraftState>,
    mut state: ResMut<ChecklistState>,
    mut panel: Query<(&mut Text, &mut Node), With<ChecklistPanel>>,
) {
    let Ok((mut text, mut node)) = panel.single_mut() else {
        return;
    };
    let piston_c172 = crate::aircraft_profile::normalized(&sim.name) == "c172p";
    let jet_737 = crate::aircraft_profile::normalized(&sim.name) == "737";
    let starter = sim.model.get("propulsion/engine[0]/starter-cmd-norm") > 0.5;
    let flap_cmd = sim.model.get("fcs/flap-cmd-norm");
    let readouts = Readouts {
        engine_cold: sim.engine_cold,
        master_on: aircraft.master_on,
        engine_running: aircraft.engine_running,
        starter,
        flap_cmd_norm: flap_cmd,
        flap_deg: aircraft.flap_deg,
        throttle: aircraft.throttle,
        on_ground: aircraft.on_ground,
        gs_kts: aircraft.gs_kts,
        cas_kts: aircraft.cas_kts,
        vs_fpm: aircraft.vs_fpm,
        gear_pos: aircraft.gear_pos,
        piston_c172,
        jet_737,
    };
    let (phase, items) = evaluate(state.phase, &readouts);
    state.phase = phase;

    if items.is_empty() {
        node.display = Display::None;
        return;
    }
    // The 737's corner PFDs share the bottom-left corner: lift the panel
    // above the overlay (400×430 at 12 px margins) while they are showing.
    // The C172 has no PFD, so the panel keeps its original perch.
    let perch = if jet_737 { 462.0 } else { 370.0 };
    if node.bottom != Val::Px(perch) {
        node.bottom = Val::Px(perch);
    }
    let title = match phase {
        Phase::ColdStart => "COLD START CHECKLIST",
        Phase::AfterTakeoff => "AFTER TAKEOFF",
        _ => "BEFORE TAKEOFF",
    };
    let mut out = format!("{title}\n");
    for item in &items {
        out.push_str(if item.done { "[x] " } else { "[ ] " });
        out.push_str(item.label);
        out.push('\n');
    }
    text.0 = out;
    node.display = Display::Flex;
}

#[cfg(test)]
mod tests {
    use super::{evaluate, Item, Phase, Readouts};
    use bevy::prelude::default;

    fn r() -> Readouts {
        Readouts {
            piston_c172: true,
            ..default()
        }
    }

    /// A 737 on the runway, engines turning.
    fn j() -> Readouts {
        Readouts {
            jet_737: true,
            on_ground: true,
            ..default()
        }
    }

    fn ticks(items: &[Item]) -> String {
        items
            .iter()
            .map(|i| if i.done { "x" } else { " " })
            .collect()
    }

    #[test]
    fn a_warm_spawn_reaches_before_takeoff_directly() {
        let (phase, items) = evaluate(Phase::Off, &r());
        assert_eq!(phase, Phase::BeforeTakeoff);
        assert_eq!(items.len(), 2, "flaps + throttle, no start items");
        assert_eq!(ticks(&items), "  ");
    }

    #[test]
    fn a_cold_spawn_walks_the_start_checklist() {
        // Cold and dark: the start items first.
        let mut cold = r();
        cold.engine_cold = true;
        let (phase, items) = evaluate(Phase::Off, &cold);
        assert_eq!(phase, Phase::ColdStart);
        assert_eq!(items.len(), 3);
        assert_eq!(ticks(&items), "   ");

        // Master on: first item ticks, the rest wait.
        cold.master_on = true;
        let (_, items) = evaluate(Phase::Off, &cold);
        assert_eq!(ticks(&items), "x  ");

        // Cranking: second item ticks while the prop swings.
        cold.starter = true;
        let (_, items) = evaluate(Phase::ColdStart, &cold);
        assert_eq!(ticks(&items), "xx ");
        assert_eq!(evaluate(Phase::ColdStart, &cold).0, Phase::ColdStart);

        // She fires: the phase rolls forward to BEFORE TAKEOFF.
        cold.engine_running = true;
        let (phase, items) = evaluate(Phase::ColdStart, &cold);
        assert_eq!(phase, Phase::BeforeTakeoff);
        assert_eq!(items.len(), 2);
    }

    #[test]
    fn before_takeoff_closes_when_airborne() {
        let mut flying = r();
        flying.flap_cmd_norm = 1.0 / 3.0;
        flying.throttle = 1.0;
        // Still on the runway: both items tick but the flow is open.
        let (phase, items) = evaluate(Phase::BeforeTakeoff, &flying);
        assert_eq!(phase, Phase::BeforeTakeoff);
        assert_eq!(ticks(&items), "xx");
        // Wheels off: done, and the panel goes quiet.
        flying.on_ground = false;
        flying.gs_kts = 40.0;
        let (phase, items) = evaluate(Phase::BeforeTakeoff, &flying);
        assert_eq!(phase, Phase::Done);
        assert!(items.is_empty());
    }

    #[test]
    fn a_flameout_reopens_the_start_checklist() {
        let mut dead = r();
        dead.engine_cold = true;
        let (phase, items) = evaluate(Phase::BeforeTakeoff, &dead);
        assert_eq!(phase, Phase::ColdStart);
        assert_eq!(items.len(), 3);
    }

    #[test]
    fn other_aircraft_never_show_a_panel() {
        // Neither the C172 nor the 737: the F-22 and the Apache stay quiet.
        let mut other = r();
        other.piston_c172 = false;
        other.jet_737 = false;
        assert_eq!(evaluate(Phase::Off, &other).0, Phase::Off);
        assert_eq!(evaluate(Phase::ColdStart, &other).0, Phase::Off);
    }

    #[test]
    fn the_737_ground_card_is_flaps_five_and_full_throttle() {
        let (phase, items) = evaluate(Phase::Off, &j());
        assert_eq!(phase, Phase::BeforeTakeoff);
        assert_eq!(items.len(), 2);
        assert_eq!(ticks(&items), "  ");

        // Flaps 5 of the wing's 40 and throttles to the stops: both tick.
        let mut set = j();
        set.flap_deg = 5.0;
        set.throttle = 1.0;
        let (_, items) = evaluate(Phase::Off, &set);
        assert_eq!(ticks(&items), "xx");
    }

    #[test]
    fn the_737_clean_up_card_follows_the_climb() {
        // Fresh airborne with the gear and flaps still out: only POSITIVE
        // RATE ticks.
        let mut climb = j();
        climb.on_ground = false;
        climb.cas_kts = 150.0;
        climb.flap_deg = 5.0;
        climb.gear_pos = 1.0;
        climb.vs_fpm = 1500.0;
        let (phase, items) = evaluate(Phase::Off, &climb);
        assert_eq!(phase, Phase::AfterTakeoff);
        assert_eq!(ticks(&items), "x  ");

        // She sinks for a moment: the rate item unticks itself again.
        climb.vs_fpm = -200.0;
        let (_, items) = evaluate(Phase::AfterTakeoff, &climb);
        assert_eq!(ticks(&items), "   ");

        // Gear and flaps up: the card closes on its own.
        climb.vs_fpm = 1500.0;
        climb.gear_pos = 0.0;
        climb.flap_deg = 0.0;
        let (phase, items) = evaluate(Phase::AfterTakeoff, &climb);
        assert_eq!(phase, Phase::Done);
        assert!(items.is_empty());
    }

    #[test]
    fn the_737_done_card_stays_quiet_until_the_next_ground_moment() {
        // On approach the pilot lowers the flaps and the gear: that is not a
        // clean-up item, so the done card must not reopen.
        let mut approach = j();
        approach.on_ground = false;
        approach.cas_kts = 140.0;
        approach.flap_deg = 30.0;
        approach.gear_pos = 1.0;
        assert_eq!(evaluate(Phase::Done, &approach).0, Phase::Done);

        // Wheels down for real: the takeoff card rolls back open, which is
        // exactly what a touch-and-go wants.
        let (phase, items) = evaluate(Phase::Done, &j());
        assert_eq!(phase, Phase::BeforeTakeoff);
        assert_eq!(items.len(), 2);
    }
}
