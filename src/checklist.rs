//! Gamified checklists for the C172: the paper procedures the tutorial
//! teaches tick themselves off as you actually fly them.
//!
//! One active phase at a time, shown in a small panel at the screen's bottom
//! left. Items are judged straight off the sim state each frame — nothing to
//! click through, the panel is purely a feedback device:
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
//! Checkbox glyphs are plain ASCII (`[x]` / `[ ]`) — the default UI font has
//! no check mark, and the terminal look fits the status block.

use bevy::prelude::*;

use crate::app_state::AppState;
use crate::sim::{AircraftState, Sim};

#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
pub enum Phase {
    /// Nothing to show (not the C172).
    #[default]
    Off,
    ColdStart,
    BeforeTakeoff,
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
    pub throttle: f64,
    pub on_ground: bool,
    pub gs_kts: f64,
    /// True for the C172 itself; every other airframe never shows the panel.
    pub piston_c172: bool,
}

/// Advance the phase machine and evaluate the active phase's items.
/// Phase transitions happen even while items are unticked: the panel always
/// shows *something helpful* for the situation at hand.
pub fn evaluate(phase: Phase, r: &Readouts) -> (Phase, Vec<Item>) {
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
        Phase::BeforeTakeoff | Phase::Done => {
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
            .add_systems(Update, update.run_if(in_state(AppState::Flying)));
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
    let starter = sim.model.get("propulsion/engine[0]/starter-cmd-norm") > 0.5;
    let flap_cmd = sim.model.get("fcs/flap-cmd-norm");
    let readouts = Readouts {
        engine_cold: sim.engine_cold,
        master_on: aircraft.master_on,
        engine_running: aircraft.engine_running,
        starter,
        flap_cmd_norm: flap_cmd,
        throttle: aircraft.throttle,
        on_ground: aircraft.on_ground,
        gs_kts: aircraft.gs_kts,
        piston_c172,
    };
    let (phase, items) = evaluate(state.phase, &readouts);
    state.phase = phase;

    if items.is_empty() {
        node.display = Display::None;
        return;
    }
    let title = match phase {
        Phase::ColdStart => "COLD START CHECKLIST",
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
        let mut jet = r();
        jet.piston_c172 = false;
        assert_eq!(evaluate(Phase::Off, &jet).0, Phase::Off);
        assert_eq!(evaluate(Phase::ColdStart, &jet).0, Phase::Off);
    }
}
