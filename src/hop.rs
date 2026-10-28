//! The hop ledger: a career line for the 737's gate-to-gate competition.
//!
//! A *hop* is one graded landing in a flight (the 737 spawns on the runway,
//! so the landing closes the loop the checklist, takeoff card and stability
//! gate opened). When the pilot leaves the Flying state with a graded
//! landing behind, the ledger counts the hop and updates:
//!
//! - `hops` — ever graded;
//! - best takeoff / landing scores and the compounds: a *green hop* needs
//!   the landing inside the GOOD band (75+) plus the takeoff and the 1000 ft
//!   stability gate whenever those were actually graded. Consecutive green
//!   hops form the streak the ledger displays;
//! - the best streak ever held.
//!
//! Storage rides the same `ConfigStore` seam as the settings (`config.rs`):
//! a file in the XDG directory on the desktop, a `localStorage` key on the
//! web — hand-rolled text (no serde), written once at flight end.

use bevy::prelude::*;

use crate::app_state::{unpaused, AppState};
use crate::approach::ApproachState;
#[cfg(not(target_arch = "wasm32"))]
use crate::config::{app_dir, FileStore};
#[cfg(target_arch = "wasm32")]
use crate::config::{LEDGER_KEY, LocalStorageStore};
use crate::config::ConfigStore;
use crate::landing::LandingState;
use crate::sim::Sim;
use crate::takeoff::TakeoffState;
use crate::ui_theme;

/// The lifetime aggregates across all hops.
#[derive(Clone, Copy, Default, PartialEq, Debug)]
pub struct Ledger {
    pub hops: u32,
    /// Best graded takeoff / landing scores ever.
    pub best_takeoff: Option<u32>,
    pub best_landing: Option<u32>,
    /// Consecutive green hops currently held.
    pub green_streak: u32,
    /// The best the streak has ever been.
    pub best_streak: u32,
}

/// Fold one flight's results into the aggregates. Pulled out for unit tests:
/// `takeoff`/`gate` are `None` when that card never graded during the flight.
pub fn record(mut ledger: Ledger, landing: Option<u32>, takeoff: Option<u32>, gate: Option<bool>) -> Ledger {
    let Some(landing) = landing else {
        return ledger;
    };
    ledger.hops += 1;
    ledger.best_landing = Some(ledger.best_landing.map_or(landing, |b| b.max(landing)));
    if let Some(takeoff) = takeoff {
        ledger.best_takeoff = Some(ledger.best_takeoff.map_or(takeoff, |b| b.max(takeoff)));
    }
    let green = landing >= 75 && takeoff.is_none_or(|t| t >= 75) && gate.unwrap_or(true);
    ledger.green_streak = if green { ledger.green_streak + 1 } else { 0 };
    ledger.best_streak = ledger.best_streak.max(ledger.green_streak);
    ledger
}

/// Parse the text format. Tolerant: missing or corrupt lines simply fall back
/// to their default, so an older (or hand-edited) file still loads.
pub fn parse(text: &str) -> Ledger {
    let mut ledger = Ledger::default();
    for line in text.lines() {
        let mut words = line.split_whitespace();
        let (key, val) = match (words.next(), words.next().and_then(|v| v.parse().ok())) {
            (Some(k), Some(v)) => (k, v),
            _ => continue,
        };
        match key {
            "hops" => ledger.hops = val,
            "took" => ledger.best_takeoff = Some(val),
            "land" => ledger.best_landing = Some(val),
            "streak" => ledger.green_streak = val,
            "beststreak" => ledger.best_streak = val,
            _ => {}
        }
    }
    ledger
}

/// Serialize. Free-form words mirror `config.cfg`'s plain "key value" look.
pub fn render(ledger: &Ledger) -> String {
    let mut out = String::new();
    out.push_str(&format!("hops {}\n", ledger.hops));
    if let Some(t) = ledger.best_takeoff {
        out.push_str(&format!("took {t}\n"));
    }
    if let Some(l) = ledger.best_landing {
        out.push_str(&format!("land {l}\n"));
    }
    out.push_str(&format!("streak {}\n", ledger.green_streak));
    out.push_str(&format!("beststreak {}\n", ledger.best_streak));
    out
}

/// The boxed ledger store, mirroring `ConfigSink` in `config.rs`.
#[derive(Resource)]
pub struct LedgerSink(pub Box<dyn ConfigStore>);

/// The store this build persists the ledger through: `ledger.cfg` on the
/// desktop, a `localStorage` key on the web.
fn ledger_store() -> Box<dyn ConfigStore> {
    #[cfg(not(target_arch = "wasm32"))]
    {
        Box::new(FileStore::at(app_dir().join("ledger.cfg")))
    }
    #[cfg(target_arch = "wasm32")]
    {
        Box::new(LocalStorageStore::new(LEDGER_KEY))
    }
}

#[derive(Resource, Default)]
pub struct HopState {
    ledger: Ledger,
    /// Whether a graded landing has already cashed in this flight: a
    /// touch-and-go second landing re-counts, which is the point.
    landed: bool,
}

#[derive(Component, Default)]
struct LedgerPanel;

pub struct HopPlugin;

impl Plugin for HopPlugin {
    fn build(&self, app: &mut App) {
        let store = ledger_store();
        let ledger = store.load().map(|t| parse(&t)).unwrap_or_default();
        app.insert_resource(HopState { ledger, landed: false })
            .insert_resource(LedgerSink(store))
            .add_systems(Startup, spawn_panel)
            .add_systems(OnEnter(AppState::Flying), reset_panel)
            .add_systems(Update, update_panel.run_if(unpaused))
            .add_systems(OnExit(AppState::Flying), flush);
    }
}

fn spawn_panel(mut commands: Commands) {
    ui_theme::float_report::<LedgerPanel>(&mut commands, ui_theme::ReportAnchor::RightTop(320.0));
}

/// Only the 737 subscribes to the ledger for now — the grades it reads are
/// 737-shaped.
fn update_panel(
    sim: Res<Sim>,
    state: Res<HopState>,
    mut panel: Query<(&mut Text, &mut Node), With<LedgerPanel>>,
) {
    let Ok((mut text, mut node)) = panel.single_mut() else {
        return;
    };
    if crate::aircraft_profile::normalized(&sim.name) != "737" {
        node.display = Display::None;
        return;
    }
    let l = &state.ledger;
    text.0 = format!(
        "HOP LEDGER\n\
         HOPS      {:>4}\n\
         BEST TO   {:>4}\n\
         BEST LDG  {:>4}\n\
         GREEN     {} / BEST {}",
        l.hops,
        l.best_takeoff.unwrap_or(0),
        l.best_landing.unwrap_or(0),
        l.green_streak,
        l.best_streak,
    );
    node.display = Display::Flex;
}

/// Flight over: fold this flight's grades into the ledger and save once.
fn flush(
    landing: Res<LandingState>,
    takeoff: Res<TakeoffState>,
    approach: Res<ApproachState>,
    mut state: ResMut<HopState>,
    sink: Res<LedgerSink>,
) {
    if state.landed || landing.last.is_none() {
        return;
    }
    state.landed = true;
    state.ledger = record(state.ledger, landing.last, takeoff.last, approach.stable);
    sink.0.save(&render(&state.ledger));
}

/// A fresh flight only clears the this-flight landed flag; the ledger itself
/// was loaded at startup and is shared across flights.
fn reset_panel(mut state: ResMut<HopState>) {
    state.landed = false;
}

#[cfg(test)]
mod tests {
    use super::{parse, record, render, Ledger};

    #[test]
    fn a_graded_landing_counts_one_hop() {
        let l = record(Ledger::default(), Some(88), Some(80), Some(true));
        assert_eq!(l.hops, 1);
        assert_eq!(l.best_landing, Some(88));
        assert_eq!(l.best_takeoff, Some(80));
        assert_eq!(l.green_streak, 1);
    }

    /// No landing graded: the flight reopens the menu without a hop — the
    /// ledger only closes the gate-to-gate loop on a graded touchdown.
    #[test]
    fn a_flight_without_a_landing_changes_nothing() {
        let l = record(Ledger::default(), None, Some(80), Some(true));
        assert_eq!(l.hops, 0);
        assert_eq!(l.best_takeoff, None);
    }

    /// Green = landing in the GOOD band, plus the takeoff and the stability
    /// gate whenever they actually graded. A missing gate neither breaks the
    /// streak nor fakes it.
    #[test]
    fn the_green_streak_needs_both_cards_and_survives_a_silent_gate() {
        let clean = record(Ledger::default(), Some(90), Some(85), Some(true));
        assert_eq!(clean.green_streak, 1);
        // Gate never armed (the pilot never descended from above): still green.
        let silent_gate = record(Ledger::default(), Some(90), Some(85), None);
        assert_eq!(silent_gate.green_streak, 1);
        // Unstable gate: not green.
        let unstable = record(Ledger::default(), Some(90), Some(85), Some(false));
        assert_eq!(unstable.green_streak, 0);
        // Soft landing with a perfect takeoff: still not green.
        let soft = record(Ledger::default(), Some(60), Some(85), Some(true));
        assert_eq!(soft.green_streak, 0);
    }

    #[test]
    fn a_broken_breaks_the_streak_but_keeps_the_best() {
        let l = record(Ledger::default(), Some(90), Some(85), Some(true));
        let l = record(l, Some(90), Some(85), Some(true));
        assert_eq!(l.green_streak, 2);
        let l = record(l, Some(40), None, None);
        assert_eq!(l.hops, 3);
        assert_eq!(l.green_streak, 0);
        assert_eq!(l.best_streak, 2);
        assert_eq!(l.best_landing, Some(90));
    }

    #[test]
    fn the_text_format_round_trips() {
        let l = Ledger {
            hops: 7,
            best_takeoff: Some(88),
            best_landing: Some(97),
            green_streak: 2,
            best_streak: 4,
        };
        let parsed = parse(&render(&l));
        assert_eq!(parsed, l);
    }

    #[test]
    fn a_corrupt_line_falls_back_to_defaults() {
        let l = parse("hops notanumber\nskip me\nland 90\n\nbeststreak");
        assert_eq!(l.hops, 0);
        assert_eq!(l.best_landing, Some(90));
        assert_eq!(l.best_streak, 0);
    }
}
