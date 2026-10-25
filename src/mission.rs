//! Lesson missions: spawn states selected from the menu's PLAY page, with
//! the engine plan decided by the lesson, not by a global setting.
//!
//! Every lesson is a *graded* scenario — the rating systems that already watch
//! the flight (the taximeter's parking grade, the takeoff report, the landing
//! card) are the mission's scoreboard; the lesson itself only chooses where
//! to sit the aircraft and whether the engine is cold or running:
//!
//! - **M1 TAXI** (C172) — apron stand, cold and dark: the full taxi drill
//!   (master, starter, taxi out, hold short). Graded by the COLD START
//!   checklist, the taximeter's stop grade and the hold-short finals card.
//! - **M2 TAKEOFF** — runway threshold, warm and running: a clean roll and
//!   rotation, graded by the takeoff report.
//! - **M3 LANDING** — 2 NM final for runway 36, flaps 20: the landing card's
//!   classroom.
//! - **M1 TAXI** (737) — warm at the ZSNM stand: the long taxi (lead-in,
//!   1.6 km of parallel, the threshold stub) graded as one jet drill by the
//!   jet taxi meter — centreline, speed band, thrust discipline, the stop.
//! - **M2 TAKEOFF** (737) — lined up on ZSNM's numbers: the PM callout flow
//!   (80 KT, V1, ROTATE, POSITIVE RATE), the acceleration-altitude clean-up,
//!   and the handoff — A/P on and clean by 5,000 ft.
//! - **M4 GO-AROUND** (737) — a mid-range ILS final, flown into the scripted
//!   GO AROUND call at the 1,000 ft gate: thrust, pitch, gear, flaps 15, the
//!   climb and the missed-approach level-off, all graded.
//! - **M5 RTO** (737) — lined up on ZSNM's numbers: takeoff thrust into the
//!   scripted STOP call at 100 kt, then idle, brakes and reverse to the
//!   graded full stop — the other half of the M2 departure.
//! - **M6 AUTOMATION** (737) — the full coupled arrival: every MCP mode from
//!   CMD to APP, window first and button second, the ATC stop-climb on ALT
//!   HOLD, the ILS established. The A/P flies it all — a disconnect fails
//!   the lesson.
//!
//! The old M4 FINAL (a full cold-start-to-touchdown run) stayed on the
//! drawing board: the landing lesson alone is the grind, and the full flight
//! can come back as free flight practice the player stitches together.
//!
//! The keyboard's R/P/Y relocate shortcuts are gone; the menu (the PLAY page
//! under the aircraft) is the only way to choose a spawn.

use bevy::prelude::*;

/// The lesson HUD tags, in menu order (`Lesson::label`).
const LESSON_LABELS: [&str; 9] = [
    "M1 TAXI",
    "M2 TAKEOFF",
    "M3 LANDING",
    "M1 TAXI",
    "M2 TAKEOFF",
    "M3 LANDING",
    "M4 GO-AROUND",
    "M5 RTO",
    "M6 AUTOMATION",
];

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Lesson {
    /// Cold and dark at the apron stand: the full taxi drill.
    Taxi,
    /// Warm on the runway threshold: rotate on the numbers.
    Takeoff,
    /// 2 NM final for runway 36, flaps 20, 70 kt.
    Landing,
    /// The 737's own taxi drill: warm at the ZSNM stand, taxi out and hold
    /// short of runway 36.
    JetTaxi,
    /// The 737's departure drill: lined up on ZSNM's numbers, the callout
    /// flow to a clean A/P handoff.
    JetTakeoff,
    /// The 737's arrival drill: a mid-range ILS final, the stabilized gate,
    /// the touchdown and the full stop.
    JetLanding,
    /// The 737's missed-approach drill: the scripted GO AROUND call at the
    /// 1,000 ft gate, flown through thrust, gear, flaps and the level-off.
    JetGoAround,
    /// The 737's rejected-takeoff drill: takeoff thrust into the scripted
    /// STOP call, then idle, brakes and reverse to the graded stop.
    JetRto,
    /// The 737's coupled-arrival drill: every MCP mode, window first and
    /// button second, flown on the A/P from CMD to the ILS established.
    JetAutomation,
}

impl Lesson {
    /// The lesson's label, which doubles as the HUD's home-field tag while
    /// the lesson is in the air.
    pub fn label(self) -> &'static str {
        LESSON_LABELS[self as usize]
    }

    /// True when the lesson starts cold and dark and the COLD START checklist
    /// is part of the drill.
    pub fn cold_start(self) -> bool {
        matches!(self, Lesson::Taxi)
    }
}

/// One PLAY-page row: free flight or a graded lesson, with a hint line.
pub struct PlayRow {
    /// The row's label ("FREE FLIGHT", "M1 TAXI", ...).
    pub label: &'static str,
    /// The lesson this row starts, or `None` for free flight.
    pub lesson: Option<Lesson>,
    /// A one-line description of the spawn and its grading.
    pub hint: &'static str,
}

/// The PLAY page's rows, in menu order, rendered by `menu.rs`'s page panel.
/// The graded lesson rows are a C172 feature; `menu.rs` greys them out for the
/// other aircraft (the airframes' own rule, not the page's).
pub const PLAY_MENU: [PlayRow; 4] = [
    PlayRow {
        label: "FREE FLIGHT",
        lesson: None,
        hint: "the classic start: warm on the runway",
    },
    PlayRow {
        label: "M1 TAXI",
        lesson: Some(Lesson::Taxi),
        hint: "cold and dark at the stand: start, taxi out, hold short",
    },
    PlayRow {
        label: "M2 TAKEOFF",
        lesson: Some(Lesson::Takeoff),
        hint: "on the runway, warm: rotate on the numbers",
    },
    PlayRow {
        label: "M3 LANDING",
        lesson: Some(Lesson::Landing),
        hint: "on a 2 NM final, flaps 20, 70 kt",
    },
];

/// The 737's PLAY page rows: free flight plus its own graded lessons at
/// ZSNM.
pub const PLAY_MENU_JET: [PlayRow; 7] = [
    PlayRow {
        label: "FREE FLIGHT",
        lesson: None,
        hint: "warm on ZSNM's runway 36",
    },
    PlayRow {
        label: "M1 TAXI",
        lesson: Some(Lesson::JetTaxi),
        hint: "warm at the stand: taxi out, hold short of RWY 36",
    },
    PlayRow {
        label: "M2 TAKEOFF",
        lesson: Some(Lesson::JetTakeoff),
        hint: "lined up on ZSNM 36: fly the callouts, clean up, hand off to the A/P",
    },
    PlayRow {
        label: "M3 LANDING",
        lesson: Some(Lesson::JetLanding),
        hint: "4.9 NM final: config, the 1000 ft gate, grease it, full stop",
    },
    PlayRow {
        label: "M4 GO-AROUND",
        lesson: Some(Lesson::JetGoAround),
        hint: "fly the final into the GO AROUND call: thrust, gear, flaps, level off",
    },
    PlayRow {
        label: "M5 RTO",
        lesson: Some(Lesson::JetRto),
        hint: "lined up: takeoff thrust, the STOP call, brakes and reverse to the stop",
    },
    PlayRow {
        label: "M6 AUTOMATION",
        lesson: Some(Lesson::JetAutomation),
        hint: "the coupled arrival: window first, button second, the ILS established",
    },
];

/// The PLAY rows an aircraft offers: the C172's lesson set, the 737's own, or
/// free flight only for the rest.
pub fn rows_for(id: &str) -> &'static [PlayRow] {
    match crate::aircraft_profile::normalized(id) {
        "c172p" => &PLAY_MENU,
        "737" => &PLAY_MENU_JET,
        _ => &PLAY_MENU[..1],
    }
}

/// Menu → `start_flight`: the lesson the player just picked (`None` when the
/// flight is a plain PLAY or a map mission).
#[derive(Resource, Default)]
pub struct PendingLesson(pub Option<Lesson>);

/// The lesson of the flight in progress, kept for the HUD tag and the score
/// systems that may want mission context. Cleared when a flight ends.
#[derive(Resource, Default)]
pub struct ActiveLesson(pub Option<Lesson>);

pub struct MissionPlugin;

impl Plugin for MissionPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<PendingLesson>()
            .init_resource::<ActiveLesson>()
            .add_systems(OnExit(crate::app_state::AppState::Flying), clear_lesson);
    }
}

/// Leaving the flight forgets which lesson it was; the menu starts clean.
fn clear_lesson(mut active: ResMut<ActiveLesson>) {
    active.0 = None;
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn play_menu_covers_free_flight_and_every_lesson_once() {
        assert_eq!(PLAY_MENU[0].lesson, None, "row 0 is free flight");
        for lesson in [Lesson::Taxi, Lesson::Takeoff, Lesson::Landing] {
            let count = PLAY_MENU
                .iter()
                .filter(|row| row.lesson == Some(lesson))
                .count();
            assert_eq!(count, 1, "{lesson:?} listed once");
        }
    }

    #[test]
    fn cold_start_lessons_start_dark() {
        assert!(Lesson::Taxi.cold_start());
        assert!(!Lesson::Takeoff.cold_start());
        assert!(!Lesson::Landing.cold_start());
        assert!(!Lesson::JetTaxi.cold_start(), "the jet taxi drill is warm");
        assert!(!Lesson::JetTakeoff.cold_start(), "the jet departs warm");
        assert!(!Lesson::JetLanding.cold_start(), "the jet arrives warm");
        assert!(
            !Lesson::JetGoAround.cold_start(),
            "the jet's go-around arrives warm"
        );
        assert!(!Lesson::JetRto.cold_start(), "the jet's RTO departs warm");
        assert!(
            !Lesson::JetAutomation.cold_start(),
            "the coupled arrival starts airborne"
        );
    }

    #[test]
    fn jet_play_menu_covers_free_flight_and_the_lessons() {
        assert_eq!(PLAY_MENU_JET[0].lesson, None, "row 0 is free flight");
        assert_eq!(PLAY_MENU_JET[1].lesson, Some(Lesson::JetTaxi));
        assert_eq!(PLAY_MENU_JET[1].label, "M1 TAXI");
        assert_eq!(PLAY_MENU_JET[2].lesson, Some(Lesson::JetTakeoff));
        assert_eq!(PLAY_MENU_JET[2].label, "M2 TAKEOFF");
        assert_eq!(PLAY_MENU_JET[3].lesson, Some(Lesson::JetLanding));
        assert_eq!(PLAY_MENU_JET[3].label, "M3 LANDING");
        assert_eq!(PLAY_MENU_JET[4].lesson, Some(Lesson::JetGoAround));
        assert_eq!(PLAY_MENU_JET[4].label, "M4 GO-AROUND");
        assert_eq!(PLAY_MENU_JET[5].lesson, Some(Lesson::JetRto));
        assert_eq!(PLAY_MENU_JET[5].label, "M5 RTO");
        assert_eq!(PLAY_MENU_JET[6].lesson, Some(Lesson::JetAutomation));
        assert_eq!(PLAY_MENU_JET[6].label, "M6 AUTOMATION");
    }

    #[test]
    fn rows_for_picks_the_aircrafts_own_menu() {
        assert_eq!(rows_for("c172p").len(), 4);
        assert_eq!(rows_for("737").len(), 7);
        assert_eq!(rows_for("737")[1].lesson, Some(Lesson::JetTaxi));
        assert_eq!(rows_for("737")[2].lesson, Some(Lesson::JetTakeoff));
        assert_eq!(rows_for("737")[3].lesson, Some(Lesson::JetLanding));
        assert_eq!(rows_for("737")[4].lesson, Some(Lesson::JetGoAround));
        assert_eq!(rows_for("737")[5].lesson, Some(Lesson::JetRto));
        assert_eq!(rows_for("737")[6].lesson, Some(Lesson::JetAutomation));
        // Anything else: free flight only.
        assert_eq!(rows_for("f22").len(), 1);
        assert_eq!(rows_for("f22")[0].lesson, None);
    }
}
