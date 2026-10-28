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

use crate::aircraft_profile::Action;
use crate::sim::Sim;

/// Everything that varies between lessons, in one table. `menu.rs` used to
/// carry a chain of `matches!(lesson, ...)` blocks for the seat, the pinned
/// flap/throttle and the MCP windows, and `lesson_flow.rs` another for the
/// grade card; both now read the row for the active lesson, so adding a
/// lesson is one entry here plus its meter.
pub struct LessonSpec {
    /// The HUD/home-field tag, and the PLAY row's label.
    pub label: &'static str,
    /// Seat the FDM at the lesson's start (stand, threshold, final, ...).
    pub seat: fn(&mut Sim),
    /// Cold and dark at spawn: the COLD START checklist is part of the drill.
    pub cold_start: bool,
    /// Controls pinned from the first frame: `(action, value)`.
    pub pins: &'static [(Action, f64)],
    /// The MCP's speed window at spawn; `None` keeps the 250 kt default.
    pub mcp_ias_kts: Option<f64>,
    /// The MCP's vertical-speed window at spawn; `None` keeps the default.
    pub mcp_vs_fpm: Option<f64>,
    /// The nose wheel's lead over the belly for the hold-line incursion.
    pub nose_lead_m: f32,
    /// The grade card's title.
    pub title: &'static str,
    /// Whether the grade card offers Enter to fly on.
    pub continue_after: bool,
    /// The off-pavement FAIL text (the landing lesson reads "OFF RUNWAY").
    pub off_pavement: &'static str,
}

/// The off-pavement FAIL card for the taxi drills.
const FAIL_OFF_PAVEMENT: &str = "FAIL - OFF TAXIWAY\nThe wheels left the pavement.";
/// The landing lesson's pavement fail reads like a landing, not a taxi.
const FAIL_OFF_RUNWAY: &str = "FAIL - OFF RUNWAY\nThe wheels settled off the pavement.";

/// Row per [`Lesson`] discriminant, in the same order as the enum.
static LESSON_SPECS: [LessonSpec; 9] = [
    // M1 TAXI (C172): cold and dark at the stand; the card ends the lesson.
    LessonSpec {
        label: "M1 TAXI",
        seat: Sim::reset_stand,
        cold_start: true,
        pins: &[],
        mcp_ias_kts: None,
        mcp_vs_fpm: None,
        nose_lead_m: 1.5,
        title: "HOLD SHORT - GRADED",
        continue_after: false,
        off_pavement: FAIL_OFF_PAVEMENT,
    },
    // M2 TAKEOFF (C172): warm on the threshold.
    LessonSpec {
        label: "M2 TAKEOFF",
        seat: Sim::reset_ground,
        cold_start: false,
        pins: &[],
        mcp_ias_kts: None,
        mcp_vs_fpm: None,
        nose_lead_m: 1.5,
        title: "TAKEOFF - GRADED",
        continue_after: true,
        off_pavement: FAIL_OFF_PAVEMENT,
    },
    // M3 LANDING (C172): 2 NM final, flaps 20 pinned from the first frame.
    LessonSpec {
        label: "M3 LANDING",
        seat: Sim::reset_final_c172,
        cold_start: false,
        pins: &[(Action::Flap, 0.67)],
        mcp_ias_kts: None,
        mcp_vs_fpm: None,
        nose_lead_m: 1.5,
        title: "LANDING - GRADED",
        continue_after: true,
        off_pavement: FAIL_OFF_RUNWAY,
    },
    // M1 TAXI (737): warm at ZSNM, idle and parked at the stand. The
    // profile's 60% taxi throttle would have her rolling before the first
    // frame; the RTO below pins the same because its SETUP row grades the
    // thrust the pilot sets himself.
    LessonSpec {
        label: "M1 TAXI",
        seat: Sim::reset_stand,
        cold_start: false,
        pins: &[(Action::Throttle, 0.0)],
        mcp_ias_kts: None,
        mcp_vs_fpm: None,
        nose_lead_m: crate::jet_taxi::NOSE_LEAD_M,
        title: "HOLD SHORT - GRADED",
        continue_after: false,
        off_pavement: FAIL_OFF_PAVEMENT,
    },
    // M2 TAKEOFF (737): lined up on the numbers.
    LessonSpec {
        label: "M2 TAKEOFF",
        seat: Sim::reset_threshold,
        cold_start: false,
        pins: &[],
        mcp_ias_kts: None,
        mcp_vs_fpm: None,
        nose_lead_m: 1.5,
        title: "TAKEOFF - GRADED",
        continue_after: true,
        off_pavement: FAIL_OFF_PAVEMENT,
    },
    // M3 LANDING (737): flaps 5 and the beam's own power pinned (~0.25 — the
    // descent's gravity share carries most of the drag). Level power (~0.5)
    // had the APP-coupled seat accelerating down the beam; dirty-ing up then
    // decelerates her into the ladder's targets on its own.
    LessonSpec {
        label: "M3 LANDING",
        seat: Sim::reset_final_jet,
        cold_start: false,
        pins: &[(Action::Flap, 5.0 / 40.0), (Action::Throttle, 0.25)],
        mcp_ias_kts: Some(160.0),
        mcp_vs_fpm: Some(-950.0),
        nose_lead_m: 1.5,
        title: "LANDING - GRADED",
        continue_after: true,
        off_pavement: FAIL_OFF_PAVEMENT,
    },
    // M4 GO-AROUND: landing config and the draggy beam's power (~0.43: the
    // m4_probe sweep crosses the 1,000 ft gate at −600 fpm and 144 kt, ~30 s
    // in, so the seat rides the descent until the call).
    LessonSpec {
        label: "M4 GO-AROUND",
        seat: Sim::reset_go_around_jet,
        cold_start: false,
        pins: &[(Action::Flap, 30.0 / 40.0), (Action::Throttle, 0.43)],
        mcp_ias_kts: Some(140.0),
        mcp_vs_fpm: Some(-700.0),
        nose_lead_m: 1.5,
        title: "GO-AROUND - GRADED",
        continue_after: true,
        off_pavement: FAIL_OFF_PAVEMENT,
    },
    // M5 RTO: lined up, idle and parked; the pilot sets takeoff thrust.
    LessonSpec {
        label: "M5 RTO",
        seat: Sim::reset_threshold,
        cold_start: false,
        pins: &[(Action::Throttle, 0.0)],
        mcp_ias_kts: None,
        mcp_vs_fpm: None,
        nose_lead_m: 1.5,
        title: "RTO - GRADED",
        continue_after: true,
        off_pavement: FAIL_OFF_PAVEMENT,
    },
    // M6 AUTOMATION: the seat's own cruise; the rest of the windows are the
    // lesson, so only the speed rides at 210 from the start.
    LessonSpec {
        label: "M6 AUTOMATION",
        seat: Sim::reset_auto_lesson,
        cold_start: false,
        pins: &[],
        mcp_ias_kts: Some(210.0),
        mcp_vs_fpm: None,
        nose_lead_m: 1.5,
        title: "AUTOMATION - GRADED",
        continue_after: false,
        off_pavement: FAIL_OFF_PAVEMENT,
    },
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
    /// Every lesson, in menu/discriminant order — the table's index is the
    /// enum's discriminant, so this also checks the two stay aligned.
    pub const ALL: [Lesson; 9] = [
        Lesson::Taxi,
        Lesson::Takeoff,
        Lesson::Landing,
        Lesson::JetTaxi,
        Lesson::JetTakeoff,
        Lesson::JetLanding,
        Lesson::JetGoAround,
        Lesson::JetRto,
        Lesson::JetAutomation,
    ];

    /// This lesson's row in [`LESSON_SPECS`].
    pub fn spec(self) -> &'static LessonSpec {
        &LESSON_SPECS[self as usize]
    }

    /// The lesson's label, which doubles as the HUD's home-field tag while
    /// the lesson is in the air.
    pub fn label(self) -> &'static str {
        self.spec().label
    }

    /// True when the lesson starts cold and dark and the COLD START checklist
    /// is part of the drill.
    pub fn cold_start(self) -> bool {
        self.spec().cold_start
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
    fn every_lesson_has_a_spec_at_its_own_index() {
        for (i, lesson) in Lesson::ALL.iter().enumerate() {
            assert_eq!(*lesson as usize, i, "{lesson:?} sits at its index");
            assert_eq!(lesson.spec().label, lesson.label());
        }
    }

    #[test]
    fn play_rows_agree_with_the_spec_labels() {
        for row in PLAY_MENU.iter().chain(PLAY_MENU_JET.iter()) {
            if let Some(lesson) = row.lesson {
                assert_eq!(row.label, lesson.label(), "{lesson:?}'s row label");
            }
        }
    }

    #[test]
    fn specs_carry_each_drills_configuration() {
        use crate::aircraft_profile::Action;

        let landing = Lesson::JetLanding.spec();
        assert_eq!(landing.mcp_ias_kts, Some(160.0));
        assert_eq!(landing.mcp_vs_fpm, Some(-950.0));
        assert!(landing.pins.contains(&(Action::Flap, 5.0 / 40.0)));
        assert!(landing.pins.contains(&(Action::Throttle, 0.25)));

        let auto = Lesson::JetAutomation.spec();
        assert_eq!(auto.mcp_ias_kts, Some(210.0));
        assert_eq!(auto.mcp_vs_fpm, None, "the V/S window is the lesson's work");
        // The ground drills end at the stop; the airborne cards fly on.
        assert!(!auto.continue_after);
        assert!(!Lesson::Taxi.spec().continue_after);
        assert!(Lesson::Takeoff.spec().continue_after);
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
