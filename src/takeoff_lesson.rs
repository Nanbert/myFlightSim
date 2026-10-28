//! The graded TAKEOFF lesson (M2): a roll-and-climb course instead of a
//! single liftoff snapshot.
//!
//! The lesson is C172 and runs the runway straight ahead: a warm spawn on
//! the threshold, a full-power roll, rotation on the book's 57 kt, then a
//! sustained climb that **freezes the card at the 200 ft gate** — climbing
//! straight, on the line-up heading, at the climb speed. The finish gate is
//! what makes M2 a lesson rather than a sandbox: every phase between the
//! spawn and the gate is watched, sampled and graded:
//!
//! - **Roll** — the runway-centreline average while rolling (the M1 meter's
//!   sampler, pointed at the runway), with the live `RLW Δ` row.
//! - **Rotation callouts** — the banner plays the single-pilot flow: the
//!   `40 KT - AIRSPEED LIVE` crosscheck crossing, then a standing `ROTATE`
//!   from VR.
//! - **Liftoff frame** — CAS against VR, pitch attitude and bank, the same
//!   terms the free-flight `takeoff.rs` card grades.
//! - **Climb** — heading error against the line-up heading and climb speed
//!   against Vy, counted straight through to the gate.
//!
//! Failures (feeding the lesson-flow FAIL popup): settling back onto the
//! runway after liftoff, banking past 30° below the gate, and stalling the
//! climb below the gate. Parking after a graded card re-arms a fresh
//! attempt, like the taxi meter's next approach.
//!
//! The card goes out through `lesson_flow`. Free flight keeps the passive
//! `takeoff.rs` report and its 737 gear timer.

use bevy::prelude::*;

use crate::aerodrome::on_runway;
use crate::app_state::{unpaused, AppState};
use crate::lesson_telem::Telem;
use crate::mission::{ActiveLesson, Lesson};
use crate::scoring::{score, word};
use crate::sim::AircraftState;

/// The C172 book rotation: 55–60 kt, graded on the 57 kt centre.
const VR: f64 = 57.0;
/// The book's best-rate-of-climb speed, the climb grading's centre.
const VY: f64 = 74.0;
/// The gate that completes the lesson: climbing through this AGL freezes
/// the card.
const GATE_AGL: f64 = 200.0;
/// The airspeed callout: past this the gauges are live (the single-pilot
/// version of the jet's 80-knot crosscheck).
const AIRSPEED_LIVE: f64 = 40.0;
/// How long a crossing callout holds the banner (the liftoff's ROTATE
/// rides a touch shorter).
const CALLOUT_SECS: f32 = 2.5;
/// Rolling below this speed is standing still, not a roll worth sampling.
const ROLLING_GS: f64 = 3.0;
/// The liftoff edge needs the roll established; below it a wow flip is the
/// spawn settling on its gear (the same gate the passive scorer uses).
const LIFT_MIN_GS: f64 = 30.0;
/// Banking under the gate is a flat FAIL above this.
const STEEP_BANK: f64 = 30.0;
/// The climb dying below this CAS under the gate is a flat FAIL.
const STALL_CAS: f64 = 44.0;

/// The pitch-attitude centre a rotation is graded against (the Cessna hauls
/// itself off a touch flatter than the jet's 10°).
const PITCH_CENTRE: f64 = 8.0;

#[derive(Clone, Copy, PartialEq, Default)]
enum Phase {
    /// On the runway: waiting at the threshold or rolling.
    #[default]
    Roll,
    /// Wheels up, climbing toward the gate.
    Climb,
    /// The gate (or a fail) froze the card.
    Graded,
}

/// The liftoff frame's graded readouts plus the climb's heading reference.
#[derive(Clone, Copy)]
struct LiftFrame {
    cas: f64,
    pitch: f64,
    bank: f64,
    /// Heading at wheels-off, degrees — the climb-out's centreline.
    hdg_ref: f64,
}

/// The frozen finals: every row keeps its deviation, its 0–10 score and the
/// word band; the final is their weighted average (bad rows pull it down,
/// nothing zeroes it).
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct Finals {
    /// Runway-centreline average while rolling, metres.
    roll_avg: f32,
    roll: f32,
    /// Liftoff speed (kt) vs VR.
    lift_cas: f64,
    lift_speed: f32,
    /// Liftoff pitch attitude, degrees.
    pitch: f64,
    pitch_score: f32,
    /// Bank at the liftoff frame, degrees.
    bank_deg: f64,
    bank: f32,
    /// Average signed heading error through the climb, degrees.
    hdg_err: f32,
    hdg: f32,
    /// Average climb CAS (kt).
    climb_cas: f64,
    climb_speed: f32,
    /// The weighted final.
    final_: f32,
}

// --- row weights ------------------------------------------------------------

/// The weighted final's row weights (they sum to 1). The attitude at
/// rotation and the two climb keepers carry the lesson; the bank row is a
/// tie-breaker, not a pillar.
const W_ROLL: f32 = 0.20;
const W_SPEED: f32 = 0.20;
const W_PITCH: f32 = 0.20;
const W_BANK: f32 = 0.05;
const W_HDG: f32 = 0.15;
const W_CLIMB: f32 = 0.20;

const FAIL_SANK: &str = "FAIL - SANK BACK\nThe climb out settled back onto the runway.";
const FAIL_STEEP: &str = "FAIL - STEEP LOW TURN\nBanked past 30 degrees below the climb gate.";
const FAIL_STALL: &str = "FAIL - STALLED IN THE CLIMB\nThe climb ate its speed below the gate.";

#[derive(Resource, Default)]
pub struct TakeoffLesson {
    phase: Phase,
    /// The live prompt text, composed every ticking frame; `None` hides the
    /// centred card.
    pub(crate) live: Option<String>,
    /// The |offset| average over the roll: sum and samples.
    roll_sum: f32,
    roll_n: u32,
    /// The liftoff frame readouts; `None` while the wheels still carry.
    lift: Option<LiftFrame>,
    /// Climb sampling below the gate: signed heading errors (+ right of the
    /// line-up heading) and CAS counts.
    hdg_err_sum: f32,
    hdg_err_n: u32,
    cas_sum: f64,
    cas_n: u32,
    /// A transient banner (a callout crossing) and when it retires.
    callout: Option<(String, f32)>,
    /// Ground speed the previous tick, to catch crossings.
    last_gs: f64,
    /// The fail that parked the attempt, for the lesson-flow popup.
    pub(crate) fail: Option<&'static str>,
    /// The frozen finals of the attempt (usually at the gate).
    pub(crate) done: Option<Finals>,
    /// A graded attempt holds its card until the next rollout re-arms.
    pub(crate) settled: bool,
    was_ground: bool,
}

impl TakeoffLesson {
    /// A fresh attempt: the same hand-out the Flying entry uses, minus the
    /// ``previous frame grounded'' latch. An airborne arm (the CONTINUE past
    /// the gate) stays quiet — the liftoff edge needs a ground frame first —
    /// while a spawn on the threshold re-grounds within one tick.
    pub(crate) fn rearm(&mut self) {
        *self = TakeoffLesson::default();
    }
}

impl TakeoffLesson {
    /// The finals card's rows: every graded row prints its deviation, its
    /// 0–10 score and the word band, with the weighted final on top (the
    /// popup's title carries the lesson name).
    pub fn finals_text(&self) -> Option<String> {
        let f = self.done?;
        Some(format!(
            "ROLL  Δ avg {ra:+.1} m     {r:.1} {rw}\n\
             LIFT  {cas:+.1} / VR {VR:.0} kt  {s:.1} {sw}\n\
             PITCH {p:+.1}° bank {b:.1}°  {pi:.1} {pw}\n\
             CLIMB hdg Δ {h:+.1}°     {hh:.1} {hw}\n\
             CLIMB {cc:+.1} / VY {VY:.0} kt  {cs:.1} {cw}\n\
             FINAL {f:.1}  {fw}",
            ra = f.roll_avg,
            r = f.roll,
            rw = word(f.roll),
            cas = f.lift_cas,
            s = f.lift_speed,
            sw = word(f.lift_speed),
            p = f.pitch,
            b = f.bank_deg,
            pi = f.pitch_score,
            pw = word(f.pitch_score),
            h = f.hdg_err,
            hh = f.hdg,
            hw = word(f.hdg),
            cc = f.climb_cas,
            cs = f.climb_speed,
            cw = word(f.climb_speed),
            f = f.final_,
            fw = word(f.final_),
        ))
    }
}

/// Feed one frame's readouts. Pure over its own state, for tests.
fn tick(tl: &mut TakeoffLesson, t: Telem, now: f32, _dt: f32) {
    // A retired banner drops before the phase logic reads it.
    if tl.callout.as_ref().is_some_and(|(_, until)| now >= *until) {
        tl.callout = None;
    }
    let agl = t.alt_ft;
    match tl.phase {
        Phase::Roll => {
            // The 40-kt crosscheck: the gauges come alive mid-roll.
            if t.on_ground && tl.last_gs < AIRSPEED_LIVE && t.gs_kts >= AIRSPEED_LIVE {
                tl.callout = Some(("40 KT - AIRSPEED LIVE".to_owned(), now + CALLOUT_SECS));
            }
            // The centreline sampler: once the aircraft actually rolls.
            if t.on_ground && t.on_runway && t.gs_kts > ROLLING_GS {
                tl.roll_sum += t.pos.x.abs();
                tl.roll_n += 1;
            }
            // The liftoff edge: wheels up with the roll established.
            if !t.on_ground && tl.was_ground && t.gs_kts > LIFT_MIN_GS {
                tl.lift = Some(LiftFrame {
                    cas: t.cas_kts,
                    pitch: t.pitch_deg,
                    bank: t.roll_deg.abs(),
                    hdg_ref: t.heading_deg,
                });
                tl.phase = Phase::Climb;
                // The standing ROTATE rides up with the aircraft for a beat.
                tl.callout = Some(("ROTATE".to_owned(), now + 1.2));
            }
        }
        Phase::Climb => {
            let grading = tl.fail.is_none();
            // Fails first: nothing past the gate can save the attempt.
            if grading && t.on_ground {
                tl.fail = Some(FAIL_SANK);
            } else if grading && agl < GATE_AGL && t.roll_deg.abs() > STEEP_BANK {
                tl.fail = Some(FAIL_STEEP);
            } else if grading && agl < GATE_AGL && t.cas_kts < STALL_CAS {
                tl.fail = Some(FAIL_STALL);
            }
            if tl.fail.is_some() {
                tl.phase = Phase::Graded;
            } else if agl >= GATE_AGL {
                // The gate: freeze the finals; the popup (or card) takes it
                // from here. This frame's sample isn't kept — the gate is
                // passed, not part of the climb.
                tl.done = Some(takeoff_finals(tl));
                tl.settled = true;
                tl.phase = Phase::Graded;
            } else if grading {
                let dh =
                    hdg_delta(t.heading_deg, tl.lift.expect("climb implies a liftoff").hdg_ref);
                tl.hdg_err_sum += dh;
                tl.hdg_err_n += 1;
                tl.cas_sum += t.cas_kts;
                tl.cas_n += 1;
            }
        }
        Phase::Graded => {
            // Touching down and parking re-arms a fresh attempt, like the
            // taxi meter's next approach after a graded stop.
            if t.on_ground && t.gs_kts < ROLLING_GS {
                *tl = TakeoffLesson {
                    was_ground: true,
                    ..default()
                };
            }
            return;
        }
    }
    tl.last_gs = t.gs_kts;
    tl.was_ground = t.on_ground;
    tl.live = compose_live(tl, &t, now);
}

/// The banner-first live prompt: a callout while it runs, otherwise the
/// phase's numbers. `None` while a card owns the display.
fn compose_live(tl: &TakeoffLesson, t: &Telem, now: f32) -> Option<String> {
    if tl.settled || tl.done.is_some() || tl.fail.is_some() {
        return None;
    }
    if let Some((text, until)) = &tl.callout {
        if now < *until {
            return Some(text.clone());
        }
    }
    match tl.phase {
        Phase::Roll => {
            // Standing ROTATE once the bug is at hand — the callout that
            // should never be missed.
            if t.on_ground && t.gs_kts >= VR {
                Some("ROTATE".to_owned())
            } else if t.on_ground && t.gs_kts > 1.0 {
                Some(format!(
                    "T.O. ROLL\nRLW Δ {:+.1} m    GS {:.0}",
                    t.pos.x, t.gs_kts
                ))
            } else if t.on_ground {
                // The standing start: the runway is live even before the
                // wheels move.
                Some("RWY 36\nFULL POWER FOR THE ROLL".to_owned())
            } else {
                None
            }
        }
        Phase::Climb => {
            let dh = hdg_delta(t.heading_deg, tl.lift.expect("climb implies a liftoff").hdg_ref);
            Some(format!(
                "CLIMB OUT\nHDG Δ {dh:+.1}°    {:.0} kt / VY {VY:.0}    {:.0} / {GATE} ft",
                t.cas_kts,
                t.alt_ft.max(0.0),
                GATE = GATE_AGL,
            ))
        }
        Phase::Graded => None,
    }
}

/// The climb averages and liftoff frame frozen into one attempt's finals:
/// every row scored 0–10 (see `scoring`), the final their weighted average.
fn takeoff_finals(tl: &TakeoffLesson) -> Finals {
    let lift = tl.lift.expect("the gate implies a liftoff");
    let roll_avg = if tl.roll_n > 0 {
        tl.roll_sum / tl.roll_n as f32
    } else {
        0.0
    };
    let hdg_err = if tl.hdg_err_n > 0 {
        tl.hdg_err_sum / tl.hdg_err_n as f32
    } else {
        0.0
    };
    let climb_cas = if tl.cas_n > 0 {
        tl.cas_sum / tl.cas_n as f64
    } else {
        lift.cas
    };
    let roll = score(roll_avg, 1.5, 8.0);
    let lift_speed = score((lift.cas - VR).abs() as f32, 1.5, 8.0);
    let pitch_score = score((lift.pitch - PITCH_CENTRE).abs() as f32, 1.5, 8.0);
    let bank = score(lift.bank as f32, 1.0, 10.0);
    let hdg = score(hdg_err.abs(), 2.0, 10.0);
    let climb_speed = score((climb_cas - VY).abs() as f32, 3.0, 12.0);
    let final_ = W_ROLL * roll
        + W_SPEED * lift_speed
        + W_PITCH * pitch_score
        + W_BANK * bank
        + W_HDG * hdg
        + W_CLIMB * climb_speed;
    Finals {
        roll_avg,
        roll,
        lift_cas: lift.cas,
        lift_speed,
        pitch: lift.pitch,
        pitch_score,
        bank_deg: lift.bank,
        bank,
        hdg_err,
        hdg,
        climb_cas,
        climb_speed,
        final_,
    }
}

/// Signed heading difference wrapped to ±180°, degrees.
fn hdg_delta(hdg: f64, reference: f64) -> f32 {
    ((hdg - reference + 180.0).rem_euclid(360.0) - 180.0) as f32
}

/// The update system: only the takeoff lesson, only the home field.
fn step_m2(
    aircraft: Res<AircraftState>,
    map: Res<crate::map::ActiveMap>,
    lesson: Res<ActiveLesson>,
    time: Res<Time>,
    mut tl: ResMut<TakeoffLesson>,
) {
    if map.0.is_some() || lesson.0 != Some(Lesson::Takeoff) {
        return;
    }
    tick(
        &mut tl,
        Telem {
            on_runway: on_runway(aircraft.pos.x, aircraft.pos.z),
            ..Telem::sample(&aircraft)
        },
        time.elapsed_secs(),
        time.delta_secs(),
    );
}

pub struct TakeoffLessonPlugin;

impl Plugin for TakeoffLessonPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<TakeoffLesson>()
            .add_systems(OnEnter(AppState::Flying), reset_takeoff_lesson)
            .add_systems(
                Update,
                // Fresh readouts in; the flow's watcher reads what we froze.
                step_m2
                    .after(crate::sim::publish_state)
                    .before(crate::lesson_flow::watch)
                    .run_if(unpaused),
            );
    }
}

/// A fresh flight hands over a fresh attempt.
fn reset_takeoff_lesson(mut tl: ResMut<TakeoffLesson>) {
    tl.rearm();
}

// --- tests -------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use bevy::prelude::*;

    use super::{tick, TakeoffLesson, Telem, VR};

    const DT: f32 = 1.0 / 60.0;
    const NOW: f32 = 100.0;

    /// A frame's telemetry: `x` in metres off the runway centreline, CAS
    /// carried explicitly (it lags the wheels in real climbs).
    fn telem_full(
        x: f32,
        gs: f64,
        cas: f64,
        alt: f64,
        roll: f64,
        pitch: f64,
        heading: f64,
        on_ground: bool,
    ) -> Telem {
        Telem {
            on_ground,
            gs_kts: gs,
            cas_kts: cas,
            alt_ft: alt,
            pitch_deg: pitch,
            roll_deg: roll,
            heading_deg: heading,
            on_runway: x.abs() <= 15.0,
            pos: Vec3::new(x, (alt / 3.28) as f32, 90.0),
            ..Telem::default()
        }
    }

    fn tick_at(tl: &mut TakeoffLesson, t: Telem, secs: f32) {
        tick(tl, t, NOW + secs, DT);
    }

    /// The straight book roll: from the line, past both callouts, up to the
    /// wheels leaving at ~58 kt.
    fn book_roll(tl: &mut TakeoffLesson) {
        tick_at(tl, telem_full(0.5, 10.0, 10.0, 0.0, 0.0, 0.0, 0.0, true), 0.0);
        tick_at(tl, telem_full(0.5, 26.0, 26.0, 0.0, 0.0, 0.0, 0.0, true), 1.0);
        tick_at(tl, telem_full(0.5, 42.0, 42.0, 0.0, 0.0, 0.0, 0.0, true), 2.0);
        // The liftoff edge: the previous ground frame then the climb.
        tick_at(
            tl,
            telem_full(0.5, 58.0, 57.0, 0.0, 0.0, 8.0, 0.0, false),
            3.0,
        );
    }

    /// A book climb: straight, at Vy, stepping over the gate.
    fn book_climb(tl: &mut TakeoffLesson, start: f32) {
        for i in 0..6 {
            let alt = 40.0 * (i + 1) as f64; // 40..240, crossing 200
            tick_at(
                tl,
                telem_full(1.0, 76.0, 74.0, alt, 0.0, 8.0, 0.0, false),
                start + i as f32 * 0.5,
            );
        }
    }

    #[test]
    fn a_book_takeoff_tops_every_row() {
        let mut tl = TakeoffLesson::default();
        book_roll(&mut tl);
        book_climb(&mut tl, 4.0);
        let f = tl.done.expect("the gate froze the finals");
        assert!(tl.settled, "the card owns the display at the gate");
        assert_eq!(f.roll, 10.0, "the 0.5 m line");
        assert_eq!(f.roll_avg, 0.5);
        assert_eq!(f.lift_speed, 10.0, "off at ~57 kt");
        assert_eq!(f.hdg, 10.0, "straight ahead");
        assert_eq!(f.climb_speed, 10.0, "at the climb speed");
        assert!((f.final_ - 10.0).abs() < 1e-3, "all rows perfect: {f:?}");
        assert_eq!(tl.live, None, "a graded attempt hides the prompt");
        let text = tl.finals_text().expect("the card");
        assert!(text.contains("TIGHT"), "{text}");
    }

    #[test]
    fn an_early_rotation_pulls_the_final_down_without_zeroing_it() {
        let mut tl = TakeoffLesson::default();
        // Rotate 12 kt under the bug (45): past the LIFT row's cap.
        tick_at(&mut tl, telem_full(0.0, 20.0, 20.0, 0.0, 0.0, 0.0, 0.0, true), 0.0);
        tick_at(
            &mut tl,
            telem_full(0.0, 45.0, 45.0, 0.0, 0.0, 12.0, 0.0, false),
            1.0,
        );
        book_climb(&mut tl, 2.0);
        let f = tl.done.expect("graded");
        assert_eq!(f.lift_speed, 0.0, "12 kt under the bug is past the cap");
        // The 12° pull also dings the pitch row; the weighted final keeps
        // the blest rows and lands mid-card instead of collapsing.
        assert!(
            (6.0..=8.0).contains(&f.final_),
            "weighted, not worst-of: {f:?}"
        );
        let text = tl.finals_text().expect("the card");
        assert!(text.contains("BAD"), "{text}");
    }

    #[test]
    fn rolling_off_the_line_fades_the_roll_row() {
        let mut tl = TakeoffLesson::default();
        // Drift out to 5 m off the centreline and rotate there.
        tick_at(&mut tl, telem_full(4.0, 20.0, 20.0, 0.0, 0.0, 0.0, 0.0, true), 0.0);
        tick_at(&mut tl, telem_full(5.0, 44.0, 44.0, 0.0, 0.0, 0.0, 0.0, true), 1.0);
        tick_at(
            &mut tl,
            telem_full(5.0, 57.0, 57.0, 0.0, 0.0, 8.0, 0.0, false),
            2.0,
        );
        book_climb(&mut tl, 3.0);
        let f = tl.done.expect("graded");
        assert!(
            f.roll < 5.5,
            "a 4.5 m average is wide of the line: {}",
            f.roll
        );
        assert!(
            (8.5..=9.5).contains(&f.final_),
            "the roll pulls, the clean rows hold: {f:?}"
        );
    }

    #[test]
    fn settling_back_after_liftoff_is_a_fail() {
        let mut tl = TakeoffLesson::default();
        book_roll(&mut tl);
        // Back on the ground at 60 ft in the climb: a settle-back.
        tick_at(
            &mut tl,
            telem_full(1.0, 70.0, 57.0, 60.0, 0.0, 2.0, 0.0, true),
            4.0,
        );
        assert_eq!(tl.fail, Some(super::FAIL_SANK));
        assert_eq!(tl.done, None, "a failed climb grades nothing");
        assert_eq!(tl.live, None, "the popup owns the display");
    }

    #[test]
    fn a_steep_bank_below_the_gate_is_a_fail() {
        let mut tl = TakeoffLesson::default();
        book_roll(&mut tl);
        tick_at(
            &mut tl,
            telem_full(1.0, 76.0, 74.0, 120.0, 35.0, 8.0, 0.0, false),
            4.0,
        );
        assert_eq!(tl.fail, Some(super::FAIL_STEEP));
    }

    #[test]
    fn stalling_the_climb_below_the_gate_is_a_fail() {
        let mut tl = TakeoffLesson::default();
        book_roll(&mut tl);
        // Haul it nose-high and let the speed die below the gate.
        tick_at(
            &mut tl,
            telem_full(1.0, 60.0, 42.0, 90.0, 0.0, 14.0, 0.0, false),
            4.0,
        );
        assert_eq!(tl.fail, Some(super::FAIL_STALL));
    }

    #[test]
    fn crossing_callouts_play_the_single_pilot_flow() {
        let mut tl = TakeoffLesson::default();
        // 40 crossing: the AIRSPEED LIVE banner rides for its window.
        tick_at(&mut tl, telem_full(0.0, 39.0, 39.0, 0.0, 0.0, 0.0, 0.0, true), 0.0);
        tick_at(&mut tl, telem_full(0.0, 41.0, 41.0, 0.0, 0.0, 0.0, 0.0, true), 0.1);
        let live = tl.live.clone().expect("a callout is up");
        assert!(live.contains("AIRSPEED LIVE"), "{live}");
        // And at VR the standing ROTATE takes the banner back.
        tick_at(&mut tl, telem_full(0.0, 58.0, 57.0, 0.0, 0.0, 0.0, 0.0, true), 3.0);
        assert_eq!(tl.live.as_deref(), Some("ROTATE"));
        // Liftoff: the ROTATE rides up with the aircraft.
        tick_at(
            &mut tl,
            telem_full(0.0, 60.0, 57.0, 5.0, 0.0, 8.0, 0.0, false),
            3.2,
        );
        assert_eq!(tl.live.as_deref(), Some("ROTATE"), "the ride-up beat");
        // Then the climb row takes over.
        tick_at(
            &mut tl,
            telem_full(0.0, 76.0, 74.0, 30.0, 0.0, 8.0, 0.0, false),
            4.5,
        );
        let live = tl.live.expect("the climb row");
        assert!(live.contains("CLIMB OUT") && live.contains("VY"), "{live}");
    }

    #[test]
    fn a_bubble_off_the_wrap_heading_stays_small() {
        // Line-up on 350: the little right-hand wobble to 355 is 5° of
        // correction, not a 355° blowup.
        let mut tl = TakeoffLesson::default();
        tick_at(
            &mut tl,
            telem_full(0.0, 20.0, 20.0, 0.0, 0.0, 0.0, 350.0, true),
            0.0,
        );
        tick_at(
            &mut tl,
            telem_full(0.0, 57.0, 57.0, 0.0, 0.0, 8.0, 350.0, false),
            1.0,
        );
        tick_at(
            &mut tl,
            telem_full(0.0, 76.0, 74.0, 60.0, 0.0, 8.0, 355.0, false),
            2.0,
        );
        tick_at(
            &mut tl,
            telem_full(0.0, 76.0, 74.0, 210.0, 0.0, 8.0, 355.0, false),
            3.0,
        );
        let f = tl.done.expect("graded");
        assert!((f.hdg_err - 5.0).abs() < 0.6, "{}", f.hdg_err);
        assert!(
            (5.5..=7.0).contains(&f.hdg),
            "5° of average drift lands mid-band: {}",
            f.hdg
        );
    }

    #[test]
    fn a_graded_attempt_rearms_after_parking() {
        let mut tl = TakeoffLesson::default();
        book_roll(&mut tl);
        book_climb(&mut tl, 4.0);
        assert!(tl.done.is_some(), "the gate graded the attempt");
        // Touch, roll out, park: a fresh attempt with the old card retired.
        tick_at(&mut tl, telem_full(1.0, 8.0, 8.0, 0.0, 0.0, 0.0, 0.0, true), 10.0);
        tick_at(&mut tl, telem_full(1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, true), 11.0);
        assert_eq!(tl.done, None);
        assert!(!tl.settled, "the fresh attempt hides the popup");
        // The parked frame after the re-arm composes the standing prompt.
        tick_at(&mut tl, telem_full(1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, true), 12.0);
        assert_eq!(
            tl.live.as_deref(),
            Some("RWY 36\nFULL POWER FOR THE ROLL"),
            "the standing start re-arms live"
        );
        assert!((VR - 57.0).abs() < 1e-9, "sanity on the graded centre");
    }
}
