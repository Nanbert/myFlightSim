//! The 737's graded RTO lesson (M5): the rejected takeoff, the other half
//! of the M2 departure.
//!
//! M2 teaches the V1 call and the continued takeoff; this one lines you up
//! on the same numbers and **scripts the reject**: takeoff thrust, the 80 KT
//! thrust crosscheck, and at 100 kt the STOP call fires — well before V1.
//! From that frame the drill is graded:
//!
//! - **SETUP** — the takeoff was real: flaps 5 and the thrust actually set
//!   before the call.
//! - **REACT** — seconds from the call to the throttles closed.
//! - **PEAK** — how far past the call the roll ran before the decel bit.
//! - **STOP** — the clock from the call to a walking pace (the quick pilot
//!   in `sim::m5_probe` stops in 7.3 s; the row's budget starts at 10).
//! - **CENTRE** — the |offset| average over the whole roll and decel.
//! - **REVERSE** — the buckets out soon after the call.
//!
//! Rejecting through V1 or flying the call through to a liftoff are flat
//! FAILs. The card freezes at the stop; Enter re-arms on the runway.

use bevy::prelude::*;

use crate::app_state::{AppState, unpaused};
use crate::mission::{ActiveLesson, Lesson};
use crate::pfd::vr_for_flaps;
use crate::scoring::{score, word};
use crate::sim::AircraftState;

/// The scripted STOP call: the crossing where the drill starts.
const CALL_GS: f64 = 100.0;
/// The thrust crosscheck riding just under it (the M2 rhythm).
const CHECK_GS: f64 = 80.0;
/// Rolling below this is standing still, not a roll worth sampling.
const ROLLING_GS: f64 = 3.0;
/// The stop gate: 5 kt on the wheels, held 3 s (the M3 stop).
const STOP_GS: f64 = 5.0;
const STOP_SECS: f32 = 3.0;
/// How long a crossing callout holds the banner.
const CALLOUT_SECS: f32 = 2.5;
/// A throttle at or under this is closed — the reaction stamp.
const THROTTLE_OFF: f64 = 0.1;
/// Reaction: full marks inside this, gone at the cap.
const REACT_IDEAL_S: f32 = 2.0;
const REACT_CAP_S: f32 = 6.0;
/// The peak's discipline: 10 kt past the call still earns full marks, 40
/// past is gone.
const PEAK_TOL_KT: f64 = 10.0;
const PEAK_CAP_KT: f64 = 40.0;
/// The stop clock: the quick pilot's 7.3 s rounds to a 10 s full mark, and
/// a 25 s trundle is gone.
const STOP_IDEAL_S: f32 = 10.0;
const STOP_CAP_S: f32 = 25.0;
/// Reverse out within this many seconds of the call earns its credit.
const REV_IDEAL_S: f32 = 2.0;
const REV_CAP_S: f32 = 8.0;
/// Reverse position above this is deployed.
const REV_ON: f64 = 0.1;
/// Centreline anchors (the M2 RLW's own).
const CENTRE_TOL: f32 = 1.5;
const CENTRE_CAP: f32 = 8.0;
/// The setup row: thrust at or above this was a takeoff, below this a
/// token push, and the flaps the drill briefs.
const THRUST_SET: f64 = 0.95;
const THRUST_FLOOR: f64 = 0.6;
const FLAP_TAKEOFF_DEG: f64 = 4.5;

// --- row weights (they sum to 1) ------------------------------------------------

const W_SETUP: f32 = 0.10;
const W_REACT: f32 = 0.30;
const W_PEAK: f32 = 0.15;
const W_STOP: f32 = 0.25;
const W_CENTRE: f32 = 0.10;
const W_REVERSE: f32 = 0.10;

const FAIL_PAST_V1: &str = "FAIL - REJECTED PAST V1\nThe reject ran through the decision speed.";
const FAIL_CONTINUED: &str =
    "FAIL - THE TAKEOFF CONTINUED\nThe STOP call was flown through to a liftoff.";

#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
enum Phase {
    /// Lined up: flaps, thrust, the roll to the call.
    #[default]
    Roll,
    /// The call is made: idle, brakes, reverse, the stop.
    Called,
    /// A fail or the stop froze the attempt.
    Graded,
}

/// One frame's readouts the tick needs: pure and testable, like the other
/// jet lessons' `Telem`.
#[derive(Clone, Copy)]
pub struct Telem {
    on_ground: bool,
    gs_kts: f64,
    /// World position — the centreline offset rides on x.
    pos: Vec3,
    on_runway: bool,
    throttle: f64,
    flap_deg: f64,
    reverse_pos: f64,
}

#[derive(Resource, Default)]
pub struct JetRtoMeter {
    phase: Phase,
    /// The live prompt text; `None` hides the centred card.
    pub(crate) live: Option<String>,
    /// A transient banner (the crosscheck, the call) and when it retires.
    callout: Option<(String, f32)>,
    /// Ground speed on the previous tick — the crossing checks.
    last_gs: f64,
    /// The strongest the levers saw before the call (the SETUP row's input).
    max_throttle: f64,
    /// Seconds since the call, and the stamps against it.
    secs: f32,
    react_s: Option<f32>,
    rev_s: Option<f32>,
    stop_s: Option<f32>,
    /// The flap setting at the call — the V1 the FAIL rides on.
    flap_at_call: f64,
    /// The call just fired this frame — the system sends the voice event and
    /// clears the flag so the callout only plays once.
    call_fired: bool,
    /// The peak after the call, and the centreline sampler.
    peak_gs: f64,
    centre_sum: f32,
    centre_n: u32,
    /// Consecutive stop-gate seconds on the wheels.
    stop_secs: f32,
    /// The fail that parked the attempt, for the lesson-flow popup.
    pub(crate) fail: Option<&'static str>,
    /// The frozen finals of the attempt (at the stop).
    pub(crate) done: Option<Finals>,
    /// A graded attempt holds its card until it re-arms.
    pub(crate) settled: bool,
}

impl JetRtoMeter {
    /// A fresh attempt: the Flying entry's hand-out, and what the CONTINUE
    /// past a finished card consumes (the aircraft is stopped on the runway;
    /// the next thrust-and-roll opens a new attempt).
    pub(crate) fn rearm(&mut self) {
        *self = JetRtoMeter::default();
    }

    /// The finals card's rows: every graded row prints its number and word
    /// band, the weighted final on top.
    pub fn finals_text(&self) -> Option<String> {
        let f = self.done?;
        let react = f
            .react_s
            .map(|s| format!("{s:.1} s"))
            .unwrap_or_else(|| "NEVER".into());
        let rev = f
            .rev_s
            .map(|s| format!("{s:.1} s"))
            .unwrap_or_else(|| "STOWED".into());
        Some(format!(
            "737 RTO\n\
             SETUP   thrust {th:4.2} / flaps {fl:2.0}  {s:4.1} {sw}\n\
             REACT   {react:>9}          {r:4.1} {rw}\n\
             PEAK    {pk:3.0} kt             {p:4.1} {pw}\n\
             STOP    call+{st:4.1} s       {sp:4.1} {spw}\n\
             CENTRE  Δ avg {ce:+5.1} m         {c:4.1} {cw}\n\
             REVERSE {rev:>9}          {v:4.1} {vw}\n\
             FINAL   {f_:4.1}  {fw}",
            th = f.thrust_set,
            fl = f.flap_at_call,
            s = f.setup,
            sw = word(f.setup),
            react = react,
            r = f.react,
            rw = word(f.react),
            pk = f.peak_gs,
            p = f.peak,
            pw = word(f.peak),
            st = f.stop_s.unwrap_or(STOP_CAP_S),
            sp = f.stop,
            spw = word(f.stop),
            ce = f.centre_avg,
            c = f.centre,
            cw = word(f.centre),
            rev = rev,
            v = f.reverse,
            vw = word(f.reverse),
            f_ = f.final_,
            fw = word(f.final_),
        ))
    }
}

/// The frozen finals: each row a 0–10 score, the final their weighted
/// average (bad rows pull it down, nothing zeroes it — the flat FAILs are
/// popups, outside the scorer).
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct Finals {
    thrust_set: f64,
    flap_at_call: f64,
    setup: f32,
    react_s: Option<f32>,
    react: f32,
    peak_gs: f64,
    peak: f32,
    stop_s: Option<f32>,
    stop: f32,
    centre_avg: f32,
    centre: f32,
    rev_s: Option<f32>,
    reverse: f32,
    final_: f32,
}

/// Feed one frame's readouts. Pure over its own state, for tests.
fn tick(m: &mut JetRtoMeter, t: Telem, now: f32, dt: f32) {
    if m.callout.as_ref().is_some_and(|(_, until)| now >= *until) {
        m.callout = None;
    }
    match m.phase {
        Phase::Roll => {
            // The takeoff was real: the strongest the levers saw.
            m.max_throttle = m.max_throttle.max(t.throttle);
            // The centreline sampler runs from the roll to the stop.
            if t.on_ground && t.on_runway && t.gs_kts > ROLLING_GS {
                m.centre_sum += t.pos.x.abs();
                m.centre_n += 1;
            }
            // The 80 KT crosscheck closes the thrust-set flow (the M2 call).
            if t.on_ground && m.last_gs < CHECK_GS && t.gs_kts >= CHECK_GS {
                let text = if t.throttle > THRUST_SET {
                    "80 KT - THRUST SET".to_owned()
                } else {
                    "80 KT - CHECK THRUST".to_owned()
                };
                m.callout = Some((text, now + CALLOUT_SECS));
            }
            // The scripted STOP call: the crossing the drill starts on.
            if t.on_ground && m.last_gs < CALL_GS && t.gs_kts >= CALL_GS {
                m.flap_at_call = t.flap_deg;
                m.callout = Some(("STOP - STOP".to_owned(), now + CALLOUT_SECS));
                m.call_fired = true;
                m.phase = Phase::Called;
            }
        }
        Phase::Called => {
            m.secs += dt;
            // Fails first: nothing past them can save the attempt.
            let v1 = vr_for_flaps(m.flap_at_call / 40.0) - 5.0;
            if !t.on_ground {
                m.fail = Some(FAIL_CONTINUED);
            } else if t.gs_kts >= v1 {
                m.fail = Some(FAIL_PAST_V1);
            }
            if m.fail.is_some() {
                m.phase = Phase::Graded;
                m.live = None;
                return;
            }
            // The centreline sampler runs on.
            if t.on_ground && t.on_runway && t.gs_kts > ROLLING_GS {
                m.centre_sum += t.pos.x.abs();
                m.centre_n += 1;
            }
            // The stamps: throttles closed, buckets out, the peak.
            m.peak_gs = m.peak_gs.max(t.gs_kts);
            if m.react_s.is_none() && t.throttle <= THROTTLE_OFF {
                m.react_s = Some(m.secs);
            }
            if m.rev_s.is_none() && t.reverse_pos > REV_ON {
                m.rev_s = Some(m.secs);
            }
            if m.stop_s.is_none() && t.gs_kts < STOP_GS {
                m.stop_s = Some(m.secs);
            }
            // The stop gate: a walking pace held 3 s freezes the card.
            if t.gs_kts < STOP_GS {
                m.stop_secs += dt;
                if m.stop_secs >= STOP_SECS {
                    m.done = Some(finals(m));
                    m.settled = true;
                    m.phase = Phase::Graded;
                    m.live = None;
                    return;
                }
            } else {
                m.stop_secs = 0.0;
            }
        }
        Phase::Graded => {
            m.live = None;
            return;
        }
    }
    m.last_gs = t.gs_kts;
    m.live = compose_live(m, &t, now);
}

/// The attempt frozen into finals at the stop.
fn finals(m: &JetRtoMeter) -> Finals {
    // SETUP: the thrust that was set, and the briefed flaps.
    let thrust_term = score(
        (THRUST_SET - m.max_throttle).max(0.0) as f32,
        0.0,
        (THRUST_SET - THRUST_FLOOR) as f32,
    );
    let flap_term = if m.flap_at_call >= FLAP_TAKEOFF_DEG {
        10.0
    } else {
        0.0
    };
    let setup = 0.7 * thrust_term + 0.3 * flap_term;
    // REACT: the clock to the throttles closed.
    let react = m
        .react_s
        .map(|s| score(s, REACT_IDEAL_S, REACT_CAP_S))
        .unwrap_or(0.0);
    // PEAK: how far past the call the roll ran.
    let peak = score(
        (m.peak_gs - CALL_GS - PEAK_TOL_KT).max(0.0) as f32,
        0.0,
        (PEAK_CAP_KT - PEAK_TOL_KT) as f32,
    );
    // STOP: the clock from the call to a walking pace.
    let stop = m
        .stop_s
        .map(|s| score((s - STOP_IDEAL_S).max(0.0), 0.0, STOP_CAP_S - STOP_IDEAL_S))
        .unwrap_or(0.0);
    // CENTRE: the |offset| average over the whole run.
    let centre_avg = if m.centre_n > 0 {
        m.centre_sum / m.centre_n as f32
    } else {
        0.0
    };
    let centre = score(centre_avg, CENTRE_TOL, CENTRE_CAP);
    // REVERSE: the buckets out soon after the call.
    let reverse = m
        .rev_s
        .map(|s| score((s - REV_IDEAL_S).max(0.0), 0.0, REV_CAP_S - REV_IDEAL_S))
        .unwrap_or(0.0);
    let final_ = W_SETUP * setup
        + W_REACT * react
        + W_PEAK * peak
        + W_STOP * stop
        + W_CENTRE * centre
        + W_REVERSE * reverse;
    Finals {
        thrust_set: m.max_throttle,
        flap_at_call: m.flap_at_call,
        setup,
        react_s: m.react_s,
        react,
        peak_gs: m.peak_gs,
        peak,
        stop_s: m.stop_s,
        stop,
        centre_avg,
        centre,
        rev_s: m.rev_s,
        reverse,
        final_,
    }
}

/// The banner-first live prompt: a callout while it runs, otherwise the
/// phase's PM flow. `None` while a card owns the display.
fn compose_live(m: &JetRtoMeter, t: &Telem, now: f32) -> Option<String> {
    if m.settled || m.done.is_some() || m.fail.is_some() {
        return None;
    }
    if let Some((text, until)) = &m.callout {
        if now < *until {
            return Some(text.clone());
        }
    }
    match m.phase {
        Phase::Roll => {
            if t.on_ground && t.gs_kts > 1.0 {
                Some(format!(
                    "T.O. ROLL\nRLW Δ {:+.1} m    {:.0} KT",
                    t.pos.x, t.gs_kts
                ))
            } else {
                Some("RWY 36 - FLAPS 5 (F)\nTHRUST FOR TAKEOFF".to_owned())
            }
        }
        Phase::Called => {
            if m.react_s.is_none() {
                Some(
                    "STOP - STOP\nTHROTTLES IDLE - BRAKES (B)\nREVERSE (K)"
                        .to_owned(),
                )
            } else if t.reverse_pos >= REV_ON && t.gs_kts < 80.0 {
                Some(format!("STOW REVERSE (K)\n{:.0} KT", t.gs_kts))
            } else if t.gs_kts > STOP_GS {
                Some(format!("DECELERATING\n{:.0} KT", t.gs_kts))
            } else {
                Some("HOLDING THE STOP".to_owned())
            }
        }
        Phase::Graded => None,
    }
}

/// The update system: only the jet RTO lesson, only the home field.
fn step_m5(
    aircraft: Res<AircraftState>,
    airfield: Res<crate::airfield::Airfield>,
    map: Res<crate::map::ActiveMap>,
    lesson: Res<ActiveLesson>,
    time: Res<Time>,
    mut m: ResMut<JetRtoMeter>,
    mut events: MessageWriter<crate::atc_voice::AtcVoiceEvent>,
) {
    if map.0.is_some() || lesson.0 != Some(Lesson::JetRto) {
        return;
    }
    let default_spec = crate::airfield::AirfieldSpec::zsnm();
    let spec = airfield.0.as_ref().unwrap_or(&default_spec);
    tick(
        &mut m,
        Telem {
            on_ground: aircraft.on_ground,
            gs_kts: aircraft.gs_kts,
            pos: aircraft.pos,
            on_runway: spec.on_runway(aircraft.pos.x, aircraft.pos.z),
            throttle: aircraft.throttle,
            flap_deg: aircraft.flap_deg,
            reverse_pos: aircraft.reverse_pos,
        },
        time.elapsed_secs(),
        time.delta_secs(),
    );
    // The scripted call just fired: send the voice event once.
    if m.call_fired {
        events.write(
            crate::atc_voice::AtcVoiceEvent(crate::atc_voice::AtcVoiceKind::Stop),
        );
        m.call_fired = false;
    }
}

pub struct JetRtoLessonPlugin;

impl Plugin for JetRtoLessonPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<JetRtoMeter>()
            .add_systems(OnEnter(AppState::Flying), reset_jet_rto)
            .add_systems(
                Update,
                // Fresh readouts in; the flow's watcher reads what we froze.
                step_m5
                    .after(crate::sim::publish_state)
                    .before(crate::lesson_flow::watch)
                    .run_if(unpaused),
            );
    }
}

/// A fresh flight hands over a fresh attempt.
fn reset_jet_rto(mut m: ResMut<JetRtoMeter>) {
    m.rearm();
}

// --- tests -------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use bevy::prelude::*;

    use super::{FAIL_CONTINUED, FAIL_PAST_V1, JetRtoMeter, Telem, tick};

    const DT: f32 = 1.0 / 30.0;
    const NOW: f32 = 100.0;

    /// A frame on the runway at the given ground speed and lever; mutate the
    /// returned struct for the off-book cases.
    fn frame(gs: f64, throttle: f64, rev: f64) -> Telem {
        Telem {
            on_ground: true,
            gs_kts: gs,
            pos: Vec3::new(0.3, 0.0, 0.0),
            on_runway: true,
            throttle,
            flap_deg: 5.0,
            reverse_pos: rev,
        }
    }

    fn tick_at(m: &mut JetRtoMeter, t: Telem, secs: f32) {
        tick(m, t, NOW + secs, DT);
    }

    /// Feed `ticks` frames of the same state, advancing the clock by DT —
    /// the clocks run on fed frames, so a timed stamp needs a dense run.
    fn fly(m: &mut JetRtoMeter, secs: &mut f32, ticks: usize, t: Telem) {
        for _ in 0..ticks {
            tick_at(m, t, *secs);
            *secs += DT;
        }
    }

    /// The book RTO: flaps 5 and full thrust from the numbers, the 80 KT
    /// crosscheck, the STOP call at 100, throttles closed at ~1 s, buckets
    /// out at 1.5, braked through the peak to a walking pace at ~8 s, held.
    fn textbook(m: &mut JetRtoMeter) {
        tick_at(m, frame(0.0, 0.0, 0.0), 0.0);
        let mut secs = 0.05;
        // The roll: full thrust, the speed building in steps.
        for gs in [20.0, 50.0, 79.0, 81.0, 95.0, 99.0] {
            fly(m, &mut secs, 30, frame(gs, 1.0, 0.0));
        }
        // The call at the crossing.
        tick_at(m, frame(100.5, 1.0, 0.0), secs);
        secs += DT;
        // Throttles closed at ~1 s.
        fly(m, &mut secs, 30, frame(102.0, 1.0, 0.0));
        // Buckets out at ~1.5 s, the decel through the peak to a walk.
        for gs in [103.0, 90.0, 60.0, 30.0] {
            fly(m, &mut secs, 30, frame(gs, 0.0, 1.0));
        }
        // Reverse stowed, braked to the stop gate at ~8 s.
        for gs in [10.0, 6.0, 4.0] {
            fly(m, &mut secs, 20, frame(gs, 0.0, 0.0));
        }
        // Held 3 s.
        fly(m, &mut secs, 100, frame(3.0, 0.0, 0.0));
    }

    #[test]
    fn a_textbook_rto_tops_the_card() {
        let mut m = JetRtoMeter::default();
        textbook(&mut m);
        let f = m.done.expect("the stop froze the finals");
        assert!(m.settled, "the card owns the display at the stop");
        assert!(f.thrust_set > 0.99, "the takeoff thrust was real: {f:?}");
        assert!(f.setup > 9.5, "flaps 5 and full thrust: {f:?}");
        let react_s = f.react_s.expect("the book reaction");
        assert!((0.8..1.5).contains(&react_s), "{react_s}");
        assert!(f.react > 9.5, "closed at ~1 s: {f:?}");
        assert!(f.peak > 9.0, "the roll ran ~3 kt past the call: {f:?}");
        assert!(f.stop > 8.5, "stopped at ~8 s: {f:?}");
        assert!(f.centre > 9.5, "the 0.3 m line: {f:?}");
        assert!(f.reverse > 9.0, "buckets out at ~1.5 s: {f:?}");
        assert!(f.final_ > 9.0, "every row near-perfect: {f:?}");
        assert_eq!(m.live, None, "a graded attempt hides the prompt");
        let text = m.finals_text().expect("the card");
        assert!(text.contains("737 RTO"), "{text}");
        assert!(text.contains("FINAL"), "{text}");
    }

    #[test]
    fn a_sluggish_reject_pays_the_rows() {
        let mut m = JetRtoMeter::default();
        tick_at(&mut m, frame(0.0, 0.0, 0.0), 0.0);
        let mut secs = 0.05;
        for gs in [50.0, 99.0] {
            fly(&mut m, &mut secs, 30, frame(gs, 1.0, 0.0));
        }
        tick_at(&mut m, frame(100.5, 1.0, 0.0), secs);
        secs += DT;
        // Numb hands: 4.5 s of full thrust (the roll runs to ~130), then
        // idle, no reverse, a long pedal-only trundle to the stop.
        for gs in [110.0, 120.0, 130.0] {
            fly(&mut m, &mut secs, 45, frame(gs, 1.0, 0.0));
        }
        for gs in [95.0, 70.0, 40.0, 20.0] {
            fly(&mut m, &mut secs, 90, frame(gs, 0.0, 0.0));
        }
        fly(&mut m, &mut secs, 60, frame(4.0, 0.0, 0.0));
        fly(&mut m, &mut secs, 100, frame(3.0, 0.0, 0.0));
        let f = m.done.expect("the stop froze the finals");
        assert!(f.react < 4.0, "4.5 s of nothing pays the row: {f:?}");
        assert!(f.peak < 4.5, "the roll ran 30 kt past the call: {f:?}");
        assert_eq!(f.reverse, 0.0, "no buckets out: {f:?}");
        assert!(f.stop < 8.0, "a 16 s trundle pays: {f:?}");
        assert!(f.final_ > 4.0, "the reject still happened: {f:?}");
    }

    #[test]
    fn flying_the_call_to_a_liftoff_fails() {
        let mut m = JetRtoMeter::default();
        tick_at(&mut m, frame(0.0, 0.0, 0.0), 0.0);
        let mut secs = 0.05;
        fly(&mut m, &mut secs, 30, frame(99.0, 1.0, 0.0));
        tick_at(&mut m, frame(100.5, 1.0, 0.0), secs);
        // The takeoff continued: wheels off after the call.
        let mut airborne = frame(140.0, 1.0, 0.0);
        airborne.on_ground = false;
        tick_at(&mut m, airborne, 3.0);
        assert_eq!(m.fail, Some(FAIL_CONTINUED));
        assert_eq!(m.done, None);
        assert_eq!(m.live, None, "the popup owns the display");
    }

    #[test]
    fn rejecting_through_v1_fails() {
        let mut m = JetRtoMeter::default();
        tick_at(&mut m, frame(0.0, 0.0, 0.0), 0.0);
        let mut secs = 0.05;
        fly(&mut m, &mut secs, 30, frame(99.0, 1.0, 0.0));
        tick_at(&mut m, frame(100.5, 1.0, 0.0), secs);
        // The levers never closed: through the flaps-5 decision speed.
        fly(&mut m, &mut secs, 120, frame(180.0, 1.0, 0.0));
        assert_eq!(m.fail, Some(FAIL_PAST_V1));
    }

    #[test]
    fn a_token_thrust_setup_pays_the_row() {
        let mut m = JetRtoMeter::default();
        tick_at(&mut m, frame(0.0, 0.0, 0.0), 0.0);
        let mut secs = 0.05;
        // A 70 % push (and flaps never set): a slow roll to the call.
        for gs in [50.0, 99.0] {
            let mut f = frame(gs, 0.70, 0.0);
            f.flap_deg = 0.0;
            fly(&mut m, &mut secs, 40, f);
        }
        let mut call = frame(100.5, 0.70, 0.0);
        call.flap_deg = 0.0;
        tick_at(&mut m, call, secs);
        secs += DT;
        fly(&mut m, &mut secs, 30, frame(102.0, 0.0, 0.0));
        for gs in [90.0, 60.0, 30.0] {
            fly(&mut m, &mut secs, 30, frame(gs, 0.0, 1.0));
        }
        fly(&mut m, &mut secs, 40, frame(4.0, 0.0, 0.0));
        fly(&mut m, &mut secs, 100, frame(3.0, 0.0, 0.0));
        let f = m.done.expect("the stop froze the finals");
        assert!(
            f.setup < 5.0,
            "a 70 % push with the flaps up pays the setup: {f:?}"
        );
        assert!(f.final_ > 5.0, "the reject itself was flown: {f:?}");
    }

    #[test]
    fn the_prompt_ladder_plays_the_calls() {
        let mut m = JetRtoMeter::default();
        // Standing start: the brief.
        tick_at(&mut m, frame(0.0, 0.0, 0.0), 0.0);
        let live = m.live.clone().expect("the standing brief");
        assert!(live.contains("FLAPS 5"), "{live}");
        // Rolling: the roll line.
        tick_at(&mut m, frame(30.0, 1.0, 0.0), 1.0);
        let live = m.live.clone().expect("the roll line");
        assert!(live.contains("ROLL"), "{live}");
        // The 80 KT crosscheck banner.
        tick_at(&mut m, frame(81.0, 1.0, 0.0), 2.0);
        let live = m.live.clone().expect("the crosscheck");
        assert!(live.contains("80 KT"), "{live}");
        // The STOP call banner.
        tick_at(&mut m, frame(100.5, 1.0, 0.0), 3.0);
        let live = m.live.clone().expect("the call");
        assert!(live.contains("STOP"), "{live}");
        // Past the banner: the reject prompt stands.
        tick_at(&mut m, frame(102.0, 1.0, 0.0), 6.0);
        let live = m.live.clone().expect("the reject prompt");
        assert!(live.contains("THROTTLES IDLE"), "{live}");
        // Reacted, buckets out, slowing through 80: stow the reverse.
        let mut secs = 6.05;
        fly(&mut m, &mut secs, 30, frame(100.0, 0.0, 1.0));
        tick_at(&mut m, frame(60.0, 0.0, 1.0), secs);
        let live = m.live.clone().expect("the stow call");
        assert!(live.contains("STOW REVERSE"), "{live}");
        // Buckets stowed, still rolling: the decel line.
        tick_at(&mut m, frame(60.0, 0.0, 0.0), secs + 1.0);
        let live = m.live.clone().expect("the decel line");
        assert!(live.contains("DECELERATING"), "{live}");
    }

    #[test]
    fn a_rearmed_attempt_rolls_again() {
        let mut m = JetRtoMeter::default();
        textbook(&mut m);
        assert!(m.done.is_some());
        // CONTINUE past the card: stopped on the runway, the next
        // thrust-and-roll opens a fresh attempt.
        m.rearm();
        tick_at(&mut m, frame(3.0, 0.0, 0.0), 30.0);
        assert_eq!(m.done, None, "no attempt while standing");
        assert!(m.live.is_some(), "the standing brief");
        let mut secs = 30.05;
        fly(&mut m, &mut secs, 30, frame(99.0, 1.0, 0.0));
        tick_at(&mut m, frame(100.5, 1.0, 0.0), secs);
        assert_eq!(m.phase, super::Phase::Called, "the call fired again");
    }
}
