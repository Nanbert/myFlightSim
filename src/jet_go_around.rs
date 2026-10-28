//! The 737's graded GO-AROUND lesson (M4): the missed approach, flown from
//! the call to the level-off.
//!
//! The M3 landing lesson's 1,000 ft gate says GO AROUND when the approach is
//! unstable — but the curriculum never taught the manoeuvre itself. This
//! lesson seats you on a mid-range ILS final, already at flaps 30 with the
//! gear down, and **scripts the call**: crossing 1,000 ft, `GO AROUND` fires
//! (the runway is no longer yours), and the grading starts on that frame:
//!
//! - **REACT** — seconds from the call to the first go-around input: thrust
//!   added, A/T N1 (the LVL CHG press sets it), or the pitch coming up.
//! - **PITCH** — the initial climb's attitude band (8–20°), until the gear
//!   is up.
//! - **GEAR UP** — seconds from POSITIVE RATE to wheels locked up.
//! - **FLAPS 15** — the retraction call, graded by the altitude it happened
//!   at.
//! - **CLIMB** — the speed band: 140–190 kt dirty, the 250-class clean.
//! - **LEVEL-OFF** — the missed-approach altitude (3,000 ft), the peak held
//!   against the window, the mode it was captured with.
//!
//! Continuing down past the call, a continued approach to a touchdown, a
//! steep bank and the go-around eating its speed are flat FAILs. The card
//! freezes at the capture; Enter re-arms, and the next descent through the
//! gate opens a fresh attempt.

use bevy::prelude::*;

use crate::app_state::{AppState, unpaused};
use crate::autopilot::{AtMode, VertMode};
use crate::lesson_telem::Telem;
use crate::mission::{ActiveLesson, Lesson};
use crate::scoring::{score, word};
use crate::sim::AircraftState;

/// The gate the call fires on: the same 1,000 ft fence M3 grades.
const GATE_AGL: f64 = 1000.0;
/// The missed-approach altitude the climb levels off at.
const MISSED_ALT: f64 = 3000.0;
/// The capture: at or above this, sinking under the level-off pace is the
/// level-off (the A/P's own capture window is 150 ft below the window).
const CAPTURE_FLOOR: f64 = 2900.0;
const CAPTURE_VS: f64 = 300.0;
/// Levelled below the missed altitude: this many seconds of dead flight
/// freezes the attempt where it sits (the capture row pays for the miss).
const LEVEL_VS: f64 = 100.0;
const LEVEL_FREEZE_S: f32 = 8.0;
/// A climb blowing through the missed altitude freezes here; the capture
/// row pays for the bust, it is not a flat FAIL.
const LEVEL_BUST: f64 = 4500.0;
/// How long a crossing callout holds the banner.
const CALLOUT_SECS: f32 = 2.5;
/// Reaction: full marks inside this, gone at the cap.
const REACT_IDEAL_S: f32 = 2.0;
const REACT_CAP_S: f32 = 6.0;
/// The throttle rise and the pitch rise that count as the reaction.
const REACT_THROTTLE: f64 = 0.15;
const REACT_PITCH: f64 = 3.0;
/// The initial climb's attitude band.
const PITCH_LO: f64 = 8.0;
const PITCH_HI: f64 = 20.0;
/// Positive rate: the gear-up clock's zero (the M2 latch).
const RATE_FPM: f64 = 100.0;
/// Gear up within this many seconds of positive rate earns its credit.
const GEAR_IDEAL_S: f32 = 3.0;
const GEAR_CAP_S: f32 = 13.0;
/// The flap lever leaving the landing setting, and the altitude window the
/// retraction call is graded against.
const FLAP15_DEG: f64 = 29.5;
const FLAP15_FULL: f64 = 2000.0;
const FLAP15_ZERO: f64 = 3200.0;
/// The dirty climb's band, then the clean 250-class (the M2 climb).
const DIRTY_LO: f64 = 140.0;
const DIRTY_HI: f64 = 190.0;
const CLEAN_LO: f64 = 180.0;
const CLEAN_HI: f64 = 260.0;
/// Dirty below this the go-around is eating its speed toward the fence.
const SLOW_CAS: f64 = 130.0;
/// Banking past this after the call is a flat FAIL (the M2 rule).
const STEEP_BANK: f64 = 30.0;
/// Continuing down this far past the call is a continued approach.
const SINK_FT: f64 = 200.0;
/// The level-off's capture accuracy: the peak against the window.
const LEVEL_TOL: f32 = 150.0;
const LEVEL_CAP: f32 = 450.0;
/// The landing flap setting the final's band rides (flaps 30).
const LAND_FLAP_NORM: f64 = 0.75;

// --- row weights (they sum to 1) ------------------------------------------------

const W_REACT: f32 = 0.20;
const W_PITCH: f32 = 0.15;
const W_GEAR: f32 = 0.15;
const W_CLEANUP: f32 = 0.15;
const W_CLIMB: f32 = 0.20;
const W_LEVEL: f32 = 0.15;

const FAIL_CONTINUED: &str =
    "FAIL - CONTINUED THE APPROACH\nThe call was made and the runway was flown to.";
const FAIL_DOWN: &str = "FAIL - CONTINUED DOWN\nThe descent ran on past the go-around call.";
const FAIL_STEEP: &str = "FAIL - STEEP BANK\nBanked past 30 degrees on the missed approach.";
const FAIL_SLOW: &str = "FAIL - SLOW\nThe go-around ate its speed toward the fence.";

#[derive(Clone, Copy, PartialEq, Default)]
enum Phase {
    /// On the final: un-graded, waiting for the scripted call.
    #[default]
    Final,
    /// The call is made: the go-around, the clean-up, the climb, the
    /// level-off.
    Initiated,
    /// A fail or the capture froze the attempt.
    Graded,
}

/// The trigger frame's readouts — the baselines the reaction is measured
/// against.
#[derive(Clone, Copy)]
struct TrigFrame {
    alt: f64,
    pitch: f64,
    throttle: f64,
    at: AtMode,
}

#[derive(Resource, Default)]
pub struct JetGoAroundMeter {
    phase: Phase,
    /// The live prompt text; `None` hides the centred card.
    pub(crate) live: Option<String>,
    /// A transient banner (the call, a callout) and when it retires.
    callout: Option<(String, f32)>,
    /// The scripted call fires once, on the airborne frame below the gate.
    gate_checked: bool,
    /// The trigger frame's baselines.
    trig: Option<TrigFrame>,
    /// The call just fired this frame — the system sends the voice event and
    /// clears the flag so the callout only plays once.
    call_fired: bool,
    /// Seconds since the call, and the reaction stamp against it.
    secs: f32,
    react_s: Option<f32>,
    /// The initial-climb pitch band: total and in-band time.
    pitch_secs: f32,
    pitch_in_secs: f32,
    /// The positive-rate latch and its stamp (the gear clock's zero).
    rate_latched: bool,
    rate_at: Option<f32>,
    /// Rate-to-gear-up seconds.
    gear_up_s: Option<f32>,
    /// The altitude the flap lever left the landing setting at.
    flap15_alt: Option<f64>,
    /// The climb band: total and out-of-band time.
    climb_secs: f32,
    climb_out_secs: f32,
    /// A LVL CHG frame after the call latches the missed-approach mode.
    saw_lvlchg: bool,
    /// The climb's peak, and the dead-flight seconds of a level below it.
    peak_alt: f64,
    level_secs: f32,
    /// The fail that parked the attempt, for the lesson-flow popup.
    pub(crate) fail: Option<&'static str>,
    /// The frozen finals of the attempt (at the capture).
    pub(crate) done: Option<Finals>,
    /// A graded attempt holds its card until it re-arms.
    pub(crate) settled: bool,
}

impl JetGoAroundMeter {
    /// A fresh attempt: the Flying entry's hand-out, and what the CONTINUE
    /// past a finished card consumes (the next descent through the gate
    /// opens a new attempt).
    pub(crate) fn rearm(&mut self) {
        *self = JetGoAroundMeter::default();
    }

    /// The finals card's rows: every graded row prints its number and word
    /// band, the weighted final on top.
    pub fn finals_text(&self) -> Option<String> {
        let f = self.done?;
        let react = f
            .react_s
            .map(|s| format!("{s:.1} s"))
            .unwrap_or_else(|| "NEVER".into());
        let gear = f
            .gear_up_s
            .map(|s| format!("{s:.1} s"))
            .unwrap_or_else(|| "STILL DOWN".into());
        let flaps = f
            .flap15_alt
            .map(|a| format!("{a:.0} ft"))
            .unwrap_or_else(|| "STILL 30".into());
        Some(format!(
            "737 GO-AROUND\n\
             REACT   {react:>9}          {r:4.1} {rw}\n\
             PITCH   {pp:3.0}% in band       {p:4.1} {pw}\n\
             GEAR UP {gear:>9}          {g:4.1} {gw}\n\
             FLAPS15 {flaps:>9}        {c:4.1} {cw}\n\
             CLIMB   {cp:3.0}% in band       {cl:4.1} {clw}\n\
             LEVEL   peak {peak:4.0} ft      {l:4.1} {lw}\n\
             FINAL   {f_:4.1}  {fw}",
            react = react,
            r = f.react,
            rw = word(f.react),
            pp = f.pitch_pct * 100.0,
            p = f.pitch,
            pw = word(f.pitch),
            gear = gear,
            g = f.gear,
            gw = word(f.gear),
            flaps = flaps,
            c = f.cleanup,
            cw = word(f.cleanup),
            cp = f.climb_pct * 100.0,
            cl = f.climb,
            clw = word(f.climb),
            peak = f.peak_alt,
            l = f.level,
            lw = word(f.level),
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
    react_s: Option<f32>,
    react: f32,
    pitch_pct: f32,
    pitch: f32,
    gear_up_s: Option<f32>,
    gear: f32,
    flap15_alt: Option<f64>,
    cleanup: f32,
    climb_pct: f32,
    climb: f32,
    peak_alt: f64,
    level: f32,
    final_: f32,
}

/// Feed one frame's readouts. Pure over its own state, for tests.
fn tick(m: &mut JetGoAroundMeter, t: Telem, now: f32, dt: f32) {
    if m.callout.as_ref().is_some_and(|(_, until)| now >= *until) {
        m.callout = None;
    }
    match m.phase {
        Phase::Final => {
            // The scripted call, once, on the airborne crossing below the
            // gate. The approach itself is not graded — this frame starts
            // the clocks.
            if !t.on_ground && !m.gate_checked && t.alt_ft < GATE_AGL {
                m.gate_checked = true;
                m.trig = Some(TrigFrame {
                    alt: t.alt_ft,
                    pitch: t.pitch_deg,
                    throttle: t.throttle,
                    at: t.at,
                });
                m.callout = Some(("GO AROUND - GO AROUND".to_owned(), now + CALLOUT_SECS));
                m.call_fired = true;
                m.phase = Phase::Initiated;
            }
        }
        Phase::Initiated => {
            let trig = m.trig.expect("the call implies its frame");
            m.secs += dt;
            // Fails first: nothing past them can save the attempt.
            if t.on_ground {
                m.fail = Some(FAIL_CONTINUED);
            } else if t.alt_ft < trig.alt - SINK_FT {
                m.fail = Some(FAIL_DOWN);
            } else if t.roll_deg.abs() > STEEP_BANK {
                m.fail = Some(FAIL_STEEP);
            } else if t.cas_kts < SLOW_CAS {
                m.fail = Some(FAIL_SLOW);
            }
            if m.fail.is_some() {
                m.phase = Phase::Graded;
                m.live = None;
                return;
            }
            // REACT: the first go-around input — thrust added, A/T N1 newly
            // set, or the pitch coming up.
            if m.react_s.is_none()
                && (t.throttle > trig.throttle + REACT_THROTTLE
                    || (t.at == AtMode::N1 && trig.at != AtMode::N1)
                    || t.pitch_deg > trig.pitch + REACT_PITCH)
            {
                m.react_s = Some(m.secs);
            }
            // The initial climb's attitude band, from the reaction to the
            // gear up: the transition before the pilot answers is not the
            // climb.
            if m.react_s.is_some() && m.gear_up_s.is_none() {
                m.pitch_secs += dt;
                if (PITCH_LO..=PITCH_HI).contains(&t.pitch_deg) {
                    m.pitch_in_secs += dt;
                }
            }
            // POSITIVE RATE, then the gear clock.
            if !m.rate_latched && t.vs_fpm > RATE_FPM {
                m.rate_latched = true;
                m.rate_at = Some(m.secs);
                m.callout = Some(("POSITIVE RATE".to_owned(), now + CALLOUT_SECS));
            }
            if m.rate_latched && m.gear_up_s.is_none() && t.gear_pos < 0.05 {
                m.gear_up_s = Some(m.secs - m.rate_at.expect("the latch implies its stamp"));
            }
            // The retraction call: the lever leaving the landing setting.
            if m.flap15_alt.is_none() && t.flap_deg < FLAP15_DEG {
                m.flap15_alt = Some(t.alt_ft);
            }
            if t.vert == VertMode::LvlChg {
                m.saw_lvlchg = true;
            }
            // The climb's speed band.
            m.climb_secs += dt;
            let (lo, hi) = if t.flap_deg >= 1.0 {
                (DIRTY_LO, DIRTY_HI)
            } else {
                (CLEAN_LO, CLEAN_HI)
            };
            if t.cas_kts < lo || t.cas_kts > hi {
                m.climb_out_secs += dt;
            }
            // The freeze: the capture at the missed altitude, a bust through
            // the bust line, or a level flown short of it.
            m.peak_alt = m.peak_alt.max(t.alt_ft);
            let captured = t.alt_ft >= CAPTURE_FLOOR && t.vs_fpm <= CAPTURE_VS;
            let bust = t.alt_ft > LEVEL_BUST;
            let levelled_short = t.vs_fpm.abs() <= LEVEL_VS;
            if levelled_short {
                m.level_secs += dt;
            } else {
                m.level_secs = 0.0;
            }
            if captured || bust || (levelled_short && m.level_secs >= LEVEL_FREEZE_S) {
                m.done = Some(finals(m, &t));
                m.settled = true;
                m.phase = Phase::Graded;
                m.live = None;
                return;
            }
        }
        Phase::Graded => {
            m.live = None;
            return;
        }
    }
    m.live = compose_live(m, &t, now);
}

/// The attempt frozen into finals at the capture.
fn finals(m: &JetGoAroundMeter, t: &Telem) -> Finals {
    // REACT: the clock to the first input.
    let react = m
        .react_s
        .map(|s| score(s, REACT_IDEAL_S, REACT_CAP_S))
        .unwrap_or(0.0);
    // PITCH: the fraction of the initial climb inside the attitude band.
    let pitch_pct = if m.pitch_secs > 0.0 {
        m.pitch_in_secs / m.pitch_secs
    } else {
        1.0
    };
    let pitch = (pitch_pct * 10.0).clamp(0.0, 10.0);
    // GEAR UP: the clock from positive rate to locked up.
    let gear = m
        .gear_up_s
        .map(|s| score((s - GEAR_IDEAL_S).max(0.0), 0.0, GEAR_CAP_S - GEAR_IDEAL_S))
        .unwrap_or(0.0);
    // FLAPS 15: the altitude the retraction call happened at.
    let cleanup = m
        .flap15_alt
        .map(|a| {
            score(
                (a - FLAP15_FULL).max(0.0) as f32,
                0.0,
                (FLAP15_ZERO - FLAP15_FULL) as f32,
            )
        })
        .unwrap_or(0.0);
    // CLIMB: the fraction of climb time inside the band.
    let climb_pct = if m.climb_secs > 0.0 {
        1.0 - m.climb_out_secs / m.climb_secs
    } else {
        1.0
    };
    let climb = (climb_pct * 10.0).clamp(0.0, 10.0);
    // LEVEL-OFF: the capture accuracy carries it, the mode keeps it honest.
    // A hand-flown level-off is respectable (a mid band); the beam modes are
    // not a missed-approach climb at all.
    let vert = if m.saw_lvlchg {
        10.0
    } else {
        match t.vert {
            VertMode::AltHold => 8.0,
            VertMode::Vs => 6.0,
            VertMode::Off => 5.0,
            _ => 0.0,
        }
    };
    let accuracy = score((m.peak_alt - MISSED_ALT).abs() as f32, LEVEL_TOL, LEVEL_CAP);
    let level = 0.5 * vert + 0.5 * accuracy;
    let final_ = W_REACT * react
        + W_PITCH * pitch
        + W_GEAR * gear
        + W_CLEANUP * cleanup
        + W_CLIMB * climb
        + W_LEVEL * level;
    Finals {
        react_s: m.react_s,
        react,
        pitch_pct,
        pitch,
        gear_up_s: m.gear_up_s,
        gear,
        flap15_alt: m.flap15_alt,
        cleanup,
        climb_pct,
        climb,
        peak_alt: m.peak_alt,
        level,
        final_,
    }
}

/// The banner-first live prompt: a callout while it runs, otherwise the
/// phase's PM flow. `None` while a card owns the display.
fn compose_live(m: &JetGoAroundMeter, t: &Telem, now: f32) -> Option<String> {
    if m.settled || m.done.is_some() || m.fail.is_some() {
        return None;
    }
    if let Some((text, until)) = &m.callout {
        if now < *until {
            return Some(text.clone());
        }
    }
    match m.phase {
        Phase::Final => {
            // The approach is not graded — but the call is coming, so the
            // band still coaches the stable gate it fires on.
            let vref30 = crate::landing::vref(LAND_FLAP_NORM);
            Some(format!(
                "FINAL\n{:.0} KT / {:.0}-{:.0}    {:.0} ft",
                t.cas_kts,
                vref30 + 5.0,
                vref30 + 25.0,
                t.alt_ft
            ))
        }
        Phase::Initiated => {
            if m.react_s.is_none() {
                Some(
                    "GO AROUND\nTHRUST (SHIFT) - PITCH UP\nA/P: 4 LVL CHG, MCP ALT 3000"
                        .to_owned(),
                )
            } else if !m.rate_latched {
                Some(format!("GO AROUND - CLIMBING\n{:.0} ft", t.alt_ft))
            } else if t.gear_pos >= 0.05 {
                Some(format!(
                    "POSITIVE RATE - GEAR UP (U)\n{:.0} KT    {:.0} ft",
                    t.cas_kts, t.alt_ft
                ))
            } else if t.flap_deg >= FLAP15_DEG {
                Some(format!(
                    "FLAPS 15 (F)\n{:.0} KT    {:.0} ft",
                    t.cas_kts, t.alt_ft
                ))
            } else {
                let (lo, hi) = if t.flap_deg >= 1.0 {
                    (DIRTY_LO, DIRTY_HI)
                } else {
                    (CLEAN_LO, CLEAN_HI)
                };
                Some(format!(
                    "CLIMB - LEVEL 3000 (4 LVL CHG)\n{:.0} KT / {:.0}-{:.0}    {:.0} ft",
                    t.cas_kts, lo, hi, t.alt_ft
                ))
            }
        }
        Phase::Graded => None,
    }
}

/// The update system: only the jet go-around lesson, only the home field.
fn step_m4(
    aircraft: Res<AircraftState>,
    ap: Res<crate::autopilot::Autopilot>,
    map: Res<crate::map::ActiveMap>,
    lesson: Res<ActiveLesson>,
    time: Res<Time>,
    mut m: ResMut<JetGoAroundMeter>,
    mut events: MessageWriter<crate::atc_voice::AtcVoiceEvent>,
) {
    if map.0.is_some() || lesson.0 != Some(Lesson::JetGoAround) {
        return;
    }
    tick(
        &mut m,
        Telem {
            at: ap.at,
            vert: ap.vert,
            ..Telem::sample(&aircraft)
        },
        time.elapsed_secs(),
        time.delta_secs(),
    );
    // The scripted call just fired: send the voice event once.
    if m.call_fired {
        events.write(crate::atc_voice::AtcVoiceEvent(crate::atc_voice::AtcVoiceKind::GoAround));
        m.call_fired = false;
    }
}

pub struct JetGoAroundLessonPlugin;

impl Plugin for JetGoAroundLessonPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<JetGoAroundMeter>()
            .add_systems(OnEnter(AppState::Flying), reset_jet_go_around)
            .add_systems(
                Update,
                // Fresh readouts in; the flow's watcher reads what we froze.
                step_m4
                    .after(crate::sim::publish_state)
                    .before(crate::lesson_flow::watch)
                    .run_if(unpaused),
            );
    }
}

/// A fresh flight hands over a fresh attempt.
fn reset_jet_go_around(mut m: ResMut<JetGoAroundMeter>) {
    m.rearm();
}

// --- tests -------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::{FAIL_CONTINUED, FAIL_DOWN, FAIL_SLOW, FAIL_STEEP, JetGoAroundMeter, Telem, tick};
    use crate::autopilot::{AtMode, VertMode};

    const DT: f32 = 1.0 / 30.0;
    const NOW: f32 = 100.0;

    /// A frame with the book seat values; mutate the returned struct for the
    /// off-book cases. The seat: 140 kt, flaps 30, gear down, descending the
    /// beam at the throttle it holds.
    fn frame(alt: f64, cas: f64, vs: f64) -> Telem {
        Telem {
            on_ground: false,
            cas_kts: cas,
            alt_ft: alt,
            vs_fpm: vs,
            pitch_deg: 2.5,
            roll_deg: 0.0,
            throttle: 0.3,
            flap_deg: 30.0,
            gear_pos: 1.0,
            at: AtMode::Off,
            vert: VertMode::Off,
            ..Telem::default()
        }
    }

    fn tick_at(m: &mut JetGoAroundMeter, t: Telem, secs: f32) {
        tick(m, t, NOW + secs, DT);
    }

    /// Feed `ticks` frames of the same state, advancing the clock by DT and
    /// the altitude by the frame's own vs — the clocks run on fed frames, so
    /// a timed stamp needs a dense run, not one frame a second apart.
    fn fly(m: &mut JetGoAroundMeter, secs: &mut f32, ticks: usize, mut t: Telem) {
        for _ in 0..ticks {
            tick_at(m, t, *secs);
            *secs += DT;
            t.alt_ft += t.vs_fpm / 60.0 * DT as f64;
        }
    }

    /// The book go-around: the call at the gate, thrust at ~1 s, positive
    /// rate, gear up 2 s after it, flaps 15 on the climb, LVL CHG to a
    /// 3,000 ft capture.
    fn textbook(m: &mut JetGoAroundMeter) {
        // Above the gate: quiet.
        tick_at(m, frame(1010.0, 140.0, -700.0), 0.0);
        // The crossing: the call fires.
        tick_at(m, frame(990.0, 140.0, -700.0), 0.5);
        let mut secs = 0.55;
        // A second of numb descent, then thrust and pitch at ~1 s.
        fly(m, &mut secs, 30, frame(990.0, 140.0, -700.0));
        let mut reacted = frame(975.0, 140.0, 0.0);
        reacted.throttle = 1.0;
        reacted.pitch_deg = 12.0;
        fly(m, &mut secs, 30, reacted);
        // The climb establishes: positive rate, gear still down 2 s.
        let mut climbing = frame(990.0, 140.0, 2000.0);
        climbing.throttle = 1.0;
        climbing.pitch_deg = 13.0;
        fly(m, &mut secs, 60, climbing);
        // Gear up at ~2 s after the rate latch, still climbing.
        let mut gear_up = frame(1120.0, 145.0, 2000.0);
        gear_up.throttle = 1.0;
        gear_up.pitch_deg = 13.0;
        gear_up.gear_pos = 0.0;
        fly(m, &mut secs, 10, gear_up);
        // Flaps 15 on the climb, dirty, in the band; LVL CHG with N1.
        let mut flaps15 = frame(1200.0, 150.0, 2200.0);
        flaps15.throttle = 1.0;
        flaps15.pitch_deg = 11.0;
        flaps15.flap_deg = 15.0;
        flaps15.vert = VertMode::LvlChg;
        flaps15.at = AtMode::N1;
        fly(m, &mut secs, 60, flaps15);
        // Clean and accelerating into the 250-class climb.
        let mut clean = frame(2600.0, 215.0, 1500.0);
        clean.throttle = 1.0;
        clean.pitch_deg = 8.0;
        clean.flap_deg = 0.0;
        clean.vert = VertMode::LvlChg;
        clean.at = AtMode::N1;
        fly(m, &mut secs, 60, clean);
        // The capture: at the window, sinking under the pace.
        let mut captured = frame(2950.0, 215.0, 280.0);
        captured.throttle = 0.6;
        captured.pitch_deg = 3.0;
        captured.flap_deg = 0.0;
        captured.vert = VertMode::AltHold;
        captured.at = AtMode::Speed;
        tick_at(m, captured, secs);
    }

    #[test]
    fn a_textbook_go_around_tops_the_card() {
        let mut m = JetGoAroundMeter::default();
        textbook(&mut m);
        let f = m.done.expect("the capture froze the finals");
        assert!(m.settled, "the card owns the display at the capture");
        let react_s = f.react_s.expect("the book reaction");
        assert!(
            (0.8..1.4).contains(&react_s),
            "thrust at ~1 s after the call: {react_s}"
        );
        assert!(f.react > 9.5, "thrust at 1 s: {f:?}");
        assert!(f.pitch > 8.5, "the climb pitched 11-13: {f:?}");
        assert!(f.gear > 9.5, "gear up 2 s after the rate: {f:?}");
        assert!(f.cleanup > 9.5, "flaps 15 on the low climb: {f:?}");
        assert!(f.climb > 9.5, "always in the band: {f:?}");
        assert!(f.level > 9.0, "LVL CHG to a 2,950 capture: {f:?}");
        assert!(f.final_ > 9.0, "every row near-perfect: {f:?}");
        assert_eq!(m.live, None, "a graded attempt hides the prompt");
        let text = m.finals_text().expect("the card");
        assert!(text.contains("FINAL"), "{text}");
        assert!(text.contains("737 GO-AROUND"), "{text}");
    }

    #[test]
    fn a_sluggish_reaction_pays_the_row() {
        let mut m = JetGoAroundMeter::default();
        tick_at(&mut m, frame(1010.0, 140.0, -700.0), 0.0);
        tick_at(&mut m, frame(990.0, 140.0, -700.0), 0.5);
        let mut secs = 0.55;
        // Numb hands: 4.5 s of a gentle sink before the pitch comes up.
        fly(&mut m, &mut secs, 135, frame(990.0, 140.0, -150.0));
        let mut late = frame(975.0, 140.0, 0.0);
        late.pitch_deg = 10.0;
        fly(&mut m, &mut secs, 1, late);
        let react_s = m.react_s.expect("the pitch-up counts");
        assert!((4.0..5.5).contains(&react_s), "{react_s}");
        // Then the book climb to the capture, gear up at once.
        let mut climbing = frame(1100.0, 150.0, 2000.0);
        climbing.pitch_deg = 12.0;
        climbing.throttle = 1.0;
        climbing.gear_pos = 0.0;
        climbing.flap_deg = 15.0;
        climbing.vert = VertMode::LvlChg;
        climbing.at = AtMode::N1;
        fly(&mut m, &mut secs, 60, climbing);
        let mut captured = climbing;
        captured.alt_ft = 2950.0;
        captured.vs_fpm = 280.0;
        captured.vert = VertMode::AltHold;
        captured.at = AtMode::Speed;
        tick_at(&mut m, captured, secs);
        let f = m.done.expect("the capture froze the finals");
        assert!(
            f.react < 4.5,
            "~4.5 s of nothing pays the row, not the card: {f:?}"
        );
        assert!(f.final_ > 6.0, "the rest was flown: {f:?}");
    }

    #[test]
    fn continuing_down_past_the_call_fails() {
        let mut m = JetGoAroundMeter::default();
        tick_at(&mut m, frame(1010.0, 140.0, -700.0), 0.0);
        tick_at(&mut m, frame(990.0, 140.0, -700.0), 0.5);
        let mut secs = 0.55;
        // The descent runs on: 200 ft under the call is the fence.
        fly(&mut m, &mut secs, 15, frame(990.0, 140.0, -700.0));
        assert_eq!(m.fail, None, "inside the fence still");
        fly(&mut m, &mut secs, 600, frame(960.0, 140.0, -700.0));
        assert_eq!(m.fail, Some(FAIL_DOWN));
        assert_eq!(m.done, None);
        assert_eq!(m.live, None, "the popup owns the display");
    }

    #[test]
    fn a_touchdown_after_the_call_fails() {
        let mut m = JetGoAroundMeter::default();
        tick_at(&mut m, frame(990.0, 140.0, -700.0), 0.0);
        let mut landed = frame(10.0, 130.0, -100.0);
        landed.on_ground = true;
        tick_at(&mut m, landed, 1.0);
        assert_eq!(m.fail, Some(FAIL_CONTINUED));
    }

    #[test]
    fn a_steep_bank_fails() {
        let mut m = JetGoAroundMeter::default();
        tick_at(&mut m, frame(990.0, 140.0, -700.0), 0.0);
        let mut steep = frame(950.0, 140.0, -400.0);
        steep.roll_deg = 35.0;
        tick_at(&mut m, steep, 1.0);
        assert_eq!(m.fail, Some(FAIL_STEEP));
    }

    #[test]
    fn eating_the_speed_fails() {
        let mut m = JetGoAroundMeter::default();
        tick_at(&mut m, frame(990.0, 140.0, -700.0), 0.0);
        let mut secs = 0.55;
        // Pitched up but the thrust never came: the speed bleeds off.
        let mut bleeding = frame(970.0, 140.0, -50.0);
        bleeding.pitch_deg = 8.0;
        fly(&mut m, &mut secs, 30, bleeding);
        fly(&mut m, &mut secs, 120, frame(965.0, 125.0, -50.0));
        assert_eq!(m.fail, Some(FAIL_SLOW));
    }

    #[test]
    fn an_overshot_level_off_pays_the_row_not_the_card() {
        let mut m = JetGoAroundMeter::default();
        tick_at(&mut m, frame(990.0, 140.0, -700.0), 0.0);
        let mut secs = 0.55;
        // Rocketed through the window, captured late at 3,600.
        let mut climbing = frame(990.0, 145.0, 2200.0);
        climbing.throttle = 1.0;
        climbing.pitch_deg = 14.0;
        climbing.gear_pos = 0.0;
        climbing.flap_deg = 15.0;
        climbing.vert = VertMode::LvlChg;
        climbing.at = AtMode::N1;
        fly(&mut m, &mut secs, 75, climbing);
        let mut captured = climbing;
        captured.alt_ft = 3600.0;
        captured.vs_fpm = 200.0;
        captured.vert = VertMode::AltHold;
        captured.at = AtMode::Speed;
        tick_at(&mut m, captured, secs);
        let f = m.done.expect("the capture froze the finals");
        assert_eq!(m.fail, None, "a bust is graded, not failed");
        assert!(
            (f.level - 5.0).abs() < 0.1,
            "600 ft over: the mode half pays, the accuracy half dies: {f:?}"
        );
        assert!(f.final_ > 8.0, "the climb itself was flown: {f:?}");
    }

    #[test]
    fn a_level_flown_short_still_freezes() {
        let mut m = JetGoAroundMeter::default();
        tick_at(&mut m, frame(990.0, 140.0, -700.0), 0.0);
        let mut secs = 0.55;
        // Climbed to 1,600 and levelled off there, dead flight.
        let mut climbing = frame(990.0, 150.0, 800.0);
        climbing.throttle = 0.5;
        climbing.pitch_deg = 6.0;
        fly(&mut m, &mut secs, 25, climbing);
        let mut level = frame(1610.0, 150.0, 50.0);
        level.throttle = 0.4;
        level.pitch_deg = 2.0;
        fly(&mut m, &mut secs, 300, level);
        let f = m.done.expect("the dead flight freezes the attempt");
        assert!(f.level < 3.0, "2,800 ft short of the window: {f:?}");
        assert!(f.react_s.is_some(), "the pitch-up still counted: {f:?}");
    }

    #[test]
    fn the_prompt_ladder_plays_the_calls() {
        let mut m = JetGoAroundMeter::default();
        // Before the gate: the un-graded final's band.
        tick_at(&mut m, frame(1010.0, 140.0, -700.0), 0.0);
        let live = m.live.clone().expect("the final's coach");
        assert!(live.contains("FINAL"), "{live}");
        // The crossing: the call owns the banner.
        tick_at(&mut m, frame(990.0, 140.0, -700.0), 0.5);
        let live = m.live.clone().expect("the call");
        assert!(live.contains("GO AROUND"), "{live}");
        // Past the banner, the reaction prompt stands.
        tick_at(&mut m, frame(970.0, 140.0, -600.0), 4.0);
        let live = m.live.clone().expect("the reaction prompt");
        assert!(live.contains("THRUST"), "{live}");
        // Reacted: the rate latch fires the POSITIVE RATE banner.
        let mut reacted = frame(1010.0, 140.0, 300.0);
        reacted.throttle = 1.0;
        reacted.pitch_deg = 12.0;
        tick_at(&mut m, reacted, 5.0);
        let live = m.live.clone().expect("the rate banner");
        assert!(live.contains("POSITIVE RATE"), "{live}");
        // Past the banner: the gear prompt stands while the gear travels.
        let mut travelling = reacted;
        travelling.gear_pos = 0.5;
        tick_at(&mut m, travelling, 9.0);
        let live = m.live.clone().expect("the standing gear prompt");
        assert!(live.contains("GEAR UP"), "{live}");
        // Gear up: the flaps call.
        let mut gear_up = reacted;
        gear_up.gear_pos = 0.0;
        tick_at(&mut m, gear_up, 10.0);
        let live = m.live.clone().expect("the flaps call");
        assert!(live.contains("FLAPS 15"), "{live}");
        // Flaps 15: the climb to the level-off.
        let mut flaps15 = gear_up;
        flaps15.flap_deg = 15.0;
        tick_at(&mut m, flaps15, 11.0);
        let live = m.live.clone().expect("the climb prompt");
        assert!(live.contains("CLIMB"), "{live}");
    }

    #[test]
    fn a_rearmed_attempt_reopens_at_the_next_gate() {
        let mut m = JetGoAroundMeter::default();
        textbook(&mut m);
        assert!(m.done.is_some());
        // CONTINUE past the card: quiet above the gate, and the next
        // descent through it opens a fresh attempt.
        m.rearm();
        tick_at(&mut m, frame(2000.0, 210.0, 0.0), 30.0);
        assert_eq!(m.done, None, "no attempt above the gate");
        assert!(m.live.is_some(), "the standing final's coach");
        tick_at(&mut m, frame(990.0, 140.0, -700.0), 31.0);
        assert_eq!(m.done, None, "the new attempt is live, not graded");
        assert!(m.trig.is_some(), "the call fired again");
    }
}
