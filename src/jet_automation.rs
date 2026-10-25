//! The 737's graded AUTOMATION lesson (M6): the full coupled arrival, flown
//! by the box from CMD to the ILS established.
//!
//! The seat hangs 20 km south and 4 km east of ZSNM's course at 3,000 ft,
//! clean, with the A/P off. From there the lesson is the airline's mode
//! discipline, one leg at a time — **window first, button second**:
//!
//! - **CMD + LVL CHG** — engage (G), MCP ALT 4000, then 4: the climb starts.
//! - **The ATC stop** — crossing 3,500 ft the call fires: `2 ALT HOLD` now,
//!   or the climb runs on past 3,900 and the leg fails. The capture-now is
//!   the one button press that beats a window.
//! - **HDG SEL** — the mode is already live after engagement (the FMA says
//!   so); the discipline is setting the bug to 330 before the jet banks.
//! - **V/S** — MCP ALT 1000, V/S 2800 down, then 3: the descent that meets
//!   the beam.
//! - **APP** — arm both beams, the A/T over to the arrival speed (IAS 180),
//!   and the LOC then G/S take — established, the card freezes.
//!
//! The A/P flies all of it: a disconnect, a missed window that sends the
//! box the wrong way, a leg that never reaches its target — each is a flat
//! FAIL. The rows grade the discipline, not the physics: what was set
//! before what, how fast the stop came, how clean the intercept rode.

use bevy::prelude::*;

use crate::app_state::{AppState, unpaused};
use crate::autopilot::{AtMode, LatMode, VertMode};
use crate::mission::{ActiveLesson, Lesson};
use crate::scoring::{score, word};
use crate::sim::AircraftState;

/// The runway-36 threshold's world z — the along-course distance reads off
/// the aircraft's own z (the arrival flies north, z shrinking).
const THRESHOLD_Z: f32 = 140.0;
/// The engage deadline: the box flies this lesson, and it starts now.
const ENGAGE_DEADLINE_S: f32 = 45.0;
/// The climb the ATC stops: the call, the fence, the reaction budget.
const STOP_ALT: f64 = 3500.0;
const STOP_FENCE: f64 = 3900.0;
const STOP_IDEAL_S: f32 = 3.0;
const STOP_CAP_S: f32 = 10.0;
/// The windows the legs brief (the check rides on the press frame).
const CLIMB_ALT: f64 = 4000.0;
const TURN_BUG: f64 = 330.0;
const DESCEND_ALT: f64 = 1000.0;
/// The window tolerances at the press frames.
const ALT_TOL: f64 = 150.0;
const BUG_TOL: f64 = 10.0;
/// The descent the leg briefs, and the band the V/S window must ride.
const VS_BAND_LO: f64 = -3500.0;
const VS_BAND_HI: f64 = -800.0;
/// The bank the A/P is limited to; the band samples the same number.
const BANK_LIMIT: f64 = 25.0;
/// The roll past which the jet is "in the turn" (the HDG SEL leg's onset).
const TURN_ROLL: f64 = 3.0;
/// The intercept heading the turn was briefed to.
const INTERCEPT_ERR_TOL: f32 = 15.0;
const INTERCEPT_ERR_CAP: f32 = 45.0;
/// The leg budgets: the mode was set but its target never came.
const LEG1_BUDGET_S: f32 = 60.0;
const LEG2_BUDGET_S: f32 = 60.0;
const LEG3_BUDGET_S: f32 = 60.0;
const ENGAGE_BUDGET_S: f32 = 90.0;
/// The missed-approach fence: past this without the G/S the arrival is
/// flown into the ground.
const ILS_MISS_M: f32 = 3000.0;
/// How long an established hold runs before the card freezes.
const HOLD_SECS: f32 = 3.0;
/// How long a crossing callout holds the banner.
const CALLOUT_SECS: f32 = 2.5;
/// The intercept tightness: the G/S deviation at the capture frame.
const GS_DEV_TOL: f64 = 0.10;
const GS_DEV_CAP: f64 = 0.25;

// --- row weights (they sum to 1) ------------------------------------------------

const W_WINDOW: f32 = 0.25;
const W_STOP: f32 = 0.15;
const W_HDG: f32 = 0.20;
const W_VS: f32 = 0.20;
const W_APP: f32 = 0.20;

const FAIL_NO_AP: &str =
    "FAIL - THE A/P NEVER CAME ON\nEngage (G) and fly the legs on the box.";
const FAIL_AP_OFF: &str =
    "FAIL - HAND-FLYING\nThis arrival is the box's; the A/P stays on.";
const FAIL_CLIMB: &str =
    "FAIL - CLIMB NOT STOPPED\nATC said stop the climb; the jet ran on past 3,900.";
const FAIL_STEEP: &str = "FAIL - STEEP BANK\nBanked past 30 degrees on the coupled arrival.";
const FAIL_LEG: &str = "FAIL - THE LEG NEVER FLEW";
const FAIL_ILS: &str = "FAIL - ILS MISSED\nThe beams never took before the field.";

#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
enum Phase {
    /// Airborne, box off: the engage deadline runs.
    #[default]
    PreArm,
    /// The A/P flies: the legs advance on the modes and the flags.
    Flying,
    /// A fail or the established hold froze the attempt.
    Graded,
}

/// One frame's readouts the tick needs: pure and testable, like the other
/// jet lessons' `Telem`.
#[derive(Clone, Copy)]
pub struct Telem {
    ap: bool,
    lat: LatMode,
    vert: VertMode,
    app_armed: bool,
    at: AtMode,
    mcp_hdg: f64,
    mcp_alt: f64,
    mcp_vs: f64,
    mcp_ias: f64,
    cas_kts: f64,
    alt_ft: f64,
    /// The actual magnetic heading (the bug is the target, not the fact).
    hdg_deg: f64,
    roll_deg: f64,
    /// ILS glideslope deviation at the frame, degrees (+ above); zero
    /// outside the volume — the capture frame is always inside it.
    gs_dev_deg: f64,
    /// World position — the along-course distance reads off z.
    pos: Vec3,
}

#[derive(Resource, Default)]
pub struct JetAutomationMeter {
    phase: Phase,
    /// The live prompt text; `None` hides the centred card.
    pub(crate) live: Option<String>,
    /// A transient banner (the ATC call) and when it retires.
    callout: Option<(String, f32)>,
    /// Seconds since spawn — the engage deadline's clock.
    secs: f32,
    /// The engagement stamp (the legs' clocks' zero).
    engage_at: Option<f32>,
    /// Leg 1: the LVL CHG frame, its window check, the ATC call and the
    /// ALT HOLD answer.
    lvlchg_at: Option<f32>,
    window1_ok: Option<bool>,
    stop_called: bool,
    stop_at: f32,
    stop_pressed: bool,
    stop_react: Option<f32>,
    /// Leg 2: the turn onset (the A/P banking on the bug), its window
    /// check, and the bank sampler.
    turn_at: Option<f32>,
    window2_ok: Option<bool>,
    bank_secs: f32,
    bank_in_secs: f32,
    /// Leg 3: the V/S frame, its window check, the band sampler.
    vs_at: Option<f32>,
    window3_ok: Option<bool>,
    vs_band_secs: f32,
    vs_band_in_secs: f32,
    /// Leg 4: the beams — LOC first, then the G/S and its hold.
    armed: bool,
    loc_at: Option<f32>,
    gs_at: Option<f32>,
    gs_dev: f64,
    gs_alt: f64,
    gs_hdg_err: f32,
    hold_secs: f32,
    /// The fail that parked the attempt, plus the leg that said so.
    pub(crate) fail: Option<&'static str>,
    pub(crate) fail_detail: Option<String>,
    /// The frozen finals of the attempt (at the established hold).
    pub(crate) done: Option<Finals>,
    /// A graded attempt holds its card until it re-arms.
    pub(crate) settled: bool,
}

impl JetAutomationMeter {
    /// A fresh attempt: the Flying entry's hand-out (the card has no
    /// Continue — the lesson ends established — so this is the spawn's
    /// hand-out and the retry's).
    pub(crate) fn rearm(&mut self) {
        *self = JetAutomationMeter::default();
    }

    /// The finals card's rows: every graded row prints its number and word
    /// band, the weighted final on top.
    pub fn finals_text(&self) -> Option<String> {
        let f = self.done?;
        let stop = f
            .stop_react
            .map(|s| format!("{s:.1} s"))
            .unwrap_or_else(|| "NEVER".into());
        Some(format!(
            "737 AUTOMATION\n\
             WINDOW  {wp:3.0}% set before mode  {w:4.1} {ww}\n\
             STOP    {stop:>9}           {s:4.1} {sw}\n\
             HDG     {bp:3.0}% bank / int {i:+3.0}°  {h:4.1} {hw}\n\
             V/S     {vp:3.0}% band / dev {d:+4.2}° {v:4.1} {vw}\n\
             APP     loc+g/s, A/T {at:>4}    {a:4.1} {aw}\n\
             FINAL   {f_:4.1}  {fw}",
            wp = f.window_pct * 100.0,
            w = f.window,
            ww = word(f.window),
            stop = stop,
            s = f.stop,
            sw = word(f.stop),
            bp = f.bank_pct * 100.0,
            i = f.intercept_err,
            h = f.hdg,
            hw = word(f.hdg),
            vp = f.vs_band_pct * 100.0,
            d = f.gs_dev,
            v = f.vs,
            vw = word(f.vs),
            at = if f.at_ok { "SPD" } else { "OFF" },
            a = f.app,
            aw = word(f.app),
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
    window_pct: f32,
    window: f32,
    stop_react: Option<f32>,
    stop: f32,
    bank_pct: f32,
    intercept_err: f32,
    hdg: f32,
    vs_band_pct: f32,
    gs_dev: f64,
    vs: f32,
    at_ok: bool,
    app: f32,
    final_: f32,
}

/// Signed heading difference wrapped to ±180°, degrees.
fn heading_err(hdg: f64, reference: f64) -> f64 {
    (hdg - reference + 180.0).rem_euclid(360.0) - 180.0
}

/// Feed one frame's readouts. Pure over its own state, for tests.
fn tick(m: &mut JetAutomationMeter, t: Telem, now: f32, dt: f32) {
    if m.callout.as_ref().is_some_and(|(_, until)| now >= *until) {
        m.callout = None;
    }
    m.secs += dt;
    match m.phase {
        Phase::PreArm => {
            if t.ap {
                m.engage_at = Some(m.secs);
                m.phase = Phase::Flying;
            } else if m.secs > ENGAGE_DEADLINE_S {
                m.fail = Some(FAIL_NO_AP);
                m.phase = Phase::Graded;
                m.live = None;
                return;
            }
        }
        Phase::Flying => {
            // Fails first: the box flies this, or the attempt is over.
            if !t.ap {
                m.fail = Some(FAIL_AP_OFF);
            } else if t.roll_deg.abs() > 30.0 {
                m.fail = Some(FAIL_STEEP);
            } else if m.stop_called && !m.stop_pressed && t.alt_ft > STOP_FENCE {
                m.fail = Some(FAIL_CLIMB);
            } else if m.gs_at.is_none() && (t.pos.z as f64 - THRESHOLD_Z as f64) < ILS_MISS_M as f64
            {
                m.fail = Some(FAIL_ILS);
            }
            // The leg budgets: the mode was set, the target never came.
            if m.fail.is_none() {
                if let Some(engage) = m.engage_at {
                    if m.lvlchg_at.is_none() && m.secs - engage > ENGAGE_BUDGET_S {
                        m.fail = Some(FAIL_LEG);
                        m.fail_detail =
                            Some("LVL CHG: the climb never started.\nMCP ALT 4000, then 4.".into());
                    }
                }
                if m.fail.is_none() {
                    if let Some(at) = m.lvlchg_at {
                        if !m.stop_called && m.secs - at > LEG1_BUDGET_S {
                            m.fail = Some(FAIL_LEG);
                            m.fail_detail = Some(
                                "LVL CHG: the climb never reached the ATC stop.".into(),
                            );
                        }
                    }
                }
                if m.fail.is_none() {
                    if let Some(at) = m.turn_at {
                        if m.gs_at.is_none() && m.secs - at > LEG2_BUDGET_S {
                            let err = heading_err(t.hdg_deg, TURN_BUG).abs();
                            if err > 5.0 {
                                m.fail = Some(FAIL_LEG);
                                m.fail_detail = Some(
                                    "HDG SEL: the jet never settled on the intercept.".into(),
                                );
                            }
                        }
                    }
                }
                if m.fail.is_none() {
                    if let Some(at) = m.vs_at {
                        if m.gs_at.is_none() && m.secs - at > LEG3_BUDGET_S && t.alt_ft > 3000.0 {
                            m.fail = Some(FAIL_LEG);
                            m.fail_detail =
                                Some("V/S: the descent never happened.\nCheck the window.".into());
                        }
                    }
                }
            }
            if m.fail.is_some() {
                m.phase = Phase::Graded;
                m.live = None;
                return;
            }

            // Leg 1: the climb mode, its window, the ATC call.
            if m.lvlchg_at.is_none() && t.vert == VertMode::LvlChg {
                m.lvlchg_at = Some(m.secs);
                m.window1_ok = Some((t.mcp_alt - CLIMB_ALT).abs() <= ALT_TOL);
            }
            if !m.stop_called && m.lvlchg_at.is_some() && t.alt_ft >= STOP_ALT {
                m.stop_called = true;
                m.stop_at = m.secs;
                m.callout =
                    Some(("ATC - STOP THE CLIMB - 2 ALT HOLD".to_owned(), now + CALLOUT_SECS));
            }
            // The capture-now: ALT HOLD over the climb (the window capture
            // at 4,000 would arrive only after the fence).
            if m.stop_called && !m.stop_pressed && t.vert == VertMode::AltHold {
                m.stop_pressed = true;
                m.stop_react = Some(m.secs - m.stop_at);
            }
            // Leg 2: the turn onset — the A/P banking on the bug.
            if m.turn_at.is_none() && m.stop_pressed && t.roll_deg.abs() > TURN_ROLL {
                m.turn_at = Some(m.secs);
                m.window2_ok = Some(heading_err(t.mcp_hdg, TURN_BUG).abs() <= BUG_TOL);
            }
            if let Some(turn) = m.turn_at {
                if m.secs >= turn {
                    m.bank_secs += dt;
                    if t.roll_deg.abs() <= BANK_LIMIT {
                        m.bank_in_secs += dt;
                    }
                }
            }
            // Leg 3: the V/S frame, its window, the band.
            if m.vs_at.is_none() && t.vert == VertMode::Vs {
                m.vs_at = Some(m.secs);
                m.window3_ok = Some(
                    (t.mcp_alt - DESCEND_ALT).abs() <= ALT_TOL && t.mcp_vs <= VS_BAND_HI,
                );
            }
            if let Some(vs) = m.vs_at {
                if m.secs >= vs {
                    m.vs_band_secs += dt;
                    if (VS_BAND_LO..=VS_BAND_HI).contains(&t.mcp_vs) {
                        m.vs_band_in_secs += dt;
                    }
                }
            }
            // Leg 4: the beams, in order, then the established hold.
            if !m.armed && t.app_armed {
                m.armed = true;
            }
            if m.armed && m.loc_at.is_none() && t.lat == LatMode::Loc {
                m.loc_at = Some(m.secs);
            }
            if m.loc_at.is_some() && m.gs_at.is_none() && t.vert == VertMode::Gs {
                m.gs_at = Some(m.secs);
                m.gs_dev = t.gs_dev_deg;
                m.gs_alt = t.alt_ft;
                m.gs_hdg_err = heading_err(t.hdg_deg, TURN_BUG).abs() as f32;
            }
            if let Some(_gs) = m.gs_at {
                if t.vert == VertMode::Gs && t.ap {
                    m.hold_secs += dt;
                    if m.hold_secs >= HOLD_SECS {
                        m.done = Some(finals(m, &t));
                        m.settled = true;
                        m.phase = Phase::Graded;
                        m.live = None;
                        return;
                    }
                } else {
                    m.hold_secs = 0.0;
                }
            }
        }
        Phase::Graded => {
            m.live = None;
            return;
        }
    }
    m.live = compose_live(m, &t, now);
}

/// The attempt frozen into finals at the established hold.
fn finals(m: &JetAutomationMeter, t: &Telem) -> Finals {
    // WINDOW: the three set-before-mode checks, averaged.
    let w1 = m.window1_ok.map(|ok| if ok { 10.0 } else { 0.0 }).unwrap_or(0.0);
    let w2 = m.window2_ok.map(|ok| if ok { 10.0 } else { 0.0 }).unwrap_or(0.0);
    let w3 = m.window3_ok.map(|ok| if ok { 10.0 } else { 0.0 }).unwrap_or(0.0);
    let window_pct = (w1 + w2 + w3) / 3.0 / 10.0;
    let window = (w1 + w2 + w3) / 3.0;
    // STOP: the clock from the ATC call to the capture-now.
    let stop = m
        .stop_react
        .map(|s| score(s, STOP_IDEAL_S, STOP_CAP_S))
        .unwrap_or(0.0);
    // HDG: the bank band carries it, the intercept angle keeps it honest.
    let bank_pct = if m.bank_secs > 0.0 {
        m.bank_in_secs / m.bank_secs
    } else {
        1.0
    };
    let bank = (bank_pct * 10.0).clamp(0.0, 10.0);
    let intercept_err = if m.turn_at.is_some() {
        m.gs_hdg_err
    } else {
        INTERCEPT_ERR_CAP
    };
    let intercept = score(intercept_err, INTERCEPT_ERR_TOL, INTERCEPT_ERR_CAP);
    let hdg = 0.5 * bank + 0.5 * intercept;
    // V/S: the window band, then how tight the intercept rode.
    let vs_band_pct = if m.vs_band_secs > 0.0 {
        m.vs_band_in_secs / m.vs_band_secs
    } else {
        1.0
    };
    let vs_band = (vs_band_pct * 10.0).clamp(0.0, 10.0);
    let tight = score(m.gs_dev.abs() as f32, GS_DEV_TOL as f32, GS_DEV_CAP as f32);
    let vs = 0.5 * vs_band + 0.5 * tight;
    // APP: the beams in order, the A/T over and the speed set.
    let loc = if m.loc_at.is_some() { 10.0 } else { 0.0 };
    let gs = if m.gs_at.is_some() { 10.0 } else { 0.0 };
    let at_ok = t.at == AtMode::Speed && t.mcp_ias <= 200.0;
    let spd = if at_ok { 10.0 } else { 0.0 };
    let app = 0.3 * loc + 0.3 * gs + 0.4 * spd;
    let final_ =
        W_WINDOW * window + W_STOP * stop + W_HDG * hdg + W_VS * vs + W_APP * app;
    Finals {
        window_pct,
        window,
        stop_react: m.stop_react,
        stop,
        bank_pct,
        intercept_err,
        hdg,
        vs_band_pct,
        gs_dev: m.gs_dev,
        vs,
        at_ok,
        app,
        final_,
    }
}

/// The banner-first live prompt: a callout while it runs, otherwise the
/// leg's flow. `None` while a card owns the display.
fn compose_live(m: &JetAutomationMeter, t: &Telem, now: f32) -> Option<String> {
    if m.settled || m.done.is_some() || m.fail.is_some() {
        return None;
    }
    if let Some((text, until)) = &m.callout {
        if now < *until {
            return Some(text.clone());
        }
    }
    let dist_km = (t.pos.z - THRESHOLD_Z).max(0.0) / 1000.0;
    match m.phase {
        Phase::PreArm => Some(
            "G - ENGAGE THE A/P\nthen MCP ALT 4000 (arrows)\n4 LVL CHG".to_owned(),
        ),
        Phase::Flying => {
            if m.gs_at.is_some() {
                Some("ESTABLISHED - HOLDING".to_owned())
            } else if m.loc_at.is_some() {
                Some(format!("LOC CAPTURED - G/S NEXT\n{:.0} ft", t.alt_ft))
            } else if m.armed {
                Some("APP ARMED - LOC NEXT\nA/T SPD - IAS 180 ([)".to_owned())
            } else if m.vs_at.is_some() {
                Some("V/S - DESCENDING\nAPP (6) WHEN READY".to_owned())
            } else if m.turn_at.is_some() {
                Some(format!(
                    "INTERCEPT 330 - AT {:.0} KM\nALT 1000 (arrows), V/S 2800 DOWN (PgDn)\nTHEN 3",
                    dist_km
                ))
            } else if m.stop_pressed {
                Some("HDG SEL IS LIVE - BUG 330 (arrows)\nintercept the course".to_owned())
            } else if m.stop_called {
                Some("ATC - STOP THE CLIMB\n2 ALT HOLD NOW".to_owned())
            } else if m.lvlchg_at.is_some() {
                Some(format!(
                    "LVL CHG - CLIMB 4000\n{:.0} ft    {:.0} KT",
                    t.alt_ft, t.cas_kts
                ))
            } else {
                Some("SET MCP ALT 4000 (arrows)\nTHEN 4 LVL CHG".to_owned())
            }
        }
        Phase::Graded => None,
    }
}

/// The update system: only the jet automation lesson, only the home field.
fn step_m6(
    aircraft: Res<AircraftState>,
    ap: Res<crate::autopilot::Autopilot>,
    mcp: Res<crate::autopilot::Mcp>,
    ils: Res<crate::ils::Ils>,
    map: Res<crate::map::ActiveMap>,
    lesson: Res<ActiveLesson>,
    time: Res<Time>,
    mut m: ResMut<JetAutomationMeter>,
) {
    if map.0.is_some() || lesson.0 != Some(Lesson::JetAutomation) {
        return;
    }
    tick(
        &mut m,
        Telem {
            ap: ap.engaged,
            lat: ap.lat,
            vert: ap.vert,
            app_armed: ap.app_armed,
            at: ap.at,
            mcp_hdg: mcp.hdg_deg,
            mcp_alt: mcp.alt_ft,
            mcp_vs: mcp.vs_fpm,
            mcp_ias: mcp.ias_kts,
            cas_kts: aircraft.cas_kts,
            alt_ft: aircraft.alt_ft,
            hdg_deg: aircraft.heading_deg,
            roll_deg: aircraft.roll_deg,
            gs_dev_deg: if ils.available { ils.gs_dev_deg } else { 0.0 },
            pos: aircraft.pos,
        },
        time.elapsed_secs(),
        time.delta_secs(),
    );
}

pub struct JetAutomationLessonPlugin;

impl Plugin for JetAutomationLessonPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<JetAutomationMeter>()
            .add_systems(OnEnter(AppState::Flying), reset_jet_automation)
            .add_systems(
                Update,
                // Fresh readouts in; the flow's watcher reads what we froze.
                step_m6
                    .after(crate::sim::publish_state)
                    .before(crate::lesson_flow::watch)
                    .run_if(unpaused),
            );
    }
}

/// A fresh flight hands over a fresh attempt.
fn reset_jet_automation(mut m: ResMut<JetAutomationMeter>) {
    m.rearm();
}

// --- tests -------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::{FAIL_AP_OFF, FAIL_CLIMB, FAIL_ILS, FAIL_LEG, FAIL_NO_AP, JetAutomationMeter, Telem, tick};
    use crate::autopilot::{AtMode, LatMode, VertMode};
    use bevy::prelude::*;

    const DT: f32 = 1.0 / 30.0;
    const NOW: f32 = 100.0;
    /// The threshold's world z; the seat hangs 20 km south of it.
    const THRESHOLD_Z: f32 = 140.0;
    const SEAT_Z: f32 = THRESHOLD_Z + 20_000.0;

    /// A frame mid-lesson; mutate the returned struct per leg. The book
    /// values: A/P on in HDG SEL + ALT HOLD (the engagement defaults), A/T
    /// off, the MCP windows at their defaults, level at the seat.
    fn frame(alt: f64, z: f32) -> Telem {
        Telem {
            ap: true,
            lat: LatMode::HdgSel,
            vert: VertMode::AltHold,
            app_armed: false,
            at: AtMode::Off,
            mcp_hdg: 0.0,
            mcp_alt: 4000.0,
            mcp_vs: 0.0,
            mcp_ias: 210.0,
            cas_kts: 210.0,
            alt_ft: alt,
            hdg_deg: 0.0,
            roll_deg: 0.0,
            gs_dev_deg: 0.0,
            pos: Vec3::new(0.0, 0.0, z),
        }
    }

    fn tick_at(m: &mut JetAutomationMeter, t: Telem, secs: f32) {
        tick(m, t, NOW + secs, DT);
    }

    /// Feed `ticks` frames of a evolving state: the clock advances by DT,
    /// the altitude by `dalt_fpm`, the world z by `dz_mps` (positive =
    /// closing on the field).
    fn run(
        m: &mut JetAutomationMeter,
        secs: &mut f32,
        ticks: usize,
        mut t: Telem,
        dalt_fpm: f64,
        dz_mps: f64,
    ) {
        for _ in 0..ticks {
            tick_at(m, t, *secs);
            *secs += DT;
            t.alt_ft += dalt_fpm / 60.0 * DT as f64;
            t.pos.z -= (dz_mps * DT as f64) as f32;
        }
    }

    /// The book arrival: engage, MCP ALT 4000 + LVL CHG, the ATC stop
    /// answered at once, the bug 330 turn, V/S 2800 down from the level,
    /// APP armed with the A/T at 180, and the beams taking — established.
    fn textbook(m: &mut JetAutomationMeter) {
        let mut secs = 0.0;
        // The seat: box off for a beat, then G.
        let mut pre = frame(3000.0, SEAT_Z);
        pre.ap = false;
        tick_at(m, pre, secs);
        secs += DT;
        // Engagement: HDG SEL + ALT HOLD, the A/P's own defaults.
        let mut engaged = frame(3000.0, SEAT_Z);
        engaged.at = AtMode::Off;
        tick_at(m, engaged, secs);
        secs += DT;
        // MCP ALT 4000 first, then 4: the climb mode, the window green.
        let mut lvlchg = frame(3000.0, SEAT_Z);
        lvlchg.vert = VertMode::LvlChg;
        lvlchg.at = AtMode::N1;
        tick_at(m, lvlchg, secs);
        secs += DT;
        // The climb to the ATC stop.
        run(m, &mut secs, 400, lvlchg, 2300.0, 120.0);
        // The banner has fired; 2 ALT HOLD at once.
        let mut stop = frame(3510.0, SEAT_Z - 1_700.0);
        stop.vert = VertMode::AltHold;
        stop.at = AtMode::Speed;
        run(m, &mut secs, 3, stop, 0.0, 120.0);
        // Level, then the bug 330 turn.
        let mut level = frame(3560.0, SEAT_Z - 1_830.0);
        level.mcp_hdg = 330.0;
        run(m, &mut secs, 30, level, 0.0, 120.0);
        let mut turning = frame(3560.0, SEAT_Z - 2_130.0);
        turning.mcp_hdg = 330.0;
        turning.hdg_deg = 340.0;
        turning.roll_deg = -20.0;
        run(m, &mut secs, 40, turning, 0.0, 120.0);
        let mut rolled = frame(3560.0, SEAT_Z - 3_530.0);
        rolled.mcp_hdg = 330.0;
        rolled.hdg_deg = 330.0;
        run(m, &mut secs, 30, rolled, 0.0, 120.0);
        // V/S 2800 down, the window set before the press.
        let mut vs = frame(3560.0, SEAT_Z - 3_830.0);
        vs.vert = VertMode::Vs;
        vs.at = AtMode::Speed;
        vs.mcp_alt = 1000.0;
        vs.mcp_vs = -2800.0;
        tick_at(m, vs, secs);
        secs += DT;
        // The descent, APP armed with the arrival speed part-way.
        run(m, &mut secs, 150, vs, -2800.0, 105.0);
        let mut armed = frame(2860.0, SEAT_Z - 5_400.0);
        armed.vert = VertMode::Vs;
        armed.at = AtMode::Speed;
        armed.app_armed = true;
        armed.mcp_alt = 1000.0;
        armed.mcp_vs = -2800.0;
        armed.mcp_ias = 180.0;
        tick_at(m, armed, secs);
        secs += DT;
        run(m, &mut secs, 300, armed, -2800.0, 105.0);
        // LOC, then the G/S takes at ~1,400 ft on the beam.
        let mut loc = frame(1500.0, SEAT_Z - 10_000.0);
        loc.lat = LatMode::Loc;
        loc.vert = VertMode::Vs;
        loc.app_armed = true;
        loc.at = AtMode::Speed;
        loc.mcp_ias = 180.0;
        loc.hdg_deg = 335.0;
        tick_at(m, loc, secs);
        secs += DT;
        let mut gs = frame(1382.0, SEAT_Z - 11_000.0);
        gs.lat = LatMode::Loc;
        gs.vert = VertMode::Gs;
        gs.app_armed = false;
        gs.at = AtMode::Speed;
        gs.mcp_ias = 180.0;
        gs.hdg_deg = 333.0;
        gs.gs_dev_deg = 0.08;
        run(m, &mut secs, 110, gs, 0.0, 0.0);
    }

    #[test]
    fn a_textbook_arrival_tops_the_card() {
        let mut m = JetAutomationMeter::default();
        textbook(&mut m);
        let f = m.done.expect("the established hold froze the finals");
        assert!(m.settled, "the card owns the display at the hold");
        assert_eq!(f.window_pct, 1.0, "all three windows set before the mode: {f:?}");
        assert!(f.stop > 9.5, "the capture-now at once: {f:?}");
        assert!(f.hdg > 9.0, "the bank rode the limit and the bug was 330: {f:?}");
        assert!(f.vs > 9.0, "the window rode the band and the intercept was tight: {f:?}");
        assert!(f.app > 9.5, "LOC, G/S, A/T SPD at 180: {f:?}");
        assert!(f.final_ > 9.0, "every row near-perfect: {f:?}");
        assert_eq!(m.live, None, "a graded attempt hides the prompt");
        let text = m.finals_text().expect("the card");
        assert!(text.contains("737 AUTOMATION"), "{text}");
        assert!(text.contains("FINAL"), "{text}");
    }

    #[test]
    fn an_unset_window_pays_the_row() {
        let mut m = JetAutomationMeter::default();
        let mut secs = 0.0;
        let mut pre = frame(3000.0, SEAT_Z);
        pre.ap = false;
        tick_at(&mut m, pre, secs);
        secs += DT;
        tick_at(&mut m, frame(3000.0, SEAT_Z), secs);
        secs += DT;
        // The 4 press with the window still at its 10,000 default.
        let mut lvlchg = frame(3000.0, SEAT_Z);
        lvlchg.vert = VertMode::LvlChg;
        lvlchg.at = AtMode::N1;
        lvlchg.mcp_alt = 10_000.0;
        tick_at(&mut m, lvlchg, secs);
        secs += DT;
        // The climb to the stop, the capture-now, the rest by the book.
        run(&mut m, &mut secs, 400, lvlchg, 2300.0, 120.0);
        let mut stop = frame(3510.0, SEAT_Z - 1_700.0);
        stop.vert = VertMode::AltHold;
        stop.at = AtMode::Speed;
        run(&mut m, &mut secs, 3, stop, 0.0, 120.0);
        let mut level = frame(3560.0, SEAT_Z - 1_830.0);
        level.mcp_hdg = 330.0;
        run(&mut m, &mut secs, 30, level, 0.0, 120.0);
        let mut turning = frame(3560.0, SEAT_Z - 2_130.0);
        turning.mcp_hdg = 330.0;
        turning.hdg_deg = 340.0;
        turning.roll_deg = -20.0;
        run(&mut m, &mut secs, 40, turning, 0.0, 120.0);
        let mut rolled = frame(3560.0, SEAT_Z - 3_530.0);
        rolled.mcp_hdg = 330.0;
        rolled.hdg_deg = 330.0;
        run(&mut m, &mut secs, 30, rolled, 0.0, 120.0);
        let mut vs = frame(3560.0, SEAT_Z - 3_830.0);
        vs.vert = VertMode::Vs;
        vs.at = AtMode::Speed;
        vs.mcp_alt = 1000.0;
        vs.mcp_vs = -2800.0;
        tick_at(&mut m, vs, secs);
        secs += DT;
        run(&mut m, &mut secs, 150, vs, -2800.0, 105.0);
        let mut armed = frame(2860.0, SEAT_Z - 5_400.0);
        armed.vert = VertMode::Vs;
        armed.app_armed = true;
        armed.at = AtMode::Speed;
        armed.mcp_ias = 180.0;
        tick_at(&mut m, armed, secs);
        secs += DT;
        run(&mut m, &mut secs, 300, armed, -2800.0, 105.0);
        let mut loc = frame(1500.0, SEAT_Z - 10_000.0);
        loc.lat = LatMode::Loc;
        loc.vert = VertMode::Vs;
        loc.app_armed = true;
        loc.at = AtMode::Speed;
        loc.mcp_ias = 180.0;
        loc.hdg_deg = 335.0;
        tick_at(&mut m, loc, secs);
        secs += DT;
        let mut gs = frame(1382.0, SEAT_Z - 11_000.0);
        gs.lat = LatMode::Loc;
        gs.vert = VertMode::Gs;
        gs.at = AtMode::Speed;
        gs.hdg_deg = 333.0;
        gs.mcp_ias = 180.0;
        gs.gs_dev_deg = 0.08;
        run(&mut m, &mut secs, 110, gs, 0.0, 0.0);
        let f = m.done.expect("graded");
        assert!(
            (f.window - 10.0 * 2.0 / 3.0).abs() < 0.01,
            "two of three windows were set: {f:?}"
        );
        assert!(f.final_ > 6.5, "the rest was flown: {f:?}");
    }

    #[test]
    fn a_late_stop_pays_and_a_missed_one_fails() {
        // Late but inside the fence: the row pays, the run lives.
        let mut m = JetAutomationMeter::default();
        let mut secs = 0.0;
        tick_at(&mut m, frame(3000.0, SEAT_Z), secs);
        secs += DT;
        let mut lvlchg = frame(3000.0, SEAT_Z);
        lvlchg.vert = VertMode::LvlChg;
        lvlchg.at = AtMode::N1;
        tick_at(&mut m, lvlchg, secs);
        secs += DT;
        // The banner at 3,500, then 8 s of climb before the answer.
        run(&mut m, &mut secs, 400, lvlchg, 2300.0, 120.0);
        run(&mut m, &mut secs, 240, lvlchg, 2300.0, 120.0);
        let stop_alt = 3511.0 + 8.0 * 2300.0 / 60.0;
        let mut stop = frame(stop_alt, SEAT_Z - 3_100.0);
        stop.vert = VertMode::AltHold;
        stop.at = AtMode::Speed;
        run(&mut m, &mut secs, 3, stop, 0.0, 120.0);
        let f = m.done.is_none(); // the run continues; check via the stamp
        let _ = f;
        let react = m.stop_react.expect("the stop was answered");
        assert!((7.0..9.5).contains(&react), "{react}");
        assert!(m.fail.is_none(), "3,807 is inside the fence");
        // Never answered: the climb runs through the fence.
        let mut m = JetAutomationMeter::default();
        let mut secs = 0.0;
        tick_at(&mut m, frame(3000.0, SEAT_Z), secs);
        secs += DT;
        let mut lvlchg = frame(3000.0, SEAT_Z);
        lvlchg.vert = VertMode::LvlChg;
        lvlchg.at = AtMode::N1;
        tick_at(&mut m, lvlchg, secs);
        secs += DT;
        run(&mut m, &mut secs, 400, lvlchg, 2300.0, 120.0);
        // The second block climbs on from where the first left off.
        lvlchg.alt_ft = 3511.0;
        run(&mut m, &mut secs, 420, lvlchg, 2300.0, 120.0);
        assert_eq!(m.fail, Some(FAIL_CLIMB));
    }

    #[test]
    fn hand_flying_fails() {
        let mut m = JetAutomationMeter::default();
        let mut secs = 0.0;
        tick_at(&mut m, frame(3000.0, SEAT_Z), secs);
        secs += DT;
        tick_at(&mut m, frame(3000.0, SEAT_Z), secs);
        secs += DT;
        let mut dropped = frame(3000.0, SEAT_Z);
        dropped.ap = false;
        tick_at(&mut m, dropped, secs);
        assert_eq!(m.fail, Some(FAIL_AP_OFF));
    }

    #[test]
    fn never_engaging_fails() {
        let mut m = JetAutomationMeter::default();
        let mut secs = 0.0;
        let mut pre = frame(3000.0, SEAT_Z);
        pre.ap = false;
        // 46 s of no box.
        for _ in 0..(46.0 / DT) as usize {
            tick_at(&mut m, pre, secs);
            secs += DT;
        }
        assert_eq!(m.fail, Some(FAIL_NO_AP));
    }

    #[test]
    fn a_leg_that_never_flew_fails() {
        let mut m = JetAutomationMeter::default();
        let mut secs = 0.0;
        tick_at(&mut m, frame(3000.0, SEAT_Z), secs);
        secs += DT;
        let mut lvlchg = frame(3000.0, SEAT_Z);
        lvlchg.vert = VertMode::LvlChg;
        lvlchg.at = AtMode::N1;
        tick_at(&mut m, lvlchg, secs);
        secs += DT;
        // The climb mode is on but the jet never climbs past the stop.
        run(&mut m, &mut secs, (62.0 / DT) as usize, lvlchg, 0.0, 120.0);
        assert_eq!(m.fail, Some(FAIL_LEG));
        let detail = m.fail_detail.expect("the leg names itself");
        assert!(detail.contains("LVL CHG"), "{detail}");
    }

    #[test]
    fn flying_the_beams_off_fails() {
        // The descent runs but APP is never armed: the field arrives.
        let mut m = JetAutomationMeter::default();
        let mut secs = 0.0;
        tick_at(&mut m, frame(3000.0, SEAT_Z), secs);
        secs += DT;
        let mut lvlchg = frame(3000.0, SEAT_Z);
        lvlchg.vert = VertMode::LvlChg;
        lvlchg.at = AtMode::N1;
        tick_at(&mut m, lvlchg, secs);
        secs += DT;
        run(&mut m, &mut secs, 400, lvlchg, 2300.0, 120.0);
        let mut stop = frame(3510.0, SEAT_Z - 1_700.0);
        stop.vert = VertMode::AltHold;
        stop.at = AtMode::Speed;
        run(&mut m, &mut secs, 3, stop, 0.0, 120.0);
        let mut level = frame(3560.0, SEAT_Z - 1_830.0);
        level.mcp_hdg = 330.0;
        run(&mut m, &mut secs, 30, level, 0.0, 120.0);
        let mut turning = frame(3560.0, SEAT_Z - 2_130.0);
        turning.mcp_hdg = 330.0;
        turning.hdg_deg = 340.0;
        turning.roll_deg = -20.0;
        run(&mut m, &mut secs, 40, turning, 0.0, 120.0);
        let mut rolled = frame(3560.0, SEAT_Z - 3_530.0);
        rolled.mcp_hdg = 330.0;
        rolled.hdg_deg = 330.0;
        run(&mut m, &mut secs, 30, rolled, 0.0, 120.0);
        let mut vs = frame(3560.0, SEAT_Z - 3_830.0);
        vs.vert = VertMode::Vs;
        vs.at = AtMode::Speed;
        vs.mcp_alt = 1000.0;
        vs.mcp_vs = -2800.0;
        tick_at(&mut m, vs, secs);
        secs += DT;
        // Down and in, never armed, never captured.
        vs.pos.z = 3_100.0;
        run(&mut m, &mut secs, 10, vs, -2800.0, 0.0);
        run(&mut m, &mut secs, 1_900, vs, -2800.0, 105.0);
        assert_eq!(m.fail, Some(FAIL_ILS));
    }

    #[test]
    fn the_prompt_ladder_plays_the_legs() {
        let mut m = JetAutomationMeter::default();
        let mut secs = 0.0;
        // The seat: the engage brief.
        let mut pre = frame(3000.0, SEAT_Z);
        pre.ap = false;
        tick_at(&mut m, pre, secs);
        secs += DT;
        let live = m.live.clone().expect("the engage brief");
        assert!(live.contains("ENGAGE"), "{live}");
        // Engaged: the window brief.
        tick_at(&mut m, frame(3000.0, SEAT_Z), secs);
        secs += DT;
        let live = m.live.clone().expect("the window brief");
        assert!(live.contains("SET MCP ALT 4000"), "{live}");
        // The climb mode: the climb line.
        let mut lvlchg = frame(3000.0, SEAT_Z);
        lvlchg.vert = VertMode::LvlChg;
        lvlchg.at = AtMode::N1;
        tick_at(&mut m, lvlchg, secs);
        secs += DT;
        let live = m.live.clone().expect("the climb line");
        assert!(live.contains("LVL CHG - CLIMB"), "{live}");
        // The ATC call rides the banner.
        run(&mut m, &mut secs, 400, lvlchg, 2300.0, 120.0);
        let live = m.live.clone().expect("the ATC call");
        assert!(live.contains("STOP THE CLIMB"), "{live}");
        // Answered: the HDG SEL is already live - set the bug. The ATC
        // banner rides its 2.5 s out before the standing brief shows.
        let mut stop = frame(3510.0, SEAT_Z - 1_700.0);
        stop.vert = VertMode::AltHold;
        stop.at = AtMode::Speed;
        run(&mut m, &mut secs, 3, stop, 0.0, 120.0);
        let mut level = frame(3560.0, SEAT_Z - 1_830.0);
        level.mcp_hdg = 330.0;
        run(&mut m, &mut secs, 85, level, 0.0, 120.0);
        let live = m.live.clone().expect("the bug brief");
        assert!(live.contains("HDG SEL IS LIVE"), "{live}");
        // The turn: the V/S brief with the distance.
        let mut turning = frame(3560.0, SEAT_Z - 2_130.0);
        turning.mcp_hdg = 330.0;
        turning.hdg_deg = 340.0;
        turning.roll_deg = -20.0;
        run(&mut m, &mut secs, 30, turning, 0.0, 120.0);
        let live = m.live.clone().expect("the intercept brief");
        assert!(live.contains("INTERCEPT 330"), "{live}");
        // The descent: the APP brief.
        let mut vs = frame(3560.0, SEAT_Z - 3_830.0);
        vs.vert = VertMode::Vs;
        vs.at = AtMode::Speed;
        vs.mcp_alt = 1000.0;
        vs.mcp_vs = -2800.0;
        tick_at(&mut m, vs, secs);
        secs += DT;
        let live = m.live.clone().expect("the descent brief");
        assert!(live.contains("V/S - DESCENDING"), "{live}");
        // Armed: the LOC brief.
        let mut armed = frame(2860.0, SEAT_Z - 5_400.0);
        armed.vert = VertMode::Vs;
        armed.app_armed = true;
        armed.at = AtMode::Speed;
        armed.mcp_ias = 180.0;
        tick_at(&mut m, armed, secs);
        secs += DT;
        let live = m.live.clone().expect("the armed brief");
        assert!(live.contains("APP ARMED"), "{live}");
        // LOC: the G/S brief.
        let mut loc = frame(1500.0, SEAT_Z - 10_000.0);
        loc.lat = LatMode::Loc;
        loc.vert = VertMode::Vs;
        loc.app_armed = true;
        loc.at = AtMode::Speed;
        loc.mcp_ias = 180.0;
        tick_at(&mut m, loc, secs);
        secs += DT;
        let live = m.live.clone().expect("the loc brief");
        assert!(live.contains("LOC CAPTURED"), "{live}");
        // Established: the hold line.
        let mut gs = frame(1382.0, SEAT_Z - 11_000.0);
        gs.lat = LatMode::Loc;
        gs.vert = VertMode::Gs;
        gs.at = AtMode::Speed;
        gs.mcp_ias = 180.0;
        gs.gs_dev_deg = 0.08;
        tick_at(&mut m, gs, secs);
        let live = m.live.clone().expect("the hold line");
        assert!(live.contains("ESTABLISHED"), "{live}");
    }

    #[test]
    fn a_rearmed_attempt_starts_over() {
        let mut m = JetAutomationMeter::default();
        textbook(&mut m);
        assert!(m.done.is_some());
        // The card has no Continue; a fresh flight hands a fresh attempt.
        m.rearm();
        let mut pre = frame(3000.0, SEAT_Z);
        pre.ap = false;
        tick_at(&mut m, pre, 0.0);
        assert_eq!(m.done, None);
        assert_eq!(m.fail, None, "a fresh attempt above the fence");
    }
}
