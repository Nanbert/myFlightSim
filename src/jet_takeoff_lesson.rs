//! The 737's graded TAKEOFF lesson (M2): the airline departure flow, flown
//! to the handoff — not just off the ground.
//!
//! The C172's M2 freezes at a 200 ft gate; the jet's takeoff isn't over when
//! the wheels leave. The lesson runs the PM's callout flow off the lined-up
//! spawn on ZSNM's numbers — `80 KT`, `V1`, `ROTATE`, `POSITIVE RATE`, the
//! acceleration-altitude clean-up — and **freezes the card at the handoff**:
//! A/P engaged, flaps up, gear up, held two quiet seconds (a stick tap drops
//! the A/P, so the window proves the aeroplane is really theirs). Whether
//! the automation *keeps climbing* is a graded row, not the gate.
//!
//! The contract has a hard edge: **clean up and hand off by 5,000 ft**. The
//! rows:
//!
//! - **ROTATION** — liftoff speed against the book VR (`pfd::vr_for_flaps`,
//!   flaps 5 ≈ 179 kt) and the attitude against the 10° centre, a ground
//!   pitch past 17° being a tailstrike FAIL.
//! - **RLW** — the runway-centreline average over the roll.
//! - **CLEAN-UP** — gear up within 5 s of positive rate, flaps up at the
//!   1,500 ft acceleration altitude.
//! - **CLIMB** — the speed band: V2+10..V2+30 dirty, then the 250 kt-class
//!   climb once clean.
//! - **AUTOMATION** — the handoff itself: engaged in the 1,000–3,000 ft
//!   sweet band, a vertical mode that keeps climbing (LVL CHG), the heading
//!   bug on the departure heading, the A/T on.
//!
//! Failures (feeding the lesson-flow FAIL popup): tailstrike, settling back
//! onto the runway, banking past 30° before the handoff, the climb eating
//! its speed — and the missed handoff over 5,000 ft.

use bevy::prelude::*;

use crate::app_state::{AppState, unpaused};
use crate::autopilot::{AtMode, VertMode};
use crate::mission::{ActiveLesson, Lesson};
use crate::pfd::vr_for_flaps;
use crate::scoring::{score, word};
use crate::sim::AircraftState;

/// The 80-kt crosscheck: the callout that closes the thrust-set flow.
const AIRSPEED_CHECK: f64 = 80.0;
/// How long a crossing callout holds the banner (ROTATE rides shorter).
const CALLOUT_SECS: f32 = 2.5;
/// Rolling below this speed is standing still, not a roll worth sampling.
const ROLLING_GS: f64 = 3.0;
/// The liftoff edge needs the roll established (a wow flip slower than this
/// is the spawn settling on its gear).
const LIFT_MIN_GS: f64 = 100.0;
/// The rotation-attitude centre, the same 10° the free-flight takeoff card
/// grades the jet against.
const PITCH_CENTRE: f64 = 10.0;
/// A ground pitch past this puts the tail in the pavement.
const TAILSTRIKE_DEG: f64 = 17.0;
/// Banking past this before the handoff is a flat FAIL.
const STEEP_BANK: f64 = 30.0;
/// The climb's speed dying below this is a flat FAIL (V2 at flaps 5 ≈ 184,
/// so this is the last warning before the stall).
const SLOW_CAS: f64 = 150.0;
/// The acceleration altitude: flaps come up from here.
const ACCEL_ALT: f64 = 1500.0;
/// The handoff contract: clean and on the A/P by this AGL, or FAIL.
const HANDOFF_CEILING: f64 = 5000.0;
/// The A/P engagement sweet band.
const ENGAGE_LO: f64 = 1000.0;
const ENGAGE_HI: f64 = 3000.0;
/// Below this an engagement is a low-level grab, scored to zero.
const ENGAGE_FLOOR: f64 = 300.0;
/// The quiet seconds that prove the handoff: A/P on and clean, untouched.
const HANDOFF_HOLD: f32 = 2.0;
/// Gear up within this many seconds of liftoff is full marks (the
/// free-flight card's anchor).
const GEAR_IDEAL_S: f64 = 5.0;
const GEAR_CAP_S: f64 = 15.0;
/// Flaps up at or above this altitude is full marks; at half the
/// acceleration altitude the row is gone.
const FLAP_UP_FULL: f64 = 1200.0;
const FLAP_UP_ZERO: f64 = 600.0;
/// The clean climb's band once the flaps are up (the 250 kt class).
const CLEAN_LO: f64 = 180.0;
const CLEAN_HI: f64 = 260.0;

// --- row weights (they sum to 1) ------------------------------------------------

const W_ROTATION: f32 = 0.30;
const W_RLW: f32 = 0.15;
const W_CLEANUP: f32 = 0.20;
const W_CLIMB: f32 = 0.15;
const W_AUTOMATION: f32 = 0.20;

const FAIL_TAIL: &str = "FAIL - TAILSTRIKE\nThe rotation pitched the tail into the runway.";
const FAIL_SANK: &str = "FAIL - SANK BACK\nThe climb out settled back onto the runway.";
const FAIL_STEEP: &str = "FAIL - STEEP BANK\nBanked past 30 degrees before the handoff.";
const FAIL_SLOW: &str = "FAIL - SLOW CLIMB\nThe climb-out ate its speed toward the stall.";
const FAIL_CEILING: &str = "FAIL - HANDOFF MISSED\nClean up and engage the A/P before 5,000 ft.";

#[derive(Clone, Copy, PartialEq, Default)]
enum Phase {
    /// On the runway: lined up on the numbers, or rolling.
    #[default]
    Roll,
    /// Wheels up: the callout flow, the clean-up, climbing toward the handoff.
    Climb,
    /// A/P on and clean: holding the quiet window that freezes the card.
    Handoff,
    /// The card (or a fail) froze the attempt.
    Graded,
}

/// The liftoff frame's graded readouts.
#[derive(Clone, Copy)]
struct LiftFrame {
    cas: f64,
    pitch: f64,
    /// The book VR at the flap setting the rotation rode (flaps 5 ≈ 179).
    vr: f64,
}

/// One frame's readouts the tick needs, extracted from the live resources:
/// pure and testable, like the C172 lesson's `Telem`.
#[derive(Clone, Copy)]
pub struct Telem {
    on_ground: bool,
    gs_kts: f64,
    cas_kts: f64,
    alt_ft: f64,
    vs_fpm: f64,
    pitch_deg: f64,
    roll_deg: f64,
    on_runway: bool,
    /// World position — the roll samples the runway centreline off it.
    pos: Vec3,
    throttle: f64,
    flap_deg: f64,
    /// Achieved gear position: 1 down, 0 up.
    gear_pos: f64,
    ap: bool,
    vert: VertMode,
    at: AtMode,
    mcp_hdg: f64,
    mcp_alt: f64,
}

#[derive(Resource, Default)]
pub struct JetTakeoffMeter {
    phase: Phase,
    /// The live prompt text, composed every ticking frame; `None` hides the
    /// centred card.
    pub(crate) live: Option<String>,
    /// The |offset| average over the roll: sum and samples.
    rlw_sum: f32,
    rlw_n: u32,
    /// The liftoff frame readouts; `None` while the wheels still carry.
    lift: Option<LiftFrame>,
    /// Seconds of climb and of them spent outside the speed band.
    climb_secs: f32,
    out_secs: f32,
    /// Liftoff-to-gear-up seconds, and the altitude the flaps reached full up.
    gear_up_s: Option<f64>,
    flap_up_alt: Option<f64>,
    /// The quiet handoff window, and the altitude the A/P was engaged at.
    hold_for: f32,
    engage_alt: Option<f64>,
    /// A transient banner (a callout crossing) and when it retires.
    callout: Option<(String, f32)>,
    /// Ground speed and the positive-rate latch from the previous tick.
    last_gs: f64,
    positive_rate: bool,
    /// The fail that parked the attempt, for the lesson-flow popup.
    pub(crate) fail: Option<&'static str>,
    /// The frozen finals of the attempt (at the handoff).
    pub(crate) done: Option<JetFinals>,
    /// A graded attempt holds its card until Enter re-arms.
    pub(crate) settled: bool,
    was_ground: bool,
}

impl JetTakeoffMeter {
    /// A fresh attempt (the CONTINUE past the card, or a new flight). An
    /// airborne arm stays quiet — the liftoff edge needs a ground frame.
    pub(crate) fn rearm(&mut self) {
        *self = JetTakeoffMeter::default();
    }

    /// The finals card's rows: every graded row prints its number and word
    /// band, the weighted final on top.
    pub fn finals_text(&self) -> Option<String> {
        let f = self.done?;
        let rotation_note = if f.tailstrike { "  TAILSTRIKE" } else { "" };
        let gear = f
            .gear_up_s
            .map(|s| format!("{s:.0} s"))
            .unwrap_or_else(|| "STILL DOWN".into());
        let flaps = f
            .flap_up_alt
            .map(|a| format!("{a:.0} ft"))
            .unwrap_or_else(|| "OUT".into());
        Some(format!(
            "737 TAKEOFF\n\
             ROTATION {cas:3.0} kt / VR {vr:3.0}  {r:4.1} {rw}{note}\n\
             RLW      Δ avg {ra:+5.1} m          {rl:4.1} {rlw}\n\
             CLEAN-UP gear {gear:>6}  flaps {flaps:>8}  {c:4.1} {cw}\n\
             CLIMB    {pct:3.0}% in band          {cl:4.1} {clw}\n\
             AUTO     A/P @ {alt:4.0} ft          {a:4.1} {aw}\n\
             FINAL    {f_:4.1}  {fw}",
            cas = f.lift_cas,
            vr = f.vr,
            r = f.rotation,
            rw = word(f.rotation),
            note = rotation_note,
            ra = f.rlw_avg,
            rl = f.rlw,
            rlw = word(f.rlw),
            gear = gear,
            flaps = flaps,
            c = f.cleanup,
            cw = word(f.cleanup),
            pct = f.band_pct * 100.0,
            cl = f.climb,
            clw = word(f.climb),
            alt = f.engage_alt,
            a = f.automation,
            aw = word(f.automation),
            f_ = f.final_,
            fw = word(f.final_),
        ))
    }
}

/// The frozen finals: each row a 0–10 score, the final their weighted
/// average (bad rows pull it down, nothing zeroes it — the flat FAILs are
/// popups, outside the scorer).
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct JetFinals {
    lift_cas: f64,
    vr: f64,
    tailstrike: bool,
    rotation: f32,
    rlw_avg: f32,
    rlw: f32,
    gear_up_s: Option<f64>,
    flap_up_alt: Option<f64>,
    cleanup: f32,
    band_pct: f32,
    climb: f32,
    engage_alt: f64,
    automation: f32,
    final_: f32,
}

/// Feed one frame's readouts. Pure over its own state, for tests.
fn tick(m: &mut JetTakeoffMeter, t: Telem, now: f32, dt: f32) {
    // A retired banner drops before the phase logic reads it.
    if m.callout.as_ref().is_some_and(|(_, until)| now >= *until) {
        m.callout = None;
    }
    let vr_now = vr_for_flaps(t.flap_deg / 40.0);
    match m.phase {
        Phase::Roll => {
            // The tailstrike: on the pavement, past 17 degrees of pitch.
            if t.on_ground && t.pitch_deg > TAILSTRIKE_DEG {
                m.fail = Some(FAIL_TAIL);
                m.phase = Phase::Graded;
                m.live = None;
                return;
            }
            // The 80-kt crosscheck closes the thrust-set flow.
            if t.on_ground && m.last_gs < AIRSPEED_CHECK && t.gs_kts >= AIRSPEED_CHECK {
                let text = if t.throttle > 0.95 {
                    "80 KT - THRUST SET".to_owned()
                } else {
                    "80 KT - CHECK THRUST".to_owned()
                };
                m.callout = Some((text, now + CALLOUT_SECS));
            }
            // V1 rides just under VR (the decision is not modelled; the
            // callout keeps the rhythm).
            if t.on_ground && m.last_gs < vr_now - 5.0 && t.gs_kts >= vr_now - 5.0 {
                let v1 = vr_now - 5.0;
                m.callout = Some((format!("V1 {v1:.0}"), now + CALLOUT_SECS));
            }
            // The centreline sampler: once the aircraft actually rolls.
            if t.on_ground && t.on_runway && t.gs_kts > ROLLING_GS {
                m.rlw_sum += t.pos.x.abs();
                m.rlw_n += 1;
            }
            // The liftoff edge: wheels up with the roll established.
            if !t.on_ground && m.was_ground && t.gs_kts > LIFT_MIN_GS {
                m.lift = Some(LiftFrame {
                    cas: t.cas_kts,
                    pitch: t.pitch_deg,
                    vr: vr_for_flaps(t.flap_deg / 40.0),
                });
                m.phase = Phase::Climb;
                // The rotation rides up with the aircraft for a beat.
                m.callout = Some(("ROTATE".to_owned(), now + 1.2));
            }
        }
        Phase::Climb | Phase::Handoff => {
            let grading = m.fail.is_none();
            // Fails first: nothing past them can save the attempt.
            if grading && t.on_ground {
                m.fail = Some(FAIL_SANK);
            } else if grading && t.roll_deg.abs() > STEEP_BANK {
                m.fail = Some(FAIL_STEEP);
            } else if grading && t.cas_kts < SLOW_CAS {
                m.fail = Some(FAIL_SLOW);
            } else if grading && m.phase == Phase::Climb && t.alt_ft > HANDOFF_CEILING {
                m.fail = Some(FAIL_CEILING);
            }
            if m.fail.is_some() {
                m.phase = Phase::Graded;
                m.live = None;
                return;
            }
            // The clean-up trackers.
            if !m.positive_rate && t.vs_fpm > 100.0 {
                m.positive_rate = true;
                m.callout = Some(("POSITIVE RATE".to_owned(), now + CALLOUT_SECS));
            }
            if m.gear_up_s.is_none() && !t.on_ground && t.gear_pos < 0.05 {
                m.gear_up_s = Some(climb_secs(m));
            }
            if m.flap_up_alt.is_none() && !t.on_ground && t.flap_deg < 1.0 {
                m.flap_up_alt = Some(t.alt_ft);
            }
            // The speed band: V2+10..V2+30 dirty, the 250-class climb clean.
            let vr = m.lift.expect("climb implies a liftoff").vr;
            let (lo, hi) = if t.flap_deg >= 1.0 {
                (vr + 10.0, vr + 30.0)
            } else {
                (CLEAN_LO, CLEAN_HI)
            };
            m.climb_secs += dt;
            if t.cas_kts < lo || t.cas_kts > hi {
                m.out_secs += dt;
            }
            // The handoff gate: A/P on and clean. Climbing afterwards is a
            // graded row (AUTOMATION), not part of the gate.
            let clean = t.flap_deg < 1.0 && t.gear_pos < 0.05;
            match m.phase {
                Phase::Climb => {
                    if t.ap && clean {
                        m.phase = Phase::Handoff;
                        m.hold_for = 0.0;
                        // The engagement altitude is a fact of the entry
                        // frame; the window itself climbs.
                        m.engage_alt = Some(t.alt_ft);
                    }
                }
                Phase::Handoff => {
                    // A stick tap drops the A/P; the window starts over.
                    if !t.ap || !clean {
                        m.phase = Phase::Climb;
                        m.hold_for = 0.0;
                    } else {
                        m.hold_for += dt;
                        if m.hold_for >= HANDOFF_HOLD {
                            m.done = Some(finals(m, &t));
                            m.settled = true;
                            m.phase = Phase::Graded;
                            m.live = None;
                            return;
                        }
                    }
                }
                _ => unreachable!("the outer match pinned the pair"),
            }
        }
        Phase::Graded => {
            m.live = None;
            return;
        }
    }
    m.last_gs = t.gs_kts;
    m.was_ground = t.on_ground;
    m.live = compose_live(m, &t, now, vr_now);
}

/// Seconds of climb logged so far — the gear clock's zero.
fn climb_secs(m: &JetTakeoffMeter) -> f64 {
    m.climb_secs as f64
}

/// The banner-first live prompt: a callout while it runs, otherwise the
/// phase's PM flow. `None` while a card owns the display.
fn compose_live(m: &JetTakeoffMeter, t: &Telem, now: f32, vr_now: f64) -> Option<String> {
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
            if t.on_ground && t.gs_kts >= vr_now {
                Some(format!("ROTATE {vr_now:.0}"))
            } else if t.on_ground && t.gs_kts > 1.0 {
                Some(format!(
                    "T.O. ROLL\nRLW Δ {:+.1} m    {:.0} KT",
                    t.pos.x, t.gs_kts
                ))
            } else if t.on_ground {
                Some("RWY 36 - CLEARED FOR TAKEOFF\nFLAPS 5, THROTTLE FULL".to_owned())
            } else {
                None
            }
        }
        Phase::Climb => {
            let vr = m.lift.expect("climb implies a liftoff").vr;
            let pace = format!("{:.0} kt", t.cas_kts);
            if t.gear_pos >= 0.05 {
                Some(format!(
                    "POSITIVE RATE - GEAR UP (U)\n{pace}    {:.0} ft",
                    t.alt_ft
                ))
            } else if t.flap_deg >= 1.0 && t.alt_ft < ACCEL_ALT {
                Some(format!(
                    "CLIMB\n{pace} / band {:.0}-{:.0}    {:.0} ft",
                    vr + 10.0,
                    vr + 30.0,
                    t.alt_ft
                ))
            } else if t.flap_deg >= 1.0 {
                Some(format!("ACCEL {:.0} FT - FLAPS UP (V)\n{pace}", t.alt_ft))
            } else {
                Some(format!(
                    "CLEAN - A/P ON (G), LVL CHG (4)\n{:.0} ft    sweet 1000-3000",
                    t.alt_ft
                ))
            }
        }
        Phase::Handoff => Some("HOLDING THE HANDOFF\nA/P ON, HANDS OFF".to_owned()),
        Phase::Graded => None,
    }
}

/// The attempt frozen into finals at the handoff.
fn finals(m: &JetTakeoffMeter, t: &Telem) -> JetFinals {
    let lift = m.lift.expect("the handoff implies a liftoff");
    let rlw_avg = if m.rlw_n > 0 {
        m.rlw_sum / m.rlw_n as f32
    } else {
        0.0
    };
    // ROTATION: speed against VR and attitude against the 10° centre; a
    // liftoff pitched into tailstrike territory caps the row whatever the
    // speed said.
    let tailstrike = lift.pitch > TAILSTRIKE_DEG;
    let speed = score((lift.cas - lift.vr).abs() as f32, 5.0, 25.0);
    let pitch = score((lift.pitch - PITCH_CENTRE).abs() as f32, 2.0, 8.0);
    let rotation = if tailstrike {
        rotation_score(speed, pitch).min(2.0)
    } else {
        rotation_score(speed, pitch)
    };
    let rlw = score(rlw_avg, 1.5, 8.0);
    // CLEAN-UP: the gear clock and the flaps-up altitude, half and half.
    let gear_term = m
        .gear_up_s
        .map(|s| {
            let over = ((s - GEAR_IDEAL_S).max(0.0) / (GEAR_CAP_S - GEAR_IDEAL_S)) as f32;
            10.0 * (1.0 - over)
        })
        .unwrap_or(0.0);
    let flap_term = m
        .flap_up_alt
        .map(|a| {
            score(
                (ACCEL_ALT - a).max(0.0) as f32,
                (ACCEL_ALT - FLAP_UP_FULL) as f32,
                (ACCEL_ALT - FLAP_UP_ZERO) as f32,
            )
        })
        .unwrap_or(0.0);
    let cleanup = 0.5 * gear_term + 0.5 * flap_term;
    // CLIMB: the fraction of climb time inside the band.
    let band_pct = if m.climb_secs > 0.0 {
        1.0 - m.out_secs / m.climb_secs
    } else {
        1.0
    };
    let climb = (band_pct * 10.0).clamp(0.0, 10.0);
    // AUTOMATION: the engagement altitude's sweet band, a vertical mode that
    // keeps climbing, the heading bug on the departure heading, the A/T.
    let engage_alt = m.engage_alt.unwrap_or(t.alt_ft);
    let alt_term = if (ENGAGE_LO..=ENGAGE_HI).contains(&engage_alt) {
        10.0
    } else if engage_alt < ENGAGE_LO {
        score(
            (ENGAGE_LO - engage_alt) as f32,
            0.0,
            (ENGAGE_LO - ENGAGE_FLOOR) as f32,
        )
    } else {
        score(
            (engage_alt - ENGAGE_HI) as f32,
            0.0,
            (HANDOFF_CEILING - ENGAGE_HI) as f32,
        )
    };
    let vert_term = match t.vert {
        // LVL CHG toward a window above: the airline climb handoff. Toward
        // one below it descends — that is a level-off in disguise.
        VertMode::LvlChg if t.mcp_alt > t.alt_ft + 200.0 => 10.0,
        VertMode::LvlChg | VertMode::AltHold => 4.0,
        VertMode::Vs => 8.0,
        _ => 0.0,
    };
    let hdg_err = heading_delta(t.mcp_hdg, 360.0);
    let hdg_term = score(hdg_err.abs(), 10.0, 30.0);
    let at_term = if t.at != AtMode::Off { 10.0 } else { 0.0 };
    let automation = 0.35 * alt_term + 0.35 * vert_term + 0.15 * hdg_term + 0.15 * at_term;
    let final_ = W_ROTATION * rotation
        + W_RLW * rlw
        + W_CLEANUP * cleanup
        + W_CLIMB * climb
        + W_AUTOMATION * automation;
    JetFinals {
        lift_cas: lift.cas,
        vr: lift.vr,
        tailstrike,
        rotation,
        rlw_avg,
        rlw,
        gear_up_s: m.gear_up_s,
        flap_up_alt: m.flap_up_alt,
        cleanup,
        band_pct,
        climb,
        engage_alt,
        automation,
        final_,
    }
}

/// The rotation row: the speed carries it, the attitude keeps it honest.
fn rotation_score(speed: f32, pitch: f32) -> f32 {
    0.6 * speed + 0.4 * pitch
}

/// Signed heading difference wrapped to ±180°, degrees.
fn heading_delta(hdg: f64, reference: f64) -> f32 {
    ((hdg - reference + 180.0).rem_euclid(360.0) - 180.0) as f32
}

/// The update system: only the jet takeoff lesson, only the home field.
fn step_m2(
    aircraft: Res<AircraftState>,
    ap: Res<crate::autopilot::Autopilot>,
    mcp: Res<crate::autopilot::Mcp>,
    airfield: Res<crate::airfield::Airfield>,
    map: Res<crate::map::ActiveMap>,
    lesson: Res<ActiveLesson>,
    time: Res<Time>,
    mut m: ResMut<JetTakeoffMeter>,
) {
    if map.0.is_some() || lesson.0 != Some(Lesson::JetTakeoff) {
        return;
    }
    let default_spec = crate::airfield::AirfieldSpec::zsnm();
    let spec = airfield.0.as_ref().unwrap_or(&default_spec);
    tick(
        &mut m,
        Telem {
            on_ground: aircraft.on_ground,
            gs_kts: aircraft.gs_kts,
            cas_kts: aircraft.cas_kts,
            alt_ft: aircraft.alt_ft,
            vs_fpm: aircraft.vs_fpm,
            pitch_deg: aircraft.pitch_deg,
            roll_deg: aircraft.roll_deg,
            on_runway: spec.on_runway(aircraft.pos.x, aircraft.pos.z),
            pos: aircraft.pos,
            throttle: aircraft.throttle,
            flap_deg: aircraft.flap_deg,
            gear_pos: aircraft.gear_pos,
            ap: ap.engaged,
            vert: ap.vert,
            at: ap.at,
            mcp_hdg: mcp.hdg_deg,
            mcp_alt: mcp.alt_ft,
        },
        time.elapsed_secs(),
        time.delta_secs(),
    );
}

pub struct JetTakeoffLessonPlugin;

impl Plugin for JetTakeoffLessonPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<JetTakeoffMeter>()
            .add_systems(OnEnter(AppState::Flying), reset_jet_takeoff)
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
fn reset_jet_takeoff(mut m: ResMut<JetTakeoffMeter>) {
    m.rearm();
}

// --- tests -------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use bevy::prelude::*;

    use super::{
        ACCEL_ALT, FAIL_CEILING, FAIL_SANK, FAIL_SLOW, FAIL_STEEP, FAIL_TAIL, JetTakeoffMeter,
        Telem, tick,
    };
    use crate::autopilot::{AtMode, VertMode};

    const DT: f32 = 1.0 / 30.0;
    const NOW: f32 = 100.0;
    /// VR at flaps 5 (185 - 50 * 5/40), the book the tests rotate against.
    const VR5: f64 = 178.75;

    /// A frame with the book jet values; mutate the returned struct for the
    /// off-book cases.
    fn frame(gs: f64, alt: f64, on_ground: bool) -> Telem {
        Telem {
            on_ground,
            gs_kts: gs,
            cas_kts: gs,
            alt_ft: alt,
            vs_fpm: if on_ground { 0.0 } else { 1800.0 },
            pitch_deg: if on_ground { 0.0 } else { 10.0 },
            roll_deg: 0.0,
            on_runway: true,
            pos: Vec3::new(0.3, 0.0, 90.0),
            throttle: 1.0,
            flap_deg: 5.0,
            gear_pos: if on_ground { 1.0 } else { 0.0 },
            ap: false,
            vert: VertMode::Off,
            at: AtMode::Off,
            mcp_hdg: 0.0,
            mcp_alt: 8000.0,
        }
    }

    fn tick_at(m: &mut JetTakeoffMeter, t: Telem, secs: f32) {
        tick(m, t, NOW + secs, DT);
    }

    /// The book roll: parked, rolling through the callouts, off at ~VR+2.
    fn book_roll(m: &mut JetTakeoffMeter) {
        tick_at(m, frame(0.0, 0.0, true), 0.0);
        tick_at(m, frame(60.0, 0.0, true), 1.0);
        tick_at(m, frame(81.0, 0.0, true), 2.0);
        tick_at(m, frame(150.0, 0.0, true), 3.0);
        // Standing ROTATE past VR, then the liftoff edge.
        tick_at(m, frame(180.0, 0.0, true), 4.0);
        let mut lift = frame(185.0, 30.0, false);
        lift.cas_kts = 180.0;
        tick_at(m, lift, 5.0);
    }

    /// The book climb to a handoff at ~2,000 ft: gear up within 5 s, flaps up
    /// through the acceleration altitude, then A/P on with LVL CHG + A/T.
    fn book_climb(m: &mut JetTakeoffMeter, start: f32) {
        // Dirty climb in the band, gear still travelling for the first beat.
        let mut gear_down = frame(200.0, 200.0, false);
        gear_down.gear_pos = 0.5;
        gear_down.cas_kts = 195.0;
        tick_at(m, gear_down, start);
        // Gear up 4 s after liftoff; flaps still 5 through 1,400 ft.
        let mut dirty = frame(200.0, 600.0, false);
        dirty.cas_kts = 200.0;
        tick_at(m, dirty, start + 1.0);
        dirty.alt_ft = 1000.0;
        tick_at(m, dirty, start + 2.0);
        dirty.alt_ft = 1400.0;
        tick_at(m, dirty, start + 3.0);
        // Through the acceleration altitude: flaps come up (flap_up_alt ~1600).
        let mut cleaning = frame(210.0, 1600.0, false);
        cleaning.cas_kts = 205.0;
        cleaning.flap_deg = 2.0;
        tick_at(m, cleaning, start + 4.0);
        let mut clean = frame(230.0, 1900.0, false);
        clean.cas_kts = 230.0;
        clean.flap_deg = 0.0;
        tick_at(m, clean, start + 5.0);
        // The handoff at ~2,000 ft: A/P on, LVL CHG with the A/T.
        let mut handoff = frame(230.0, 2000.0, false);
        handoff.cas_kts = 230.0;
        handoff.flap_deg = 0.0;
        handoff.ap = true;
        handoff.vert = VertMode::LvlChg;
        handoff.at = AtMode::N1;
        // The quiet window: 2+ seconds of the same.
        let mut secs = start + 6.0;
        while secs < start + 9.0 {
            tick_at(m, handoff, secs);
            secs += DT;
        }
    }

    #[test]
    fn a_book_departure_tops_the_card() {
        let mut m = JetTakeoffMeter::default();
        book_roll(&mut m);
        book_climb(&mut m, 6.0);
        let f = m.done.expect("the handoff froze the finals");
        assert!(m.settled, "the card owns the display at the handoff");
        assert_eq!(f.vr, VR5, "the book VR at flaps 5");
        assert!(f.rotation > 9.5, "off at VR+1 pitched 10: {f:?}");
        assert_eq!(f.rlw, 10.0, "the 0.3 m line");
        assert!(f.cleanup > 9.0, "gear at 4 s, flaps at 1,600 ft: {f:?}");
        assert!(f.climb > 9.0, "always in the band: {f:?}");
        assert!(f.automation > 9.5, "A/P at 2,000 ft LVL CHG A/T: {f:?}");
        assert!(f.final_ > 9.0, "every row near-perfect: {f:?}");
        assert_eq!(m.live, None, "a graded attempt hides the prompt");
        let text = m.finals_text().expect("the card");
        assert!(text.contains("TIGHT"), "{text}");
        assert!(!text.contains("TAILSTRIKE"), "{text}");
    }

    #[test]
    fn an_overrotation_caps_the_row_and_names_itself() {
        let mut m = JetTakeoffMeter::default();
        tick_at(&mut m, frame(150.0, 0.0, true), 0.0);
        let mut lift = frame(180.0, 30.0, false);
        lift.cas_kts = 180.0;
        lift.pitch_deg = 18.0; // into the tailstrike zone off the pavement
        tick_at(&mut m, lift, 1.0);
        let mut climb = frame(200.0, 400.0, false);
        climb.cas_kts = 200.0;
        climb.pitch_deg = 14.0;
        tick_at(&mut m, climb, 2.0);
        let mut clean = frame(230.0, 1600.0, false);
        clean.cas_kts = 230.0;
        clean.flap_deg = 0.0;
        clean.ap = true;
        clean.vert = VertMode::LvlChg;
        clean.at = AtMode::N1;
        let mut secs = 3.0;
        while m.done.is_none() && secs < 10.0 {
            tick_at(&mut m, clean, secs);
            secs += DT;
        }
        let f = m.done.expect("graded");
        assert!(f.tailstrike, "18 degrees at the liftoff frame");
        assert!(f.rotation <= 2.0, "the row caps: {f:?}");
        let text = m.finals_text().expect("the card");
        assert!(text.contains("TAILSTRIKE"), "{text}");
    }

    #[test]
    fn the_callout_flow_plays_the_pms_rhythm() {
        let mut m = JetTakeoffMeter::default();
        tick_at(&mut m, frame(79.0, 0.0, true), 0.0);
        tick_at(&mut m, frame(81.0, 0.0, true), 0.1);
        let live = m.live.clone().expect("a callout is up");
        assert!(
            live.contains("80 KT") && live.contains("THRUST SET"),
            "{live}"
        );
        tick_at(&mut m, frame(174.0, 0.0, true), 1.0);
        let live = m.live.clone().expect("V1 rides the banner");
        assert!(live.contains("V1"), "{live}");
        // The standing ROTATE at the bug, the banner retired by then.
        tick_at(&mut m, frame(180.0, 0.0, true), 6.0);
        assert!(
            m.live.as_deref().unwrap_or("").contains("ROTATE"),
            "{:?}",
            m.live
        );
        // The parked brief before any of it.
        let mut m2 = JetTakeoffMeter::default();
        tick_at(&mut m2, frame(0.0, 0.0, true), 0.0);
        let live = m2.live.expect("the standing start");
        assert!(live.contains("CLEARED FOR TAKEOFF"), "{live}");
    }

    #[test]
    fn a_dirty_handoff_waits_for_the_clean_up() {
        let mut m = JetTakeoffMeter::default();
        book_roll(&mut m);
        // A/P on with the flaps still out: the gate must not open.
        let mut dirty_ap = frame(210.0, 1800.0, false);
        dirty_ap.cas_kts = 205.0;
        dirty_ap.ap = true;
        dirty_ap.vert = VertMode::LvlChg;
        dirty_ap.at = AtMode::N1;
        tick_at(&mut m, dirty_ap, 6.0);
        assert!(m.done.is_none(), "a dirty handoff grades nothing");
        // Past the POSITIVE RATE banner, the standing prompt coaches the
        // clean-up.
        tick_at(&mut m, dirty_ap, 8.0);
        tick_at(&mut m, dirty_ap, 9.0);
        let live = m.live.clone().expect("the prompt coaches the clean-up");
        assert!(live.contains("FLAPS UP"), "{live}");
        // Clean, and the window runs — with the flaps dragged up early at
        // 800 ft, well under the 1,500 ft acceleration altitude.
        let mut clean = dirty_ap;
        clean.flap_deg = 0.0;
        clean.cas_kts = 230.0;
        clean.alt_ft = 800.0;
        let mut secs = 9.5;
        while m.done.is_none() && secs < 14.0 {
            tick_at(&mut m, clean, secs);
            secs += DT;
        }
        assert!(m.done.is_some(), "the clean handoff freezes the card");
        // The CLEAN-UP row pays for the 800 ft retraction without
        // collapsing: the gear term still carries its half.
        let f = m.done.unwrap();
        assert!((f.flap_up_alt.unwrap() - 800.0).abs() < 1.0, "{f:?}");
        assert!(
            (5.0..=7.5).contains(&f.cleanup),
            "the early retraction pays: {f:?}"
        );
    }

    #[test]
    fn a_stick_tap_reopens_the_window() {
        let mut m = JetTakeoffMeter::default();
        book_roll(&mut m);
        let mut clean = frame(230.0, 2000.0, false);
        clean.cas_kts = 230.0;
        clean.flap_deg = 0.0;
        clean.ap = true;
        clean.vert = VertMode::LvlChg;
        clean.at = AtMode::N1;
        // One second quiet, then the stick drops the A/P.
        let mut secs = 6.0;
        while secs < 7.0 {
            tick_at(&mut m, clean, secs);
            secs += DT;
        }
        let mut dropped = clean;
        dropped.ap = false;
        dropped.vert = VertMode::Off;
        dropped.at = AtMode::Off;
        tick_at(&mut m, dropped, 7.0);
        assert!(m.done.is_none(), "the window reset");
        // Re-engage: the full two seconds again.
        let mut secs = 7.1;
        while m.done.is_none() && secs < 9.0 {
            tick_at(&mut m, clean, secs);
            secs += DT;
        }
        assert!(m.done.is_none(), "not yet two quiet seconds");
        while m.done.is_none() && secs < 10.0 {
            tick_at(&mut m, clean, secs);
            secs += DT;
        }
        assert!(m.done.is_some(), "the second window freezes the card");
    }

    #[test]
    fn handholding_past_five_thousand_is_a_fail() {
        let mut m = JetTakeoffMeter::default();
        book_roll(&mut m);
        // Climbing through the ceiling hand-flown and dirty.
        let mut high = frame(200.0, 4600.0, false);
        high.cas_kts = 200.0;
        tick_at(&mut m, high, 6.0);
        high.alt_ft = 5100.0;
        tick_at(&mut m, high, 7.0);
        assert_eq!(m.fail, Some(FAIL_CEILING));
        assert_eq!(m.done, None);
        assert_eq!(m.live, None, "the popup owns the display");
    }

    #[test]
    fn the_flat_fails_park_the_attempt() {
        // Tailstrike on the pavement.
        let mut m = JetTakeoffMeter::default();
        let mut rotate = frame(170.0, 0.0, true);
        rotate.pitch_deg = 18.0;
        tick_at(&mut m, rotate, 0.0);
        assert_eq!(m.fail, Some(FAIL_TAIL));
        // Settling back onto the runway.
        let mut m = JetTakeoffMeter::default();
        book_roll(&mut m);
        let mut sank = frame(160.0, 50.0, true);
        sank.cas_kts = 160.0;
        tick_at(&mut m, sank, 6.0);
        assert_eq!(m.fail, Some(FAIL_SANK));
        // A steep bank before the handoff.
        let mut m = JetTakeoffMeter::default();
        book_roll(&mut m);
        let mut steep = frame(200.0, 800.0, false);
        steep.cas_kts = 200.0;
        steep.roll_deg = 35.0;
        tick_at(&mut m, steep, 6.0);
        assert_eq!(m.fail, Some(FAIL_STEEP));
        // The climb eating its speed.
        let mut m = JetTakeoffMeter::default();
        book_roll(&mut m);
        let mut slow = frame(145.0, 900.0, false);
        slow.cas_kts = 145.0;
        slow.pitch_deg = 16.0;
        tick_at(&mut m, slow, 6.0);
        assert_eq!(m.fail, Some(FAIL_SLOW));
    }

    #[test]
    fn a_leveled_handoff_keeps_the_card_but_pays_for_it() {
        let mut m = JetTakeoffMeter::default();
        book_roll(&mut m);
        // Clean at 2,200 ft but ALT HOLD level: the gate opens (clean and on),
        // the AUTOMATION row pays for the level-off.
        let mut level = frame(230.0, 2200.0, false);
        level.cas_kts = 230.0;
        level.vs_fpm = 0.0;
        level.flap_deg = 0.0;
        level.ap = true;
        level.vert = VertMode::AltHold;
        level.at = AtMode::Speed;
        let mut secs = 6.0;
        while m.done.is_none() && secs < 12.0 {
            tick_at(&mut m, level, secs);
            secs += DT;
        }
        let f = m.done.expect("the gate is configuration, not the climb");
        assert!((f.engage_alt - 2200.0).abs() < 1.0);
        assert!(
            f.automation < 8.0 && f.automation > 5.0,
            "ALT HOLD at a good altitude pays, not collapses: {f:?}"
        );
    }

    #[test]
    fn a_low_grab_scores_the_automation_row_down() {
        let mut m = JetTakeoffMeter::default();
        book_roll(&mut m);
        let mut low = frame(230.0, 450.0, false);
        low.cas_kts = 230.0;
        low.flap_deg = 0.0;
        low.ap = true;
        low.vert = VertMode::LvlChg;
        low.at = AtMode::N1;
        // The climb-out continues while the window runs.
        let mut secs = 6.0;
        let mut alt = 450.0;
        while m.done.is_none() && secs < 12.0 {
            low.alt_ft = alt;
            tick_at(&mut m, low, secs);
            secs += DT;
            alt += 10.0;
        }
        let f = m.done.expect("graded");
        assert!(
            (f.engage_alt - 450.0).abs() < 1.0,
            "the engagement altitude is the entry frame: {f:?}"
        );
        assert!(
            (7.0..=7.6).contains(&f.automation),
            "a ~450 ft grab pays the altitude term, not the whole row: {f:?}"
        );
    }

    #[test]
    fn a_late_gear_hurts_the_clean_up_row() {
        let mut m = JetTakeoffMeter::default();
        book_roll(&mut m);
        // Gear still travelling at 14 s of climb: the term is near zero.
        let mut dirty = frame(205.0, 1200.0, false);
        dirty.cas_kts = 200.0;
        dirty.gear_pos = 0.5;
        let mut secs = 6.0;
        while secs < 19.5 {
            tick_at(&mut m, dirty, secs);
            secs += DT;
        }
        let mut clean = dirty;
        clean.gear_pos = 0.0;
        clean.cas_kts = 230.0;
        clean.flap_deg = 0.0;
        clean.alt_ft = 2000.0;
        clean.ap = true;
        clean.vert = VertMode::LvlChg;
        clean.at = AtMode::N1;
        let mut secs = 20.0;
        while m.done.is_none() && secs < 24.0 {
            tick_at(&mut m, clean, secs);
            secs += DT;
        }
        let f = m.done.expect("graded");
        assert!(f.gear_up_s.unwrap() > 13.0, "{:?}", f.gear_up_s);
        assert!(f.cleanup < 6.0, "the 14 s gear pays half the row: {f:?}");
    }

    #[test]
    fn a_graded_attempt_rearms_quietly_in_the_air() {
        let mut m = JetTakeoffMeter::default();
        book_roll(&mut m);
        book_climb(&mut m, 6.0);
        assert!(m.done.is_some());
        // CONTINUE past the card: a fresh attempt that stays quiet airborne
        // (the liftoff edge needs a ground frame).
        m.rearm();
        let fly_on = frame(230.0, 4000.0, false);
        tick_at(&mut m, fly_on, 30.0);
        assert_eq!(m.done, None);
        assert!(m.live.is_none() || m.phase == super::Phase::Roll);
        let _ = ACCEL_ALT;
    }
}
