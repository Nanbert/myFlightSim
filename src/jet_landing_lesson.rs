//! The 737's graded LANDING lesson (M3): fly the needles, stabilize, grease
//! it, and bring 60 tonnes to a stop on the runway.
//!
//! The seat is a mid-range final — 4.9 NM out at ~1,650 ft, 180 kt, flaps 5,
//! gear up — just above the 3° beam so it captures at once. From there the
//! lesson is the airline arrival discipline:
//!
//! - **The config ladder** — flaps 15, gear, flaps 30, slow to Vref+5 —
//!   coached by the PM prompt, flown by hand or APP-coupled alike (the A/P
//!   hands over at touchdown on its own; RETARD has already left the
//!   throttles at idle).
//! - **The 1,000 ft stabilized gate**, the airline's hard rule: gear down,
//!   flaps 15 or more, at or under Vref+25, sinking no harder than 1,000
//!   fpm
//!   (the same `approach::criteria` the free-flight gate checks). Anything
//!   red there is a **GO AROUND FAIL** — R and fly it again.
//! - **The touchdown** — sink at the wheels, the 300 m aiming point, the
//!   speed against Vref, on the centreline; past 600 fpm at contact is a
//!   hard-landing FAIL.
//! - **The rollout** — spoilers and reverse out by 80 kt, the centreline
//!   held, throttles idle, braked to a full stop (5 kt held 3 s freezes the
//!   card). A bounce costs, a touch-and-go re-seats the touchdown.

use bevy::prelude::*;

use crate::airfield::AirfieldSpec;
use crate::app_state::{AppState, unpaused};
use crate::autopilot::VertMode;
use crate::mission::{ActiveLesson, Lesson};
use crate::scoring::{score, word};
use crate::sim::AircraftState;

/// The stabilized gate's altitude, feet AAL.
const GATE_AGL: f64 = 1000.0;
/// Approach sampling stops inside the flare: its arc must not spoil the
/// averages, and inside 400 m the beam math goes tiny anyway.
const FLARE_AGL: f64 = 50.0;
const FLARE_M: f64 = 400.0;
/// The landing flap setting the speed band is graded against (flaps 30).
const LAND_FLAP_NORM: f64 = 0.75;
/// The final's speed band: Vref+5 .. Vref+25 (131..151 kt at flaps 30).
/// Vref(flaps 30) = 145 - 25*0.75 = 126.25, spelled out for the consts.
const VREF30: f64 = 126.25;
const BAND_LO: f64 = VREF30 + 5.0;
const BAND_HI: f64 = VREF30 + 25.0;
/// LOC deviation anchors, degrees (one needle-dot discipline).
const LOC_TOL: f32 = 0.3;
const LOC_CAP: f32 = 1.5;
/// GS deviation anchors, degrees.
const GS_TOL: f32 = 0.15;
const GS_CAP: f32 = 0.7;
/// The aiming point (the free-flight card's 300 m mark) and its window.
const POINT_M: f64 = 300.0;
const POINT_WINDOW: f32 = 150.0;
const POINT_CAP: f32 = 350.0;
/// Sink at contact: gentler than the pivot greases, the row dies at the cap.
const RATE_PIVOT: f64 = 100.0;
const RATE_CAP: f32 = 500.0;
/// Contact-speed cap, kt off Vref.
const SPEED_CAP: f32 = 20.0;
/// Centreline-at-contact cap, metres.
const CENTRE_CAP: f32 = 10.0;
/// Past this sink at contact the landing is a hard-landing FAIL.
const HARD_FPM: f64 = -600.0;
/// Reverse out within this many seconds of touchdown earns its credit.
const REV_ENGAGE_S: f32 = 3.0;
/// Reverse must be stowed by this ground speed.
const REV_STOW_KT: f64 = 80.0;
/// The stop gate: 5 kt on the wheels, held 3 s.
const STOP_GS: f64 = 5.0;
const STOP_SECS: f32 = 3.0;
/// Rolling below this is the parking brake, not a rollout to sample.
const ROLLING_GS: f64 = 3.0;
/// A touchdown edge needs credible air time and a moving ground speed.
const TOUCH_MIN_AIR: f32 = 1.0;
const TOUCH_MIN_GS: f64 = 20.0;
/// A hop within this window of the touchdown is a bounce; longer is a
/// touch-and-go and the re-contact becomes the graded touchdown.
const BOUNCE_WINDOW: f32 = 3.0;
/// Shorter airborne gaps than this are wow flicker, not bounces.
const BOUNCE_MIN_GAP: f32 = 0.25;
/// A bounce's flat price on the weighted final.
const BOUNCE_PENALTY: f32 = 0.8;
/// How long a crossing callout holds the banner.
const CALLOUT_SECS: f32 = 2.5;

// --- row weights (they sum to 1) ------------------------------------------------

const W_LOC: f32 = 0.10;
const W_GS: f32 = 0.15;
const W_SPEED: f32 = 0.10;
const W_TOUCH: f32 = 0.45;
const W_ROLLOUT: f32 = 0.20;

const FAIL_GO_AROUND: &str = "FAIL - UNSTABLE AT 1000";
const FAIL_HARD: &str = "FAIL - HARD LANDING\nMore than 600 fpm at the wheels.";

#[derive(Clone, Copy, PartialEq, Default)]
enum Phase {
    /// On the final: config ladder, the gate, the beam samplers.
    #[default]
    Approach,
    /// Wheels on: the touchdown frame, the rollout, the stop gate.
    Rollout,
    /// A fail or the stopped grade froze the attempt.
    Graded,
}

/// The contact frame's graded readouts.
#[derive(Clone, Copy)]
struct TouchFrame {
    /// Smoothed sink at contact, fpm.
    fpm: f64,
    /// Contact speed, kt.
    cas: f64,
    /// Flap setting at contact (normalized) — the speed row's Vref.
    flap_norm: f64,
    /// Lateral offset at contact, metres (signed).
    lateral: f64,
    /// Distance past the 36 threshold at contact, metres (the mark is 300).
    along: f64,
}

/// One frame's readouts the tick needs: pure and testable, like the jet
/// takeoff lesson's `Telem`.
/// The MCP speed window the frame flew with — the `[` arithmetic's input.
fn mcp_window(t: &Telem) -> f64 {
    t.mcp_ias
}

pub struct Telem {
    on_ground: bool,
    gs_kts: f64,
    cas_kts: f64,
    /// AAL, feet (the field sits at sea level).
    alt_ft: f64,
    vs_fpm: f64,
    /// World position — the centreline offset and the along-track come off it.
    pos: Vec3,
    on_runway: bool,
    /// ILS deviations, degrees (+ right / + above); zero outside the volume.
    loc_dev_deg: f64,
    gs_dev_deg: f64,
    /// Along-course distance to the threshold, metres.
    dist_m: f64,
    /// The MCP IAS window, kt (the prompt's [ arithmetic reads it).
    mcp_ias: f64,
    gear_pos: f64,
    flap_deg: f64,
    reverse_pos: f64,
    /// The A/P's vertical mode - the coupling recipe stays up until the
    /// beams are flying.
    vert: VertMode,
}

#[derive(Resource, Default)]
pub struct JetLandingMeter {
    phase: Phase,
    /// The live prompt text; `None` hides the centred card.
    pub(crate) live: Option<String>,
    /// A transient banner (a callout crossing) and when it retires.
    callout: Option<(String, f32)>,
    /// The beam samplers: |dev| sums over the approach, outside the flare.
    loc_sum: f32,
    loc_n: u32,
    gs_sum: f32,
    gs_n: u32,
    /// The speed band between the gate and the flare: total and in-band time.
    band_secs: f32,
    in_secs: f32,
    /// A smoothed vertical speed (the raw readout is noisy at contact).
    vs_smooth: f64,
    smoothed: bool,
    /// The gate fires once, on the crossing frame below 1,000 ft.
    gate_checked: bool,
    /// The 500 ft callout crossing.
    five_checked: bool,
    /// The contact frame; `None` until the wheels first carry.
    td: Option<TouchFrame>,
    td_at: Option<f32>,
    /// Reverse bookkeeping on the rollout.
    rev_engage_at: Option<f32>,
    rev_stowed_by_80: Option<bool>,
    /// Credible air time; gates the touchdown edge against spawn flicker.
    air_time: f32,
    air_gap: f32,
    bounces: u32,
    /// The rollout's centreline sampler.
    roll_sum: f32,
    roll_n: u32,
    /// Consecutive stop-gate seconds on the wheels.
    stop_secs: f32,
    /// The fail that parked the attempt, for the lesson-flow popup.
    pub(crate) fail: Option<&'static str>,
    /// The gate items that were red at the crossing, as the popup's body.
    pub(crate) fail_detail: Option<String>,
    /// The frozen finals of the attempt (at the stop).
    pub(crate) done: Option<Finals>,
    /// A graded attempt holds its card until it re-arms.
    pub(crate) settled: bool,
    was_ground: bool,
}

impl JetLandingMeter {
    /// A fresh attempt: the Flying entry's hand-out, and what the CONTINUE
    /// past a finished card consumes.
    pub(crate) fn rearm(&mut self) {
        *self = JetLandingMeter::default();
    }

    /// The finals card's rows: every graded row prints its number and word
    /// band, the weighted final on top.
    pub fn finals_text(&self) -> Option<String> {
        let f = self.done?;
        let mut text = format!(
            "737 LANDING\n\
             LOC   Δ avg {:4.1}°      {:4.1} {}\n\
             GS    Δ avg {:4.1}°      {:4.1} {}\n\
             SPEED {:3.0}% in band    {:4.1} {}\n\
             RATE  {:+.0} fpm         {:4.1} {}\n\
             POINT {:4.0} m           {:4.1} {}\n\
             SPEED {:+.1} / VREF      {:4.1} {}\n\
             CENTRE {:+.1} m          {:4.1} {}\n\
             ROLLOUT Δ {:4.1} m       {:4.1} {}",
            f.loc_dev,
            f.loc,
            word(f.loc),
            f.gs_dev,
            f.gs,
            word(f.gs),
            f.band_pct * 100.0,
            f.speed,
            word(f.speed),
            f.td_fpm,
            f.rate,
            word(f.rate),
            f.td_along,
            f.point,
            word(f.point),
            f.td_cas,
            f.td_speed,
            word(f.td_speed),
            f.td_lateral,
            f.centre,
            word(f.centre),
            f.roll_dev,
            f.rollout,
            word(f.rollout),
        );
        if f.bounces > 0 {
            text.push_str(&format!(
                "\nBOUNCE ×{}  −{:.1}",
                f.bounces,
                BOUNCE_PENALTY * f.bounces as f32
            ));
        }
        text.push_str(&format!("\nFINAL {:.1}  {}", f.final_, word(f.final_)));
        Some(text)
    }
}

/// The frozen finals: each row a 0–10 score, the final their weighted
/// average minus a flat price per bounce.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct Finals {
    loc_dev: f32,
    loc: f32,
    gs_dev: f32,
    gs: f32,
    band_pct: f32,
    speed: f32,
    td_fpm: f64,
    rate: f32,
    td_along: f64,
    point: f32,
    td_cas: f64,
    td_speed: f32,
    td_lateral: f64,
    centre: f32,
    roll_dev: f32,
    rollout: f32,
    bounces: u32,
    final_: f32,
}

/// Feed one frame's readouts. Pure over its own state, for tests.
fn tick(m: &mut JetLandingMeter, t: Telem, now: f32, dt: f32) {
    if m.callout.as_ref().is_some_and(|(_, until)| now >= *until) {
        m.callout = None;
    }
    if !m.smoothed {
        m.smoothed = true;
        m.vs_smooth = t.vs_fpm;
    } else {
        m.vs_smooth += (t.vs_fpm - m.vs_smooth) * 0.18;
    }
    let contact = t.on_ground && !m.was_ground;
    let credible = contact && m.air_time > TOUCH_MIN_AIR && t.gs_kts > TOUCH_MIN_GS;
    if !t.on_ground && t.gs_kts > 10.0 {
        m.air_time += dt;
        m.air_gap += dt;
    } else if t.on_ground {
        m.air_time = 0.0;
    }
    let in_flare = !t.on_ground && t.alt_ft < FLARE_AGL && t.dist_m < FLARE_M;

    match m.phase {
        Phase::Approach => {
            // The stabilized gate, once, on the airborne frame that slides
            // below 1,000 ft: the airline's go-around rule, the lesson's
            // flat FAIL.
            if !m.gate_checked && !t.on_ground && t.alt_ft < GATE_AGL {
                m.gate_checked = true;
                let flap_norm = (t.flap_deg / 40.0).clamp(0.0, 1.0);
                let items = crate::approach::criteria(
                    t.gear_pos, t.flap_deg, t.cas_kts, t.vs_fpm, flap_norm,
                );
                let red: Vec<&str> = items.iter().filter(|i| !i.ok).map(|i| i.label).collect();
                if !red.is_empty() {
                    m.fail = Some(FAIL_GO_AROUND);
                    m.fail_detail =
                        Some(format!("Go around and fly it again.\n\n{}", red.join("\n")));
                } else {
                    m.callout = Some(("1000 — STABLE".to_owned(), now + CALLOUT_SECS));
                }
            }
            if m.fail.is_some() {
                m.phase = Phase::Graded;
                m.live = None;
                return;
            }
            if !m.five_checked && !t.on_ground && t.alt_ft < 500.0 {
                m.five_checked = true;
                m.callout = Some(("500".to_owned(), now + CALLOUT_SECS));
            }
            // The beam samplers, outside the flare.
            if !t.on_ground && !in_flare && t.dist_m > FLARE_M && t.alt_ft > FLARE_AGL {
                m.loc_sum += t.loc_dev_deg.abs() as f32;
                m.loc_n += 1;
                m.gs_sum += t.gs_dev_deg.abs() as f32;
                m.gs_n += 1;
            }
            // The speed band runs from the gate to the flare.
            if !t.on_ground && m.gate_checked && !in_flare {
                m.band_secs += dt;
                if (BAND_LO..=BAND_HI).contains(&t.cas_kts) {
                    m.in_secs += dt;
                }
            }
            // First contact: the graded touchdown frame.
            if credible {
                let frame = touch_frame(&t, m);
                if frame.fpm < HARD_FPM {
                    m.fail = Some(FAIL_HARD);
                    m.fail_detail = None;
                    m.phase = Phase::Graded;
                    m.live = None;
                    return;
                }
                m.td = Some(frame);
                m.td_at = Some(now);
                m.air_gap = 0.0;
                m.phase = Phase::Rollout;
            }
        }
        Phase::Rollout => {
            if contact {
                // A hop past the flicker bar inside the bounce window is a
                // bounce; a longer absence is a touch-and-go and the
                // re-contact becomes the graded touchdown.
                if m.td.is_some() && m.air_gap > BOUNCE_MIN_GAP {
                    if now - m.td_at.expect("td implies its stamp") < BOUNCE_WINDOW {
                        m.bounces += 1;
                    } else {
                        m.td = Some(touch_frame(&t, m));
                        m.td_at = Some(now);
                        m.bounces = 0;
                        m.roll_sum = 0.0;
                        m.roll_n = 0;
                        m.stop_secs = 0.0;
                        m.rev_engage_at = None;
                        m.rev_stowed_by_80 = None;
                    }
                }
                m.air_gap = 0.0;
            } else if m.fail.is_none() {
                // The reverse bookkeeping and the rollout sampler.
                if m.rev_engage_at.is_none() && t.reverse_pos > 0.1 {
                    m.rev_engage_at = Some(now);
                }
                if t.gs_kts < REV_STOW_KT && m.rev_stowed_by_80.is_none() {
                    m.rev_stowed_by_80 = Some(t.reverse_pos < 0.1);
                }
                if t.on_ground && t.gs_kts > ROLLING_GS && t.on_runway {
                    m.roll_sum += t.pos.x.abs();
                    m.roll_n += 1;
                }
                if t.on_ground && t.gs_kts < STOP_GS {
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
        }
        Phase::Graded => {
            m.live = None;
            return;
        }
    }
    m.was_ground = t.on_ground;
    m.live = compose_live(m, &t, now);
}

/// The contact frame off a credible touchdown edge.
fn touch_frame(t: &Telem, m: &JetLandingMeter) -> TouchFrame {
    TouchFrame {
        fpm: m.vs_smooth,
        cas: t.cas_kts,
        flap_norm: (t.flap_deg / 40.0).clamp(0.0, 1.0),
        lateral: t.pos.x as f64,
        // Northbound landing: the threshold sits at z = 140, the rollout
        // runs to smaller z.
        along: 140.0 - t.pos.z as f64,
    }
}

/// The banner-first live prompt: a callout while it runs, otherwise the
/// phase's PM flow.
fn compose_live(m: &JetLandingMeter, t: &Telem, now: f32) -> Option<String> {
    if m.settled || m.done.is_some() || m.fail.is_some() {
        return None;
    }
    if let Some((text, until)) = &m.callout {
        if now < *until {
            return Some(text.clone());
        }
    }
    let in_flare = !t.on_ground && t.alt_ft < FLARE_AGL && t.dist_m < FLARE_M;
    match m.phase {
        Phase::Approach => {
            if in_flare {
                return Some("RETARD — IDLE".to_owned());
            }
            if m.gate_checked {
                return Some(format!(
                    "FINAL\n{:.0} KT / {:.0}-{:.0}    {:.0} ft",
                    t.cas_kts, BAND_LO, BAND_HI, t.alt_ft
                ));
            }
            // The config ladder: the first item not yet done is the call.
            // The window is 5 kt a press: the prompt does the arithmetic,
            // naming the target and how many [ presses take it there.
            let window = mcp_window(t);
            let config = if t.flap_deg < 4.5 {
                format!(
                    "FLAPS 5 (F)\n[ x{} -> 160 KT",
                    (window - 160.0).max(0.0) / 5.0
                )
            } else if t.flap_deg < 14.5 {
                format!(
                    "FLAPS 15 (F)\n[ x{} -> 160 KT",
                    (window - 160.0).max(0.0) / 5.0
                )
            } else if t.gear_pos < 0.95 {
                format!(
                    "FLAPS 15 SET — GEAR DOWN (U)\n[ x{} -> 131 KT",
                    (window - 131.0).max(0.0) / 5.0
                )
            } else if t.flap_deg < 29.5 {
                format!(
                    "GEAR DOWN — FLAPS 30 (F)\n[ x{} -> 131 KT",
                    (window - 131.0).max(0.0) / 5.0
                )
            } else {
                format!(
                    "CLEAN CONFIG — STABLE BY 1000\n{:.0} KT    {:.0} ft",
                    t.cas_kts, t.alt_ft
                )
            };
            // Until the beams fly (and there is room above the gate), the
            // coupling recipe rides on top - the SOP the real crew runs off
            // procedure, not instruments.
            let coupled = matches!(t.vert, VertMode::Gs | VertMode::Flare);
            if coupled || t.alt_ft <= 1200.0 {
                Some(config)
            } else {
                // G first: the mode keys are dead until the A/P is engaged.
                Some(format!("COUPLE: G A/P - 5 A/T - 3 V/S - 6 APP\n{config}"))
            }
        }
        Phase::Rollout => {
            if t.reverse_pos < 0.1 && t.gs_kts > 60.0 {
                Some("SPOILERS (L) — REVERSE (K)".to_owned())
            } else if t.gs_kts > REV_STOW_KT {
                Some(format!("ROLLOUT\n{:.0} KT", t.gs_kts))
            } else if t.reverse_pos >= 0.1 {
                Some("80 KT — STOW REVERSE".to_owned())
            } else if t.gs_kts > STOP_GS {
                Some("THROTTLES IDLE — BRAKE TO A STOP".to_owned())
            } else {
                Some("HOLDING THE STOP".to_owned())
            }
        }
        Phase::Graded => None,
    }
}

/// The attempt frozen into finals at the stop.
fn finals(m: &JetLandingMeter) -> Finals {
    let td = m.td.expect("the stop implies a touchdown");
    let loc_dev = if m.loc_n > 0 {
        m.loc_sum / m.loc_n as f32
    } else {
        0.0
    };
    let gs_dev = if m.gs_n > 0 {
        m.gs_sum / m.gs_n as f32
    } else {
        0.0
    };
    let band_pct = if m.band_secs > 0.0 {
        m.in_secs / m.band_secs
    } else {
        1.0
    };
    let loc = score(loc_dev, LOC_TOL, LOC_CAP);
    let gs = score(gs_dev, GS_TOL, GS_CAP);
    let speed = (band_pct * 10.0).clamp(0.0, 10.0);
    // The touchdown row: the sink carries it, the point and the speed keep
    // it honest, the centreline is the tie-breaker.
    let rate = score(((-td.fpm) - RATE_PIVOT).max(0.0) as f32, 0.0, RATE_CAP);
    let point = score((td.along - POINT_M).abs() as f32, POINT_WINDOW, POINT_CAP);
    let td_speed = score(
        (td.cas - crate::landing::vref(td.flap_norm)).abs() as f32,
        0.0,
        SPEED_CAP,
    );
    let centre = score(td.lateral.abs() as f32, 0.0, CENTRE_CAP);
    let touch = 0.35 * rate + 0.25 * point + 0.25 * td_speed + 0.15 * centre;
    // The rollout: the centreline and the reverse discipline.
    let roll_dev = if m.roll_n > 0 {
        m.roll_sum / m.roll_n as f32
    } else {
        0.0
    };
    let centre_roll = score(roll_dev, 1.5, 8.0);
    let engage = m
        .rev_engage_at
        .zip(m.td_at)
        // Full credit inside a second, gone three seconds past the deadline.
        .map(|(eng, td)| score((eng - td).max(0.0), 1.0, REV_ENGAGE_S + 3.0))
        .unwrap_or(0.0);
    let stowed = m.rev_stowed_by_80.unwrap_or(true);
    let reverse_term = engage * 0.4 + if stowed { 6.0 } else { 0.0 };
    let rollout = 0.6 * centre_roll + 0.4 * reverse_term;
    let final_ =
        (W_LOC * loc + W_GS * gs + W_SPEED * speed + W_TOUCH * touch + W_ROLLOUT * rollout
            - BOUNCE_PENALTY * m.bounces as f32)
            .max(0.0);
    Finals {
        loc_dev,
        loc,
        gs_dev,
        gs,
        band_pct,
        speed,
        td_fpm: td.fpm,
        rate,
        td_along: td.along,
        point,
        td_cas: td.cas,
        td_speed,
        td_lateral: td.lateral,
        centre,
        roll_dev,
        rollout,
        bounces: m.bounces,
        final_,
    }
}

/// The update system: only the jet landing lesson, only the home field.
fn step_m3(
    aircraft: Res<AircraftState>,
    ap: Res<crate::autopilot::Autopilot>,
    mcp: Res<crate::autopilot::Mcp>,
    ils: Res<crate::ils::Ils>,
    airfield: Res<crate::airfield::Airfield>,
    map: Res<crate::map::ActiveMap>,
    lesson: Res<ActiveLesson>,
    time: Res<Time>,
    mut m: ResMut<JetLandingMeter>,
) {
    if map.0.is_some() || lesson.0 != Some(Lesson::JetLanding) {
        return;
    }
    let default_spec = AirfieldSpec::zsnm();
    let spec = airfield.0.as_ref().unwrap_or(&default_spec);
    // The runway-36 volume: course 0, beam within ~9.3 km. Outside it the
    // deviations read zero (and the flare window guards the last metres).
    let on_beam = ils.available && ils.course_deg == 0.0;
    tick(
        &mut m,
        Telem {
            on_ground: aircraft.on_ground,
            gs_kts: aircraft.gs_kts,
            cas_kts: aircraft.cas_kts,
            alt_ft: aircraft.alt_ft,
            vs_fpm: aircraft.vs_fpm,
            pos: aircraft.pos,
            on_runway: spec.on_runway(aircraft.pos.x, aircraft.pos.z),
            loc_dev_deg: if on_beam { ils.loc_dev_deg } else { 0.0 },
            gs_dev_deg: if on_beam { ils.gs_dev_deg } else { 0.0 },
            dist_m: ils.dist_m,
            gear_pos: aircraft.gear_pos,
            mcp_ias: mcp.ias_kts,
            flap_deg: aircraft.flap_deg,
            reverse_pos: aircraft.reverse_pos,
            vert: ap.vert,
        },
        time.elapsed_secs(),
        time.delta_secs(),
    );
}

pub struct JetLandingLessonPlugin;

impl Plugin for JetLandingLessonPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<JetLandingMeter>()
            .add_systems(OnEnter(AppState::Flying), reset_jet_landing)
            .add_systems(
                Update,
                // Fresh readouts in; the flow's watcher reads what we froze.
                step_m3
                    .after(crate::sim::publish_state)
                    .before(crate::lesson_flow::watch)
                    .run_if(unpaused),
            );
    }
}

/// A fresh flight hands over a fresh attempt.
fn reset_jet_landing(mut m: ResMut<JetLandingMeter>) {
    m.rearm();
}

#[cfg(test)]
mod tests {
    use super::{
        FAIL_GO_AROUND, FAIL_HARD, FLARE_AGL, FLARE_M, GATE_AGL, JetLandingMeter, Telem, tick,
    };
    use crate::autopilot::VertMode;
    use bevy::prelude::*;

    const DT: f32 = 1.0 / 30.0;
    const NOW: f32 = 100.0;
    /// The threshold sits at z = 140; touchdown at the 300 m mark is z = -160.
    const Z_TD: f32 = -160.0;

    fn frame(alt: f64, dist: f64, cas: f64, flap: f64, gear: f64) -> Telem {
        Telem {
            on_ground: false,
            gs_kts: cas,
            cas_kts: cas,
            alt_ft: alt,
            vs_fpm: -700.0,
            pos: Vec3::new(0.5, 0.0, 140.0 + dist as f32),
            on_runway: false,
            loc_dev_deg: 0.1,
            gs_dev_deg: 0.05,
            dist_m: dist,
            mcp_ias: 160.0,
            gear_pos: gear,
            flap_deg: flap,
            reverse_pos: 0.0,
            vert: VertMode::Off,
        }
    }

    fn tick_at(m: &mut JetLandingMeter, t: Telem, secs: f32) {
        tick(m, t, NOW + secs, DT);
    }

    /// The book arrival: the config ladder before the gate, on the needles,
    /// in the band, a -150 fpm touchdown at the 300 m mark, reverse out at
    /// once and stowed by 80, braked to a stop.
    fn textbook(m: &mut JetLandingMeter) {
        let mut secs = 0.0;
        // The ladder: flaps 5 -> 15 -> gear -> 30.
        for t in [
            frame(1600.0, 8900.0, 180.0, 5.0, 0.0),
            frame(1400.0, 7600.0, 170.0, 15.0, 0.0),
            frame(1250.0, 6800.0, 160.0, 15.0, 1.0),
            frame(1150.0, 6200.0, 150.0, 30.0, 1.0),
        ] {
            tick_at(m, t, secs);
            secs += DT;
        }
        // The gate crossing, stable: gear down, flaps 30, 140 kt, -700 fpm.
        tick_at(m, frame(990.0, 5300.0, 140.0, 30.0, 1.0), secs);
        secs += DT;
        // The band down to the flare.
        let mut alt = 900.0;
        let mut dist = 4800.0;
        while alt > 120.0 {
            tick_at(m, frame(alt, dist, 135.0, 30.0, 1.0), secs);
            secs += DT;
            alt -= 25.0;
            dist -= 130.0;
        }
        // The flare: gentler sink, so the smoothed rate reads ~-150.
        for _ in 0..30 {
            let mut t = frame(100.0, 500.0, 130.0, 30.0, 1.0);
            t.vs_fpm = -150.0;
            tick_at(m, t, secs);
            secs += DT;
        }
        // The touchdown edge: airborne for well over a second first.
        let mut td = frame(20.0, 100.0, 128.0, 30.0, 1.0);
        td.vs_fpm = -150.0;
        td.on_ground = true;
        td.pos = Vec3::new(0.5, 0.0, Z_TD);
        tick_at(m, td, secs);
        secs += DT;
        // The rollout: reverse out at once, stowed by 80.
        let mut roll = |gs: f64, rev: f64, secs: f32| {
            let mut t = frame(0.0, 0.0, gs, 30.0, 1.0);
            t.on_ground = true;
            t.gs_kts = gs;
            t.vs_fpm = 0.0;
            t.reverse_pos = rev;
            t.on_runway = true;
            t.pos = Vec3::new(0.5, 0.0, Z_TD);
            tick_at(m, t, secs);
        };
        roll(120.0, 0.0, secs);
        secs += DT;
        for _ in 0..30 {
            roll(110.0, 0.5, secs);
            secs += DT;
        }
        roll(85.0, 0.0, secs);
        secs += DT;
        for gs in [70.0, 50.0, 30.0] {
            for _ in 0..10 {
                roll(gs, 0.0, secs);
                secs += DT;
            }
        }
        for _ in 0..(3.5 / DT) as usize {
            roll(3.0, 0.0, secs);
            secs += DT;
        }
    }

    #[test]
    fn a_textbook_arrival_scores_well() {
        let mut m = JetLandingMeter::default();
        textbook(&mut m);
        let f = m.done.expect("the stop froze the finals");
        assert!(m.settled, "the card owns the display at the stop");
        assert!(f.final_ > 9.0, "every row near-perfect: {f:?}");
        assert!(f.rate >= 8.5, "the -150 fpm grease: {f:?}");
        assert!((f.td_along - 300.0).abs() < 1.0, "{}", f.td_along);
        assert_eq!(f.bounces, 0);
        let text = m.finals_text().expect("the card");
        assert!(text.contains("FINAL"), "{text}");
    }

    #[test]
    fn the_stable_gate_callout_and_the_ladder_play() {
        let mut m = JetLandingMeter::default();
        // Pre-config, beams not flying: the coupling recipe rides on top.
        tick_at(&mut m, frame(1600.0, 8900.0, 180.0, 5.0, 0.0), 0.0);
        let live = m.live.clone().expect("the ladder");
        assert!(live.contains("COUPLE"), "the SOP recipe: {live}");
        assert!(live.contains("FLAPS 15"), "{live}");
        // Beams flying: the recipe retires.
        let mut coupled = frame(1500.0, 8300.0, 170.0, 5.0, 0.0);
        coupled.vert = VertMode::Gs;
        tick_at(&mut m, coupled, 0.05);
        let live = m.live.clone().expect("config only");
        assert!(!live.contains("COUPLE"), "{live}");
        tick_at(&mut m, frame(1500.0, 8300.0, 170.0, 15.0, 0.0), 0.1);
        let live = m.live.clone().expect("gear next");
        assert!(live.contains("GEAR DOWN"), "{live}");
        tick_at(&mut m, frame(1400.0, 7600.0, 160.0, 15.0, 1.0), 0.2);
        let live = m.live.clone().expect("flaps 30 next");
        assert!(live.contains("FLAPS 30"), "{live}");
        // Config done, above the gate.
        tick_at(&mut m, frame(1300.0, 7000.0, 145.0, 30.0, 1.0), 0.3);
        let live = m.live.clone().expect("stable by 1000");
        assert!(live.contains("STABLE BY 1000"), "{live}");
        // The crossing: stable — the banner takes the prompt.
        tick_at(&mut m, frame(990.0, 5300.0, 140.0, 30.0, 1.0), 0.4);
        let live = m.live.clone().expect("the gate callout");
        assert!(live.contains("1000"), "{live}");
        // Past the banner, the final's numbers carry the frame.
        tick_at(&mut m, frame(800.0, 4300.0, 135.0, 30.0, 1.0), 3.5);
        let live = m.live.clone().expect("the final line");
        assert!(live.contains("FINAL"), "{live}");
        // The flare hands the throttles to idle (the 500-callout banner
        // rides first; the compressed descent fires it late here).
        for i in 0..90 {
            tick_at(
                &mut m,
                frame(40.0, 350.0, 130.0, 30.0, 1.0),
                4.0 + i as f32 * DT,
            );
        }
        let live = m.live.clone().expect("the retard call");
        assert!(live.contains("RETARD"), "{live}");
    }

    #[test]
    fn an_unstable_gate_is_a_go_around() {
        let mut m = JetLandingMeter::default();
        // Config still clean at the gate: gear up, flaps 5, fast.
        tick_at(&mut m, frame(1010.0, 5400.0, 165.0, 5.0, 0.0), 0.0);
        tick_at(&mut m, frame(990.0, 5300.0, 165.0, 5.0, 0.0), 0.1);
        assert_eq!(m.fail, Some(FAIL_GO_AROUND));
        // The card names what was red: clean at the gate fails the gear and
        // the flap setting; 165 kt is inside Vref(5)+25 and the sink is
        // gentle, so those two stay green.
        let detail = m.fail_detail.clone().expect("the red list");
        assert!(detail.contains("GEAR DOWN"), "{detail}");
        assert!(detail.contains("FLAPS 15"), "{detail}");
        assert!(!detail.contains("\nSPEED"), "{detail}");
        assert!(!detail.contains("SINK"), "{detail}");
        assert_eq!(m.done, None);
        assert_eq!(m.live, None, "the popup owns the display");
    }

    #[test]
    fn a_hard_touchdown_fails() {
        let mut m = JetLandingMeter::default();
        // Stable config down to the flare, then a -750 fpm slam.
        let mut secs = 0.0;
        tick_at(&mut m, frame(990.0, 5300.0, 140.0, 30.0, 1.0), secs);
        secs += DT;
        let mut alt = 900.0;
        while alt > 120.0 {
            tick_at(&mut m, frame(alt, 400.0 + alt, 135.0, 30.0, 1.0), secs);
            secs += DT;
            alt -= 100.0;
        }
        for _ in 0..40 {
            let mut t = frame(60.0, 300.0, 130.0, 30.0, 1.0);
            t.vs_fpm = -750.0;
            tick_at(&mut m, t, secs);
            secs += DT;
        }
        let mut td = frame(10.0, 150.0, 128.0, 30.0, 1.0);
        td.vs_fpm = -750.0;
        td.on_ground = true;
        td.pos = Vec3::new(0.5, 0.0, Z_TD);
        tick_at(&mut m, td, secs);
        assert_eq!(m.fail, Some(FAIL_HARD));
    }

    #[test]
    fn a_bounce_pays_the_flat_price() {
        let mut m = JetTaxiBounce();
        fn JetTaxiBounce() -> JetLandingMeter {
            let mut m = JetLandingMeter::default();
            let mut secs = 0.0;
            // Compress the approach: gate, band, flare, touchdown.
            tick_at(&mut m, frame(990.0, 5300.0, 140.0, 30.0, 1.0), secs);
            secs += DT;
            for i in 0..20 {
                tick_at(
                    &mut m,
                    frame(900.0 - 40.0 * i as f64, 4500.0, 135.0, 30.0, 1.0),
                    secs,
                );
                secs += DT;
            }
            for _ in 0..30 {
                let mut t = frame(100.0, 500.0, 130.0, 30.0, 1.0);
                t.vs_fpm = -150.0;
                tick_at(&mut m, t, secs);
                secs += DT;
            }
            let mut td = frame(20.0, 100.0, 128.0, 30.0, 1.0);
            td.vs_fpm = -150.0;
            td.on_ground = true;
            td.pos = Vec3::new(0.5, 0.0, Z_TD);
            tick_at(&mut m, td, secs);
            secs += DT;
            m
        }
        // Airborne again for half a second (past the flicker bar), then back
        // on the wheels inside the bounce window.
        let mut secs = 1.0;
        for _ in 0..15 {
            let mut t = frame(5.0, 50.0, 120.0, 30.0, 1.0);
            t.vs_fpm = -100.0;
            t.pos = Vec3::new(0.5, 0.0, Z_TD);
            tick_at(&mut m, t, secs);
            secs += DT;
        }
        let mut back = frame(0.0, 0.0, 120.0, 30.0, 1.0);
        back.on_ground = true;
        back.gs_kts = 120.0;
        back.pos = Vec3::new(0.5, 0.0, Z_TD);
        back.on_runway = true;
        tick_at(&mut m, back, secs);
        assert_eq!(m.bounces, 1, "the hop counts");
        // And the stop still grades, the price paid on the final.
        secs += DT;
        for _ in 0..(3.5 / DT) as usize {
            let mut t = frame(0.0, 0.0, 3.0, 30.0, 1.0);
            t.on_ground = true;
            t.gs_kts = 3.0;
            t.pos = Vec3::new(0.5, 0.0, Z_TD);
            t.on_runway = true;
            tick_at(&mut m, t, secs);
            secs += DT;
        }
        let f = m.done.expect("the stop grades after the bounce");
        assert_eq!(f.bounces, 1);
        let text = m.finals_text().expect("the card");
        assert!(text.contains("BOUNCE"), "{text}");
    }

    #[test]
    fn reverse_left_out_dings_the_rollout() {
        let mut m = JetLandingMeter::default();
        let mut secs = 0.0;
        tick_at(&mut m, frame(990.0, 5300.0, 140.0, 30.0, 1.0), secs);
        secs += DT;
        for _ in 0..30 {
            let mut t = frame(100.0, 500.0, 130.0, 30.0, 1.0);
            t.vs_fpm = -150.0;
            tick_at(&mut m, t, secs);
            secs += DT;
        }
        let mut td = frame(20.0, 100.0, 128.0, 30.0, 1.0);
        td.vs_fpm = -150.0;
        td.pos = Vec3::new(0.5, 0.0, Z_TD);
        tick_at(&mut m, td, secs);
        secs += DT;
        // Rollout without any reverse: prompt coaches, the row pays.
        for gs in [110.0, 90.0, 60.0, 30.0] {
            for _ in 0..10 {
                let mut t = frame(0.0, 0.0, gs, 30.0, 1.0);
                t.on_ground = true;
                t.gs_kts = gs;
                t.on_runway = true;
                t.pos = Vec3::new(0.5, 0.0, Z_TD);
                tick_at(&mut m, t, secs);
                secs += DT;
            }
        }
        let mut coach = None;
        for i in 0..90 {
            let mut t = frame(0.0, 0.0, 110.0, 30.0, 1.0);
            t.on_ground = true;
            t.gs_kts = 110.0;
            t.on_runway = true;
            t.pos = Vec3::new(0.5, 0.0, Z_TD);
            tick_at(&mut m, t, secs);
            secs += DT;
            coach = m.live.clone();
            if i > 85 {
                break;
            }
        }
        let early = coach.expect("the coach");
        assert!(early.contains("REVERSE"), "{early}");
        for _ in 0..(3.5 / DT) as usize {
            let mut t = frame(0.0, 0.0, 3.0, 30.0, 1.0);
            t.on_ground = true;
            t.gs_kts = 3.0;
            t.on_runway = true;
            t.pos = Vec3::new(0.5, 0.0, Z_TD);
            tick_at(&mut m, t, secs);
            secs += DT;
        }
        let f = m.done.expect("graded");
        assert!(
            (7.5..9.5).contains(&f.rollout),
            "no reverse pays its 40%: {f:?}"
        );
    }
}
