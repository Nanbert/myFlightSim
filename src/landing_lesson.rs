//! The graded LANDING lesson (M3): the approach, the touchdown and the
//! rollout — the whole illness, not just the touch frame.
//!
//! The lesson is C172 and starts seated 2 NM out on a 3° final for runway 36
//! (the `reset_final_c172` seat: 640 ft, 70 kt, flaps 20). From there every
//! phase is watched, sampled and graded:
//!
//! - **Approach** — a continuous glide-path sampler (the flown path angle
//!   against the 3° line) and the centreline |offset| average, both counted
//!   from the spawn seat down to the flare window. Crossing 500 ft the
//!   approach *should* be stabilized — inside the speed band, within 6 m of
//!   the centreline and sinking no harder than 700 fpm — but that hard gate
//!   is **temporarily disarmed** (`GATE_ON`): on the keyboard the lateral
//!   band was too coarse to hold, and the fail read as the sim's fault. The
//!   sampler keeps counting; the gate's return is one flag away.
//! - **Touchdown frame** — the same readouts the free-flight `landing.rs`
//!   card grades: sink rate, contact speed, centreline at contact and the
//!   touchdown point against the 300 m aiming mark. The mark is scored
//!   inside a ±150 m window; past ±350 m of it the row bottoms out —
//!   landing long has stolen the stopping distance.
//! - **Rollout** — the centreline |offset| average while decelerating (the
//!   M2 roll sampler pointed the other way), counted until the stop.
//!
//! A bounced touchdown costs the final 0.8 a bounce, but never zeroes it —
//! the same weighted-average rule as every graded card. A hop past the
//! short-bounce window is a touch-and-go: the re-contact becomes the graded
//! touchdown and the card forgets the first one. The finish gate is a
//! stopped aircraft: 5 kt on the wheels, held for 3 s, freezes the card.
//!
//! Failures (feeding the lesson-flow FAIL popup): an unstable approach at
//! the 500 ft gate, and — through the flow's always-on pavement check — the
//! wheels touching anything but paved surface. No go-around walk: a fail is
//! a fail, like M2's hedge rules. The free flight's passive `landing.rs`
//! card stays untouched (and mutes while this lesson flies).

use bevy::prelude::*;

use crate::aerodrome::{on_runway, runway_along, THRESHOLD_Z};
use crate::app_state::{unpaused, AppState};
use crate::mission::{ActiveLesson, Lesson};
use crate::scoring::{score, word};
use crate::sim::AircraftState;

/// The glide path the approach is graded against, degrees — the seat is
/// right on it (640 ft at 2 NM is the 3° line), so the first seconds are free.
const GS_PATH_DEG: f64 = 3.0;
/// The 500 ft stabilized-approach gate — **temporarily disarmed**. On the
/// keyboard the lateral band is too coarse to hold through the gate (the
/// plane's own spiral divergence drifts 1–2° too), and a fail there reads
/// as the sim's fault, not the student's. Re-arm by flipping this to true
/// once the fine-key drills settle.
const GATE_ON: bool = false;
/// The 500 ft stabilized-approach gate altitude: inside it, the band below
/// must hold.
const GATE_AGL: f64 = 500.0;
/// The stabilized band at the gate: contact speed well clear of 65.
const GATE_MIN_V: f64 = 52.0;
const GATE_MAX_V: f64 = 75.0;
/// The gate's centreline tolerance, metres off the runway axis.
const GATE_MAX_OFF: f32 = 6.0;
/// The gate's sink budget (fpm); deeper is unstabilized, not a steep approach.
const GATE_MAX_SINK: f64 = 700.0;
/// Approach sampling stops here: past this point the flare owns the path,
/// and its gentle arc must not spoil the approach's own average.
const FLARE_WINDOW_AGL: f64 = 30.0;
/// ...and inside 150 m of the threshold the path math goes tiny anyway.
const FLARE_WINDOW_M: f64 = 150.0;
/// The aiming mark: the C172 touches down at the 300 m point (the mark the
/// free-flight card grades against too).
const POINT_M: f64 = 300.0;
/// The touchdown-point window: inside ±150 m of the mark is full marks.
const POINT_WINDOW: f32 = 150.0;
/// ...and the row dies at ±350 m of it — landed long into the back half of
/// a 1200 m runway has stolen the stopping distance.
const POINT_CAP: f32 = 350.0;
/// The book contact speed, the speed row's centre.
const V_TOUCH: f64 = 65.0;
/// Sink-rate anchor: contacts gentler than this grease. The cap ends 700 —
/// the gate's own sink budget — so the row bottoms out where the rule
/// calls the whole approach unstabilized.
const RATE_PIVOT: f64 = 120.0;
const RATE_CAP: f32 = 480.0;
/// Contact speed cap (kt off the book): the same budget the free-flight
/// card grades (25 kt off target is a zero).
const SPEED_CAP: f32 = 25.0;
/// Centreline-at-contact cap, metres — matching the free-flight card's 10 m.
const CENTRE_CAP: f32 = 10.0;
/// The stop gate: 5 kt on the wheels, held for 3 s, freezes the card.
const STOP_GS: f64 = 5.0;
const STOP_SECS: f32 = 3.0;
/// Rolling below this speed is the parking brake, not a rollout to sample;
/// above it on the ground the centreline sampler counts.
const ROLLING_GS: f64 = 3.0;
/// A touchdown edge needs credible air time behind it (the spawn's wow
/// flicker has none) and a moving ground speed.
const TOUCH_MIN_AIR: f32 = 1.0;
const TOUCH_MIN_GS: f64 = 20.0;
/// A hop within this window of the recorded touchdown is a bounce; a longer
/// absence is a touch-and-go and the re-contact becomes the graded touchdown.
const BOUNCE_WINDOW: f32 = 3.0;
/// Shorter airborne gaps than this are wow flicker, not bounces.
const BOUNCE_MIN_GAP: f32 = 0.25;
/// A bounce's flat price on the weighted final.
const BOUNCE_PENALTY: f32 = 0.8;

// --- row weights ------------------------------------------------------------

const W_RATE: f32 = 0.30;
const W_SPEED: f32 = 0.20;
const W_GS: f32 = 0.15;
const W_TRACK: f32 = 0.10;
const W_POINT: f32 = 0.10;
const W_CENTRE: f32 = 0.05;
const W_ROLLOUT: f32 = 0.10;

/// The stabilized-approach rule, the lesson's own FAIL: inside 500 ft the
/// approach must hold the speed band, the line and the sink. The wheels on
/// grass FAIL through `lesson_flow`'s always-on pavement check.
const FAIL_UNSTABLE: &str = "FAIL - UNSTABLE ON FINAL\nFive hundred feet, and the approach isn't stabilized.";

#[derive(Clone, Copy, PartialEq, Default)]
enum Phase {
    /// Flying the final: the glide-path and centreline samplers run.
    #[default]
    Approach,
    /// Wheels on: the rollout sampler runs to the stop gate.
    Rollout,
    /// A fail or the stopped grade froze the attempt.
    Graded,
}

/// The contact frame's graded readouts (the same terms the free-flight
/// card grades, kept in the lesson's own units).
#[derive(Clone, Copy)]
struct TouchFrame {
    /// Smoothed sink at contact, fpm (negative = descending).
    fpm: f64,
    /// Contact speed, kt.
    cas: f64,
    /// Lateral offset at contact, metres off the centreline (signed).
    lateral: f64,
    /// Distance from the threshold at contact, metres (the mark is 300).
    along: f64,
}

/// The frozen finals: every row keeps its deviation, its 0–10 score and the
/// word band; the final is their weighted average (bad rows pull it down,
/// nothing zeroes it), minus a flat price per bounce.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct Finals {
    /// |path angle − 3°| average (degrees).
    gs_dev: f32,
    gs: f32,
    /// |centreline offset| average through the approach, metres.
    track_dev: f32,
    track: f32,
    /// Smoothed sink at contact, fpm.
    rate_fpm: f64,
    rate: f32,
    /// Contact speed, kt.
    cas: f64,
    speed: f32,
    /// Distance from the threshold at contact, metres.
    along: f64,
    point: f32,
    /// Lateral offset at contact, metres.
    lateral: f64,
    centre: f32,
    /// |centreline offset| average on the rollout, metres.
    roll_dev: f32,
    rollout: f32,
    /// Bounces counted after the recorded touchdown.
    bounces: u32,
    /// The weighted final, bounce price already paid.
    final_: f32,
}

#[derive(Resource, Default)]
pub struct LandingLesson {
    phase: Phase,
    /// The live prompt text, composed every ticking frame; `None` hides the
    /// centred card.
    pub(crate) live: Option<String>,
    /// The glide-path sampler: |path − 3°| sums over the approach.
    gs_dev_sum: f32,
    gs_n: u32,
    /// The centreline sampler: |offset| sums over the approach.
    track_sum: f32,
    track_n: u32,
    /// A smoothed vertical speed (the raw FDM readout is noisy at contact).
    smoothed: bool,
    vs_smooth: f64,
    /// The 500 ft gate fires once, on the crossing frame below it.
    gate_checked: bool,
    /// The contact frame; `None` until the wheels first carry.
    td: Option<TouchFrame>,
    td_at: Option<f32>,
    /// Credible air time (weight off wheels, actually moving); gates the
    /// touchdown edge against the spawn's wow flicker.
    air_time: f32,
    /// Airborne gap since the last ground frame, for bounce counting.
    air_gap: f32,
    bounces: u32,
    /// The rollout sampler: |offset| sums while decelerating.
    roll_sum: f32,
    roll_n: u32,
    /// Consecutive stop-gate seconds on the wheels.
    stop_secs: f32,
    /// The fail that parked the attempt, for the lesson-flow popup.
    pub(crate) fail: Option<&'static str>,
    /// The frozen finals of the attempt (usually at the stop).
    pub(crate) done: Option<Finals>,
    /// A graded attempt holds its card until it re-arms.
    pub(crate) settled: bool,
    was_ground: bool,
}

impl LandingLesson {
    /// A fresh attempt: the hand-out the Flying entry uses, and what the
    /// CONTINUE past a finished card consumes.
    pub(crate) fn rearm(&mut self) {
        *self = LandingLesson::default();
    }

    /// The finals card's rows: every graded row prints its deviation, its
    /// 0–10 score and the word band, with the weighted final on top (the
    /// popup's title carries the lesson name).
    pub fn finals_text(&self) -> Option<String> {
        let f = self.done?;
        let mut text = format!(
            "GS    Δ avg {g:.1}°      {gs:.1} {gw}\n\
             TRACK Δ avg {t:.1} m     {tk:.1} {tw}\n\
             RATE  {r:+.0} fpm       {rt:.1} {rw}\n\
             SPEED {c:+.1} / 65 kt    {sp:.1} {sw}\n\
             POINT {a:.0} m           {po:.1} {pw}\n\
             CENTRE {l:+.1} m         {ce:.1} {cw}\n\
             ROLLOUT Δ avg {ro:.1} m  {rl:.1} {ow}",
            g = f.gs_dev,
            gs = f.gs,
            gw = word(f.gs),
            t = f.track_dev,
            tk = f.track,
            tw = word(f.track),
            r = f.rate_fpm,
            rt = f.rate,
            rw = word(f.rate),
            c = f.cas,
            sp = f.speed,
            sw = word(f.speed),
            a = f.along,
            po = f.point,
            pw = word(f.point),
            l = f.lateral,
            ce = f.centre,
            cw = word(f.centre),
            ro = f.roll_dev,
            rl = f.rollout,
            ow = word(f.rollout),
        );
        if f.bounces > 0 {
            text.push_str(&format!(
                "\nBOUNCE ×{}  −{:.1}",
                f.bounces,
                BOUNCE_PENALTY * f.bounces as f32
            ));
        }
        text.push_str(&format!("\nFINAL {f:.1}  {fw}", f = f.final_, fw = word(f.final_)));
        Some(text)
    }
}

/// One frame's readouts the tick needs, extracted from [`AircraftState`]:
/// pure and testable, like the takeoff lesson's `Telem`.
pub struct Telem {
    on_ground: bool,
    gs_kts: f64,
    cas_kts: f64,
    /// AGL above the field (the aerodrome datum), feet.
    agl_ft: f64,
    vs_fpm: f64,
    heading_deg: f64,
    /// World position — the centreline offset and the along-track come off it.
    pos: Vec3,
    on_runway: bool,
}

/// Feed one frame's readouts. Pure over its own state, for tests.
fn tick(ll: &mut LandingLesson, t: Telem, now: f32, dt: f32) {
    // The vertical-speed smoothing runs every frame like the free-flight
    // card's: contact reads the last word of it, not the raw noise.
    if !ll.smoothed {
        ll.smoothed = true;
        ll.vs_smooth = t.vs_fpm;
    } else {
        ll.vs_smooth += (t.vs_fpm - ll.vs_smooth) * 0.18;
    }
    // The wow edge reads the air accumulated *before* this frame: the
    // contact frame's own ground reset (below) would zero the very value
    // the edge needs — the free-flight card's crashing-order lesson.
    let contact = t.on_ground && !ll.was_ground;
    let credible = contact && ll.air_time > TOUCH_MIN_AIR && t.gs_kts > TOUCH_MIN_GS;
    // Air-time bookkeeping for every phase: the touchdown edge reads it,
    // and a bounce's gap accumulates in the same breath.
    if !t.on_ground && t.gs_kts > 10.0 {
        ll.air_time += dt;
        ll.air_gap += dt;
    } else if t.on_ground {
        ll.air_time = 0.0;
    }
    // The broad form of the edge above (already computed).

    match ll.phase {
        Phase::Approach => {
            // The stabilized gate, once, on the airborne frame that slides
            // below 500 ft — a plane parked on the concrete never gates.
            // (Disarmed for now: see GATE_ON.)
            if GATE_ON
                && ll.fail.is_none()
                && !ll.gate_checked
                && !t.on_ground
                && t.agl_ft < GATE_AGL
            {
                ll.gate_checked = true;
                if unstable(&t) {
                    ll.fail = Some(FAIL_UNSTABLE);
                }
            }
            if ll.fail.is_some() {
                ll.phase = Phase::Graded;
            } else {
                // The approach sampler: the flare window is left alone.
                let dist_m = (t.pos.z - THRESHOLD_Z) as f64;
                if !t.on_ground && dist_m > FLARE_WINDOW_M && t.agl_ft > FLARE_WINDOW_AGL {
                    let agl_m = t.agl_ft * 0.3048;
                    let path_deg = (agl_m / dist_m).atan().to_degrees();
                    ll.gs_dev_sum += (path_deg - GS_PATH_DEG).abs() as f32;
                    ll.gs_n += 1;
                    ll.track_sum += (t.pos.x as f64).abs() as f32;
                    ll.track_n += 1;
                }
                // First contact: the graded touchdown frame.
                if credible {
                    ll.td = Some(TouchFrame {
                        fpm: ll.vs_smooth,
                        cas: t.cas_kts,
                        lateral: t.pos.x as f64,
                        along: runway_along(t.heading_deg as f32, t.pos.z) as f64,
                    });
                    ll.td_at = Some(now);
                    ll.air_gap = 0.0;
                    ll.phase = Phase::Rollout;
                }
            }
        }
        Phase::Rollout => {
            if contact {
                // A hop past the flicker bar inside the bounce window is a
                // bounce; a longer absence is a touch-and-go and the
                // re-contact becomes the graded touchdown.
                if ll.td.is_some() && ll.air_gap > BOUNCE_MIN_GAP {
                    if now - ll.td_at.expect("td implies its stamp") < BOUNCE_WINDOW {
                        ll.bounces += 1;
                    } else {
                        ll.td = Some(TouchFrame {
                            fpm: ll.vs_smooth,
                            cas: t.cas_kts,
                            lateral: t.pos.x as f64,
                            along: runway_along(t.heading_deg as f32, t.pos.z) as f64,
                        });
                        ll.td_at = Some(now);
                        ll.bounces = 0;
                        // The rollout's old sums die with the old touchdown.
                        ll.roll_sum = 0.0;
                        ll.roll_n = 0;
                        ll.stop_secs = 0.0;
                    }
                }
                ll.air_gap = 0.0;
            } else if ll.fail.is_none() {
                // The rollout sampler and the stop gate.
                if t.on_ground && t.gs_kts > ROLLING_GS && t.on_runway {
                    ll.roll_sum += (t.pos.x as f64).abs() as f32;
                    ll.roll_n += 1;
                }
                if t.on_ground && t.gs_kts < STOP_GS {
                    ll.stop_secs += dt;
                    if ll.stop_secs >= STOP_SECS {
                        ll.done = Some(finals(ll));
                        ll.settled = true;
                        ll.phase = Phase::Graded;
                    }
                } else {
                    ll.stop_secs = 0.0;
                }
            }
        }
        Phase::Graded => {}
    }
    ll.was_ground = t.on_ground;
    ll.live = compose_live(ll, &t);
}

/// The stabilized rule at the 500 ft gate: speed inside the band, on the
/// line, and sinking no harder than the sink budget.
fn unstable(t: &Telem) -> bool {
    (t.cas_kts < GATE_MIN_V || t.cas_kts > GATE_MAX_V)
        || t.pos.x.abs() > GATE_MAX_OFF
        || (-t.vs_fpm > GATE_MAX_SINK)
}

/// The approach averages and contact frame frozen into one attempt's finals:
/// every row scored 0–10 (see `scoring`), the final their weighted average,
/// minus the flat bounce price.
fn finals(ll: &LandingLesson) -> Finals {
    let td = ll.td.expect("the stop gate implies a touchdown");
    let gs_dev = if ll.gs_n > 0 {
        ll.gs_dev_sum / ll.gs_n as f32
    } else {
        0.0
    };
    let track_dev = if ll.track_n > 0 {
        ll.track_sum / ll.track_n as f32
    } else {
        0.0
    };
    let roll_dev = if ll.roll_n > 0 {
        ll.roll_sum / ll.roll_n as f32
    } else {
        0.0
    };
    let gs = score(gs_dev, 0.8, 3.0);
    let track = score(track_dev, 1.5, 6.0);
    // Sink rate: contacts gentler than the pivot grease; the cap bottoms
    // out at 700 fpm, where the stabilized gate itself fails the approach.
    let rate = score(((td.fpm.abs() - RATE_PIVOT).max(0.0)) as f32, 0.0, RATE_CAP);
    let speed = score((td.cas - V_TOUCH).abs() as f32, 0.0, SPEED_CAP);
    let point = score((td.along - POINT_M).abs() as f32, POINT_WINDOW, POINT_CAP);
    let centre = score(td.lateral.abs() as f32, 0.0, CENTRE_CAP);
    let rollout = score(roll_dev, 1.5, 8.0);
    let final_ = (W_RATE * rate
        + W_SPEED * speed
        + W_GS * gs
        + W_TRACK * track
        + W_POINT * point
        + W_CENTRE * centre
        + W_ROLLOUT * rollout
        - BOUNCE_PENALTY * ll.bounces as f32)
        .max(0.0);
    Finals {
        gs_dev,
        gs,
        track_dev,
        track,
        rate_fpm: td.fpm,
        rate,
        cas: td.cas,
        speed,
        along: td.along,
        point,
        lateral: td.lateral,
        centre,
        roll_dev,
        rollout,
        bounces: ll.bounces,
        final_,
    }
}

/// The quiet live prompt (no banners — the flare needs eyes, not words):
/// the approach row, a brake reminder on the rollout, silence at the stop.
fn compose_live(ll: &LandingLesson, t: &Telem) -> Option<String> {
    if ll.settled || ll.done.is_some() || ll.fail.is_some() {
        return None;
    }
    match ll.phase {
        Phase::Approach => {
            if t.on_ground {
                return None;
            }
            Some(format!(
                "FINAL RWY 36\n{:+.0} kt   {:+.0} fpm   Δ {:+.1} m",
                t.cas_kts, t.vs_fpm, t.pos.x
            ))
        }
        Phase::Rollout => {
            if !t.on_ground || t.gs_kts < STOP_GS {
                return None;
            }
            Some(format!(
                "ROLLOUT\nΔ {:+.1} m   {:+.0} kt   BRAKE",
                t.pos.x, t.gs_kts
            ))
        }
        Phase::Graded => None,
    }
}

/// The update system: only the landing lesson, only the home field.
fn step_m3(
    aircraft: Res<AircraftState>,
    map: Res<crate::map::ActiveMap>,
    lesson: Res<ActiveLesson>,
    time: Res<Time>,
    mut ll: ResMut<LandingLesson>,
) {
    if map.0.is_some() || lesson.0 != Some(Lesson::Landing) {
        return;
    }
    tick(
        &mut ll,
        Telem {
            on_ground: aircraft.on_ground,
            gs_kts: aircraft.gs_kts,
            cas_kts: aircraft.cas_kts,
            agl_ft: aircraft.alt_ft,
            vs_fpm: aircraft.vs_fpm,
            heading_deg: aircraft.heading_deg,
            pos: aircraft.pos,
            on_runway: on_runway(aircraft.pos.x, aircraft.pos.z),
        },
        time.elapsed_secs(),
        time.delta_secs(),
    );
}

pub struct LandingLessonPlugin;

impl Plugin for LandingLessonPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<LandingLesson>()
            .add_systems(OnEnter(AppState::Flying), reset_landing_lesson)
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
fn reset_landing_lesson(mut ll: ResMut<LandingLesson>) {
    ll.rearm();
}

// --- tests -------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use bevy::prelude::*;

    use super::{tick, FAIL_UNSTABLE, LandingLesson, Telem, POINT_M};

    const DT: f32 = 1.0 / 60.0;
    const NOW: f32 = 100.0;

    /// A frame's telemetry: `z` world metres (the spawn seat is 3704 m south
    /// of the threshold), `agl` feet AGL, VS in fpm. The runway 36 line-up.
    fn telem(x: f32, z: f32, agl: f64, gs: f64, cas: f64, vs: f64, ground: bool) -> Telem {
        Telem {
            on_ground: ground,
            gs_kts: gs,
            cas_kts: cas,
            agl_ft: agl,
            vs_fpm: vs,
            heading_deg: 0.0,
            pos: Vec3::new(x, (agl * 0.3048) as f32, z),
            on_runway: x.abs() <= 15.0 && (-1061.0..141.0).contains(&z),
        }
    }

    fn tick_at(ll: &mut LandingLesson, t: Telem, secs: f32) {
        tick(ll, t, NOW + secs, DT);
    }

    /// A stable book approach: on the 3° path the whole way down (the agl
    /// tracks the distance to the threshold), on speed, centred 1 m. Frame
    /// steps decay asymptotically toward the threshold (70 ticks ≈ 1.2 s of
    /// credible air time, the touchdown edge's own bar).
    fn book_approach(ll: &mut LandingLesson, frames: usize, start: f32) {
        for i in 0..frames {
            let dist = 3704.0 * 0.985_f64.powi(i as i32);
            let agl_m = dist * 3f64.to_radians().tan();
            tick_at(
                ll,
                telem(
                    1.0,
                    (140.0 + dist) as f32,
                    agl_m / 0.3048,
                    65.0,
                    65.0,
                    -350.0,
                    false,
                ),
                start + i as f32 * 0.5,
            );
        }
    }

    /// The flare: sink easing low while the last metres burn off (inside
    /// 150 m, so the approach sampler is out of the picture), then a kiss
    /// at the aiming mark.
    fn book_touch(ll: &mut LandingLesson, at: f32) {
        for i in 0..10 {
            let dist = 120.0 - 9.0 * i as f64;
            tick_at(
                ll,
                telem(0.5, (140.0 + dist) as f32, 8.0, 65.0, 63.0, -110.0, false),
                at + i as f32 * 0.15,
            );
        }
        tick_at(ll, telem(0.5, -160.0, 0.0, 65.0, 63.0, -110.0, true), at + 1.5);
    }

    /// Roll out down the centreline decelerating to a stop, then hold the
    /// stop gate's 3 seconds on the wheels.
    fn book_rollout(ll: &mut LandingLesson, at: f32) {
        for i in 0..5 {
            tick_at(
                ll,
                telem(0.5, -160.0, 0.0, 55.0 - i as f64 * 12.0, 55.0, 0.0, true),
                at + i as f32,
            );
        }
        for i in 0..200 {
            tick_at(ll, telem(0.5, -160.0, 0.0, 0.0, 0.0, 0.0, true), at + 5.0 + i as f32 * DT);
        }
    }

    #[test]
    fn a_book_landing_keeps_the_weighted_final_high() {
        let mut ll = LandingLesson::default();
        // The standing prompt while the final runs.
        book_approach(&mut ll, 70, 0.0);
        assert_eq!(ll.done, None, "still flying");
        assert!(!ll.settled, "no card yet");
        let live = ll.live.clone().expect("the approach prompt");
        assert!(live.contains("FINAL RWY 36"), "{live}");
        assert!(live.contains("Δ"), "{live}");

        book_touch(&mut ll, 6.0);
        assert!(ll.done.is_none(), "the card waits for the stop");
        let live = ll.live.clone().expect("the rollout prompt");
        assert!(live.contains("ROLLOUT"), "{live}");

        book_rollout(&mut ll, 8.0);
        let f = ll.done.expect("the stop froze the finals");
        assert!(ll.settled, "the card owns the display at the stop");
        assert_eq!(f.gs, 10.0, "on the path the whole way");
        assert_eq!(f.track, 10.0, "1 m off the line throughout");
        assert!(f.speed > 9.0, "63 kt at the book's 65: {}", f.speed);
        assert!((f.along - POINT_M).abs() < 1.0, "the aiming mark");
        assert!(f.point > 9.5, "on the mark: {}", f.point);
        assert!(f.final_ > 8.5, "a textbook landing: {f:?}");
        let text = ll.finals_text().expect("the card");
        assert!(!text.contains("BOUNCE"), "a clean arrival: {text}");
        assert!(text.contains("FINAL"), "{text}");
    }

    #[test]
    fn landing_long_eats_the_point_row_without_zeroing_the_card() {
        // Touch down past the point cap: 1000 m along, 700 m past the mark.
        let long = {
            let mut ll = LandingLesson::default();
            book_approach(&mut ll, 70, 0.0);
            tick_at(
                &mut ll,
                telem(0.5, -560.0, 0.0, 65.0, 65.0, -110.0, true),
                8.0,
            );
            book_rollout(&mut ll, 9.0);
            ll.done.expect("graded")
        };
        assert_eq!(long.point, 0.0, "700 m past the mark under the cap");
        assert!(long.final_ > 7.0, "the clean rows hold it up: {long:?}");
    }

    #[test]
    fn a_bounce_costs_flat_but_never_zeroes() {
        let clean = {
            let mut ll = LandingLesson::default();
            book_approach(&mut ll, 70, 0.0);
            book_touch(&mut ll, 6.0);
            book_rollout(&mut ll, 8.0);
            ll.done.expect("graded")
        };
        // A hop back up for half a second, then down again: a bounce.
        let bounced = {
            let mut ll = LandingLesson::default();
            book_approach(&mut ll, 70, 0.0);
            book_touch(&mut ll, 6.0);
            for i in 0..35 {
                tick_at(
                    &mut ll,
                    telem(0.5, -165.0, 1.0, 65.0, 65.0, 0.0, false),
                    7.6 + i as f32 * DT,
                );
            }
            book_rollout(&mut ll, 9.0);
            ll.done.expect("graded")
        };
        assert_eq!(bounced.bounces, 1, "the hop counted");
        assert_eq!(
            clean.rate, bounced.rate,
            "the graded frame is the first contact"
        );
        assert!(
            (clean.final_ - bounced.final_ - super::BOUNCE_PENALTY).abs() < 0.01,
            "the flat price: clean {} bounced {}",
            clean.final_,
            bounced.final_
        );
        assert!(bounced.final_ > 0.0, "a bounce pulls, never zeroes");
        // The card carries the bounce line.
        let mut ll = LandingLesson::default();
        ll.done = Some(bounced);
        ll.settled = true;
        let text = ll.finals_text().expect("the card");
        assert!(text.contains("BOUNCE ×1"), "{text}");
    }

    #[test]
    fn an_unstable_crossing_of_the_gate_fails_the_attempt() {
        let mut ll = LandingLesson::default();
        // Slow (45 kt) through 500 ft: past the band's floor.
        tick_at(&mut ll, telem(1.0, 3844.0, 620.0, 65.0, 65.0, -350.0, false), 0.0);
        tick_at(&mut ll, telem(1.0, 3844.0, 480.0, 65.0, 45.0, -350.0, false), 1.0);
        if super::GATE_ON {
            assert_eq!(ll.fail, Some(FAIL_UNSTABLE));
            assert_eq!(ll.done, None, "a failed approach grades nothing");
            assert_eq!(ll.live, None, "the popup owns the display");
            // The gate fires once: staying slow below it doesn't rewrite words.
            tick_at(&mut ll, telem(1.0, 3844.0, 460.0, 65.0, 45.0, -350.0, false), 2.0);
            assert_eq!(ll.fail, Some(FAIL_UNSTABLE));
        } else {
            // Disarmed: the same approach just... lands slow. The flag owns
            // the swap, one line to re-arm the classroom.
            assert_eq!(ll.fail, None, "the gate is off");
            assert_eq!(ll.live.is_some(), true, "the prompt still runs");
        }
    }

    #[test]
    fn a_lateral_drift_on_the_rollout_fades_only_that_row() {
        let mut ll = LandingLesson::default();
        book_approach(&mut ll, 70, 0.0);
        book_touch(&mut ll, 6.0);
        // Drift 4 m off the line while the brakes bite.
        for i in 0..5 {
            tick_at(
                &mut ll,
                telem(4.0, -160.0, 0.0, 55.0 - i as f64 * 12.0, 55.0, 0.0, true),
                8.0 + i as f32,
            );
        }
        for i in 0..200 {
            tick_at(&mut ll, telem(4.0, -160.0, 0.0, 0.0, 0.0, 0.0, true), 13.0 + i as f32 * DT);
        }
        let f = ll.done.expect("graded");
        assert!(
            f.rollout < 7.0,
            "a 4 m rollout average is wide: {}",
            f.rollout
        );
        assert!(
            f.final_ > 8.5,
            "the rollout pulls, the clean rows hold: {f:?}"
        );
    }

    #[test]
    fn a_touch_and_go_recontact_beyond_the_window_replaces_the_frame() {
        let mut ll = LandingLesson::default();
        book_approach(&mut ll, 70, 0.0);
        book_touch(&mut ll, 6.0);
        // Fly away for two seconds, then come back and land again.
        for i in 0..130 {
            tick_at(
                &mut ll,
                telem(1.0, -200.0 - i as f32, 30.0, 65.0, 65.0, 0.0, false),
                7.6 + i as f32 * 0.0167,
            );
        }
        tick_at(&mut ll, telem(0.5, -160.0, 0.0, 65.0, 65.0, -110.0, true), 10.5);
        for i in 0..200 {
            tick_at(&mut ll, telem(0.5, -160.0, 0.0, 0.0, 0.0, 0.0, true), 11.0 + i as f32 * DT);
        }
        let f = ll.done.expect("graded");
        assert_eq!(f.bounces, 0, "the re-contact is a fresh touchdown");
        assert!((f.along - POINT_M).abs() < 1.0, "the fresh frame graded");
    }

    #[test]
    fn a_failed_attempt_rearms_for_the_next_flight() {
        let mut ll = LandingLesson::default();
        book_approach(&mut ll, 70, 0.0);
        book_touch(&mut ll, 6.0);
        book_rollout(&mut ll, 8.0);
        assert!(ll.done.is_some(), "the stop graded the attempt");
        // CONTINUE consumes the card and the attempt: a fresh state.
        ll.rearm();
        assert_eq!(ll.done, None);
        assert!(!ll.settled, "the fresh attempt hides the popup");
        // The parked frame after the re-arm stays quiet (grounded, Approach).
        tick_at(&mut ll, telem(0.5, -160.0, 0.0, 0.0, 0.0, 0.0, true), 21.0);
        assert_eq!(ll.live, None, "grounded, in the Approach phase: quiet");
        // Back on a final, the prompt is up again.
        tick_at(&mut ll, telem(1.0, 3000.0, 500.0, 70.0, 70.0, -350.0, false), 22.0);
        let live = ll.live.expect("the prompt on a fresh final");
        assert!(live.contains("FINAL RWY 36"), "{live}");
    }
}
