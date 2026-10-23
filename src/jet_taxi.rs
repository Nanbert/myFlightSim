//! The 737's M1 TAXI meter: the whole stand-to-hold-short leg graded as one
//! jet taxi — centreline keeping, the speed band, thrust discipline, and the
//! one-shot stop at the holding pair.
//!
//! The C172's M1 grades two windows (the lead-in corridor and the hold-short
//! approach). The jet's lesson is the long route: lead-in out of the ZSNM
//! stand, 1.6 km down the parallel, the turn onto the runway-36 threshold
//! stub, and a deliberate stop before the solid pair. What makes it a *jet*
//! drill is the mass: 60 tonnes of momentum that must be planned ahead, so
//! the meter samples continuously instead of windowing:
//!
//! - **CENTRELINE** — average |offset| from the painted line over the whole
//!   route (`AirfieldSpec::lesson_corridor`), the turn junctions excluded.
//! - **SPEED BAND** — the fraction of rolling time inside the band: 20 kt on
//!   the parallel's straights, 10 kt everywhere else (apron, stubs, turns).
//! - **THRUST** — the fraction of established rolling time (above breakaway)
//!   spent off idle: bump the thrust, get her rolling, then idle.
//! - **LINE** — the stop itself: nose distance to the solid holding pair.
//!
//! A 3 s stop on the departure stub freezes the finals; anything else (an
//! en-route pause, a stop on the parallel) is just a pause.

use bevy::prelude::*;

use crate::airfield::{AirfieldSpec, CorridorSeg};
use crate::app_state::{AppState, unpaused};
use crate::mission::{ActiveLesson, Lesson};
use crate::sim::AircraftState;

/// Above this ground speed a frame counts as rolling for the time-based
/// samplers; below [`REST_MAX_GS`] for long enough counts as a stop.
const MOVE_MIN_GS: f32 = 3.0;
const REST_MAX_GS: f32 = 0.5;
/// Seconds parked on the stub before the stop grades — a deliberate stop,
/// not the pause between two creeps.
const REST_SECONDS: f32 = 3.0;
/// Above this ground speed the aircraft is flying, not taxiing: the corridor
/// query goes quiet so a pass over the field smears nothing.
const TAXI_MAX_GS: f32 = 30.0;

/// The speed band: the parallel's clean straights cruise, everything else
/// (apron, stub, the turn junctions) is manoeuvring pace.
const CRUISE_MAX_KT: f32 = 20.0;
const MANOEUVRE_MAX_KT: f32 = 10.0;
/// Within this distance of a leg's junction with the parallel the frame is a
/// turn frame: capped at manoeuvring pace and off the centreline sampler.
const JOINT_M: f32 = 25.0;

/// Throttle above this counts as "riding the thrust" once rolling is
/// established. The turbine is quasi-steady, so the lever *is* the thrust.
const IDLE_CMD: f64 = 0.06;
/// Below this ground speed thrust is break-away, not riding: getting 60
/// tonnes moving takes a bump.
const BREAKAWAY_GS: f32 = 5.0;

/// The stop grades nose-to-line, so the datum-to-nose offset: the 737's
/// reference sits essentially on the nose tip (the model's datum is at the
/// fuselage nose; nose gear 158 in aft of it).
pub(crate) const NOSE_LEAD_M: f32 = 1.0;

/// The centreline row's anchors, in metres of average |offset|. Scaled by the
/// field's connector width over the 6 m reference (ZSNM's 18 m stubs read
/// 1.5x) — a wide junction deserves a wide tolerance.
const CENTER_TIGHT: f32 = 1.0;
const CENTER_CAP: f32 = 8.0;
/// The stop-margin row's anchors, same scaling.
const LINE_TIGHT: f32 = 1.0;
const LINE_CAP: f32 = 8.5;

/// The finals' row weights: the stop pays the most, the centreline next, and
/// the two discipline rows share the rest.
const W_CENTER: f32 = 0.30;
const W_BAND: f32 = 0.20;
const W_THRUST: f32 = 0.10;
const W_LINE: f32 = 0.40;

/// The live HOLD SHORT prompt wakes inside this distance of the pair.
const LIVE_RANGE: f32 = 20.0;

/// Whether a frame on `seg` sits in the junction with the parallel — the arc
/// between two painted lines, where neither offset nor speed is a mark.
fn in_joint(seg: CorridorSeg, x: f32, z: f32, spec: &AirfieldSpec) -> bool {
    match seg {
        // The lead-in's mouth and the stub's mouth both meet the parallel at
        // its centreline x.
        CorridorSeg::LeadIn | CorridorSeg::Stub => (x - spec.taxi_x).abs() < JOINT_M,
        CorridorSeg::Parallel => {
            spec.stand.is_some_and(|s| (z - s.z).abs() < JOINT_M)
                || spec
                    .lesson_stub_z()
                    .is_some_and(|sz| (z - sz).abs() < JOINT_M)
        }
    }
}

/// The frame's speed-band ceiling.
fn speed_cap(corridor: Option<(CorridorSeg, f32)>, x: f32, z: f32, spec: &AirfieldSpec) -> f32 {
    match corridor {
        Some((CorridorSeg::Parallel, _)) if !in_joint(CorridorSeg::Parallel, x, z, spec) => {
            CRUISE_MAX_KT
        }
        _ => MANOEUVRE_MAX_KT,
    }
}

fn seg_label(seg: CorridorSeg) -> &'static str {
    match seg {
        CorridorSeg::LeadIn => "LEAD-IN LINE",
        CorridorSeg::Parallel => "PARALLEL TAXIWAY",
        CorridorSeg::Stub => "DEPARTURE STUB",
    }
}

#[derive(Resource, Default)]
pub struct JetTaxiMeter {
    /// The flow prompt's live text while rolling; `None` while parked (the
    /// metre has nothing to say until the grade or the next rollout).
    pub(crate) live: Option<String>,
    /// Frozen finals after the graded stop on the departure stub.
    pub(crate) done: Option<JetFinals>,
    /// A graded stop is parked: the lesson pins its card on this latch.
    pub(crate) settled: bool,
    samples: u32,
    sum: f32,
    roll_secs: f32,
    over_secs: f32,
    thrust_secs: f32,
    peak_gs: f32,
    rest_for: f32,
    /// Diagnostic heartbeat spacing, `TAXI_DEBUG=1` only.
    dbg_t: f32,
}

/// The 737 taxi finals: four graded rows on the shared 0–10 scorer, the
/// weighted final on top.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct JetFinals {
    /// Average |offset| over the route's clean legs, metres.
    avg: f32,
    /// Fraction of rolling time inside the speed band.
    band_pct: f32,
    /// Fraction of established rolling time spent at idle.
    idle_pct: f32,
    /// Nose distance to the solid pair at the graded stop, metres.
    dist: f32,
    center: f32,
    band: f32,
    thrust: f32,
    line: f32,
    peak: f32,
    score: f32,
}

impl JetTaxiMeter {
    /// Feed one frame's readouts. Pure over its own state, for tests.
    fn step(
        &mut self,
        on_ground: bool,
        gs_kts: f64,
        throttle: f64,
        x: f32,
        z: f32,
        spec: &AirfieldSpec,
        dt: f32,
    ) {
        if self.done.is_some() && self.settled {
            // A graded stop is parked: the finals are frozen and the lesson
            // owns the display.
            self.live = None;
            return;
        }
        let gs = gs_kts as f32;
        self.peak_gs = self.peak_gs.max(gs);
        let corridor = if on_ground && gs < TAXI_MAX_GS {
            spec.lesson_corridor(x, z)
        } else {
            None
        };
        if gs > REST_MAX_GS {
            // Rolling — any taxi pace counts, creeping to the line included.
            self.rest_for = 0.0;
            self.settled = false;
            self.live = self.prompt(gs, corridor, x, z, spec);
            if gs > MOVE_MIN_GS {
                self.roll_secs += dt;
                if gs > speed_cap(corridor, x, z, spec) {
                    self.over_secs += dt;
                }
                if gs > BREAKAWAY_GS && throttle > IDLE_CMD {
                    self.thrust_secs += dt;
                }
                if let Some((seg, off)) = corridor {
                    if !in_joint(seg, x, z, spec) {
                        self.samples += 1;
                        self.sum += off.abs();
                    }
                }
            }
        } else {
            // Parked. Before the first move the prompt is the break-away
            // brief (the checklist panel is suppressed for this lesson);
            // afterwards a stop is just a stop until the stub grades it.
            self.live = (self.roll_secs == 0.0).then(|| {
                "TAXI CHECK\nBUMP THE THRUST (SHIFT) TO BREAK AWAY\nTHEN IDLE - FOLLOW THE YELLOW LINE".to_owned()
            });
            self.rest_for += dt;
            if self.rest_for >= REST_SECONDS && matches!(corridor, Some((CorridorSeg::Stub, _))) {
                self.done = Some(self.grade(x, corridor, spec));
                self.settled = true;
                self.rest_for = 0.0;
            }
        }
    }

    fn prompt(
        &self,
        gs: f32,
        corridor: Option<(CorridorSeg, f32)>,
        x: f32,
        z: f32,
        spec: &AirfieldSpec,
    ) -> Option<String> {
        let cap = speed_cap(corridor, x, z, spec);
        let pace = if gs > cap + 1.0 {
            format!("SLOW <={cap:.0} KT")
        } else {
            format!("{gs:.0} KT (<={cap:.0})")
        };
        match corridor {
            Some((CorridorSeg::Stub, off)) if (x - spec.hold_line_x).abs() <= LIVE_RANGE => {
                Some(format!(
                    "HOLD SHORT\nLINE {:+.1} m   TAXI Δ {off:+.1} m",
                    x - spec.hold_line_x
                ))
            }
            Some((seg, off)) => Some(format!("{}\nΔ {off:+.1} m   {pace}", seg_label(seg))),
            None => Some(format!("REJOIN THE CENTRELINE\n{pace}")),
        }
    }

    fn grade(
        &self,
        x: f32,
        corridor: Option<(CorridorSeg, f32)>,
        spec: &AirfieldSpec,
    ) -> JetFinals {
        let scale = spec.connector_half_depth / 6.0;
        let avg = if self.samples > 0 {
            self.sum / self.samples as f32
        } else {
            // A dead-slow crawl gathers no samples; the parked offset is the
            // centreline grade then.
            corridor.map(|(_, off)| off.abs()).unwrap_or(0.0)
        };
        let center = crate::scoring::score(avg, CENTER_TIGHT * scale, CENTER_CAP * scale);
        let band_pct = if self.roll_secs > 0.0 {
            1.0 - self.over_secs / self.roll_secs
        } else {
            1.0
        };
        let idle_pct = if self.roll_secs > 0.0 {
            1.0 - self.thrust_secs / self.roll_secs
        } else {
            1.0
        };
        let band = (band_pct * 10.0).clamp(0.0, 10.0);
        let thrust = (idle_pct * 10.0).clamp(0.0, 10.0);
        let dist = (x - NOSE_LEAD_M) - spec.hold_line_x;
        let line = if dist >= 0.0 {
            crate::scoring::score(dist, LINE_TIGHT * scale, LINE_CAP * scale)
        } else {
            0.0
        };
        let score = if dist < 0.0 {
            0.0
        } else {
            W_CENTER * center + W_BAND * band + W_THRUST * thrust + W_LINE * line
        };
        JetFinals {
            avg,
            band_pct,
            idle_pct,
            dist,
            center,
            band,
            thrust,
            line,
            peak: self.peak_gs,
            score,
        }
    }

    /// The finals card's body: each graded row a number and a word, the
    /// weighted final on top. `None` until the stop has graded.
    pub fn finals_text(&self) -> Option<String> {
        let f = self.done?;
        let line_word = if f.dist < 0.0 {
            "OVER LINE"
        } else {
            crate::scoring::word(f.line)
        };
        let final_word = if f.dist < 0.0 {
            "OVER"
        } else {
            crate::scoring::word(f.score)
        };
        Some(format!(
            "737 TAXI\n\
             CENTRELINE Δ {:5.1} m   {:4.1} {}\n\
             SPEED BAND  {:3.0}%      {:4.1} {}\n\
             THRUST idle {:3.0}%      {:4.1} {}\n\
             LINE        {:+5.1} m   {:4.1} {}\n\
             PEAK        {:3.0} kt\n\
             FINAL       {:4.1}  {}",
            f.avg,
            f.center,
            crate::scoring::word(f.center),
            f.band_pct * 100.0,
            f.band,
            crate::scoring::word(f.band),
            f.idle_pct * 100.0,
            f.thrust,
            crate::scoring::word(f.thrust),
            f.dist,
            f.line,
            line_word,
            f.peak,
            f.score,
            final_word,
        ))
    }

    /// A fresh attempt from the card: the stats start over.
    pub fn rearm(&mut self) {
        *self = JetTaxiMeter::default();
    }
}

/// `TAXI_DEBUG=1` turns on the meter's 2 Hz heartbeat (see `update`).
fn debug_on() -> bool {
    use std::sync::OnceLock;
    static ON: OnceLock<bool> = OnceLock::new();
    *ON.get_or_init(|| std::env::var("TAXI_DEBUG").is_ok())
}

fn reset(mut meter: ResMut<JetTaxiMeter>) {
    meter.rearm();
}

fn update(
    time: Res<Time>,
    aircraft: Res<AircraftState>,
    lesson: Res<ActiveLesson>,
    airfield: Res<crate::airfield::Airfield>,
    mut meter: ResMut<JetTaxiMeter>,
) {
    if lesson.0 != Some(Lesson::JetTaxi) {
        return;
    }
    let default_spec = AirfieldSpec::zsnm();
    let spec = airfield.0.as_ref().unwrap_or(&default_spec);
    meter.step(
        aircraft.on_ground,
        aircraft.gs_kts,
        aircraft.throttle,
        aircraft.pos.x,
        aircraft.pos.z,
        spec,
        time.delta_secs(),
    );
    // `TAXI_DEBUG=1` prints the meter's verdict inputs twice a second, so a
    // missing prompt can be diagnosed from the terminal alone.
    if debug_on() {
        meter.dbg_t += time.delta_secs();
        if meter.dbg_t >= 0.5 {
            meter.dbg_t = 0.0;
            let seg = spec
                .lesson_corridor(aircraft.pos.x, aircraft.pos.z)
                .map(|(s, off)| format!("{s:?} {off:+.1}"))
                .unwrap_or_else(|| "None".into());
            eprintln!(
                "[jettaxi] {seg} gs={:.2} thr={:.2} cap={:.0} roll={:.1} over={:.1} \
                 thrust={:.1} samples={} rest={:.1} peak={:.1} done={} settled={}",
                aircraft.gs_kts,
                aircraft.throttle,
                speed_cap(
                    spec.lesson_corridor(aircraft.pos.x, aircraft.pos.z),
                    aircraft.pos.x,
                    aircraft.pos.z,
                    spec
                ),
                meter.roll_secs,
                meter.over_secs,
                meter.thrust_secs,
                meter.samples,
                meter.rest_for,
                meter.peak_gs,
                meter.done.is_some(),
                meter.settled,
            );
        }
    }
}

pub struct JetTaxiPlugin;

impl Plugin for JetTaxiPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<JetTaxiMeter>()
            .add_systems(OnEnter(AppState::Flying), reset)
            .add_systems(Update, update.run_if(unpaused));
    }
}

#[cfg(test)]
mod tests {
    use super::JetTaxiMeter;
    use crate::airfield::AirfieldSpec;

    const DT: f32 = 1.0 / 60.0;
    /// Seconds parked that fire the 3 s grade, with margin.
    const PARK: f32 = 3.5;

    fn spec() -> AirfieldSpec {
        AirfieldSpec::zsnm()
    }

    fn roll(m: &mut JetTaxiMeter, secs: f32, gs: f64, thr: f64, x: f32, z: f32) {
        let s = spec();
        let steps = (secs / DT).ceil() as usize;
        for _ in 0..steps {
            m.step(true, gs, thr, x, z, &s, DT);
        }
    }

    /// The textbook run: out of the stand at a walking pace, down the
    /// parallel at 16 kt, the stub at 8, stop 2 m short of the pair.
    fn textbook(m: &mut JetTaxiMeter) {
        roll(m, 6.0, 8.0, 0.02, 200.0, -1501.0); // lead-in, clean
        roll(m, 40.0, 16.0, 0.02, 130.0, -800.0); // parallel, clean
        roll(m, 6.0, 8.0, 0.02, 80.0, 134.0); // stub, clean
        roll(m, PARK, 0.2, 0.0, 40.0, 134.0); // parked ~2 m short
    }

    #[test]
    fn the_stand_spawn_shows_the_break_away_brief() {
        let mut m = JetTaxiMeter::default();
        roll(&mut m, 6.0, 0.0, 0.0, 252.0, -1500.0);
        // Never rolled: no grade, and the parked prompt is the brief.
        assert_eq!(m.done, None);
        let live = m.live.clone().expect("the brief");
        assert!(live.contains("TAXI CHECK"), "{live}");
        assert!(live.contains("BREAK AWAY"), "{live}");
        // Once rolling, the leg prompt takes over.
        roll(&mut m, 1.0, 8.0, 0.30, 240.0, -1500.0);
        let live = m.live.expect("the leg prompt");
        assert!(live.contains("LEAD-IN LINE"), "{live}");
    }

    #[test]
    fn a_textbook_run_scores_well() {
        let mut m = JetTaxiMeter::default();
        textbook(&mut m);
        let f = m.done.expect("the stop on the stub grades");
        assert!(m.settled);
        // The line: dist = 40 - 1 (nose) - 37 = 2 m -> near-full marks.
        assert!((f.dist - 2.0).abs() < 1e-3, "{}", f.dist);
        assert!(f.line > 8.5, "{}", f.line);
        // Clean legs, always in band, always at idle.
        assert!(f.avg < 1.2, "{}", f.avg);
        assert_eq!(f.band, 10.0);
        assert_eq!(f.thrust, 10.0);
        assert!(f.score > 9.0, "{f:?}");
        let text = m.finals_text().expect("the card");
        assert!(text.contains("FINAL"), "{text}");
    }

    #[test]
    fn an_en_route_pause_is_just_a_pause() {
        let mut m = JetTaxiMeter::default();
        roll(&mut m, 10.0, 16.0, 0.02, 130.0, -800.0);
        roll(&mut m, PARK, 0.2, 0.0, 130.0, -800.0);
        assert_eq!(m.done, None, "a stop on the parallel grades nothing");
        assert_eq!(m.live, None);
        // And the route continues: the stats still hold the leg so far.
        roll(&mut m, 6.0, 8.0, 0.02, 80.0, 134.0);
        roll(&mut m, PARK, 0.2, 0.0, 40.0, 134.0);
        assert!(m.done.is_some(), "the stub stop still grades");
    }

    #[test]
    fn overspeed_on_the_parallel_dings_the_band() {
        let mut m = JetTaxiMeter::default();
        // A third of the rolling time at 26 kt on the clean parallel (over
        // the 20 kt band): legs 6 + 12 + 6 s, 12 of them over -> half marks.
        roll(&mut m, 6.0, 16.0, 0.02, 130.0, -800.0);
        roll(&mut m, 12.0, 26.0, 0.02, 130.0, -800.0);
        roll(&mut m, 6.0, 8.0, 0.02, 80.0, 134.0);
        roll(&mut m, PARK, 0.2, 0.0, 40.0, 134.0);
        let f = m.done.expect("graded");
        assert!((f.band - 5.0).abs() < 0.1, "half the time over: {f:?}");
    }

    #[test]
    fn riding_the_thrust_dings_the_discipline() {
        let mut m = JetTaxiMeter::default();
        // 10 s of established rolling with the lever up over 36 s of roll:
        // 10 * (1 - 10/36) = 7.2.
        roll(&mut m, 10.0, 16.0, 0.30, 130.0, -800.0);
        roll(&mut m, 20.0, 16.0, 0.02, 130.0, -800.0);
        roll(&mut m, 6.0, 8.0, 0.02, 80.0, 134.0);
        roll(&mut m, PARK, 0.2, 0.0, 40.0, 134.0);
        let f = m.done.expect("graded");
        assert!((f.thrust - 7.2).abs() < 0.2, "a third riding: {f:?}");
    }

    #[test]
    fn breakaway_thrust_is_not_riding() {
        let mut m = JetTaxiMeter::default();
        // Full thrust below breakaway speed (the bump that gets her moving),
        // then idle once established.
        roll(&mut m, 3.0, 4.0, 0.40, 200.0, -1500.0);
        roll(&mut m, 30.0, 16.0, 0.02, 130.0, -800.0);
        roll(&mut m, 6.0, 8.0, 0.02, 80.0, 134.0);
        roll(&mut m, PARK, 0.2, 0.0, 40.0, 134.0);
        let f = m.done.expect("graded");
        assert_eq!(f.thrust, 10.0, "the breakaway bump is free: {f:?}");
    }

    #[test]
    fn the_turn_junctions_are_neither_cruised_nor_sampled() {
        let s = spec();
        // Rolling through the lead-in's mouth (within 25 m of the parallel):
        // the prompt caps at 10 kt and the frame stays off the sampler.
        let mut m = JetTaxiMeter::default();
        m.step(true, 12.0, 0.02, 140.0, -1500.0, &s, DT);
        assert!(
            m.live.as_deref().unwrap_or("").contains("SLOW"),
            "the junction prompts the slow band: {:?}",
            m.live
        );
        assert_eq!(m.samples, 0, "the junction arc is not a centreline mark");
        // The same speed on the clean parallel is inside the cruise band and
        // on the sampler.
        m.step(true, 12.0, 0.02, 130.0, -800.0, &s, DT);
        assert!(
            !m.live.as_deref().unwrap_or("").contains("SLOW"),
            "the straight parallel is in band at 12 kt: {:?}",
            m.live
        );
        assert_eq!(m.samples, 1);
    }

    #[test]
    fn a_dead_slow_crawl_still_grades() {
        let mut m = JetTaxiMeter::default();
        roll(&mut m, PARK, 0.4, 0.02, 40.0, 135.0);
        let f = m.done.expect("the crawl onto the stub grades");
        assert!(
            (f.avg - 1.0).abs() < 1e-3,
            "parked offset is the grade: {}",
            f.avg
        );
        assert!((f.dist - 2.0).abs() < 1e-3, "{}", f.dist);
    }

    #[test]
    fn crossing_the_pair_collapses_the_final() {
        let mut m = JetTaxiMeter::default();
        roll(&mut m, 6.0, 8.0, 0.02, 80.0, 134.0);
        roll(&mut m, PARK, 0.2, 0.0, 36.0, 134.0);
        let f = m.done.expect("graded");
        assert!(f.dist < 0.0, "{}", f.dist);
        assert_eq!(f.score, 0.0);
        let text = m.finals_text().expect("the card");
        assert!(text.contains("OVER"), "{text}");
    }

    #[test]
    fn the_hold_short_prompt_wakes_near_the_pair() {
        let mut m = JetTaxiMeter::default();
        roll(&mut m, 2.0, 8.0, 0.02, 50.0, 134.0);
        assert!(
            m.live.as_deref().unwrap_or("").contains("HOLD SHORT"),
            "{:?}",
            m.live
        );
        // Wide of the pair the leg prompt carries the frame instead.
        roll(&mut m, 1.0, 8.0, 0.02, 90.0, 134.0);
        assert!(
            m.live.as_deref().unwrap_or("").contains("DEPARTURE STUB"),
            "{:?}",
            m.live
        );
    }
}
