//! Takeoff scoring for the 737 and the C172 leaving the MYFS runway.
//!
//! The mirror of `landing.rs`: the module watches the weight-on-wheels
//! `true -> false` transition while the aircraft is already rolling (over
//! 30 kt — anything below that is the spawn settle), records the liftoff
//! frame, then watches the following seconds for the gear to come up. It
//! grades:
//!
//! - **speed** of liftoff against the VR target — the 737 rotates on the
//!   PFD's VR line (`pfd::vr_for_flaps`), the C172 on its book's 55-60 kt
//!   (57 kt centre), i.e. rotate on the numbers and this is full marks;
//! - **pitch** at the liftoff frame — between 5 and 15 degrees, like the
//!   book's rotation attitude; over-rotating also buys a tailstrike-class
//!   flat penalty (the jet past 17 degrees, the Cessna past 15);
//! - **bank** at the liftoff frame, which should be nearly nothing;
//! - **gear time** (737 only) from wheels-off to gear-indicator-up, with
//!   5 s the book's "positive rate, gear up" and 15 s of no gear worth
//!   zero. The Cessna's gear is fixed, so the row shows FIXED and the
//!   weight rebalances onto speed and pitch.
//!
//! The card shows for twelve seconds, same as the landing report, and keeps
//! a best score for the session (persistence arrives with the hop ledger).

use bevy::prelude::*;

use crate::aerodrome::on_runway;
use crate::aircraft_profile;
use crate::app_state::{unpaused, AppState};
use crate::map::ActiveMap;
use crate::sim::{AircraftState, Sim};
use crate::ui_theme;

/// Minutes of sidewind and metars aside, the roll has started above this
/// ground speed (kt). Below it a false wow read is the parked aircraft
/// settling on its gear, not a liftoff.
const ROLL_MIN_GS: f64 = 30.0;

/// Seconds after the liftoff frame within which the gear must come up for
/// the timing grade to stay sensible; beyond this the attempt reads "LATE".
const GEAR_WINDOW: f32 = 10.0;

/// How long the report card stays up after grading.
const REPORT_SECS: f32 = 12.0;

/// One liftoff's numbers, plus when it happened.
#[derive(Clone, Copy)]
struct Liftoff {
    /// Which aircraft class this liftoff belongs to: the grading profile
    /// (speed target, weights, gear row) comes from it.
    kind: Kind,
    /// Calibrated airspeed at the wheels-off frame, kt.
    cas: f64,
    /// Flap-handle position (0..1) at the liftoff frame, to pull the VR line.
    flap_norm: f64,
    /// Pitch attitude at the liftoff frame, degrees.
    pitch: f64,
    /// Bank angle at the liftoff frame, degrees (sign lost).
    bank: f64,
    /// Seconds from wheels-off until the gear indicator read up. `None`
    /// means the window elapsed and the gear was still hanging.
    gear_up_s: Option<f32>,
    on_runway: bool,
    runway: &'static str,
    /// When it happened, for future replay bookkeeping.
    #[allow(dead_code)]
    at: f32,
}

/// The takeoff report's aircraft classes. The jet rotates on the PFD's VR
/// bug and gets a gear-timer row; the Cessna rotates on the book numbers
/// with the gear bolted down.
#[derive(Clone, Copy, PartialEq, Eq)]
enum Kind {
    Jet737,
    C172,
}

/// The VR target a liftoff is graded against, by aircraft class.
fn vr_of(kind: Kind, flap_norm: f64) -> f64 {
    match kind {
        Kind::Jet737 => crate::pfd::vr_for_flaps(flap_norm),
        // The C172's book rotation: 55-60 kt, graded on the 57 kt centre.
        Kind::C172 => 57.0,
    }
}

#[derive(Resource, Default)]
pub struct TakeoffState {
    was_ground: bool,
    lift: Option<Liftoff>,
    /// Seconds accumulated airborne since the liftoff frame, for the gear
    /// timer and the settle delay.
    since_lift: f32,
    report_until: f32,
    best: Option<u32>,
    /// The last graded takeoff score, for the hop ledger to read.
    pub last: Option<u32>,
}

#[derive(Component, Default)]
struct TakeoffReport;

pub struct TakeoffPlugin;

impl Plugin for TakeoffPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<TakeoffState>()
            .add_systems(Startup, spawn_report)
            .add_systems(OnEnter(AppState::Flying), reset_takeoff)
            .add_systems(Update, update_takeoff.run_if(unpaused))
            .add_systems(OnExit(AppState::Flying), hide_report);
    }
}

fn spawn_report(mut commands: Commands) {
    ui_theme::float_report::<TakeoffReport>(&mut commands, ui_theme::ReportAnchor::RightTop(430.0));
}

fn reset_takeoff(mut state: ResMut<TakeoffState>) {
    // A fresh flight keeps nothing but the session best.
    let best = state.best;
    *state = TakeoffState { best, ..default() };
}

fn hide_report(mut q: Query<&mut Node, With<TakeoffReport>>) {
    for mut node in &mut q {
        node.display = Display::None;
    }
}

fn update_takeoff(
    time: Res<Time>,
    sim: Res<Sim>,
    map: Res<ActiveMap>,
    aircraft: Res<AircraftState>,
    mut state: ResMut<TakeoffState>,
    mut report: Query<(&mut Text, &mut Node), With<TakeoffReport>>,
) {
    let Ok((mut text, mut node)) = report.single_mut() else {
        return;
    };
    // The early exits keep the card hidden: no report on a mission map, none
    // for aircraft this module doesn't grade.
    if map.0.is_some() {
        node.display = Display::None;
        return;
    }
    let kind = match aircraft_profile::normalized(&sim.name) {
        "737" => Kind::Jet737,
        "c172p" => Kind::C172,
        _ => {
            node.display = Display::None;
            return;
        }
    };

    let now = time.elapsed_secs();
    let dt = time.delta_secs();
    let ground = aircraft.on_ground;

    // The liftoff edge: wheels leave with the roll already established.
    if !ground && state.was_ground && aircraft.gs_kts > ROLL_MIN_GS && state.lift.is_none() {
        let runway = if aircraft.heading_deg.to_radians().cos() >= 0.0 {
            "36"
        } else {
            "18"
        };
        state.lift = Some(Liftoff {
            kind,
            cas: aircraft.cas_kts,
            flap_norm: (aircraft.flap_deg / 40.0).clamp(0.0, 1.0),
            pitch: aircraft.pitch_deg,
            bank: aircraft.roll_deg.abs(),
            gear_up_s: None,
            on_runway: on_runway(aircraft.pos.x, aircraft.pos.z),
            runway,
            at: now,
        });
        state.since_lift = 0.0;
    }

    if state.lift.is_some() {
        // Work on a copy (Liftoff is Copy) so the score/GS reads don't fight
        // the field updates below.
        let mut lift = state.lift.unwrap();
        state.since_lift += dt;
        // The gear clock runs until the indicator reads up, or the window
        // closes and the attempt is graded as-is.
        if lift.gear_up_s.is_none() && aircraft.gear_pos < 0.05 {
            lift.gear_up_s = Some(state.since_lift);
        }
        // Settle: grade after the window so the gear term had its chance,
        // then free the state for the next takeoff (touch-and-go included).
        if state.since_lift >= GEAR_WINDOW {
            let (score, grade) = score(lift);
            if state.best.is_none_or(|b| score > b) {
                state.best = Some(score);
            }
            state.last = Some(score);
            let vr = vr_of(lift.kind, lift.flap_norm);
            // The jet clocks its gear-up; the Cessna's gear never moves.
            let gear = match lift.kind {
                Kind::Jet737 => lift
                    .gear_up_s
                    .map_or("STILL DOWN".to_owned(), |s| format!("{s:.1} s")),
                Kind::C172 => "FIXED".to_owned(),
            };
            let pitch_note = match lift.kind {
                // Over-rotation risks the tail (jet) or an inadvertent stall
                // off the roll (Cessna): a flat hit on top of the pitch grade.
                Kind::Jet737 if lift.pitch > 17.0 => "  TAILSTRIKE",
                Kind::C172 if lift.pitch > 15.0 => "  NOSE HIGH",
                _ => "",
            };
            text.0 = format!(
                "TAKEOFF  MYFS RWY {}\n\
                 LIFT    {:>5.0} kt  VR {:>3.0}\n\
                 PITCH   {:>5.1} deg{}\n\
                 BANK    {:>5.1} deg\n\
                 GEAR    {:>10}\n\
                 SCORE   {:>3}  {}\n\
                 BEST    {:>3}",
                lift.runway,
                lift.cas,
                vr,
                lift.pitch,
                pitch_note,
                lift.bank,
                gear,
                score,
                grade,
                state.best.unwrap_or(score),
            );
            node.display = Display::Flex;
            state.report_until = now + REPORT_SECS;
            state.since_lift = 0.0;
        } else {
            state.lift = Some(lift);
        }
    }

    // Back on the ground with nothing in flight: forget the stale timer so a
    // touchdown-and-roll doesn't feed the next liftoff's gear clock.
    if ground {
        state.since_lift = 0.0;
    }
    state.was_ground = ground;

    if now > state.report_until && state.lift.is_none() {
        node.display = Display::None;
    }
}

/// Grade a liftoff. Speed against the VR target is worth the most, then pitch
/// and bank. The 737 also carries a gear timing term; the C172's fixed gear
/// hands that weight back to speed and pitch.
fn score(lift: Liftoff) -> (u32, &'static str) {
    if !lift.on_runway {
        return (0, "OFF-FIELD");
    }
    let vr = vr_of(lift.kind, lift.flap_norm);
    // The Cessna's band is tighter: its 57 kt target sits on a shallow
    // gradient, so a 15 kt-off liftoff reads nearly as bad as the jet's.
    let speed_tol = match lift.kind {
        Kind::Jet737 => 25.0,
        Kind::C172 => 15.0,
    };
    let speed = (1.0 - (lift.cas - vr).abs() / speed_tol).clamp(0.0, 1.0);
    // The book's rotation attitude: 5-15 degrees, dead centre 10 (jet) or 8
    // (Cessna, which hauls itself off a touch flatter).
    let pitch_centre = match lift.kind {
        Kind::Jet737 => 10.0,
        Kind::C172 => 8.0,
    };
    let pitch = (1.0 - (lift.pitch - pitch_centre).abs().max(0.0) / 12.0).clamp(0.0, 1.0);
    let bank = (1.0 - lift.bank / 10.0).clamp(0.0, 1.0);
    let (w_speed, w_gear, w_pitch, w_bank) = match lift.kind {
        // Jet: the gear clock is the second term (see the module header).
        Kind::Jet737 => (0.35, 0.25, 0.25, 0.15),
        // Cessna: fixed gear, so speed and pitch split the missing weight.
        Kind::C172 => (0.45, 0.0, 0.40, 0.15),
    };
    let gear = match lift.gear_up_s {
        Some(s) => (1.0 - ((s as f64) - 5.0).max(0.0) / 10.0).clamp(0.0, 1.0),
        None => 0.0,
    };
    let raw = 100.0 * (w_speed * speed + w_gear * gear + w_pitch * pitch + w_bank * bank);
    let mut score = raw.round().clamp(0.0, 100.0) as u32;
    // Over-rotation at the liftoff frame risks the tail (jet past 17 degrees)
    // or a drooping wing off the roll (Cessna past 15): a flat hit on top of
    // the pitch grade.
    match lift.kind {
        Kind::Jet737 if lift.pitch > 17.0 => score = score.saturating_sub(15),
        Kind::C172 if lift.pitch > 15.0 => score = score.saturating_sub(10),
        _ => {}
    }
    let grade = if score >= 90 {
        "TEXTBOOK"
    } else if score >= 75 {
        "GOOD"
    } else if score >= 55 {
        "HEAVY"
    } else {
        "SLOPPY"
    };
    (score, grade)
}

#[cfg(test)]
mod tests {
    use super::{score, Kind, Liftoff};

    fn liftoff(kind: Kind, cas: f64, pitch: f64) -> Liftoff {
        Liftoff {
            kind,
            cas,
            flap_norm: 0.125,
            pitch,
            bank: 0.0,
            gear_up_s: Some(3.0),
            on_runway: true,
            runway: "36",
            at: 0.0,
        }
    }

    fn jet(cas: f64, pitch: f64) -> Liftoff {
        liftoff(Kind::Jet737, cas, pitch)
    }

    fn c172(cas: f64, pitch: f64) -> Liftoff {
        Liftoff {
            gear_up_s: None, // the Cessna's gear never moves: the row is FIXED
            ..liftoff(Kind::C172, cas, pitch)
        }
    }

    #[test]
    fn a_book_rotation_scores_textbook() {
        // Flaps 0.125 puts the VR line at 178.75; rotating on the bug at the
        // book attitude with the gear up in 3 s is full marks.
        let (score, grade) = score(jet(179.0, 10.0));
        assert_eq!(score, 100);
        assert_eq!(grade, "TEXTBOOK");
    }

    #[test]
    fn a_c172_book_rotation_scores_textbook_without_any_gear_clock() {
        // 57 kt at the book attitude: the fixed gear neither helps nor hurts.
        let (marks, grade) = score(c172(57.0, 8.0));
        assert_eq!(marks, 100);
        assert_eq!(grade, "TEXTBOOK");
        // 15 kt fast is already the whole band for the Cessna (nearly zero
        // speed marks), while the jet's band still forgives it.
        let (slow_pig, _) = score(c172(72.0, 8.0));
        let (jet_same, _) = score(jet(179.0 + 15.0, 10.0));
        assert!(slow_pig < 60);
        assert!(jet_same > slow_pig);
        // And the nose-high flat penalty bites past 15 degrees of rotation,
        // even at the perfect speed.
        let (rotten, grade_low) = score(c172(57.0, 16.0));
        assert!(rotten < 100);
        assert_eq!(grade_low, "HEAVY");
    }

    #[test]
    fn dragging_off_the_runway_early_costs_speed_and_tailstrike() {
        // Rotate 15 kt slow and haul it off at 19 degrees: speed loss plus
        // the tailstrike flat penalty, and the grade shows it.
        let (early, grade) = score(jet(163.0, 19.0));
        assert!(early < 75);
        assert_eq!(grade, "SLOPPY");
        // Without the over-rotation the same speed band is merely middling.
        let (level, _) = score(jet(163.0, 10.0));
        assert!(level > early);
    }

    #[test]
    fn a_late_or_missing_gear_timer_sinks_the_card() {
        let (slow, _) = score(Liftoff {
            gear_up_s: Some(9.0),
            ..jet(179.0, 10.0)
        });
        let (stuck, _) = score(Liftoff {
            gear_up_s: None,
            ..jet(179.0, 10.0)
        });
        let (prompt, _) = score(jet(179.0, 10.0));
        assert!(stuck < slow && slow < prompt);
    }

    #[test]
    fn sideways_liftoffs_and_off_field_leave_early() {
        let (banked, _) = score(Liftoff {
            bank: 9.0,
            ..jet(179.0, 10.0)
        });
        assert!(banked < 100);
        let mut off = jet(179.0, 10.0);
        off.on_runway = false;
        assert_eq!(score(off), (0, "OFF-FIELD"));
    }
}
