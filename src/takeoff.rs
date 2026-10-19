//! Takeoff scoring for the 737 leaving the MYFS runway.
//!
//! The mirror of `landing.rs`: the module watches the weight-on-wheels
//! `true -> false` transition while the aircraft is already rolling (over
//! 30 kt — anything below that is the spawn settle), records the liftoff
//! frame, then watches the following seconds for the gear to come up. It
//! grades:
//!
//! - **speed** of liftoff against the PFD's VR line (`pfd::vr_for_flaps`),
//!   i.e. rotate on the bug and this is full marks;
//! - **pitch** at the liftoff frame — between 5 and 15 degrees, like the
//!   book's rotation attitude; over-rotating past 17 degrees also buys a
//!   tailstrike-class flat penalty;
//! - **bank** at the liftoff frame, which should be nearly nothing;
//! - **gear time** from wheels-off to gear-indicator-up, with 5 s the
//!   book's "positive rate, gear up" and 15 s of no gear worth zero.
//!
//! The card shows for twelve seconds, same as the landing report, and keeps
//! a best score for the session (persistence arrives with the hop ledger).

use bevy::picking::Pickable;
use bevy::prelude::*;
use bevy::text::FontSize;

use crate::aerodrome::on_runway;
use crate::aircraft_profile;
use crate::app_state::AppState;
use crate::map::ActiveMap;
use crate::sim::{AircraftState, Sim};

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
    at: f32,
}

#[derive(Resource, Default)]
struct TakeoffState {
    was_ground: bool,
    lift: Option<Liftoff>,
    /// Seconds accumulated airborne since the liftoff frame, for the gear
    /// timer and the settle delay.
    since_lift: f32,
    report_until: f32,
    best: Option<u32>,
}

#[derive(Component)]
struct TakeoffReport;

pub struct TakeoffPlugin;

impl Plugin for TakeoffPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<TakeoffState>()
            .add_systems(Startup, spawn_report)
            .add_systems(OnEnter(AppState::Flying), reset_takeoff)
            .add_systems(Update, update_takeoff.run_if(in_state(AppState::Flying)))
            .add_systems(OnExit(AppState::Flying), hide_report);
    }
}

fn spawn_report(mut commands: Commands) {
    commands.spawn((
        TakeoffReport,
        Text::new(""),
        TextFont {
            font_size: FontSize::Px(14.0),
            ..default()
        },
        TextColor(Color::srgb(0.92, 0.94, 0.90)),
        BackgroundColor(Color::srgba(0.02, 0.03, 0.04, 0.80)),
        Node {
            position_type: PositionType::Absolute,
            right: Val::Px(18.0),
            top: Val::Px(430.0),
            padding: UiRect::all(Val::Px(10.0)),
            border: UiRect::all(Val::Px(1.0)),
            display: Display::None,
            ..default()
        },
        BorderColor::all(Color::srgba(0.7, 0.75, 0.7, 0.35)),
        GlobalZIndex(14),
        Pickable::IGNORE,
    ));
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
    if aircraft_profile::normalized(&sim.name) != "737" || map.0.is_some() {
        node.display = Display::None;
        return;
    }

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
            if state.best.map_or(true, |b| score > b) {
                state.best = Some(score);
            }
            let vr = crate::pfd::vr_for_flaps(lift.flap_norm);
            let gear = lift
                .gear_up_s
                .map_or("STILL DOWN".to_owned(), |s| format!("{s:.1} s"));
            let pitch_note = if lift.pitch > 17.0 { "  TAILSTRIKE" } else { "" };
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

/// Grade a liftoff. Speed against the VR line is worth the most, then the
/// gear timing, then pitch and bank.
fn score(lift: Liftoff) -> (u32, &'static str) {
    if !lift.on_runway {
        return (0, "OFF-FIELD");
    }
    let vr = crate::pfd::vr_for_flaps(lift.flap_norm);
    let speed = (1.0 - (lift.cas - vr).abs() / 25.0).clamp(0.0, 1.0);
    // The book's rotation attitude: 5-15 degrees, dead centre 10.
    let pitch = (1.0 - (lift.pitch - 10.0).abs().max(0.0) / 12.0).clamp(0.0, 1.0);
    let bank = (1.0 - lift.bank / 10.0).clamp(0.0, 1.0);
    let gear = match lift.gear_up_s {
        Some(s) => (1.0 - ((s as f64) - 5.0).max(0.0) / 10.0).clamp(0.0, 1.0),
        None => 0.0,
    };
    let raw = 100.0 * (0.35 * speed + 0.25 * gear + 0.25 * pitch + 0.15 * bank);
    let mut score = raw.round().clamp(0.0, 100.0) as u32;
    // Over-rotation past 17 degrees at the liftoff frame risks the tail: a
    // flat strike on top of the pitch grade.
    if lift.pitch > 17.0 {
        score = score.saturating_sub(15);
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
    use super::{score, Liftoff};

    fn liftoff(cas: f64, pitch: f64) -> Liftoff {
        Liftoff {
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

    #[test]
    fn a_book_rotation_scores_textbook() {
        // Flaps 0.125 puts the VR line at 178.75; rotating on the bug at the
        // book attitude with the gear up in 3 s is full marks.
        let (score, grade) = score(liftoff(179.0, 10.0));
        assert_eq!(score, 100);
        assert_eq!(grade, "TEXTBOOK");
    }

    #[test]
    fn dragging_off_the_runway_early_costs_speed_and_tailstrike() {
        // Rotate 15 kt slow and haul it off at 19 degrees: speed loss plus
        // the tailstrike flat penalty, and the grade shows it.
        let (early, grade) = score(liftoff(163.0, 19.0));
        assert!(early < 75);
        assert_eq!(grade, "SLOPPY");
        // Without the over-rotation the same speed band is merely middling.
        let (level, _) = score(liftoff(163.0, 10.0));
        assert!(level > early);
    }

    #[test]
    fn a_late_or_missing_gear_timer_sinks_the_card() {
        let (slow, _) = score(Liftoff {
            gear_up_s: Some(9.0),
            ..liftoff(179.0, 10.0)
        });
        let (stuck, _) = score(Liftoff {
            gear_up_s: None,
            ..liftoff(179.0, 10.0)
        });
        let (prompt, _) = score(liftoff(179.0, 10.0));
        assert!(stuck < slow && slow < prompt);
    }

    #[test]
    fn sideways_liftoffs_and_off_field_leave_early() {
        let (banked, _) = score(Liftoff {
            bank: 9.0,
            ..liftoff(179.0, 10.0)
        });
        assert!(banked < 100);
        let mut off = liftoff(179.0, 10.0);
        off.on_runway = false;
        assert_eq!(score(off), (0, "OFF-FIELD"));
    }
}
