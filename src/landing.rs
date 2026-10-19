//! Landing scoring for the MYFS airfield (C172 and 737).
//!
//! The FDM already reports weight-on-wheels, airspeed, vertical speed and the
//! world pose. This module watches the `false -> true` transition, records the
//! touchdown metrics, waits two seconds for a possible bounce, then grades the
//! landing and shows a small report card. Grading is per-aircraft: the club
//! 172 wants a slow backtrack of a touchdown at 65 kt, the 737 gets approach
//! numbers (a flap-scheduled Vref and a shallower rate budget). Mission
//! flights are F-22 and have their own terrain.

use bevy::picking::Pickable;
use bevy::prelude::*;
use bevy::text::FontSize;

use crate::aerodrome::{on_runway, runway_along, runway_lateral};
use crate::aircraft_profile;
use crate::app_state::AppState;
use crate::map::ActiveMap;
use crate::sim::{AircraftState, Sim};

#[derive(Clone, Copy, PartialEq, Eq)]
enum Kind {
    C172,
    B737,
}

#[derive(Clone, Copy)]
struct Touchdown {
    fpm: f64,
    cas: f64,
    lateral: f64,
    along: f64,
    bank: f64,
    flap_norm: f64,
    owner: Kind,
    on_runway: bool,
    runway: &'static str,
    at: f32,
}

#[derive(Resource, Default)]
struct LandingState {
    was_ground: bool,
    /// Unbroken seconds of *real* air time (weight-on-wheels false and the
    /// vehicle actually moving over the ground) since leaving the runway. The
    /// speed requirement rejects the spring-settle bounce of a parked aircraft:
    /// at spawn the gear drops, wow reads false for a while, but the aircraft
    /// is not going anywhere.
    air_time: f32,
    /// Seconds airborne since the last touchdown, for bounce counting.
    air_gap: f32,
    touchdown: Option<Touchdown>,
    bounces: u32,
    report_until: f32,
    best: Option<u32>,
    vs_smooth: f64,
}

#[derive(Component)]
struct LandingReport;

pub struct LandingPlugin;

impl Plugin for LandingPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<LandingState>()
            .add_systems(Startup, spawn_report)
            .add_systems(OnEnter(AppState::Flying), reset_landing)
            .add_systems(Update, update_landing.run_if(in_state(AppState::Flying)))
            .add_systems(OnExit(AppState::Flying), hide_report);
    }
}

fn spawn_report(mut commands: Commands) {
    commands.spawn((
        LandingReport,
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
            top: Val::Px(120.0),
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

fn reset_landing(mut state: ResMut<LandingState>) {
    let best = state.best;
    *state = LandingState { best, ..default() };
}

/// A hop shorter than this is the gear settling, not a touch-and-go bounce.
const BOUNCE_MIN_GAP: f32 = 0.25;
/// This much unbroken air time is required before a ground contact counts as a
/// touchdown — anything shorter is the wow flicker of a taxiing or settling
/// aircraft.
const TOUCHDOWN_MIN_AIR: f32 = 1.5;
/// Ground speed above which the vehicle is credibly *flying* rather than
/// sitting (or dropping onto) its gear.
const AIRBORNE_GS: f64 = 10.0;

fn hide_report(mut q: Query<&mut Node, With<LandingReport>>) {
    for mut node in &mut q {
        node.display = Display::None;
    }
}

fn update_landing(
    time: Res<Time>,
    sim: Res<Sim>,
    map: Res<ActiveMap>,
    aircraft: Res<AircraftState>,
    mut state: ResMut<LandingState>,
    mut report: Query<(&mut Text, &mut Node), With<LandingReport>>,
) {
    let Ok((mut text, mut node)) = report.single_mut() else {
        return;
    };
    if !matches!(
        aircraft_profile::normalized(&sim.name),
        "c172p" | "737"
    ) || map.0.is_some()
    {
        node.display = Display::None;
        return;
    }

    let now = time.elapsed_secs();
    // A lightly smoothed vertical speed: the raw FDM value is noisy at the
    // moment of contact.
    state.vs_smooth += (aircraft.vs_fpm - state.vs_smooth) * 0.18;

    let ground = aircraft.on_ground;
    let speed = aircraft.cas_kts;
    let gs = aircraft.gs_kts;
    let dt = time.delta_secs();

    // Touchdown edge first (it reads the air time accumulated *so far*), then
    // accumulate/reset the timers — the crashing order would zero a fresh
    // touchdown's own air time on the contact frame itself.
    if ground && !state.was_ground {
        if let Some(td) = state.touchdown {
            // A hop after the recorded touchdown is a bounce; a longer gap is
            // a touch-and-go and the report is already due.
            if now - td.at < 3.0 && state.air_gap > BOUNCE_MIN_GAP {
                state.bounces += 1;
            }
        } else if state.air_time > TOUCHDOWN_MIN_AIR {
            state.touchdown = Some(Touchdown {
                fpm: state.vs_smooth,
                cas: speed,
                lateral: runway_lateral(aircraft.pos.x) as f64,
                along: runway_along(aircraft.heading_deg as f32, aircraft.pos.z) as f64,
                bank: aircraft.roll_deg,
                flap_norm: (aircraft.flap_deg / 40.0).clamp(0.0, 1.0),
                owner: if aircraft_profile::normalized(&sim.name) == "737" {
                    Kind::B737
                } else {
                    Kind::C172
                },
                on_runway: on_runway(aircraft.pos.x, aircraft.pos.z),
                runway: if aircraft.heading_deg.to_radians().cos() >= 0.0 {
                    "36"
                } else {
                    "18"
                },
                at: now,
            });
            state.bounces = 0;
        }
    }

    // Accumulate air time away from the wheels. The ground-speed requirement
    // keeps a parked (or wind-pitot-reading-stationary) aircraft from scoring.
    if !ground && gs > AIRBORNE_GS {
        state.air_time += dt;
        if state.touchdown.is_some() {
            state.air_gap += dt;
        }
    } else if ground {
        state.air_time = 0.0;
        state.air_gap = 0.0;
    }
    state.was_ground = ground;

    // Give a bounce a chance to happen, then grade the first touchdown.
    if let Some(td) = state.touchdown {
        if now - td.at >= 2.0 {
            let (score, grade) = score(td, state.bounces);
            if state.best.map_or(true, |b| score > b) {
                state.best = Some(score);
            }
            text.0 = format!(
                "LANDING  MYFS RWY {}\n\
                 RATE    {:>5.0} fpm\n\
                 CENTRE  {:>5.1} m\n\
                 SPEED   {:>5.0} kt\n\
                 POINT   {:>5.0} m\n\
                 BANK    {:>5.1} deg\n\
                 BOUNCES {:>5}\n\
                 SCORE   {:>3}  {}\n\
                 BEST    {:>3}",
                td.runway,
                td.fpm,
                td.lateral,
                td.cas,
                td.along,
                td.bank,
                state.bounces,
                score,
                grade,
                state.best.unwrap_or(score),
            );
            node.display = Display::Flex;
            state.report_until = now + 12.0;
            state.touchdown = None;
            state.air_time = 0.0;
            state.air_gap = 0.0;
        }
    }

    if now > state.report_until && state.touchdown.is_none() {
        node.display = Display::None;
    }
}

/// Grade a touchdown. Vertical speed is worth the most, then centreline,
/// speed, and the touchdown point; each bounce costs 10 points. The speed
/// target and the rate budget differ per airframe: the 172 greases on at 65 kt
/// with a 400-fpm budget, the jet flies a flap-scheduled Vref and only half
/// that budget.
fn score(td: Touchdown, bounces: u32) -> (u32, &'static str) {
    if !td.on_runway {
        return (0, "OFF-FIELD");
    }
    let fpm = td.fpm.abs();
    let (speed_target, speed_spread, pivot, divisor) = if td.owner == Kind::B737 {
        (vref(td.flap_norm), 20.0, 100.0, 500.0)
    } else {
        (65.0, 25.0, 80.0, 400.0)
    };
    let vertical = (1.0 - ((fpm - pivot).max(0.0) / divisor)).clamp(0.0, 1.0);
    let centre = (1.0 - td.lateral / 10.0).clamp(0.0, 1.0);
    let speed = (1.0 - (td.cas - speed_target).abs() / speed_spread).clamp(0.0, 1.0);
    let point = (1.0 - (td.along - 300.0).abs() / 300.0).clamp(0.0, 1.0);
    let raw = 100.0 * (0.40 * vertical + 0.30 * centre + 0.20 * speed + 0.10 * point);
    let score = (raw - 10.0 * bounces as f64).round().clamp(0.0, 100.0) as u32;
    let grade = if score >= 90 {
        "GREASE"
    } else if score >= 75 {
        "GOOD"
    } else if score >= 55 {
        "FIRM"
    } else if score >= 30 {
        "HARD"
    } else {
        "CRASH"
    };
    (score, grade)
}

/// Approach reference speed the jet should carry over the fence, scheduled on
/// the flap handle (the PFD's VR line uses the same schedule): full boards
/// touch down under 120, a no-flap approach over the water wants ~145.
fn vref(flap_norm: f64) -> f64 {
    (145.0 - 25.0 * flap_norm).clamp(118.0, 145.0)
}

#[cfg(test)]
mod tests {
    use super::{score, Kind, Touchdown};
    use bevy::prelude::*;

    fn touchdown(fpm: f64, cas: f64, lateral: f64, along: f64) -> Touchdown {
        Touchdown {
            fpm,
            cas,
            lateral,
            along,
            bank: 0.0,
            flap_norm: 0.0,
            owner: Kind::C172,
            on_runway: true,
            runway: "36",
            at: 0.0,
        }
    }

    #[test]
    fn a_gentle_centreline_touchdown_scores_grease() {
        let (score, grade) = score(touchdown(80.0, 65.0, 0.0, 300.0), 0);
        assert_eq!(score, 100);
        assert_eq!(grade, "GREASE");
    }

    #[test]
    fn a_hard_off_centre_touchdown_scores_lower() {
        let (hard, _) = score(touchdown(500.0, 85.0, 8.0, 600.0), 2);
        let (soft, _) = score(touchdown(100.0, 65.0, 1.0, 300.0), 0);
        assert!(hard < soft);
        assert!(hard < 30);
    }

    #[test]
    fn off_field_has_no_score() {
        let mut td = touchdown(80.0, 65.0, 0.0, 300.0);
        td.on_runway = false;
        assert_eq!(score(td, 0), (0, "OFF-FIELD"));
    }

    #[test]
    fn a_jet_greases_on_at_vref_and_a_jet_stiff_arm_penalises() {
        // Flap 40 Vref ≈ 120: an on-speed, on-centreline touchdown scores the
        // same as the 172's textbook one.
        let mut td = touchdown(100.0, 120.0, 0.0, 300.0);
        td.owner = Kind::B737;
        td.flap_norm = 1.0;
        let (score737, grade) = score(td, 0);
        assert_eq!(score737, 100, "textbook jet landing");
        assert_eq!(grade, "GREASE");

        // Fast, deep and off centre: solid deduction spiral.
        let mut bad = touchdown(450.0, 190.0, 6.0, 900.0);
        bad.owner = Kind::B737;
        bad.flap_norm = 0.5;
        let (hard, _) = score(bad, 1);
        assert!(hard < 30, "score {hard}");
    }

    #[test]
    fn bounces_cost_ten_points_each() {
        let clean = score(touchdown(80.0, 65.0, 0.0, 300.0), 0).0;
        let bounced = score(touchdown(80.0, 65.0, 0.0, 300.0), 2).0;
        assert_eq!(clean - bounced, 20);
    }

    #[test]
    fn a_touchdown_produces_a_report_card() {
        use std::time::Duration;

        let mut app = App::new();
        app.insert_resource(Time::<()>::default());
        app.insert_resource(crate::sim::Sim::new("c172p"));
        app.insert_resource(crate::map::ActiveMap::default());
        app.insert_resource(super::LandingState::default());
        app.insert_resource(crate::sim::AircraftState {
            on_ground: true,
            cas_kts: 0.0,
            ..default()
        });
        let report = app
            .world_mut()
            .spawn((super::LandingReport, Text::new(""), Node::default()))
            .id();
        app.add_systems(Update, super::update_landing);

        // Initial parked frame: not a landing.
        app.update();
        // Airborne.
        {
            let mut s = app.world_mut().resource_mut::<crate::sim::AircraftState>();
            s.on_ground = false;
            s.cas_kts = 70.0;
            s.gs_kts = 70.0;
            s.vs_fpm = -100.0;
        }
        // A couple of airborne frames, so the wow flicker gate is satisfied.
        app.update();
        app.world_mut()
            .resource_mut::<Time>()
            .advance_by(Duration::from_secs_f32(2.0));
        app.update();
        // Touchdown at the aiming point, runway 36.
        {
            let mut s = app.world_mut().resource_mut::<crate::sim::AircraftState>();
            s.on_ground = true;
            s.cas_kts = 65.0;
            s.gs_kts = 65.0;
            s.vs_fpm = -100.0;
            s.pos = bevy::math::Vec3::new(0.0, 0.0, -160.0);
            s.heading_deg = 0.0;
        }
        app.update();

        // Wait past the two-second bounce window.
        app.world_mut()
            .resource_mut::<Time>()
            .advance_by(Duration::from_secs_f32(2.1));
        app.update();

        let node = app.world().get::<Node>(report).unwrap();
        assert_eq!(node.display, bevy::prelude::Display::Flex);
        let text = app.world().get::<Text>(report).unwrap();
        assert!(text.0.contains("SCORE"), "{}", text.0);
        assert!(text.0.contains("RWY 36"), "{}", text.0);
    }

    #[test]
    fn a_one_frame_wow_flicker_never_records_a_touchdown() {
        use std::time::Duration;

        let mut app = App::new();
        app.insert_resource(Time::<()>::default());
        app.insert_resource(crate::sim::Sim::new("c172p"));
        app.insert_resource(crate::map::ActiveMap::default());
        app.insert_resource(super::LandingState::default());
        app.insert_resource(crate::sim::AircraftState {
            on_ground: true,
            cas_kts: 0.0,
            ..default()
        });
        let report = app
            .world_mut()
            .spawn((super::LandingReport, Text::new(""), Node::default()))
            .id();
        app.add_systems(Update, super::update_landing);

        app.update();
        // The settling aircraft: one frame of wow=false while rolling, then
        // back on the wheels. Too short to be a landing.
        app.world_mut()
            .resource_mut::<crate::sim::AircraftState>()
            .on_ground = false;
        app.update();
        app.world_mut()
            .resource_mut::<crate::sim::AircraftState>()
            .on_ground = true;
        app.update();
        app.world_mut()
            .resource_mut::<Time>()
            .advance_by(Duration::from_secs_f32(3.0));
        app.update();

        let node = app.world().get::<Node>(report).unwrap();
        assert_eq!(node.display, bevy::prelude::Display::None);
        let text = app.world().get::<Text>(report).unwrap();
        assert!(text.0.is_empty(), "spurious report: {}", text.0);
    }
}
