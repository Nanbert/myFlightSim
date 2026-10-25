//! The stability gate: a ferry's-company 1000 ft check the sim grades.
//!
//! Every airline hops in on a stabilized-approach rule: by 1000 ft above the
//! field the configuration, speed and sink must be inside the envelope, or
//! the approach is abandoned. This module watches the 737 descend through
//! 1000 ft AAL (via the Y-see-off style spawn this happens for real, since
//! the runway spawn starts on the ground) and flashes a small banner —
//! `STABLE` or the list of broken criteria — like the landing and takeoff
//! report cards.
//!
//! The criteria are the textbook ones, tuned to this sim's schedule:
//!
//! - gear down (indicator up-lock, `gear_pos` > 0.95);
//! - flaps at the approach setting (15 of the wing's 40);
//! - speed no more than Vref + 25 (`landing::vref`, the fence schedule);
//! - sink inside 1,000 fpm.
//!
//! The gate arms again above 1500 ft, so a go-around gets re-graded on the
//! next crossing instead of carrying the old verdict forever.

use bevy::prelude::*;

use crate::aircraft_profile;
use crate::app_state::{AppState, unpaused};
use crate::map::ActiveMap;
use crate::sim::{AircraftState, Sim};
use crate::ui_theme;

/// Altitude AAL the gate grades at.
const GATE_ALT: f64 = 1000.0;
/// The gate re-arms above this, so a go-around can earn a second look.
const REARM_ALT: f64 = 1500.0;
/// Ground speed above which a descent through the altitude gate is an
/// approach, not the balloon of a touch-and-go or the settle at spawn.
const APPROACH_MIN_GS: f64 = 90.0;
/// Sink budget at the gate, ft/min.
const SINK_BUDGET: f64 = 1_000.0;
/// How long the verdict banner stays up.
const BANNER_SECS: f32 = 8.0;
/// Flap degrees the "approach flaps" criterion wants (15 of 40).
const APPROACH_FLAP_DEG: f64 = 14.0;

/// One criterion: a label and whether the airframe currently meets it.
pub struct GateItem {
    pub label: &'static str,
    pub ok: bool,
}

/// Evaluate the four stability criteria. Kept plain for unit tests.
pub fn criteria(
    gear_pos: f64,
    flap_deg: f64,
    cas: f64,
    vs_fpm: f64,
    flap_norm: f64,
) -> Vec<GateItem> {
    vec![
        GateItem {
            label: "GEAR DOWN",
            ok: gear_pos > 0.95,
        },
        GateItem {
            label: "FLAPS 15",
            ok: flap_deg >= APPROACH_FLAP_DEG,
        },
        GateItem {
            label: "SPEED",
            ok: cas <= crate::landing::vref(flap_norm) + 25.0,
        },
        GateItem {
            label: "SINK",
            ok: vs_fpm.abs() <= SINK_BUDGET,
        },
    ]
}

#[derive(Resource, Default)]
pub struct ApproachState {
    /// Verdict of the most recent crossing, for the hop ledger to read.
    pub stable: Option<bool>,
    armed: bool,
    banner_until: f32,
}

#[derive(Component, Default)]
struct ApproachBanner;

pub struct ApproachPlugin;

impl Plugin for ApproachPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<ApproachState>()
            .add_systems(Startup, spawn_banner)
            .add_systems(OnEnter(AppState::Flying), reset_approach)
            .add_systems(Update, update_approach.run_if(unpaused));
    }
}

fn spawn_banner(mut commands: Commands) {
    ui_theme::float_report::<ApproachBanner>(&mut commands, ui_theme::ReportAnchor::LeftTop(120.0));
}

fn reset_approach(mut state: ResMut<ApproachState>) {
    *state = default();
}

fn update_approach(
    time: Res<Time>,
    sim: Res<Sim>,
    map: Res<ActiveMap>,
    lesson: Res<crate::mission::ActiveLesson>,
    aircraft: Res<AircraftState>,
    mut state: ResMut<ApproachState>,
    mut banner: Query<(&mut Text, &mut Node), With<ApproachBanner>>,
) {
    let Ok((mut text, mut node)) = banner.single_mut() else {
        return;
    };
    // The jet's landing lesson owns its gate: its FAIL card and PM prompt
    // carry the verdict, and the free-flight banner would double them up.
    // The go-around lesson fires its own scripted call on the same crossing,
    // and the automation lesson's descent brushes the same fence.
    if aircraft_profile::normalized(&sim.name) != "737"
        || map.0.is_some()
        || lesson.0 == Some(crate::mission::Lesson::JetLanding)
        || lesson.0 == Some(crate::mission::Lesson::JetGoAround)
        || lesson.0 == Some(crate::mission::Lesson::JetAutomation)
    {
        node.display = Display::None;
        return;
    }

    let now = time.elapsed_secs();

    // Above the re-arm line: again armed. A go-around climbing away and
    // rejoining gets a fresh verdict.
    if !aircraft.on_ground && aircraft.alt_ft > REARM_ALT {
        state.armed = true;
    }

    // The crossing: airborne, descending through the gate with approach speed.
    if state.armed
        && !aircraft.on_ground
        && aircraft.gs_kts > APPROACH_MIN_GS
        && aircraft.alt_ft <= GATE_ALT
    {
        state.armed = false;
        let flap_norm = (aircraft.flap_deg / 40.0).clamp(0.0, 1.0);
        let items = criteria(
            aircraft.gear_pos,
            aircraft.flap_deg,
            aircraft.cas_kts,
            aircraft.vs_fpm,
            flap_norm,
        );
        let stable = items.iter().all(|i| i.ok);
        state.stable = Some(stable);
        let mut out = if stable {
            "1000 FT GATE  STABLE".to_owned()
        } else {
            "1000 FT GATE  UNSTABLE\n".to_owned()
        };
        if !stable {
            for item in items.iter().filter(|i| !i.ok) {
                out.push_str("FAIL  ");
                out.push_str(item.label);
                out.push('\n');
            }
        }
        text.0 = out;
        node.display = Display::Flex;
        state.banner_until = now + BANNER_SECS;
    }

    if now > state.banner_until {
        node.display = Display::None;
    }
}

#[cfg(test)]
mod tests {
    use super::criteria;

    /// The book approach: gear down, flaps 15, on speed, inside the sink
    /// budget. Everything green.
    #[test]
    fn the_gate_passes_the_book_configuration() {
        let items = criteria(1.0, 15.0, 130.0, -600.0, 15.0 / 40.0);
        assert!(items.iter().all(|i| i.ok));
    }

    /// Nothing configured, far too fast, sinking like a stone: all four
    /// criteria named on the banner.
    #[test]
    fn a_naked_fast_plunge_names_every_failure() {
        let items = criteria(0.0, 5.0, 200.0, -1200.0, 5.0 / 40.0);
        assert_eq!(items.iter().filter(|i| !i.ok).count(), 4);
    }

    /// The speed envelope is Vref + 25 on the flap schedule: at flaps 15 the
    /// fence is 135.6, so 160 inside, 170 outside.
    #[test]
    fn the_speed_criterion_rides_the_vref_schedule() {
        let items = criteria(1.0, 15.0, 160.0, -600.0, 15.0 / 40.0);
        assert!(items.iter().all(|i| i.ok));
        let items = criteria(1.0, 15.0, 170.0, -600.0, 15.0 / 40.0);
        assert!(items.iter().any(|i| !i.ok && i.label == "SPEED"));
    }
}
