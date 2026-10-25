//! Lesson flow: the graded lessons (the C172's M1..M3, the 737's M1) get
//! actionable overlays instead of passive meters. Free flight keeps the
//! meters' passive displays; in a lesson this module owns the hold-short
//! prompt and pins two modals over the frozen sim:
//!
//! - **FAIL** — the wheels leave the pavement, or (per lesson) an attempt's
//!   own rule trips: the M1 holding-pair incursion (either airframe), the M2
//!   climb failures, the M3 unstable approach at the 500 ft gate, the M4
//!   continued approach past the go-around call: paused,
//!   with RID/back-to-menu rows.
//! - **The finals cards** — a graded stop (M1, either airframe), the M2
//!   climb gate or the M3 stopped landing pauses the flight under its
//!   lesson's numbers.
//!
//! Every card answers on keyboard rows in the codebase's pause-plate idiom:
//! R retries the lesson (routed via the hangar with the lesson pending, so
//! both flows rebuild through the standard chains), Esc returns to the menu
//! (flight discarded), and Enter continues past a finals card.

use bevy::ecs::system::SystemParam;
use bevy::prelude::*;

use crate::airfield::AirfieldSpec;
use crate::app_state::{AppState, Paused, flying};
use crate::mission::{ActiveLesson, Lesson, PendingLesson};
use crate::sim::AircraftState;
use crate::taxi::HoldShort;
use crate::ui_theme;

/// The NOSE wheel rides ~1.5 m ahead of the belly's centre, so the incursion
/// fires when the nose crosses the solid pair. (The grade waits for a stop.)
const NOSE_LEAD: f32 = 1.5;

/// Which modal card, if any, is pinned over the flight.
#[derive(Resource, Default)]
pub struct FlowCard {
    kind: Option<Card>,
}

enum Card {
    /// The hold-short finals card, popped at a graded stop.
    Grade,
    /// Immediate failure; the reason rides in the popup's title.
    Fail,
}

/// True while a lesson popup pins the flight: the pause plate stands down.
pub fn modal(card: Option<Res<FlowCard>>) -> bool {
    card.map_or(false, |c| c.kind.is_some())
}

#[derive(Component, Default)]
pub(crate) struct FlowPrompt;

pub struct LessonFlowPlugin;

impl Plugin for LessonFlowPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<FlowCard>()
            .add_systems(Startup, spawn_prompt)
            // `after(publish_state)`: during a frozen popup `AircraftState` is
            // never written, so the last write lingers at the failure spot —
            // an unordered watch could read that stale state on the first
            // frame after a respawn and re-arm the very same FAIL.
            .add_systems(
                Update,
                watch.after(crate::sim::publish_state).run_if(flying),
            )
            .add_systems(OnEnter(AppState::Menu), replay_lesson)
            .add_systems(OnExit(AppState::Flying), reset_flow)
            // The centred prompt's lifetime is state-driven: every watcher
            // arrangement early-returns in Menu, so the exit must hide it —
            // otherwise the last leg's card lingers over the hangar.
            .add_systems(OnExit(AppState::Flying), hide_prompt_on_exit);
    }
}

/// A flight ending leaves the modal behind no matter how it ended.
fn reset_flow(mut flow: ResMut<FlowCard>) {
    flow.kind = None;
}

/// A flight ending also hides the live prompt for the same reason.
fn hide_prompt_on_exit(mut prompt: Query<(&mut Text, &mut Node), With<FlowPrompt>>) {
    hide_prompt(&mut prompt);
}

/// RETRY routes through the hangar: the lesson is pending again, and the menu
/// flips straight back into the air (`OnEnter(Flying)` re-runs the spawn
/// chains — FDM seats, meters, checklists — exactly as a menu start does).
fn replay_lesson(pending: Res<PendingLesson>, mut next: ResMut<NextState<AppState>>) {
    if pending.0.is_some() {
        next.set(AppState::Flying);
    }
}

fn spawn_prompt(mut commands: Commands) {
    ui_theme::float_report::<FlowPrompt>(&mut commands, ui_theme::ReportAnchor::Center);
}

/// The reason lines the FAIL card carries.
const FAIL_OFF_PAVEMENT: &str = "FAIL - OFF TAXIWAY\nThe wheels left the pavement.";
const FAIL_INCURSION: &str = "FAIL - RUNWAY INCURSION\nThe nose passed the solid holding pair.";
/// The landing lesson's pavement fail reads like a landing, not a taxi.
const FAIL_OFF_RUNWAY: &str = "FAIL - OFF RUNWAY\nThe wheels settled off the pavement.";

/// The lesson meters `watch` steers, bundled so the watcher stays inside
/// Bevy's system-parameter limit (five `ResMut`s ride as one param).
#[derive(SystemParam)]
pub struct LessonMeters<'w> {
    pub hs: ResMut<'w, HoldShort>,
    pub tl: ResMut<'w, crate::takeoff_lesson::TakeoffLesson>,
    pub ll: ResMut<'w, crate::landing_lesson::LandingLesson>,
    pub jt: ResMut<'w, crate::jet_taxi::JetTaxiMeter>,
    pub jtk: ResMut<'w, crate::jet_takeoff_lesson::JetTakeoffMeter>,
    pub jll: ResMut<'w, crate::jet_landing_lesson::JetLandingMeter>,
    pub jga: ResMut<'w, crate::jet_go_around::JetGoAroundMeter>,
}

pub(crate) fn watch(
    keys: Res<ButtonInput<KeyCode>>,
    aircraft: Res<AircraftState>,
    map: Res<crate::map::ActiveMap>,
    airfield: Res<crate::airfield::Airfield>,
    lesson: Res<ActiveLesson>,
    mut meters: LessonMeters,
    mut flow: ResMut<FlowCard>,
    paused: Option<Res<Paused>>,
    mut commands: Commands,
    mut next: ResMut<NextState<AppState>>,
    mut pending: ResMut<PendingLesson>,
    mut prompt: Query<(&mut Text, &mut Node), With<FlowPrompt>>,
    popup: Query<Entity, With<FlowPopup>>,
) {
    let Some(lesson) = lesson.0 else {
        return;
    };
    // --- modal keys: the pinned card owns R / Enter / Esc.
    if let Some(kind) = flow.kind.as_ref() {
        if keys.just_pressed(KeyCode::KeyR) {
            pending.0 = Some(lesson);
            tear_down(&mut flow, &mut commands, &popup);
            next.set(AppState::Menu);
            return;
        }
        if keys.just_pressed(KeyCode::Escape) || keys.just_pressed(KeyCode::KeyM) {
            tear_down(&mut flow, &mut commands, &popup);
            next.set(AppState::Menu);
            return;
        }
        if matches!(kind, Card::Grade)
            && (keys.just_pressed(KeyCode::Enter) || keys.just_pressed(KeyCode::NumpadEnter))
        {
            // Continue past the grade: unpause, consume the card so the
            // trigger can't re-pin it next frame, and fly on.
            tear_down(&mut flow, &mut commands, &popup);
            match lesson {
                // The takeoff lesson re-arms a fresh attempt; the landing
                // lesson does the same (the card's attempt is consumed, the
                // next touchdown grades again); the ground meter just drops
                // its settled latch (its `done` stays for the debug report).
                Lesson::Takeoff => meters.tl.rearm(),
                Lesson::Landing => meters.ll.rearm(),
                Lesson::JetTaxi => meters.jt.rearm(),
                Lesson::JetTakeoff => meters.jtk.rearm(),
                Lesson::JetLanding => meters.jll.rearm(),
                Lesson::JetGoAround => meters.jga.rearm(),
                _ => meters.hs.settled = false,
            }
        }
        return; // the modal holds the display; the prompt stays hidden
    }
    let in_lesson = map.0.is_none();
    if !in_lesson {
        return;
    }
    if paused.is_some() {
        hide_prompt(&mut prompt);
        return;
    }
    let default_spec = AirfieldSpec::myfs();
    let spec = airfield.0.as_ref().unwrap_or(&default_spec);
    // --- triggers. A popup's despawn is one command flush behind
    // `tear_down`, so a lingering entity means a card is still on screen
    // this frame: never double-open underneath it.
    let no_popup = popup.is_empty();
    let off_pavement = aircraft.on_ground && !spec.on_pavement(aircraft.pos.x, aircraft.pos.z);
    if no_popup && off_pavement {
        let reason = if matches!(lesson, Lesson::Landing) {
            FAIL_OFF_RUNWAY
        } else {
            FAIL_OFF_PAVEMENT
        };
        open_fail(&mut flow, &mut commands, reason);
        return;
    }
    // The ground-lesson FAIL list is the taxi drills' (either airframe), and
    // the same lines re-cross legally once the grade is in.
    let nose_lead = match lesson {
        Lesson::JetTaxi => crate::jet_taxi::NOSE_LEAD_M,
        _ => NOSE_LEAD,
    };
    let graded = match lesson {
        Lesson::Taxi => meters.hs.done.is_some(),
        Lesson::JetTaxi => meters.jt.done.is_some(),
        _ => false,
    };
    if no_popup
        && !graded
        && matches!(lesson, Lesson::Taxi | Lesson::JetTaxi)
        && aircraft.on_ground
        && spec
            .connector_offset(aircraft.pos.x, aircraft.pos.z)
            .is_some()
        && aircraft.pos.x < spec.hold_line_x - nose_lead
    {
        open_fail(&mut flow, &mut commands, FAIL_INCURSION);
        return;
    }
    // --- M2: the roll-and-climb lesson owns the triggers from here on.
    if matches!(lesson, Lesson::Takeoff) {
        if no_popup {
            if let Some(reason) = meters.tl.fail {
                open_fail(&mut flow, &mut commands, reason);
                return;
            }
            if meters.tl.settled && meters.tl.done.is_some() {
                let body = meters.tl.finals_text();
                flow.kind = Some(Card::Grade);
                commands.insert_resource(Paused);
                spawn_popup(&mut commands, "TAKEOFF - GRADED", body, true);
                return;
            }
            show_prompt(&mut prompt, meters.tl.live.clone());
        }
        return;
    }
    // --- M3: the approach-touchdown-rollout lesson owns the triggers here.
    // Its own FAIL is the unstable-at-the-gate rule; the wheels on grass go
    // through the always-on pavement check above.
    if matches!(lesson, Lesson::Landing) {
        if no_popup {
            if let Some(reason) = meters.ll.fail {
                open_fail(&mut flow, &mut commands, reason);
                return;
            }
            if meters.ll.settled && meters.ll.done.is_some() {
                let body = meters.ll.finals_text();
                flow.kind = Some(Card::Grade);
                commands.insert_resource(Paused);
                spawn_popup(&mut commands, "LANDING - GRADED", body, true);
                return;
            }
            show_prompt(&mut prompt, meters.ll.live.clone());
        }
        return;
    }
    // --- the 737's M1: the whole-route meter owns the triggers here. Its
    // graded stop ends the lesson, so the card has no Continue row.
    if matches!(lesson, Lesson::JetTaxi) {
        if no_popup {
            if meters.jt.settled && meters.jt.done.is_some() {
                let body = meters.jt.finals_text();
                flow.kind = Some(Card::Grade);
                commands.insert_resource(Paused);
                spawn_popup(&mut commands, "HOLD SHORT - GRADED", body, false);
                return;
            }
            show_prompt(&mut prompt, meters.jt.live.clone());
        }
        return;
    }
    // --- the 737's M2: the callout-flow takeoff owns the triggers here,
    // like the C172's M2 before it. The handoff freezes the card; Enter
    // continues past it to fly on.
    if matches!(lesson, Lesson::JetTakeoff) {
        if no_popup {
            if let Some(reason) = meters.jtk.fail {
                open_fail(&mut flow, &mut commands, reason);
                return;
            }
            if meters.jtk.settled && meters.jtk.done.is_some() {
                let body = meters.jtk.finals_text();
                flow.kind = Some(Card::Grade);
                commands.insert_resource(Paused);
                spawn_popup(&mut commands, "TAKEOFF - GRADED", body, true);
                return;
            }
            show_prompt(&mut prompt, meters.jtk.live.clone());
        }
        return;
    }
    // --- the 737's M3: the arrival meter owns the triggers here, like the
    // C172's M3 before it. The stop freezes the card; Enter continues.
    if matches!(lesson, Lesson::JetLanding) {
        if no_popup {
            if let Some(reason) = meters.jll.fail {
                open_fail_detail(
                    &mut flow,
                    &mut commands,
                    reason,
                    meters.jll.fail_detail.clone(),
                );
                return;
            }
            if meters.jll.settled && meters.jll.done.is_some() {
                let body = meters.jll.finals_text();
                flow.kind = Some(Card::Grade);
                commands.insert_resource(Paused);
                spawn_popup(&mut commands, "LANDING - GRADED", body, true);
                return;
            }
            show_prompt(&mut prompt, meters.jll.live.clone());
        }
        return;
    }
    // --- the 737's M4: the scripted-go-around meter owns the triggers here,
    // like the M2/M3 pair before it. The capture freezes the card; Enter
    // re-arms and the next descent through the gate opens a fresh attempt.
    if matches!(lesson, Lesson::JetGoAround) {
        if no_popup {
            if let Some(reason) = meters.jga.fail {
                open_fail(&mut flow, &mut commands, reason);
                return;
            }
            if meters.jga.settled && meters.jga.done.is_some() {
                let body = meters.jga.finals_text();
                flow.kind = Some(Card::Grade);
                commands.insert_resource(Paused);
                spawn_popup(&mut commands, "GO-AROUND - GRADED", body, true);
                return;
            }
            show_prompt(&mut prompt, meters.jga.live.clone());
        }
        return;
    }
    // A graded stop in a ground lesson pins the finals card. M1 TAXI ends at
    // the hold short, so the card has no Continue row.
    if no_popup && meters.hs.settled && meters.hs.done.is_some() {
        let body = meters.hs.finals_text();
        flow.kind = Some(Card::Grade);
        commands.insert_resource(Paused);
        spawn_popup(&mut commands, "HOLD SHORT - GRADED", body, false);
        return;
    }
    // --- the live prompt while the flight runs.
    if no_popup {
        let live = meters
            .hs
            .live
            .map(|(dist, off)| format!("HOLD SHORT\nLINE {dist:+.1} m   TAXI Δ {off:+.1} m"));
        show_prompt(&mut prompt, live);
    }
}

fn open_fail(flow: &mut FlowCard, commands: &mut Commands, reason: &'static str) {
    open_fail_detail(flow, commands, reason, None);
}

/// The FAIL popup, optionally with a detail body (the jet landing gate
/// names the criteria that were red).
fn open_fail_detail(
    flow: &mut FlowCard,
    commands: &mut Commands,
    reason: &'static str,
    detail: Option<String>,
) {
    flow.kind = Some(Card::Fail);
    commands.insert_resource(Paused);
    spawn_popup(commands, reason, detail, false);
}

/// Every exit from a modal card scrubs it and the freeze explicitly. The
/// state-hop despawn (`DespawnOnExit`) is only a backstop — the retry's
/// Flying→Menu→Flying double-hop can lose its exit despawn through bevy
/// state's latest-event-only reader, which left the card pinned after a
/// respawn.
fn tear_down(flow: &mut FlowCard, commands: &mut Commands, popup: &Query<Entity, With<FlowPopup>>) {
    flow.kind = None;
    commands.remove_resource::<Paused>();
    for root in popup.iter() {
        commands.entity(root).despawn();
    }
}

fn hide_prompt(prompt: &mut Query<(&mut Text, &mut Node), With<FlowPrompt>>) {
    if let Ok((mut text, mut node)) = prompt.single_mut() {
        text.0 = String::new();
        node.display = Display::None;
    }
}

fn show_prompt(prompt: &mut Query<(&mut Text, &mut Node), With<FlowPrompt>>, live: Option<String>) {
    if let Ok((mut text, mut node)) = prompt.single_mut() {
        if let Some(body) = live {
            text.0 = body;
            node.display = Display::Flex;
        } else {
            text.0 = String::new();
            node.display = Display::None;
        }
    }
}

#[derive(Component, Default)]
pub(crate) struct FlowPopup;

/// The pause-plate look: a dimmed full-screen root, an avionics panel, a big
/// title, optional reason/body, then the keyboard rows.
fn spawn_popup(commands: &mut Commands, title: &str, body: Option<String>, with_continue: bool) {
    let root = commands
        .spawn((
            Node {
                position_type: PositionType::Absolute,
                width: Val::Percent(100.0),
                height: Val::Percent(100.0),
                align_items: AlignItems::Center,
                justify_content: JustifyContent::Center,
                display: Display::Flex,
                ..default()
            },
            // The scene dims under the card, same as the pause plate.
            BackgroundColor(Color::srgba(0.0, 0.0, 0.0, 0.55)),
            // Above the cockpit overlay's instrument windows (z 15-16) and
            // the HUD edge panes (z 20) — the M2 card pops mid-climb and
            // must read over the gauges. Combat/MCP panes (30+) stay above.
            GlobalZIndex(25),
            FlowPopup,
            DespawnOnExit(AppState::Flying),
        ))
        .id();
    let card = commands
        .spawn((
            Node {
                flex_direction: FlexDirection::Column,
                width: Val::Px(640.0),
                padding: UiRect {
                    left: Val::Px(40.0),
                    right: Val::Px(40.0),
                    top: Val::Px(28.0),
                    bottom: Val::Px(24.0),
                },
                border: UiRect::all(Val::Px(1.0)),
                border_radius: BorderRadius::all(Val::Px(10.0)),
                ..default()
            },
            BorderColor::all(ui_theme::accent_soft()),
            BackgroundColor(ui_theme::panel_bg()),
            BoxShadow::new(
                ui_theme::panel_glow(),
                Val::ZERO,
                Val::ZERO,
                Val::Px(18.0),
                Val::Px(56.0),
            ),
            ChildOf(root),
        ))
        .id();
    // Title (and reason) read big first; the meter's own body follows.
    let mut lines: Vec<(&str, f32, bool)> = Vec::new();
    let headline = match title.split_once('\n') {
        Some((head, tail)) => {
            lines.push((tail, 16.0, false));
            head
        }
        None => title,
    };
    lines.insert(0, (headline, 26.0, true));
    let mut node_lines: Vec<(String, f32, bool)> = lines
        .into_iter()
        .map(|(t, s, big)| (t.to_string(), s, big))
        .collect();
    if let Some(body) = body {
        node_lines.push((body, 14.0, false));
    }
    for (text, size, big) in node_lines {
        commands.spawn((
            Node {
                margin: UiRect::top(Val::Px(8.0)),
                ..default()
            },
            Text::new(text),
            TextFont {
                font_size: FontSize::Px(size),
                ..default()
            },
            TextColor(match big {
                true => ui_theme::accent(),
                false => ui_theme::text(),
            }),
            ChildOf(card),
        ));
    }
    // The footer: no fake-button rows (nothing is selected with the cursor
    // keys) — a dim hint line, actions spread left to right, built by the
    // shared `ui_theme::key_footer`.
    let mut rows: Vec<(&str, &str)> = vec![("[R]", "RETRY"), ("[Esc]", "MENU")];
    if with_continue {
        rows.insert(0, ("[Enter]", "CONTINUE"));
    }
    ui_theme::key_footer(commands, card, 20.0, &rows);
}
