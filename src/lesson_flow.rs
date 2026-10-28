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
    card.is_some_and(|c| c.kind.is_some())
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
            .add_systems(OnExit(AppState::Flying), reset_flow);
    }
}

/// A flight ending leaves the modal behind no matter how it ended.
fn reset_flow(mut flow: ResMut<FlowCard>) {
    flow.kind = None;
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

/// The reason the ground-lesson FAIL card carries when the nose crosses the
/// solid holding pair.
const FAIL_INCURSION: &str = "FAIL - RUNWAY INCURSION\nThe nose passed the solid holding pair.";

/// The lesson meters `watch` steers, bundled so the watcher stays inside
/// Bevy's system-parameter limit (the `ResMut`s ride as one param).
#[derive(SystemParam)]
pub struct LessonMeters<'w> {
    pub hs: ResMut<'w, HoldShort>,
    pub tl: ResMut<'w, crate::takeoff_lesson::TakeoffLesson>,
    pub ll: ResMut<'w, crate::landing_lesson::LandingLesson>,
    pub jt: ResMut<'w, crate::jet_taxi::JetTaxiMeter>,
    pub jtk: ResMut<'w, crate::jet_takeoff_lesson::JetTakeoffMeter>,
    pub jll: ResMut<'w, crate::jet_landing_lesson::JetLandingMeter>,
    pub jga: ResMut<'w, crate::jet_go_around::JetGoAroundMeter>,
    pub jrt: ResMut<'w, crate::jet_rto::JetRtoMeter>,
    pub jau: ResMut<'w, crate::jet_automation::JetAutomationMeter>,
}

/// What `watch` needs from whichever meter is running the active lesson.
/// The watcher used to carry one `if matches!(lesson, ...)` block per drill;
/// now every meter answers the same questions and `LessonMeters::active_mut`
/// hands over the one for the flight, so the triggers live in one place.
pub trait LessonMeter {
    /// The attempt's own FAIL reason, if a rule tripped.
    fn fail(&self) -> Option<&'static str>;
    /// Extra lines for the FAIL card (the jet gates name the red criteria).
    fn fail_detail(&self) -> Option<String> {
        None
    }
    /// A graded attempt is in the books — a taxi may re-cross its hold line.
    fn done(&self) -> bool;
    /// The grade is latched and its card should be pinned.
    fn settled(&self) -> bool;
    /// The grade card's body, `None` before the first grade.
    fn card_body(&self) -> Option<String>;
    /// The live prompt line, `None` to hide the prompt.
    fn live(&self) -> Option<String>;
    /// Clear the latch so the next attempt grades again.
    fn rearm(&mut self);
}

impl LessonMeters<'_> {
    /// The meter running `lesson`. Every lesson has exactly one.
    fn active_mut(&mut self, lesson: Lesson) -> &mut dyn LessonMeter {
        match lesson {
            Lesson::Taxi => &mut *self.hs,
            Lesson::Takeoff => &mut *self.tl,
            Lesson::Landing => &mut *self.ll,
            Lesson::JetTaxi => &mut *self.jt,
            Lesson::JetTakeoff => &mut *self.jtk,
            Lesson::JetLanding => &mut *self.jll,
            Lesson::JetGoAround => &mut *self.jga,
            Lesson::JetRto => &mut *self.jrt,
            Lesson::JetAutomation => &mut *self.jau,
        }
    }
}

impl LessonMeter for HoldShort {
    fn fail(&self) -> Option<&'static str> {
        None
    }
    fn done(&self) -> bool {
        self.done.is_some()
    }
    fn settled(&self) -> bool {
        self.settled
    }
    fn card_body(&self) -> Option<String> {
        self.finals_text()
    }
    fn live(&self) -> Option<String> {
        self.live
            .map(|(dist, off)| format!("HOLD SHORT\nLINE {dist:+.1} m   TAXI Δ {off:+.1} m"))
    }
    fn rearm(&mut self) {
        self.settled = false;
    }
}

/// The graded meters share their field shapes: every one has `done`/`settled`/
/// `live` and an inherent `finals_text`/`rearm`; the fail-capable drills add
/// a `fail`, and the two gate lessons a `fail_detail`. Three shapes cover all
/// eight, so the trait impls are one table entry each instead of eight copies.
macro_rules! meter_impl {
    ($ty:ty, fail: none) => {
        impl LessonMeter for $ty {
            fn fail(&self) -> Option<&'static str> {
                None
            }
            fn done(&self) -> bool {
                self.done.is_some()
            }
            fn settled(&self) -> bool {
                self.settled
            }
            fn card_body(&self) -> Option<String> {
                <$ty>::finals_text(self)
            }
            fn live(&self) -> Option<String> {
                self.live.clone()
            }
            fn rearm(&mut self) {
                <$ty>::rearm(self);
            }
        }
    };
    ($ty:ty, fail: field) => {
        impl LessonMeter for $ty {
            fn fail(&self) -> Option<&'static str> {
                self.fail
            }
            fn done(&self) -> bool {
                self.done.is_some()
            }
            fn settled(&self) -> bool {
                self.settled
            }
            fn card_body(&self) -> Option<String> {
                <$ty>::finals_text(self)
            }
            fn live(&self) -> Option<String> {
                self.live.clone()
            }
            fn rearm(&mut self) {
                <$ty>::rearm(self);
            }
        }
    };
    ($ty:ty, fail: field, detail: field) => {
        impl LessonMeter for $ty {
            fn fail(&self) -> Option<&'static str> {
                self.fail
            }
            fn fail_detail(&self) -> Option<String> {
                self.fail_detail.clone()
            }
            fn done(&self) -> bool {
                self.done.is_some()
            }
            fn settled(&self) -> bool {
                self.settled
            }
            fn card_body(&self) -> Option<String> {
                <$ty>::finals_text(self)
            }
            fn live(&self) -> Option<String> {
                self.live.clone()
            }
            fn rearm(&mut self) {
                <$ty>::rearm(self);
            }
        }
    };
}

meter_impl!(crate::takeoff_lesson::TakeoffLesson, fail: field);
meter_impl!(crate::landing_lesson::LandingLesson, fail: field);
meter_impl!(crate::jet_taxi::JetTaxiMeter, fail: none);
meter_impl!(crate::jet_takeoff_lesson::JetTakeoffMeter, fail: field);
meter_impl!(crate::jet_landing_lesson::JetLandingMeter, fail: field, detail: field);
meter_impl!(crate::jet_go_around::JetGoAroundMeter, fail: field);
meter_impl!(crate::jet_rto::JetRtoMeter, fail: field);
meter_impl!(crate::jet_automation::JetAutomationMeter, fail: field, detail: field);
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
    let spec = lesson.spec();
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
            // trigger can't re-pin it next frame, and fly on. The meter
            // decides what a fresh attempt needs (a re-armed rollout, a
            // dropped settle latch).
            tear_down(&mut flow, &mut commands, &popup);
            meters.active_mut(lesson).rearm();
        }
        return; // the modal holds the display; the prompt stays hidden
    }
    // Lessons fly over the airfield; a map mission has its own rules.
    if map.0.is_some() {
        return;
    }
    if paused.is_some() {
        hide_prompt(&mut prompt);
        return;
    }
    let default_spec = AirfieldSpec::myfs();
    let af = airfield.0.as_ref().unwrap_or(&default_spec);
    // --- triggers. A popup's despawn is one command flush behind
    // `tear_down`, so a lingering entity means a card is still on screen
    // this frame: never double-open underneath it.
    let no_popup = popup.is_empty();
    if no_popup && aircraft.on_ground && !af.on_pavement(aircraft.pos.x, aircraft.pos.z) {
        open_fail(&mut flow, &mut commands, spec.off_pavement);
        return;
    }
    let meter = meters.active_mut(lesson);
    // The ground-lesson FAIL list is the taxi drills' (either airframe), and
    // the same lines re-cross legally once the grade is in.
    if no_popup
        && !meter.done()
        && matches!(lesson, Lesson::Taxi | Lesson::JetTaxi)
        && aircraft.on_ground
        && af
            .connector_offset(aircraft.pos.x, aircraft.pos.z)
            .is_some()
        && aircraft.pos.x < af.hold_line_x - spec.nose_lead_m
    {
        open_fail(&mut flow, &mut commands, FAIL_INCURSION);
        return;
    }
    // The meter's own triggers. Every lesson answers through the same four
    // reads, so this block no longer knows which aircraft or drill is flying:
    // a FAIL parks the attempt, a settled grade pins the finals card,
    // otherwise the live prompt runs.
    if no_popup {
        if let Some(reason) = meter.fail() {
            open_fail_detail(&mut flow, &mut commands, reason, meter.fail_detail());
            return;
        }
        if meter.settled() {
            if let Some(body) = meter.card_body() {
                flow.kind = Some(Card::Grade);
                commands.insert_resource(Paused);
                spawn_popup(&mut commands, spec.title, Some(body), spec.continue_after);
                return;
            }
        }
        show_prompt(&mut prompt, meter.live());
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
        if !text.0.is_empty() {
            text.0.clear();
        }
        if node.display != Display::None {
            node.display = Display::None;
        }
    }
}

fn show_prompt(prompt: &mut Query<(&mut Text, &mut Node), With<FlowPrompt>>, live: Option<String>) {
    if let Ok((mut text, mut node)) = prompt.single_mut() {
        if let Some(body) = live {
            // The prompt's numbers change often, but the hidden/empty state
            // must not rewrite the text (and re-shape it) every frame.
            if text.0 != body {
                text.0 = body;
            }
            if node.display != Display::Flex {
                node.display = Display::Flex;
            }
        } else {
            if !text.0.is_empty() {
                text.0.clear();
            }
            if node.display != Display::None {
                node.display = Display::None;
            }
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
