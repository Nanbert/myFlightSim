//! Start menu: a 3D hangar with all aircraft parked in a row and a text overlay.
//!
//! [`AppState`] gates the flight sim — every flight system carries
//! `run_if(unpaused)`, so while the menu is up the sim sits
//! frozen. PLAY swaps in the selected aircraft's FDM and enters `Flying`.
//!
//! Navigation: Left/Right switch the top row, Down/Up move focus between the top
//! row and a sub-row, Enter activates, Escape closes a page / pops the focus.
//! The aircraft TUTORIAL page scrolls with Up/Down (one line) and PgUp/PgDn
//! (one screen).

use bevy::prelude::*;
use bevy::text::FontSize;

use crate::aircraft_profile;
use crate::app_state::AppState;
use crate::autopilot::{Autopilot, Mcp};
use crate::config::{self, ConfigSink, FeelSetting, GameConfig};
use crate::input::FeelBaseline;
use crate::input::Bindings;
use crate::map::{self, MapId, PendingMission};
use crate::mission::{Lesson, PendingLesson};
use crate::menu_page::{self, PageLine};
use crate::sim::{RenderPose, Sim};
use crate::tutorial;
use crate::ui_theme;
use crate::view::spawn_model_child;
use crate::weather::Weather;

/// One selectable aircraft plus the numbers the hangar and showcase camera need.
pub struct MenuAircraft {
    /// FDM id, also the key `view::model_spec` and `Sim::new` expect.
    pub id: &'static str,
    /// Display label for the top row.
    pub label: &'static str,
    /// Model-root height that seats the wheels/skids on the ground grid (m);
    /// the FDM's rest CG height, matching the `model_spec` drop comments.
    pub root_y: f32,
    /// Camera distance from the model for its showcase framing (m).
    pub frame_dist: f32,
    /// Camera aim height above the model origin (m).
    pub frame_h: f32,
}

pub const AIRCRAFT: [MenuAircraft; 4] = [
    MenuAircraft {
        id: "c172p",
        label: "CESSNA 172",
        root_y: 1.4326,
        frame_dist: 16.0,
        frame_h: 1.6,
    },
    MenuAircraft {
        id: "f22",
        label: "F-22",
        root_y: 1.6946,
        frame_dist: 24.0,
        frame_h: 2.0,
    },
    MenuAircraft {
        id: "ah1s",
        label: "AH-64D",
        root_y: 1.8288,
        frame_dist: 24.0,
        frame_h: 2.8,
    },
    MenuAircraft {
        id: "737",
        label: "737",
        root_y: 1.2446,
        frame_dist: 80.0,
        frame_h: 6.0,
    },
];
/// Top-row entries after the aircraft: global settings, then about, then exit.
const EXTRA_TOP: [&str; 3] = ["SETTINGS", "ABOUT", "EXIT"];
const SETTINGS_TOP: usize = AIRCRAFT.len();
const ABOUT_TOP: usize = AIRCRAFT.len() + 1;
const EXIT_TOP: usize = AIRCRAFT.len() + 2;
const TOP_COUNT: usize = AIRCRAFT.len() + EXTRA_TOP.len();
/// The F-22's row: the only aircraft with a mission entry.
const F22_INDEX: usize = 1;
/// World spacing between parked showcase models (m).
const SPACING: f32 = 48.0;

const MENU_CAM_RATE: f32 = 4.0;
/// Camera pose for the no-model entries (SETTINGS / ABOUT): pulled back wide.
const WIDE_EYE: Vec3 = Vec3::new(0.0, 55.0, -230.0);
const WIDE_TARGET: Vec3 = Vec3::new(0.0, 6.0, 0.0);

#[derive(Component)]
struct ShowcaseAircraft;
#[derive(Component)]
struct TopLabel(usize);
/// The selection rail under a top-row entry: lit for the pointed-at entry.
#[derive(Component)]
struct TopUnderline(usize);
#[derive(Component)]
struct PanelRoot;
#[derive(Component)]
struct CardRoot;
/// The footer text cell's query filter, factored out to keep the signature
/// readable.
type FooterTextFilter = (With<FooterText>, Without<TopLabel>, Without<TopUnderline>);
/// The one-line key hints at the bottom of the hangar; text follows the state
/// (browsing vs. an open page).
#[derive(Component)]
struct FooterText;

#[derive(Clone, Copy, PartialEq, Eq)]
enum Page {
    /// The aircraft's sub-menu card: PLAY / TUTORIAL / INFO / SETTINGS / FEEL.
    Submenu,
    About,
    /// The exit confirmation card.
    Exit,
    /// How to fly the selected aircraft, in the sim and in reality.
    Tutorial,
    Info,
    Settings,
    /// The start-state page the aircraft's PLAY row opens: free flight plus
    /// the graded lesson missions.
    Play,
    /// The F-22's arcade combat maps.
    Missions,
    /// The global, aircraft-independent settings page (weather, ...).
    Global,
    /// The per-aircraft CONTROL FEEL page (C172 and 737 only): dial the
    /// keyboard feel, persist as defaults-relative multipliers.
    Feel,
}

/// Cursor within the PLAY start-state page.
#[derive(Resource, Default)]
struct PlayState {
    row: usize,
}

/// Cursor within the F-22's MISSIONS page.
#[derive(Resource, Default)]
struct MissionState {
    row: usize,
}

/// Rows on the global settings page: weather, wind and time of day. The engine
/// start state used to be here too; it now belongs to the lesson (M1 starts
/// cold, everything else runs), so the row is gone.
const SETTINGS_ROWS: usize = 3;

/// Cursor within the global settings page.
#[derive(Resource, Default)]
struct SettingsState {
    row: usize,
}

/// Scroll offset (in body lines) of the open aircraft tutorial. Reset whenever
/// the page is opened or closed so each visit starts at the top.
#[derive(Resource, Default)]
struct TutorialState {
    scroll: usize,
}

/// Cursor within the per-aircraft settings page.
#[derive(Resource, Default)]
struct AircraftSettings {
    row: usize,
}

/// Cursor within the per-aircraft CONTROL FEEL page.
#[derive(Resource, Default)]
struct FeelState {
    row: usize,
}

/// Whether the aircraft's settings page carries the Realistic/Arcade switch.
fn has_flight_model_row(id: &str) -> bool {
    aircraft_profile::profile(id).arcade_selectable
}

/// C172-only keyboard-assist rows: bank hold and turn coordination.
fn assist_row_count(id: &str) -> usize {
    if aircraft_profile::normalized(id) == "c172p" {
        2
    } else {
        0
    }
}

/// Wrap the selected row of a paged list: clamp to the row count, then wrap
/// around the top/bottom on ArrowUp/ArrowDown (`n` is expected >= 1 by the
/// callers, which all `.max(1)` their row sources).
fn cursor_nav(cur: usize, n: usize, up: bool, down: bool) -> usize {
    let cur = cur.min(n - 1);
    if up {
        (cur + n - 1) % n
    } else if down {
        (cur + 1) % n
    } else {
        cur
    }
}

/// Rows on the per-aircraft settings page. Zero for aircraft whose settings are
/// entirely fixed (the F-22 and the 737 hide the SETTINGS entry).
fn settings_row_count(id: &str) -> usize {
    has_flight_model_row(id) as usize + assist_row_count(id)
}

/// Where the cursor is: the highlighted top entry, the submenu card cursor,
/// and the stack of open page cards.
#[derive(Resource, Default)]
struct MenuSelection {
    top: usize,
    sub: usize,
    page_stack: Vec<Page>,
}

impl MenuSelection {
    /// The showcased aircraft, or `None` for the SETTINGS/ABOUT entries.
    fn aircraft_index(&self) -> Option<usize> {
        (self.top < AIRCRAFT.len()).then_some(self.top)
    }

    /// The selected aircraft's FDM id (only meaningful on an aircraft entry).
    fn aircraft_id(&self) -> &'static str {
        AIRCRAFT[self.top.min(AIRCRAFT.len() - 1)].id
    }

    /// The page on top of the stack, or `None` if no card is open.
    fn current_page(&self) -> Option<&Page> {
        self.page_stack.last()
    }

    /// Push a page onto the stack (open a sub-card).
    fn push_page(&mut self, page: Page) {
        self.page_stack.push(page);
    }

    /// Close the current card: pop if there's a card beneath, else clear.
    fn close_card(&mut self) {
        if self.page_stack.len() > 1 {
            self.page_stack.pop();
        } else {
            self.page_stack.clear();
        }
    }

    /// Open the initial card for the current top entry (sub-menu, settings,
    /// or about), resetting the submenu cursor.
    fn open_initial(&mut self) {
        self.sub = 0;
        self.page_stack = vec![initial_page_for(self.top)];
    }
}

/// Smoothed aim point for the showcase camera (so switching models pans rather
/// than snapping the rotation).
#[derive(Resource)]
struct MenuCamLook(Vec3);

impl Default for MenuCamLook {
    fn default() -> Self {
        Self(WIDE_TARGET)
    }
}

fn top_labels() -> [&'static str; TOP_COUNT] {
    let mut out = [""; TOP_COUNT];
    for (i, a) in AIRCRAFT.iter().enumerate() {
        out[i] = a.label;
    }
    for (i, e) in EXTRA_TOP.iter().enumerate() {
        out[AIRCRAFT.len() + i] = e;
    }
    out
}

/// The sub-row for a top entry. The global SETTINGS/ABOUT entries have none —
/// SETTINGS opens its own page (Enter), ABOUT opens straight away. The F-22
/// flies realistic FREE FLY or arcade MISSIONS; the other aircraft keep the
/// start-state page (its lessons are the C172's graded missions).
fn sub_items(top: usize) -> &'static [&'static str] {
    match top {
        F22_INDEX => &["FREE FLY", "MISSIONS", "TUTORIAL", "INFO"],
        t if t < AIRCRAFT.len() => match AIRCRAFT[t].id {
            // The C172 also flies its keyboard assists; both the C172 and the
            // 737 offer the CONTROL FEEL dial page.
            "c172p" => &["PLAY", "TUTORIAL", "INFO", "SETTINGS", "FEEL"],
            "737" => &["PLAY", "TUTORIAL", "INFO", "FEEL"],
            // Only the AH-64D still has anything else to configure: its
            // flight model.
            "ah1s" => &["PLAY", "TUTORIAL", "INFO", "SETTINGS"],
            _ => &["PLAY", "TUTORIAL", "INFO"],
        },
        _ => &[],
    }
}

/// The initial card that auto-opens when a top entry is selected. Aircraft
/// entries open their sub-menu card; SETTINGS/ABOUT/EXIT open their page directly.
fn initial_page_for(top: usize) -> Page {
    if top == SETTINGS_TOP {
        Page::Global
    } else if top == ABOUT_TOP {
        Page::About
    } else if top == EXIT_TOP {
        Page::Exit
    } else {
        Page::Submenu
    }
}

/// Parked position of showcase model `i`, centred on the origin along +X.
fn showcase_pos(i: usize) -> Vec3 {
    let half = (AIRCRAFT.len() as f32 - 1.0) / 2.0;
    Vec3::new(i as f32 - half, 0.0, 0.0) * SPACING
}

pub struct MenuPlugin;

impl Plugin for MenuPlugin {
    fn build(&self, app: &mut App) {
        app.init_state::<AppState>()
            .init_resource::<MenuSelection>()
            .init_resource::<MenuCamLook>()
            .init_resource::<SettingsState>()
            .init_resource::<AircraftSettings>()
            .init_resource::<FeelState>()
            .init_resource::<TutorialState>()
            .init_resource::<PlayState>()
            .init_resource::<MissionState>()
            .init_resource::<menu_page::PageLines>()
            .add_systems(Startup, auto_start_mission)
            .add_systems(
                OnEnter(AppState::Menu),
                (enter_menu, auto_start_aircraft, spawn_hangar, setup_menu_ui).chain(),
            )
            // `start_flight` must run before the mesh spawns, so the root picks
            // up the newly selected model. The set also lets other plugins'
            // spawners run after `Sim` reflects the selected aircraft.
            .add_systems(
                OnEnter(AppState::Flying),
                (
                    start_flight,
                    crate::view::spawn_flight_aircraft,
                    crate::view::reset_control_rig,
                )
                    .chain()
                    .in_set(crate::sim::StartFlight),
            )
            .add_systems(
                Update,
                (
                    menu_input.run_if(in_state(AppState::Menu)),
                    // `update_menu_ui` writes the row model, `render_page_lines`
                    // paints it — chained so one frame's state reaches both.
                    (update_menu_ui, menu_page::render_page_lines)
                        .chain()
                        .run_if(in_state(AppState::Menu)),
                    auto_open_page.run_if(in_state(AppState::Menu)),
                    menu_camera.run_if(in_state(AppState::Menu)),
                ),
            );
    }
}

/// Development hook: `MAP=ARCHIPELAGO` / `MAP=ALPINE` boots straight into that
/// mission, so a screenshot pass does not need to drive the menu.
fn auto_start_mission(mut pending: ResMut<PendingMission>, mut next: ResMut<NextState<AppState>>) {
    let Ok(label) = std::env::var("MAP") else {
        return;
    };
    if let Some(map) = MapId::from_label(&label.to_ascii_uppercase()) {
        pending.0 = Some(map);
        next.set(AppState::Flying);
    }
}

/// Development hook: `AUTOFLY=ah1s` (or `ah64d`) boots straight into that
/// aircraft, so screenshots and headless checks don't need to drive the menu.
/// `COCKPIT=1` additionally opens the pilot-eye view.
fn auto_start_aircraft(mut sel: ResMut<MenuSelection>, mut next: ResMut<NextState<AppState>>) {
    let Ok(id) = std::env::var("AUTOFLY") else {
        return;
    };
    let id = aircraft_profile::normalized(&id);
    if let Some(i) = AIRCRAFT.iter().position(|a| a.id == id) {
        sel.top = i;
        sel.sub = 0;
        sel.page_stack.clear();
        next.set(AppState::Flying);
    }
}

/// Development hook: `PAGE=GLOBAL` (also ABOUT/TUTORIAL/INFO/SETTINGS/PLAY/
/// MISSIONS) opens that menu page, so a screenshot pass can capture the panel
/// card without driving the menu. Idempotent, and gated to the menu state.
fn auto_open_page(mut sel: ResMut<MenuSelection>) {
    let Ok(page) = std::env::var("PAGE") else {
        return;
    };
    let opened = match page.to_ascii_uppercase().as_str() {
        "ABOUT" => {
            sel.top = ABOUT_TOP;
            Page::About
        }
        "GLOBAL" => {
            sel.top = SETTINGS_TOP;
            Page::Global
        }
        "EXIT" => {
            sel.top = EXIT_TOP;
            Page::Exit
        }
        "TUTORIAL" => Page::Tutorial,
        "INFO" => Page::Info,
        "SETTINGS" => Page::Settings,
        "PLAY" => Page::Play,
        "MISSIONS" => Page::Missions,
        "FEEL" => Page::Feel,
        _ => return,
    };
    // Sub-pages sit beneath the sub-menu card; initial cards stand alone.
    if matches!(
        opened,
        Page::Tutorial
            | Page::Info
            | Page::Settings
            | Page::Play
            | Page::Missions
            | Page::Feel
    ) {
        sel.page_stack = vec![Page::Submenu, opened];
    } else {
        sel.page_stack = vec![opened];
    }
}

/// Select the aircraft matching the loaded sim (startup default or the one we
/// just flew) and reset the cursor, opening its sub-menu card.
fn enter_menu(mut sel: ResMut<MenuSelection>, sim: Res<Sim>) {
    let cur = if sim.name == "b737" {
        "737"
    } else {
        sim.name.as_str()
    };
    let top = AIRCRAFT.iter().position(|a| a.id == cur).unwrap_or(0);
    *sel = MenuSelection {
        top,
        sub: 0,
        page_stack: vec![initial_page_for(top)],
    };
}

/// Park all aircraft in a row. Static display: no control surfaces are driven,
/// and the models carry `DespawnOnExit(Menu)` so they vanish on PLAY.
fn spawn_hangar(mut commands: Commands, assets: Res<AssetServer>) {
    for (i, a) in AIRCRAFT.iter().enumerate() {
        let root = commands
            .spawn((
                ShowcaseAircraft,
                Transform::from_translation(showcase_pos(i) + Vec3::Y * a.root_y),
                Visibility::default(),
                DespawnOnExit(AppState::Menu),
            ))
            .id();
        spawn_model_child(&mut commands, &assets, root, a.id);
    }
}

/// Build the menu overlay: a top row, a sub-row, and a full-page panel.
fn setup_menu_ui(mut commands: Commands) {
    let root = commands
        .spawn((
            Node {
                position_type: PositionType::Absolute,
                width: Val::Percent(100.0),
                height: Val::Percent(100.0),
                flex_direction: FlexDirection::Column,
                align_items: AlignItems::Center,
                justify_content: JustifyContent::FlexStart,
                padding: UiRect::top(Val::Px(40.0)),
                ..default()
            },
            DespawnOnExit(AppState::Menu),
        ))
        .id();

    // Corner branding: a quiet wordmark over the hangar floor.
    commands.spawn((
        Node {
            position_type: PositionType::Absolute,
            left: Val::Px(32.0),
            top: Val::Px(24.0),
            ..default()
        },
        ChildOf(root),
        Text::new("MYFLIGHTSIM"),
        TextFont {
            font_size: FontSize::Px(18.0),
            ..default()
        },
        TextColor(ui_theme::dim()),
    ));

    let top_row = commands
        .spawn((
            Node {
                flex_direction: FlexDirection::Row,
                column_gap: Val::Px(28.0),
                padding: UiRect {
                    left: Val::Px(32.0),
                    right: Val::Px(32.0),
                    top: Val::Px(16.0),
                    bottom: Val::Px(14.0),
                },
                border: UiRect::all(Val::Px(1.0)),
                border_radius: BorderRadius::all(Val::Px(10.0)),
                margin: UiRect::bottom(Val::Px(18.0)),
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
    for i in 0..TOP_COUNT {
        // Each entry is a column of [label, rail]; the rail's width tracks the
        // label above it, so every entry gets its own underline.
        let entry = commands
            .spawn((
                Node {
                    flex_direction: FlexDirection::Column,
                    align_items: AlignItems::Center,
                    row_gap: Val::Px(7.0),
                    ..default()
                },
                ChildOf(top_row),
            ))
            .id();
        commands.spawn((
            Text::new(""),
            TextFont {
                font_size: FontSize::Px(30.0),
                ..default()
            },
            TextColor(Color::WHITE),
            TopLabel(i),
            ChildOf(entry),
        ));
        commands.spawn((
            Node {
                width: Val::Percent(100.0),
                height: Val::Px(3.0),
                ..default()
            },
            // Invisible until `update_menu_ui` lights the pointed-at entry.
            BackgroundColor(Color::NONE),
            TopUnderline(i),
            ChildOf(entry),
        ));
    }

    // Full-page overlay, spawned after the rows so it draws on top of them.
    // A scene-darkening backdrop plus a centred card keeps the hangar frame
    // visible while the page reads; the rows dim out under it.
    let panel = commands
        .spawn((
            PanelRoot,
            Node {
                position_type: PositionType::Absolute,
                left: Val::Px(0.0),
                top: Val::Px(0.0),
                width: Val::Percent(100.0),
                height: Val::Percent(100.0),
                flex_direction: FlexDirection::Column,
                align_items: AlignItems::Center,
                justify_content: JustifyContent::Center,
                display: Display::None,
                ..default()
            },
            BackgroundColor(Color::srgba(0.0, 0.0, 0.0, 0.62)),
            DespawnOnExit(AppState::Menu),
        ))
        .id();
    // The card: rounded, hairline green border, backlit-glass glow.
    let card = commands
        .spawn((
            CardRoot,
            Node {
                flex_direction: FlexDirection::Column,
                width: Val::Percent(92.0),
                max_width: Val::Px(1180.0),
                max_height: Val::Percent(86.0),
                padding: UiRect {
                    left: Val::Px(48.0),
                    right: Val::Px(48.0),
                    top: Val::Px(30.0),
                    bottom: Val::Px(26.0),
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
                Val::Px(12.0),
            ),
            ChildOf(panel),
        ))
        .id();
    // The pooled page rows paint the card's contents (menu_page module).
    menu_page::spawn_rows(&mut commands, card);

    // Bottom key-hint bar; `update_menu_ui` swaps the text to match the state.
    // The bar is a full-width row whose only child (the text) is centred.
    let footer_bar = commands
        .spawn((
            Node {
                position_type: PositionType::Absolute,
                bottom: Val::Px(22.0),
                left: Val::Px(0.0),
                right: Val::Px(0.0),
                justify_content: JustifyContent::Center,
                align_items: AlignItems::Center,
                ..default()
            },
            ChildOf(root),
        ))
        .id();
    commands.spawn((
        Text::new(""),
        TextFont {
            font_size: FontSize::Px(16.0),
            ..default()
        },
        TextColor(ui_theme::dim()),
        FooterText,
        ChildOf(footer_bar),
    ));
}

fn color_active() -> Color {
    ui_theme::accent()
}
fn color_selected() -> Color {
    ui_theme::selected()
}
fn color_dim() -> Color {
    Color::srgb(0.42, 0.47, 0.58)
}

/// Repaint the labels and the panel from the current selection. The panel
/// shows the top of the page stack (sub-menu card, settings page, etc.).
#[allow(clippy::too_many_arguments)]
fn update_menu_ui(
    sel: Res<MenuSelection>,
    weather: Res<Weather>,
    settings: Res<SettingsState>,
    play: Res<PlayState>,
    missions: Res<MissionState>,
    config: Res<GameConfig>,
    aircraft_settings: Res<AircraftSettings>,
    feel_state: Res<FeelState>,
    tutorial_state: Res<TutorialState>,
    mut tops: Query<(&TopLabel, &mut Text, &mut TextColor)>,
    mut bars: Query<(&TopUnderline, &mut BackgroundColor)>,
    mut panel: Query<&mut Node, (With<PanelRoot>, Without<CardRoot>)>,
    mut card: Query<&mut Node, With<CardRoot>>,
    mut footer: Query<&mut Text, FooterTextFilter>,
    mut pages: ResMut<menu_page::PageLines>,
) {
    // Writes are guarded throughout: an unconditional `Mut` write marks the
    // component changed, and a changed text/color/layout node forces Bevy UI
    // to re-layout and repaint that subtree on an otherwise idle menu frame.
    let labels = top_labels();
    for (TopLabel(i), mut text, mut color) in &mut tops {
        let i = *i;
        if text.0 != labels[i] {
            text.0 = labels[i].to_string();
        }
        // The top entry under a sub-menu card is "pointed at" (selected
        // colour); a focused entry with its own card is accent.
        let on_aircraft_card = matches!(sel.current_page(), Some(Page::Submenu));
        let want = if i == sel.top {
            if on_aircraft_card {
                color_active()
            } else {
                color_selected()
            }
        } else {
            color_dim()
        };
        if color.0 != want {
            color.0 = want;
        }
    }
    // The rail under the pointed-at entry lights; the rest stay dark so the
    // column reads as a tab strip rather than a row of links.
    for (TopUnderline(i), mut bg) in &mut bars {
        let want = if *i == sel.top {
            color_active()
        } else {
            Color::NONE
        };
        if bg.0 != want {
            bg.0 = want;
        }
    }

    let want_display = if sel.page_stack.is_empty() {
        Display::None
    } else {
        Display::Flex
    };
    for mut node in &mut panel {
        if node.display != want_display {
            node.display = want_display;
        }
    }
    // Narrow cards for EXIT, Submenu, and Settings pages; wider for content-heavy pages.
    for mut node in &mut card {
        let narrow = matches!(
            sel.current_page(),
            Some(Page::Exit)
                | Some(Page::Submenu)
                | Some(Page::Global)
                | Some(Page::Settings)
        );
        let want_width = if narrow { Val::Px(450.0) } else { Val::Percent(92.0) };
        let want_max = if narrow { Val::Px(450.0) } else { Val::Px(1180.0) };
        if node.width != want_width {
            node.width = want_width;
        }
        if node.max_width != want_max {
            node.max_width = want_max;
        }
    }
    // The page renders as structured rows (menu_page module); freeform pages
    // fold their prose through `freeform`, which keeps the old text look.
    pages.0 = match sel.current_page().copied() {
        Some(Page::Submenu) => submenu_lines(sel.top, sel.sub),
        Some(Page::Exit) => exit_lines(),
        Some(Page::Global) => global_settings_lines(&weather, &settings),
        Some(Page::Play) => play_lines(sel.aircraft_id(), &play),
        Some(Page::Missions) => missions_lines(&missions),
        Some(Page::Settings) => {
            aircraft_settings_lines(sel.aircraft_id(), &config, &aircraft_settings)
        }
        Some(Page::Feel) => feel_lines(sel.aircraft_id(), &config, &feel_state),
        Some(Page::Tutorial) => menu_page::freeform(&tutorial::render(
            sel.aircraft_id(),
            tutorial_state.scroll,
        )),
        Some(p) => menu_page::freeform(&page_text(p, &sel)),
        None => Vec::new(),
    };

    // The hint bar follows the current card, so the player always sees what
    // the next keypress does.
    if let Ok(mut text) = footer.single_mut() {
        let want = match sel.current_page().copied() {
            Some(Page::Feel) => "[,/.] ADJUST   [R] RESET   [ENTER]/[ESC] CLOSE   [LEFT/RIGHT] TAB",
            Some(Page::Submenu) => "[UP/DOWN] SELECT   [ENTER] OPEN   [ESC] BACK   [LEFT/RIGHT] TAB",
            Some(Page::Exit) => "[ENTER] EXIT   [ESC] BACK   [LEFT/RIGHT] TAB",
            Some(Page::Play) | Some(Page::Missions) => {
                "[UP/DOWN] SELECT   [ENTER] FLY   [ESC] BACK   [LEFT/RIGHT] TAB"
            }
            Some(Page::Global) | Some(Page::Settings) => {
                "[UP/DOWN] SELECT   [,/.] CHANGE   [ENTER]/[ESC] CLOSE   [LEFT/RIGHT] TAB"
            }
            Some(Page::About) | Some(Page::Info) => "[ENTER]/[ESC] CLOSE   [LEFT/RIGHT] TAB",
            Some(Page::Tutorial) => "[UP/DOWN] SCROLL   [ENTER]/[ESC] CLOSE   [LEFT/RIGHT] TAB",
            None => "[LEFT/RIGHT] BROWSE   [ENTER] OPEN",
        };
        if text.0 != want {
            text.0 = want.to_string();
        }
    }
}

/// The aircraft's settings page rows: the Realistic/Arcade switch (AH-64D
/// only) or the C172's keyboard assists. Bindings themselves are fixed.
fn aircraft_settings_lines(id: &str, config: &GameConfig, settings: &AircraftSettings) -> Vec<PageLine> {
    let n = settings_row_count(id).max(1);
    let cursor = settings.row.min(n - 1);

    let mut out = vec![
        PageLine::heading(format!("{} - SETTINGS", aircraft_label(id))),
        PageLine::blank(),
    ];
    let mut row = 0;
    if has_flight_model_row(id) {
        let model = if config.arcade(id) {
            "Arcade"
        } else {
            "Realistic"
        };
        out.push(PageLine::option("FLIGHT MODEL", model, row == cursor));
        row += 1;
    }
    if assist_row_count(id) > 0 {
        let a = config.assists(id);
        for (label, on) in [("BANK HOLD", a.bank_hold), ("TURN COORD", a.turn_coord)] {
            let value = if on { "ON" } else { "OFF" };
            out.push(PageLine::option(label, value, row == cursor));
            row += 1;
        }
    }

    out.push(PageLine::blank());
    out.push(PageLine::hint("[Up/Down] select   [,/.] change   [Esc] close   [Left/Right] tab"));
    out
}

/// One knob on the CONTROL FEEL page, mapped to its `FeelSetting` field
/// (`FeelKnob` carries the axis for the travel knobs; `Restore` is the
/// Enter-to-wipe row).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum TravelAxis {
    Elevator,
    Aileron,
    Rudder,
    Throttle,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum FeelKnob {
    Travel(TravelAxis),
    Tap,
    Linger,
    Return,
    Fine,
    Restore,
}

struct FeelRow {
    label: &'static str,
    knob: FeelKnob,
}

/// The FEEL page's rows for an aircraft, in display order. The travel knobs
/// exist for every aircraft that can fly at all; tap/fine ride their class
/// baselines (the 737 has neither).
fn feel_rows(id: &str) -> Vec<FeelRow> {
    let b = crate::input::Bindings::feel_baseline(id);
    let mut rows = vec![
        FeelRow { label: "ELEVATOR TRAVEL", knob: FeelKnob::Travel(TravelAxis::Elevator) },
        FeelRow { label: "AILERON TRAVEL", knob: FeelKnob::Travel(TravelAxis::Aileron) },
        FeelRow { label: "RUDDER TRAVEL", knob: FeelKnob::Travel(TravelAxis::Rudder) },
        FeelRow { label: "THROTTLE SWEEP", knob: FeelKnob::Travel(TravelAxis::Throttle) },
    ];
    if b.tap_enabled {
        rows.push(FeelRow { label: "TAP NUDGE", knob: FeelKnob::Tap });
    }
    rows.push(FeelRow { label: "RELEASE LINGER", knob: FeelKnob::Linger });
    rows.push(FeelRow { label: "RETURN SPEED", knob: FeelKnob::Return });
    if b.fine_enabled {
        rows.push(FeelRow { label: "FINE RATE", knob: FeelKnob::Fine });
    }
    rows.push(FeelRow { label: "RESTORE ALL", knob: FeelKnob::Restore });
    rows
}

/// One Left/Right press on a row: the multiplier's delta. In multiplier
/// space, tuned so the displayed value moves in round steps (ten percent of
/// travel time, one percent of nudge, ten percent of return speed, five
/// percent of fine rate).
fn feel_step(knob: FeelKnob, dir: isize) -> f64 {
    let d = dir as f64;
    match knob {
        FeelKnob::Travel(_) => 0.1 * d,
        FeelKnob::Tap => 0.25 * d,
        FeelKnob::Linger => 0.1 * d,
        FeelKnob::Return => 0.125 * d,
        FeelKnob::Fine => 0.2 * d,
        FeelKnob::Restore => 0.0,
    }
}

/// The multiplier a knob reads and writes on `FeelSetting`.
fn knob_mult(knob: FeelKnob, f: &mut FeelSetting) -> &mut f64 {
    match knob {
        FeelKnob::Travel(TravelAxis::Elevator) => &mut f.travel_elevator,
        FeelKnob::Travel(TravelAxis::Aileron) => &mut f.travel_aileron,
        FeelKnob::Travel(TravelAxis::Rudder) => &mut f.travel_rudder,
        FeelKnob::Travel(TravelAxis::Throttle) => &mut f.travel_throttle,
        FeelKnob::Tap => &mut f.tap,
        FeelKnob::Linger => &mut f.linger,
        FeelKnob::Return => &mut f.release,
        FeelKnob::Fine => &mut f.fine,
        FeelKnob::Restore => &mut f.tap, // dead end: Restore never mutates (see the callers)
    }
}

/// Nudge one knob by `dir presses`. `set_feel` clamps and erases defaults.
fn feel_change(config: &mut GameConfig, id: &str, knob: FeelKnob, dir: isize) {
    if knob == FeelKnob::Restore {
        return;
    }
    let step = feel_step(knob, dir);
    config.set_feel(id, |f| *knob_mult(knob, f) += step);
}

/// R on a row: one knob back to the default multiplier.
fn feel_reset_row(config: &mut GameConfig, id: &str, knob: FeelKnob) {
    if knob == FeelKnob::Restore {
        return;
    }
    config.set_feel(id, |f| *knob_mult(knob, f) = 1.0);
}

/// The knob's stored multiplier and displayed baseline.
fn feel_knob_state(knob: FeelKnob, fs: &FeelSetting, baseline: &FeelBaseline) -> (f64, f64) {
    let (mult, base) = match knob {
        FeelKnob::Travel(TravelAxis::Elevator) => (fs.travel_elevator, baseline.travel_elevator),
        FeelKnob::Travel(TravelAxis::Aileron) => (fs.travel_aileron, baseline.travel_aileron),
        FeelKnob::Travel(TravelAxis::Rudder) => (fs.travel_rudder, baseline.travel_rudder),
        FeelKnob::Travel(TravelAxis::Throttle) => (fs.travel_throttle, baseline.travel_throttle),
        FeelKnob::Tap => (fs.tap, baseline.tap_step),
        FeelKnob::Linger => (fs.linger, baseline.release_hold),
        FeelKnob::Return => (fs.release, baseline.release_scale),
        FeelKnob::Fine => (fs.fine, baseline.fine_scale),
        FeelKnob::Restore => (1.0, 1.0),
    };
    (mult, base)
}

/// The FEEL page rows: absolute values with the stock value in brackets.
fn feel_lines(id: &str, config: &GameConfig, state: &FeelState) -> Vec<PageLine> {
    let baseline = crate::input::Bindings::feel_baseline(id);
    let fs = config.feel(id);
    let rows = feel_rows(id);
    let cursor = state.row.min(rows.len() - 1);

    let mut out = vec![
        PageLine::heading(format!("{} - CONTROL FEEL", aircraft_label(id))),
        PageLine::note("  Dial the keyboard feel. [stock] lists the value R resets to."),
        PageLine::blank(),
    ];
    for (i, row) in rows.iter().enumerate() {
        let value = if row.knob == FeelKnob::Restore {
            "WIPE EVERY KNOB".to_string()
        } else {
            let (mult, base) = feel_knob_state(row.knob, &fs, &baseline);
            match row.knob {
                FeelKnob::Travel(_) | FeelKnob::Linger => {
                    format!("{:.2} s   [stock {:.2} s]", base * mult, base)
                }
                _ => format!(
                    "{:.0} %   [stock {:.0} %]",
                    base * mult * 100.0,
                    base * 100.0
                ),
            }
        };
        out.push(PageLine::option(row.label, value, i == cursor));
    }
    out.push(PageLine::blank());
    out.push(PageLine::hint(
        "[Up/Down] select   [,/.] adjust   [R] reset row   [Enter/Esc] close   [Left/Right] tab",
    ));
    out
}

/// Display label for a canonical aircraft id.
fn aircraft_label(id: &str) -> &'static str {
    let id = aircraft_profile::normalized(id);
    AIRCRAFT
        .iter()
        .find(|a| a.id == id)
        .map_or("AIRCRAFT", |a| a.label)
}

/// The global settings page rows: one option row per setting, the cursor row
/// lit and its value in the amber cell.
fn global_settings_lines(weather: &Weather, settings: &SettingsState) -> Vec<PageLine> {
    let rows: [(&str, String); SETTINGS_ROWS] = [
        ("WEATHER", weather.label().to_string()),
        ("WIND 300DEG", weather.wind_label().to_string()),
        ("TIME OF DAY", weather.time_label().to_string()),
    ];
    let mut out = vec![PageLine::heading("GLOBAL SETTINGS"), PageLine::blank()];
    for (i, (label, value)) in rows.iter().enumerate() {
        out.push(PageLine::option(*label, value.clone(), i == settings.row));
    }
    out.push(PageLine::blank());
    out.push(PageLine::hint(
        "[Up/Down] select   [,/.] change   [Enter] or [Esc] to close   [Left/Right] tab",
    ));
    out
}

/// The PLAY page rows: the start-state list. The C172 and the 737 carry their
/// own graded lessons; the rest see only the free row (and a note).
fn play_lines(id: &str, play: &PlayState) -> Vec<PageLine> {
    let rows = crate::mission::rows_for(id);
    let mut out = vec![
        PageLine::heading(format!("{} - SELECT START", aircraft_label(id))),
        PageLine::blank(),
    ];
    for (i, row) in rows.iter().enumerate() {
        out.push(PageLine::option(row.label, "", i == play.row));
        out.push(PageLine::note(format!("      {}", row.hint)));
        out.push(PageLine::blank());
    }
    if rows.len() == 1 {
        out.push(PageLine::note(
            "The graded lessons fly the Cessna 172 and the 737.",
        ));
        out.push(PageLine::blank());
    }
    out.push(PageLine::hint("[Up/Down] select   [Enter] fly   [Esc] close"));
    out
}

/// The F-22's MISSIONS page rows: the arcade combat maps.
fn missions_lines(missions: &MissionState) -> Vec<PageLine> {
    let mut out = vec![PageLine::heading("F-22 - MISSIONS"), PageLine::blank()];
    for (i, label) in map::MAP_LABELS.iter().enumerate() {
        out.push(PageLine::option(*label, "", i == missions.row));
    }
    out.push(PageLine::blank());
    out.push(PageLine::note(
        "Arcade air combat over a bounded arena: drones and an escort.",
    ));
    out.push(PageLine::blank());
    out.push(PageLine::hint("[Up/Down] select   [Enter] fly   [Esc] close"));
    out
}

/// The sub-menu card rows: the aircraft's playable options, one per row.
fn submenu_lines(top: usize, sub: usize) -> Vec<PageLine> {
    let items = sub_items(top);
    let mut out = vec![
        PageLine::heading(format!("{} - MENU", AIRCRAFT[top].label)),
        PageLine::blank(),
    ];
    for (i, label) in items.iter().enumerate() {
        out.push(PageLine::option(*label, "", i == sub));
    }
    out.push(PageLine::blank());
    out.push(PageLine::hint("[Up/Down] select   [Enter] open   [Esc] back   [Left/Right] tab"));
    out
}

/// The exit confirmation card rows: a single question and an OK button.
fn exit_lines() -> Vec<PageLine> {
    vec![
        PageLine::heading("Are you sure to exit?"),
        PageLine::blank(),
        PageLine::option("OK", "", true),
        PageLine::blank(),
        PageLine::hint("[Enter] exit   [Esc] back"),
    ]
}

fn page_text(page: Page, sel: &MenuSelection) -> String {
    let close = "\n\n[Enter] or [Esc] to close";
    let label = AIRCRAFT[sel.aircraft_index().unwrap_or(0)].label;
    match page {
        Page::About => format!(
            "myFlightSim\n\n\
             A pure-Rust flight dynamics model driving Bevy-rendered aircraft.\n\
             The Cessna 172, F-22, AH-64D and 737 fly the same neutral property bus,\n\
             with a JSBSim backend available as a drop-in alternative.\n\n\
             Developer: Nanbert{close}"
        ),
        // The built-in font is ASCII-only, so no em dashes here (they render as
        // missing-glyph boxes).
        Page::Info => format!("{label} - INFO\n\nAircraft info coming soon.{close}"),
        // Folded through `menu_page::freeform`. Rendered by `tutorial::render`
        // and the `*_lines` builders, which need live state; `page_text` never
        // sees these.
        Page::Tutorial | Page::Settings | Page::Global | Page::Play | Page::Feel
        | Page::Missions | Page::Submenu | Page::Exit => {
            String::new()
        }
    }
}

/// Arrow/Enter/Escape navigation. Gated to `Menu`.
fn menu_input(
    keys: Res<ButtonInput<KeyCode>>,
    mut sel: ResMut<MenuSelection>,
    mut next: ResMut<NextState<AppState>>,
    mut weather: ResMut<Weather>,
    mut settings: ResMut<SettingsState>,
    mut aircraft_settings: ResMut<AircraftSettings>,
    mut tutorial_state: ResMut<TutorialState>,
    mut play: ResMut<PlayState>,
    mut mission_state: ResMut<MissionState>,
    mut feel_state: ResMut<FeelState>,
    mut config: ResMut<GameConfig>,
    mut pending: ResMut<PendingMission>,
    mut lesson_pending: ResMut<PendingLesson>,
    sink: Res<ConfigSink>,
) {
    let enter = keys.just_pressed(KeyCode::Enter) || keys.just_pressed(KeyCode::NumpadEnter);
    let esc = keys.just_pressed(KeyCode::Escape);

    // A card is open: handle its navigation.
    if let Some(page) = sel.current_page().copied() {
        match page {
            // The sub-menu card: Up/Down picks an item, Enter opens the
            // sub-page or launches a flight. Left/Right closes the card and
            // switches tabs.
            Page::Submenu => {
                let items = sub_items(sel.top);
                let n = items.len();
                if n > 0 {
                    let up = keys.just_pressed(KeyCode::ArrowUp);
                    let down = keys.just_pressed(KeyCode::ArrowDown);
                    sel.sub = cursor_nav(sel.sub, n, up, down);

                    if enter {
                        match items.get(sel.sub).copied() {
                            // The F-22's FREE FLY launches straight onto the
                            // runway with the realistic model.
                            Some("FREE FLY") => {
                                pending.0 = None;
                                lesson_pending.0 = None;
                                next.set(AppState::Flying);
                                sel.page_stack.clear();
                            }
                            Some("MISSIONS") => {
                                mission_state.row = 0;
                                sel.push_page(Page::Missions);
                            }
                            Some("PLAY") => {
                                play.row = 0;
                                sel.push_page(Page::Play);
                            }
                            Some("TUTORIAL") => {
                                tutorial_state.scroll = 0;
                                sel.push_page(Page::Tutorial);
                            }
                            Some("INFO") => sel.push_page(Page::Info),
                            Some("SETTINGS") => {
                                aircraft_settings.row = 0;
                                sel.push_page(Page::Settings);
                            }
                            Some("FEEL") => {
                                feel_state.row = 0;
                                sel.push_page(Page::Feel);
                            }
                            _ => {}
                        }
                    }

                    if esc {
                        sel.close_card();
                    }

                    // Left/Right closes the card and switches tabs.
                    if keys.just_pressed(KeyCode::ArrowLeft) {
                        sel.close_card();
                        sel.top = (sel.top + TOP_COUNT - 1) % TOP_COUNT;
                        sel.open_initial();
                    }
                    if keys.just_pressed(KeyCode::ArrowRight) {
                        sel.close_card();
                        sel.top = (sel.top + 1) % TOP_COUNT;
                        sel.open_initial();
                    }
                }
            }
            // The start-state page: free flight or a graded lesson. The C172
            // and the 737 carry their own lesson rows; the rest see just the
            // free row.
            Page::Play => {
                let rows = crate::mission::rows_for(sel.aircraft_id());
                let up = keys.just_pressed(KeyCode::ArrowUp);
                let down = keys.just_pressed(KeyCode::ArrowDown);
                play.row = cursor_nav(play.row, rows.len(), up, down);
                if enter {
                    lesson_pending.0 = rows.get(play.row).and_then(|row| row.lesson);
                    next.set(AppState::Flying);
                    sel.page_stack.clear();
                }
                if esc {
                    sel.close_card();
                }

                // Left/Right closes the card and switches tabs.
                if keys.just_pressed(KeyCode::ArrowLeft) {
                    sel.close_card();
                    sel.top = (sel.top + TOP_COUNT - 1) % TOP_COUNT;
                    sel.open_initial();
                }
                if keys.just_pressed(KeyCode::ArrowRight) {
                    sel.close_card();
                    sel.top = (sel.top + 1) % TOP_COUNT;
                    sel.open_initial();
                }
            }
            // The F-22 mission list: pick an arcade combat map.
            Page::Missions => {
                let n = map::MAP_LABELS.len();
                let up = keys.just_pressed(KeyCode::ArrowUp);
                let down = keys.just_pressed(KeyCode::ArrowDown);
                mission_state.row = cursor_nav(mission_state.row, n, up, down);
                if enter {
                    if let Some(map) = MapId::from_label(map::MAP_LABELS[mission_state.row]) {
                        pending.0 = Some(map);
                        next.set(AppState::Flying);
                        sel.page_stack.clear();
                    }
                }
                if esc {
                    sel.close_card();
                }

                // Left/Right closes the card and switches tabs.
                if keys.just_pressed(KeyCode::ArrowLeft) {
                    sel.close_card();
                    sel.top = (sel.top + TOP_COUNT - 1) % TOP_COUNT;
                    sel.open_initial();
                }
                if keys.just_pressed(KeyCode::ArrowRight) {
                    sel.close_card();
                    sel.top = (sel.top + 1) % TOP_COUNT;
                    sel.open_initial();
                }
            }
            // GLOBAL is a settings list (Up/Down select, ,/. change); it is
            // the initial card for the SETTINGS top entry.
            Page::Global => {
                if keys.just_pressed(KeyCode::ArrowUp) {
                    settings.row = (settings.row + SETTINGS_ROWS - 1) % SETTINGS_ROWS;
                }
                if keys.just_pressed(KeyCode::ArrowDown) {
                    settings.row = (settings.row + 1) % SETTINGS_ROWS;
                }
                let dir = if keys.just_pressed(KeyCode::Comma) {
                    -1
                } else if keys.just_pressed(KeyCode::Period) {
                    1
                } else {
                    0
                };
                if dir != 0 {
                    match settings.row {
                        0 => {
                            let level = weather.level() as isize + dir;
                            weather.set_level(level);
                            config.weather_level = weather.level();
                            config::save(&config, &sink);
                        }
                        1 => {
                            let level = weather.wind_level() as isize + dir;
                            config.wind_level = weather.set_wind_level(level);
                            config::save(&config, &sink);
                        }
                        2 => {
                            let level = weather.time_level as isize + dir;
                            config.time_level = weather.set_time_level(level);
                            config::save(&config, &sink);
                        }
                        _ => {}
                    }
                }
                if enter || esc {
                    sel.close_card();
                }

                // Left/Right closes the card and switches tabs.
                if keys.just_pressed(KeyCode::ArrowLeft) {
                    sel.close_card();
                    sel.top = (sel.top + TOP_COUNT - 1) % TOP_COUNT;
                    sel.open_initial();
                }
                if keys.just_pressed(KeyCode::ArrowRight) {
                    sel.close_card();
                    sel.top = (sel.top + 1) % TOP_COUNT;
                    sel.open_initial();
                }
            }
            Page::Settings => {
                let id = sel.aircraft_id();
                let n = settings_row_count(id).max(1);
                let up = keys.just_pressed(KeyCode::ArrowUp);
                let down = keys.just_pressed(KeyCode::ArrowDown);
                aircraft_settings.row = cursor_nav(aircraft_settings.row, n, up, down);

                let dir = if keys.just_pressed(KeyCode::Comma) {
                    -1
                } else if keys.just_pressed(KeyCode::Period) {
                    1
                } else {
                    0
                };
                if dir != 0 {
                    let row = aircraft_settings.row;
                    if has_flight_model_row(id) && row == 0 {
                        let on = !config.arcade(id);
                        config.set_arcade(id, on);
                    } else if assist_row_count(id) > 0 {
                        let assist = row.saturating_sub(has_flight_model_row(id) as usize);
                        let a = config.assists(id);
                        match assist {
                            0 => config.set_bank_hold(id, !a.bank_hold),
                            _ => config.set_turn_coord(id, !a.turn_coord),
                        }
                    }
                    config::save(&config, &sink);
                }
                if enter || esc {
                    sel.close_card();
                }

                // Left/Right closes the card and switches tabs.
                if keys.just_pressed(KeyCode::ArrowLeft) {
                    sel.close_card();
                    sel.top = (sel.top + TOP_COUNT - 1) % TOP_COUNT;
                    sel.open_initial();
                }
                if keys.just_pressed(KeyCode::ArrowRight) {
                    sel.close_card();
                    sel.top = (sel.top + 1) % TOP_COUNT;
                    sel.open_initial();
                }
            }
            // The CONTROL FEEL page: Up/Down picks a knob, ,/. steps
            // its multiplier (persisted instantly), R resets the row; Enter
            // on RESTORE ALL wipes the aircraft's feel back to stock.
            Page::Feel => {
                let id = sel.aircraft_id();
                let rows = feel_rows(id);
                let n = rows.len();
                feel_state.row = cursor_nav(feel_state.row, n,
                    keys.just_pressed(KeyCode::ArrowUp), keys.just_pressed(KeyCode::ArrowDown));
                let dir = if keys.just_pressed(KeyCode::Comma) {
                    -1
                } else if keys.just_pressed(KeyCode::Period) {
                    1
                } else {
                    0
                };
                if dir != 0 {
                    feel_change(&mut config, id, rows[feel_state.row % n].knob, dir);
                    config::save(&config, &sink);
                }
                if keys.just_pressed(KeyCode::KeyR) {
                    feel_reset_row(&mut config, id, rows[feel_state.row % n].knob);
                    config::save(&config, &sink);
                }
                if enter {
                    if rows[feel_state.row % n].knob == FeelKnob::Restore {
                        config.reset_feel(id);
                        config::save(&config, &sink);
                    } else {
                        sel.close_card();
                    }
                }
                if esc {
                    sel.close_card();
                }

                // Left/Right closes the card and switches tabs.
                if keys.just_pressed(KeyCode::ArrowLeft) {
                    sel.close_card();
                    sel.top = (sel.top + TOP_COUNT - 1) % TOP_COUNT;
                    sel.open_initial();
                }
                if keys.just_pressed(KeyCode::ArrowRight) {
                    sel.close_card();
                    sel.top = (sel.top + 1) % TOP_COUNT;
                    sel.open_initial();
                }
            }
            Page::Tutorial => {
                let id = sel.aircraft_id();
                let max = tutorial::max_scroll(id);
                let mut scroll = tutorial_state.scroll;
                if keys.just_pressed(KeyCode::ArrowDown) {
                    scroll += 1;
                }
                if keys.just_pressed(KeyCode::ArrowUp) {
                    scroll = scroll.saturating_sub(1);
                }
                if keys.just_pressed(KeyCode::PageDown) {
                    scroll += tutorial::ROWS;
                }
                if keys.just_pressed(KeyCode::PageUp) {
                    scroll = scroll.saturating_sub(tutorial::ROWS);
                }
                if keys.just_pressed(KeyCode::Home) {
                    scroll = 0;
                }
                if keys.just_pressed(KeyCode::End) {
                    scroll = max;
                }
                tutorial_state.scroll = scroll.min(max);
                if enter || esc {
                    sel.close_card();
                    tutorial_state.scroll = 0;
                }

                // Left/Right closes the card and switches tabs.
                if keys.just_pressed(KeyCode::ArrowLeft) {
                    sel.close_card();
                    sel.top = (sel.top + TOP_COUNT - 1) % TOP_COUNT;
                    sel.open_initial();
                }
                if keys.just_pressed(KeyCode::ArrowRight) {
                    sel.close_card();
                    sel.top = (sel.top + 1) % TOP_COUNT;
                    sel.open_initial();
                }
            }
            Page::Exit => {
                if enter {
                    std::process::exit(0);
                }
                if esc {
                    sel.close_card();
                }

                // Left/Right closes the card and switches tabs.
                if keys.just_pressed(KeyCode::ArrowLeft) {
                    sel.close_card();
                    sel.top = (sel.top + TOP_COUNT - 1) % TOP_COUNT;
                    sel.open_initial();
                }
                if keys.just_pressed(KeyCode::ArrowRight) {
                    sel.close_card();
                    sel.top = (sel.top + 1) % TOP_COUNT;
                    sel.open_initial();
                }
            }
            Page::Info | Page::About => {
                if enter || esc {
                    sel.close_card();
                }

                // Left/Right closes the card and switches tabs.
                if keys.just_pressed(KeyCode::ArrowLeft) {
                    sel.close_card();
                    sel.top = (sel.top + TOP_COUNT - 1) % TOP_COUNT;
                    sel.open_initial();
                }
                if keys.just_pressed(KeyCode::ArrowRight) {
                    sel.close_card();
                    sel.top = (sel.top + 1) % TOP_COUNT;
                    sel.open_initial();
                }
            }
        }
        return;
    }

    // No card open: top-row navigation. Left/Right switches entries and
    // auto-opens the new entry's card; Enter re-opens the current card.
    if keys.just_pressed(KeyCode::ArrowLeft) {
        sel.top = (sel.top + TOP_COUNT - 1) % TOP_COUNT;
        sel.open_initial();
    }
    if keys.just_pressed(KeyCode::ArrowRight) {
        sel.top = (sel.top + 1) % TOP_COUNT;
        sel.open_initial();
    }
    if enter {
        sel.open_initial();
    }
}

/// Fly the selected aircraft: rebuild the FDM only if the aircraft changed,
/// then seat the spawn — a plain PLAY/FREE FLY parks on the runway, a lesson
/// mission moves to its own state (stand / threshold / final), each with the
/// engine plan the lesson prescribes.
fn start_flight(
    mut sim: ResMut<Sim>,
    mut ap: ResMut<Autopilot>,
    mut mcp: ResMut<Mcp>,
    mut bindings: ResMut<Bindings>,
    mut rp: ResMut<RenderPose>,
    mut active: ResMut<crate::mission::ActiveLesson>,
    sel: Res<MenuSelection>,
    config: Res<GameConfig>,
    pending: Res<PendingMission>,
    mut lesson_pending: ResMut<PendingLesson>,
) {
    let mission = pending.0;
    // Lesson missions are a C172 feature; a map mission wins regardless, and
    // the pending lesson is consumed either way so it never leaks. The env
    // hook `LESSON=taxi|takeoff|landing|final` (dev/screenshot harness) routes
    // the same way.
    let lesson = lesson_pending
        .0
        .take()
        .or_else(dev_lesson)
        .filter(|_| mission.is_none());
    *active = crate::mission::ActiveLesson(lesson);
    // Map missions fly the arcade F-22, airborne; every other entry uses the
    // selected aircraft. Missions always rebuild on the Rust engine, where
    // arcade dynamics exist.
    let id = mission.map_or_else(|| sel.aircraft_id(), |_| "f22");
    if mission.is_some() {
        *sim = Sim::new_arcade(id);
    } else if sim.name != id {
        *sim = Sim::new(id);
    }
    // The cold-start plan is the mission's, not a global setting: M1 starts
    // cold and dark, everything else runs. A fresh FDM starts with the flag
    // unset, so only the cold lessons need this.
    sim.set_engine_cold(active.0.is_some_and(|l| l.cold_start()));
    // Arcade is the mission's baseline; `config.arcade` only reports aircraft
    // that still offer the switch, so the F-22's FREE FLY is always realistic.
    let arcade =
        mission.is_some() || config.arcade(id) || std::env::var("ARCADE").is_ok_and(|v| v == "1");
    sim.model.set("fcs/arcade-enable", arcade as i32 as f64);
    if let Some(map) = mission {
        // Seat the IC with arcade dynamics already on, so the trim solver
        // seats the arcade jet rather than the realistic one.
        sim.reset_air(map.spawn_alt_ft(), map.spawn_kts());
    } else {
        match active.0 {
            Some(Lesson::Taxi) | Some(Lesson::JetTaxi) => sim.reset_stand(),
            Some(Lesson::Takeoff) => sim.reset_ground(),
            Some(Lesson::JetTakeoff) => sim.reset_threshold(),
            Some(Lesson::JetLanding) => sim.reset_final_jet(),
            Some(Lesson::Landing) => sim.reset_final_c172(),
            None => sim.reset_ground(),
        }
    }
    // Development hooks for screenshots: start airborne and optionally engage
    // the AH-64D auto-hover, without having to drive the menu/keys.
    if std::env::var("AIRBORNE").is_ok() {
        sim.reset_air(1500.0, 60.0);
    }
    // Seat on a 5 km final for runway 36 with the gear down — the ILS/APP
    // practice start.
    if std::env::var("FINAL").is_ok() {
        sim.reset_final();
    }
    if std::env::var("AHOVER").is_ok() {
        sim.model.set("fcs/heli-hover-enable", 1.0);
    }
    // The FEEL page's multipliers ride on: the flight is built with the
    // pilot's dialed keyboard feel, not the raw defaults.
    let mut controls = Bindings::for_aircraft_with(id, &config.feel(id));
    if arcade {
        controls.enable_arcade(id);
    }
    // The landing lesson flies flaps 20: the app writes its bindings every
    // step, so the flap lever and displays hold the approach detent from the
    // first frame (the FDM seat itself places command + position after IC —
    // see `sim::reset_final_at`).
    if matches!(active.0, Some(Lesson::Landing)) {
        if let Some(b) = controls
            .list
            .iter_mut()
            .find(|b| b.action == crate::aircraft_profile::Action::Flap)
        {
            b.value = 0.67;
        }
    }
    // The jet taxi lesson starts at idle with the parking brake holding her:
    // the profile's 60% taxi throttle would have the 737 rolling before the
    // first frame.
    if matches!(active.0, Some(Lesson::JetTaxi)) {
        if let Some(b) = controls
            .list
            .iter_mut()
            .find(|b| b.action == crate::aircraft_profile::Action::Throttle)
        {
            b.value = 0.0;
        }
    }
    // The jet landing lesson arrives at flaps 5. The pinned throttle is the
    // 3-degree beam's own book value (~0.25: the descent's gravity share
    // carries most of the drag), not level power - level power (~0.5) had
    // the APP-coupled seat accelerating down the beam. Dirty-ing up then
    // decelerates her into the ladder's targets on its own.
    if matches!(active.0, Some(Lesson::JetLanding)) {
        for b in controls.list.iter_mut() {
            match b.action {
                crate::aircraft_profile::Action::Flap => b.value = 5.0 / 40.0,
                crate::aircraft_profile::Action::Throttle => b.value = 0.25,
                _ => {}
            }
        }
    }
    *bindings = controls;
    // Every flight starts under manual control with the MCP at its defaults;
    // the 737's autopilot can be engaged once airborne.
    ap.reset();
    mcp.reset();
    // The arrival's own speed plan in the window, so an A/T engage (5) holds
    // the seat's 180 instead of the 250 default.
    if matches!(active.0, Some(Lesson::JetLanding)) {
        mcp.ias_kts = 180.0;
    }
    rp.init = false;
}

/// Development hook: `LESSON=taxi|takeoff|landing|jettaxi` parses to the
/// matching lesson, so screenshots can boot straight into a spawn without the
/// menu.
fn dev_lesson() -> Option<Lesson> {
    let key = std::env::var("LESSON").ok()?;
    let key = key.to_ascii_uppercase();
    match key.as_str() {
        "TAXI" | "M1" => Some(Lesson::Taxi),
        "TAKEOFF" | "M2" => Some(Lesson::Takeoff),
        "LANDING" | "M3" => Some(Lesson::Landing),
        "JETTAXI" | "737TAXI" => Some(Lesson::JetTaxi),
        "JETTAKEOFF" | "737TAKEOFF" => Some(Lesson::JetTakeoff),
        "JETLANDING" | "737LANDING" => Some(Lesson::JetLanding),
        _ => None,
    }
}

/// Showcase camera: ease toward a front-3/4 framing of the selected model, or a
/// wide pose for the no-model entries. The only camera driver in `Menu`
/// (`view::follow_camera` is gated to `Flying`).
fn menu_camera(
    time: Res<Time>,
    sel: Res<MenuSelection>,
    mut look: ResMut<MenuCamLook>,
    mut q: Query<&mut Transform, (With<Camera3d>, Without<crate::cloud_camera::CloudCamera>)>,
) {
    let Ok(mut t) = q.single_mut() else {
        return;
    };
    let (eye, aim) = match sel.aircraft_index() {
        Some(i) => {
            let a = &AIRCRAFT[i];
            let p = showcase_pos(i) + Vec3::Y * a.root_y;
            // ~29 degrees off the nose, ~17 degrees up; models face -Z, so the
            // eye sits on the -Z side.
            let (az, el) = (0.5_f32, 0.30_f32);
            let horiz = a.frame_dist * el.cos();
            let eye = p + Vec3::new(horiz * az.sin(), a.frame_dist * el.sin(), -horiz * az.cos());
            (eye, p + Vec3::Y * a.frame_h)
        }
        None => (WIDE_EYE, WIDE_TARGET),
    };
    // Frame-rate independent exponential smoothing, same form as `follow_camera`.
    let alpha = 1.0 - (-MENU_CAM_RATE * time.delta_secs()).exp();
    t.translation = t.translation.lerp(eye, alpha);
    look.0 = look.0.lerp(aim, alpha);
    t.look_at(look.0, Vec3::Y);
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_f22_offers_free_fly_and_missions_while_other_aircraft_keep_play() {
        assert_eq!(
            sub_items(F22_INDEX),
            &["FREE FLY", "MISSIONS", "TUTORIAL", "INFO"]
        );
        for (top, aircraft) in AIRCRAFT.iter().enumerate() {
            match aircraft.id {
                "c172p" => assert_eq!(
                    sub_items(top),
                    &["PLAY", "TUTORIAL", "INFO", "SETTINGS", "FEEL"],
                    "{}",
                    aircraft.id
                ),
                "737" => assert_eq!(
                    sub_items(top),
                    &["PLAY", "TUTORIAL", "INFO", "FEEL"],
                    "{}",
                    aircraft.id
                ),
                "ah1s" => assert_eq!(
                    sub_items(top),
                    &["PLAY", "TUTORIAL", "INFO", "SETTINGS"],
                    "{}",
                    aircraft.id
                ),
                // The F-22's combat maps are fixed arcade fare; no feel dial.
                "f22" => assert!(!sub_items(top).contains(&"FEEL"), "{}", aircraft.id),
                _ => assert!(!sub_items(top).contains(&"SETTINGS"), "{}", aircraft.id),
            }
        }
        assert!(sub_items(SETTINGS_TOP).is_empty());
        assert!(sub_items(ABOUT_TOP).is_empty());
        assert!(sub_items(EXIT_TOP).is_empty());
    }

    /// The lit `Option` row of a page model, for the given label.
    fn option_row<'a>(lines: &'a [PageLine], label: &str) -> &'a PageLine {
        lines
            .iter()
            .find(
                |l| l.kind == menu_page::LineKind::Option && l.left == label,
            )
            .unwrap_or_else(|| panic!("no option row {label}"))
    }

    #[test]
    fn the_missions_page_lists_every_map_in_order() {
        let lines = missions_lines(&MissionState { row: 1 });
        assert_eq!(lines[0].left, "F-22 - MISSIONS");
        for (i, label) in map::MAP_LABELS.iter().enumerate() {
            let row = option_row(&lines, label);
            assert_eq!(row.active, i == 1, "row {label}");
        }
    }

    #[test]
    fn the_play_page_lists_lessons_for_the_cessna_only() {
        let mine = crate::mission::PLAY_MENU[3]
            .lesson
            .expect("M3 LANDING is a lesson");
        assert_eq!(mine.label(), "M3 LANDING");
        assert!(!mine.cold_start());
        assert_eq!(crate::mission::PLAY_MENU[0].lesson, None);
    }

    #[test]
    fn the_play_page_text_marks_the_cursor_and_hides_lessons_off_the_graded_two() {
        let out = play_lines("c172p", &PlayState { row: 2 });
        assert!(out.iter().any(|l| l.kind == menu_page::LineKind::Heading
            && l.left.contains("SELECT START")),
            "{out:?}");
        assert!(option_row(&out, "M1 TAXI").right.is_empty());
        assert!(option_row(&out, "M3 LANDING").right.is_empty());
        assert!(option_row(&out, "M2 TAKEOFF").active, "cursor on row 2");
        let other = play_lines("ah1s", &PlayState::default());
        assert!(option_row(&other, "FREE FLIGHT").active);
        assert!(other.iter().all(|l| !l.left.starts_with('M')), "{other:?}");
        assert!(
            other.iter().any(|l| l.left.contains("Cessna 172 and the 737")),
            "{other:?}"
        );
        // The 737's own PLAY page: free flight plus its taxi lesson, no note.
        let jet = play_lines("737", &PlayState { row: 1 });
        assert!(option_row(&jet, "M1 TAXI").active, "cursor on the lesson");
        assert!(option_row(&jet, "FREE FLIGHT").right.is_empty());
        assert!(!jet.iter().any(|l| l.left.contains("Cessna 172")), "{jet:?}");
    }

    #[test]
    fn only_the_apache_has_a_flight_model_row() {
        assert!(!has_flight_model_row("f22"));
        assert!(has_flight_model_row("ah1s"));
        assert!(!has_flight_model_row("c172p"));
        assert!(!has_flight_model_row("737"));
        assert_eq!(settings_row_count("ah1s"), 1);
        assert_eq!(settings_row_count("c172p"), 2);
        assert_eq!(settings_row_count("f22"), 0);
        assert_eq!(settings_row_count("737"), 0);
    }

    #[test]
    fn the_apache_page_renders_the_flight_model_switch() {
        let mut config = GameConfig::default();
        let lines = aircraft_settings_lines("ah1s", &config, &AircraftSettings::default());
        assert_eq!(option_row(&lines, "FLIGHT MODEL").right, "Realistic");

        config.set_arcade("ah1s", true);
        let lines = aircraft_settings_lines("ah1s", &config, &AircraftSettings::default());
        assert_eq!(option_row(&lines, "FLIGHT MODEL").right, "Arcade");
    }

    #[test]
    fn the_c172_page_has_the_two_assist_rows() {
        let config = GameConfig::default();
        let lines = aircraft_settings_lines("c172p", &config, &AircraftSettings::default());
        assert_eq!(option_row(&lines, "BANK HOLD").right, "OFF");
        assert_eq!(option_row(&lines, "TURN COORD").right, "OFF");
        assert!(lines.iter().all(|l| l.left != "FLIGHT MODEL"));
    }

    #[test]
    fn the_global_settings_row_tracks_the_cursor() {
        use crate::weather::Weather;
        let lines = global_settings_lines(&Weather::default(), &SettingsState { row: 2 });
        assert_eq!(lines[0].left, "GLOBAL SETTINGS");
        assert!(option_row(&lines, "TIME OF DAY").active);
        assert!(!option_row(&lines, "WEATHER").active);
    }

    #[test]
    fn the_c172_feel_rows_cover_every_knob() {
        let rows = feel_rows("c172p");
        let labels: Vec<&str> = rows.iter().map(|r| r.label).collect();
        assert_eq!(
            labels,
            &[
                "ELEVATOR TRAVEL",
                "AILERON TRAVEL",
                "RUDDER TRAVEL",
                "THROTTLE SWEEP",
                "TAP NUDGE",
                "RELEASE LINGER",
                "RETURN SPEED",
                "FINE RATE",
                "RESTORE ALL",
            ]
        );
        // The 737 has no tap or fine knobs (no detent-free twitches), but does
        // dial its travel, linger and return.
        let jet: Vec<&str> = feel_rows("737").iter().map(|r| r.label).collect();
        assert!(!jet.contains(&"TAP NUDGE"));
        assert!(!jet.contains(&"FINE RATE"));
        assert!(jet.contains(&"RELEASE LINGER"));
        assert_eq!(jet.len(), 7);
    }

    #[test]
    fn feel_baselines_show_the_stock_numbers() {
        let b = Bindings::feel_baseline("c172p");
        assert!((b.travel_elevator - 1.0).abs() < 1e-9, "{}", b.travel_elevator);
        assert!((b.travel_aileron - 0.5).abs() < 1e-9, "{}", b.travel_aileron);
        assert!((b.travel_rudder - 1.0 / 1.5).abs() < 1e-9, "{}", b.travel_rudder);
        assert!((b.travel_throttle - 2.0).abs() < 1e-9, "{}", b.travel_throttle);
        assert!((b.tap_step - 0.04).abs() < 1e-9);
        assert!((b.release_hold - 0.35).abs() < 1e-9);
        assert!((b.release_scale - 0.8).abs() < 1e-9);
        assert!((b.fine_scale - 0.25).abs() < 1e-9);
        assert!(b.tap_enabled && b.fine_enabled);

        // The 737's hydraulic feel: slower surfaces, no tap/fine.
        let j = Bindings::feel_baseline("737");
        assert!((j.travel_elevator - 1.0 / 0.85).abs() < 1e-9, "{}", j.travel_elevator);
        assert!((j.travel_aileron - 1.0).abs() < 1e-9);
        assert!(!j.tap_enabled && !j.fine_enabled);
        assert!((j.release_hold - 0.2).abs() < 1e-9);
    }

    #[test]
    fn the_feel_page_renders_absolute_values_with_stock_brackets() {
        let mut config = GameConfig::default();
        config.set_feel("c172p", |f| {
            f.travel_elevator = 1.1;
            f.tap = 2.0;
        });
        let lines = feel_lines("c172p", &config, &FeelState { row: 2 });
        assert!(lines[0].left.ends_with("- CONTROL FEEL"), "{}", lines[0].left);
        assert_eq!(
            option_row(&lines, "ELEVATOR TRAVEL").right,
            "1.10 s   [stock 1.00 s]"
        );
        assert_eq!(
            option_row(&lines, "TAP NUDGE").right,
            "8 %   [stock 4 %]"
        );
        // The untouched knobs show their defaults, and the cursor sits on row
        // 2 (rudder travel).
        assert_eq!(
            option_row(&lines, "AILERON TRAVEL").right,
            "0.50 s   [stock 0.50 s]"
        );
        assert!(option_row(&lines, "RUDDER TRAVEL").active);
        assert!(!option_row(&lines, "ELEVATOR TRAVEL").active);
        assert_eq!(option_row(&lines, "RESTORE ALL").right, "WIPE EVERY KNOB");
    }

    #[test]
    fn feel_stepping_persists_round_values_and_r_resets_one_row() {
        let mut config = GameConfig::default();

        // Right on elevator travel: +0.1 s, straight into the persisted
        // multiplier (1.1, i.e. display 1.10 s). The on-disk format itself is
        // covered by the config round-trip tests.
        feel_change(&mut config, "c172p", FeelKnob::Travel(TravelAxis::Elevator), 1);
        assert!((config.feel("c172p").travel_elevator - 1.1).abs() < 1e-9);

        // Left twice on tap: 4 % -> 2 %.
        feel_change(&mut config, "c172p", FeelKnob::Tap, -1);
        feel_change(&mut config, "c172p", FeelKnob::Tap, -1);
        assert!((config.feel("c172p").tap - 0.5).abs() < 1e-9);

        // R puts the tapped row back; the travel change survives.
        feel_reset_row(&mut config, "c172p", FeelKnob::Tap);
        assert_eq!(config.feel("c172p").tap, 1.0);
        assert!((config.feel("c172p").travel_elevator - 1.1).abs() < 1e-9);

        // Resetting the last changed knob erases the entry entirely.
        feel_reset_row(&mut config, "c172p", FeelKnob::Travel(TravelAxis::Elevator));
        assert_eq!(config.feel("c172p"), FeelSetting::default());

        // RESTORE ALL behaves on a dialed feel the same way.
        config.set_feel("c172p", |f| f.linger = 3.0);
        config.reset_feel("c172p");
        assert_eq!(config.feel("c172p"), FeelSetting::default());
    }
}
