//! Start menu: a 3D hangar with all aircraft parked in a row and a text overlay.
//!
//! [`AppState`] gates the flight sim — every flight system carries
//! `run_if(in_state(AppState::Flying))`, so while the menu is up the sim sits
//! frozen. PLAY swaps in the selected aircraft's FDM and enters `Flying`.
//!
//! Navigation: Left/Right switch the top row, Down/Up move focus between the top
//! row and a sub-row, Enter activates, Escape closes a page / pops the focus.

use bevy::prelude::*;
use bevy::text::FontSize;

use crate::aircraft_profile::{self, Action, ToggleAction};
use crate::app_state::AppState;
use crate::config::{self, ConfigSink, GameConfig, Keymap};
use crate::input::Bindings;
use crate::autopilot::{Autopilot, Mcp};
use crate::map::{self, MapId, PendingMission};
use crate::sim::{RenderPose, Sim};
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
    MenuAircraft { id: "c172p", label: "CESSNA 172", root_y: 1.4326, frame_dist: 16.0, frame_h: 1.6 },
    MenuAircraft { id: "f22", label: "F-22", root_y: 1.6946, frame_dist: 24.0, frame_h: 2.0 },
    MenuAircraft { id: "ah1s", label: "AH-1S", root_y: 1.8288, frame_dist: 22.0, frame_h: 2.4 },
    MenuAircraft { id: "737", label: "737", root_y: 1.2446, frame_dist: 80.0, frame_h: 6.0 },
];
/// Top-row entries after the aircraft: the arcade missions, global settings,
/// then about.
const EXTRA_TOP: [&str; 3] = ["MISSIONS", "SETTINGS", "ABOUT"];
const MISSIONS_TOP: usize = AIRCRAFT.len();
const SETTINGS_TOP: usize = AIRCRAFT.len() + 1;
const ABOUT_TOP: usize = AIRCRAFT.len() + 2;
const TOP_COUNT: usize = AIRCRAFT.len() + EXTRA_TOP.len();
/// The F-22's row, shown behind the MISSIONS entry.
const F22_INDEX: usize = 1;
/// Number of pooled sub-row slots (the widest sub-row: PLAY / INFO / SETTINGS).
const SUB_SLOTS: usize = 3;
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
#[derive(Component)]
struct SubLabel(usize);
#[derive(Component)]
struct PanelRoot;
#[derive(Component)]
struct PanelText;

#[derive(Default, Clone, Copy, PartialEq, Eq)]
enum Focus {
    #[default]
    Top,
    Sub,
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum Page {
    About,
    Info,
    Settings,
    /// The global, aircraft-independent settings page (weather, ...).
    Global,
}

/// Rows on the global settings page. Only weather today.
const SETTINGS_ROWS: usize = 1;

/// Cursor within the global settings page.
#[derive(Resource, Default)]
struct SettingsState {
    row: usize,
}

/// Cursor and capture mode for the per-aircraft keymap editor (the aircraft
/// SETTINGS page).
#[derive(Resource, Default)]
struct EditorState {
    /// Selected row; clamped to the current aircraft's slot count.
    sel: usize,
    /// Waiting for the next key press to bind to the selected row.
    capturing: bool,
    /// Transient status line ("bound to W", "swapped with roll", ...).
    msg: String,
}

/// One editable key slot on the aircraft settings page. Paired axes contribute
/// a `Pos` and a `Neg` row; unpaired axes (brakes) just `Pos`; toggles one row.
#[derive(Clone, Copy, PartialEq, Eq)]
enum Slot {
    Pos(Action),
    Neg(Action),
    Toggle(ToggleAction),
}

/// The aircraft's editable slots, in display order, each with its label and
/// current effective key.
fn editor_slots(id: &str, keymap: &Keymap) -> Vec<(Slot, String, KeyCode)> {
    let bindings = Bindings::for_aircraft(id, keymap);
    let mut rows = Vec::new();
    for b in &bindings.list {
        if b.action.is_paired() {
            rows.push((Slot::Pos(b.action), b.action.label().to_owned(), b.pos));
            if let Some(neg) = b.neg {
                rows.push((Slot::Neg(b.action), b.action.label().to_owned(), neg));
            }
        } else {
            rows.push((Slot::Pos(b.action), b.action.label().to_owned(), b.pos));
        }
    }
    for t in &bindings.toggles {
        rows.push((Slot::Toggle(t.action), t.action.label().to_owned(), t.key));
    }
    rows
}

/// Whether the aircraft's settings page carries the Realistic/Arcade flight-model
/// switch as a synthetic row above the key slots.
fn has_flight_model_row(id: &str) -> bool {
    aircraft_profile::profile(id).has_arcade
}

/// Rows on the settings page: the flight-model switch (when present) plus the key
/// slots. `editor.sel` indexes this whole list, so the key rows are offset by the
/// header and `bind_key` never sees the synthetic row.
fn settings_row_count(id: &str, config: &GameConfig) -> usize {
    editor_slots(id, &config.keymap(id)).len() + has_flight_model_row(id) as usize
}

/// Menu-navigation keys can never be bound, or the page would be unusable.
fn reserved_key(k: KeyCode) -> bool {
    matches!(
        k,
        KeyCode::Escape
            | KeyCode::Enter
            | KeyCode::NumpadEnter
            | KeyCode::Tab
            | KeyCode::ArrowUp
            | KeyCode::ArrowDown
            | KeyCode::ArrowLeft
            | KeyCode::ArrowRight
    )
}

/// Where the cursor is: the highlighted top entry, the focus level, the sub
/// index, and any full-page overlay that is open.
#[derive(Resource, Default)]
struct MenuSelection {
    top: usize,
    focus: Focus,
    sub: usize,
    page: Option<Page>,
}

impl MenuSelection {
    /// The showcased aircraft, or `None` for the SETTINGS/ABOUT entries.
    /// MISSIONS shows the jet the missions fly.
    fn aircraft_index(&self) -> Option<usize> {
        if self.top == MISSIONS_TOP {
            return Some(F22_INDEX);
        }
        (self.top < AIRCRAFT.len()).then_some(self.top)
    }

    /// The selected aircraft's FDM id (only meaningful on an aircraft entry).
    fn aircraft_id(&self) -> &'static str {
        AIRCRAFT[self.top.min(AIRCRAFT.len() - 1)].id
    }

    /// Keep the sub index valid for the current top entry; entries with no
    /// sub-row can't hold sub focus.
    fn clamp_sub(&mut self) {
        let n = sub_items(self.top).len();
        if n == 0 {
            self.sub = 0;
            self.focus = Focus::Top;
        } else if self.sub >= n {
            self.sub = n - 1;
        }
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
/// SETTINGS opens its own page (Enter), ABOUT opens straight away.
fn sub_items(top: usize) -> &'static [&'static str] {
    match top {
        MISSIONS_TOP => &map::MAP_LABELS,
        t if t < AIRCRAFT.len() => &["PLAY", "INFO", "SETTINGS"],
        _ => &[],
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
            .init_resource::<EditorState>()
            .add_systems(Startup, auto_start_mission)
            .add_systems(
                OnEnter(AppState::Menu),
                (enter_menu, spawn_hangar, setup_menu_ui).chain(),
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
                    update_menu_ui.run_if(in_state(AppState::Menu)),
                    menu_camera.run_if(in_state(AppState::Menu)),
                    exit_to_menu.run_if(in_state(AppState::Flying)),
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

/// Select the aircraft matching the loaded sim (startup default or the one we
/// just flew) and reset the cursor.
fn enter_menu(mut sel: ResMut<MenuSelection>, sim: Res<Sim>) {
    let cur = if sim.name == "b737" { "737" } else { sim.name.as_str() };
    let top = AIRCRAFT.iter().position(|a| a.id == cur).unwrap_or(0);
    *sel = MenuSelection {
        top,
        focus: Focus::Top,
        sub: 0,
        page: None,
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

    let top_row = commands
        .spawn((
            Node {
                flex_direction: FlexDirection::Row,
                column_gap: Val::Px(28.0),
                margin: UiRect::bottom(Val::Px(18.0)),
                ..default()
            },
            ChildOf(root),
        ))
        .id();
    for i in 0..TOP_COUNT {
        commands.spawn((
            Text::new(""),
            TextFont {
                font_size: FontSize::Px(30.0),
                ..default()
            },
            TextColor(Color::WHITE),
            TopLabel(i),
            ChildOf(top_row),
        ));
    }

    let sub_row = commands
        .spawn((
            Node {
                flex_direction: FlexDirection::Row,
                column_gap: Val::Px(36.0),
                ..default()
            },
            ChildOf(root),
        ))
        .id();
    for i in 0..SUB_SLOTS {
        commands.spawn((
            Text::new(""),
            TextFont {
                font_size: FontSize::Px(22.0),
                ..default()
            },
            TextColor(Color::WHITE),
            SubLabel(i),
            ChildOf(sub_row),
        ));
    }

    // Full-page overlay, spawned after the rows so it draws on top of them.
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
                padding: UiRect::all(Val::Px(48.0)),
                display: Display::None,
                ..default()
            },
            BackgroundColor(Color::srgba(0.0, 0.0, 0.0, 0.82)),
            DespawnOnExit(AppState::Menu),
        ))
        .id();
    commands.spawn((
        Text::new(""),
        TextFont {
            font_size: FontSize::Px(20.0),
            ..default()
        },
        TextColor(Color::WHITE),
        PanelText,
        ChildOf(panel),
    ));
}

fn color_active() -> Color {
    Color::WHITE
}
fn color_selected() -> Color {
    Color::srgb(0.70, 0.78, 0.92)
}
fn color_dim() -> Color {
    Color::srgb(0.42, 0.47, 0.58)
}

/// Repaint the labels and the panel from the current selection. Cheap, and the
/// only reason one pooled row can serve both the aircraft (`PLAY/INFO/SETTINGS`)
/// and SETTINGS (`COMING SOON`) sub-rows.
fn update_menu_ui(
    sel: Res<MenuSelection>,
    weather: Res<Weather>,
    settings: Res<SettingsState>,
    config: Res<GameConfig>,
    editor: Res<EditorState>,
    mut tops: Query<(&TopLabel, &mut Text, &mut TextColor), (Without<SubLabel>, Without<PanelText>)>,
    mut subs: Query<(&SubLabel, &mut Text, &mut TextColor), (Without<TopLabel>, Without<PanelText>)>,
    mut panel: Query<&mut Node, With<PanelRoot>>,
    mut ptext: Query<&mut Text, (With<PanelText>, Without<TopLabel>, Without<SubLabel>)>,
) {
    let labels = top_labels();
    for (TopLabel(i), mut text, mut color) in &mut tops {
        let i = *i;
        text.0 = labels[i].to_string();
        color.0 = if i == sel.top {
            if sel.focus == Focus::Top {
                color_active()
            } else {
                color_selected()
            }
        } else {
            color_dim()
        };
    }

    let items = sub_items(sel.top);
    for (SubLabel(i), mut text, mut color) in &mut subs {
        let i = *i;
        match items.get(i) {
            Some(label) => {
                text.0 = (*label).to_string();
                color.0 = if sel.focus == Focus::Sub && i == sel.sub {
                    color_active()
                } else if i == sel.sub {
                    color_selected()
                } else {
                    color_dim()
                };
            }
            None => text.0.clear(),
        }
    }

    for mut node in &mut panel {
        node.display = if sel.page.is_some() {
            Display::Flex
        } else {
            Display::None
        };
    }
    if let Ok(mut text) = ptext.single_mut() {
        text.0 = match sel.page {
            Some(Page::Global) => global_settings_text(&weather, &settings),
            Some(Page::Settings) => editor_text(sel.aircraft_id(), &config, &editor),
            Some(p) => page_text(p, &sel),
            None => String::new(),
        };
    }
}

/// The aircraft's settings page: the Realistic/Arcade flight-model switch (F-22
/// only), then one row per rebindable key, the selected one marked, and a live
/// prompt while a capture is pending.
fn editor_text(id: &str, config: &GameConfig, editor: &EditorState) -> String {
    let slots = editor_slots(id, &config.keymap(id));
    let header = has_flight_model_row(id) as usize;
    let sel = editor.sel.min((header + slots.len()).saturating_sub(1));

    let mut out = format!("{} - SETTINGS\n\n", aircraft_label(id));
    if header == 1 {
        let cursor = if sel == 0 { ">" } else { " " };
        let model = if config.arcade(id) { "Arcade" } else { "Realistic" };
        out.push_str(&format!("{cursor} {:<10} < {model} >\n", "flight model"));
    }
    for (i, (_, label, key)) in slots.iter().enumerate() {
        let row = header + i;
        let cursor = if row == sel { ">" } else { " " };
        let shown = if editor.capturing && row == sel {
            "...".to_owned()
        } else {
            aircraft_profile::key_name(*key)
        };
        out.push_str(&format!("{cursor} {label:<10} {shown}\n"));
    }

    out.push('\n');
    if editor.capturing {
        out.push_str("Press a key to bind   [Esc] cancel\n");
    } else if header == 1 && sel == 0 {
        out.push_str("[Up/Down] select   [Left/Right] change   [Esc] close\n");
    } else {
        out.push_str("[Up/Down] select   [Enter] rebind   [Esc] close\n");
    }
    if !editor.msg.is_empty() {
        out.push_str(&format!("\n{}", editor.msg));
    }
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

/// The global settings page: one row per setting, the selected one marked and
/// its value in `< >` brackets.
fn global_settings_text(weather: &Weather, settings: &SettingsState) -> String {
    let mut out = String::from("GLOBAL SETTINGS\n\n");
    for i in 0..SETTINGS_ROWS {
        let cursor = if i == settings.row { ">" } else { " " };
        match i {
            0 => out.push_str(&format!("{cursor} WEATHER        < {} >\n", weather.label())),
            _ => {}
        }
    }
    out.push_str("\n\n[Up/Down] select   [Left/Right] change\n[Enter] or [Esc] to close");
    out
}

fn page_text(page: Page, sel: &MenuSelection) -> String {
    let close = "\n\n[Enter] or [Esc] to close";
    let label = AIRCRAFT[sel.aircraft_index().unwrap_or(0)].label;
    match page {
        Page::About => format!(
            "myFlightSim\n\n\
             A pure-Rust flight dynamics model driving Bevy-rendered aircraft.\n\
             The Cessna 172, F-22, AH-1S and 737 fly the same neutral property bus,\n\
             with a JSBSim backend available as a drop-in alternative.\n\n\
             Developer: Nanbert{close}"
        ),
        // The built-in font is ASCII-only, so no em dashes here (they render as
        // missing-glyph boxes).
        Page::Info => format!("{label} - INFO\n\nAircraft info coming soon.{close}"),
        // Rendered by `editor_text` / `global_settings_text`, which need live
        // state; `page_text` never sees these.
        Page::Settings | Page::Global => String::new(),
    }
}

/// Arrow/Enter/Escape navigation. Gated to `Menu`.
fn menu_input(
    keys: Res<ButtonInput<KeyCode>>,
    mut sel: ResMut<MenuSelection>,
    mut next: ResMut<NextState<AppState>>,
    mut weather: ResMut<Weather>,
    mut settings: ResMut<SettingsState>,
    mut editor: ResMut<EditorState>,
    mut config: ResMut<GameConfig>,
    mut pending: ResMut<PendingMission>,
    sink: Res<ConfigSink>,
) {
    let enter = keys.just_pressed(KeyCode::Enter) || keys.just_pressed(KeyCode::NumpadEnter);
    let esc = keys.just_pressed(KeyCode::Escape);

    // A page swallows navigation. GLOBAL is a settings list (Up/Down select,
    // Left/Right change); SETTINGS is the aircraft's keymap editor; the rest are
    // read-only. Enter/Escape dismiss the read-only pages.
    if let Some(page) = sel.page {
        match page {
            // The wrap arithmetic is a no-op while GLOBAL has its single row,
            // but it stays general so adding a row is a one-line change.
            #[allow(clippy::modulo_one)]
            Page::Global => {
                if keys.just_pressed(KeyCode::ArrowUp) {
                    settings.row = (settings.row + SETTINGS_ROWS - 1) % SETTINGS_ROWS;
                }
                if keys.just_pressed(KeyCode::ArrowDown) {
                    settings.row = (settings.row + 1) % SETTINGS_ROWS;
                }
                let dir = keys.just_pressed(KeyCode::ArrowRight) as isize
                    - keys.just_pressed(KeyCode::ArrowLeft) as isize;
                if dir != 0 {
                    match settings.row {
                        0 => {
                            let level = weather.level() as isize + dir;
                            weather.set_level(level);
                            config.weather_level = weather.level();
                            config::save(&config, &sink);
                        }
                        _ => {}
                    }
                }
                if enter || esc {
                    sel.page = None;
                }
            }
            Page::Settings => {
                let id = sel.aircraft_id();
                let header = has_flight_model_row(id) as usize;
                let n = settings_row_count(id, &config).max(1);
                editor.sel = editor.sel.min(n - 1);

                if editor.capturing {
                    // Enter/Escape cancel; any other key is captured (or
                    // rejected, with a note).
                    if enter || esc {
                        editor.capturing = false;
                        editor.msg.clear();
                    } else if let Some(&k) = keys.get_just_pressed().next() {
                        bind_key(&mut config, &sink, &mut editor, id, k);
                    }
                } else if keys.just_pressed(KeyCode::ArrowUp) {
                    editor.sel = (editor.sel + n - 1) % n;
                    editor.msg.clear();
                } else if keys.just_pressed(KeyCode::ArrowDown) {
                    editor.sel = (editor.sel + 1) % n;
                    editor.msg.clear();
                } else if editor.sel == 0 && header == 1 {
                    // The flight-model row: Left/Right flip it, Enter does not
                    // rebind. Persisted so the choice survives a restart.
                    let dir = keys.just_pressed(KeyCode::ArrowRight) as isize
                        - keys.just_pressed(KeyCode::ArrowLeft) as isize;
                    if dir != 0 {
                        let on = !config.arcade(id);
                        config.set_arcade(id, on);
                        config::save(&config, &sink);
                    }
                    if esc {
                        sel.page = None;
                    }
                } else if enter {
                    editor.capturing = true;
                    editor.msg.clear();
                } else if esc {
                    sel.page = None;
                }
            }
            Page::Info | Page::About => {
                if enter || esc {
                    sel.page = None;
                }
            }
        }
        return;
    }

    // Left/Right are context-sensitive: on the top row they switch entries, on
    // the sub-row they move within it. (Up/Down own the focus change, so the
    // sub-row needs the horizontal keys to be reachable at all.)
    if keys.just_pressed(KeyCode::ArrowLeft) {
        match sel.focus {
            Focus::Top => {
                sel.top = (sel.top + TOP_COUNT - 1) % TOP_COUNT;
                sel.clamp_sub();
            }
            Focus::Sub => {
                let n = sub_items(sel.top).len();
                if n > 0 {
                    sel.sub = (sel.sub + n - 1) % n;
                }
            }
        }
    }
    if keys.just_pressed(KeyCode::ArrowRight) {
        match sel.focus {
            Focus::Top => {
                sel.top = (sel.top + 1) % TOP_COUNT;
                sel.clamp_sub();
            }
            Focus::Sub => {
                let n = sub_items(sel.top).len();
                if n > 0 {
                    sel.sub = (sel.sub + 1) % n;
                }
            }
        }
    }
    if keys.just_pressed(KeyCode::ArrowDown) && !sub_items(sel.top).is_empty() {
        sel.focus = Focus::Sub;
    }
    if keys.just_pressed(KeyCode::ArrowUp) {
        sel.focus = Focus::Top;
    }
    if keys.just_pressed(KeyCode::Escape) && sel.focus == Focus::Sub {
        sel.focus = Focus::Top;
    }

    if enter {
        match sel.focus {
            Focus::Top => {
                if sel.top == SETTINGS_TOP {
                    sel.page = Some(Page::Global); // SETTINGS opens its page
                } else if sel.top == ABOUT_TOP {
                    sel.page = Some(Page::About); // ABOUT opens straight away
                } else if !sub_items(sel.top).is_empty() {
                    sel.focus = Focus::Sub;
                }
            }
            Focus::Sub => match sub_items(sel.top).get(sel.sub).copied() {
                Some("PLAY") => next.set(AppState::Flying),
                Some("INFO") => sel.page = Some(Page::Info),
                Some("SETTINGS") => {
                    sel.page = Some(Page::Settings);
                    editor.sel = 0;
                    editor.capturing = false;
                    editor.msg.clear();
                }
                Some(label) => {
                    if let Some(map) = MapId::from_label(label) {
                        pending.0 = Some(map);
                        next.set(AppState::Flying);
                    }
                }
                None => {}
            },
        }
    }
}

/// Bind `key` to the selected slot, then persist. Every slot stays bound:
/// if another slot already held `key` the two keys are swapped, so the map can
/// never end up with a duplicate or a missing key.
fn bind_key(
    config: &mut GameConfig,
    sink: &ConfigSink,
    editor: &mut EditorState,
    id: &str,
    key: KeyCode,
) {
    editor.capturing = false;
    if reserved_key(key) || !aircraft_profile::is_bindable(key) {
        editor.msg = format!("{} cannot be bound", aircraft_profile::key_name(key));
        return;
    }

    let slots = editor_slots(id, &config.keymap(id));
    // The settings page may carry a flight-model row above the key slots; the
    // cursor already excludes it (a capture can only target a key row).
    let sel = editor
        .sel
        .saturating_sub(has_flight_model_row(id) as usize)
        .min(slots.len().saturating_sub(1));
    let victim = slots
        .iter()
        .position(|(_, _, k)| *k == key)
        .filter(|&oi| oi != sel);

    let mut keys: Vec<KeyCode> = slots.iter().map(|(_, _, k)| *k).collect();
    if let Some(oi) = victim {
        keys.swap(sel, oi);
        editor.msg = format!("swapped with {}", slots[oi].1);
    } else {
        keys[sel] = key;
        editor.msg = format!("bound to {}", aircraft_profile::key_name(key));
    }

    // Rebuild the aircraft's keymap from the slot keys.
    let mut keymap = Keymap::default();
    let mut i = 0;
    while i < slots.len() {
        match slots[i].0 {
            Slot::Pos(action) => {
                let pos = keys[i];
                let neg = if slots.get(i + 1).map(|s| s.0) == Some(Slot::Neg(action)) {
                    i += 1;
                    Some(keys[i])
                } else {
                    None
                };
                keymap.set_axis(action, pos, neg);
            }
            Slot::Neg(action) => keymap.set_axis(action, keys[i], None),
            Slot::Toggle(action) => keymap.set_toggle(action, keys[i]),
        }
        i += 1;
    }

    *config.keymap_mut(id) = keymap;
    config::save(config, sink);
}

/// Fly the selected aircraft: rebuild the FDM only if the aircraft changed, then
/// start parked on the runway with neutral controls.
fn start_flight(
    mut sim: ResMut<Sim>,
    mut ap: ResMut<Autopilot>,
    mut mcp: ResMut<Mcp>,
    mut bindings: ResMut<Bindings>,
    mut rp: ResMut<RenderPose>,
    sel: Res<MenuSelection>,
    config: Res<GameConfig>,
    pending: Res<PendingMission>,
) {
    let mission = pending.0;
    // Missions fly the arcade F-22, airborne; every other entry uses the
    // selected aircraft and starts parked. Missions always rebuild on the
    // Rust engine, where arcade dynamics exist.
    let id = mission.map_or_else(|| sel.aircraft_id(), |_| "f22");
    if mission.is_some() {
        *sim = Sim::new_arcade(id);
    } else if sim.name != id {
        *sim = Sim::new(id);
    }
    // Arcade is the mission's baseline, and the per-aircraft setting the
    // player saved decides for every other flight. Only the F-22 reads the
    // property; every other model ignores it.
    let arcade = mission.is_some() || config.arcade(id);
    sim.model.set("fcs/arcade-enable", arcade as i32 as f64);
    if let Some(map) = mission {
        // Seat the IC with arcade dynamics already on, so the trim solver
        // seats the arcade jet rather than the realistic one.
        sim.reset_air(map.spawn_alt_ft(), map.spawn_kts());
    } else {
        sim.reset_ground();
    }
    let mut controls = Bindings::for_aircraft(id, &config.keymap(id));
    if arcade {
        controls.enable_arcade(id);
    }
    *bindings = controls;
    // Every flight starts under manual control with the MCP at its defaults;
    // the 737's autopilot can be engaged once airborne.
    ap.reset();
    mcp.reset();
    rp.init = false;
}

/// Escape while flying returns to the menu (the flight is discarded).
fn exit_to_menu(keys: Res<ButtonInput<KeyCode>>, mut next: ResMut<NextState<AppState>>) {
    if keys.just_pressed(KeyCode::Escape) {
        next.set(AppState::Menu);
    }
}

/// Showcase camera: ease toward a front-3/4 framing of the selected model, or a
/// wide pose for the no-model entries. The only camera driver in `Menu`
/// (`view::follow_camera` is gated to `Flying`).
fn menu_camera(
    time: Res<Time>,
    sel: Res<MenuSelection>,
    mut look: ResMut<MenuCamLook>,
    mut q: Query<&mut Transform, With<Camera3d>>,
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
            let eye = p
                + Vec3::new(
                    horiz * az.sin(),
                    a.frame_dist * el.sin(),
                    -horiz * az.cos(),
                );
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
    use crate::config::ConfigStore;

    struct NullStore;
    impl ConfigStore for NullStore {
        fn load(&self) -> Option<String> {
            None
        }
        fn save(&self, _: &str) {}
    }

    fn sink() -> ConfigSink {
        ConfigSink(Box::new(NullStore))
    }

    fn keys_of(id: &str, config: &GameConfig) -> Vec<KeyCode> {
        editor_slots(id, &config.keymap(id))
            .into_iter()
            .map(|(_, _, k)| k)
            .collect()
    }

    #[test]
    fn editor_lists_a_row_per_key_slot() {
        // c172p: 6 paired axes (12) + 2 brakes + 2 toggles.
        assert_eq!(editor_slots("c172p", &Keymap::default()).len(), 16);
        // ah1s: 5 paired axes (10) + 2 toggles (rotor-trim, trim).
        assert_eq!(editor_slots("ah1s", &Keymap::default()).len(), 12);
        // 737: c172p's slots + autopilot, gear, and the six MCP toggles.
        assert_eq!(editor_slots("737", &Keymap::default()).len(), 24);
    }

    #[test]
    fn only_the_f22_settings_page_gains_a_flight_model_row() {
        let config = GameConfig::default();
        // The row sits above the key slots, so the page has one more row.
        assert!(has_flight_model_row("f22"));
        assert_eq!(
            settings_row_count("f22", &config),
            editor_slots("f22", &Keymap::default()).len() + 1
        );
        // Every other aircraft's page is exactly its key slots.
        assert!(!has_flight_model_row("737"));
        assert_eq!(
            settings_row_count("737", &config),
            editor_slots("737", &Keymap::default()).len()
        );
    }

    #[test]
    fn the_flight_model_row_does_not_shift_a_f22_key_binding() {
        // The header offset must be invisible to `bind_key`: binding the F-22's
        // first key row (index 1, after the flight-model row) must land on
        // elevator+, not on some row off by one.
        let mut config = GameConfig::default();
        let mut editor = EditorState { capturing: true, sel: 1, ..default() };
        let s = sink();
        bind_key(&mut config, &s, &mut editor, "f22", KeyCode::Digit5);

        let b = Bindings::for_aircraft("f22", &config.keymap("f22"));
        let elev = b.list.iter().find(|b| b.action == Action::Elevator).unwrap();
        assert_eq!(elev.pos, KeyCode::Digit5, "elevator+ takes the new key");
    }

    #[test]
    fn the_f22_page_renders_the_flight_model_switch() {
        let editor = EditorState::default();
        let mut config = GameConfig::default();
        let realistic = editor_text("f22", &config, &editor);
        assert!(realistic.contains("flight model"), "{realistic}");
        assert!(realistic.contains("< Realistic >"), "{realistic}");

        config.set_arcade("f22", true);
        let arcade = editor_text("f22", &config, &editor);
        assert!(arcade.contains("< Arcade >"), "{arcade}");
        assert!(!arcade.contains("Realistic"), "{arcade}");

        // A non-arcade aircraft's page has no such row.
        let c172 = editor_text("c172p", &config, &editor);
        assert!(!c172.contains("flight model"), "{c172}");
    }

    #[test]
    fn binding_an_occupied_key_swaps_without_losing_or_duplicating() {
        let mut config = GameConfig::default();
        let mut editor = EditorState { capturing: true, ..default() };
        let s = sink();

        // Row 0 is elevator+ (default S). Assign it aileron+'s default D.
        let before = keys_of("c172p", &config);
        bind_key(&mut config, &s, &mut editor, "c172p", KeyCode::KeyD);

        let after = keys_of("c172p", &config);
        assert_eq!(after[0], KeyCode::KeyD, "target takes the new key");
        assert_eq!(after[2], before[0], "victim takes the target's old key");
        assert_eq!(after.len(), before.len(), "no slot added or dropped");

        // A swap only permutes: the multiset of keys is unchanged, so nothing is
        // lost or invented. (Defaults already share a key: both brakes on B.)
        let mut want: Vec<String> = before.iter().map(|k| format!("{k:?}")).collect();
        let mut got: Vec<String> = after.iter().map(|k| format!("{k:?}")).collect();
        want.sort();
        got.sort();
        assert_eq!(got, want, "swap must only permute the keys");
    }

    #[test]
    fn binding_to_a_free_key_keeps_the_old_one_released() {
        let mut config = GameConfig::default();
        let mut editor = EditorState { capturing: true, ..default() };
        let s = sink();

        // Numpad5 is bindable (Digit5) but unused, so nothing is displaced.
        let before = keys_of("c172p", &config);
        bind_key(&mut config, &s, &mut editor, "c172p", KeyCode::Digit5);
        let after = keys_of("c172p", &config);

        assert_eq!(after[0], KeyCode::Digit5);
        assert!(!after.contains(&before[0]), "the released key is gone");
        assert_eq!(after.len(), before.len());
    }

    #[test]
    fn reserved_and_unbindable_keys_are_rejected() {
        let mut config = GameConfig::default();
        let mut editor = EditorState { capturing: true, ..default() };
        let s = sink();
        let before = config.keymap("c172p");

        bind_key(&mut config, &s, &mut editor, "c172p", KeyCode::ArrowUp); // nav
        bind_key(&mut config, &s, &mut editor, "c172p", KeyCode::F12); // not in the table
        assert_eq!(config.keymap("c172p"), before, "rejected keys change nothing");
        assert!(!editor.msg.is_empty(), "a rejection is reported");
    }

    #[test]
    fn bindings_follow_the_edited_keymap() {
        let mut config = GameConfig::default();
        let mut editor = EditorState { capturing: true, ..default() };
        let s = sink();
        // Move the 737's gear toggle.
        let rows = editor_slots("737", &config.keymap("737"));
        let gear_row = rows
            .iter()
            .position(|(slot, _, _)| *slot == Slot::Toggle(ToggleAction::Gear))
            .unwrap();
        editor.sel = gear_row;
        bind_key(&mut config, &s, &mut editor, "737", KeyCode::KeyK);

        let b = Bindings::for_aircraft("737", &config.keymap("737"));
        let gear = b
            .toggles
            .iter()
            .find(|t| t.action == ToggleAction::Gear)
            .unwrap();
        assert_eq!(gear.key, KeyCode::KeyK);
    }
}
