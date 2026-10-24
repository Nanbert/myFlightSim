//! Clickable Mode Control Panel for the 737, drawn as a baked-image strip along
//! the bottom of the screen. The four live setpoint windows are redrawn with
//! seven-segment glyphs; invisible UI hotspots over the baked knobs and buttons
//! carry the `Mcp`/`Autopilot` wiring.
//!
//! The strip is the chase-view instrument. In the cockpit the pilot clicks the
//! modelled board instead, and `mcp_panel` raises the same baked image as a
//! centred 2D panel. This module owns the shared wiring: the `Mcp` setpoints,
//! the hotspot components, the press/hold/scroll systems, and the shared
//! `spawn_panel_contents` that lays windows, labels and hotspots over the image
//! at any scale. `mcp_layout` owns the asset-pixel coordinates; `mcp_bake`
//! produced the PNG from the old 3D model.

use bevy::input::mouse::MouseWheel;
use bevy::picking::hover::Hovered;
use bevy::picking::Pickable;
use bevy::prelude::*;
use bevy::ui::Pressed;
use bevy::ui_widgets::{Activate, Button};
use bevy::window::PrimaryWindow;

use crate::aircraft_profile::{self, ToggleAction};
use crate::app_state::{unpaused, AppState};
use crate::autopilot::{AtMode, Autopilot, LatMode, Mcp, McpWindow, SpeedUnit, VertMode};
use crate::mcp_layout::{
    cap_font, glass, slots, McpPress, PRect, AT_SWITCH, BUTTONS, COURSE_L_WINDOW, COURSE_R_WINDOW,
    CROP_W, DECOR_CAPS, DECOR_ROUND, DISENGAGE, EXTRA_LABELS, KNOBS, LABEL, PANEL_H, PANEL_W,
    VS_WHEEL, WINDOWS,
};
use crate::sim::Sim;
use crate::ui_theme;
use crate::view::Ui;

/// Red used for the focus tooltip's outline.
const FOCUS_RED: Color = Color::srgba(1.0, 0.30, 0.20, 1.0);

pub(crate) const FMA_BG: Color = Color::srgb(0.01, 0.01, 0.01);
pub(crate) const FMA_GREEN: Color = Color::srgb(0.25, 0.90, 0.30);
const FMA_AMBER: Color = Color::srgb(0.95, 0.66, 0.16);
const FMA_GREY: Color = Color::srgb(0.62, 0.65, 0.67);
const LED: Color = Color::srgb(0.86, 0.93, 1.0);
const LED_GLOW: Color = Color::srgba(0.45, 0.65, 1.0, 0.35);
const AMBER: Color = Color::srgb(1.0, 0.62, 0.12);

/// How long an arrow must be held before it starts repeating, and the repeat
/// period after that.
const HOLD_DELAY_S: f64 = 0.40;
const HOLD_INTERVAL_S: f64 = 0.05;

/// One half of a knob (or the upper/lower half of the V/S wheel).
#[derive(Component, Clone, Copy)]
pub(crate) struct McpArrow {
    pub(crate) window: McpWindow,
    pub(crate) dir: f64,
}

/// The clickable window body (wheel target).
#[derive(Component, Clone, Copy)]
pub(crate) struct McpWindowBox(pub(crate) McpWindow);

/// Which bar of a seven-segment glyph a node draws.
#[derive(Clone, Copy)]
pub(crate) enum Seg {
    A,
    B,
    C,
    D,
    E,
    F,
    G,
}

/// One light-emitting node inside a window.
#[derive(Component, Clone, Copy)]
pub(crate) enum LedPart {
    Seg {
        window: McpWindow,
        slot: usize,
        seg: Seg,
    },
    Dot {
        window: McpWindow,
        slot: usize,
    },
    /// A plus/minus bar in the leading slot of the V/S window.
    Sign {
        horizontal: bool,
    },
}

#[derive(Component)]
pub(crate) struct FmaLabel;

/// The 2D baked-image panel — the chase-view strip.
#[derive(Component)]
struct McpPanel;

/// Hover text for one hotspot.
#[derive(Component, Clone, Copy)]
pub(crate) struct McpHelp {
    title: &'static str,
    body: &'static str,
}

/// "MCP focus" hint shown above the cockpit panel.
#[derive(Component)]
struct FocusHint;

/// The hover tooltip panel (child text carries `TooltipText`).
#[derive(Component)]
struct HotspotTooltip;

#[derive(Component)]
struct TooltipText;

/// Which arrow is currently held, for click-and-hold repeating.
#[derive(Resource, Default)]
pub(crate) struct ArrowHold {
    entity: Option<Entity>,
    held_s: f64,
}

/// Eased 0..1 focus transition: 0 = chase strip / cockpit, 1 = the cockpit
/// panel is fully raised. `mcp_panel` drives it; the tooltip uses it to wait
/// for the panel.
#[derive(Resource, Default)]
pub(crate) struct FocusAnim {
    pub(crate) e: f32,
}

pub struct McpUiPlugin;

impl Plugin for McpUiPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<ArrowHold>()
            .init_resource::<FocusAnim>()
            .add_observer(on_mcp_activate)
            .add_systems(
                OnEnter(AppState::Flying),
                spawn_panel.after(crate::sim::StartFlight),
            )
            .add_systems(
                Update,
                (
                    press_arrows,
                    scroll_windows,
                    focus_input,
                    focus_hint,
                    focus_tooltip,
                    sync_panel,
                    cockpit_visibility,
                )
                    .chain()
                    .run_if(unpaused),
            );
    }
}

fn spawn_panel(mut commands: Commands, sim: Res<Sim>, assets: Res<AssetServer>) {
    if !aircraft_profile::profile(&sim.name).has_autopilot {
        return;
    }
    let mcp_image = assets.load("ui/737_mcp_3d.png");

    commands
        .spawn((
            McpPanel,
            Node {
                position_type: PositionType::Absolute,
                left: Val::Percent(50.0),
                margin: UiRect::left(Val::Px(-0.5 * PANEL_W)),
                bottom: Val::Px(12.0),
                width: Val::Px(PANEL_W),
                flex_direction: FlexDirection::Column,
                row_gap: Val::Px(4.0),
                ..default()
            },
            UiTransform::IDENTITY,
            BoxShadow::new(
                Color::srgba(0.0, 0.0, 0.0, 0.65),
                Val::Px(0.0),
                Val::Px(6.0),
                Val::Px(0.0),
                Val::Px(18.0),
            ),
            GlobalZIndex(15),
            DespawnOnExit(AppState::Flying),
        ))
        .with_children(|root| {
            root.spawn((
                Node {
                    width: Val::Percent(100.0),
                    height: Val::Px(22.0),
                    align_items: AlignItems::Center,
                    padding: UiRect::horizontal(Val::Px(8.0)),
                    border_radius: BorderRadius::all(Val::Px(3.0)),
                    ..default()
                },
                BackgroundColor(FMA_BG),
            ))
            .with_children(|bar| {
                bar.spawn((
                    Text::new("MANUAL"),
                    TextFont {
                        font_size: FontSize::Px(15.0),
                        ..default()
                    },
                    TextColor(FMA_GREEN),
                    FmaLabel,
                ));
            });

            root.spawn((
                ImageNode::new(mcp_image),
                Node {
                    width: Val::Px(PANEL_W),
                    height: Val::Px(PANEL_H),
                    ..default()
                },
            ))
            .with_children(|panel| spawn_panel_contents(panel, PANEL_W / CROP_W));
        });

    // Focus hint. It floats over the cockpit panel that `mcp_panel` raises.
    commands
        .spawn((
            FocusHint,
            Pickable::IGNORE,
            Node {
                position_type: PositionType::Absolute,
                bottom: Val::Px(24.0),
                width: Val::Percent(100.0),
                justify_content: JustifyContent::Center,
                display: Display::None,
                ..default()
            },
            GlobalZIndex(32),
            DespawnOnExit(AppState::Flying),
        ))
        .with_children(|hint| {
            hint.spawn((
                Pickable::IGNORE,
                Text::new("MCP FOCUS  -  ESC or click outside to close"),
                TextFont {
                    font_size: FontSize::Px(15.0),
                    ..default()
                },
                TextColor(Color::srgba(1.0, 0.82, 0.55, 0.95)),
            ));
        });

    commands
        .spawn((
            HotspotTooltip,
            Pickable::IGNORE,
            Node {
                position_type: PositionType::Absolute,
                padding: UiRect::all(Val::Px(7.0)),
                border: UiRect::all(Val::Px(1.5)),
                border_radius: BorderRadius::all(Val::Px(3.0)),
                max_width: Val::Px(340.0),
                display: Display::None,
                ..default()
            },
            BorderColor::all(FOCUS_RED),
            BackgroundColor(Color::srgba(0.03, 0.02, 0.02, 0.92)),
            GlobalZIndex(33),
            DespawnOnExit(AppState::Flying),
        ))
        .with_children(|tip| {
            tip.spawn((
                TooltipText,
                Pickable::IGNORE,
                Text::new(""),
                TextFont {
                    font_size: FontSize::Px(14.0),
                    ..default()
                },
                TextColor(Color::srgb(1.0, 0.92, 0.88)),
                TextLayout::new(Justify::Left, LineBreak::WordBoundary),
            ));
        });
}

/// Lay the live windows, engraved labels and click hotspots over the baked
/// image. `scale` is on-screen px per asset px; the cockpit panel passes 1.0
/// and scales the whole node instead.
pub(crate) fn spawn_panel_contents(panel: &mut ChildSpawnerCommands, scale: f32) {
    for window in [
        McpWindow::Speed,
        McpWindow::Heading,
        McpWindow::Altitude,
        McpWindow::VerticalSpeed,
    ] {
        spawn_window(panel, window, scale);
    }
    spawn_labels(panel, scale);
    spawn_hotspots(panel, scale);
}

/// Lay out the live seven-segment glyphs on the baked glass.
fn spawn_window(panel: &mut ChildSpawnerCommands, window: McpWindow, scale: f32) {
    let (gx, gy, gw, gh) = glass(window).scaled(scale);

    // The baked digits are ~40 asset px apart and ~48 px tall; keep that pitch
    // so the redrawn glyphs read at the same scale as the panel.
    let count = slots(window);
    let pitch = 40.0 * scale;
    let dw = 30.0 * scale;
    let dh = 48.0 * scale;
    let stroke = (6.2 * scale).max(1.5);
    let right_margin = 4.0 * scale;

    for slot in 0..count {
        let group_left = gx + gw - right_margin - pitch * count as f32;
        let dx = group_left + slot as f32 * pitch + (pitch - dw) * 0.5;
        let dy = gy + (gh - dh) * 0.5;

        for (seg, left, top, width, height) in segment_rects(dw, dh, stroke) {
            panel.spawn((
                Node {
                    position_type: PositionType::Absolute,
                    left: Val::Px(dx + left),
                    top: Val::Px(dy + top),
                    width: Val::Px(width),
                    height: Val::Px(height),
                    border_radius: BorderRadius::all(Val::Px(stroke * 0.45)),
                    ..default()
                },
                BackgroundColor(LED),
                BoxShadow::new(
                    LED_GLOW,
                    Val::Px(0.0),
                    Val::Px(0.0),
                    Val::Px(0.5),
                    Val::Px(3.0),
                ),
                LedPart::Seg { window, slot, seg },
                Visibility::Hidden,
            ));
        }

        panel.spawn((
            Node {
                position_type: PositionType::Absolute,
                left: Val::Px(dx + dw - stroke),
                top: Val::Px(dy + dh - stroke),
                width: Val::Px(stroke),
                height: Val::Px(stroke),
                border_radius: BorderRadius::all(Val::Px(stroke)),
                ..default()
            },
            BackgroundColor(LED),
            LedPart::Dot { window, slot },
            Visibility::Hidden,
        ));

        if window == McpWindow::VerticalSpeed && slot == 0 {
            for horizontal in [true, false] {
                let (left, top, width, height) = if horizontal {
                    (dx, dy + dh * 0.5 - stroke * 0.5, dw, stroke)
                } else {
                    (dx + dw * 0.5 - stroke * 0.5, dy, stroke, dh)
                };
                panel.spawn((
                    Node {
                        position_type: PositionType::Absolute,
                        left: Val::Px(left),
                        top: Val::Px(top),
                        width: Val::Px(width),
                        height: Val::Px(height),
                        border_radius: BorderRadius::all(Val::Px(stroke * 0.45)),
                        ..default()
                    },
                    BackgroundColor(LED),
                    LedPart::Sign { horizontal },
                    Visibility::Hidden,
                ));
            }
        }
    }
}

/// `(segment, left, top, width, height)` inside a glyph box.
pub(crate) fn segment_rects(w: f32, h: f32, s: f32) -> [(Seg, f32, f32, f32, f32); 7] {
    let half = h * 0.5;
    [
        (Seg::A, 0.0, 0.0, w, s),
        (Seg::F, 0.0, s * 0.6, s, half - s * 1.2),
        (Seg::B, w - s, s * 0.6, s, half - s * 1.2),
        (Seg::G, 0.0, half - s * 0.5, w, s),
        (Seg::E, 0.0, half + s * 0.6, s, half - s * 1.2),
        (Seg::C, w - s, half + s * 0.6, s, half - s * 1.2),
        (Seg::D, 0.0, h - s, w, s),
    ]
}

/// Every engraved label is a `Text` overlay: the bake carries hardware only, so
/// the same label table reads the same on the strip and the cockpit panel.
fn spawn_labels(panel: &mut ChildSpawnerCommands, scale: f32) {
    for &(rect, _, label) in BUTTONS {
        spawn_label(
            panel,
            label,
            rect.cx(),
            rect.cy(),
            cap_font(label),
            LABEL,
            scale,
        );
    }
    for &(rect, label) in DECOR_CAPS {
        spawn_label(
            panel,
            label,
            rect.cx(),
            rect.cy(),
            cap_font(label),
            LABEL,
            scale,
        );
    }
    for &(rect, label) in DECOR_ROUND {
        spawn_label(panel, label, rect.cx(), rect.y0 - 12.0, 20.0, LABEL, scale);
    }
    for &(rect, _, label) in WINDOWS {
        spawn_label(panel, label, rect.cx(), rect.y0 - 38.0, 28.0, LABEL, scale);
    }
    for rect in [COURSE_L_WINDOW, COURSE_R_WINDOW] {
        spawn_label(panel, "077", rect.cx(), rect.cy(), 34.0, LABEL, scale);
        spawn_label(
            panel,
            "COURSE",
            rect.cx(),
            rect.y0 - 20.0,
            24.0,
            LABEL,
            scale,
        );
    }
    for &(text, x, y, size) in EXTRA_LABELS {
        spawn_label(panel, text, x, y, size, LABEL, scale);
    }
    spawn_label(
        panel,
        "DISENGAGE",
        DISENGAGE.cx(),
        DISENGAGE.cy() - DISENGAGE.h() * 0.08,
        24.0,
        LABEL,
        scale,
    );
}

fn spawn_label(
    parent: &mut ChildSpawnerCommands,
    text: &str,
    x: f32,
    y: f32,
    size: f32,
    color: Color,
    scale: f32,
) {
    parent.spawn((
        Pickable::IGNORE,
        Text::new(text),
        TextFont {
            font_size: FontSize::Px(size * scale),
            ..default()
        },
        TextColor(color),
        Node {
            position_type: PositionType::Absolute,
            left: Val::Px(x * scale),
            top: Val::Px(y * scale),
            ..default()
        },
        UiTransform {
            translation: Val2::percent(-50.0, -50.0),
            ..default()
        },
    ));
}

fn spawn_hotspots(panel: &mut ChildSpawnerCommands, scale: f32) {
    for &(rect, press, _) in BUTTONS {
        spawn_press(panel, rect, press, 4.0 * scale, scale);
    }
    spawn_press(panel, AT_SWITCH, McpPress::AtArm, 4.0 * scale, scale);

    for window in [
        McpWindow::Speed,
        McpWindow::Heading,
        McpWindow::Altitude,
        McpWindow::VerticalSpeed,
    ] {
        spawn_window_box(panel, glass(window).expand(2.0), window, scale);
    }

    for &(rect, window) in KNOBS {
        spawn_knob(panel, window, rect, scale);
    }

    // The V/S wheel: upper half climbs, lower half descends.
    let (x, y, w, h) = VS_WHEEL.scaled(scale);
    for (dir, top, height) in [(1.0, y, h * 0.5), (-1.0, y + h * 0.5, h * 0.5)] {
        panel.spawn((
            Button,
            Hovered(false),
            McpArrow {
                window: McpWindow::VerticalSpeed,
                dir,
            },
            knob_help(McpWindow::VerticalSpeed),
            absolute(x, top, w, height, 4.0 * scale),
            BackgroundColor(Color::NONE),
            BorderColor::all(Color::NONE),
        ));
    }
}

pub(crate) fn press_help(press: McpPress) -> McpHelp {
    match press {
        McpPress::AtArm => McpHelp {
            title: "A/T ARM",
            body: "Arm the auto-throttle. Push again to disarm.",
        },
        McpPress::Cmd => McpHelp {
            title: "CMD A",
            body: "Engage the autopilot in HDG SEL and ALT HOLD. Click again to disengage.",
        },
        McpPress::HdgSel => McpHelp {
            title: "HDG SEL",
            body: "Steer to the heading in the heading window.",
        },
        McpPress::AltHold => McpHelp {
            title: "ALT HOLD",
            body: "Hold the altitude in the altitude window.",
        },
        McpPress::Vs => McpHelp {
            title: "V/S",
            body: "Climb or descend at the rate in the V/S window.",
        },
        McpPress::LvlChg => McpHelp {
            title: "LVL CHG",
            body: "Level change: full climb or descent at the speed window's speed.",
        },
        McpPress::Approach => McpHelp {
            title: "APP",
            body: "Arm the ILS approach: captures the localizer, then the glideslope. Push again to disarm.",
        },
        McpPress::SpeedUnit => McpHelp {
            title: "SPEED WINDOW",
            body: "Click: switch KIAS / MACH. Scroll: coarse speed step. Knob halves: 1 kt.",
        },
    }
}

pub(crate) fn window_help(window: McpWindow) -> McpHelp {
    match window {
        McpWindow::Speed => McpHelp {
            title: "SPEED WINDOW",
            body: "Click: switch KIAS / MACH. Scroll: coarse step. Knob halves: 1 kt.",
        },
        McpWindow::Heading => McpHelp {
            title: "HEADING WINDOW",
            body: "Scroll: 5 deg coarse step. Knob halves: 1 deg fine step.",
        },
        McpWindow::Altitude => McpHelp {
            title: "ALTITUDE WINDOW",
            body: "Scroll: 1000 ft coarse step. Knob halves: 100 ft fine step.",
        },
        McpWindow::VerticalSpeed => McpHelp {
            title: "V/S WINDOW",
            body: "Scroll or use the wheel: 100 ft/min per step.",
        },
    }
}

pub(crate) fn knob_help(window: McpWindow) -> McpHelp {
    match window {
        McpWindow::Speed => McpHelp {
            title: "SPEED KNOB",
            body: "Left half down, right half up. 1 kt per fine step.",
        },
        McpWindow::Heading => McpHelp {
            title: "HEADING KNOB",
            body: "Left half down, right half up. 1 deg per fine step.",
        },
        McpWindow::Altitude => McpHelp {
            title: "ALTITUDE KNOB",
            body: "Left half down, right half up. 100 ft per fine step.",
        },
        McpWindow::VerticalSpeed => McpHelp {
            title: "V/S WHEEL",
            body: "Upper half climbs, lower half descends. 100 ft/min per step.",
        },
    }
}

fn spawn_press(
    panel: &mut ChildSpawnerCommands,
    area: PRect,
    press: McpPress,
    radius: f32,
    scale: f32,
) {
    let (x, y, w, h) = area.scaled(scale);
    panel.spawn((
        Button,
        Hovered(false),
        press,
        press_help(press),
        absolute(x, y, w, h, radius),
        BackgroundColor(Color::NONE),
        BorderColor::all(Color::NONE),
    ));
}

fn spawn_window_box(panel: &mut ChildSpawnerCommands, area: PRect, window: McpWindow, scale: f32) {
    let (x, y, w, h) = area.scaled(scale);
    let mut e = panel.spawn((
        Button,
        Hovered(false),
        McpWindowBox(window),
        window_help(window),
        absolute(x, y, w, h, 3.0 * scale),
        BackgroundColor(Color::NONE),
        BorderColor::all(Color::NONE),
    ));
    if window == McpWindow::Speed {
        e.insert(McpPress::SpeedUnit);
    }
}

/// Left half steps fine down, right half fine up.
fn spawn_knob(panel: &mut ChildSpawnerCommands, window: McpWindow, area: PRect, scale: f32) {
    let (x, y, w, h) = area.scaled(scale);
    for (dir, left) in [(-1.0, x), (1.0, x + w * 0.5)] {
        panel.spawn((
            Button,
            Hovered(false),
            McpArrow { window, dir },
            knob_help(window),
            absolute(left, y, w * 0.5, h, 6.0 * scale),
            BackgroundColor(Color::NONE),
            BorderColor::all(Color::NONE),
        ));
    }
}

fn absolute(left: f32, top: f32, width: f32, height: f32, radius: f32) -> Node {
    Node {
        position_type: PositionType::Absolute,
        left: Val::Px(left),
        top: Val::Px(top),
        width: Val::Px(width),
        height: Val::Px(height),
        border: UiRect::all(Val::Px(1.5)),
        border_radius: BorderRadius::all(Val::Px(radius)),
        ..default()
    }
}

/// Buttons activate on release (the widget's `Activate` event).
pub(crate) fn on_mcp_activate(
    activate: On<Activate>,
    presses: Query<&McpPress>,
    sim: Res<Sim>,
    mut ap: ResMut<Autopilot>,
    mut mcp: ResMut<Mcp>,
) {
    let Ok(press) = presses.get(activate.entity) else {
        return;
    };
    match *press {
        McpPress::AtArm => ap.press_mode(ToggleAction::AutoThrottle, &*sim.model),
        McpPress::Cmd => ap.toggle(&*sim.model),
        McpPress::HdgSel => ap.press_mode(ToggleAction::HdgSel, &*sim.model),
        McpPress::AltHold => ap.press_mode(ToggleAction::AltHold, &*sim.model),
        McpPress::Vs => ap.press_mode(ToggleAction::VsMode, &*sim.model),
        McpPress::LvlChg => ap.press_mode(ToggleAction::LvlChg, &*sim.model),
        McpPress::Approach => ap.press_mode(ToggleAction::Approach, &*sim.model),
        McpPress::SpeedUnit => mcp.toggle_speed_unit(),
    }
}

/// Knob halves nudge one fine step on press, then repeat while held.
pub(crate) fn press_arrows(
    time: Res<Time>,
    mut hold: ResMut<ArrowHold>,
    mut mcp: ResMut<Mcp>,
    q: Query<(Entity, Has<Pressed>, &McpArrow)>,
) {
    let dt = time.delta_secs_f64();
    let mut held = None;
    for (entity, pressed, arrow) in &q {
        if !pressed {
            continue;
        }
        held = Some(entity);
        if hold.entity != Some(entity) {
            hold.entity = Some(entity);
            hold.held_s = 0.0;
            mcp.nudge(arrow.window, arrow.dir, true);
        } else {
            hold.held_s += dt;
            if hold.held_s >= HOLD_DELAY_S {
                hold.held_s -= HOLD_INTERVAL_S;
                mcp.nudge(arrow.window, arrow.dir, true);
            }
        }
    }
    if held.is_none() {
        hold.entity = None;
    }
}

/// The outer knob: one coarse step per wheel notch while a window is hovered.
pub(crate) fn scroll_windows(
    mut wheel: MessageReader<MouseWheel>,
    q: Query<(&Hovered, &McpWindowBox)>,
    mut mcp: ResMut<Mcp>,
) {
    let mut delta = 0.0;
    for ev in wheel.read() {
        delta += ev.y;
    }
    if delta == 0.0 {
        return;
    }
    let dir = delta.signum() as f64;
    for (hover, window) in &q {
        if hover.0 {
            mcp.nudge(window.0, dir, false);
        }
    }
}

/// Esc leaves focus; the cockpit panel eases back down and the cockpit returns.
pub(crate) fn focus_input(
    keys: Res<ButtonInput<KeyCode>>,
    ui: Option<ResMut<Ui>>,
    mut hold: ResMut<ArrowHold>,
) {
    let Some(mut ui) = ui else {
        return;
    };
    if ui.mcp_focus && keys.just_pressed(KeyCode::Escape) {
        ui.mcp_focus = false;
        hold.entity = None;
    }
}

/// The focus hint belongs to the cockpit panel: show it while the cockpit is in
/// MCP focus, hide it otherwise.
fn focus_hint(ui: Option<Res<Ui>>, q: Query<&mut Node, With<FocusHint>>) {
    let focused = ui.as_deref().is_some_and(|ui| ui.cockpit && ui.mcp_focus);
    ui_theme::set_display(
        q,
        if focused {
            Display::Flex
        } else {
            Display::None
        },
    );
}

/// Hovering a focused hotspot shows its name and what it does.
fn focus_tooltip(
    ui: Option<Res<Ui>>,
    anim: Res<FocusAnim>,
    windows: Query<&Window, With<PrimaryWindow>>,
    helped: Query<(&Hovered, &McpHelp)>,
    mut tooltip: Query<&mut Node, With<HotspotTooltip>>,
    mut texts: Query<&mut Text, With<TooltipText>>,
) {
    let focused = ui.as_deref().is_some_and(|ui| ui.mcp_focus) && anim.e > 0.9;
    let help = if focused {
        helped
            .iter()
            .find(|(hover, _)| hover.0)
            .map(|(_, help)| *help)
    } else {
        None
    };
    let placed = windows.single().ok().and_then(|window| {
        window
            .cursor_position()
            .map(|cursor| (cursor, window.size()))
    });

    let (Some((cursor, size)), Some(help)) = (placed, help) else {
        for mut node in &mut tooltip {
            node.display = Display::None;
        }
        return;
    };

    for mut node in &mut tooltip {
        node.display = Display::Flex;
        node.left = Val::Px((cursor.x + 18.0).min((size.x - 360.0).max(8.0)));
        node.top = Val::Px((cursor.y + 22.0).min((size.y - 140.0).max(8.0)));
    }
    let want = format!("{}\n{}", help.title, help.body);
    for mut text in &mut texts {
        if text.0 != want {
            text.0 = want.clone();
        }
    }
}

fn sync_panel(
    ap: Res<Autopilot>,
    mcp: Res<Mcp>,
    mut fma: Query<(&mut Text, &mut TextColor), With<FmaLabel>>,
    mut leds: Query<(&LedPart, &mut Visibility)>,
    mut buttons: Query<
        (&McpPress, &Hovered, Has<Pressed>, &mut BackgroundColor),
        Without<McpArrow>,
    >,
    mut arrows: Query<(&Hovered, Has<Pressed>, &mut BackgroundColor), With<McpArrow>>,
) {
    let speed = display_chars(McpWindow::Speed, &mcp);
    let heading = display_chars(McpWindow::Heading, &mcp);
    let altitude = display_chars(McpWindow::Altitude, &mcp);
    let vs = display_chars(McpWindow::VerticalSpeed, &mcp);
    let chars = |window: McpWindow| match window {
        McpWindow::Speed => &speed,
        McpWindow::Heading => &heading,
        McpWindow::Altitude => &altitude,
        McpWindow::VerticalSpeed => &vs,
    };

    for (part, mut vis) in &mut leds {
        let lit = match *part {
            LedPart::Seg { window, slot, seg } => {
                let ch = chars(window).get(slot).copied().unwrap_or(' ');
                seg_lit(ch, seg)
            }
            LedPart::Dot { window, slot } => chars(window).get(slot).copied().unwrap_or(' ') == '.',
            LedPart::Sign { horizontal } => matches!(
                (chars(McpWindow::VerticalSpeed)[0], horizontal),
                ('+', _) | ('-', true)
            ),
        };
        *vis = if lit {
            Visibility::Inherited
        } else {
            Visibility::Hidden
        };
    }

    let mode = ap.fma();
    let color = match mode.as_str() {
        "AP DISC" => FMA_AMBER,
        "MANUAL" => FMA_GREY,
        _ => FMA_GREEN,
    };
    for (mut text, mut text_color) in &mut fma {
        text_color.0 = color;
        text.0 = mode.clone();
    }

    for (press, hover, pressed, mut bg) in &mut buttons {
        let active = match press {
            McpPress::AtArm => ap.at != AtMode::Off,
            McpPress::Cmd => ap.engaged,
            McpPress::HdgSel => ap.engaged && ap.lat == LatMode::HdgSel,
            McpPress::AltHold => ap.engaged && ap.vert == VertMode::AltHold,
            McpPress::Vs => ap.engaged && ap.vert == VertMode::Vs,
            McpPress::LvlChg => ap.engaged && ap.vert == VertMode::LvlChg,
            // The APP lamp burns while armed or while a captured beam mode is
            // flying (LOC, G/S), as on the real panel.
            McpPress::Approach => ap.engaged && (ap.app_armed || ap.lat == LatMode::Loc),
            McpPress::SpeedUnit => false,
        };
        bg.0 = glow(active, hover.0, pressed);
    }

    for (hover, pressed, mut bg) in &mut arrows {
        bg.0 = glow(false, hover.0, pressed);
    }
}

/// The baked strip is a chase-view instrument only. The cockpit's MCP is the
/// centred panel in `mcp_panel`, so the strip stays out of the way there.
fn cockpit_visibility(ui: Option<Res<Ui>>, q: Query<&mut Node, With<McpPanel>>) {
    // `Ui` is absent in tests that mount the panel on its own.
    let cockpit = ui.as_deref().is_some_and(|ui| ui.cockpit);
    ui_theme::set_display(
        q,
        if cockpit {
            Display::None
        } else {
            Display::Flex
        },
    );
}

fn glow(active: bool, hovered: bool, pressed: bool) -> Color {
    let mut alpha: f32 = if active { 0.20 } else { 0.0 };
    if hovered {
        alpha += 0.08;
    }
    if pressed {
        alpha += 0.10;
    }
    let base = AMBER.to_srgba();
    Color::srgba(base.red, base.green, base.blue, alpha.min(0.45))
}

pub(crate) fn display_chars(window: McpWindow, mcp: &Mcp) -> Vec<char> {
    let text = match window {
        McpWindow::Speed => match mcp.speed_unit {
            SpeedUnit::Ias => format!("{:>3.0}", mcp.ias_kts),
            SpeedUnit::Mach => format!("{:.2}", mcp.mach),
        },
        McpWindow::Heading => format!("{:03.0}", mcp.heading_display()),
        McpWindow::Altitude => format!("{:05.0}", mcp.alt_ft),
        McpWindow::VerticalSpeed => {
            if mcp.vs_fpm < 0.0 {
                format!("-{:04.0}", -mcp.vs_fpm)
            } else {
                format!("+{:04.0}", mcp.vs_fpm)
            }
        }
    };
    let count = slots(window);
    let mut out = vec![' '; count];
    let glyphs: Vec<char> = text.chars().collect();
    let n = glyphs.len().min(count);
    out[count - n..].copy_from_slice(&glyphs[glyphs.len() - n..]);
    out
}

pub(crate) fn seg_lit(ch: char, seg: Seg) -> bool {
    use Seg::*;
    match ch {
        '0' => matches!(seg, A | B | C | D | E | F),
        '1' => matches!(seg, B | C),
        '2' => matches!(seg, A | B | G | E | D),
        '3' => matches!(seg, A | B | G | C | D),
        '4' => matches!(seg, F | G | B | C),
        '5' => matches!(seg, A | F | G | C | D),
        '6' => matches!(seg, A | F | G | E | C | D),
        '7' => matches!(seg, A | B | C),
        '8' => true,
        '9' => matches!(seg, A | B | C | D | F | G),
        _ => false,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use bevy::app::TaskPoolPlugin;
    use bevy::asset::AssetPlugin;
    use bevy::input::mouse::{MouseScrollUnit, MouseWheel};
    use bevy::input::touch::TouchPhase;
    use bevy::state::app::StatesPlugin;
    use std::time::Duration;

    /// A headless app with the panel spawned and the sim in `Flying`.
    fn test_app() -> App {
        let mut app = App::new();
        app.add_plugins((
            TaskPoolPlugin::default(),
            AssetPlugin::default(),
            StatesPlugin,
        ))
        .init_asset::<Image>()
        .add_message::<MouseWheel>()
        .init_resource::<ButtonInput<KeyCode>>()
        .insert_resource(Time::<()>::default())
        .insert_resource(Sim::new("737"))
        .init_resource::<Autopilot>()
        .init_resource::<Mcp>()
        .add_plugins(McpUiPlugin)
        .insert_state(AppState::Flying);
        app.update();
        app
    }

    fn find<T: Component>(app: &mut App, pred: impl Fn(&T) -> bool) -> Entity {
        let mut query = app.world_mut().query::<(Entity, &T)>();
        let world = app.world();
        for (entity, value) in query.iter(world) {
            if pred(value) {
                return entity;
            }
        }
        panic!("panel entity not found");
    }

    #[test]
    fn clicking_at_arm_engages_the_autothrottle() {
        let mut app = test_app();
        let button = find(&mut app, |press: &McpPress| *press == McpPress::AtArm);
        app.world_mut().trigger(Activate { entity: button });
        assert_eq!(app.world().resource::<Autopilot>().at, AtMode::Speed);
    }

    #[test]
    fn clicking_cmd_a_engages_the_autopilot() {
        let mut app = test_app();
        // Engagement is airborne-only (weight off the wheels); seat the sim in
        // the air first — the parked 737's gear now settles under load, which
        // correctly refuses the ground engagement this test used to rely on.
        // `wow` only refreshes on a step, so fly a few frames airborne.
        let mut sim = app.world_mut().resource_mut::<Sim>();
        sim.reset_air(5000.0, 250.0);
        for _ in 0..10 {
            sim.model.step();
        }
        assert!(sim.model.get("gear/wow") < 0.5, "should be airborne");
        let button = find(&mut app, |press: &McpPress| *press == McpPress::Cmd);
        app.world_mut().trigger(Activate { entity: button });
        let ap = app.world().resource::<Autopilot>();
        assert!(ap.engaged);
        assert_eq!(ap.lat, LatMode::HdgSel);
        assert_eq!(ap.vert, VertMode::AltHold);
    }

    #[test]
    fn arrow_press_steps_once_then_repeats_and_stops_on_release() {
        let mut app = test_app();
        let arrow = find(&mut app, |a: &McpArrow| {
            a.window == McpWindow::Heading && a.dir > 0.0
        });

        app.world_mut().entity_mut(arrow).insert(Pressed);
        app.update();
        let after_press = app.world().resource::<Mcp>().hdg_deg;
        assert!((after_press - 1.0).abs() < 1e-9);

        // Held past the repeat delay: the arrow keeps adding fine steps.
        for _ in 0..20 {
            app.world_mut()
                .resource_mut::<Time>()
                .advance_by(Duration::from_secs_f64(HOLD_INTERVAL_S));
            app.update();
        }
        let after_hold = app.world().resource::<Mcp>().hdg_deg;
        assert!(
            after_hold > after_press + 1.0,
            "expected repeat, got {after_hold}"
        );

        app.world_mut().entity_mut(arrow).remove::<Pressed>();
        app.update();
        assert!(app.world().resource::<ArrowHold>().entity.is_none());
        let after_release = app.world().resource::<Mcp>().hdg_deg;
        app.update();
        assert_eq!(app.world().resource::<Mcp>().hdg_deg, after_release);
    }

    /// The baked strip is a chase-view instrument: in the cockpit it must be
    /// out of the way, because `mcp_panel` owns the panel there.
    #[test]
    fn the_strip_is_hidden_in_the_cockpit() {
        let mut app = test_app();
        let panel = find(&mut app, |_: &McpPanel| true);

        app.insert_resource(Ui {
            cockpit: true,
            ..default()
        });
        app.update();
        assert_eq!(
            app.world().entity(panel).get::<Node>().unwrap().display,
            Display::None
        );

        app.insert_resource(Ui {
            cockpit: false,
            ..default()
        });
        app.update();
        assert_eq!(
            app.world().entity(panel).get::<Node>().unwrap().display,
            Display::Flex
        );
    }

    #[test]
    fn escape_leaves_focus() {
        let mut app = test_app();
        app.insert_resource(Ui {
            cockpit: true,
            mcp_focus: true,
            ..default()
        });
        app.world_mut()
            .resource_mut::<ButtonInput<KeyCode>>()
            .press(KeyCode::Escape);
        app.update();
        assert!(!app.world().resource::<Ui>().mcp_focus);
    }

    #[test]
    fn wheel_over_a_window_takes_a_coarse_step() {
        let mut app = test_app();
        let window = find(&mut app, |w: &McpWindowBox| w.0 == McpWindow::Altitude);
        app.world_mut().entity_mut(window).insert(Hovered(true));

        app.world_mut().write_message(MouseWheel {
            unit: MouseScrollUnit::Line,
            x: 0.0,
            y: 1.0,
            window: Entity::PLACEHOLDER,
            phase: TouchPhase::Moved,
        });
        app.update();
        assert_eq!(app.world().resource::<Mcp>().alt_ft, 11_000.0);
    }
}
