//! Clickable Mode Control Panel for the 737, drawn as a photo strip along the
//! bottom of the screen. The four live setpoint windows are masked and redrawn
//! with seven-segment glyphs; invisible hotspots over the photographed knobs
//! and buttons carry the same `Mcp`/`Autopilot` wiring as before.
//!
//! The panel art is a JetEx 737 cockpit photo, cropped to the MCP. It is fine
//! for personal use but is not ours to redistribute.

use bevy::input::mouse::MouseWheel;
use bevy::picking::hover::Hovered;
use bevy::picking::Pickable;
use bevy::prelude::*;
use bevy::ui::Pressed;
use bevy::ui_widgets::{Activate, Button};
use bevy::window::PrimaryWindow;

use crate::aircraft_profile::{self, ToggleAction};
use crate::app_state::AppState;
use crate::autopilot::{AtMode, Autopilot, LatMode, Mcp, McpWindow, SpeedUnit, VertMode};
use crate::sim::Sim;
use crate::view::Ui;

/// Panel size on screen. The texture is 2470×449, i.e. the same 5.5:1 ratio.
pub(crate) const PANEL_W: f32 = 1100.0;
const PANEL_H: f32 = 200.0;

/// The chase-view strip sits this far off the bottom edge, and is the FMA bar
/// plus a gap plus the photo tall. Focus mode lifts its centre to mid-screen.
const STRIP_BOTTOM: f32 = 12.0;
const STRIP_H: f32 = 22.0 + 4.0 + PANEL_H;

/// Red used for the focus outline on the panel and its hotspots.
const FOCUS_RED: Color = Color::srgba(1.0, 0.30, 0.20, 1.0);

/// Region of the source photo baked into `assets/ui/737_mcp.jpg`.
const CROP_X: f32 = 745.0;
const CROP_Y: f32 = 40.0;
const CROP_W: f32 = 2470.0;
const CROP_H: f32 = 449.0;

const FMA_BG: Color = Color::srgb(0.01, 0.01, 0.01);
const FMA_GREEN: Color = Color::srgb(0.25, 0.90, 0.30);
const FMA_AMBER: Color = Color::srgb(0.95, 0.66, 0.16);
const FMA_GREY: Color = Color::srgb(0.62, 0.65, 0.67);
/// Matches the warm near-black of the photographed window glass.
const GLASS: Color = Color::srgb(0.11, 0.063, 0.027);
const LED: Color = Color::srgb(0.86, 0.93, 1.0);
const LED_GLOW: Color = Color::srgba(0.45, 0.65, 1.0, 0.35);
const AMBER: Color = Color::srgb(1.0, 0.62, 0.12);

/// How long an arrow must be held before it starts repeating, and the repeat
/// period after that.
const HOLD_DELAY_S: f64 = 0.40;
const HOLD_INTERVAL_S: f64 = 0.05;

/// A clickable control that is not a setpoint knob.
#[derive(Component, Clone, Copy, PartialEq, Eq)]
enum McpPress {
    AtArm,
    Cmd,
    HdgSel,
    AltHold,
    Vs,
    LvlChg,
    /// APP: arm the ILS beams (LOC + GS). On the photo this sits one row below
    /// the LNAV/VNAV column, left of ALT HOLD.
    Approach,
    /// The speed window itself switches between kt and Mach, like pushing the
    /// speed knob on the real MCP.
    SpeedUnit,
}

/// One half of a knob (or the upper/lower half of the V/S wheel).
#[derive(Component, Clone, Copy)]
struct McpArrow {
    window: McpWindow,
    dir: f64,
}

/// The clickable window body (wheel target).
#[derive(Component, Clone, Copy)]
struct McpWindowBox(McpWindow);

/// Which bar of a seven-segment glyph a node draws.
#[derive(Clone, Copy)]
enum Seg {
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
enum LedPart {
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
struct FmaLabel;

/// The 2D photo panel — the chase-view strip, and the focused overlay in the
/// cockpit.
#[derive(Component)]
struct McpPanel;

/// Every clickable hotspot, so focus mode can outline them in red.
#[derive(Component)]
struct McpHotspot;

/// Hover text for one hotspot.
#[derive(Component, Clone, Copy)]
struct McpHelp {
    title: &'static str,
    body: &'static str,
}

/// Full-screen dim layer behind the focused panel; clicking it closes focus.
#[derive(Component)]
struct FocusDim;

/// "MCP focus" hint shown above the floating panel.
#[derive(Component)]
struct FocusHint;

/// The hover tooltip panel (child text carries `TooltipText`).
#[derive(Component)]
struct HotspotTooltip;

#[derive(Component)]
struct TooltipText;

/// Which arrow is currently held, for click-and-hold repeating.
#[derive(Resource, Default)]
struct ArrowHold {
    entity: Option<Entity>,
    held_s: f64,
}

/// Eased 0..1 focus transition: 0 = bottom photo strip, 1 = floating over a
/// dimmed cockpit.
#[derive(Resource, Default)]
struct FocusAnim {
    e: f32,
}

pub struct McpUiPlugin;

impl Plugin for McpUiPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<ArrowHold>()
            .init_resource::<FocusAnim>()
            .add_observer(on_mcp_activate)
            .add_observer(on_focus_dim)
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
                    focus_anim,
                    focus_hotspots,
                    focus_tooltip,
                    sync_panel,
                    cockpit_visibility,
                )
                    .chain()
                    .run_if(in_state(AppState::Flying)),
            );
    }
}

/// A rectangle in source-photo pixels.
#[derive(Clone, Copy)]
struct Area {
    x0: f32,
    y0: f32,
    x1: f32,
    y1: f32,
}

impl Area {
    const fn new(x0: f32, y0: f32, x1: f32, y1: f32) -> Self {
        Area { x0, y0, x1, y1 }
    }

    /// `left, top, width, height` in panel pixels.
    fn px(self) -> (f32, f32, f32, f32) {
        let sx = PANEL_W / CROP_W;
        let sy = PANEL_H / CROP_H;
        (
            (self.x0 - CROP_X) * sx,
            (self.y0 - CROP_Y) * sy,
            (self.x1 - self.x0) * sx,
            (self.y1 - self.y0) * sy,
        )
    }
}

/// The lit glass of a window, excluding its bezel.
fn glass(window: McpWindow) -> Area {
    match window {
        McpWindow::Speed => Area::new(1211.0, 152.0, 1479.0, 220.0),
        McpWindow::Heading => Area::new(1673.0, 150.0, 1816.0, 220.0),
        McpWindow::Altitude => Area::new(2011.0, 147.0, 2277.0, 218.0),
        McpWindow::VerticalSpeed => Area::new(2340.0, 144.0, 2611.0, 216.0),
    }
}

fn slots(window: McpWindow) -> usize {
    match window {
        McpWindow::Speed => 4,
        McpWindow::Heading => 3,
        McpWindow::Altitude => 5,
        McpWindow::VerticalSpeed => 5,
    }
}

fn spawn_panel(mut commands: Commands, sim: Res<Sim>, assets: Res<AssetServer>) {
    if !aircraft_profile::profile(&sim.name).has_autopilot {
        return;
    }
    let mcp_image = assets.load("ui/737_mcp.jpg");

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
            .with_children(|panel| {
                for window in [
                    McpWindow::Speed,
                    McpWindow::Heading,
                    McpWindow::Altitude,
                    McpWindow::VerticalSpeed,
                ] {
                    spawn_window(panel, window);
                }
                spawn_hotspots(panel);
            });
        });

    // Focus furniture. These are siblings of the panel so the panel's focus
    // scale does not stretch them.
    commands.spawn((
        FocusDim,
        Button,
        Hovered(false),
        Node {
            position_type: PositionType::Absolute,
            width: Val::Percent(100.0),
            height: Val::Percent(100.0),
            display: Display::None,
            ..default()
        },
        BackgroundColor(Color::NONE),
        GlobalZIndex(30),
        DespawnOnExit(AppState::Flying),
    ));

    commands
        .spawn((
            FocusHint,
            Pickable::IGNORE,
            Node {
                position_type: PositionType::Absolute,
                top: Val::Px(20.0),
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
                TextColor(Color::srgba(1.0, 0.55, 0.45, 0.95)),
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

/// Mask the baked-in digits, then lay out the live seven-segment glyphs.
fn spawn_window(panel: &mut ChildSpawnerCommands, window: McpWindow) {
    let (gx, gy, gw, gh) = glass(window).px();
    panel.spawn((
        Node {
            position_type: PositionType::Absolute,
            left: Val::Px(gx),
            top: Val::Px(gy),
            width: Val::Px(gw),
            height: Val::Px(gh),
            border_radius: BorderRadius::all(Val::Px(2.0)),
            ..default()
        },
        BackgroundColor(GLASS),
    ));

    // The photographed digits are ~40 source px apart and ~48 px tall; keep
    // that pitch so the redrawn glyphs read at the same scale as the panel.
    let count = slots(window);
    let pitch = 40.0 * PANEL_W / CROP_W;
    let dw = pitch * 0.75;
    let dh = 48.0 * PANEL_H / CROP_H;
    let stroke = (dh * 0.13).max(2.0);
    let right_margin = 4.0;

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
fn segment_rects(w: f32, h: f32, s: f32) -> [(Seg, f32, f32, f32, f32); 7] {
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

fn spawn_hotspots(panel: &mut ChildSpawnerCommands) {
    for (press, area) in [
        (McpPress::AtArm, Area::new(1070.0, 155.0, 1175.0, 290.0)),
        (McpPress::Cmd, Area::new(2679.0, 146.0, 2784.0, 255.0)),
        (McpPress::LvlChg, Area::new(1524.0, 361.0, 1628.0, 463.0)),
        (McpPress::HdgSel, Area::new(1694.0, 361.0, 1798.0, 463.0)),
        (McpPress::Approach, Area::new(1102.0, 310.0, 1208.0, 430.0)),
        (McpPress::AltHold, Area::new(2090.0, 355.0, 2205.0, 470.0)),
        (McpPress::Vs, Area::new(2208.0, 355.0, 2323.0, 470.0)),
    ] {
        spawn_press(panel, area, press, 4.0);
    }

    for window in [
        McpWindow::Speed,
        McpWindow::Heading,
        McpWindow::Altitude,
        McpWindow::VerticalSpeed,
    ] {
        let a = glass(window);
        let area = Area::new(a.x0 - 2.0, a.y0 - 2.0, a.x1 + 2.0, a.y1 + 2.0);
        spawn_window_box(panel, area, window);
    }

    spawn_knob(
        panel,
        McpWindow::Speed,
        Area::new(1340.0, 272.0, 1450.0, 390.0),
    );
    spawn_knob(
        panel,
        McpWindow::Heading,
        Area::new(1680.0, 245.0, 1800.0, 355.0),
    );
    spawn_knob(
        panel,
        McpWindow::Altitude,
        Area::new(2105.0, 245.0, 2230.0, 355.0),
    );

    // The V/S wheel: upper half climbs, lower half descends.
    let (x, y, w, h) = Area::new(2515.0, 255.0, 2570.0, 445.0).px();
    for (dir, top, height) in [(1.0, y, h * 0.5), (-1.0, y + h * 0.5, h * 0.5)] {
        panel.spawn((
            Button,
            Hovered(false),
            McpHotspot,
            McpArrow {
                window: McpWindow::VerticalSpeed,
                dir,
            },
            knob_help(McpWindow::VerticalSpeed),
            absolute(x, top, w, height, 4.0),
            BackgroundColor(Color::NONE),
            BorderColor::all(Color::NONE),
        ));
    }
}

fn press_help(press: McpPress) -> McpHelp {
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

fn window_help(window: McpWindow) -> McpHelp {
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

fn knob_help(window: McpWindow) -> McpHelp {
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

fn spawn_press(panel: &mut ChildSpawnerCommands, area: Area, press: McpPress, radius: f32) {
    let (x, y, w, h) = area.px();
    panel.spawn((
        Button,
        Hovered(false),
        McpHotspot,
        press,
        press_help(press),
        absolute(x, y, w, h, radius),
        BackgroundColor(Color::NONE),
        BorderColor::all(Color::NONE),
    ));
}

fn spawn_window_box(panel: &mut ChildSpawnerCommands, area: Area, window: McpWindow) {
    let (x, y, w, h) = area.px();
    let mut e = panel.spawn((
        Button,
        Hovered(false),
        McpHotspot,
        McpWindowBox(window),
        window_help(window),
        absolute(x, y, w, h, 3.0),
        BackgroundColor(Color::NONE),
        BorderColor::all(Color::NONE),
    ));
    if window == McpWindow::Speed {
        e.insert(McpPress::SpeedUnit);
    }
}

/// Left half steps fine down, right half fine up.
fn spawn_knob(panel: &mut ChildSpawnerCommands, window: McpWindow, area: Area) {
    let (x, y, w, h) = area.px();
    for (dir, left) in [(-1.0, x), (1.0, x + w * 0.5)] {
        panel.spawn((
            Button,
            Hovered(false),
            McpHotspot,
            McpArrow { window, dir },
            knob_help(window),
            absolute(left, y, w * 0.5, h, 6.0),
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
fn on_mcp_activate(
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
fn press_arrows(
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
fn scroll_windows(
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

/// Esc leaves focus; the panel eases back down to the strip.
fn focus_input(
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

/// Clicking the dimmed backdrop closes focus.
fn on_focus_dim(activate: On<Activate>, dims: Query<(), With<FocusDim>>, ui: Option<ResMut<Ui>>) {
    if dims.get(activate.entity).is_err() {
        return;
    }
    if let Some(mut ui) = ui {
        ui.mcp_focus = false;
    }
}

/// Ease the panel between the bottom strip and a raised, scaled "book" over a
/// dimmed cockpit. The dark layer and hint appear with it.
fn focus_anim(
    time: Res<Time>,
    ui: Option<Res<Ui>>,
    mut anim: ResMut<FocusAnim>,
    windows: Query<&Window, With<PrimaryWindow>>,
    mut panel: Query<(&mut UiTransform, &mut GlobalZIndex), With<McpPanel>>,
    mut dim: Query<(&mut Node, &mut BackgroundColor), (With<FocusDim>, Without<McpPanel>)>,
    mut hint: Query<&mut Node, (With<FocusHint>, Without<McpPanel>, Without<FocusDim>)>,
) {
    let cockpit = ui.as_deref().is_some_and(|ui| ui.cockpit);
    let target = cockpit && ui.as_deref().is_some_and(|ui| ui.mcp_focus);
    let want: f32 = if target { 1.0 } else { 0.0 };
    if cockpit {
        anim.e += (want - anim.e) * (1.0 - (-8.0 * time.delta_secs()).exp());
        if (want - anim.e).abs() < 0.002 {
            anim.e = want;
        }
    } else {
        // Leaving the cockpit: the strip is already the chase-view pose.
        anim.e = 0.0;
    }
    let e = anim.e;

    let Ok(window) = windows.single() else {
        return;
    };
    let lift = (window.height() * 0.5 - (STRIP_BOTTOM + STRIP_H * 0.5)).max(0.0) * e;
    let fit = (window.width() * 0.92 / PANEL_W).clamp(0.8, 1.6);
    let scale = 1.0 + (fit - 1.0) * e;

    for (mut tf, mut z) in &mut panel {
        tf.translation = Val2::new(Val::Px(0.0), Val::Px(-lift));
        tf.scale = Vec2::splat(scale);
        let z_want = if e > 0.001 { 31 } else { 15 };
        if z.0 != z_want {
            z.0 = z_want;
        }
    }

    let show = if e > 0.001 {
        Display::Flex
    } else {
        Display::None
    };
    for (mut node, mut bg) in &mut dim {
        if node.display != show {
            node.display = show;
        }
        bg.0 = Color::srgba(0.0, 0.0, 0.0, 0.72 * e);
    }

    let hint_show = if e > 0.3 {
        Display::Flex
    } else {
        Display::None
    };
    for mut node in &mut hint {
        if node.display != hint_show {
            node.display = hint_show;
        }
    }
}

/// Outline every clickable control in red while the panel is focused.
fn focus_hotspots(anim: Res<FocusAnim>, mut q: Query<&mut BorderColor, With<McpHotspot>>) {
    let alpha = (anim.e * 0.9).clamp(0.0, 1.0);
    let color = Color::srgba(1.0, 0.30, 0.20, alpha);
    for mut border in &mut q {
        *border = BorderColor::all(color);
    }
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
            LedPart::Sign { horizontal } => {
                match (chars(McpWindow::VerticalSpeed)[0], horizontal) {
                    ('+', _) | ('-', true) => true,
                    _ => false,
                }
            }
        };
        *vis = if lit {
            Visibility::Inherited
        } else {
            Visibility::Hidden
        };
    }

    if let Ok((mut text, mut color)) = fma.single_mut() {
        let mode = ap.fma();
        color.0 = match mode.as_str() {
            "AP DISC" => FMA_AMBER,
            "MANUAL" => FMA_GREY,
            _ => FMA_GREEN,
        };
        text.0 = mode;
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

/// The photo panel is a chase-view instrument; in the cockpit it only lifts
/// into view while focused (or while easing back down).
fn cockpit_visibility(
    ui: Option<Res<Ui>>,
    anim: Res<FocusAnim>,
    mut q: Query<&mut Node, With<McpPanel>>,
) {
    // `Ui` is absent in tests that mount the panel on its own.
    let ui = ui.as_deref();
    let cockpit = ui.is_some_and(|ui| ui.cockpit);
    let focused = ui.is_some_and(|ui| ui.mcp_focus);
    let display = if cockpit && !focused && anim.e <= 0.001 {
        Display::None
    } else {
        Display::Flex
    };
    for mut node in &mut q {
        if node.display != display {
            node.display = display;
        }
    }
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

fn display_chars(window: McpWindow, mcp: &Mcp) -> Vec<char> {
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

fn seg_lit(ch: char, seg: Seg) -> bool {
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

    /// A focused panel must actually lift to mid-screen, scale up and sit above
    /// the dim layer — the "book" pose — and the dim must be visible.
    #[test]
    fn focus_raises_the_panel_over_a_dim_layer() {
        let mut app = test_app();
        app.world_mut().spawn((Window::default(), PrimaryWindow));
        app.insert_resource(Ui {
            cockpit: true,
            mcp_focus: true,
            ..default()
        });

        for _ in 0..120 {
            app.world_mut()
                .resource_mut::<Time>()
                .advance_by(Duration::from_millis(16));
            app.update();
        }

        let panel = find(&mut app, |_: &McpPanel| true);
        let entity = app.world().entity(panel);
        let scale = entity.get::<UiTransform>().unwrap().scale.x;
        assert!(scale > 1.0, "panel did not grow: {scale}");
        let Val::Px(lift) = entity.get::<UiTransform>().unwrap().translation.y else {
            panic!("unexpected lift");
        };
        assert!(lift < -100.0, "panel did not rise: {lift}");
        assert_eq!(entity.get::<GlobalZIndex>().unwrap().0, 31);

        let dim = find(&mut app, |_: &FocusDim| true);
        let dim = app.world().entity(dim);
        assert_eq!(dim.get::<Node>().unwrap().display, Display::Flex);
        assert_eq!(app.world().resource::<FocusAnim>().e, 1.0);
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
    fn clicking_the_dim_layer_leaves_focus() {
        let mut app = test_app();
        app.insert_resource(Ui {
            cockpit: true,
            mcp_focus: true,
            ..default()
        });
        let dim = find(&mut app, |_: &FocusDim| true);
        app.world_mut().trigger(Activate { entity: dim });
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
