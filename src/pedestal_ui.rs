//! The 737 control stand: a baked plate with live levers, lights and readouts.
//!
//! The strip sits above the MCP in chase view. In the cockpit it is hidden
//! until the pilot looks down at the 3D pedestal and clicks it (or presses
//! `;`); then the same plate is raised as a centred hero panel. Its hotspots
//! write the same `Bindings` the keyboard writes and call the same
//! `input::apply_toggle` for the latching controls, so a click and a key
//! cannot land on different states.
//!
//! The plate is static; the moving parts are nodes:
//! - five lever handles on stems (flap, speedbrake, two throttles, gear),
//! - the actual-flap fill and pointer (which lags the lever),
//! - two N2 bars, eight lamp lenses, and the reverse latch's glow,
//! - live degree / N2 digits.
//!
//! Coordinates come from `pedestal_layout`; this module only draws and wires.

use bevy::picking::hover::Hovered;
use bevy::picking::Pickable;
use bevy::prelude::*;
use bevy::ui::{Pressed, RelativeCursorPosition};
use bevy::ui_widgets::Button;
use bevy::window::PrimaryWindow;

use crate::aircraft_profile::{Action, ToggleAction};
use crate::app_state::{unpaused, AppState};
use crate::autopilot::{AtMode, Autopilot};
use crate::hero_panel::{self, HeroSpec};
use crate::input::{apply_toggle, Bindings};
use crate::mcp_layout::LABEL;
use crate::mcp_ui::spawn_label;
use crate::panel_geom::PRect;
use crate::pedestal_layout::{
    flap_unit, lever, LeverId, LightId, CROP_H, CROP_W, FLAP_INDICATOR, FLAP_LABEL_X, FLAP_STOPS,
    FLAP_TRACK, LABELS, LEVERS, LIGHTS, N2_TRACKS, PANEL_W, REVERSE_BAR,
};
use crate::sim::{AircraftState, Sim};
use crate::ui_theme;
use crate::view::{PanelFocus, Ui};

/// The strip is hidden below this window height: at 720p the MCP and the PFDs
/// already crowd the bottom of the screen.
const MIN_HEIGHT: f32 = 900.0;

/// Chase-view strip z. Above the MCP strip (15), below the cockpit hero (30).
const STRIP_Z: i32 = 16;

#[derive(Resource)]
struct StandView {
    open: bool,
}

impl Default for StandView {
    fn default() -> Self {
        Self { open: true }
    }
}

/// The baked strip in the chase view.
#[derive(Component)]
struct StandPanel;

/// The cockpit hero: the same plate, laid out at asset size and scaled to the
/// viewport, so its hotspots keep `pedestal_layout` coordinates.
#[derive(Component)]
struct StandHero;

#[derive(Component)]
struct StandBackdrop;

/// Raise/dismiss progress for the hero. One panel is focused at a time, so a
/// single shared value is enough.
#[derive(Resource, Default)]
struct StandFocusAnim {
    e: f32,
}

/// Hero fit: leave a margin on the long axis so the labels breathe.
const HERO: HeroSpec = HeroSpec {
    crop: Vec2::new(CROP_W, CROP_H),
    fit: Vec2::new(1.14, 1.28),
};

/// A moving lever part. `scale` is the panel's asset-px-to-screen-px factor,
/// stored so the sync system can place a node without a transform query.
#[derive(Component, Clone, Copy)]
struct LeverPart {
    id: LeverId,
    part: Part,
    scale: f32,
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum Part {
    Stem,
    Knob,
}

/// A clickable gate.
#[derive(Component, Clone, Copy)]
enum StandSpot {
    Lever(LeverId),
    Reverse,
}

/// The gate's asset-px area, for mapping the cursor's normalized position.
#[derive(Component, Clone, Copy)]
struct SpotArea {
    rect: PRect,
    spot: StandSpot,
}

#[derive(Component, Clone, Copy)]
enum Fill {
    Flap { scale: f32 },
    N2 { scale: f32 },
}

#[derive(Component, Clone, Copy)]
enum Readout {
    FlapDeg,
    N2,
}

#[derive(Component, Clone, Copy)]
struct StandLamp(LightId);

/// The amber wash behind the reverse latch's groove.
#[derive(Component)]
struct ReverseFill;

pub struct PedestalUiPlugin;

impl Plugin for PedestalUiPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<StandView>()
            .init_resource::<StandFocusAnim>()
            .add_systems(
                OnEnter(AppState::Flying),
                (spawn_stand, spawn_hero).after(crate::sim::StartFlight),
            )
            .add_systems(
                Update,
                (
                    toggle_stand,
                    poll_levers,
                    sync_levers,
                    sync_fills,
                    sync_readouts,
                    sync_lamps,
                    sync_reverse,
                    stand_visibility,
                    close_on_escape,
                    sync_hero,
                    hero_pointer,
                )
                    .chain()
                    .run_if(unpaused),
            );
    }
}

fn spawn_stand(
    mut commands: Commands,
    sim: Res<Sim>,
    assets: Res<AssetServer>,
    windows: Query<&Window, With<PrimaryWindow>>,
) {
    if sim.name != "737" {
        return;
    }
    let plate = assets.load("ui/737_pedestal_plate.png");
    // Fit the strip to the window at spawn: 1100 px on a 1080p screen, a little
    // narrower on a small one. The game starts fullscreen, so this is the size
    // it keeps.
    let win_w = windows.single().map(|w| w.width()).unwrap_or(PANEL_W + 40.0);
    let panel_w = PANEL_W.min((win_w - 40.0).max(640.0));
    let panel_h = CROP_H * panel_w / CROP_W;
    let scale = panel_w / CROP_W;

    commands
        .spawn((
            StandPanel,
            Node {
                position_type: PositionType::Absolute,
                left: Val::Percent(50.0),
                margin: UiRect::left(Val::Px(-0.5 * panel_w)),
                bottom: Val::Px(crate::pedestal_layout::BOTTOM),
                width: Val::Px(panel_w),
                height: Val::Px(panel_h),
                ..default()
            },
            BoxShadow::new(
                Color::srgba(0.0, 0.0, 0.0, 0.60),
                Val::Px(0.0),
                Val::Px(4.0),
                Val::Px(0.0),
                Val::Px(16.0),
            ),
            GlobalZIndex(STRIP_Z),
            DespawnOnExit(AppState::Flying),
        ))
        .with_children(|root| {
            root.spawn((
                ImageNode::new(plate),
                Node {
                    width: Val::Px(panel_w),
                    height: Val::Px(panel_h),
                    ..default()
                },
            ))
            .with_children(|panel| spawn_stand_contents(panel, scale));
        });
}

/// The cockpit control-stand hero: a second copy of the plate at asset size,
/// shown when the pedestal is focused. It is a separate copy rather than a
/// reparented strip so the chase strip's screen scale and the hero's viewport
/// scale never fight over one node tree.
fn spawn_hero(mut commands: Commands, sim: Res<Sim>, assets: Res<AssetServer>) {
    if sim.name != "737" {
        return;
    }
    let plate = assets.load("ui/737_pedestal_plate.png");

    commands.spawn((
        StandBackdrop,
        Pickable::IGNORE,
        Node {
            position_type: PositionType::Absolute,
            left: Val::Px(0.0),
            top: Val::Px(0.0),
            width: Val::Percent(100.0),
            height: Val::Percent(100.0),
            display: Display::None,
            ..default()
        },
        BackgroundColor(Color::NONE),
        GlobalZIndex(29),
        DespawnOnExit(AppState::Flying),
    ));

    commands
        .spawn((
            StandHero,
            Pickable::IGNORE,
            Node {
                position_type: PositionType::Absolute,
                left: Val::Percent(50.0),
                top: Val::Percent(50.0),
                width: Val::Px(CROP_W),
                height: Val::Px(CROP_H),
                display: Display::None,
                ..default()
            },
            UiTransform {
                translation: Val2::percent(-50.0, -50.0),
                scale: Vec2::ONE,
                ..default()
            },
            GlobalZIndex(30),
            DespawnOnExit(AppState::Flying),
        ))
        .with_children(|root| {
            root.spawn((
                ImageNode::new(plate),
                Node {
                    width: Val::Px(CROP_W),
                    height: Val::Px(CROP_H),
                    ..default()
                },
            ))
            .with_children(|panel| spawn_stand_contents(panel, 1.0));
        });
}

/// The hero's width at this viewport. It is a wide strip, so it is almost
/// always width-limited; the height term only matters on very wide, short
/// windows.
#[cfg(test)]
fn hero_width(viewport: Vec2) -> f32 {
    HERO.width(viewport)
}

fn sync_hero(
    time: Res<Time>,
    ui: Option<Res<Ui>>,
    mut anim: ResMut<StandFocusAnim>,
    windows: Query<&Window, With<PrimaryWindow>>,
    mut root: Query<(&mut Node, &mut UiTransform), With<StandHero>>,
    mut backdrop: Query<(&mut Node, &mut BackgroundColor), (With<StandBackdrop>, Without<StandHero>)>,
) {
    hero_panel::sync::<StandHero, StandBackdrop>(
        PanelFocus::Pedestal,
        &HERO,
        &time,
        ui.as_deref(),
        &mut anim.e,
        &windows,
        &mut root,
        &mut backdrop,
    );
}

/// Esc lowers the stand when it is the raised panel. The MCP owns its own Esc
/// so the two plugins do not need each other.
fn close_on_escape(keys: Res<ButtonInput<KeyCode>>, mut ui: Option<ResMut<Ui>>) {
    if !keys.just_pressed(KeyCode::Escape) {
        return;
    }
    if let Some(ui) = ui.as_mut() {
        if ui.is_focused(PanelFocus::Pedestal) {
            ui.panel_focus = None;
        }
    }
}

/// A left press outside the hero closes it, but the press that opened it must
/// not. Mirrors the MCP panel's rule.
fn hero_pointer(
    mouse: Res<ButtonInput<MouseButton>>,
    windows: Query<&Window, With<PrimaryWindow>>,
    mut ui: Option<ResMut<Ui>>,
    mut was_focused: Local<bool>,
) {
    hero_panel::pointer_closes(
        PanelFocus::Pedestal,
        &HERO,
        &mouse,
        &windows,
        ui.as_deref_mut(),
        &mut was_focused,
    );
}

/// Lay the live layer over the baked plate. `scale` is screen px per asset px;
/// the cockpit hero passes `1.0` and scales the whole panel instead.
pub(crate) fn spawn_stand_contents(panel: &mut ChildSpawnerCommands, scale: f32) {
    spawn_static_labels(panel, scale);
    spawn_fills(panel, scale);
    spawn_lamps(panel, scale);
    spawn_levers(panel, scale);
    spawn_readouts(panel, scale);
    spawn_reverse(panel, scale);
    spawn_spots(panel, scale);
}

fn spawn_static_labels(panel: &mut ChildSpawnerCommands, scale: f32) {
    for &(text, x, y, size) in LABELS {
        spawn_label(panel, text, x, y, size, LABEL, scale);
    }
    for (caption, norm) in FLAP_STOPS {
        let y = lever(LeverId::Flap).y(*norm);
        spawn_label(panel, caption, FLAP_LABEL_X, y, 22.0, LABEL, scale);
    }
}

fn spawn_fills(panel: &mut ChildSpawnerCommands, scale: f32) {
    let (x, y, w, _) = FLAP_TRACK.scaled(scale);
    panel.spawn((
        Pickable::IGNORE,
        Fill::Flap { scale },
        Node {
            position_type: PositionType::Absolute,
            left: Val::Px(x),
            top: Val::Px(y),
            width: Val::Px(w),
            height: Val::Px(0.0),
            border: UiRect::top(Val::Px((2.5 * scale).max(1.0))),
            ..default()
        },
        BorderColor::all(Color::srgb(1.0, 0.86, 0.55)),
        BackgroundColor(Color::srgba(0.95, 0.62, 0.16, 0.32)),
        BoxShadow::new(
            Color::srgba(1.0, 0.70, 0.20, 0.25),
            Val::Px(0.0),
            Val::Px(0.0),
            Val::Px(1.0),
            Val::Px(6.0),
        ),
    ));

    for (_, track) in N2_TRACKS {
        // Inset from the baked track so the bezel stays visible.
        let r = track.expand(-3.0);
        let (x, y, w, h) = r.scaled(scale);
        panel.spawn((
            Pickable::IGNORE,
            Fill::N2 { scale },
            Node {
                position_type: PositionType::Absolute,
                left: Val::Px(x),
                top: Val::Px(y + h),
                width: Val::Px(w),
                height: Val::Px(0.0),
                border: UiRect::top(Val::Px((2.0 * scale).max(1.0))),
                ..default()
            },
            BorderColor::all(Color::srgb(0.86, 0.95, 0.86)),
            BackgroundColor(Color::srgb(0.78, 0.86, 0.76)),
            BoxShadow::new(
                Color::srgba(0.55, 0.90, 0.60, 0.18),
                Val::Px(0.0),
                Val::Px(0.0),
                Val::Px(1.0),
                Val::Px(5.0),
            ),
        ));
    }
}

fn spawn_lamps(panel: &mut ChildSpawnerCommands, scale: f32) {
    for spec in LIGHTS {
        let lamp = lamp_color(spec.id, false);
        let r = spec.lens.expand(-1.5);
        let (x, y, w, h) = r.scaled(scale);
        panel.spawn((
            Pickable::IGNORE,
            StandLamp(spec.id),
            Node {
                position_type: PositionType::Absolute,
                left: Val::Px(x),
                top: Val::Px(y),
                width: Val::Px(w),
                height: Val::Px(h),
                border_radius: if spec.round {
                    BorderRadius::all(Val::Px(h * 0.5))
                } else {
                    BorderRadius::all(Val::Px((3.0 * scale).max(1.0)))
                },
                ..default()
            },
            BackgroundColor(lamp),
            BoxShadow::new(
                Color::NONE,
                Val::Px(0.0),
                Val::Px(0.0),
                Val::Px(2.0),
                Val::Px(14.0),
            ),
        ));
    }
}

/// The lever handles: a stem from the bottom of the gate to the handle, and the
/// handle itself. Stems are spawned first so the handle covers their top end.
fn spawn_levers(panel: &mut ChildSpawnerCommands, scale: f32) {
    for l in LEVERS {
        let look = lever_look(l.id);
        let stem_w = l.stem_w * scale;
        panel.spawn((
            Pickable::IGNORE,
            LeverPart {
                id: l.id,
                part: Part::Stem,
                scale,
            },
            Node {
                position_type: PositionType::Absolute,
                left: Val::Px((l.cx() - l.stem_w * 0.5) * scale),
                top: Val::Px(l.channel.y1 * scale),
                width: Val::Px(stem_w),
                height: Val::Px(0.0),
                border_radius: BorderRadius::all(Val::Px(stem_w * 0.5)),
                ..default()
            },
            BackgroundColor(Color::srgb(0.52, 0.51, 0.47)),
        ));

        // The handle.
        let w = l.knob.x * scale;
        let h = l.knob.y * scale;
        let round = l.id == LeverId::Gear;
        let mut handle = panel.spawn((
            Pickable::IGNORE,
            LeverPart {
                id: l.id,
                part: Part::Knob,
                scale,
            },
            Node {
                position_type: PositionType::Absolute,
                left: Val::Px((l.cx() - l.knob.x * 0.5) * scale),
                top: Val::Px(l.y(0.0) * scale - h * 0.5),
                width: Val::Px(w),
                height: Val::Px(h),
                border: UiRect::all(Val::Px((1.5 * scale).max(1.0))),
                border_radius: if round {
                    BorderRadius::all(Val::Px(h * 0.5))
                } else {
                    BorderRadius::all(Val::Px((7.0 * scale).max(2.0)))
                },
                ..default()
            },
            BackgroundColor(look.body),
            BorderColor::all(look.edge),
            BoxShadow::new(
                Color::srgba(0.0, 0.0, 0.0, 0.55),
                Val::Px(0.0),
                Val::Px(2.0 * scale),
                Val::Px(0.0),
                Val::Px(6.0 * scale),
            ),
        ));
        handle.with_children(|k| {
            if round {
                // The gear handle is a wheel: a dark hub and a centre dot.
                k.spawn((
                    Pickable::IGNORE,
                    Node {
                        position_type: PositionType::Absolute,
                        left: Val::Percent(22.0),
                        top: Val::Percent(22.0),
                        width: Val::Percent(56.0),
                        height: Val::Percent(56.0),
                        border_radius: BorderRadius::all(Val::Percent(50.0)),
                        ..default()
                    },
                    BackgroundColor(Color::srgb(0.17, 0.165, 0.155)),
                ));
            } else {
                // A brushed highlight along the top of the handle.
                k.spawn((
                    Pickable::IGNORE,
                    Node {
                        position_type: PositionType::Absolute,
                        left: Val::Percent(9.0),
                        top: Val::Percent(9.0),
                        width: Val::Percent(82.0),
                        height: Val::Percent(28.0),
                        border_radius: BorderRadius::all(Val::Px(3.0 * scale)),
                        ..default()
                    },
                    BackgroundColor(look.high),
                ));
                for f in [0.52, 0.68] {
                    k.spawn((
                        Pickable::IGNORE,
                        Node {
                            position_type: PositionType::Absolute,
                            left: Val::Percent(22.0),
                            top: Val::Percent(f * 100.0),
                            width: Val::Percent(56.0),
                            height: Val::Px((1.5 * scale).max(1.0)),
                            ..default()
                        },
                        BackgroundColor(look.grip),
                    ));
                }
            }
        });
    }
}

fn spawn_readouts(panel: &mut ChildSpawnerCommands, scale: f32) {
    for (text, x, y, size, readout) in [
        ("0.0", FLAP_INDICATOR.cx(), 486.0, 24.0, Readout::FlapDeg),
        ("0", N2_TRACKS[0].1.cx(), 452.0, 22.0, Readout::N2),
        ("0", N2_TRACKS[1].1.cx(), 452.0, 22.0, Readout::N2),
    ] {
        panel.spawn((
            Pickable::IGNORE,
            readout,
            Text::new(text),
            TextFont {
                font_size: FontSize::Px(size * scale),
                ..default()
            },
            TextColor(LABEL),
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
}

fn spawn_reverse(panel: &mut ChildSpawnerCommands, scale: f32) {
    let r = REVERSE_BAR.expand(-4.0);
    let (x, y, w, h) = r.scaled(scale);
    panel.spawn((
        Pickable::IGNORE,
        ReverseFill,
        Node {
            position_type: PositionType::Absolute,
            left: Val::Px(x),
            top: Val::Px(y),
            width: Val::Px(w),
            height: Val::Px(h),
            border_radius: BorderRadius::all(Val::Px((5.0 * scale).max(2.0))),
            ..default()
        },
        BackgroundColor(Color::NONE),
    ));
    spawn_label(panel, "REV", REVERSE_BAR.cx(), REVERSE_BAR.cy(), 20.0, LABEL, scale);
}

/// The click targets. They sit above the handles in picking order; the handles
/// are `Pickable::IGNORE`, so a press anywhere on a gate reaches its spot.
fn spawn_spots(panel: &mut ChildSpawnerCommands, scale: f32) {
    for l in LEVERS {
        let (x, y, w, h) = l.hit.scaled(scale);
        panel.spawn((
            Button,
            Hovered(false),
            RelativeCursorPosition::default(),
            SpotArea {
                rect: l.hit,
                spot: StandSpot::Lever(l.id),
            },
            Node {
                position_type: PositionType::Absolute,
                left: Val::Px(x),
                top: Val::Px(y),
                width: Val::Px(w),
                height: Val::Px(h),
                border_radius: BorderRadius::all(Val::Px(4.0 * scale)),
                ..default()
            },
            BackgroundColor(Color::NONE),
            BorderColor::all(Color::NONE),
        ));
    }
    let (x, y, w, h) = REVERSE_BAR.scaled(scale);
    panel.spawn((
        Button,
        Hovered(false),
        RelativeCursorPosition::default(),
        SpotArea {
            rect: REVERSE_BAR,
            spot: StandSpot::Reverse,
        },
        Node {
            position_type: PositionType::Absolute,
            left: Val::Px(x),
            top: Val::Px(y),
            width: Val::Px(w),
            height: Val::Px(h),
            border_radius: BorderRadius::all(Val::Px(4.0 * scale)),
            ..default()
        },
        BackgroundColor(Color::NONE),
        BorderColor::all(Color::NONE),
    ));
}

/// The gate value a cursor at `norm_y` asks for. Split out so the mapping is
/// testable without a window.
fn spot_value(spot: StandSpot, area: PRect, norm_y: f32) -> Option<f64> {
    match spot {
        StandSpot::Lever(id) => {
            let y = area.y0 + (norm_y + 0.5) * area.h();
            Some(lever(id).value_at(y))
        }
        StandSpot::Reverse => None,
    }
}

fn toggle_stand(
    keys: Res<ButtonInput<KeyCode>>,
    mut view: ResMut<StandView>,
    mut ui: Option<ResMut<Ui>>,
) {
    if !keys.just_pressed(KeyCode::Semicolon) {
        return;
    }
    // In the cockpit `;` raises the same stand as a hero panel; in the chase
    // view it toggles the strip above the MCP.
    if let Some(ui) = ui.as_mut() {
        if ui.cockpit {
            ui.panel_focus = if ui.is_focused(PanelFocus::Pedestal) {
                None
            } else {
                Some(PanelFocus::Pedestal)
            };
            return;
        }
    }
    view.open = !view.open;
}

fn stand_visible(open: bool, height: f32, cockpit: bool) -> bool {
    open && height >= MIN_HEIGHT && !cockpit
}

fn stand_visibility(
    ui: Option<Res<Ui>>,
    view: Res<StandView>,
    windows: Query<&Window, With<PrimaryWindow>>,
    q: Query<&mut Node, With<StandPanel>>,
) {
    let height = windows.single().map(|w| w.size().y).unwrap_or(1080.0);
    let cockpit = ui.as_deref().is_some_and(|ui| ui.cockpit);
    ui_theme::set_display(
        q,
        if stand_visible(view.open, height, cockpit) {
            Display::Flex
        } else {
            Display::None
        },
    );
}

/// A press (or drag) on a gate writes the value the keyboard would have
/// reached. The latching controls reuse `apply_toggle`, so the gear, the
/// speedbrake and reverse cannot drift from their keys.
fn poll_levers(
    mouse: Res<ButtonInput<MouseButton>>,
    mut bindings: ResMut<Bindings>,
    mut sim: ResMut<Sim>,
    mut ap: ResMut<Autopilot>,
    mut mcp: ResMut<crate::autopilot::Mcp>,
    mut lights: ResMut<crate::lights::AircraftLights>,
    weather: Res<crate::weather::Weather>,
    mut q: Query<(
        Entity,
        &SpotArea,
        Has<Pressed>,
        &RelativeCursorPosition,
        &Hovered,
        &mut BackgroundColor,
    )>,
    mut held: Local<Option<Entity>>,
) {
    let left = mouse.pressed(MouseButton::Left);
    let mut now = None;
    for (entity, area, pressed, rel, hover, mut bg) in &mut q {
        // A faint wash on hover and a stronger one while held: the gate is a
        // click target even though its handle is drawn on top.
        bg.0 = if pressed {
            Color::srgba(1.0, 0.82, 0.45, 0.10)
        } else if hover.0 {
            Color::srgba(1.0, 0.82, 0.45, 0.045)
        } else {
            Color::NONE
        };
        if !left || !pressed {
            continue;
        }
        now = Some(entity);
        let first = *held != Some(entity);
        let Some(norm) = rel.normalized else {
            continue;
        };
        match area.spot {
            StandSpot::Reverse => {
                if first {
                    apply_toggle(
                        ToggleAction::Reverse,
                        &mut sim,
                        &mut ap,
                        &mut mcp,
                        &mut bindings,
                        &mut lights,
                        &weather,
                    );
                }
            }
            StandSpot::Lever(id) => {
                let Some(want) = spot_value(area.spot, area.rect, norm.y) else {
                    continue;
                };
                match id {
                    LeverId::Flap => {
                        bindings.set(Action::Flap, want);
                    }
                    LeverId::ThrottleLeft | LeverId::ThrottleRight => {
                        if first {
                            ap.disconnect_autothrottle();
                        }
                        bindings.set(Action::Throttle, want);
                    }
                    LeverId::Speedbrake => {
                        let now = sim.model.get("fcs/speedbrake-cmd-norm");
                        if (now > 0.5) != (want > 0.5) {
                            apply_toggle(
                                ToggleAction::SpeedBrake,
                                &mut sim,
                                &mut ap,
                                &mut mcp,
                                &mut bindings,
                                &mut lights,
                                &weather,
                            );
                        }
                    }
                    LeverId::Gear => {
                        let now = sim.model.get("gear/gear-cmd-norm");
                        if (now > 0.5) != (want > 0.5) {
                            apply_toggle(
                                ToggleAction::Gear,
                                &mut sim,
                                &mut ap,
                                &mut mcp,
                                &mut bindings,
                                &mut lights,
                                &weather,
                            );
                        }
                    }
                }
            }
        }
    }
    *held = if left { now } else { None };
}

/// Where each gate sits now. Flap and throttle come from the same state the
/// flight model reads; the latching three come from the model properties the
/// keyboard writes.
fn lever_value(id: LeverId, bindings: &Bindings, sim: &Sim, state: &AircraftState) -> f64 {
    match id {
        LeverId::Flap => bindings
            .binding(Action::Flap)
            .map(|b| b.value)
            .unwrap_or(0.0),
        LeverId::Speedbrake => sim.model.get("fcs/speedbrake-cmd-norm"),
        LeverId::ThrottleLeft | LeverId::ThrottleRight => state.throttle,
        LeverId::Gear => sim.model.get("gear/gear-cmd-norm"),
    }
}

fn sync_levers(
    bindings: Res<Bindings>,
    sim: Res<Sim>,
    state: Res<AircraftState>,
    mut q: Query<(&LeverPart, &mut Node)>,
) {
    for (part, mut node) in &mut q {
        let l = lever(part.id);
        let v = lever_value(part.id, &bindings, &sim, &state);
        let knob_y = l.y(v);
        match part.part {
            Part::Knob => {
                node.top = Val::Px((knob_y - l.knob.y * 0.5) * part.scale);
            }
            Part::Stem => {
                let top = knob_y * part.scale;
                node.top = Val::Px(top);
                node.height = Val::Px(((l.channel.y1 - knob_y) * part.scale).max(0.0));
            }
        }
    }
}

fn sync_fills(state: Res<AircraftState>, mut q: Query<(&Fill, &mut Node)>) {
    for (fill, mut node) in &mut q {
        match *fill {
            Fill::Flap { scale } => {
                let frac = flap_unit((state.flap_deg / 40.0).clamp(0.0, 1.0)) as f32;
                let h = FLAP_TRACK.h() * frac;
                node.top = Val::Px((FLAP_TRACK.y1 - h) * scale);
                node.height = Val::Px((h * scale).max(0.0));
            }
            Fill::N2 { scale } => {
                let frac = (state.rpm / 100.0).clamp(0.0, 1.0) as f32;
                let track = N2_TRACKS[0].1;
                let inner = track.h() - 6.0;
                let h = inner * frac;
                node.top = Val::Px((track.y1 - 3.0 - h) * scale);
                node.height = Val::Px((h * scale).max(0.0));
            }
        }
    }
}

fn sync_readouts(state: Res<AircraftState>, mut q: Query<(&Readout, &mut Text, &mut TextColor)>) {
    for (readout, mut text, mut color) in &mut q {
        let want = match readout {
            // One decimal, not a degree glyph: at strip scale the sign reads as
            // a zero, and during transit the tenths are the interesting part.
            Readout::FlapDeg => format!("{:.1}", state.flap_deg),
            Readout::N2 => format!("{:.0}", state.rpm),
        };
        if text.0 != want {
            text.0 = want;
        }
        color.0 = match readout {
            Readout::FlapDeg => Color::srgb(1.0, 0.80, 0.42),
            Readout::N2 if !state.engine_running => Color::srgba(0.80, 0.88, 0.80, 0.30),
            Readout::N2 => Color::srgb(0.78, 0.86, 0.76),
        };
    }
}

fn lamp_color(id: LightId, lit: bool) -> Color {
    let on = match id {
        LightId::GearNose | LightId::GearLeft | LightId::GearRight | LightId::At => {
            Color::srgb(0.18, 0.95, 0.33)
        }
        LightId::GearUnsafe => Color::srgb(1.0, 0.12, 0.08),
        LightId::GearTransit | LightId::Rev | LightId::SpdBrk => Color::srgb(1.0, 0.64, 0.10),
    };
    if lit {
        on
    } else {
        let c = on.to_srgba();
        Color::srgba(c.red * 0.10, c.green * 0.10, c.blue * 0.10, 1.0)
    }
}

/// `cmd` is the handle's commanded position (1 = down), `pos` the achieved one.
/// The red UNSAFE lamp means "handle down, gear not locked", so it must not
/// burn in cruise with the gear cleanly up.
fn lamp_lit(id: LightId, state: &AircraftState, ap: &Autopilot, gear_cmd: f64) -> bool {
    let pos = state.gear_pos;
    match id {
        LightId::GearNose | LightId::GearLeft | LightId::GearRight => pos > 0.999,
        LightId::GearUnsafe => gear_cmd > 0.5 && pos < 0.999,
        LightId::GearTransit => pos > 0.001 && pos < 0.999,
        LightId::At => ap.at != AtMode::Off,
        LightId::Rev => state.reverse_pos > 0.05,
        LightId::SpdBrk => state.speedbrake_pos > 0.05,
    }
}

fn sync_lamps(
    sim: Res<Sim>,
    state: Res<AircraftState>,
    ap: Res<Autopilot>,
    mut q: Query<(&StandLamp, &mut BackgroundColor, &mut BoxShadow)>,
) {
    let gear_cmd = sim.model.get("gear/gear-cmd-norm");
    for (lamp, mut bg, mut shadow) in &mut q {
        let lit = lamp_lit(lamp.0, &state, &ap, gear_cmd);
        bg.0 = lamp_color(lamp.0, lit);
        if let Some(style) = shadow.0.first_mut() {
            style.color = if lit {
                let c = bg.0.to_srgba();
                Color::srgba(c.red, c.green, c.blue, 0.45)
            } else {
                Color::NONE
            };
        }
    }
}

fn sync_reverse(sim: Res<Sim>, mut q: Query<&mut BackgroundColor, With<ReverseFill>>) {
    let armed = sim.model.get("fcs/reverse-cmd-norm") > 0.5;
    for mut bg in &mut q {
        bg.0 = if armed {
            Color::srgba(1.0, 0.64, 0.10, 0.22)
        } else {
            Color::NONE
        };
    }
}

struct Look {
    body: Color,
    edge: Color,
    high: Color,
    grip: Color,
}

fn lever_look(id: LeverId) -> Look {
    match id {
        LeverId::Flap | LeverId::Gear => Look {
            body: Color::srgb(0.60, 0.58, 0.53),
            edge: Color::srgb(0.80, 0.78, 0.72),
            high: Color::srgb(0.72, 0.70, 0.64),
            grip: Color::srgb(0.22, 0.21, 0.19),
        },
        LeverId::Speedbrake => Look {
            body: Color::srgb(0.20, 0.19, 0.18),
            edge: Color::srgb(0.42, 0.40, 0.37),
            high: Color::srgb(0.30, 0.29, 0.27),
            grip: Color::srgb(0.72, 0.30, 0.10),
        },
        LeverId::ThrottleLeft | LeverId::ThrottleRight => Look {
            body: Color::srgb(0.11, 0.11, 0.105),
            edge: Color::srgb(0.55, 0.54, 0.50),
            high: Color::srgb(0.34, 0.34, 0.32),
            grip: Color::srgb(0.78, 0.76, 0.70),
        },
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::pedestal_layout::TRAVEL_BOTTOM;

    #[test]
    fn a_click_on_the_flap_scale_lands_on_its_stop() {
        let area = lever(LeverId::Flap).hit;
        // The printed "15" caption sits at the 15-degree stop.
        let y = lever(LeverId::Flap).y(15.0 / 40.0);
        let norm_y = (y - area.y0) / area.h() - 0.5;
        let want = spot_value(StandSpot::Lever(LeverId::Flap), area, norm_y).unwrap();
        assert!((want - 15.0 / 40.0).abs() < 1e-9);
    }

    #[test]
    fn the_gear_gate_commands_the_half_you_click() {
        let area = lever(LeverId::Gear).hit;
        let top = (0.15 - 0.5) as f32;
        let bottom = (0.85 - 0.5) as f32;
        assert_eq!(
            spot_value(StandSpot::Lever(LeverId::Gear), area, top).unwrap(),
            0.0
        );
        assert_eq!(
            spot_value(StandSpot::Lever(LeverId::Gear), area, bottom).unwrap(),
            1.0
        );
    }

    #[test]
    fn the_reverse_bar_has_no_position() {
        assert!(spot_value(StandSpot::Reverse, REVERSE_BAR, 0.0).is_none());
    }

    #[test]
    fn the_hero_fits_the_viewport() {
        // Landscape: width-limited, with margins on both sides.
        let w = hero_width(Vec2::new(1920.0, 1080.0));
        assert!(w < 1920.0);
        assert!(w * CROP_H / CROP_W < 1080.0);
        // Portrait: still width-limited (a wide strip), and the label column
        // is not clipped.
        let portrait = hero_width(Vec2::new(1024.0, 1280.0));
        assert!((portrait - 1024.0 / HERO.fit.x).abs() < 1e-3);
        // A very wide, short window becomes height-limited.
        let short = hero_width(Vec2::new(4000.0, 600.0));
        assert!(short < 4000.0 / HERO.fit.x);
        assert!((short * CROP_H / CROP_W - 600.0 / HERO.fit.y).abs() < 1e-3);
    }

    #[test]
    fn the_strip_hides_when_the_window_is_short_or_in_the_cockpit() {
        assert!(stand_visible(true, 1080.0, false));
        assert!(!stand_visible(true, 899.0, false));
        assert!(!stand_visible(true, 1080.0, true));
        assert!(!stand_visible(false, 1080.0, false));
    }

    #[test]
    fn the_throttle_value_is_the_state_not_the_binding() {
        // The autothrottle writes the model after `Bindings`; the handle must
        // follow the state, or it would lie while the A/T is flying.
        let mut sim = Sim::new("737");
        sim.model.set("fcs/throttle-cmd-norm", 0.42);
        let state = AircraftState {
            throttle: 0.42,
            ..Default::default()
        };
        let bindings = Bindings::for_aircraft("737");
        let v = lever_value(LeverId::ThrottleLeft, &bindings, &sim, &state);
        assert!((v - 0.42).abs() < 1e-9);
    }

    #[test]
    fn the_flap_pointer_lags_the_lever() {
        // The lever goes to 40 at once; the surface takes 22 s, so at t=0 the
        // pointer is still at UP while the handle is at the top.
        let mut sim = Sim::new("737");
        let mut state = AircraftState::default();
        let mut bindings = Bindings::for_aircraft("737");
        bindings.set(Action::Flap, 1.0);
        let lever_v = lever_value(LeverId::Flap, &bindings, &sim, &state);
        assert_eq!(lever_v, 1.0);
        state.flap_deg = 0.0;
        assert_eq!(flap_unit(state.flap_deg / 40.0), 0.0);
        let _ = &mut sim;
    }

    #[test]
    fn the_gear_lamp_is_lit_only_when_locked_down() {
        let mut state = AircraftState::default();
        let ap = Autopilot::default();
        state.gear_pos = 1.0;
        assert!(lamp_lit(LightId::GearNose, &state, &ap, 1.0));
        assert!(!lamp_lit(LightId::GearUnsafe, &state, &ap, 1.0));
        state.gear_pos = 0.5;
        assert!(!lamp_lit(LightId::GearNose, &state, &ap, 1.0));
        assert!(lamp_lit(LightId::GearUnsafe, &state, &ap, 1.0));
        assert!(lamp_lit(LightId::GearTransit, &state, &ap, 1.0));
        // Gear cleanly up: the red light is out, the amber transit is out.
        state.gear_pos = 0.0;
        assert!(!lamp_lit(LightId::GearUnsafe, &state, &ap, 0.0));
        assert!(!lamp_lit(LightId::GearTransit, &state, &ap, 0.0));
    }

    #[test]
    fn the_gear_handle_is_drawn_at_its_gate() {
        let l = lever(LeverId::Gear);
        assert!(l.y(1.0) <= TRAVEL_BOTTOM + 1.0);
    }

    fn press(app: &mut App, key: KeyCode) {
        app.world_mut()
            .resource_mut::<ButtonInput<KeyCode>>()
            .press(key);
        app.update();
    }

    #[test]
    fn semicolon_toggles_the_chase_strip() {
        let mut app = App::new();
        app.init_resource::<ButtonInput<KeyCode>>()
            .init_resource::<StandView>()
            .add_systems(Update, toggle_stand);
        assert!(app.world().resource::<StandView>().open);
        press(&mut app, KeyCode::Semicolon);
        assert!(!app.world().resource::<StandView>().open);
    }

    #[test]
    fn semicolon_raises_the_stand_in_the_cockpit() {
        let mut app = App::new();
        app.init_resource::<ButtonInput<KeyCode>>()
            .init_resource::<StandView>()
            .insert_resource(Ui {
                cockpit: true,
                ..default()
            })
            .add_systems(Update, toggle_stand);
        press(&mut app, KeyCode::Semicolon);
        assert_eq!(
            app.world().resource::<Ui>().panel_focus,
            Some(PanelFocus::Pedestal)
        );
        // The chase strip's own flag is untouched; the hero owns the cockpit.
        assert!(app.world().resource::<StandView>().open);
    }

    #[test]
    fn escape_lowers_the_stand() {
        let mut app = App::new();
        app.init_resource::<ButtonInput<KeyCode>>()
            .insert_resource(Ui {
                cockpit: true,
                panel_focus: Some(PanelFocus::Pedestal),
                ..default()
            })
            .add_systems(Update, close_on_escape);
        press(&mut app, KeyCode::Escape);
        assert!(!app.world().resource::<Ui>().has_focus());
    }
}
