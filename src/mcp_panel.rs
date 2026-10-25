//! The 737 Mode Control Panel raised as a 2D panel in the cockpit.
//!
//! Clicking the modelled board (`cockpit.rs` sets `ui.panel_focus`) keeps the
//! cockpit where it is and dims it with a translucent scrim, so the PFDs stay
//! readable behind the panel instead of swapping to a black room. It is the
//! same image and the same `mcp_layout` table as the chase strip, so every
//! hotspot, label and seven-segment window lands in exactly the same place.
//! The only differences are scale (the panel is fitted to the viewport) and a
//! dolly-in on focus.
//!
//! Runtime is plain Bevy UI: a fresh left press outside the panel closes it,
//! Esc does too, and the hotspot systems in `mcp_ui` drive both presentations.

use bevy::input::mouse::MouseButton;
use bevy::prelude::*;
use bevy::window::PrimaryWindow;

use crate::aircraft_profile;
use crate::app_state::{unpaused, AppState};
use crate::mcp_layout::{CROP_H, CROP_W};
use crate::mcp_ui::{spawn_panel_contents, FmaLabel, FocusAnim, FMA_BG, FMA_GREEN};
use crate::sim::Sim;
use crate::view::{PanelFocus, Ui};

/// The panel leaves this much of the viewport free on each axis, matching the
/// old 3D stage's fit. It is wider than the HUD margins so the labels breathe.
const FIT_W: f32 = 1.14;
const FIT_H: f32 = 1.20;

/// The scrim's tone and how opaque it is at full focus. The cockpit shows
/// through undimmed enough to read the PFDs, and is never blacked out.
const SCRIM_RGB: [f32; 3] = [0.02, 0.02, 0.02];
const SCRIM_ALPHA: f32 = 0.40;

/// Raise speed: a slow dolly-in reads as deliberate.
const RAISE_K: f32 = 9.0;
/// Dismiss speed: a slow fade-out reads as a laggy click.
const CLOSE_K: f32 = 55.0;

/// The centred panel. Its node is laid out at the asset size and scaled by
/// `UiTransform`, so all children stay in `mcp_layout` coordinates.
#[derive(Component)]
struct HeroPanel;

#[derive(Component)]
struct HeroBackdrop;

/// The FMA bar above the panel. It is separate so its 22 px height and 15 px
/// text stay screen-sized instead of scaling with the panel.
#[derive(Component)]
struct HeroFma;

pub struct McpPanelPlugin;

impl Plugin for McpPanelPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<FocusAnim>()
            .add_systems(
                OnEnter(AppState::Flying),
                spawn_hero.after(crate::sim::StartFlight),
            )
            .add_systems(
                Update,
                (sync_hero, sync_hero_fma, hero_pointer)
                    .chain()
                    .run_if(unpaused),
            );
    }
}

fn spawn_hero(mut commands: Commands, sim: Res<Sim>, assets: Res<AssetServer>) {
    if !aircraft_profile::profile(&sim.name).has_autopilot {
        return;
    }
    let image = assets.load("ui/737_mcp_3d.png");

    commands.spawn((
        HeroBackdrop,
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
        BackgroundColor(scrim(1.0)),
        GlobalZIndex(29),
        DespawnOnExit(AppState::Flying),
    ));

    commands
        .spawn((
            HeroPanel,
            Pickable::IGNORE,
            ImageNode::new(image),
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
        .with_children(|panel| spawn_panel_contents(panel, 1.0));

    commands
        .spawn((
            HeroFma,
            Pickable::IGNORE,
            Node {
                position_type: PositionType::Absolute,
                height: Val::Px(22.0),
                align_items: AlignItems::Center,
                padding: UiRect::horizontal(Val::Px(8.0)),
                border_radius: BorderRadius::all(Val::Px(3.0)),
                display: Display::None,
                ..default()
            },
            BackgroundColor(FMA_BG),
            GlobalZIndex(31),
            DespawnOnExit(AppState::Flying),
        ))
        .with_children(|bar| {
            bar.spawn((
                FmaLabel,
                Text::new("MANUAL"),
                TextFont {
                    font_size: FontSize::Px(15.0),
                    ..default()
                },
                TextColor(FMA_GREEN),
            ));
        });
}

/// The panel width that fits `viewport` with `FIT_W` / `FIT_H` margins.
fn hero_width(viewport: Vec2) -> f32 {
    let by_width = viewport.x / FIT_W;
    let by_height = viewport.y / FIT_H * (CROP_W / CROP_H);
    by_width.min(by_height)
}

fn viewport(windows: &Query<&Window, With<PrimaryWindow>>) -> Vec2 {
    windows
        .single()
        .map(|window| window.size())
        .unwrap_or(Vec2::new(1280.0, 800.0))
}

fn set_display(node: &mut Node, show: bool) {
    let display = if show { Display::Flex } else { Display::None };
    if node.display != display {
        node.display = display;
    }
}

fn sync_hero(
    time: Res<Time>,
    ui: Option<Res<Ui>>,
    mut anim: ResMut<FocusAnim>,
    windows: Query<&Window, With<PrimaryWindow>>,
    mut root: Query<(&mut Node, &mut UiTransform), With<HeroPanel>>,
    mut backdrop: Query<(&mut Node, &mut BackgroundColor), (With<HeroBackdrop>, Without<HeroPanel>)>,
) {
    let cockpit = ui.as_deref().is_some_and(|ui| ui.cockpit);
    let focused = cockpit && ui.as_deref().is_some_and(|ui| ui.is_focused(PanelFocus::Mcp));
    let want = if focused { 1.0 } else { 0.0 };
    if cockpit {
        // Exponential ease; snap when close so the panel stops re-laying out.
        // Raising and dismissing use different rates: a slow fade-out reads as
        // a laggy dismissal.
        let dt = time.delta_secs();
        anim.e += (want - anim.e) * ease_k(dt, want < anim.e);
        if (want - anim.e).abs() < 0.002 {
            anim.e = want;
        }
    } else {
        anim.e = 0.0;
    }

    let show = cockpit && (focused || anim.e > 0.001);
    // A small dolly: 94% at the start of the raise, 100% when settled.
    let scale = (hero_width(viewport(&windows)) / CROP_W) * (0.94 + 0.06 * anim.e);
    for (mut node, mut tf) in &mut root {
        set_display(&mut node, show);
        tf.scale = Vec2::splat(scale);
    }
    // The scrim fades with the panel, so the cockpit returns at the same
    // moment the panel leaves.
    for (mut node, mut color) in &mut backdrop {
        set_display(&mut node, show);
        color.0 = scrim(anim.e);
    }
}

/// The ease constant for a frame: `closing` picks the fast dismiss. `dt` is
/// frame time in seconds.
fn ease_k(dt: f32, closing: bool) -> f32 {
    let k = if closing { CLOSE_K } else { RAISE_K };
    (1.0_f64 - (-(k as f64 * dt as f64)).exp()) as f32
}

/// The scrim's opacity at progress `e` in the raise.
fn scrim_alpha(e: f32) -> f32 {
    SCRIM_ALPHA * e
}

/// The scrim at progress `e`: clear at 0, deepest at 1.
fn scrim(e: f32) -> Color {
    Color::srgba(SCRIM_RGB[0], SCRIM_RGB[1], SCRIM_RGB[2], scrim_alpha(e))
}

fn sync_hero_fma(
    ui: Option<Res<Ui>>,
    anim: Res<FocusAnim>,
    windows: Query<&Window, With<PrimaryWindow>>,
    mut bar: Query<&mut Node, With<HeroFma>>,
) {
    let show = ui.as_deref().is_some_and(|ui| ui.cockpit) && anim.e > 0.25;
    let viewport = viewport(&windows);
    let hero_w = hero_width(viewport);
    let hero_h = hero_w * CROP_H / CROP_W;
    let left = (viewport.x - hero_w) * 0.5;
    let top = ((viewport.y - hero_h) * 0.5 - 30.0).max(8.0);

    for mut node in &mut bar {
        set_display(&mut node, show);
        if show {
            node.left = Val::Px(left);
            node.top = Val::Px(top);
            node.width = Val::Px(hero_w);
        }
    }
}

/// A press outside closes the panel, but the click that opened it must not.
fn outside_click_closes(was_focused: bool, just_pressed: bool, inside: bool) -> bool {
    was_focused && just_pressed && !inside
}

fn hero_pointer(
    mouse: Res<ButtonInput<MouseButton>>,
    windows: Query<&Window, With<PrimaryWindow>>,
    ui: Option<ResMut<Ui>>,
    mut was_focused: Local<bool>,
) {
    let Some(mut ui) = ui else {
        return;
    };
    let focused = ui.cockpit && ui.is_focused(PanelFocus::Mcp);
    if !focused {
        *was_focused = false;
        return;
    }
    let Some(window) = windows.single().ok() else {
        return;
    };
    let Some(cursor) = window.cursor_position() else {
        return;
    };
    let viewport = window.size();
    let hero_w = hero_width(viewport);
    let hero_h = hero_w * CROP_H / CROP_W;
    let inside = Rect::from_center_size(viewport * 0.5, Vec2::new(hero_w, hero_h)).contains(cursor);
    let closes = outside_click_closes(*was_focused, mouse.just_pressed(MouseButton::Left), inside);
    *was_focused = true;
    if closes {
        ui.panel_focus = None;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_click_that_opens_the_panel_does_not_close_it() {
        // The first focused frame ignores the opening press.
        assert!(!outside_click_closes(false, true, false));
        // A fresh press outside closes; inside never does.
        assert!(outside_click_closes(true, true, false));
        assert!(!outside_click_closes(true, true, true));
        assert!(!outside_click_closes(true, false, false));
    }

    #[test]
    fn the_panel_fits_the_viewport() {
        // Portrait: width-limited, exactly `FIT_W` margins.
        let portrait = hero_width(Vec2::new(1024.0, 1280.0));
        assert!((portrait - 1024.0 / FIT_W).abs() < 1e-3);
        // Landscape: still width-limited, and it leaves side margins.
        let landscape = hero_width(Vec2::new(1920.0, 1080.0));
        assert!((landscape - 1920.0 / FIT_W).abs() < 1e-3);
        assert!(landscape < 1920.0);
        // A very wide, short viewport becomes height-limited.
        let short = hero_width(Vec2::new(4000.0, 600.0));
        assert!((short - 600.0 / FIT_H * (CROP_W / CROP_H)).abs() < 1e-3);
        assert!(short < 4000.0 / FIT_W);
    }

    #[test]
    fn dismissing_is_faster_than_raising() {
        // Same ease, two rates: the dismiss must clear the 0.002 snap
        // threshold in a small handful of frames at 60 fps, so an outside
        // click does not hang.
        let dt = 1.0 / 60.0;
        let mut raising = 0usize;
        let mut e = 1.0_f32;
        while e > 0.002 {
            e -= e * ease_k(dt, false);
            raising += 1;
        }
        let mut closing = 0usize;
        let mut e = 1.0_f32;
        while e > 0.002 {
            e -= e * ease_k(dt, true);
            closing += 1;
        }
        assert!(ease_k(dt, true) > ease_k(dt, false));
        assert!(closing <= 12);
        assert!(closing * 3 < raising);
    }

    #[test]
    fn the_scrim_follows_focus() {
        // Unfocused: the scrim is fully clear, so the cockpit is untouched.
        assert!(scrim_alpha(0.0) == 0.0);
        // Focused: the scrim reaches its deepest alpha — dim, never opaque, so
        // the PFDs stay readable behind the panel.
        assert!((scrim_alpha(1.0) - SCRIM_ALPHA).abs() < 1e-6);
        assert!(SCRIM_ALPHA < 0.5);
    }
}
