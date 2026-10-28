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
use crate::hero_panel::{self, HeroSpec};
use crate::mcp_layout::{CROP_H, CROP_W};
use crate::mcp_ui::{spawn_panel_contents, FmaLabel, FocusAnim, FMA_BG, FMA_GREEN};
use crate::sim::Sim;
use crate::view::{PanelFocus, Ui};

/// The panel leaves this much of the viewport free on each axis, matching the
/// old 3D stage's fit. It is wider than the HUD margins so the labels breathe.
const HERO: HeroSpec = HeroSpec {
    crop: Vec2::new(CROP_W, CROP_H),
    fit: Vec2::new(1.14, 1.20),
};

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
        BackgroundColor(HERO.scrim(1.0)),
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

/// The panel width that fits the viewport — the FMA bar rides the same fit.
fn hero_width(viewport: Vec2) -> f32 {
    HERO.width(viewport)
}

fn sync_hero(
    time: Res<Time>,
    ui: Option<Res<Ui>>,
    mut anim: ResMut<FocusAnim>,
    windows: Query<&Window, With<PrimaryWindow>>,
    mut root: Query<(&mut Node, &mut UiTransform), With<HeroPanel>>,
    mut backdrop: Query<(&mut Node, &mut BackgroundColor), (With<HeroBackdrop>, Without<HeroPanel>)>,
) {
    hero_panel::sync::<HeroPanel, HeroBackdrop>(
        PanelFocus::Mcp,
        &HERO,
        &time,
        ui.as_deref(),
        &mut anim.e,
        &windows,
        &mut root,
        &mut backdrop,
    );
}

fn sync_hero_fma(
    ui: Option<Res<Ui>>,
    anim: Res<FocusAnim>,
    windows: Query<&Window, With<PrimaryWindow>>,
    mut bar: Query<&mut Node, With<HeroFma>>,
) {
    let show = ui.as_deref().is_some_and(|ui| ui.cockpit) && anim.e > 0.25;
    let viewport = hero_panel::viewport(&windows);
    let hero_w = hero_width(viewport);
    let hero_h = HERO.height(viewport);
    let left = (viewport.x - hero_w) * 0.5;
    let top = ((viewport.y - hero_h) * 0.5 - 30.0).max(8.0);

    for mut node in &mut bar {
        hero_panel::set_display(&mut node, show);
        if show {
            node.left = Val::Px(left);
            node.top = Val::Px(top);
            node.width = Val::Px(hero_w);
        }
    }
}

fn hero_pointer(
    mouse: Res<ButtonInput<MouseButton>>,
    windows: Query<&Window, With<PrimaryWindow>>,
    mut ui: Option<ResMut<Ui>>,
    mut was_focused: Local<bool>,
) {
    hero_panel::pointer_closes(
        PanelFocus::Mcp,
        &HERO,
        &mouse,
        &windows,
        ui.as_deref_mut(),
        &mut was_focused,
    );
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::hero_panel::{ease_k, outside_click_closes, scrim_alpha, SCRIM_ALPHA};

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
        assert!((portrait - 1024.0 / HERO.fit.x).abs() < 1e-3);
        // Landscape: still width-limited, and it leaves side margins.
        let landscape = hero_width(Vec2::new(1920.0, 1080.0));
        assert!((landscape - 1920.0 / HERO.fit.x).abs() < 1e-3);
        assert!(landscape < 1920.0);
        // A very wide, short viewport becomes height-limited.
        let short = hero_width(Vec2::new(4000.0, 600.0));
        assert!((short - 600.0 / HERO.fit.y * (CROP_W / CROP_H)).abs() < 1e-3);
        assert!(short < 4000.0 / HERO.fit.x);
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
        const { assert!(SCRIM_ALPHA < 0.5) };
    }
}
