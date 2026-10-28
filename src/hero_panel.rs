//! Shared geometry and motion for the cockpit's hero overlays — the MCP panel
//! and the pedestal stand. Both are a centred crop of a baked image, fitted to
//! the viewport, dollied in on focus, dimmed by a scrim and dismissed by a
//! click outside. Only the crop, the fit margins, the focus slot and the
//! marker components differ, so that machinery lives here once and each panel
//! keeps a thin system wrapper.

use bevy::input::mouse::MouseButton;
use bevy::prelude::*;
use bevy::window::PrimaryWindow;

use crate::view::{PanelFocus, Ui};

/// The scrim's tone, and how opaque it is at full focus: the cockpit shows
/// through undimmed enough to read the PFDs, and is never blacked out.
const SCRIM_RGB: [f32; 3] = [0.02, 0.02, 0.02];
pub const SCRIM_ALPHA: f32 = 0.40;

/// Raise speed: a slow dolly-in reads as deliberate. Dismiss is faster — a
/// slow fade-out reads as a laggy click.
const RAISE_K: f32 = 9.0;
const CLOSE_K: f32 = 55.0;

/// The ease constant for a frame: `closing` picks the fast dismiss. `dt` is
/// frame time in seconds.
pub fn ease_k(dt: f32, closing: bool) -> f32 {
    let k = if closing { CLOSE_K } else { RAISE_K };
    (1.0_f64 - (-(k as f64 * dt as f64)).exp()) as f32
}

/// One hero overlay's static geometry: the baked crop (asset pixels) and the
/// margin factors it is fitted with.
#[derive(Clone, Copy)]
pub struct HeroSpec {
    pub crop: Vec2,
    pub fit: Vec2,
}

impl HeroSpec {
    /// The panel's on-screen width at `viewport`.
    pub fn width(&self, viewport: Vec2) -> f32 {
        let by_width = viewport.x / self.fit.x;
        let by_height = viewport.y / self.fit.y * (self.crop.x / self.crop.y);
        by_width.min(by_height)
    }

    /// The dolly scale at progress `e`: 94% when the raise starts, 100% when
    /// it settles.
    pub fn scale(&self, viewport: Vec2, e: f32) -> f32 {
        (self.width(viewport) / self.crop.x) * (0.94 + 0.06 * e)
    }

    /// The panel's on-screen height at `viewport`.
    pub fn height(&self, viewport: Vec2) -> f32 {
        self.width(viewport) * self.crop.y / self.crop.x
    }

    /// Advance `e` toward `want` by one frame, snapping when close so the
    /// panel stops re-laying out.
    pub fn step(&self, dt: f32, want: f32, e: &mut f32) {
        let closing = want < *e;
        *e += (want - *e) * ease_k(dt, closing);
        if (want - *e).abs() < 0.002 {
            *e = want;
        }
    }

    /// True when `cursor` is over the centred panel.
    pub fn contains(&self, viewport: Vec2, cursor: Vec2) -> bool {
        let size = Vec2::new(self.width(viewport), self.height(viewport));
        Rect::from_center_size(viewport * 0.5, size).contains(cursor)
    }

    /// The scrim at progress `e`: clear at 0, deepest at 1.
    pub fn scrim(&self, e: f32) -> Color {
        Color::srgba(SCRIM_RGB[0], SCRIM_RGB[1], SCRIM_RGB[2], scrim_alpha(e))
    }
}

/// The scrim's opacity at progress `e` in the raise.
pub fn scrim_alpha(e: f32) -> f32 {
    SCRIM_ALPHA * e
}

/// The viewport size, with the old stage's default when no window exists yet.
pub fn viewport(windows: &Query<&Window, With<PrimaryWindow>>) -> Vec2 {
    windows
        .single()
        .map(|window| window.size())
        .unwrap_or(Vec2::new(1280.0, 800.0))
}

pub fn set_display(node: &mut Node, show: bool) {
    let display = if show { Display::Flex } else { Display::None };
    if node.display != display {
        node.display = display;
    }
}

/// A press outside closes the panel, but the click that opened it must not.
pub fn outside_click_closes(was_focused: bool, just_pressed: bool, inside: bool) -> bool {
    was_focused && just_pressed && !inside
}

/// The shared body of both panels' `sync_hero`: advance the animation while
/// the cockpit is up, then show and scale the root and fade its scrim.
pub fn sync<Root: Component, Back: Component>(
    focus: PanelFocus,
    spec: &HeroSpec,
    time: &Time,
    ui: Option<&Ui>,
    anim: &mut f32,
    windows: &Query<&Window, With<PrimaryWindow>>,
    root: &mut Query<(&mut Node, &mut UiTransform), With<Root>>,
    backdrop: &mut Query<(&mut Node, &mut BackgroundColor), (With<Back>, Without<Root>)>,
) {
    let cockpit = ui.is_some_and(|ui| ui.cockpit);
    let focused = cockpit && ui.is_some_and(|ui| ui.is_focused(focus));
    let want = if focused { 1.0 } else { 0.0 };
    if cockpit {
        spec.step(time.delta_secs(), want, anim);
    } else {
        *anim = 0.0;
    }

    let show = cockpit && (focused || *anim > 0.001);
    let scale = spec.scale(viewport(windows), *anim);
    for (mut node, mut tf) in root.iter_mut() {
        set_display(&mut node, show);
        tf.scale = Vec2::splat(scale);
    }
    // The scrim fades with the panel, so the cockpit returns at the same
    // moment the panel leaves.
    for (mut node, mut color) in backdrop.iter_mut() {
        set_display(&mut node, show);
        color.0 = spec.scrim(*anim);
    }
}

/// The shared body of both panels' `hero_pointer`: an outside left press
/// clears the focus slot, ignoring the press that opened the panel.
pub fn pointer_closes(
    focus: PanelFocus,
    spec: &HeroSpec,
    mouse: &ButtonInput<MouseButton>,
    windows: &Query<&Window, With<PrimaryWindow>>,
    ui: Option<&mut Ui>,
    was_focused: &mut bool,
) -> bool {
    let Some(ui) = ui else {
        return false;
    };
    let focused = ui.cockpit && ui.is_focused(focus);
    if !focused {
        *was_focused = false;
        return false;
    }
    let Some(window) = windows.single().ok() else {
        return false;
    };
    let Some(cursor) = window.cursor_position() else {
        return false;
    };
    let inside = spec.contains(window.size(), cursor);
    let closes = outside_click_closes(
        *was_focused,
        mouse.just_pressed(MouseButton::Left),
        inside,
    );
    *was_focused = true;
    if closes {
        ui.panel_focus = None;
    }
    closes
}

#[cfg(test)]
mod tests {
    use super::*;

    const HERO: HeroSpec = HeroSpec {
        crop: Vec2::new(1100.0, 200.0),
        fit: Vec2::new(1.14, 1.20),
    };

    #[test]
    fn width_is_fitted_to_the_viewport() {
        // Portrait: width-limited, exactly `FIT_W` margins.
        assert!((HERO.width(Vec2::new(1024.0, 1280.0)) - 1024.0 / 1.14).abs() < 1e-3);
        // A very wide, short viewport becomes height-limited.
        let short = HERO.width(Vec2::new(4000.0, 600.0));
        assert!((short - 600.0 / 1.20 * (1100.0 / 200.0)).abs() < 1e-3);
        assert!(short < 4000.0 / 1.14);
    }

    #[test]
    fn dismiss_is_faster_than_raise() {
        let dt = 1.0 / 60.0;
        assert!(ease_k(dt, true) > ease_k(dt, false));
    }

    #[test]
    fn the_click_that_opens_does_not_close() {
        assert!(!outside_click_closes(false, true, false));
        assert!(outside_click_closes(true, true, false));
        assert!(!outside_click_closes(true, true, true));
        assert!(!outside_click_closes(true, false, false));
    }
}
