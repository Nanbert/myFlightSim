//! Shared UI theme: the avionics palette and the single UI font face.
//!
//! The sim deliberately renders cockpit-style UI — flat dark panes, phosphor
//! green accents, amber for values — so the menus echo the in-cockpit MFD/HUD
//! look instead of a generic OS theme. Everything visual should pull its
//! colors from here; nothing hard-codes a `Color::srgb` outside this module.
//!
//! Fonts: the whole app shares one monospace face (`ShareTechMono-Regular`,
//! SIL OFL — see `assets/fonts/OFL.txt`). A patch system rewrites the engine's
//! default font face on every `TextFont` as it spawns, so flight UIs (combat,
//! gauges, checklist, ...) pick it up without touching each spawn site.

use bevy::prelude::*;
use bevy::text::FontSize;

/// Bright phosphor green: the active/selected accent, borders, indicators.
pub fn accent() -> Color {
    Color::srgb(0.36, 0.94, 0.52)
}

/// Faint green for borders, cursor rails and the selection bar under a top row.
pub fn accent_soft() -> Color {
    Color::srgba(0.36, 0.94, 0.52, 0.30)
}

/// Amber — the value/secondary copy across menus and MFD-like UI.
pub fn amber() -> Color {
    Color::srgb(1.00, 0.72, 0.10)
}

/// An entry that is currently pointed at but doesn't hold keyboard focus.
pub fn selected() -> Color {
    Color::srgb(0.58, 0.88, 0.66)
}

/// Inert labels and hints: legible, clearly not interactive.
pub fn dim() -> Color {
    Color::srgb(0.40, 0.52, 0.44)
}

/// Body copy on panels: soft green-white, easier on the eyes than pure white.
pub fn text() -> Color {
    Color::srgb(0.80, 0.90, 0.82)
}

/// Panel background: near-black with a green cast, translucent enough to keep
/// the hangar visible around the card.
pub fn panel_bg() -> Color {
    Color::srgba(0.012, 0.031, 0.020, 0.90)
}

/// Soft outer glow behind a panel; evokes backlit cockpit glass.
pub fn panel_glow() -> Color {
    Color::srgba(0.15, 0.60, 0.32, 0.32)
}

/// The one face every `TextFont` carries unless something opts out.
#[derive(Resource)]
pub struct UiFonts {
    pub mono: Handle<Font>,
}

pub struct UiThemePlugin;

impl Plugin for UiThemePlugin {
    fn build(&self, app: &mut App) {
        let mono = app
            .world()
            .resource::<AssetServer>()
            .load("fonts/ShareTechMono-Regular.ttf");
        app.insert_resource(UiFonts { mono })
            .add_systems(Update, apply_default_font)
            // The floating reports are spawned once at `Startup` and their
            // display is written only while unpaused, so leaving the flight
            // must hide them all together — see `hide_float_panels`.
            .add_systems(
                OnExit(crate::app_state::AppState::Flying),
                hide_float_panels,
            );
    }
}

/// Rewrites the engine's fallback font (the embedded Fira Mono subset) to the
/// theme face on every freshly spawned `TextFont`. One system here beats a
/// font argument at every text spawn site across the codebase.
fn apply_default_font(fonts: Res<UiFonts>, mut q: Query<&mut TextFont, Added<TextFont>>) {
    for mut tf in &mut q {
        let ours = match &tf.font {
            FontSource::Handle(h) => h.id() == fonts.mono.id(),
            _ => false,
        };
        if !ours {
            tf.font = FontSource::Handle(fonts.mono.clone());
        }
    }
}

/// Screen anchor for [`float_report`]. The floating report panels always hang
/// 18 px from their chosen edge; the payload is the y offset in px, measured
/// from the top (or, for the bottom variants, from the bottom).
pub enum ReportAnchor {
    RightTop(f32),
    LeftTop(f32),
    LeftBottom(f32),
    /// Dead centre of the window: the panel becomes the child of a full-screen
    /// flex wrapper (an absolute panel keeps its own insets, so it can't be
    /// flex-centred from the inside).
    Center,
}

/// Marker on everything [`float_report`] spawns. One shared `OnExit` system
/// hides them all, so a panel added later cannot forget its own hide and
/// linger over the menu.
#[derive(Component, Default)]
pub struct FloatPanel;

/// One of the floating flight-report panels (takeoff grade, landing roll,
/// approach banner, checklist card, hop ledger, lesson prompt). They render the
/// same 14 px monospace box — dark translucent pane, hairline border, absolute
/// pin, `GlobalZIndex(14)`, hidden until their update system fills the text in.
/// Only the marker component and the anchor vary. Call sites pass their unit
/// marker via `M::default()`.
pub fn float_report<M: Component + Default>(commands: &mut Commands, anchor: ReportAnchor) {
    if matches!(anchor, ReportAnchor::Center) {
        let wrapper = commands
            .spawn((Node {
                position_type: PositionType::Absolute,
                width: Val::Percent(100.0),
                height: Val::Percent(100.0),
                align_items: AlignItems::Center,
                justify_content: JustifyContent::Center,
                ..default()
            }, Pickable::IGNORE))
            .id();
        commands.spawn((
            M::default(),
            FloatPanel,
            Text::new(""),
            TextFont {
                font_size: FontSize::Px(14.0),
                ..default()
            },
            TextColor(Color::srgb(0.92, 0.94, 0.90)),
            BackgroundColor(Color::srgba(0.02, 0.03, 0.04, 0.80)),
            Node {
                padding: UiRect::all(Val::Px(10.0)),
                border: UiRect::all(Val::Px(1.0)),
                display: Display::None,
                ..default()
            },
            BorderColor::all(Color::srgba(0.7, 0.75, 0.7, 0.35)),
            GlobalZIndex(14),
            Pickable::IGNORE,
            ChildOf(wrapper),
        ));
        return;
    }
    let (left, right, top, bottom) = match anchor {
        ReportAnchor::RightTop(y) => (Val::Auto, Val::Px(18.0), Val::Px(y), Val::Auto),
        ReportAnchor::LeftTop(y) => (Val::Px(18.0), Val::Auto, Val::Px(y), Val::Auto),
        ReportAnchor::LeftBottom(y) => (Val::Px(18.0), Val::Auto, Val::Auto, Val::Px(y)),
        ReportAnchor::Center => unreachable!("handled above"),
    };
    let node = Node {
        position_type: PositionType::Absolute,
        left,
        right,
        top,
        bottom,
        padding: UiRect::all(Val::Px(10.0)),
        border: UiRect::all(Val::Px(1.0)),
        display: Display::None,
        ..default()
    };

    commands.spawn((
        M::default(),
        FloatPanel,
        Text::new(""),
        TextFont {
            font_size: FontSize::Px(14.0),
            ..default()
        },
        TextColor(Color::srgb(0.92, 0.94, 0.90)),
        BackgroundColor(Color::srgba(0.02, 0.03, 0.04, 0.80)),
        node,
        BorderColor::all(Color::srgba(0.7, 0.75, 0.7, 0.35)),
        GlobalZIndex(14),
        Pickable::IGNORE,
    ));
}

// --- Bevy Material impl boilerplate -----------------------------------------
//
// Every custom `Material` in the sim carries the same three "calm the renderer
// down" overrides: null depth prepass, null shadow pass, and (for translucent
// effects) `AlphaMode::Blend`. Nine copies of that four-method impl used to be
// hand-typed next to each shader type; these two macros replace them. `mat`
// must be a unit-ish struct deriving `AsBindGroup` + `ShaderType`.

/// A translucent `Material` (`AlphaMode::Blend`) bound to one `.wesl` shader:
/// `blend_material!(PuffMaterial, "shaders/vapor_puff.wesl");`
///
/// An optional third argument biases the material's transparent sort distance
/// in metres (see `Material::depth_bias`): the transparent phase draws the
/// smallest `view depth + bias` key first, and view depth runs negative in
/// front of the camera — so a **positive** bias always draws the material
/// **last**, over every other translucent surface (clouds, sea), a negative
/// one always first. Same-material entities still order among themselves by
/// real distance, so the bias only moves a whole family, never reorders
/// inside it.
macro_rules! blend_material {
    ($mat:ty, $path:literal) => {
        blend_material!($mat, $path, 0.0);
    };
    ($mat:ty, $path:literal, $bias:expr) => {
        impl Material for $mat {
            fn fragment_shader() -> ShaderRef {
                $path.into()
            }
            fn alpha_mode(&self) -> AlphaMode {
                AlphaMode::Blend
            }
            fn depth_bias(&self) -> f32 {
                $bias
            }
            fn enable_prepass() -> bool {
                false
            }
            fn enable_shadows() -> bool {
                false
            }
        }
    };
}
pub(crate) use blend_material;

/// Transparent sort biases, metres (see `blend_material!`). The transparent
/// phase sorts ascending on `rangefinder.distance + bias` — view depth, which
/// runs *negative* in front of the camera — and draws the smallest key first
/// (back to front). A positive bias therefore pushes a material toward the
/// end of the draw order (on top), a negative one toward the start (under).
/// Both dwarf the distances a player can span inside the 48 km arena, so no
/// genuine view distance can outweigh them.
/// `TOP` pins a family (wing vapor, exhaust plume) to draw **last**: over the
/// cloud cards and the sea, which mis-sort it because their huge footprints
/// sort by a distant centre. `BOTTOM` pins the sea to draw **first**, under
/// everything translucent.
pub(crate) const SORT_TOP_BIAS: f32 = 60_000.0;
pub(crate) const SORT_BOTTOM_BIAS: f32 = -60_000.0;

/// An opaque `Material` bound to one `.wesl` shader:
/// `opaque_material!(TerrainMaterial, "shaders/terrain.wesl");`
macro_rules! opaque_material {
    ($mat:ty, $path:literal) => {
        impl Material for $mat {
            fn fragment_shader() -> ShaderRef {
                $path.into()
            }
            fn enable_prepass() -> bool {
                false
            }
            fn enable_shadows() -> bool {
                false
            }
        }
    };
}
pub(crate) use opaque_material;

/// A fixed-size box centred on the screen, optionally offset in px. Used by
/// the cockpit crosshair and its tick marks.
pub fn centered(w: f32, h: f32, dx: f32, dy: f32) -> Node {
    Node {
        position_type: PositionType::Absolute,
        left: Val::Percent(50.0),
        top: Val::Percent(50.0),
        width: Val::Px(w),
        height: Val::Px(h),
        margin: UiRect {
            left: Val::Px(-w * 0.5 + dx),
            top: Val::Px(-h * 0.5 + dy),
            ..default()
        },
        ..default()
    }
}

/// Set every listed node's `display` to `want`, writing only on change (the
/// read-then-write keeps Bevy change detection from replaying the render sync
/// each frame). All the "hide this overlay unless state X" systems converge
/// here; their bodies shrink to computing `want`.
/// A dim one-row footer of key hints, styled the same everywhere it appears
/// (lesson popups, the pause plate): actions spread across the row, each
/// cell's key in the accent colour and its label dimmed. Keys keep whatever
/// bracket/spelling the caller passes (`"[R]"` or `"Enter"`). The row lands
/// under `parent` (the card it summarizes).
pub fn key_footer(
    commands: &mut Commands,
    parent: Entity,
    gap_above: f32,
    items: &[(&str, &str)],
) {
    let footer = commands
        .spawn((
            Node {
                margin: UiRect::top(Val::Px(gap_above)),
                justify_content: JustifyContent::SpaceBetween,
                column_gap: Val::Px(24.0),
                ..default()
            },
            ChildOf(parent),
        ))
        .id();
    for (key, label) in items {
        let cell = commands
            .spawn((
                // Base text stays empty: the spans below are the whole cell
                // (a base text would double-render the row).
                Text::new(""),
            ))
            .id();
        for (text, color) in [
            (format!("{key} "), accent()),
            ((*label).to_string(), dim()),
        ] {
            commands.spawn((
                TextSpan::new(text),
                TextFont {
                    font_size: FontSize::Px(14.0),
                    ..default()
                },
                TextColor(color),
                ChildOf(cell),
            ));
        }
        commands.entity(footer).add_child(cell);
    }
}

/// Hide every floating report on leaving the flight; the flight systems that
/// would otherwise rewrite their display are gated on `unpaused` and stop.
fn hide_float_panels(q: Query<&mut Node, With<FloatPanel>>) {
    set_display(q, Display::None);
}

pub fn set_display<M: Component>(mut q: Query<&mut Node, With<M>>, want: Display) {
    for mut node in q.iter_mut() {
        if node.display != want {
            node.display = want;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The translucent draw order is a sandwich: the sea always first
    /// (`BOTTOM` sorts farthest), cloud cards by their real distance in the
    /// middle, the vapor and plume families always last (`TOP` sorts nearest).
    /// The biases only have to dwarf the distances a *player* can span — the
    /// 48 km arena boundary — so an in-game view distance can never outweigh
    /// them. (Beyond the boundary the plain distance may win, and then it is
    /// also the physically correct order.)
    #[test]
    fn sort_biases_sandwich_the_translucent_order() {
        // `TOP` positive draws last (nearest key), `BOTTOM` negative first;
        // the magnitude checks imply the signs.
        assert!(
            SORT_TOP_BIAS > 48_000.0 && SORT_BOTTOM_BIAS.abs() > 48_000.0,
            "an in-arena distance could outweigh the bias"
        );
        // The cloud cards take the macro's two-argument form (bias 0, the
        // bevy default): they keep the plain distance sort — the sandwich's
        // middle layer.
    }
}
