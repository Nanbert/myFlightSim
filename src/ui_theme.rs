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
            .add_systems(Update, apply_default_font);
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
}

/// One of the five floating flight-report panels (takeoff grade, landing
/// roll, approach banner, checklist card, hop ledger). They render the same
/// 14 px monospace box — dark translucent pane, hairline border, absolute pin,
/// `GlobalZIndex(14)`, hidden until their update system fills the text in.
/// Only the marker component and the anchor previously varied, which used to
/// mean five near-identical 23-line spawn blocks. Call sites pass their unit
/// marker via `M::default()`.
pub fn float_report<M: Component + Default>(commands: &mut Commands, anchor: ReportAnchor) {
    let (left, right, top, bottom) = match anchor {
        ReportAnchor::RightTop(y) => (Val::Auto, Val::Px(18.0), Val::Px(y), Val::Auto),
        ReportAnchor::LeftTop(y) => (Val::Px(18.0), Val::Auto, Val::Px(y), Val::Auto),
        ReportAnchor::LeftBottom(y) => (Val::Px(18.0), Val::Auto, Val::Auto, Val::Px(y)),
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
macro_rules! blend_material {
    ($mat:ty, $path:literal) => {
        impl Material for $mat {
            fn fragment_shader() -> ShaderRef {
                $path.into()
            }
            fn alpha_mode(&self) -> AlphaMode {
                AlphaMode::Blend
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
pub fn set_display<M: Component>(mut q: Query<&mut Node, With<M>>, want: Display) {
    for mut node in q.iter_mut() {
        if node.display != want {
            node.display = want;
        }
    }
}
