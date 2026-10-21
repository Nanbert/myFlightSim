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

/// Bright phosphor green: the active/selected accent, borders, indicators.
pub fn accent() -> Color {
    Color::srgb(0.36, 0.94, 0.52)
}

/// Faint green for borders, cursor rails and the selection bar under a top row.
pub fn accent_soft() -> Color {
    Color::srgba(0.36, 0.94, 0.52, 0.30)
}

/// Amber — reserved for values and warnings, never for structure.
#[allow(dead_code)] // palette completeness; the flight-UI phases will use it
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
