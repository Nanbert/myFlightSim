//! Menu page model: pages paint as pooled UI rows, not one big string.
//!
//! The interactive pages (GLOBAL SETTINGS, per-aircraft SETTINGS, PLAY,
//! MISSIONS) assemble a `Vec<PageLine>` every frame; `render_page_lines`
//! paints it into a pool of row nodes, so a cursor row reads as a lit bar
//! over amber values instead of an ASCII `> WEATHER   < STORM >` block.
//!
//! [`PageLine`] carries no Bevy types and is cheap to build every frame;
//! the renderer owns all styling.

use bevy::prelude::*;

use crate::ui_theme;

/// Vocabulary for one row of a page card.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LineKind {
    /// The page's title strip (aircraft label + page name).
    Heading,
    /// Read-only copy on the main text colour.
    Body,
    /// Secondary copy: row hints, notes. Amber, dimmer than a value.
    Note,
    /// A keyboard-cursor row: label left, value right, lit when `active`.
    Option,
    /// Key hints; smaller and dimmed.
    Hint,
}

/// One rendered row of a menu page. `left` is the primary label, `right` the
/// value cell (empty for single-column lines).
#[derive(Clone, Debug)]
pub struct PageLine {
    pub kind: LineKind,
    pub left: String,
    pub right: String,
    /// `Option` rows only: this is the row the keyboard cursor sits on.
    pub active: bool,
}

impl PageLine {
    pub fn heading(text: impl Into<String>) -> Self {
        Self::plain(LineKind::Heading, text)
    }

    pub fn body(text: impl Into<String>) -> Self {
        Self::plain(LineKind::Body, text)
    }

    pub fn note(text: impl Into<String>) -> Self {
        Self::plain(LineKind::Note, text)
    }

    pub fn hint(text: impl Into<String>) -> Self {
        Self::plain(LineKind::Hint, text)
    }

    /// A spacer line: keeps the prose rhythm of the old text pages without
    /// holding a text node that does nothing. Builders push one between
    /// sections; the renderer gives it a fixed height.
    pub fn blank() -> Self {
        Self::plain(LineKind::Body, "")
    }

    pub fn option(label: impl Into<String>, value: impl Into<String>, active: bool) -> Self {
        Self {
            kind: LineKind::Option,
            left: label.into(),
            right: value.into(),
            active,
        }
    }

    fn plain(kind: LineKind, text: impl Into<String>) -> Self {
        Self {
            kind,
            left: text.into(),
            right: String::new(),
            active: false,
        }
    }
}

/// Folds the freeform pages (ABOUT / INFO / the TUTORIAL prose, which still
/// render as one string) into lines: the first line becomes the heading,
/// `[...]` lines become key hints, blanks become spacers. Body lines keep
/// their leading indent so sub-steps stay visually nested.
pub fn freeform(text: &str) -> Vec<PageLine> {
    let mut out = Vec::new();
    for raw in text.lines() {
        let line = raw.trim_end();
        if line.is_empty() {
            out.push(PageLine::blank());
            continue;
        }
        if out.is_empty() {
            out.push(PageLine::heading(line.trim()));
        } else if line.trim_start().starts_with('[') {
            out.push(PageLine::hint(line.trim_start()));
        } else {
            out.push(PageLine::body(line));
        }
    }
    out
}

/// What `update_menu_ui` fills every frame from the live state.
#[derive(Resource, Default)]
pub struct PageLines(pub Vec<PageLine>);

/// Pooled slots: the widest page (the C172's PLAY rows with hints, notes and
/// key hints) stays well inside this.
pub const MAX_LINES: usize = 40;

/// A pooled page row: the bar node holding a left and a right text cell.
#[derive(Component)]
pub struct PageRow(pub usize);
#[derive(Component)]
pub struct PageLeft;
#[derive(Component)]
pub struct PageRight;

/// Spawns the row pool under the page card. Card children stretch to the
/// card's content width by default, so each row paints a full-width bar.
pub fn spawn_rows(commands: &mut Commands, card: Entity) {
    for i in 0..MAX_LINES {
        let row = commands
            .spawn((
                PageRow(i),
                Node {
                    flex_direction: FlexDirection::Row,
                    justify_content: JustifyContent::SpaceBetween,
                    column_gap: Val::Px(24.0),
                    display: Display::None,
                    padding: UiRect {
                        left: Val::Px(12.0),
                        right: Val::Px(12.0),
                        top: Val::Px(5.0),
                        bottom: Val::Px(5.0),
                    },
                    border_radius: BorderRadius::all(Val::Px(4.0)),
                    ..default()
                },
                BorderColor::all(Color::NONE),
                ChildOf(card),
            ))
            .id();
        // The two cells; distinct marker types, so they can't be spawned from
        // one homogeneous array.
        commands.spawn((
            Text::new(""),
            TextFont {
                font_size: FontSize::Px(20.0),
                ..default()
            },
            TextColor(Color::NONE),
            PageLeft,
            ChildOf(row),
        ));
        commands.spawn((
            Text::new(""),
            TextFont {
                font_size: FontSize::Px(20.0),
                ..default()
            },
            TextColor(Color::NONE),
            PageRight,
            ChildOf(row),
        ));    }
}

/// Paints `PageLines` into the pool. Registered directly after
/// `update_menu_ui` (`chain`), so each frame's rows render with this frame's
/// state.
pub fn render_page_lines(
    lines: Res<PageLines>,
    mut rows: Query<(Entity, &PageRow, &mut Node, &mut BackgroundColor, &mut BorderColor)>,
    kids: Query<&Children>,
    mut texts: Query<&mut Text>,
    mut fonts: Query<&mut TextFont>,
    mut colors: Query<&mut TextColor>,
) {
    let note_amber = ui_theme::amber().with_alpha(0.80);
    for (row_e, row, mut node, mut bg, mut border) in &mut rows {
        let Some(line) = lines.0.get(row.0) else {
            if node.display != Display::None {
                node.display = Display::None;
            }
            continue;
        };
        if node.display != Display::Flex {
            node.display = Display::Flex;
        }
        let content = !line.left.is_empty() || !line.right.is_empty();
        if !content && node.min_height != Val::Px(24.0) {
            node.min_height = Val::Px(24.0);
        }
        // A cursor row = translucent bar + a lit left rail; everything else
        // stays a floating line of text. Every write here is guarded:
        // unconditional `Mut` writes mark the node changed and force a layout
        // pass for the whole card on every menu frame.
        let lit = content && line.kind == LineKind::Option && line.active;
        let want_bg = if lit {
            ui_theme::accent_soft()
        } else {
            Color::NONE
        };
        if bg.0 != want_bg {
            bg.0 = want_bg;
        }
        let want_rail = if lit { ui_theme::accent() } else { Color::NONE };
        if border.left != want_rail {
            *border = BorderColor {
                top: Color::NONE,
                right: Color::NONE,
                bottom: Color::NONE,
                left: want_rail,
            };
        }
        let Ok(children) = kids.get(row_e) else {
            continue;
        };
        for (slot, child) in children.iter().take(2).enumerate() {
            let text = if slot == 0 { &line.left } else { &line.right };
            if let Ok(mut t) = texts.get_mut(child) {
                if t.0 != *text {
                    t.0.clone_from(text);
                }
            }
            let (color, size) = match (slot, line.kind, content) {
                (0, LineKind::Heading, _) => (ui_theme::accent(), 26.0),
                (1, LineKind::Option, _) if line.active => (ui_theme::amber(), 20.0),
                (1, LineKind::Option, _) => (note_amber, 20.0),
                (0, LineKind::Note, _) => (note_amber, 20.0),
                (0, LineKind::Hint, _) => (ui_theme::dim(), 16.0),
                _ if content => (ui_theme::text(), 20.0),
                _ => (Color::NONE, 20.0), // spacer line: no glyphs to show
            };
            if let Ok(mut c) = colors.get_mut(child) {
                if c.0 != color {
                    c.0 = color;
                }
            }
            if let Ok(mut f) = fonts.get_mut(child) {
                let want = FontSize::Px(size);
                if f.font_size != want {
                    f.font_size = want;
                }
            }
        }
    }
}
