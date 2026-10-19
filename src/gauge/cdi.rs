//! The Cessna-172 course deviation indicator — the VOR/ILS head.
//!
//! A KI-209-shaped simplified analog CDI. Behind the dots sits a rotating
//! compass course card; the course selector isn't wired yet, so the card
//! follows the active MYFS ILS course (36 or 18) — the runway's own OBS,
//! in effect. The vertical deviation needle swings ~22° per dot in the
//! direction you must fly to rejoin the course (needle left = course to the
//! left), and the amber-flagged glideslope pointer slides against the right
//! dot column, up when the beam is above you. Off the beam volume the white
//! `TO` flag drops and an amber `OFF` flag shows, with the needles centred —
//! what the real head does without signal.
//!
//! Data comes straight from `crate::ils::Ils`, recomputed inside `advance_fdm`
//! each step. The same numbers feed the 737's PFD, so the C172 and the seven-
//! three agree as they fly into the same beams.

use bevy::prelude::*;

use super::*;
use crate::ils::{GS_DOT_DEG, LOC_DOT_DEG, Ils};

/// Needle swing per dot of localizer deviation; two dots (half scale) is 45°,
/// which puts the needle tips on the outer dots with the `len` below.
const LOC_DEG_PER_DOT: f32 = 22.5;
/// Half-scale in dots either side.
const DOTS: f64 = 2.0;
/// GS pointer travel per dot, in dial-radius units (the GS dot spacing).
const GS_STEP: f32 = 0.2;

/// The rotating course card.
#[derive(Component)]
pub(super) struct CdiCard;
#[derive(Component)]
pub(super) struct CdiNeedle;
#[derive(Component)]
pub(super) struct CdiGsPointer;

/// The flag window labels — one visible at a time.
#[derive(Component, Clone, Copy, PartialEq)]
pub(super) enum CdiFlag {
    /// White triangle-ish `TO`: beam captured, fly to the threshold.
    To,
    /// Amber `OFF`: outside the approach volume, no signal.
    Off,
}

// --- Rasters ----------------------------------------------------------------

const CARD: [u8; 4] = [20, 20, 24, 255];
const CARD_RING: [u8; 4] = [218, 218, 222, 255];
const BEZEL: [u8; 4] = [14, 14, 16, 255];
const BEZEL_HI: [u8; 4] = [46, 48, 54, 255];
const BEZEL_RIM: [u8; 4] = [104, 108, 116, 255];
const DOT: [u8; 4] = [226, 226, 230, 255];
const BAR: [u8; 4] = [60, 62, 66, 255];

/// The rotating card: a numerals ring around the rim, transparent centre so
/// the dot rows painted on the face stay visible through it.
fn card_sample(x: f32, y: f32) -> [u8; 4] {
    let u = (x * x + y * y).sqrt() / DIAL_R;
    if u > 0.88 {
        return [0, 0, 0, 0];
    }
    let ang = y.atan2(x).to_degrees();
    let cw = (ang - 270.0).rem_euclid(360.0); // clockwise from the top
    let step = (cw / 30.0).round() * 30.0;
    let d = (cw - step).abs();
    let mut c = CARD;
    if d < 0.5 && (0.80..0.90).contains(&u) {
        c = CARD_RING;
    }
    c
}

/// The fixed part: bezel, the top course index, the dot rows and the GS
/// column. Centre transparent — the card shows through behind the needle.
fn face_sample(x: f32, y: f32) -> [u8; 4] {
    let u = (x * x + y * y).sqrt() / DIAL_R;
    if u > 1.0 {
        return [0, 0, 0, 0];
    }
    if u >= 0.88 {
        return if u > 0.985 {
            BEZEL_RIM
        } else if u > 0.95 {
            BEZEL_HI
        } else {
            BEZEL
        };
    }
    let (nx, ny) = (x / DIAL_R, y / DIAL_R);

    // Course index: a small white wedge at the top of the bezel.
    let ang = y.atan2(x).to_degrees();
    let cw = (ang - 270.0).rem_euclid(360.0);
    let cw_signed = if cw > 180.0 { cw - 360.0 } else { cw };
    if (0.885..0.945).contains(&u) && cw_signed.abs() < 4.0 {
        return DOT;
    }

    let dot_row = |dx: f32, dy: f32| -> bool {
        let r = (dx * dx + dy * dy).sqrt();
        r < 0.036 && r > 0.012
    };
    // LOC dots: a row across the middle, ±1/±2 dots, no dot in the middle
    // (that lane belongs to the needle's pivot).
    for k in [-2.0, -1.0, 1.0, 2.0] {
        if dot_row(nx - k * 0.22, ny) {
            return DOT;
        }
    }
    // GS dots: a vertical column on the right, ±1/±2 dots either side.
    for k in [-2.0, -1.0, 0.0, 1.0, 2.0] {
        if dot_row(nx - 0.48, ny - k * GS_STEP) {
            return DOT;
        }
    }
    // The two reference bars the LOC needle tips swing across: dark bands
    // either side of centre along the dot row, like the real head's shading.
    if ny.abs() < 0.05 && (0.10..0.16).contains(&nx.abs()) {
        return BAR;
    }
    [0, 0, 0, 0]
}

// --- Scene ------------------------------------------------------------------

/// Offset from the dial centre to the point `r` px out at clockwise angle
/// `cw` degrees from the top.
fn at(r: f32, cw_deg: f32) -> (f32, f32) {
    let a = (270.0 + cw_deg).to_radians();
    (r * a.cos(), r * a.sin())
}

/// A centred text label carrying a [`CdiFlag`] marker, so the update can
/// toggle its `Node::display`.
fn spawn_flag(p: &mut ChildSpawnerCommands, text: &str, x: f32, y: f32, color: Color, flag: CdiFlag) {
    let box_px = 48.0 * FS;
    p.spawn((
        Text::new(text),
        TextFont {
            font_size: FontSize::Px(15.0 * FS),
            ..default()
        },
        TextColor(color),
        Node {
            position_type: PositionType::Absolute,
            left: Val::Px(x - box_px * 0.5),
            top: Val::Px(y - box_px * 0.5),
            width: Val::Px(box_px),
            height: Val::Px(box_px),
            justify_content: JustifyContent::Center,
            align_items: AlignItems::Center,
            display: if flag == CdiFlag::To {
                Display::Flex
            } else {
                Display::None
            },
            ..default()
        },
        flag,
    ));
}

pub(super) fn setup(mut commands: Commands, mut images: ResMut<Assets<Image>>) {
    let card = images.add(make_image(raster_face(card_sample)));
    let face = images.add(make_image(raster_face(face_sample)));
    let needle = images.add(make_image(needle_pixels(
        &NeedleShape {
            len: 0.62,
            base_w: 0.016,
            tip_w: 0.010,
            tail: 0.52,
            tail_w: 0.022,
            hub: 0.045,
        },
        [238, 238, 242, 255],
        [40, 40, 46, 255],
    )));
    let gs = images.add(make_image(needle_pixels(
        &NeedleShape {
            len: 0.17,
            base_w: 0.05,
            tip_w: 0.05,
            tail: 0.05,
            tail_w: 0.02,
            hub: 0.0,
        },
        [232, 186, 84, 255],
        [40, 40, 46, 255],
    )));

    let c = NODE * 0.5;
    // Mirrors the fuel gauge's second row, on the right cluster this time:
    // the bottom row's three-dial contract belongs to the chase-view discs.
    let root = spawn_root(&mut commands, PanelKey::Cdi, Side::Right, 2, 1);
    commands.entity(root).with_children(|p| {
        // The course card turns under the fixed index.
        p.spawn((
            ImageNode::new(card),
            Node {
                position_type: PositionType::Absolute,
                left: Val::Px(0.0),
                top: Val::Px(0.0),
                width: Val::Px(NODE),
                height: Val::Px(NODE),
                ..default()
            },
            UiTransform::IDENTITY,
            CdiCard,
        ))
        .with_children(|card| {
            for k in 0..12 {
                let text = format!("{}", k * 30);
                let (x, y) = at(DIAL_PX * 0.62, (k * 30) as f32);
                spawn_label(card, &text, c + x, c + y, 15.0, Color::WHITE);
            }
        });

        spawn_image(p, face, ());
        spawn_label(p, "ILS", c, c - DIAL_PX * 0.60, 12.0, Color::WHITE);
        // The master switch of the flag windows: TO while the beam holds,
        // OFF outside the volume.
        spawn_flag(p, "TO", c, c + DIAL_PX * 0.42, Color::WHITE, CdiFlag::To);
        spawn_flag(
            p,
            "OFF",
            c,
            c + DIAL_PX * 0.42,
            Color::srgb(0.92, 0.65, 0.10),
            CdiFlag::Off,
        );

        spawn_image(p, needle, CdiNeedle);
        spawn_image(p, gs, CdiGsPointer);
    });
}

/// Swing the card to the active beam's course, the LOC needle to the lateral
/// deviation and the GS pointer along the dot column; swap the TO/OFF flags.
pub(super) fn update(
    ils: Res<Ils>,
    mut card: Query<&mut UiTransform, (With<CdiCard>, Without<CdiNeedle>, Without<CdiGsPointer>)>,
    mut needle: Query<&mut UiTransform, (With<CdiNeedle>, Without<CdiCard>, Without<CdiGsPointer>)>,
    mut gs: Query<&mut UiTransform, (With<CdiGsPointer>, Without<CdiCard>, Without<CdiNeedle>)>,
    mut flags: Query<(&CdiFlag, &mut Node), Without<CdiCard>>,
) {
    let info = ils.available;

    if let Ok(mut t) = card.single_mut() {
        // Same mechanism as the heading card: the numeral for the course sits
        // under the fixed index when rotated by -(course).
        t.rotation = Rot2::degrees(-(ils.course_deg as f32));
    }
    if let Ok(mut t) = needle.single_mut() {
        // Off-beam: the needle hangs centred, like the real head's scaled-zero
        // fall-back. On beam: deflect 22.5° *per dot* toward the direction to
        // fly — right of course drives it left.
        let dots = if info {
            (ils.loc_dev_deg / LOC_DOT_DEG).clamp(-DOTS, DOTS) as f32
        } else {
            0.0
        };        t.rotation = Rot2::degrees(-(dots * LOC_DEG_PER_DOT));
    }
    if let Ok(mut t) = gs.single_mut() {
        // Fly toward the pointer: below the beam it slides up the column. The
        // texture points up, so rotate it to a horizontal bar first.
        let dots = if info {
            (ils.gs_dev_deg / GS_DOT_DEG).clamp(-DOTS, DOTS) as f32
        } else {
            0.0
        };
        t.rotation = Rot2::degrees(90.0);
        // The dot column is painted at nx = 0.48 on the face; the pointer
        // image sits centred on the dial, so park it on the column.
        t.translation = Val2::px(0.48 * DIAL_PX, (dots * GS_STEP) * DIAL_PX);
    }
    for (flag, mut node) in &mut flags {
        node.display = match (info, flag) {
            (true, CdiFlag::To) => Display::Flex,
            (false, CdiFlag::Off) => Display::Flex,
            _ => Display::None,
        };
    }
}
