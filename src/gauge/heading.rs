//! The Cessna-172 heading indicator (directional gyro).
//!
//! A gyroscopic compass card that turns *under* a fixed index at the top. The
//! card carries the compass rose — a tick every 5°, longer every 10°, and a
//! numeral every 30° with the cardinals as `N`/`E`/`S`/`W` — and the whole card
//! rotates so the current heading sits under the index. Unlike a magnetic
//! compass it reads a steady heading in turns, but it drifts and must be re-synced
//! to the compass; the sim publishes the true heading, so the card follows it.

use bevy::prelude::*;

use super::*;
use crate::sim::AircraftState;

/// Card disc radius, as a fraction of the dial radius.
const CARD_R: f32 = 0.90;
/// Sweep of the tick ring: inner radius (by class) out to the rim.
const TICK_OUT: f32 = 0.80;

/// The rotating compass card.
#[derive(Component)]
pub(super) struct HeadingCard;

// --- Face rasterisation ---------------------------------------------------

const CARD: [u8; 4] = [20, 20, 24, 255];
const TICK: [u8; 4] = [226, 226, 230, 255];
const BEZEL: [u8; 4] = [14, 14, 16, 255];
const BEZEL_HI: [u8; 4] = [46, 48, 54, 255];
const BEZEL_RIM: [u8; 4] = [104, 108, 116, 255];
const HUB: [u8; 4] = [210, 210, 214, 255];

/// The card: rose ticks on a dark disc, transparent outside.
fn card_sample(x: f32, y: f32) -> [u8; 4] {
    let u = (x * x + y * y).sqrt() / DIAL_R;
    if u > CARD_R {
        return [0, 0, 0, 0];
    }
    let ang = y.atan2(x).to_degrees();
    let cw = (ang - 270.0).rem_euclid(360.0); // clockwise from the top
    let step = (cw / 5.0).round() * 5.0;
    let d = (cw - step).abs();
    let inner = if (step % 30.0).abs() < 0.1 {
        0.64
    } else if (step % 10.0).abs() < 0.1 {
        0.68
    } else {
        0.74
    };
    let mut c = CARD;
    if d < 0.55 && u >= inner && u < TICK_OUT {
        c = TICK;
    }
    // Small hub cap in the middle, under the fixed aircraft symbol.
    if u < 0.035 {
        c = HUB;
    }
    c
}

/// The fixed parts: the bezel ring, the index at the top, and the little
/// aircraft symbol in the centre.
fn frame_sample(x: f32, y: f32) -> [u8; 4] {
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

    // Index: a triangle pointing down at the top of the dial.
    let ang = y.atan2(x).to_degrees();
    let cw = (ang - 270.0).rem_euclid(360.0);
    let cw_signed = if cw > 180.0 { cw - 360.0 } else { cw };
    if (0.80..0.875).contains(&u) {
        let half = 5.0 * (u - 0.80) / 0.075; // widens inward → a downward wedge
        if cw_signed.abs() < half.max(0.6) {
            return TICK;
        }
    }

    // Aircraft symbol: a slim cross with a tail, all inside the tick ring.
    let (ax, ay) = (x / DIAL_R, y / DIAL_R);
    let body = ax.abs() < 0.012 && ay.abs() < 0.20;
    let wings = ay.abs() < 0.012 && ax.abs() < 0.18;
    let tail = (ay - 0.19).abs() < 0.012 && ax.abs() < 0.075;
    if body || wings || tail {
        return HUB;
    }
    [0, 0, 0, 0]
}

// --- Scene ----------------------------------------------------------------

/// Offset from the dial centre to the point `r` px out at clockwise angle
/// `cw` degrees from the top.
fn at(r: f32, cw_deg: f32) -> (f32, f32) {
    let a = (270.0 + cw_deg).to_radians();
    (r * a.cos(), r * a.sin())
}

pub(super) fn setup(mut commands: Commands, mut images: ResMut<Assets<Image>>) {
    let card = images.add(make_image(raster_face(card_sample)));
    let frame = images.add(make_image(raster_face(frame_sample)));

    let c = NODE * 0.5;
    let root = spawn_root(&mut commands, PanelKey::Heading, Side::Right, 0, 0);
    commands.entity(root).with_children(|p| {
        // The card carries its numerals so they turn with it.
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
            HeadingCard,
        ))
        .with_children(|card| {
            for k in 0..12 {
                let text = match k * 30 {
                    0 => "N".to_string(),
                    90 => "E".to_string(),
                    180 => "S".to_string(),
                    270 => "W".to_string(),
                    h => format!("{}", h / 10),
                };
                let (x, y) = at(DIAL_PX * 0.50, (k * 30) as f32);
                spawn_label(card, &text, c + x, c + y, 17.0, Color::WHITE);
            }
        });

        spawn_image(p, frame, ());
    });
}

/// Turn the card so the live heading sits under the fixed index.
pub(super) fn update_card(
    state: Res<AircraftState>,
    mut q: Query<&mut UiTransform, With<HeadingCard>>,
) {
    let Ok(mut t) = q.single_mut() else {
        return;
    };
    t.rotation = Rot2::degrees(-(state.heading_deg as f32));
}
