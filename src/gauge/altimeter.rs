//! The Cessna-172 three-pointer altimeter.
//!
//! The dial is numbered 0–9 with `0` at the top, one digit every 36°, running
//! clockwise — each digit is 100 ft. Three needles share that dial:
//!
//! * the long, thin *hundreds* needle — one revolution per 1,000 ft, sweeping
//!   the numbered scale;
//! * the thick *thousands* needle — one revolution per 10,000 ft;
//! * the thin *ten-thousands* needle, tipped with a pointer — one revolution
//!   per 100,000 ft.
//!
//! So 3,827 ft reads: tens-needle a hair past 8, hundreds-needle near 3, and
//! the thousands-needle just short of 0. The face (ticks, numerals' ring, the
//! pressure window card) is rasterised once; the needles are textures rotated
//! every frame from `alt_ft`.

use bevy::prelude::*;

use super::*;
use crate::sim::AircraftState;

/// Feet per revolution of the hundreds (100-ft) needle.
const FT_PER_REV: f32 = 1000.0;

/// The dial's linear scale: altitude (ft) → clockwise degrees from the top.
fn ft_to_cw(ft: f32) -> f32 {
    ft / FT_PER_REV * 360.0
}

/// Kollsman (pressure-setting) window: centre and half-extents, texture px
/// offsets from the dial centre. Sits at 3 o'clock, in the gap between the `2`
/// and `3` numerals.
const KOL_C: (f32, f32) = (154.0, 0.0);
const KOL_H: (f32, f32) = (30.0, 20.0);
/// The baro setting the window displays. The sim exposes no altimeter-setting
/// property, so this is the fixed standard-datum 1013 hPa.
const KOL_HPA: &str = "1013";

#[derive(Component)]
pub(super) struct AltNeedle {
    /// Feet per revolution.
    rev: f32,
}

// --- Face rasterisation ---------------------------------------------------

const FACE: [u8; 4] = [16, 16, 18, 255];
const BEZEL: [u8; 4] = [14, 14, 16, 255];
const BEZEL_HI: [u8; 4] = [46, 48, 54, 255];
const BEZEL_RIM: [u8; 4] = [104, 108, 116, 255];
const TICK: [u8; 4] = [228, 228, 228, 255];

/// The pressure-window card in the *texture* is skipped — it is drawn as a UI
/// node instead, so the card and its readout text share one coordinate frame.
/// Per-angle tick class (0 none, 1 minor, 2 major), one entry a spoke.
struct Tables {
    tick: Vec<u8>,
}

const BUCKETS: usize = 1440; // 4 buckets per degree

fn build_tables() -> Tables {
    let mut t = Tables {
        tick: vec![0; BUCKETS],
    };
    // A tick every 20 ft, longer and heavier every 100 ft (at each numeral).
    let mut ft = 0.0f32;
    while ft <= FT_PER_REV + 0.1 {
        let major = (ft as i32) % 100 == 0;
        let (class, half_w) = if major { (2u8, 0.7) } else { (1u8, 0.45) };
        let center = 270.0 + ft_to_cw(ft); // screen angle (up = 270°)
        let mut d = -half_w;
        while d <= half_w {
            let b = ((center + d).rem_euclid(360.0) / 360.0 * BUCKETS as f32) as usize % BUCKETS;
            if t.tick[b] < class {
                t.tick[b] = class;
            }
            d += 0.06;
        }
        ft += 20.0;
    }
    t
}

/// The colour of one dial sample at offset `(x, y)` from the centre (px).
fn face_sample(t: &Tables, x: f32, y: f32) -> [u8; 4] {
    let u = (x * x + y * y).sqrt() / DIAL_R;
    if u > 1.0 {
        return [0, 0, 0, 0];
    }

    if u >= 0.90 {
        return if u > 0.985 {
            BEZEL_RIM
        } else if u > 0.95 {
            BEZEL_HI
        } else {
            BEZEL
        };
    }

    let ang = y.atan2(x).to_degrees();
    let b = (ang.rem_euclid(360.0) / 360.0 * BUCKETS as f32) as usize % BUCKETS;

    let mut c = FACE;
    let inner = match t.tick[b] {
        2 => 0.72,
        1 => 0.80,
        _ => 1.1,
    };
    if u >= inner && u < 0.885 {
        c = TICK;
    }
    c
}

// --- Scene ------------------------------------------------------------------

/// Offset from the dial centre to the point `r` px out at clockwise angle
/// `cw` degrees from the top.
fn at(r: f32, cw_deg: f32) -> (f32, f32) {
    let a = (270.0 + cw_deg).to_radians();
    (r * a.cos(), r * a.sin())
}

pub(super) fn setup(mut commands: Commands, mut images: ResMut<Assets<Image>>) {
    let tables = build_tables();
    let face = images.add(make_image(raster_face(|x, y| face_sample(&tables, x, y))));

    const WHITE: [u8; 4] = [232, 232, 238, 255];
    const HUB: [u8; 4] = [40, 40, 46, 255];
    // Ten-thousands needle: shortest, widening to a pointer tip.
    let n10k = images.add(make_image(needle_pixels(
        &NeedleShape {
            len: 0.48,
            base_w: 0.007,
            tip_w: 0.018,
            tail: 0.09,
            tail_w: 0.005,
            hub: 0.026,
        },
        WHITE,
        HUB,
    )));
    // Thousands needle: thick and heavy.
    let n1k = images.add(make_image(needle_pixels(
        &NeedleShape {
            len: 0.62,
            base_w: 0.030,
            tip_w: 0.015,
            tail: 0.12,
            tail_w: 0.011,
            hub: 0.040,
        },
        WHITE,
        HUB,
    )));
    // Hundreds needle: long and thin, sweeping the numerals.
    let n100 = images.add(make_image(needle_pixels(
        &NeedleShape {
            len: 0.80,
            base_w: 0.016,
            tip_w: 0.006,
            tail: 0.10,
            tail_w: 0.006,
            hub: 0.028,
        },
        WHITE,
        HUB,
    )));

    let c = NODE * 0.5;

    let root = spawn_root(&mut commands, PanelKey::Altimeter, Side::Left, 1, 0);
    commands.entity(root).with_children(|p| {
        spawn_image(p, face, ());

        // Numerals 0–9, one every 36°, 0 at the top.
        for d in 0..10 {
            let (x, y) = at(DIAL_PX * 0.62, d as f32 * 36.0);
            spawn_label(p, &format!("{d}"), c + x, c + y, 20.0, Color::WHITE);
        }

        // Centre caption.
        spawn_label(p, "ALTITUDE", c, c + DIAL_PX * 0.36, 12.0, Color::srgb(0.85, 0.85, 0.85));
        spawn_label(p, "FEET", c, c + DIAL_PX * 0.47, 10.0, Color::srgb(0.75, 0.75, 0.75));

        // Pressure-setting window: a light card holding the readout. The number
        // and its unit are *children* of the card, so flex centring puts them
        // inside it, whatever the surrounding layout does.
        let s = NODE / TEX as f32;
        let (wx, wy) = (c + KOL_C.0 * s, c + KOL_C.1 * s);
        let (w, h) = (2.0 * KOL_H.0 * s, 2.0 * KOL_H.1 * s);
        p.spawn((
            Node {
                position_type: PositionType::Absolute,
                left: Val::Px(wx - w * 0.5),
                top: Val::Px(wy - h * 0.5),
                width: Val::Px(w),
                height: Val::Px(h),
                flex_direction: FlexDirection::Column,
                justify_content: JustifyContent::Center,
                align_items: AlignItems::Center,
                border: UiRect::all(Val::Px(1.5)),
                ..default()
            },
            BackgroundColor(Color::srgb(0.91, 0.89, 0.84)),
            BorderColor::all(Color::srgb(0.14, 0.14, 0.16)),
        ))
        .with_children(|card| {
            card.spawn((
                Text::new(KOL_HPA),
                TextFont {
                    font_size: FontSize::Px(12.0),
                    ..default()
                },
                TextColor(Color::srgb(0.12, 0.12, 0.12)),
            ));
            card.spawn((
                Text::new("hPa"),
                TextFont {
                    font_size: FontSize::Px(8.0),
                    ..default()
                },
                TextColor(Color::srgb(0.30, 0.30, 0.30)),
            ));
        });

        // Needles, coarsest first so the finest is on top.
        spawn_image(p, n10k, AltNeedle { rev: 100_000.0 });
        spawn_image(p, n1k, AltNeedle { rev: 10_000.0 });
        spawn_image(p, n100, AltNeedle { rev: 1_000.0 });
    });
}

/// Rotate each needle to its place on the 0–9 dial. The needle textures point
/// up, so the rotation is exactly the scale angle measured from the top.
pub(super) fn update_needles(
    state: Res<AircraftState>,
    mut q: Query<(&mut UiTransform, &AltNeedle)>,
) {
    let alt = state.alt_ft as f32;
    for (mut t, n) in &mut q {
        let frac = alt.rem_euclid(n.rev) / n.rev;
        t.rotation = Rot2::degrees(frac * 360.0);
    }
}
