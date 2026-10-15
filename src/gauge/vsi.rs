//! The Cessna-172 vertical speed indicator.
//!
//! Zero sits at 9 o'clock. Climb swings the needle clockwise (up, over the
//! top); descent swings it counter-clockwise (down, under the bottom). The
//! scale is *logarithmic* — the dial gives far more room to the first few
//! hundred fpm than to the last, because holding altitude is where you need
//! the resolution. `5, 10, 15, 20` read as hundreds of feet per minute, so the
//! full scale is ±2,000 fpm.

use bevy::prelude::*;

use super::*;
use crate::sim::AircraftState;

const MAX_FPM: f32 = 2000.0;
/// Curvature of the log scale: larger = straighter (more linear) dial.
const K: f32 = 200.0;
/// Zero at 9 o'clock, and the full-scale swing either side of it.
const ZERO_CW: f32 = 270.0;
const SPAN: f32 = 150.0;

/// Fraction of full-scale swing for a given rate, 0..1 (logarithmic).
fn frac(fpm: f32) -> f32 {
    (1.0 + fpm.abs() / K).ln() / (1.0 + MAX_FPM / K).ln()
}

/// The dial's scale: rate (fpm, + = climb) → clockwise degrees from the top.
fn fpm_to_cw(fpm: f32) -> f32 {
    let off = SPAN * frac(fpm.clamp(-MAX_FPM, MAX_FPM));
    if fpm >= 0.0 {
        ZERO_CW + off
    } else {
        ZERO_CW - off
    }
}

#[derive(Component)]
pub(super) struct VsiNeedle;

// --- Face rasterisation ---------------------------------------------------

const FACE: [u8; 4] = [16, 16, 18, 255];
const BEZEL: [u8; 4] = [14, 14, 16, 255];
const BEZEL_HI: [u8; 4] = [46, 48, 54, 255];
const BEZEL_RIM: [u8; 4] = [104, 108, 116, 255];
const TICK: [u8; 4] = [228, 228, 228, 255];

/// Per-angle tick class (0 none, 1 minor, 2 major), one entry a spoke.
struct Tables {
    tick: Vec<u8>,
}

const BUCKETS: usize = 1440; // 4 buckets per degree

fn build_tables() -> Tables {
    let mut t = Tables {
        tick: vec![0; BUCKETS],
    };
    // A tick every 100 fpm, longer and heavier every 500.
    for k in -20..=20i32 {
        let fpm = k as f32 * 100.0;
        let (class, half_w) = if k % 5 == 0 { (2u8, 0.55) } else { (1u8, 0.35) };
        let center = fpm_to_cw(fpm);
        let mut d = -half_w;
        while d <= half_w {
            let b = ((center + d).rem_euclid(360.0) / 360.0 * BUCKETS as f32) as usize % BUCKETS;
            if t.tick[b] < class {
                t.tick[b] = class;
            }
            d += 0.06;
        }
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
        2 => 0.70,
        1 => 0.79,
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
    let needle = images.add(make_image(needle_pixels(
        &NeedleShape {
            len: 0.78,
            base_w: 0.012,
            tip_w: 0.004,
            tail: 0.10,
            tail_w: 0.006,
            hub: 0.030,
        },
        [232, 232, 238, 255],
        [40, 40, 46, 255],
    )));

    let c = NODE * 0.5;
    let root = spawn_root(&mut commands, Side::Left, 2, 0);
    commands.entity(root).with_children(|p| {
        spawn_image(p, face, ());

        // Hundred-fpm numerals, mirrored above (climb) and below (descent).
        for h in [5.0f32, 10.0, 15.0, 20.0] {
            for fpm in [h * 100.0, -h * 100.0] {
                let (x, y) = at(DIAL_PX * 0.66, fpm_to_cw(fpm));
                spawn_label(p, &format!("{h:.0}"), c + x, c + y, 18.0, Color::WHITE);
            }
        }

        // Centre captions, to the right of the needle hub.
        let dim = Color::srgb(0.85, 0.85, 0.85);
        spawn_label(p, "VERTICAL", c + DIAL_PX * 0.20, c - DIAL_PX * 0.14, 12.0, dim);
        spawn_label(p, "SPEED", c + DIAL_PX * 0.20, c - DIAL_PX * 0.02, 12.0, dim);
        spawn_label(p, "100 FEET", c + DIAL_PX * 0.24, c + DIAL_PX * 0.18, 10.0, dim);
        spawn_label(p, "PER MIN", c + DIAL_PX * 0.24, c + DIAL_PX * 0.29, 10.0, dim);

        spawn_image(p, needle, VsiNeedle);
    });
}

/// Point the needle at the live vertical speed. The needle texture points up,
/// so the rotation is the scale angle measured clockwise from the top.
pub(super) fn update_needle(
    state: Res<AircraftState>,
    mut q: Query<&mut UiTransform, With<VsiNeedle>>,
) {
    let Ok(mut t) = q.single_mut() else {
        return;
    };
    t.rotation = Rot2::degrees(fpm_to_cw(state.vs_fpm as f32));
}
