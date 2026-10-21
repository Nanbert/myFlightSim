//! The Cessna-172 fuel gauges.
//!
//! One dial, two needles — like the C172S's pair of fuel-quantity gauges
//! compressed to one instrument. The scale runs 0..185 lb per tank (`E` at
//! the 9-o'clock end, `F` at the 3-o'clock end, a 140° arc), each needle
//! reading one wing tank. On multi-tank or JSBSim backends — anything that
//! doesn't publish contents — both needles park at empty, which is honest:
//! the sim genuinely doesn't know the fuel in those models.

use bevy::prelude::*;

use super::*;
use crate::sim::AircraftState;

/// Per-tank capacity (lb), matching the FDM's FuelSpec.
const CAP_LB: f32 = 185.0;
/// `E` sits 70° left of the top, `F` 70° right of it.
const ZERO_CW: f32 = -70.0;
const SPAN: f32 = 140.0;

/// Tank contents (lb) → clockwise degrees from the top.
fn lb_to_cw(lb: f32) -> f32 {
    let frac = (lb / CAP_LB).clamp(0.0, 1.0);
    ZERO_CW + SPAN * frac
}

/// One marker per needle, carrying which wing tank it reads.
#[derive(Component, Clone, Copy)]
pub(super) enum FuelNeedle {
    L,
    R,
}

// --- Face rasterisation ---------------------------------------------------

const REDZONE: [u8; 4] = [168, 40, 34, 255];

/// Per-angle tick class (0 none, 1 minor, 2 major), one entry per spoke.
struct Tables {
    tick: Vec<u8>,
}

fn build_tables() -> Tables {
    let mut t = Tables {
        tick: vec![0; BUCKETS],
    };
    // A tick every 25 lb, heavier every 50.
    for lb in [0.0, 25.0, 50.0, 75.0, 100.0, 125.0, 150.0, 175.0, 185.0] {
        let (class, half_w) = if lb % 50.0 == 0.0 {
            (2u8, 0.55)
        } else {
            (1u8, 0.35)
        };
        let center = lb_to_cw(lb);
        paint_tick(&mut t.tick, center, class, half_w);
    }
    t
}

/// The colour of one dial sample at offset `(x, y)` from the centre (px).
fn face_sample(t: &Tables, x: f32, y: f32) -> [u8; 4] {
    let u = (x * x + y * y).sqrt() / DIAL_R;
    if let Some(c) = bezel_grade(u, 0.90) {
        return c;
    }

    let ang = y.atan2(x).to_degrees();
    let b = ang_bucket(ang);

    let mut c = FACE;
    let inner = match t.tick[b] {
        2 => 0.70,
        1 => 0.79,
        _ => 1.1,
    };
    if u >= inner && u < 0.885 {
        c = TICK;
    }
    // The E-side quarter of the arc is the red no-fuel zone, like the real
    // gauges mark it.
    let cw = ang.rem_euclid(360.0) - 270.0;
    if (0.80..0.885).contains(&u) && cw < ZERO_CW + 0.6 * SPAN {
        c = REDZONE;
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
    let needle_l = images.add(make_image(needle_pixels(
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
    let needle_r = images.add(make_image(needle_pixels(
        &NeedleShape {
            len: 0.78,
            base_w: 0.012,
            tip_w: 0.004,
            tail: 0.10,
            tail_w: 0.006,
            hub: 0.030,
        },
        [214, 168, 66, 255],
        [40, 40, 46, 255],
    )));

    let c = NODE * 0.5;
    // Second row of the left corner stack: the bottom row belongs to the
    // three-dial trio the chase-view discs lay out against, so the fuel
    // gauges sit one row up to keep that geometry intact.
    let root = spawn_root(&mut commands, PanelKey::Fuel, Side::Left, 0, 1);
    commands.entity(root).with_children(|p| {
        spawn_image(p, face, ());

        // Fraction labels across the arc: E, 1/2, F.
        for (lb, text) in [(0.0f32, "E"), (92.5, "1/2"), (185.0, "F")] {
            let (x, y) = at(DIAL_PX * 0.62, lb_to_cw(lb));
            spawn_label(p, text, c + x, c + y, 16.0, Color::WHITE);
        }

        // Wing-tank captions above the hub: which needle reads which tank.
        let dim = Color::srgb(0.85, 0.85, 0.85);
        spawn_label(
            p,
            "L",
            c - DIAL_PX * 0.26,
            c - DIAL_PX * 0.02,
            12.0,
            Color::WHITE,
        );
        spawn_label(
            p,
            "R",
            c + DIAL_PX * 0.26,
            c - DIAL_PX * 0.02,
            12.0,
            Color::WHITE,
        );
        spawn_label(p, "FUEL", c, c + 0.22 * DIAL_PX, 12.0, dim);
        spawn_label(p, "LB x100", c, c + 0.32 * DIAL_PX, 10.0, dim);

        spawn_image(p, needle_l, FuelNeedle::L);
        spawn_image(p, needle_r, FuelNeedle::R);
    });
}

/// Point both needles at the live tank contents. One query over both needles,
/// each carrying its own tank marker.
pub(super) fn update_needles(
    state: Res<AircraftState>,
    mut q: Query<(&FuelNeedle, &mut UiTransform)>,
) {
    // The needle texture points up, so the rotation is the scale angle
    // measured clockwise from the top.
    for (needle, mut t) in &mut q {
        let lb = match needle {
            FuelNeedle::L => state.fuel_left_lb,
            FuelNeedle::R => state.fuel_right_lb,
        };
        t.rotation = Rot2::degrees(lb_to_cw(lb as f32));
    }
}
