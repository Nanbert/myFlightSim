//! The Cessna-172 airspeed indicator.
//!
//! The dial *face* (bezel, tick marks, the white/green/yellow arcs and the red
//! VNE line) is rasterised once into a texture at startup; the numerals and
//! captions are UI text; the needle is a second texture rotated every frame
//! from the live calibrated airspeed. All three share one scale — the cubic
//! [`kt_to_angle`] below, fitted to a reference C172 ASI photo — so the arcs,
//! the numerals and the needle agree.

use bevy::prelude::*;

use super::*;
use crate::sim::AircraftState;

/// Airspeeds (kt) that place the colour-coded arcs and the red line — the
/// aircraft's real operating limits, read off a C172 ASI.
const V_FLAP_LO: f32 = 40.0; // white arc bottom — VSO, full-flap stall
const V_FE: f32 = 85.0; // white arc top — VFE, max flaps-extended
const V_S1: f32 = 48.0; // green arc bottom — VS, clean stall
const V_NO: f32 = 129.0; // green arc top — VNO, max structural cruise
const V_NE: f32 = 163.0; // red line — VNE, never exceed
const V_MAX: f32 = 200.0;

/// The dial's scale: airspeed (kt) → screen angle in degrees, measured
/// clockwise from screen-right. Cubic fitted to a reference C172 ASI.
fn kt_to_angle(kt: f32) -> f32 {
    268.499 + 1.376184 * kt + 5.932e-3 * kt * kt - 2.6e-5 * kt * kt * kt
}

#[derive(Component)]
pub(super) struct AirspeedNeedle;

// --- Face rasterisation ---------------------------------------------------

// Colours (sRGB). A black-faced ASI: dark dial, white markings.
const FACE: [u8; 4] = [16, 16, 18, 255];
const BEZEL: [u8; 4] = [14, 14, 16, 255];
const BEZEL_HI: [u8; 4] = [46, 48, 54, 255];
const BEZEL_RIM: [u8; 4] = [104, 108, 116, 255];
const TICK: [u8; 4] = [228, 228, 228, 255];
const ARC_WHITE: [u8; 4] = [236, 236, 236, 255];
const ARC_GREEN: [u8; 4] = [26, 158, 68, 255];
const ARC_YELLOW: [u8; 4] = [240, 205, 40, 255];
const RED: [u8; 4] = [214, 40, 40, 255];

/// Per-angle lookups built once from `kt_to_angle`: which arc colour a spoke
/// falls in, and whether it carries a (minor/major) tick. Everything the face
/// draws is purely a function of angle, so a per-degree table keeps rasterising
/// to one array index per sample.
struct Tables {
    outer: Vec<u8>, // 0 none, 1 white  (flap range)
    inner: Vec<u8>, // 0 none, 1 green, 2 yellow
    tick: Vec<u8>,  // 0 none, 1 minor, 2 major
}

const BUCKETS: usize = 1440; // 4 buckets per degree

fn build_tables() -> Tables {
    let mut t = Tables {
        outer: vec![0; BUCKETS],
        inner: vec![0; BUCKETS],
        tick: vec![0; BUCKETS],
    };
    for b in 0..BUCKETS {
        let a = b as f32 * 360.0 / BUCKETS as f32;
        if cw_between(a, kt_to_angle(V_FLAP_LO), kt_to_angle(V_FE)) {
            t.outer[b] = 1;
        }
        if cw_between(a, kt_to_angle(V_S1), kt_to_angle(V_NO)) {
            t.inner[b] = 1;
        } else if cw_between(a, kt_to_angle(V_NO), kt_to_angle(V_NE)) {
            t.inner[b] = 2;
        }
    }
    // A tick every 5 kt, longer and heavier every 10.
    let mut kt = 0.0f32;
    while kt <= V_MAX + 0.1 {
        let major = (kt as i32) % 10 == 0;
        let (class, half_w) = if major { (2u8, 0.65) } else { (1u8, 0.45) };
        let center = kt_to_angle(kt);
        let mut d = -half_w;
        while d <= half_w {
            let b = ((center + d).rem_euclid(360.0) / 360.0 * BUCKETS as f32) as usize % BUCKETS;
            if t.tick[b] < class {
                t.tick[b] = class;
            }
            d += 0.06;
        }
        kt += 5.0;
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
    // Coloured arcs: the white flap band sits just outside green/yellow.
    if (0.815..0.905).contains(&u) {
        if u >= 0.862 {
            if t.outer[b] == 1 {
                c = ARC_WHITE;
            }
        } else if t.inner[b] == 1 {
            c = ARC_GREEN;
        } else if t.inner[b] == 2 {
            c = ARC_YELLOW;
        }
    }
    // Tick marks.
    let inner = match t.tick[b] {
        2 => 0.68,
        1 => 0.745,
        _ => 1.1,
    };
    if u >= inner && u < 0.815 {
        c = TICK;
    }
    // The red VNE line, drawn last so it rides over ticks and arc.
    if (0.68..0.905).contains(&u) && cw_between(ang, kt_to_angle(V_NE) - 0.6, kt_to_angle(V_NE) + 0.6) {
        c = RED;
    }
    c
}

// --- Scene ------------------------------------------------------------------

/// Offset from the dial centre to the point at `(r, kt)`.
fn polar(r: f32, kt: f32) -> (f32, f32) {
    let a = kt_to_angle(kt).to_radians();
    (r * a.cos(), r * a.sin())
}

pub(super) fn setup(mut commands: Commands, mut images: ResMut<Assets<Image>>) {
    let tables = build_tables();
    let face = images.add(make_image(raster_face(|x, y| face_sample(&tables, x, y))));
    let needle = images.add(make_image(needle_pixels(
        &NeedleShape {
            len: 0.80,
            base_w: 5.5 / DIAL_R,
            tip_w: 1.5 / DIAL_R,
            tail: 0.16,
            tail_w: 3.0 / DIAL_R,
            hub: 0.045,
        },
        [232, 232, 238, 255],
        [40, 40, 46, 255],
    )));
    let c = NODE * 0.5;

    let root = spawn_root(&mut commands, Side::Left, 0, 0);
    commands.entity(root).with_children(|p| {
        spawn_image(p, face, ());

        // Numerals every 20 kt on the main scale.
        let mut kt = 40.0f32;
        while kt <= V_MAX {
            let (x, y) = polar(DIAL_PX * 0.62, kt);
            spawn_label(p, &format!("{kt:.0}"), c + x, c + y, 18.0, Color::WHITE);
            kt += 20.0;
        }

        // Centre captions.
        spawn_label(p, "AIRSPEED", c, c - DIAL_PX * 0.11, 12.0, Color::srgb(0.85, 0.85, 0.85));
        spawn_label(p, "KNOTS", c, c + DIAL_PX * 0.11, 12.0, Color::srgb(0.85, 0.85, 0.85));

        // V-speed limit labels, just outside the bezel (yellow, like the
        // real placards).
        let yellow = Color::srgb(0.95, 0.82, 0.15);
        for (name, kt) in [
            ("VSO", V_FLAP_LO),
            ("VS", V_S1),
            ("VFE", V_FE),
            ("VNO", V_NO),
            ("VNE", V_NE),
        ] {
            let (x, y) = polar(DIAL_PX * 1.06, kt);
            spawn_label(p, name, c + x, c + y, 12.0, yellow);
        }

        // Needle on top of the face and numerals.
        spawn_image(p, needle, AirspeedNeedle);
    });
}

/// Point the needle at the live calibrated airspeed. The needle texture points
/// up (270°), so the rotation is the scale angle measured from there.
pub(super) fn update_needle(
    state: Res<AircraftState>,
    mut q: Query<&mut UiTransform, With<AirspeedNeedle>>,
) {
    let Ok(mut t) = q.single_mut() else {
        return;
    };
    let kt = state.cas_kts as f32;
    t.rotation = Rot2::degrees(kt_to_angle(kt.clamp(0.0, 210.0)) - 270.0);
}
