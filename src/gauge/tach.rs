//! The Cessna-172 engine tachometer.
//!
//! A circular RPM scale, `0` low on the left and `30` (×100 rpm) around on the
//! right. The green arc is a *stepped* normal-operating range — its width steps
//! at 2,450 and 2,575 rpm to mark the 75%-power setting at sea level, 5,000 ft
//! and 10,000 ft — and the red line is at 2,700. A little hour-meter drum sits
//! at the bottom.

use bevy::prelude::*;

use super::*;
use crate::sim::AircraftState;

/// Clockwise degrees from the top where `0` rpm sits, and the scale's gain.
const ZERO_CW: f32 = 225.0;
const PER_DEG: f32 = 0.08; // 100 rpm per 8°

fn rpm_to_cw(rpm: f32) -> f32 {
    ZERO_CW + rpm * PER_DEG
}

/// Red line (max continuous rpm).
const RPM_RED: f32 = 2700.0;
/// Bottom of the normal-operating (green) arc.
const GREEN_LO: f32 = 2100.0;
/// Upper bound of each step of the normal-operating (green) arc, and the inner
/// radius of that segment — the band narrows as rpm rises.
const GREEN_STEPS: [(f32, f32); 3] = [(2450.0, 0.815), (2575.0, 0.850), (2700.0, 0.882)];

/// Engine hours on the (decorative) drum. The sim tracks no such property.
const HOURS: &str = "2345.0";

#[derive(Component)]
pub(super) struct TachNeedle;

// --- Face rasterisation ---------------------------------------------------

const ARC_GREEN: [u8; 4] = [26, 158, 68, 255];
const RED: [u8; 4] = [214, 40, 40, 255];

/// Per-angle lookups, built once: tick class, green-band index, red flag.
struct Tables {
    tick: Vec<u8>,  // 0 none, 1 minor, 2 major
    green: Vec<u8>, // 0 none, 1..3 = band
    red: Vec<u8>,   // 0/1
}

fn build_tables() -> Tables {
    let mut t = Tables {
        tick: vec![0; BUCKETS],
        green: vec![0; BUCKETS],
        red: vec![0; BUCKETS],
    };
    for b in 0..BUCKETS {
        let a = b as f32 * 360.0 / BUCKETS as f32; // screen angle
        let cw = (a - 270.0).rem_euclid(360.0); // clockwise from the top
        let rpm = (cw - ZERO_CW).rem_euclid(360.0) / PER_DEG;
        if rpm > 3010.0 {
            continue;
        }
        let band = GREEN_STEPS
            .iter()
            .position(|&(hi, _)| rpm <= hi)
            .unwrap_or(3);
        if (GREEN_LO..=RPM_RED).contains(&rpm) {
            t.green[b] = (band + 1) as u8;
        }
        if (rpm - RPM_RED).abs() <= 8.0 {
            t.red[b] = 1;
        }
    }
    // Ticks every 100 rpm, longer and heavier every 500.
    for k in 0..=30i32 {
        let (class, half_w) = if k % 5 == 0 { (2u8, 0.6) } else { (1u8, 0.35) };
        let center = rpm_to_cw(k as f32 * 100.0);
        let mut d = -half_w;
        while d <= half_w {
            let b = ang_bucket(center + d);
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
    if let Some(c) = bezel_grade(u, 0.90) {
        return c;
    }

    let ang = y.atan2(x).to_degrees();
    let b = ang_bucket(ang);

    let mut c = FACE;
    // Ticks, just inside the arc band.
    let inner = match t.tick[b] {
        2 => 0.68,
        1 => 0.745,
        _ => 1.1,
    };
    if u >= inner && u < 0.815 {
        c = TICK;
    }
    // Stepped normal-operating arc.
    if t.green[b] > 0 && (0.815..0.905).contains(&u) {
        let step_inner = GREEN_STEPS[(t.green[b] - 1) as usize].1;
        if u >= step_inner {
            c = ARC_GREEN;
        }
    }
    // Red line, drawn last so it rides over the arc.
    if t.red[b] == 1 && (0.66..0.905).contains(&u) {
        c = RED;
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
            len: 0.80,
            base_w: 0.014,
            tip_w: 0.005,
            tail: 0.10,
            tail_w: 0.006,
            hub: 0.030,
        },
        [232, 232, 238, 255],
        [40, 40, 46, 255],
    )));

    let c = NODE * 0.5;
    let root = spawn_root(&mut commands, PanelKey::Tach, Side::Right, 2, 0);
    commands.entity(root).with_children(|p| {
        spawn_image(p, face, ());

        // Numerals 0..30 in steps of 5 (×100 rpm).
        for k in 0..=6 {
            let (x, y) = at(DIAL_PX * 0.66, rpm_to_cw(k as f32 * 500.0));
            spawn_label(p, &format!("{}", k * 5), c + x, c + y, 18.0, Color::WHITE);
        }

        // Centre captions.
        let dim = Color::srgb(0.85, 0.85, 0.85);
        spawn_label(p, "RPM", c - DIAL_PX * 0.10, c - DIAL_PX * 0.10, 13.0, dim);
        spawn_label(p, "X100", c - DIAL_PX * 0.10, c + DIAL_PX * 0.02, 11.0, dim);

        // Hour-meter drum at the bottom.
        let (hx, hy) = at(DIAL_PX * 0.46, 180.0);
        let (w, h) = (DIAL_PX * 0.62, DIAL_PX * 0.20);
        p.spawn((
            Node {
                position_type: PositionType::Absolute,
                left: Val::Px(c + hx - w * 0.5),
                top: Val::Px(c + hy - h * 0.5),
                width: Val::Px(w),
                height: Val::Px(h),
                justify_content: JustifyContent::Center,
                align_items: AlignItems::Center,
                border: UiRect::all(Val::Px(1.5)),
                ..default()
            },
            BackgroundColor(Color::srgb(0.06, 0.06, 0.07)),
            BorderColor::all(Color::srgb(0.72, 0.72, 0.74)),
        ))
        .with_children(|drum| {
            drum.spawn((
                Text::new(HOURS),
                TextFont {
                    font_size: FontSize::Px(13.0),
                    ..default()
                },
                TextColor(Color::srgb(0.93, 0.93, 0.93)),
            ));
        });
        spawn_label(p, "HOURS", c + hx, c + hy + h * 0.85, 9.0, dim);

        spawn_image(p, needle, TachNeedle);
    });
}

/// Point the needle at the live engine rpm. The needle texture points up, so
/// the rotation is the scale angle measured clockwise from the top.
pub(super) fn update_needle(
    state: Res<AircraftState>,
    mut q: Query<&mut UiTransform, With<TachNeedle>>,
) {
    let Ok(mut t) = q.single_mut() else {
        return;
    };
    t.rotation = Rot2::degrees(rpm_to_cw(state.rpm as f32));
}
