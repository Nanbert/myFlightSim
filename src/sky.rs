//! Procedural sky: a large unlit sphere centred on the camera, textured with a
//! baked gradient (zenith → horizon → ground haze) and a layer of fBm-noise
//! clouds. Generated, not modelled — there are no sky art assets.
//!
//! The dome only *translates* with the camera, so the horizon stays level no
//! matter the aircraft's attitude. It also slowly *spins* about world Y: since
//! the gradient is azimuthally uniform, that rotation is invisible except for
//! the clouds, which drift around the sky for free (no per-cloud entities).

use bevy::asset::RenderAssetUsages;
use bevy::prelude::*;
use bevy::render::render_resource::{Extent3d, TextureDimension, TextureFormat};
use std::f32::consts::FRAC_PI_2;

use crate::weather::{Weather, RAIN_LEVELS};

/// Radius of the dome. Must stay inside the camera's far plane (see `view.rs`).
const DOME_RADIUS: f32 = 8_000.0;
/// Resolution of the baked sky texture. The horizontal axis wraps all the way
/// around the dome, so it must tile seamlessly in x.
const SKY_COLS: u32 = 1024;
const SKY_ROWS: u32 = 512;
/// Cloud drift, radians per second.
const CLOUD_DRIFT: f32 = 0.004;

#[derive(Component)]
struct SkyDome;

/// One baked sky texture per [`RAIN_LEVELS`] intensity, so switching weather is
/// just a handle swap — no re-bake (and no hitch) at the moment of the `T` press.
#[derive(Resource)]
struct SkyTextures(Vec<Handle<Image>>);

pub struct SkyPlugin;

impl Plugin for SkyPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Startup, setup_sky)
            .add_systems(Update, (follow_camera, apply_weather_sky));
    }
}

fn setup_sky(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut images: ResMut<Assets<Image>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    // Bake the clear sky plus each overcast level up front.
    let skies: Vec<Handle<Image>> = RAIN_LEVELS
        .iter()
        .map(|&overcast| images.add(sky_image(overcast)))
        .collect();

    let material = materials.add(StandardMaterial {
        base_color_texture: Some(skies[0].clone()),
        unlit: true,
        // The camera sits *inside* the sphere, so we must render its inner
        // surface; disabling culling does that without reversing the winding.
        cull_mode: None,
        ..default()
    });
    commands.insert_resource(SkyTextures(skies));

    // The UV sphere's poles run along its local Z, so roll -90° about X to put
    // v = 0 (zenith) on world +Y. Base is overwritten each frame by the drift.
    commands.spawn((
        SkyDome,
        Mesh3d(meshes.add(Sphere::new(DOME_RADIUS).mesh().uv(48, 24))),
        MeshMaterial3d(material),
        Transform::from_rotation(base_rotation(0.0)),
    ));
}

/// Dome orientation for a given drift angle: spin about world Y, then the fixed
/// roll that puts the texture's zenith on +Y.
fn base_rotation(drift: f32) -> Quat {
    Quat::from_rotation_y(drift) * Quat::from_rotation_x(-FRAC_PI_2)
}

/// Keep the dome centred on the camera and slowly drift it. Rotation is
/// otherwise fixed — the sky must not bank with the aircraft.
fn follow_camera(
    time: Res<Time>,
    camera: Query<&Transform, (With<Camera3d>, Without<SkyDome>)>,
    mut dome: Query<&mut Transform, With<SkyDome>>,
) {
    let (Ok(cam), Ok(mut dome)) = (camera.single(), dome.single_mut()) else {
        return;
    };
    dome.translation = cam.translation;
    dome.rotation = base_rotation(time.elapsed_secs() * CLOUD_DRIFT);
}

/// Swap the dome to the baked texture matching the current rain intensity
/// (nearest level). Runs only when the level actually changes.
fn apply_weather_sky(
    weather: Res<Weather>,
    textures: Res<SkyTextures>,
    dome: Query<&MeshMaterial3d<StandardMaterial>, With<SkyDome>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut current: Local<usize>,
) {
    let Ok(mat) = dome.single() else {
        return;
    };
    let idx = RAIN_LEVELS
        .iter()
        .enumerate()
        .min_by(|(_, a), (_, b)| {
            (*a - weather.rain)
                .abs()
                .partial_cmp(&(*b - weather.rain).abs())
                .unwrap()
        })
        .map(|(i, _)| i)
        .unwrap_or(0);
    // Only touch the material when the level actually changes (the default `0`
    // already matches the clear texture set at startup).
    if *current == idx {
        return;
    }
    *current = idx;
    if let Some(mut m) = materials.get_mut(&mat.0) {
        m.base_color_texture = Some(textures.0[idx].clone());
    }
}

/// Build the sky texture: gradient (row 0 = zenith, last row = straight down)
/// with clouds blended into the upper band. Columns wrap around the dome.
/// `overcast` (0 = clear … 1 = storm) greys and darkens the whole thing.
fn sky_image(overcast: f32) -> Image {
    let mut data = Vec::with_capacity((SKY_COLS * SKY_ROWS * 4) as usize);
    for row in 0..SKY_ROWS {
        let v = row as f32 / (SKY_ROWS - 1) as f32;
        let base = gradient_rgb(v);
        for col in 0..SKY_COLS {
            let u = col as f32 / SKY_COLS as f32; // [0, 1) — wraps, so periodic
            let alpha = cloud_alpha(u, v, overcast);
            let rgb = [
                blend(base[0], CLOUD[0], alpha),
                blend(base[1], CLOUD[1], alpha),
                blend(base[2], CLOUD[2], alpha),
            ];
            let rgb = apply_overcast(rgb, overcast);
            data.extend_from_slice(&[
                rgb[0] as u8,
                rgb[1] as u8,
                rgb[2] as u8,
                255,
            ]);
        }
    }
    Image::new(
        Extent3d {
            width: SKY_COLS,
            height: SKY_ROWS,
            depth_or_array_layers: 1,
        },
        TextureDimension::D2,
        data,
        TextureFormat::Rgba8UnormSrgb,
        RenderAssetUsages::RENDER_WORLD,
    )
}

/// Cloud colour (near-white, a touch cool).
const CLOUD: [f32; 3] = [246.0, 249.0, 253.0];

fn blend(a: f32, b: f32, t: f32) -> f32 {
    a + (b - a) * t.clamp(0.0, 1.0)
}

/// Sky colour at height fraction `v` (0 = straight up, 1 = straight down),
/// interpolated over a few sRGB stops. No clouds — this is the clear-sky base.
fn gradient_rgb(v: f32) -> [f32; 3] {
    const STOPS: &[(f32, [f32; 3])] = &[
        (0.00, [22.0, 76.0, 186.0]),   // zenith: deep blue
        (0.42, [100.0, 158.0, 226.0]), // upper sky
        (0.50, [196.0, 218.0, 240.0]), // horizon haze: pale, brightest
        (0.55, [176.0, 186.0, 192.0]), // just below: neutral haze
        (0.75, [138.0, 146.0, 152.0]), // distance haze
        (1.00, [94.0, 100.0, 106.0]),  // nadir
    ];

    let v = v.clamp(0.0, 1.0);
    let mut rgb = STOPS[0].1;
    for pair in STOPS.windows(2) {
        let (v0, c0) = pair[0];
        let (v1, c1) = pair[1];
        if v <= v1 {
            let t = if v1 > v0 { (v - v0) / (v1 - v0) } else { 0.0 };
            rgb = [
                c0[0] + (c1[0] - c0[0]) * t,
                c0[1] + (c1[1] - c0[1]) * t,
                c0[2] + (c1[2] - c0[2]) * t,
            ];
            break;
        }
    }
    rgb
}

/// Clear-sky colour at `v` as sRGB bytes (used by tests and callers that want
/// the gradient without clouds).
#[allow(dead_code)]
fn gradient_color(v: f32) -> [u8; 4] {
    let c = gradient_rgb(v);
    [c[0] as u8, c[1] as u8, c[2] as u8, 255]
}

/// Cloud opacity at dome coordinates (`u` around, `v` up→down). `u` is
/// periodic, so the clouds tile seamlessly as the dome spins.
fn cloud_alpha(u: f32, v: f32, overcast: f32) -> f32 {
    // A narrow threshold band (not 0.42..0.72) gives the puffs defined edges
    // instead of a soft grey wash: most of the noise saturates to clear or
    // opaque, so cloud cores read as near-white. Overcast lowers the threshold,
    // swelling the cloud cover toward a solid deck.
    let t0 = 0.44 - 0.06 * overcast;
    let t1 = 0.56 - 0.02 * overcast;
    let coverage = smoothstep(t0, t1, fbm(u, v, 0xC10D_5EED));
    coverage * cloud_band(v)
}

/// Push a sky colour toward grey and darken it, for overcast weather. `k` is
/// the overcast amount (0 = untouched). A plain tint can only darken (which
/// reads as dusk, not cloud), so we explicitly desaturate toward luminance
/// first — that grey is what sells an overcast sky.
fn apply_overcast(c: [f32; 3], k: f32) -> [f32; 3] {
    let l = 0.299 * c[0] + 0.587 * c[1] + 0.114 * c[2];
    let grey = [l * 0.98, l, l * 1.04];
    let desat = 0.85 * k;
    let dark = 1.0 - 0.38 * k;
    [
        blend(c[0], grey[0], desat) * dark,
        blend(c[1], grey[1], desat) * dark,
        blend(c[2], grey[2], desat) * dark,
    ]
}

/// Where clouds are allowed to live: none at the zenith or below the horizon,
/// densest a bit above the horizon. The fade *to* the horizon is deliberately
/// narrow (`0.45..0.50`): the chase camera looks down at the aircraft, so in
/// level flight the horizon sits near the top of the frame and only a thin
/// near-horizon strip of sky (v ≈ 0.46–0.50) is ever visible — clouds must
/// still be dense there to show up at all.
fn cloud_band(v: f32) -> f32 {
    let from_zenith = smoothstep(0.05, 0.22, v);
    let to_horizon = 1.0 - smoothstep(0.45, 0.50, v);
    from_zenith * to_horizon
}

/// Fractal value noise in [0, 1]. Periodic in `u` (seamless wrap), not in `v`.
fn fbm(u: f32, v: f32, seed: u64) -> f32 {
    let mut sum = 0.0;
    let mut norm = 0.0;
    let mut amp = 0.5;
    let mut freq: f32 = 1.0;
    for octave in 0..5u64 {
        // Base frequency tuned for cumulus-scale puffs: a handful of blobs
        // around the dome, refined by five octaves of detail.
        let cells = (9.0 * freq).round() as i64;
        sum += amp * value_noise(u * cells as f32, v * cells as f32, cells, seed + octave);
        norm += amp;
        amp *= 0.5;
        freq *= 2.0;
    }
    sum / norm
}

/// Bilinear value noise on an integer lattice, wrapping `x` modulo `cells_x` so
/// the field is continuous across `u = 1 → 0`.
fn value_noise(x: f32, y: f32, cells_x: i64, seed: u64) -> f32 {
    let x0 = x.floor() as i64;
    let y0 = y.floor() as i64;
    let fx = smoothstep(0.0, 1.0, x - x0 as f32);
    let fy = smoothstep(0.0, 1.0, y - y0 as f32);
    let wrap = |i: i64| i.rem_euclid(cells_x.max(1));
    let a = hash2(wrap(x0), y0, seed);
    let b = hash2(wrap(x0 + 1), y0, seed);
    let c = hash2(wrap(x0), y0 + 1, seed);
    let d = hash2(wrap(x0 + 1), y0 + 1, seed);
    blend(blend(a, b, fx), blend(c, d, fx), fy)
}

/// Integer hash → [0, 1).
fn hash2(x: i64, y: i64, seed: u64) -> f32 {
    let mut h = seed
        .wrapping_add((x as u64).wrapping_mul(0x9E37_79B9_7F4A_7C15))
        .wrapping_add((y as u64).wrapping_mul(0xC2B2_AE3D_27D4_EB4F));
    h ^= h >> 29;
    h = h.wrapping_mul(0xBF58_476D_1CE4_E5B9);
    h ^= h >> 32;
    (h & 0xFF_FFFF) as f32 / 0xFF_FFFF as f32
}

fn smoothstep(e0: f32, e1: f32, x: f32) -> f32 {
    let t = ((x - e0) / (e1 - e0)).clamp(0.0, 1.0);
    t * t * (3.0 - 2.0 * t)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn luma(c: [u8; 4]) -> u32 {
        c[0] as u32 + c[1] as u32 + c[2] as u32
    }

    #[test]
    fn zenith_reads_as_blue_sky() {
        let z = gradient_color(0.0);
        assert!(
            z[2] as i32 - z[0] as i32 > 60,
            "zenith should be strongly blue, got {z:?}"
        );
    }

    #[test]
    fn horizon_is_the_brightest_band() {
        let h = gradient_color(0.5);
        assert!(luma(h) > luma(gradient_color(0.0)), "horizon vs zenith");
        assert!(luma(h) > luma(gradient_color(1.0)), "horizon vs nadir");
    }

    #[test]
    fn below_horizon_is_earth_toned() {
        // Neutral haze: no channel dominates like the blue sky.
        let g = gradient_color(0.62);
        assert!((g[2] as i32 - g[0] as i32).abs() < 40, "not blue-dominant: {g:?}");
    }

    #[test]
    fn sky_image_has_expected_size() {
        let img = sky_image(0.0);
        let data = img.data.as_ref().expect("image has data");
        assert_eq!(data.len(), (SKY_COLS * SKY_ROWS * 4) as usize);
        assert_eq!(img.texture_descriptor.size.width, SKY_COLS);
        assert_eq!(img.texture_descriptor.size.height, SKY_ROWS);
    }

    #[test]
    fn overcast_greys_and_darkens_the_sky() {
        // A clear blue zenith, run through the overcast transform.
        let blue = gradient_rgb(0.0);
        let storm = apply_overcast(blue, 1.0);
        let blue_dominance = |c: [f32; 3]| c[2] - c[0];
        assert!(
            blue_dominance(storm) < blue_dominance(blue) * 0.5,
            "storm sky should lose most of its blue: {blue:?} -> {storm:?}"
        );
        let luma = |c: [f32; 3]| 0.299 * c[0] + 0.587 * c[1] + 0.114 * c[2];
        assert!(luma(storm) < luma(blue), "storm sky should be darker");
    }

    #[test]
    fn overcast_zero_is_a_no_op() {
        let c = gradient_rgb(0.3);
        assert_eq!(apply_overcast(c, 0.0), c);
    }

    #[test]
    fn clouds_stay_out_of_the_zenith_and_below_the_horizon() {
        assert_eq!(cloud_band(0.0), 0.0, "no clouds at the zenith");
        assert_eq!(cloud_band(0.5), 0.0, "no clouds at the horizon");
        assert_eq!(cloud_band(1.0), 0.0, "no clouds below the horizon");
        assert!(cloud_band(0.3) > 0.5, "clouds should be dense mid-band");
    }

    #[test]
    fn fbm_tiles_seamlessly_in_u() {
        // The dome spins, so u = 0 and u = 1 must be the same point.
        for v in [0.1, 0.25, 0.33, 0.45] {
            let a = fbm(0.0, v, 0xC10D_5EED);
            let b = fbm(1.0, v, 0xC10D_5EED);
            assert!((a - b).abs() < 1e-4, "seam at v={v}: {a} vs {b}");
        }
    }

    #[test]
    fn fbm_output_is_in_unit_range() {
        for i in 0..200 {
            let u = (i as f32 * 0.0137).fract();
            let v = (i as f32 * 0.0311).fract();
            let n = fbm(u, v, 12345);
            assert!((0.0..=1.0).contains(&n), "fbm out of range: {n}");
        }
    }

    #[test]
    fn drift_keeps_the_zenith_pointing_up() {
        // Rotating about world Y must leave the zenith (dome +Z after the fixed
        // roll) sitting on world +Y.
        let zenith = base_rotation(1.234) * Vec3::Z;
        assert!(zenith.distance(Vec3::Y) < 1e-4, "zenith drifted: {zenith}");
    }
}
