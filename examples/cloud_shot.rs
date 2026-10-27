//! Renders the sky plugin alone from four useful altitudes/directions, so the
//! cloud look can be compared against the previous screenshots in `.tmp/`.
//!
//! Run: `cargo run --features shots --example cloud_shot` (set `RAIN=1` for the
//! storm deck).

use bevy::asset::RenderAssetUsages;
#[cfg(not(target_arch = "wasm32"))]
use bevy::camera::Hdr;
use bevy::core_pipeline::tonemapping::Tonemapping;
use bevy::image::{Image, ImageAddressMode, ImageFilterMode, ImageSampler, ImageSamplerDescriptor};
use bevy::light::{FogVolume, VolumetricFog, VolumetricLight};
use bevy::prelude::*;
use bevy::render::render_resource::{Extent3d, TextureDimension, TextureFormat};
use bevy::render::view::window::screenshot::{save_to_disk, Screenshot};
use bevy::winit::WinitSettings;
#[cfg(not(target_arch = "wasm32"))]
use my_flight_sim::cloud_volume::{CloudDomain, VolumetricCloudPlugin};
use my_flight_sim::{cloud_camera, sky, weather};

/// `CLOUD_MODE=fogvolume` swaps the cards for a Bevy `FogVolume` raymarched with
/// a baked density texture. This is the Phase-0 spike from the AC7 research
/// note: it answers "does a raymarched volume read better than cards here, and
/// what does it cost" without touching the main render path.
fn fog_mode() -> bool {
    // Bevy's `VolumetricFog` needs WebGPU on wasm; keep the spike desktop-only.
    cfg!(not(target_arch = "wasm32"))
        && std::env::var("CLOUD_MODE").is_ok_and(|v| v == "fogvolume")
}

/// `CLOUD_MODE=volume` runs the AC7-style custom pass instead of the Bevy fog
/// spike: procedural density, a sun march for self-shadowing, sky ambient.
// Both call sites are desktop-gated with the pass itself, so the helper goes
// with them rather than lingering as dead code on the web build.
#[cfg(not(target_arch = "wasm32"))]
fn volume_mode() -> bool {
    // The volumetric pass is desktop-only: web gets the card clouds.
    cfg!(not(target_arch = "wasm32"))
        && std::env::var("CLOUD_MODE").is_ok_and(|v| v == "volume")
}

fn envf(key: &str, default: f32) -> f32 {
    std::env::var(key)
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(default)
}

/// Voxels per axis of the baked density volume. 96³ is 884 k voxels, which
/// generates in well under a second in a release-ish build and still gives
/// ~270 m features across the 26 km test box.
const CLOUD_N: u32 = 96;

#[derive(Resource, Default)]
struct Frame(u32);

#[derive(Resource)]
struct SunDir(Vec3);

/// Poses to render, one per `HOLD` frames. `POSES` overrides the defaults with
/// `x,y,z,dx,dy,dz;...`, so a density sweep needs one process, not ten.
#[derive(Resource)]
struct Poses(Option<Vec<(Vec3, Vec3)>>);

/// Frames per pose. The first shot must land after the WESL pipeline has
/// compiled, otherwise it captures Bevy's fallback material.
const HOLD: u32 = 40;

fn parse_poses() -> Option<Vec<(Vec3, Vec3)>> {
    let spec = std::env::var("POSES").ok()?;
    Some(
        spec.split(';')
            .filter_map(|pose| {
                let v: Vec<f32> = pose
                    .split(',')
                    .map(|n| n.trim().parse().ok())
                    .collect::<Option<_>>()?;
                (v.len() == 6).then(|| (Vec3::new(v[0], v[1], v[2]), Vec3::new(v[3], v[4], v[5])))
            })
            .collect(),
    )
}

fn main() {
    let rain = if std::env::var("RAIN").is_ok_and(|v| v == "1") {
        0.9
    } else {
        0.0
    };
    // NIGHT=1 dusk, NIGHT=2 deep night (indexes `NIGHT_FACTORS`); 0 keeps the
    // day sky so the two presets compare against the same harness.
    let night = std::env::var("NIGHT")
        .ok()
        .and_then(|v| v.parse::<usize>().ok())
        .unwrap_or(0)
        .min(2);
    let poses = parse_poses();

    let mut app = App::new();
    app.add_plugins(DefaultPlugins.set(WindowPlugin {
        primary_window: Some(Window {
            title: "cloud_shot".into(),
            resolution: (1000, 640).into(),
            ..default()
        }),
        ..default()
    }))
    .insert_resource(WinitSettings::continuous())
    .insert_resource(weather::Weather {
        rain,
        time_level: night,
        ..Default::default()
    })
    .add_plugins(sky::SkyPlugin)
    .insert_resource(Poses(poses))
    .add_systems(Startup, setup)
    .init_resource::<Frame>()
    .add_systems(Update, (pose, shot, drift));

    #[cfg(not(target_arch = "wasm32"))]
    if volume_mode() {
        app.add_plugins(VolumetricCloudPlugin);
    }

    app.run();
}

fn setup(mut commands: Commands, mut images: ResMut<Assets<Image>>) {
    // The archipelago's afternoon sun, so the shots match the mission.
    let rot = Quat::from_euler(EulerRot::XYZ, -0.62, -0.65, 0.0);
    let sun = (rot * Vec3::Z).normalize();
    commands.insert_resource(SunDir(sun));

    let camera = commands
        .spawn((
            Camera3d::default(),
            // The sim's flight camera uses 80 km (see `view.rs`); the default
            // 1 km would clip both the 60 km sky dome and every test pose.
            Projection::Perspective(PerspectiveProjection {
                far: 80_000.0,
                ..default()
            }),
            Transform::from_xyz(0.0, 700.0, 0.0),
            Tonemapping::None,
        ))
        .id();
    let sun_light = commands
        .spawn((
            DirectionalLight::default(),
            Transform::from_rotation(rot),
        ))
        .id();

    #[cfg(not(target_arch = "wasm32"))]
    if volume_mode() {
        commands.entity(camera).insert((
            Hdr,
            CloudDomain {
                base: envf("BASE", 1_200.0),
                top: envf("TOP", 2_800.0),
                coverage_bias: envf("BIAS", -0.25),
                extinction: envf("EXTINCT", 0.04),
                wind: envf("WIND", 12.0),
                half_extent: Vec2::splat(envf("HALF", 10_000.0)),
                ..default()
            },
        ));
        return;
    }

    if !fog_mode() {
        return;
    }

    // The volumetric pass is a post-process that ray-marches a `FogVolume`;
    // 64 steps at this window size is the shipping-ish quality point.
    commands.entity(camera).insert(VolumetricFog {
        step_count: 64,
        jitter: 1.0,
        ambient_color: Color::srgb(0.55, 0.70, 0.90),
        ambient_intensity: 0.35,
    });

    // The pass samples the sun's shadow map for self-shadowing, so the light
    // must actually cast shadows; the sky-only harness disables them by
    // default (`shadow_maps_enabled: false`).
    commands.entity(sun_light).insert((
        DirectionalLight {
            shadow_maps_enabled: true,
            illuminance: 12_000.0,
            ..default()
        },
        VolumetricLight,
    ));

    let coverage = envf("COVERAGE", 0.45);
    let density_scale = envf("DENSITY", 0.06);
    let absorption = envf("ABSORB", 0.35);
    let scattering = envf("SCATTER", 0.35);
    let base = envf("BASE", 1_200.0);
    let top = envf("TOP", 2_800.0);
    let width = envf("WIDTH", 16_000.0);
    info!(
        "fog volume: base={base} top={top} width={width} density={density_scale} \
         absorb={absorption} scatter={scattering} coverage={coverage}"
    );

    let density = images.add(cloud_image(CLOUD_N, coverage));
    commands.spawn((
        FogVolume {
            fog_color: Color::srgb(0.96, 0.98, 1.0),
            density_factor: density_scale,
            density_texture: Some(density),
            density_texture_offset: Vec3::ZERO,
            absorption,
            scattering,
            scattering_asymmetry: 0.65,
            light_tint: Color::WHITE,
            ..default()
        },
        Transform::from_xyz(0.0, (base + top) * 0.5, 0.0)
            .with_scale(Vec3::new(width, top - base, width)),
    ));
}

/// Drift the density texture through the fog volume so the deck moves with the
/// wind. `density_texture_offset` wraps (the image uses `Repeat`), so a fract
/// keeps the numbers small and the motion seamless.
fn drift(time: Res<Time>, mut volumes: Query<&mut FogVolume>) {
    for mut fog in &mut volumes {
        fog.density_texture_offset.x =
            (fog.density_texture_offset.x + time.delta_secs() * 0.0025).fract();
    }
}

/// Bakes a tiling-free cumulus density volume: a height profile (flat base,
/// rounded top), Perlin-Worley shape, Worley erosion, and a border fade so the
/// cube never reads as a cut plane.
///
/// This is deliberately throwaway: the AC7-style density model in the plan is
/// evaluated per sample in a custom pass, not baked.
fn cloud_image(n: u32, coverage: f32) -> Image {
    let mut data = Vec::with_capacity((n * n * n) as usize);
    for z in 0..n {
        for y in 0..n {
            for x in 0..n {
                let u = (x as f32 + 0.5) / n as f32;
                let v = (y as f32 + 0.5) / n as f32;
                let w = (z as f32 + 0.5) / n as f32;
                let d = density_at(u, v, w, coverage);
                data.push((d * 255.0 + 0.5) as u8);
            }
        }
    }

    if std::env::var("DUMP").is_ok() {
        let mut min = 255u8;
        let mut max = 0u8;
        let mut sum = 0u64;
        let mut solid = 0usize;
        let mut bins = [0usize; 8];
        for &b in &data {
            min = min.min(b);
            max = max.max(b);
            sum += b as u64;
            if b > 25 {
                solid += 1;
            }
            bins[(b as usize * 8 / 256).min(7)] += 1;
        }
        let total = data.len();
        println!(
            "density: min={min} max={max} mean={:.1} nonempty={:.1}% bins={bins:?}",
            sum as f32 / total as f32,
            solid as f32 * 100.0 / total as f32,
        );
        let z = n / 2;
        let step = (n / 24).max(1) as usize;
        let wide = (n / 60).max(1) as usize;
        for y in (0..n).step_by(step) {
            let mut line = String::with_capacity((n as usize / wide) + 1);
            for x in (0..n).step_by(wide) {
                line.push(match data[((z * n + y) * n + x) as usize] {
                    0..=8 => ' ',
                    9..=40 => '.',
                    41..=100 => '+',
                    101..=180 => '*',
                    _ => '#',
                });
            }
            println!("{line}");
        }
    }

    let mut image = Image::new(
        Extent3d {
            width: n,
            height: n,
            depth_or_array_layers: n,
        },
        TextureDimension::D3,
        data,
        TextureFormat::R8Unorm,
        RenderAssetUsages::default(),
    );
    image.sampler = ImageSampler::Descriptor(ImageSamplerDescriptor {
        address_mode_u: ImageAddressMode::Repeat,
        address_mode_v: ImageAddressMode::Repeat,
        address_mode_w: ImageAddressMode::Repeat,
        mag_filter: ImageFilterMode::Linear,
        min_filter: ImageFilterMode::Linear,
        mipmap_filter: ImageFilterMode::Nearest,
        ..default()
    });
    image
}

fn density_at(u: f32, v: f32, w: f32, coverage: f32) -> f32 {
    // Flat base, rounded top: the silhouette that reads as cumulus from below
    // and as a deck from above.
    let height = smoothstep(0.02, 0.16, v) * (1.0 - smoothstep(0.55, 1.0, v));
    if height <= 0.0 {
        return 0.0;
    }

    // Five weather cells across the volume, each broken up by the Perlin-Worley
    // shape; a second octave adds the cauliflower clumps.
    let p = Vec3::new(u, v, w) * 5.0;
    let shape = perlin_worley(p) * 0.72 + perlin_worley(p * 2.17 + Vec3::splat(19.0)) * 0.28;
    let detail = worley(p * 11.0);

    let thresh = (1.0 - coverage).max(0.05);
    let mut d = ((shape * height - thresh) / thresh).clamp(0.0, 1.0);
    // Erode the edges, more where the cloud is thin, so the puffs stay crisp
    // instead of ballooning into one blob.
    d *= 1.0 - 0.55 * detail * (1.0 - d);

    // Fade the volume's outer walls: the box is 26 km wide, so a 2 km skirt is
    // invisible from the test poses but hides the boundary.
    let edge = smoothstep(0.0, 0.08, u)
        * smoothstep(0.0, 0.08, 1.0 - u)
        * smoothstep(0.0, 0.08, w)
        * smoothstep(0.0, 0.08, 1.0 - w);
    (d * edge).clamp(0.0, 1.0)
}

fn smoothstep(a: f32, b: f32, x: f32) -> f32 {
    let t = ((x - a) / (b - a)).clamp(0.0, 1.0);
    t * t * (3.0 - 2.0 * t)
}

/// 12-direction gradient noise with quintic interpolation: enough for cloud
/// lumps, and small enough to keep this spike self-contained.
fn perlin(p: Vec3) -> f32 {
    const GRAD: [[f32; 3]; 12] = [
        [1.0, 1.0, 0.0],
        [-1.0, 1.0, 0.0],
        [1.0, -1.0, 0.0],
        [-1.0, -1.0, 0.0],
        [1.0, 0.0, 1.0],
        [-1.0, 0.0, 1.0],
        [1.0, 0.0, -1.0],
        [-1.0, 0.0, -1.0],
        [0.0, 1.0, 1.0],
        [0.0, -1.0, 1.0],
        [0.0, 1.0, -1.0],
        [0.0, -1.0, -1.0],
    ];

    let i = p.floor();
    let f = p - i;
    let (ix, iy, iz) = (i.x as i32, i.y as i32, i.z as i32);
    let u = Vec3::new(fade(f.x), fade(f.y), fade(f.z));

    let mut n = 0.0;
    for dz in 0..2 {
        for dy in 0..2 {
            for dx in 0..2 {
                let g = GRAD[(hash3(ix + dx, iy + dy, iz + dz) % 12) as usize];
                let d = Vec3::new(f.x - dx as f32, f.y - dy as f32, f.z - dz as f32);
                let wx = if dx == 1 { u.x } else { 1.0 - u.x };
                let wy = if dy == 1 { u.y } else { 1.0 - u.y };
                let wz = if dz == 1 { u.z } else { 1.0 - u.z };
                n += wx * wy * wz * (g[0] * d.x + g[1] * d.y + g[2] * d.z);
            }
        }
    }
    n
}

/// Worley F1 in [0, 1]: cell distance, the classic cloud-erosion pattern.
fn worley(p: Vec3) -> f32 {
    let i = p.floor();
    let (ix, iy, iz) = (i.x as i32, i.y as i32, i.z as i32);
    let mut best = f32::MAX;
    for dz in -1..=1 {
        for dy in -1..=1 {
            for dx in -1..=1 {
                let h = hash3(ix + dx, iy + dy, iz + dz);
                let f = Vec3::new(
                    (ix + dx) as f32 + unit(h),
                    (iy + dy) as f32 + unit(h.rotate_left(11)),
                    (iz + dz) as f32 + unit(h.rotate_left(22)),
                );
                best = best.min(p.distance_squared(f));
            }
        }
    }
    best.sqrt().min(1.0)
}

/// Perlin modulated by Worley: the standard "billowy" cloud base density.
fn perlin_worley(p: Vec3) -> f32 {
    let d = 1.0 - worley(p);
    if d <= 1e-4 {
        return 1.0;
    }
    ((perlin(p) * 0.5 + 0.5) / d).clamp(0.0, 1.0)
}

fn fade(t: f32) -> f32 {
    t * t * t * (t * (t * 6.0 - 15.0) + 10.0)
}

fn unit(h: u32) -> f32 {
    (h >> 8) as f32 * (1.0 / 16_777_216.0)
}

fn hash00(mut h: u32) -> u32 {
    h ^= h >> 16;
    h = h.wrapping_mul(0x7feb_352d);
    h ^= h >> 15;
    h = h.wrapping_mul(0x846c_a68b);
    h ^= h >> 16;
    h
}

fn hash3(x: i32, y: i32, z: i32) -> u32 {
    hash00(
        (x as u32).wrapping_mul(0x8da6_b343)
            ^ (y as u32).wrapping_mul(0xd816_3841)
            ^ (z as u32).wrapping_mul(0xcb1a_b31f),
    )
}

fn pose(
    frame: Res<Frame>,
    poses: Res<Poses>,
    sun: Res<SunDir>,
    mut cam: Query<&mut Transform, (With<Camera3d>, Without<cloud_camera::CloudCamera>)>,
) {
    let Ok(mut t) = cam.single_mut() else {
        return;
    };
    let (pos, dir) = match &poses.0 {
        Some(list) => {
            let Some(&(pos, dir)) = list.get(frame.0 as usize / HOLD as usize) else {
                return;
            };
            (pos, dir)
        }
        None => match frame.0 {
            0..=39 => (Vec3::new(0.0, 700.0, 0.0), Vec3::new(0.0, 0.55, -0.83)),
            40..=79 => (Vec3::new(0.0, 2_400.0, 0.0), Vec3::new(0.0, 0.0, -1.0)),
            80..=119 => (Vec3::new(0.0, 8_000.0, 0.0), Vec3::new(0.0, -0.45, -0.89)),
            _ => (Vec3::new(0.0, 1_200.0, 0.0), sun.0),
        },
    };
    *t = Transform::from_translation(pos).looking_to(dir, Vec3::Y);
}

fn shot(
    mut commands: Commands,
    mut frame: ResMut<Frame>,
    poses: Res<Poses>,
    mut exit: MessageWriter<AppExit>,
) {
    frame.0 += 1;
    let (path, done) = match &poses.0 {
        Some(list) => {
            let idx = frame.0 as usize / HOLD as usize;
            let path = (frame.0 % HOLD == 38 && idx < list.len()).then(|| {
                let out = std::env::var("OUT").unwrap_or_else(|_| "/tmp/".into());
                format!("{out}cloud_pose_{idx}.png")
            });
            (path, frame.0 == list.len() as u32 * HOLD + 10)
        }
        None => {
            let path = match frame.0 {
                40 => Some("/tmp/cloud_low.png".into()),
                80 => Some("/tmp/cloud_inside.png".into()),
                120 => Some("/tmp/cloud_above.png".into()),
                160 => Some("/tmp/cloud_sun.png".into()),
                _ => None,
            };
            (path, frame.0 == 170)
        }
    };
    if let Some(path) = path {
        commands
            .spawn(Screenshot::primary_window())
            .observe(save_to_disk(path.clone()));
        info!("screenshot -> {path}");
    }
    if done {
        exit.write(AppExit::Success);
    }
}
