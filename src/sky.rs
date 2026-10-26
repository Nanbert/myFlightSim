//! Procedural sky dome and the cloud-shadow field.
//!
//! Clouds themselves are world-space cards (`src/clouds.rs`), not a raymarched
//! sky layer. This module keeps the sky gradient/lighting and the small 3D
//! noise volume that the terrain and sea shaders still use for soft cloud
//! shadows.

use bevy::asset::RenderAssetUsages;
use bevy::camera::visibility::VisibilitySystems;
use bevy::image::{ImageAddressMode, ImageFilterMode, ImageSampler, ImageSamplerDescriptor};
use bevy::mesh::Indices;
use bevy::prelude::*;
use bevy::render::render_resource::{
    AsBindGroup, Extent3d, ShaderType, TextureDimension, TextureFormat,
};
use bevy::shader::ShaderRef;
use bevy::transform::TransformSystems;

use crate::weather::Weather;

/// Radius of the dome. Must stay inside the camera's far plane (see `view.rs`);
/// large enough that the 96 km mission terrain and its haze sit inside it.
const DOME_RADIUS: f32 = 60_000.0;
/// Resolution of the tileable 3D noise texture used by ground cloud shadows.
const NOISE_SIZE: u32 = 64;
/// Cloud-shadow layer height above the ground plane, in metres. The fixed
/// cloud cards use their own per-map heights; this pair keeps the existing
/// ground-shadow projection stable.
const CLOUD_BASE: f32 = 1_500.0;
const CLOUD_TOP: f32 = 6_500.0;
/// Fixed clouds do not advect; a zero wind keeps their ground shadows static.
const WIND: f32 = 0.0;

#[derive(Component)]
struct SkyDome;

/// Shader parameters. Field order and packing match `SkyParams` in
/// `assets/shaders/sky.wesl`.
#[derive(Clone, Copy, ShaderType, Default)]
struct SkyParams {
    /// xyz = camera position, w = time (s).
    camera: Vec4,
    /// xyz = direction *toward* the sun, w = rain intensity (0..1).
    sun: Vec4,
    /// x = night factor, yzw unused.
    misc: Vec4,
}

#[derive(Asset, TypePath, AsBindGroup, Clone)]
struct SkyMaterial {
    #[uniform(0)]
    params: SkyParams,
}

impl Material for SkyMaterial {
    fn fragment_shader() -> ShaderRef {
        "shaders/sky.wesl".into()
    }

    fn enable_prepass() -> bool {
        false
    }

    fn enable_shadows() -> bool {
        false
    }
}

/// The cloud-shadow field shared with the terrain and sea shaders so they can
/// project matching shadows onto the ground. This is no longer the visible
/// cloud layer.
#[derive(Resource, Clone)]
pub struct CloudField {
    pub noise: Handle<Image>,
    pub base: f32,
    pub top: f32,
    pub wind: f32,
}

/// Coverage for a rain level. The ground shadows and the old cloud presets
/// agree on this curve.
pub fn coverage_for(rain: f32) -> f32 {
    0.35 + 0.55 * rain
}

pub struct SkyPlugin;

impl Plugin for SkyPlugin {
    fn build(&self, app: &mut App) {
        app.add_plugins(MaterialPlugin::<SkyMaterial>::default())
            .add_systems(Startup, setup_sky)
            .add_systems(
                PostUpdate,
                follow_camera
                    .after(TransformSystems::Propagate)
                    .before(VisibilitySystems::UpdateFrusta),
            )
            .add_systems(
                PostUpdate,
                update_sky_params.after(TransformSystems::Propagate),
            );
    }
}

fn setup_sky(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut images: ResMut<Assets<Image>>,
    mut materials: ResMut<Assets<SkyMaterial>>,
) {
    let noise = images.add(noise_image());
    commands.insert_resource(CloudField {
        noise,
        base: CLOUD_BASE,
        top: CLOUD_TOP,
        wind: WIND,
    });

    let material = materials.add(SkyMaterial {
        params: SkyParams::default(),
    });
    commands.spawn((
        SkyDome,
        Mesh3d(meshes.add(inside_out_sphere(DOME_RADIUS))),
        MeshMaterial3d(material),
    ));
}

/// Keep the dome centred on the camera. Rotation is irrelevant: the shader
/// uses the camera→fragment direction, not the sphere's UVs.
fn follow_camera(
    camera: Query<&GlobalTransform, (With<Camera3d>, Without<SkyDome>)>,
    mut domes: Query<(&mut Transform, &mut GlobalTransform), With<SkyDome>>,
) {
    let Ok(cam) = camera.single() else {
        return;
    };
    let pos = cam.translation();
    for (mut t, mut g) in &mut domes {
        t.translation = pos;
        *g = GlobalTransform::from_translation(pos);
    }
}

fn update_sky_params(
    time: Res<Time>,
    weather: Res<Weather>,
    camera: Query<&GlobalTransform, (With<Camera3d>, Without<SkyDome>)>,
    sun: Query<&GlobalTransform, With<DirectionalLight>>,
    handles: Query<&MeshMaterial3d<SkyMaterial>, With<SkyDome>>,
    mut materials: ResMut<Assets<SkyMaterial>>,
) {
    let (Ok(cam), Ok(sun), Ok(material)) = (camera.single(), sun.single(), handles.single()) else {
        return;
    };
    let sun_dir = (sun.rotation() * Vec3::Z).normalize_or_zero();
    let pos = cam.translation();
    if let Some(mut m) = materials.get_mut(&material.0) {
        m.params.camera = Vec4::new(pos.x, pos.y, pos.z, time.elapsed_secs());
        m.params.sun = Vec4::new(sun_dir.x, sun_dir.y, sun_dir.z, weather.rain);
        m.params.misc = Vec4::new(weather.night_factor(), 0.0, 0.0, 0.0);
    }
}

/// The camera sits inside the dome, so its triangles must face inward.
fn inside_out_sphere(radius: f32) -> Mesh {
    let mut mesh = Sphere::new(radius).mesh().uv(48, 24);
    reverse_winding(&mut mesh);
    mesh
}

fn reverse_winding(mesh: &mut Mesh) {
    if let Some(indices) = mesh.indices_mut() {
        match indices {
            Indices::U16(v) => {
                for tri in v.as_chunks_mut::<3>().0 {
                    tri.swap(1, 2);
                }
            }
            Indices::U32(v) => {
                for tri in v.as_chunks_mut::<3>().0 {
                    tri.swap(1, 2);
                }
            }
        }
    }
}

// --- tileable 3D noise for ground cloud shadows ------------------------------

fn noise_image() -> Image {
    let n = NOISE_SIZE as usize;
    let mut levels: Vec<Vec<u8>> = Vec::new();
    let mut data = Vec::with_capacity(n * n * n);
    for z in 0..n {
        for y in 0..n {
            for x in 0..n {
                let v = noise_fbm(
                    x as f32 / n as f32,
                    y as f32 / n as f32,
                    z as f32 / n as f32,
                );
                data.push((v.clamp(0.0, 1.0) * 255.0).round() as u8);
            }
        }
    }
    levels.push(data);

    let mut dim = n;
    while dim > 1 {
        let prev = levels.last().expect("level 0 exists");
        let half = dim / 2;
        let mut next = Vec::with_capacity(half * half * half);
        for z in 0..half {
            for y in 0..half {
                for x in 0..half {
                    let mut sum = 0u32;
                    for dz in 0..2 {
                        for dy in 0..2 {
                            for dx in 0..2 {
                                let i = ((2 * z + dz) * dim + (2 * y + dy)) * dim + (2 * x + dx);
                                sum += prev[i] as u32;
                            }
                        }
                    }
                    next.push((sum / 8) as u8);
                }
            }
        }
        levels.push(next);
        dim = half;
    }
    let mip_count = levels.len() as u32;

    let mut image = Image::new_uninit(
        Extent3d {
            width: NOISE_SIZE,
            height: NOISE_SIZE,
            depth_or_array_layers: NOISE_SIZE,
        },
        TextureDimension::D3,
        TextureFormat::R8Unorm,
        RenderAssetUsages::RENDER_WORLD,
    );
    image.texture_descriptor.mip_level_count = mip_count;
    image.data = Some(levels.concat());
    image.sampler = ImageSampler::Descriptor(ImageSamplerDescriptor {
        address_mode_u: ImageAddressMode::Repeat,
        address_mode_v: ImageAddressMode::Repeat,
        address_mode_w: ImageAddressMode::Repeat,
        mag_filter: ImageFilterMode::Linear,
        min_filter: ImageFilterMode::Linear,
        mipmap_filter: ImageFilterMode::Linear,
        ..default()
    });
    image
}

fn noise_fbm(x: f32, y: f32, z: f32) -> f32 {
    let mut sum = 0.0;
    let mut norm = 0.0;
    let mut amp = 0.5;
    let mut cells = 4;
    for octave in 0..4u32 {
        sum += amp * value_noise(x, y, z, cells, 0xC10D_5EED + octave);
        norm += amp;
        amp *= 0.5;
        cells *= 2;
    }
    sum / norm
}

fn value_noise(x: f32, y: f32, z: f32, cells: i32, seed: u32) -> f32 {
    let xf = x * cells as f32;
    let yf = y * cells as f32;
    let zf = z * cells as f32;
    let xi = xf.floor() as i32;
    let yi = yf.floor() as i32;
    let zi = zf.floor() as i32;
    let fx = smooth(xf - xi as f32);
    let fy = smooth(yf - yi as f32);
    let fz = smooth(zf - zi as f32);
    let wrap = |i: i32| i.rem_euclid(cells.max(1));

    let c000 = hash3(wrap(xi), wrap(yi), wrap(zi), seed);
    let c100 = hash3(wrap(xi + 1), wrap(yi), wrap(zi), seed);
    let c010 = hash3(wrap(xi), wrap(yi + 1), wrap(zi), seed);
    let c110 = hash3(wrap(xi + 1), wrap(yi + 1), wrap(zi), seed);
    let c001 = hash3(wrap(xi), wrap(yi), wrap(zi + 1), seed);
    let c101 = hash3(wrap(xi + 1), wrap(yi), wrap(zi + 1), seed);
    let c011 = hash3(wrap(xi), wrap(yi + 1), wrap(zi + 1), seed);
    let c111 = hash3(wrap(xi + 1), wrap(yi + 1), wrap(zi + 1), seed);

    let c00 = c000 + (c100 - c000) * fx;
    let c10 = c010 + (c110 - c010) * fx;
    let c01 = c001 + (c101 - c001) * fx;
    let c11 = c011 + (c111 - c011) * fx;
    let c0 = c00 + (c10 - c00) * fy;
    let c1 = c01 + (c11 - c01) * fy;
    c0 + (c1 - c0) * fz
}

fn hash3(x: i32, y: i32, z: i32, seed: u32) -> f32 {
    let mut h = seed
        .wrapping_add((x as u32).wrapping_mul(0x9E37_79B1))
        .wrapping_add((y as u32).wrapping_mul(0x85EB_CA77))
        .wrapping_add((z as u32).wrapping_mul(0xC2B2_AE3D));
    h ^= h >> 16;
    h = h.wrapping_mul(0x7FEB_352D);
    h ^= h >> 15;
    h = h.wrapping_mul(0x846C_A68B);
    h ^= h >> 16;
    (h & 0x00FF_FFFF) as f32 / 0x0100_0000 as f32
}

fn smooth(t: f32) -> f32 {
    t * t * (3.0 - 2.0 * t)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn noise_texture_is_a_tileable_r8_volume() {
        let image = noise_image();
        let data = image.data.as_ref().expect("image has data");
        let levels = (NOISE_SIZE as usize).ilog2() + 1;
        let expected: usize = (0..levels)
            .map(|l| {
                let d = (NOISE_SIZE as usize) >> l;
                d * d * d
            })
            .sum();
        assert_eq!(data.len(), expected);
        assert_eq!(image.texture_descriptor.mip_level_count, levels);
        assert_eq!(image.texture_descriptor.size.width, NOISE_SIZE);
        assert_eq!(image.texture_descriptor.size.height, NOISE_SIZE);
        assert_eq!(
            image.texture_descriptor.size.depth_or_array_layers,
            NOISE_SIZE
        );
        assert_eq!(image.texture_descriptor.format, TextureFormat::R8Unorm);
    }

    #[test]
    fn value_noise_wraps_on_every_axis() {
        for axis in 0..3 {
            let mut a = [0.25, 0.5, 0.75];
            let mut b = a;
            a[axis] = 0.0;
            b[axis] = 1.0;
            let n0 = value_noise(a[0], a[1], a[2], 8, 123);
            let n1 = value_noise(b[0], b[1], b[2], 8, 123);
            assert!((n0 - n1).abs() < 1e-5, "seam on axis {axis}: {n0} vs {n1}");
        }
    }

    #[test]
    fn reversing_winding_flips_each_triangle() {
        let mut mesh = Sphere::new(1.0).mesh().uv(8, 4);
        let first = |m: &Mesh| -> Vec<u32> {
            m.indices()
                .expect("indexed")
                .iter()
                .take(3)
                .map(|i| i as u32)
                .collect()
        };
        let before = first(&mesh);
        reverse_winding(&mut mesh);
        let after = first(&mesh);
        assert_eq!(
            after,
            vec![before[0], before[2], before[1]],
            "triangle winding was not reversed"
        );
    }
}
