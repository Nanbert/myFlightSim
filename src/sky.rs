//! Procedural sky and volumetric clouds.
//!
//! The old sky was a baked gradient painted onto a sphere, with the clouds
//! stamped into the same texture. They had no depth, no lighting and no height,
//! so they read as wallpaper. This version keeps the dome only as a canvas: a
//! fragment shader reconstructs the view ray from the camera, marches it through
//! a world-space cloud layer and shades each sample against the sun.
//!
//! The cloud field is driven by a small tileable 3D fBm texture generated at
//! startup. A low-frequency coverage field decides where clouds exist at all, so
//! the layer breaks into scattered clumps instead of a uniform sea. The dome
//! still follows the camera, but the noise is sampled in world space, so the
//! clouds stay put in the world and gain real parallax as the aircraft moves.

use bevy::asset::RenderAssetUsages;
use bevy::image::{ImageAddressMode, ImageFilterMode, ImageSampler, ImageSamplerDescriptor};
use bevy::mesh::Indices;
use bevy::prelude::*;
use bevy::render::render_resource::{
    AsBindGroup, Extent3d, ShaderType, TextureDimension, TextureFormat,
};
use bevy::shader::ShaderRef;

use crate::weather::Weather;

/// Radius of the dome. Must stay inside the camera's far plane (see `view.rs`);
/// large enough that the 96 km mission terrain and its haze sit inside it.
const DOME_RADIUS: f32 = 60_000.0;
/// Resolution of the tileable 3D noise texture.
const NOISE_SIZE: u32 = 64;
/// Cloud layer height above the ground plane, in metres.
const CLOUD_BASE: f32 = 1_500.0;
const CLOUD_TOP: f32 = 3_400.0;
/// Horizontal drift of the cloud field, metres per second.
const WIND: f32 = 6.0;

#[derive(Component)]
struct SkyDome;

/// Shader parameters. Field order and packing match `CloudParams` in
/// `assets/shaders/cloud_sky.wesl`.
#[derive(Clone, Copy, ShaderType, Default)]
struct CloudParams {
    /// xyz = camera position, w = time (s).
    camera: Vec4,
    /// xyz = direction *toward* the sun, w = rain intensity (0..1).
    sun: Vec4,
    /// x = cloud base, y = cloud top, z = cloud coverage, w = wind speed.
    layer: Vec4,
}

#[derive(Asset, TypePath, AsBindGroup, Clone)]
struct CloudMaterial {
    #[uniform(0)]
    params: CloudParams,
    /// Tileable 3D fBm sampled by the raymarch.
    #[texture(1, dimension = "3d")]
    #[sampler(2)]
    #[dependency]
    noise: Handle<Image>,
}

impl Material for CloudMaterial {
    fn fragment_shader() -> ShaderRef {
        "shaders/cloud_sky.wesl".into()
    }

    // The dome is the background: opaque, no prepass, no shadow casting.
    fn enable_prepass() -> bool {
        false
    }

    fn enable_shadows() -> bool {
        false
    }
}

#[derive(Resource)]
struct SkyMaterialHandle(Handle<CloudMaterial>);

pub struct SkyPlugin;

impl Plugin for SkyPlugin {
    fn build(&self, app: &mut App) {
        app.add_plugins(MaterialPlugin::<CloudMaterial>::default())
            .add_systems(Startup, setup_sky)
            .add_systems(Update, (follow_camera, update_cloud_params));
    }
}

fn setup_sky(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut images: ResMut<Assets<Image>>,
    mut materials: ResMut<Assets<CloudMaterial>>,
) {
    let noise = images.add(noise_image());
    let material = materials.add(CloudMaterial {
        params: CloudParams::default(),
        noise,
    });
    commands.insert_resource(SkyMaterialHandle(material.clone()));

    commands.spawn((
        SkyDome,
        Mesh3d(meshes.add(inside_out_sphere(DOME_RADIUS))),
        MeshMaterial3d(material),
    ));
}

/// The camera sits inside the dome, so its triangles must face inward. The
/// generated sphere faces outward; swapping each triangle's last two vertices
/// reverses the winding without touching positions or UVs.
fn inside_out_sphere(radius: f32) -> Mesh {
    let mut mesh = Sphere::new(radius).mesh().uv(48, 24);
    reverse_winding(&mut mesh);
    mesh
}

fn reverse_winding(mesh: &mut Mesh) {
    if let Some(indices) = mesh.indices_mut() {
        match indices {
            Indices::U16(v) => {
                for tri in v.chunks_exact_mut(3) {
                    tri.swap(1, 2);
                }
            }
            Indices::U32(v) => {
                for tri in v.chunks_exact_mut(3) {
                    tri.swap(1, 2);
                }
            }
        }
    }
}

/// Keep the dome centred on the camera. Rotation is irrelevant: the shader uses
/// the camera→fragment direction, not the sphere's UVs.
fn follow_camera(
    camera: Query<&Transform, (With<Camera3d>, Without<SkyDome>)>,
    mut dome: Query<&mut Transform, With<SkyDome>>,
) {
    let (Ok(cam), Ok(mut dome)) = (camera.single(), dome.single_mut()) else {
        return;
    };
    dome.translation = cam.translation;
}

/// Push the camera, sun and weather into the cloud material every frame.
fn update_cloud_params(
    time: Res<Time>,
    weather: Res<Weather>,
    camera: Query<&GlobalTransform, (With<Camera3d>, Without<SkyDome>)>,
    sun: Query<&GlobalTransform, With<DirectionalLight>>,
    handle: Res<SkyMaterialHandle>,
    mut materials: ResMut<Assets<CloudMaterial>>,
) {
    let (Ok(cam), Ok(sun)) = (camera.single(), sun.single()) else {
        return;
    };
    // The directional light shines along -Z; the sun itself lies along +Z.
    let sun_dir = (sun.rotation() * Vec3::Z).normalize_or_zero();
    let pos = cam.translation();
    let coverage = 0.35 + 0.55 * weather.rain;

    if let Some(mut m) = materials.get_mut(&handle.0) {
        m.params = CloudParams {
            camera: Vec4::new(pos.x, pos.y, pos.z, time.elapsed_secs()),
            sun: Vec4::new(sun_dir.x, sun_dir.y, sun_dir.z, weather.rain),
            layer: Vec4::new(CLOUD_BASE, CLOUD_TOP, coverage, WIND),
        };
    }
}

// --- tileable 3D fBm -------------------------------------------------------

/// Build a tileable 3D fBm volume. The sampler wraps on all three axes, so the
/// field can be sampled at any world scale without seams.
fn noise_image() -> Image {
    let n = NOISE_SIZE;
    let mut data = Vec::with_capacity((n * n * n) as usize);
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

    let mut image = Image::new(
        Extent3d {
            width: n,
            height: n,
            depth_or_array_layers: n,
        },
        TextureDimension::D3,
        data,
        TextureFormat::R8Unorm,
        RenderAssetUsages::RENDER_WORLD,
    );
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

/// Four octaves of wrapped value noise, normalised to [0, 1].
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

/// Trilinear value noise on a wrapped integer lattice.
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
        assert_eq!(data.len(), (NOISE_SIZE * NOISE_SIZE * NOISE_SIZE) as usize);
        assert_eq!(image.texture_descriptor.size.width, NOISE_SIZE);
        assert_eq!(image.texture_descriptor.size.height, NOISE_SIZE);
        assert_eq!(
            image.texture_descriptor.size.depth_or_array_layers,
            NOISE_SIZE
        );
        assert_eq!(image.texture_descriptor.format, TextureFormat::R8Unorm);
        match &image.sampler {
            ImageSampler::Descriptor(d) => {
                assert_eq!(d.address_mode_u, ImageAddressMode::Repeat);
                assert_eq!(d.address_mode_v, ImageAddressMode::Repeat);
                assert_eq!(d.address_mode_w, ImageAddressMode::Repeat);
            }
            ImageSampler::Default => panic!("noise volume needs wrapping"),
        }
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
    fn fbm_stays_in_unit_range() {
        for i in 0..200 {
            let x = (i as f32 * 0.0137).fract();
            let y = (i as f32 * 0.0311).fract();
            let z = (i as f32 * 0.0479).fract();
            let n = noise_fbm(x, y, z);
            assert!((0.0..=1.0).contains(&n), "fbm out of range: {n}");
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
