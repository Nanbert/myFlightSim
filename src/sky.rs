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
use bevy::camera::visibility::{RenderLayers, VisibilitySystems};
use bevy::camera::{ImageRenderTarget, RenderTarget};
use bevy::core_pipeline::tonemapping::Tonemapping;
use bevy::image::{ImageAddressMode, ImageFilterMode, ImageSampler, ImageSamplerDescriptor};
use bevy::mesh::Indices;
use bevy::prelude::*;
use bevy::render::render_resource::{
    AsBindGroup, Extent3d, ShaderType, TextureDimension, TextureFormat,
};
use bevy::render::view::Msaa;
use bevy::shader::ShaderRef;
use bevy::transform::TransformSystems;

use crate::cloud_camera::CloudCamera;
use crate::weather::Weather;

/// Radius of the dome. Must stay inside the camera's far plane (see `view.rs`);
/// large enough that the 96 km mission terrain and its haze sit inside it.
const DOME_RADIUS: f32 = 60_000.0;
/// Resolution of the tileable 3D noise texture.
const NOISE_SIZE: u32 = 64;
/// Cloud layer height above the ground plane, in metres. Five kilometres
/// thick so the towers reach the 8-12 km band the aircraft fight in: a thin
/// deck sits so far below that it collapses into a strip near the horizon.
const CLOUD_BASE: f32 = 1_500.0;
const CLOUD_TOP: f32 = 6_500.0;
/// Horizontal drift of the cloud field, metres per second.
const WIND: f32 = 6.0;
/// The raymarch renders into an offscreen target at this fraction of the
/// window resolution. Halving each axis quarters the fragment cost; the
/// temporal filter below recovers the missing detail.
const CLOUD_SCALE: f32 = 0.5;
/// Weight of the current frame when reprojected history agrees. Low enough to
/// average several jitters, high enough that a camera in motion does not
/// smear for long.
const TEMPORAL_WEIGHT: f32 = 0.15;

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
    /// x = mode (0 raymarch, 1 composite), y = temporal weight of the current
    /// frame, z = frame index, w = history valid.
    misc: Vec4,
    /// xy = full-resolution target size in physical pixels, zw unused.
    screen: Vec4,
    /// Previous frame's clip-from-world, for reprojecting the cloud target.
    prev_clip_from_world: Mat4,
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
    /// Half-resolution cloud buffer: rgb = premultiplied in-scattered
    /// radiance, a = transmittance. The march material binds the previous
    /// frame (history); the composite material binds the frame being drawn.
    #[texture(3)]
    #[sampler(4)]
    #[dependency]
    cloud_rt: Handle<Image>,
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
struct SkyMaterialHandles {
    march: Handle<CloudMaterial>,
    composite: Handle<CloudMaterial>,
}

/// Ping-pong pair of half-resolution cloud targets. `current` is rendered
/// into this frame and sampled by the composite; the other one holds the
/// previous frame and is read by the march for temporal accumulation.
#[derive(Resource)]
struct CloudTargets {
    images: [Handle<Image>; 2],
    current: usize,
    size: UVec2,
}

/// State the shader needs that changes once per frame.
#[derive(Resource, Default)]
struct CloudFrame {
    prev_clip_from_world: Mat4,
    valid: bool,
    index: u32,
}

/// The cloud field shared with the terrain and sea shaders so they can project
/// matching shadows onto the ground.
#[derive(Resource, Clone)]
pub struct CloudField {
    pub noise: Handle<Image>,
    pub base: f32,
    pub top: f32,
    pub wind: f32,
}

/// Coverage for a rain level. The dome and the ground shadows must agree.
pub fn coverage_for(rain: f32) -> f32 {
    0.35 + 0.55 * rain
}

pub struct SkyPlugin;

impl Plugin for SkyPlugin {
    fn build(&self, app: &mut App) {
        app.add_plugins(MaterialPlugin::<CloudMaterial>::default())
            .init_resource::<CloudFrame>()
            .add_systems(Startup, setup_sky)
            .add_systems(Update, (resize_cloud_targets, swap_cloud_targets).chain())
            .add_systems(
                PostUpdate,
                (sync_cloud_camera, follow_camera)
                    .after(TransformSystems::Propagate)
                    .before(VisibilitySystems::UpdateFrusta),
            )
            .add_systems(
                PostUpdate,
                update_cloud_params.after(TransformSystems::Propagate),
            );
    }
}

fn setup_sky(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut images: ResMut<Assets<Image>>,
    mut materials: ResMut<Assets<CloudMaterial>>,
) {
    let noise = images.add(noise_image());
    commands.insert_resource(CloudField {
        noise: noise.clone(),
        base: CLOUD_BASE,
        top: CLOUD_TOP,
        wind: WIND,
    });

    // Placeholder size; `resize_cloud_targets` matches the window on the first
    // update, once the main camera knows its physical target size.
    let targets = [
        images.add(cloud_image(640, 360)),
        images.add(cloud_image(640, 360)),
    ];
    let march = materials.add(CloudMaterial {
        params: CloudParams::default(),
        noise: noise.clone(),
        cloud_rt: targets[1].clone(),
    });
    let composite = materials.add(CloudMaterial {
        params: CloudParams::default(),
        noise,
        cloud_rt: targets[0].clone(),
    });
    commands.insert_resource(SkyMaterialHandles {
        march: march.clone(),
        composite: composite.clone(),
    });
    commands.insert_resource(CloudTargets {
        images: targets.clone(),
        current: 0,
        size: UVec2::new(640, 360),
    });

    let dome = meshes.add(inside_out_sphere(DOME_RADIUS));
    // The offscreen camera only sees this copy, on its own render layer.
    commands.spawn((
        SkyDome,
        RenderLayers::layer(1),
        Mesh3d(dome.clone()),
        MeshMaterial3d(march),
    ));
    commands.spawn((SkyDome, Mesh3d(dome), MeshMaterial3d(composite)));

    commands.spawn((
        CloudCamera,
        Camera3d::default(),
        Camera {
            // Before the main camera, so its target is ready to sample.
            order: -1,
            clear_color: ClearColorConfig::Custom(Color::NONE),
            ..default()
        },
        RenderTarget::Image(ImageRenderTarget::from(targets[0].clone())),
        Projection::Perspective(PerspectiveProjection {
            far: 80_000.0,
            ..default()
        }),
        // The march's own shoulder keeps the radiance in range; a tonemapper
        // here would flatten it a second time.
        Tonemapping::None,
        Msaa::Off,
        RenderLayers::layer(1),
        Transform::default(),
    ));
}

/// A half-resolution HDR target the cloud camera draws into. Transmittance
/// rides in alpha, so the buffer composites over the full-resolution sky.
fn cloud_image(width: u32, height: u32) -> Image {
    let mut image = Image::new_target_texture(width, height, TextureFormat::Rgba16Float, None);
    image.sampler = ImageSampler::Descriptor(ImageSamplerDescriptor {
        address_mode_u: ImageAddressMode::ClampToEdge,
        address_mode_v: ImageAddressMode::ClampToEdge,
        address_mode_w: ImageAddressMode::ClampToEdge,
        mag_filter: ImageFilterMode::Linear,
        min_filter: ImageFilterMode::Linear,
        mipmap_filter: ImageFilterMode::Nearest,
        ..default()
    });
    image
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

/// Keep both domes centred on the camera. Rotation is irrelevant: the shader
/// uses the camera→fragment direction, not the sphere's UVs. This runs after
/// transform propagation and writes `GlobalTransform` too, so the extracted
/// dome position matches the camera position pushed to the shader in the same
/// frame; otherwise fast motion skews every ray by one frame.
fn follow_camera(
    camera: Query<&GlobalTransform, (With<Camera3d>, Without<CloudCamera>, Without<SkyDome>)>,
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

/// Give the offscreen camera exactly the main camera's pose and lens.
fn sync_cloud_camera(
    main: Query<
        (&GlobalTransform, &Projection),
        (With<Camera3d>, Without<CloudCamera>),
    >,
    mut cloud: Query<
        (&mut Transform, &mut GlobalTransform, &mut Projection),
        With<CloudCamera>,
    >,
) {
    let (Ok((cam, projection)), Ok((mut t, mut g, mut p))) =
        (main.single(), cloud.single_mut())
    else {
        return;
    };
    *t = Transform::from(*cam);
    *g = *cam;
    *p = projection.clone();
}

/// Match the cloud targets to half the window's physical size.
fn resize_cloud_targets(
    mut targets: ResMut<CloudTargets>,
    mut frame: ResMut<CloudFrame>,
    mut images: ResMut<Assets<Image>>,
    camera: Query<&Camera, (With<Camera3d>, Without<CloudCamera>)>,
) {
    let Ok(cam) = camera.single() else {
        return;
    };
    let Some(size) = cam.physical_target_size() else {
        return;
    };
    let size = UVec2::new(
        (size.x as f32 * CLOUD_SCALE).max(1.0) as u32,
        (size.y as f32 * CLOUD_SCALE).max(1.0) as u32,
    );
    if size == targets.size {
        return;
    }
    for handle in &targets.images {
        if let Some(mut image) = images.get_mut(handle) {
            image.resize(Extent3d {
                width: size.x,
                height: size.y,
                depth_or_array_layers: 1,
            });
        }
    }
    targets.size = size;
    // The old contents are gone; start the accumulation over.
    frame.valid = false;
}

/// Alternate which target the offscreen camera draws into. The march samples
/// the other one (last frame's result), the composite samples this frame's.
fn swap_cloud_targets(
    mut targets: ResMut<CloudTargets>,
    handles: Res<SkyMaterialHandles>,
    mut materials: ResMut<Assets<CloudMaterial>>,
    mut camera: Query<&mut RenderTarget, With<CloudCamera>>,
) {
    targets.current ^= 1;
    let current = targets.images[targets.current].clone();
    let history = targets.images[1 - targets.current].clone();

    if let Some(mut m) = materials.get_mut(&handles.march) {
        m.cloud_rt = history;
    }
    if let Some(mut m) = materials.get_mut(&handles.composite) {
        m.cloud_rt = current.clone();
    }
    if let Ok(mut target) = camera.single_mut() {
        *target = RenderTarget::Image(ImageRenderTarget::from(current));
    }
}

/// Push the camera, sun, weather and reprojection state into both materials.
fn update_cloud_params(
    time: Res<Time>,
    weather: Res<Weather>,
    mut frame: ResMut<CloudFrame>,
    camera: Query<
        (&GlobalTransform, &Projection, &Camera),
        (With<Camera3d>, Without<CloudCamera>),
    >,
    sun: Query<&GlobalTransform, With<DirectionalLight>>,
    handles: Res<SkyMaterialHandles>,
    mut materials: ResMut<Assets<CloudMaterial>>,
) {
    let (Ok((cam, projection, camera)), Ok(sun)) = (camera.single(), sun.single()) else {
        return;
    };
    // The directional light shines along -Z; the sun itself lies along +Z.
    let sun_dir = (sun.rotation() * Vec3::Z).normalize_or_zero();
    let pos = cam.translation();
    let coverage = coverage_for(weather.rain);
    let clip_from_world =
        projection.get_clip_from_view() * cam.to_matrix().inverse();
    let screen = camera
        .physical_target_size()
        .map(|s| Vec2::new(s.x as f32, s.y as f32))
        .unwrap_or(Vec2::ONE);

    let prev_clip_from_world = frame.prev_clip_from_world;
    let history_valid = frame.valid;
    let index = frame.index;
    frame.prev_clip_from_world = clip_from_world;
    frame.valid = true;
    frame.index = index.wrapping_add(1);

    let params = CloudParams {
        camera: Vec4::new(pos.x, pos.y, pos.z, time.elapsed_secs()),
        sun: Vec4::new(sun_dir.x, sun_dir.y, sun_dir.z, weather.rain),
        layer: Vec4::new(CLOUD_BASE, CLOUD_TOP, coverage, WIND),
        misc: Vec4::new(
            0.0,
            TEMPORAL_WEIGHT,
            (index % 64) as f32,
            if history_valid { 1.0 } else { 0.0 },
        ),
        screen: Vec4::new(screen.x, screen.y, 0.0, 0.0),
        // The shader projects a world point into *last* frame's screen.
        prev_clip_from_world,
    };

    if let Some(mut m) = materials.get_mut(&handles.march) {
        m.params = params;
    }
    if let Some(mut m) = materials.get_mut(&handles.composite) {
        let mut p = params;
        p.misc.x = 1.0;
        m.params = p;
    }
}

// --- tileable 3D fBm -------------------------------------------------------

/// Build a tileable 3D fBm volume. The sampler wraps on all three axes, so the
/// field can be sampled at any world scale without seams. A box-filtered mip
/// chain lets the raymarch pick a footprint-appropriate level instead of
/// aliasing the high octaves at grazing distances.
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

    // wgpu uploads a 3D texture's levels back to back: the whole mip 0 volume,
    // then mip 1, and so on. `Image::new` asserts the data covers mip 0 only,
    // so build the image uninitialised and attach the full chain.
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
        let levels = (NOISE_SIZE as usize).ilog2() + 1;
        let expected: usize = (0..levels)
            .map(|l| {
                let d = (NOISE_SIZE as usize) >> l;
                d * d * d
            })
            .sum();
        assert_eq!(data.len(), expected);
        assert_eq!(image.texture_descriptor.mip_level_count, levels as u32);
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
