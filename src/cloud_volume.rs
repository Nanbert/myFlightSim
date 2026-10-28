//! AC7-style volumetric clouds, built on what the CEDEC talk actually does:
//! a procedural 3D density field, ray-marched with a short sun march for
//! self-shadowing, plus a weather map that decides where clouds live.
//!
//! This is the Phase 1/2 path from the research note. It deliberately avoids
//! the built-in `VolumetricFog` pass: that one has no light-space density
//! march, so a deck renders as a flat grey sheet (see `/tmp/b_*.png`).
//!
//! Layout:
//! - [`CloudDomain`] on a camera marks it as a volumetric-cloud view and gives
//!   the world-space box the density lives in (it follows the camera in XZ).
//! - [`CloudTextures`] holds the baked shape volume and weather map.
//! - `render` (below) adds one `Core3d` pass between the main pass and the
//!   early post-process, compositing premultiplied in-scatter into the HDR
//!   target before bloom/tonemapping.
//!
//! `CloudProbe` is the CPU twin of the shader density, for gameplay queries
//! ("is the aircraft inside a cloud?") and tests.

use bevy::asset::RenderAssetUsages;
use bevy::core_pipeline::core_3d::prepare_core_3d_depth_textures;
use bevy::core_pipeline::schedule::{Core3d, Core3dSystems};
use bevy::image::{
    Image, ImageAddressMode, ImageFilterMode, ImageSampler, ImageSamplerDescriptor,
};
use bevy::prelude::*;
use bevy::camera::Camera3d;
use bevy::render::render_asset::RenderAssets;
use bevy::render::render_resource::{
    binding_types::{
        sampler, texture_2d, texture_3d, texture_depth_2d_multisampled, uniform_buffer,
    },
    BindGroupEntry, BindGroupLayoutDescriptor, BindGroupLayoutEntries, BindingResource,
    BlendComponent, BlendFactor, BlendOperation, BlendState, CachedRenderPipelineId,
    ColorTargetState, ColorWrites, DynamicUniformBuffer, Extent3d, FragmentState, LoadOp,
    MultisampleState, Operations, PipelineCache, PrimitiveState, RenderPassColorAttachment,
    RenderPassDescriptor, RenderPipelineDescriptor, ShaderStages, ShaderType, StoreOp,
    TextureDimension, TextureFormat, TextureSampleType, TextureUsages, VertexState,
};
use bevy::render::renderer::{RenderContext, RenderDevice, RenderQueue, ViewQuery};
use bevy::render::texture::GpuImage;
use bevy::render::view::{ExtractedView, Msaa, ViewDepthStencilTexture, ViewTarget};
use bevy::render::{
    Extract, ExtractSchedule, Render, RenderApp, RenderStartup, RenderSystems,
};
use bevy::shader::Shader;

/// Resolution of the baked Perlin-Worley volume. 64³ generates in well under a
/// second at startup; the shader adds a 4x "detail" octave from the same
/// texture, so this reads finer than the voxel size suggests.
const SHAPE_N: u32 = 128;
/// World size (m) the density field is authored over, before the domain box
/// stretches it vertically.
const DOMAIN_M: f32 = 20_000.0;
/// Weather-map tile size in metres.
const WEATHER_M: f32 = 24_000.0;

/// Marks a camera as a volumetric-cloud view and describes the cloud box.
#[derive(Component, Clone, Copy, Debug)]
pub struct CloudDomain {
    /// XZ follows the camera; `y` is ignored (the box spans `base..top`).
    pub center: Vec3,
    /// Half-width of the box in XZ.
    pub half_extent: Vec2,
    /// Cloud base and top, in metres above sea level.
    pub base: f32,
    pub top: f32,
    /// -1..1, added to the weather map's coverage. Driven by `Weather`.
    pub coverage_bias: f32,
    /// Extinction at full density, 1/m.
    pub extinction: f32,
    /// Wind along +X, m/s.
    pub wind: f32,
}

impl Default for CloudDomain {
    fn default() -> Self {
        Self {
            center: Vec3::new(0.0, 2_100.0, 0.0),
            half_extent: Vec2::splat(DOMAIN_M * 0.5),
            base: 1_300.0,
            top: 3_000.0,
            coverage_bias: -0.25,
            extinction: 0.04,
            wind: 10.0,
        }
    }
}

/// Baked cloud textures plus the shader, created once at startup.
#[derive(Resource, Clone)]
pub struct CloudTextures {
    pub shape: Handle<Image>,
    pub weather: Handle<Image>,
}

pub struct VolumetricCloudPlugin;

impl Plugin for VolumetricCloudPlugin {
    fn build(&self, app: &mut App) {
        let shader = app
            .world()
            .resource::<AssetServer>()
            .load("shaders/cloud_volume.wesl");

        app.add_systems(Startup, bake_textures);

        let Some(render_app) = app.get_sub_app_mut(RenderApp) else {
            return;
        };

        render_app
            .insert_resource(RenderShader(shader))
            .add_systems(ExtractSchedule, (extract_textures, extract_domain, extract_sun))
            .add_systems(RenderStartup, init_cloud_pass)
            .add_systems(
                Render,
                (
                    prepare_depth_usage
                        .in_set(RenderSystems::Prepare)
                        .before(prepare_core_3d_depth_textures),
                ),
            )
            .add_systems(
                Core3d,
                draw_clouds
                    .after(Core3dSystems::MainPass)
                    .before(Core3dSystems::EarlyPostProcess),
            );
    }
}

#[derive(Resource)]
struct RenderShader(Handle<Shader>);

fn bake_textures(mut commands: Commands, mut images: ResMut<Assets<Image>>) {
    let shape = images.add(shape_volume(SHAPE_N));
    let weather = images.add(weather_map(128));
    commands.insert_resource(CloudTextures { shape, weather });
}

// --- extraction -------------------------------------------------------------

#[derive(Resource, Clone)]
struct RenderTextures {
    shape: Handle<Image>,
    weather: Handle<Image>,
}

fn extract_textures(
    mut commands: Commands,
    textures: Extract<Option<Res<CloudTextures>>>,
) {
    let Some(textures) = textures.as_ref() else {
        return;
    };
    commands.insert_resource(RenderTextures {
        shape: textures.shape.clone(),
        weather: textures.weather.clone(),
    });
}

#[derive(Resource, Clone, Copy, Default)]
struct RenderDomain(Option<DomainSample>);

#[derive(Clone, Copy)]
struct DomainSample {
    min: Vec3,
    max: Vec3,
    coverage_bias: f32,
    extinction: f32,
    wind: f32,
}

fn extract_domain(
    mut commands: Commands,
    domains: Extract<Query<(&CloudDomain, &GlobalTransform)>>,
) {
    let sample = domains.iter().next().map(|(domain, transform)| {
        let mut center = transform.translation();
        center.y = (domain.base + domain.top) * 0.5;
        DomainSample {
            min: Vec3::new(
                center.x - domain.half_extent.x,
                domain.base,
                center.z - domain.half_extent.y,
            ),
            max: Vec3::new(
                center.x + domain.half_extent.x,
                domain.top,
                center.z + domain.half_extent.y,
            ),
            coverage_bias: domain.coverage_bias,
            extinction: domain.extinction,
            wind: domain.wind,
        }
    });
    commands.insert_resource(RenderDomain(sample));
}

#[derive(Resource, Clone, Copy)]
struct SunSample {
    dir: Vec3,
    color: Vec3,
    time: f32,
}

impl Default for SunSample {
    fn default() -> Self {
        Self {
            dir: Vec3::Y,
            color: Vec3::ONE,
            time: 0.0,
        }
    }
}

fn extract_sun(
    mut commands: Commands,
    sun: Extract<Query<(&DirectionalLight, &GlobalTransform)>>,
    time: Extract<Res<Time>>,
) {
    let (dir, color) = sun
        .iter()
        .next()
        .map(|(light, transform)| {
            let linear = light.color.to_linear();
            (
                (transform.rotation() * Vec3::Z).normalize_or_zero(),
                Vec3::new(linear.red, linear.green, linear.blue) * 1.6,
            )
        })
        .unwrap_or((Vec3::Y, Vec3::ONE));
    commands.insert_resource(SunSample {
        dir,
        color,
        time: time.elapsed_secs(),
    });
}

// --- render resources -------------------------------------------------------

#[derive(ShaderType, Clone, Copy)]
struct CloudUniform {
    inv_view_proj: Mat4,
    camera_pos: Vec4,
    sun_dir: Vec4,
    sun_color: Vec4,
    domain_min: Vec4,
    domain_max: Vec4,
    zenith: Vec4,
    horizon: Vec4,
    params: Vec4,
}

impl Default for CloudUniform {
    fn default() -> Self {
        Self {
            inv_view_proj: Mat4::IDENTITY,
            camera_pos: Vec4::ZERO,
            sun_dir: Vec4::Y,
            sun_color: Vec4::ONE,
            domain_min: Vec4::ZERO,
            domain_max: Vec4::ONE,
            zenith: Vec4::new(0.44, 0.53, 0.70, 0.62),
            horizon: Vec4::new(0.66, 0.76, 0.90, 0.0),
            params: Vec4::new(64.0, 0.0, 10.0, 1.0),
        }
    }
}

#[derive(Resource, Default)]
struct CloudUniforms(DynamicUniformBuffer<CloudUniform>);

#[derive(Resource)]
struct CloudPass {
    pipeline: CachedRenderPipelineId,
    params_layout: BindGroupLayoutDescriptor,
    volume_layout: BindGroupLayoutDescriptor,
}

fn init_cloud_pass(
    mut commands: Commands,
    shader: Option<Res<RenderShader>>,
    pipeline_cache: Res<PipelineCache>,
) {
    let Some(shader) = shader else {
        return;
    };

    let params_entries = BindGroupLayoutEntries::single(
        ShaderStages::FRAGMENT,
        uniform_buffer::<CloudUniform>(false),
    );
    let params_layout =
        BindGroupLayoutDescriptor::new("cloud params layout", &params_entries);

    let volume_entries = BindGroupLayoutEntries::with_indices(
        ShaderStages::FRAGMENT,
        (
            (0, texture_depth_2d_multisampled()),
            (
                1,
                texture_3d(TextureSampleType::Float { filterable: true }),
            ),
            (2, sampler(bevy::render::render_resource::SamplerBindingType::Filtering)),
            (
                3,
                texture_2d(TextureSampleType::Float { filterable: true }),
            ),
            (4, sampler(bevy::render::render_resource::SamplerBindingType::Filtering)),
        ),
    );
    let volume_layout =
        BindGroupLayoutDescriptor::new("cloud volume layout", &volume_entries[..]);

    let pipeline = pipeline_cache.queue_render_pipeline(RenderPipelineDescriptor {
        label: Some("volumetric clouds".into()),
        layout: vec![params_layout.clone(), volume_layout.clone()],
        vertex: VertexState {
            shader: shader.0.clone(),
            entry_point: Some("vertex".into()),
            buffers: vec![],
            ..default()
        },
        primitive: PrimitiveState::default(),
        depth_stencil: None,
        multisample: MultisampleState::default(),
        fragment: Some(FragmentState {
            shader: shader.0.clone(),
            entry_point: Some("fragment".into()),
            targets: vec![Some(ColorTargetState {
                // The app renders HDR (`view.rs` adds `Hdr`); the pass refuses
                // to run on an LDR target rather than guessing the format.
                format: TextureFormat::Rgba16Float,
                blend: Some(BlendState {
                    color: BlendComponent {
                        src_factor: BlendFactor::One,
                        dst_factor: BlendFactor::OneMinusSrcAlpha,
                        operation: BlendOperation::Add,
                    },
                    alpha: BlendComponent {
                        src_factor: BlendFactor::One,
                        dst_factor: BlendFactor::OneMinusSrcAlpha,
                        operation: BlendOperation::Add,
                    },
                }),
                write_mask: ColorWrites::ALL,
            })],
            ..default()
        }),
        ..default()
    });

    commands.insert_resource(CloudPass {
        pipeline,
        params_layout,
        volume_layout,
    });
    commands.init_resource::<CloudUniforms>();
}

// --- prepare / draw ---------------------------------------------------------

fn prepare_depth_usage(mut cameras: Query<&mut Camera3d>) {
    for mut camera in &mut cameras {
        camera.depth_texture_usages.0 |= TextureUsages::TEXTURE_BINDING.bits();
    }
}

fn draw_clouds(
    pass: Option<Res<CloudPass>>,
    mut uniforms: ResMut<CloudUniforms>,
    domain: Res<RenderDomain>,
    sun: Res<SunSample>,
    device: Res<RenderDevice>,
    queue: Res<RenderQueue>,
    textures: Option<Res<RenderTextures>>,
    images: Res<RenderAssets<GpuImage>>,
    pipeline_cache: Res<PipelineCache>,
    view: ViewQuery<(
        &ExtractedView,
        &ViewTarget,
        &ViewDepthStencilTexture,
        &Msaa,
    )>,
    mut ctx: RenderContext,
) {
    let (view, view_target, depth, msaa) = view.into_inner();
    let Some(domain) = domain.0 else {
        return;
    };

    // The uniform is written here rather than in `Prepare`: `ViewQuery` is
    // per-view and only valid inside the camera's `Core3d` run.
    // `world_from_view` is the camera transform; the projection needs the
    // inverse (view from world), not the camera transform itself.
    let clip_from_world = view
        .clip_from_world
        .unwrap_or_else(|| view.clip_from_view * Mat4::from(view.world_from_view.affine().inverse()));
    let camera = view.world_from_view.translation();
    uniforms.0.clear();
    uniforms.0.push(&CloudUniform {
        inv_view_proj: clip_from_world.inverse(),
        camera_pos: Vec4::new(camera.x, camera.y, camera.z, sun.time),
        sun_dir: Vec4::new(sun.dir.x, sun.dir.y, sun.dir.z, 0.0),
        sun_color: Vec4::new(sun.color.x, sun.color.y, sun.color.z, domain.extinction),
        domain_min: Vec4::new(
            domain.min.x,
            domain.min.y,
            domain.min.z,
            domain.coverage_bias,
        ),
        domain_max: Vec4::new(domain.max.x, domain.max.y, domain.max.z, 0.0),
        zenith: Vec4::new(0.44, 0.53, 0.70, 0.62),
        horizon: Vec4::new(0.66, 0.76, 0.90, 0.0),
        params: Vec4::new(96.0, (sun.time * 60.0).fract(), domain.wind, 1.0),
    });
    uniforms.0.write_buffer(&device, &queue);

    let (pass, textures) = match (pass.as_ref(), textures.as_ref()) {
        (Some(pass), Some(textures)) => (pass, textures),
        _ => return,
    };
    let Some(pipeline) = pipeline_cache.get_render_pipeline(pass.pipeline) else {
        return;
    };
    if matches!(*msaa, Msaa::Off) {
        return;
    }
    if view_target.main_texture_format() != TextureFormat::Rgba16Float {
        return;
    }
    let (Some(shape), Some(weather)) = (
        images.get(&textures.shape),
        images.get(&textures.weather),
    ) else {
        return;
    };
    let Some(uniform_binding) = uniforms.0.binding() else {
        return;
    };
    let Some(depth_view) = depth.attachment.depth_stencil_views().depth_only_view() else {
        return;
    };

    let params_layout = pipeline_cache.get_bind_group_layout(&pass.params_layout);
    let volume_layout = pipeline_cache.get_bind_group_layout(&pass.volume_layout);

    let (params, volume) = {
        let device = ctx.render_device();
        let params = device.create_bind_group(
            Some("cloud params"),
            &params_layout,
            &[BindGroupEntry {
                binding: 0,
                resource: uniform_binding,
            }],
        );
        let volume = device.create_bind_group(
            Some("cloud volume"),
            &volume_layout,
            &[
                BindGroupEntry {
                    binding: 0,
                    resource: BindingResource::TextureView(depth_view),
                },
                BindGroupEntry {
                    binding: 1,
                    resource: BindingResource::TextureView(&shape.texture_view),
                },
                BindGroupEntry {
                    binding: 2,
                    resource: BindingResource::Sampler(&shape.sampler),
                },
                BindGroupEntry {
                    binding: 3,
                    resource: BindingResource::TextureView(&weather.texture_view),
                },
                BindGroupEntry {
                    binding: 4,
                    resource: BindingResource::Sampler(&weather.sampler),
                },
            ],
        );
        (params, volume)
    };

    let descriptor = RenderPassDescriptor {
        label: Some("volumetric clouds pass"),
        color_attachments: &[Some(RenderPassColorAttachment {
            view: view_target.main_texture_view(),
            depth_slice: None,
            resolve_target: None,
            ops: Operations {
                load: LoadOp::Load,
                store: StoreOp::Store,
            },
        })],
        depth_stencil_attachment: None,
        timestamp_writes: None,
        occlusion_query_set: None,
        multiview_mask: None,
    };

    let mut render_pass = ctx.command_encoder().begin_render_pass(&descriptor);
    render_pass.set_pipeline(pipeline);
    render_pass.set_bind_group(0, &params, &[]);
    render_pass.set_bind_group(1, &volume, &[]);
    render_pass.draw(0..3, 0..1);
}

// --- CPU density ------------------------------------------------------------

/// CPU mirror of the shader's density, for gameplay and tests.
#[derive(Clone, Copy)]
pub struct CloudProbe {
    pub domain: CloudDomain,
    pub seed: u32,
}

impl CloudProbe {
    pub fn new(domain: CloudDomain, seed: u32) -> Self {
        Self { domain, seed }
    }

    /// Density in [0, 1] (before the shader's per-metre scaling). Cheap and
    /// smooth: it is the same height profile and shape threshold, minus the
    /// shader's high-frequency erosion.
    pub fn density_at(&self, p: Vec3) -> f32 {
        let d = &self.domain;
        let uvw = Vec3::new(
            (p.x - (d.center.x - d.half_extent.x)) / (d.half_extent.x * 2.0),
            (p.y - d.base) / (d.top - d.base),
            (p.z - (d.center.z - d.half_extent.y)) / (d.half_extent.y * 2.0),
        );
        if uvw.min_element() < 0.0 || uvw.max_element() > 1.0 {
            return 0.0;
        }
        let profile = smoothstep(0.0, 0.10, uvw.y) * (1.0 - smoothstep(0.52, 1.0, uvw.y));
        let weather = value_noise(
            Vec3::new(p.x / WEATHER_M, 0.5, p.z / WEATHER_M),
            self.seed,
        );
        let coverage = (weather * 1.7 - 0.35 + d.coverage_bias).clamp(0.0, 1.0);
        let shape = perlin_worley(uvw * 5.0 + Vec3::splat(self.seed as f32 * 0.01));
        let mass = shape * profile;
        ((mass - (1.0 - coverage)) / coverage.max(0.06)).clamp(0.0, 1.0)
    }
}

fn smoothstep(a: f32, b: f32, x: f32) -> f32 {
    let t = ((x - a) / (b - a)).clamp(0.0, 1.0);
    t * t * (3.0 - 2.0 * t)
}

// --- texture baking ---------------------------------------------------------

fn shape_volume(n: u32) -> Image {
    let mut data = Vec::with_capacity((n * n * n) as usize);
    for z in 0..n {
        for y in 0..n {
            for x in 0..n {
                let p = Vec3::new(x as f32, y as f32, z as f32) / n as f32 * 5.0;
                let d = perlin_worley_periodic(p, 5);
                data.push((d * 255.0 + 0.5) as u8);
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

fn weather_map(n: u32) -> Image {
    let mut data = Vec::with_capacity((n * n) as usize);
    for y in 0..n {
        for x in 0..n {
            // Tileable: lattice cells wrap at the noise period, so the texture
            // repeats across the 8 km weather tile without a seam.
            let p = Vec3::new(x as f32 / n as f32, 0.0, y as f32 / n as f32) * 4.0;
            let v = 0.55 * value_noise_periodic(p, 4, 7)
                + 0.45 * value_noise_periodic(p * 2.0, 8, 11);
            data.push((v.clamp(0.0, 1.0) * 255.0) as u8);
        }
    }
    let mut image = Image::new(
        Extent3d {
            width: n,
            height: n,
            depth_or_array_layers: 1,
        },
        TextureDimension::D2,
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

fn perlin_worley(p: Vec3) -> f32 {
    let d = 1.0 - worley(p);
    if d <= 1e-4 {
        return 1.0;
    }
    ((perlin(p) * 0.5 + 0.5) / d).clamp(0.0, 1.0)
}

/// Tileable Perlin-Worley: cell indices wrap at `period`, so the baked volume
/// repeats in every axis without the seams that otherwise cut the sky in two.
fn perlin_worley_periodic(p: Vec3, period: i32) -> f32 {
    let d = 1.0 - worley_periodic(p, period);
    if d <= 1e-4 {
        return 1.0;
    }
    ((perlin_periodic(p, period) * 0.5 + 0.5) / d).clamp(0.0, 1.0)
}

fn perlin_periodic(p: Vec3, period: i32) -> f32 {
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
                let g = GRAD[(hash3(
                    (ix + dx).rem_euclid(period),
                    (iy + dy).rem_euclid(period),
                    (iz + dz).rem_euclid(period),
                    0,
                ) % 12) as usize];
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

fn worley_periodic(p: Vec3, period: i32) -> f32 {
    let i = p.floor();
    let (ix, iy, iz) = (i.x as i32, i.y as i32, i.z as i32);
    let mut best = f32::MAX;
    for dz in -1..=1 {
        for dy in -1..=1 {
            for dx in -1..=1 {
                // Hash the wrapped cell, but measure to the unwrapped cell so
                // the field stays continuous across the period boundary.
                let h = hash3(
                    (ix + dx).rem_euclid(period),
                    (iy + dy).rem_euclid(period),
                    (iz + dz).rem_euclid(period),
                    3,
                );
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

fn value_noise_periodic(p: Vec3, period: i32, seed: u32) -> f32 {
    let i = p.floor();
    let f = p - i;
    let u = Vec3::new(fade(f.x), fade(f.y), fade(f.z));
    let (ix, iy, iz) = (i.x as i32, i.y as i32, i.z as i32);
    let mut sum = 0.0;
    for dz in 0..2 {
        for dy in 0..2 {
            for dx in 0..2 {
                let h = hash3(
                    (ix + dx).rem_euclid(period),
                    (iy + dy).rem_euclid(period),
                    (iz + dz).rem_euclid(period),
                    seed,
                );
                let wx = if dx == 1 { u.x } else { 1.0 - u.x };
                let wy = if dy == 1 { u.y } else { 1.0 - u.y };
                let wz = if dz == 1 { u.z } else { 1.0 - u.z };
                sum += wx * wy * wz * unit(h);
            }
        }
    }
    sum
}

fn value_noise(p: Vec3, seed: u32) -> f32 {
    let i = p.floor();
    let f = p - i;
    let u = Vec3::new(fade(f.x), fade(f.y), fade(f.z));
    let (ix, iy, iz) = (i.x as i32, i.y as i32, i.z as i32);
    let mut sum = 0.0;
    for dz in 0..2 {
        for dy in 0..2 {
            for dx in 0..2 {
                let h = hash3(ix + dx, iy + dy, iz + dz, seed);
                let wx = if dx == 1 { u.x } else { 1.0 - u.x };
                let wy = if dy == 1 { u.y } else { 1.0 - u.y };
                let wz = if dz == 1 { u.z } else { 1.0 - u.z };
                sum += wx * wy * wz * unit(h);
            }
        }
    }
    sum
}

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
                let g = GRAD[(hash3(ix + dx, iy + dy, iz + dz, 0) % 12) as usize];
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

fn worley(p: Vec3) -> f32 {
    let i = p.floor();
    let (ix, iy, iz) = (i.x as i32, i.y as i32, i.z as i32);
    let mut best = f32::MAX;
    for dz in -1..=1 {
        for dy in -1..=1 {
            for dx in -1..=1 {
                let h = hash3(ix + dx, iy + dy, iz + dz, 3);
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

fn hash3(x: i32, y: i32, z: i32, seed: u32) -> u32 {
    hash00(
        (x as u32)
            .wrapping_mul(0x8da6_b343)
            .wrapping_add((y as u32).wrapping_mul(0xd816_3841))
            .wrapping_add((z as u32).wrapping_mul(0xcb1a_b31f))
            .wrapping_add(seed.wrapping_mul(0x9e37_79b9)),
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn probe_is_zero_outside_the_box() {
        let probe = CloudProbe::new(CloudDomain::default(), 1);
        assert_eq!(probe.density_at(Vec3::new(0.0, 0.0, 0.0)), 0.0);
        assert_eq!(probe.density_at(Vec3::new(0.0, 99_000.0, 0.0)), 0.0);
    }

    #[test]
    fn probe_matches_the_coverage_bias() {
        let mut domain = CloudDomain {
            coverage_bias: -2.0,
            ..Default::default()
        };
        let probe = CloudProbe::new(domain, 1);
        let p = Vec3::new(domain.center.x, 2_000.0, domain.center.z);
        assert_eq!(probe.density_at(p), 0.0, "no coverage means no cloud");

        domain.coverage_bias = 2.0;
        let probe = CloudProbe::new(domain, 1);
        assert!(probe.density_at(p) > 0.0, "full coverage means cloud");
    }

    #[test]
    fn textures_bake_deterministically() {
        let a = shape_volume(8).data.unwrap();
        let b = shape_volume(8).data.unwrap();
        assert_eq!(a, b);
    }
}
