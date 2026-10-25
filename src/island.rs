//! The archipelago, ported back from the terrain lab (`~/mapCreate`).
//!
//! The lab reworked the flight sim's copy in three ways that matter on the
//! web: the height field ships **pre-baked** (`assets/maps/archipelago.f32`,
//! 1201² f32 at 80 m spacing — the erosion filter never runs at runtime), the
//! mesh is three times denser, and the sea is a **transparent surface** over a
//! painted seabed. That last point is what kills the shoreline flicker: the
//! old sea was opaque, so every beach triangle that dipped near y=0 fought the
//! plane for depth every frame; now the shallows simply blend through.
//!
//! The land material paints a proper coastline (wet sand → beach → scrub →
//! jungle → rock) with triplanar rock detail, height-field AO, height-field
//! sun shadows and animated surf, driven by the tileable RGBA noise texture
//! the lab generates. Only the archipelago lives here — Alpine and Glacial
//! stay on `map.rs`'s procedural fields.
//!
//! Weather: `Weather` feeds in every frame — rain dims the sun term and
//! raises wetness, the night factor sweeps the palette toward the same navy
//! floor the other maps use.

use bevy::asset::RenderAssetUsages;
use bevy::image::{ImageAddressMode, ImageFilterMode, ImageSampler, ImageSamplerDescriptor};
use bevy::mesh::{Indices, PrimitiveTopology};
use bevy::prelude::*;
use bevy::render::render_resource::{
    AsBindGroup, Extent3d, ShaderType, TextureDimension, TextureFormat,
};
use bevy::shader::ShaderRef;
use std::sync::OnceLock;

use crate::app_state::AppState;
use crate::weather::Weather;

/// Half-width of the square arena, metres. The baked height field must agree.
const ARENA_HALF: f32 = 48_000.0;
/// Water plane is larger than the arena so its edge stays beyond the haze.
const WATER_SIZE: f32 = 140_000.0;
/// Resolution of the tileable RGBA detail texture.
const NOISE_SIZE: u32 = 512;
/// Aerial haze density, 1/m.
const HAZE: f32 = 0.000018;
/// Afternoon sun of the archipelago mission (35° up, north-west) — the lab's
/// reworked angle, which rakes the ridges for longer shadows.
const SUN_DIR: Vec3 = Vec3::new(-0.640, 0.420, 0.644);

/// Pre-baked height field: little-endian `u32 stride`, then stride² f32
/// heights, row-major from `-ARENA_HALF` at 80 m spacing (regenerate with
/// `mapCreate`'s `export_archipelago` bin).
const HEIGHTS_BIN: &[u8] = include_bytes!("../assets/maps/archipelago.f32");

// --- height field ------------------------------------------------------------

/// The baked grid: `(stride, heights)` parsed once from the binary.
fn heights() -> &'static (usize, &'static [f32]) {
    static HEIGHTS: OnceLock<(usize, &'static [f32])> = OnceLock::new();
    HEIGHTS.get_or_init(|| {
        let stride = u32::from_le_bytes(HEIGHTS_BIN[..4].try_into().unwrap()) as usize;
        let bytes = &HEIGHTS_BIN[4..];
        assert_eq!(
            bytes.len(),
            stride * stride * 4,
            "archipelago.f32: expected {stride}² f32 heights"
        );
        let mut out = Vec::with_capacity(stride * stride);
        for chunk in bytes.chunks_exact(4) {
            out.push(f32::from_le_bytes(chunk.try_into().unwrap()));
        }
        (stride, out.leak())
    })
}

/// Terrain mesh spacing, metres — the lab ships 80 m, 3× the old grid.
fn grid_step() -> f32 {
    ARENA_HALF * 2.0 / (heights().0 - 1) as f32
}

/// Ground height (metres above sea level) at world x/z, bilinear over the
/// baked grid and clamped to the arena. This is the archipelago's
/// `ground_height` answer everywhere the flight sim asks (mesh, spawn heights,
/// sea-floor assist).
pub fn ground_height(x: f32, z: f32) -> f32 {
    let (stride, hs) = *heights();
    let step = grid_step();
    let gx = ((x + ARENA_HALF) / step).clamp(0.0, (stride - 1) as f32);
    let gz = ((z + ARENA_HALF) / step).clamp(0.0, (stride - 1) as f32);
    let ix = gx.floor() as usize;
    let iz = gz.floor() as usize;
    let fx = gx - ix as f32;
    let fz = gz - iz as f32;
    let at = |ix: usize, iz: usize| hs[iz * stride + ix];
    let ix1 = (ix + 1).min(stride - 1);
    let iz1 = (iz + 1).min(stride - 1);
    let c00 = at(ix, iz);
    let c10 = at(ix1, iz);
    let c01 = at(ix, iz1);
    let c11 = at(ix1, iz1);
    let c0 = c00 + (c10 - c00) * fx;
    let c1 = c01 + (c11 - c01) * fx;
    c0 + (c1 - c0) * fz
}

// --- shader parameters -------------------------------------------------------

#[derive(Clone, Copy, ShaderType)]
pub struct TerrainParams {
    /// xyz = camera position, w = time (s).
    camera: Vec4,
    /// xyz = direction toward the sun, w = rain (0..1).
    sun: Vec4,
    /// Dry sand.
    low: Vec4,
    /// Vegetation.
    mid: Vec4,
    /// Rock.
    high: Vec4,
    /// Seabed.
    deep: Vec4,
    /// Haze colour.
    fog: Vec4,
    /// x = rock line, y = palette mode (0 = coastline), z = haze density,
    /// w = night factor.
    rules: Vec4,
    /// x = cloud coverage, y = wind, z = shadow strength, w = surf strength.
    cloud: Vec4,
}

impl TerrainParams {
    fn new(fog: Vec4) -> Self {
        Self {
            camera: Vec4::ZERO,
            sun: Vec4::new(SUN_DIR.x, SUN_DIR.y, SUN_DIR.z, 0.0),
            // The lab's archipelago palette: dry sand, jungle, rock, seabed.
            low: Vec4::new(0.60, 0.47, 0.30, 1.0),
            mid: Vec4::new(0.032, 0.092, 0.026, 1.0),
            high: Vec4::new(0.25, 0.215, 0.175, 1.0),
            deep: Vec4::new(0.005, 0.020, 0.038, 1.0),
            fog,
            rules: Vec4::new(500.0, 0.0, HAZE, 0.0),
            // coverage / wind / shadow strength / surf strength.
            cloud: Vec4::new(0.34, 1.0, 0.55, 1.0),
        }
    }
}

#[derive(Asset, TypePath, AsBindGroup, Clone)]
pub struct TerrainMaterial {
    #[uniform(0)]
    params: TerrainParams,
    /// Tileable 2D RGBA: R broad fBm, G ridged fBm, B cellular, A fine grain.
    #[texture(1)]
    #[sampler(2)]
    #[dependency]
    noise: Handle<Image>,
    /// Single-channel height field covering the arena, for sun shadows and AO.
    #[texture(3)]
    #[sampler(4)]
    #[dependency]
    height: Handle<Image>,
}

impl Material for TerrainMaterial {
    fn fragment_shader() -> ShaderRef {
        "shaders/island_terrain.wesl".into()
    }
    fn enable_prepass() -> bool {
        false
    }
    fn enable_shadows() -> bool {
        false
    }
}

#[derive(Clone, Copy, ShaderType)]
pub struct WaterParams {
    /// xyz = camera position, w = time (s).
    camera: Vec4,
    /// xyz = direction toward the sun, w = rain (0..1).
    sun: Vec4,
    /// Deep-water body colour.
    deep: Vec4,
    /// Shallow-water tint.
    shallow: Vec4,
    /// Haze colour, w = density.
    fog: Vec4,
    /// x = wave-LOD scale, y = swell height gain, z = glitter gain,
    /// w = glitter exponent.
    tune: Vec4,
    /// x = cloud base, y = cloud top, z = coverage, w = wind (m/s).
    cloud: Vec4,
    /// Night factor (0 day .. 1 deep night).
    night: f32,
}

impl WaterParams {
    fn new(fog: Vec4) -> Self {
        Self {
            camera: Vec4::ZERO,
            sun: Vec4::new(SUN_DIR.x, SUN_DIR.y, SUN_DIR.z, 0.0),
            deep: Vec4::new(0.003, 0.035, 0.075, 1.0),
            shallow: Vec4::new(0.015, 0.130, 0.145, 1.0),
            fog,
            tune: Vec4::new(1.0, 1.0, 1.0, 0.78),
            cloud: Vec4::ZERO,
            night: 0.0,
        }
    }
}

#[derive(Asset, TypePath, AsBindGroup, Clone)]
pub struct WaterMaterial {
    #[uniform(0)]
    params: WaterParams,
    /// Tileable 2D RGBA, for the breaking-surf noise.
    #[texture(1)]
    #[sampler(2)]
    #[dependency]
    noise: Handle<Image>,
    /// Arena height field, for depth-based shallows, transparency and surf.
    #[texture(3)]
    #[sampler(4)]
    #[dependency]
    height: Handle<Image>,
    /// The sky's 3D weather field, for cloud shadows on the sea.
    #[texture(5, dimension = "3d")]
    #[sampler(6)]
    #[dependency]
    cloud_noise: Handle<Image>,
}

impl Material for WaterMaterial {
    fn fragment_shader() -> ShaderRef {
        "shaders/island_water.wesl".into()
    }
    fn enable_prepass() -> bool {
        false
    }
    fn enable_shadows() -> bool {
        false
    }
    /// The sea blends over the painted seabed: shallow shelves stay turquoise,
    /// deep water reads blue because the seabed under it is nearly black.
    fn alpha_mode(&self) -> AlphaMode {
        AlphaMode::Blend
    }
}

// --- scene -------------------------------------------------------------------

/// Handles to the materials built for the current archipelago flight, so the
/// per-frame systems can drive the camera and weather uniforms.
#[derive(Resource, Default)]
pub struct IslandHandles {
    terrain: Option<Handle<TerrainMaterial>>,
    water: Option<Handle<WaterMaterial>>,
}

pub struct IslandPlugin;

impl Plugin for IslandPlugin {
    fn build(&self, app: &mut App) {
        app.add_plugins((
            MaterialPlugin::<TerrainMaterial>::default(),
            MaterialPlugin::<WaterMaterial>::default(),
        ))
        .init_resource::<IslandHandles>()
        .add_systems(Update, update_params);
    }
}

/// Build the terrain mesh, the water plane and their materials for one
/// mission entry. Called from `map.rs`'s `spawn_arena` on
/// `MapId::Archipelago`; the entities despawn with the flight like the rest
/// of the arena.
pub fn build(
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    images: &mut Assets<Image>,
    terrain_mats: &mut Assets<TerrainMaterial>,
    water_mats: &mut Assets<WaterMaterial>,
    handles: &mut IslandHandles,
    weather: &Weather,
    cloud_noise: Handle<Image>,
    cloud: Vec4,
) {
    let noise = images.add(detail_image());
    let (mesh, height_img) = terrain_mesh_and_height_image();
    let height = images.add(height_img);
    let f3 = crate::map::fog_color(0.0, weather.night_factor());
    let fog = Vec4::new(f3[0], f3[1], f3[2], 0.0);

    let terrain = terrain_mats.add(TerrainMaterial {
        params: TerrainParams::new(fog),
        noise: noise.clone(),
        height: height.clone(),
    });
    commands.spawn((
        Mesh3d(meshes.add(mesh)),
        MeshMaterial3d(terrain.clone()),
        Transform::default(),
        DespawnOnExit(AppState::Flying),
    ));

    let water = water_mats.add(WaterMaterial {
        params: {
            let mut p = WaterParams::new(fog);
            // The water shader reads the haze density from fog.w.
            p.fog.w = HAZE;
            p.cloud = cloud;
            p.night = weather.night_factor();
            p
        },
        noise,
        height,
        cloud_noise,
    });
    commands.spawn((
        Mesh3d(meshes.add(Plane3d::default().mesh().size(WATER_SIZE, WATER_SIZE))),
        MeshMaterial3d(water.clone()),
        Transform::default(),
        DespawnOnExit(AppState::Flying),
    ));

    *handles = IslandHandles {
        terrain: Some(terrain),
        water: Some(water),
    };
}

/// Per-frame uniforms: camera pose, time, and the weather the rest of the sim
/// flies under (rain dims the sun and raises wetness, night sweeps the palette
/// toward the navy floor the other maps use).
fn update_params(
    time: Res<Time>,
    weather: Res<Weather>,
    cloud: Res<crate::sky::CloudField>,
    handles: Res<IslandHandles>,
    camera: Query<&GlobalTransform, (With<Camera3d>, Without<crate::cloud_camera::CloudCamera>)>,
    mut terrain_mats: ResMut<Assets<TerrainMaterial>>,
    mut water_mats: ResMut<Assets<WaterMaterial>>,
) {
    let Ok(cam) = camera.single() else {
        return;
    };
    let pos = cam.translation();
    let t = time.elapsed_secs();
    let rain = weather.rain;
    let night = weather.night_factor();

    if let Some(handle) = &handles.terrain {
        if let Some(mut mat) = terrain_mats.get_mut(handle) {
            mat.params.camera = Vec4::new(pos.x, pos.y, pos.z, t);
            mat.params.sun.w = rain;
            mat.params.rules.w = night;
        }
    }
    if let Some(handle) = &handles.water {
        if let Some(mut mat) = water_mats.get_mut(handle) {
            mat.params.camera = Vec4::new(pos.x, pos.y, pos.z, t);
            mat.params.sun.w = rain;
            mat.params.cloud = Vec4::new(
                cloud.base,
                cloud.top,
                crate::sky::coverage_for(rain),
                cloud.wind,
            );
            mat.params.night = night;
        }
    }
}

// --- mesh and textures -------------------------------------------------------

/// The terrain mesh (80 m grid straight off the baked heights, normals from
/// the same grid so neighbouring quads agree across their seams) plus the
/// height field as an R32Float texture for the shaders' shadow/AO/depth reads.
fn terrain_mesh_and_height_image() -> (Mesh, Image) {
    let (stride, hs) = *heights();
    let n = stride - 1;
    let step = grid_step();

    let mut positions = Vec::with_capacity(stride * stride);
    for iz in 0..stride {
        let z = -ARENA_HALF + iz as f32 * step;
        for ix in 0..stride {
            let x = -ARENA_HALF + ix as f32 * step;
            positions.push([x, hs[iz * stride + ix], z]);
        }
    }

    let mut normals = Vec::with_capacity(stride * stride);
    for iz in 0..stride {
        for ix in 0..stride {
            let left = hs[iz * stride + ix.saturating_sub(1)];
            let right = hs[iz * stride + (ix + 1).min(n)];
            let back = hs[iz.saturating_sub(1) * stride + ix];
            let front = hs[(iz + 1).min(n) * stride + ix];
            let dhdx = (right - left) / (2.0 * step);
            let dhdz = (front - back) / (2.0 * step);
            normals.push(Vec3::new(-dhdx, 1.0, -dhdz).normalize().to_array());
        }
    }

    let mut indices = Vec::with_capacity(n * n * 6);
    for iz in 0..n {
        for ix in 0..n {
            let i = (iz * stride + ix) as u32;
            let s = stride as u32;
            // Counter-clockwise seen from above, so the front face is the top.
            indices.extend_from_slice(&[i, i + s, i + 1, i + 1, i + s, i + s + 1]);
        }
    }

    let mut mesh = Mesh::new(
        PrimitiveTopology::TriangleList,
        RenderAssetUsages::RENDER_WORLD,
    );
    mesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, positions);
    mesh.insert_attribute(Mesh::ATTRIBUTE_NORMAL, normals);
    mesh.insert_indices(Indices::U32(indices));

    let mut data = Vec::with_capacity(hs.len() * 4);
    for h in hs {
        data.extend_from_slice(&h.to_le_bytes());
    }
    let mut image = Image::new_uninit(
        Extent3d {
            width: stride as u32,
            height: stride as u32,
            depth_or_array_layers: 1,
        },
        TextureDimension::D2,
        TextureFormat::R32Float,
        RenderAssetUsages::RENDER_WORLD,
    );
    image.data = Some(data);
    image.sampler = ImageSampler::Descriptor(ImageSamplerDescriptor {
        address_mode_u: ImageAddressMode::ClampToEdge,
        address_mode_v: ImageAddressMode::ClampToEdge,
        mag_filter: ImageFilterMode::Linear,
        min_filter: ImageFilterMode::Linear,
        mipmap_filter: ImageFilterMode::Linear,
        ..default()
    });

    (mesh, image)
}

// --- tileable detail texture -------------------------------------------------

/// Four decorrelated tileable channels at 512² with a full mip pyramid:
/// R broad fBm (colour breakup / micro relief), G ridged fBm (rock strata),
/// B cellular (patchiness), A fine grain.
fn detail_image() -> Image {
    let n = NOISE_SIZE as usize;
    let mut base = Vec::with_capacity(n * n * 4);
    for y in 0..n {
        for x in 0..n {
            let u = x as f32 / n as f32;
            let v = y as f32 / n as f32;
            let r = tile_fbm(u, v, 4, 5, 0x5EED_1234);
            let g = tile_ridged(u, v, 4, 5, 0x1234_ABCD);
            let b = tile_worley(u, v, 8, 0x0BAD_F00D);
            let a = tile_fbm(u, v, 32, 3, 0xFEED_5678);
            base.extend_from_slice(&[
                (r * 255.0 + 0.5) as u8,
                (g * 255.0 + 0.5) as u8,
                (b * 255.0 + 0.5) as u8,
                (a * 255.0 + 0.5) as u8,
            ]);
        }
    }

    let mut levels = vec![base];
    let mut size = n;
    while size > 1 {
        let next = (size / 2).max(1);
        let src = levels.last().unwrap();
        let mut out = Vec::with_capacity(next * next * 4);
        for y in 0..next {
            for x in 0..next {
                for c in 0..4 {
                    let at = |sx: usize, sy: usize| -> u32 {
                        src[((sy.min(size - 1) * size) + sx.min(size - 1)) * 4 + c] as u32
                    };
                    let sum = at(x * 2, y * 2)
                        + at(x * 2 + 1, y * 2)
                        + at(x * 2, y * 2 + 1)
                        + at(x * 2 + 1, y * 2 + 1);
                    out.push((sum / 4) as u8);
                }
            }
        }
        levels.push(out);
        size = next;
    }

    let mut data = Vec::new();
    for level in &levels {
        data.extend_from_slice(level);
    }

    let mut image = Image::new_uninit(
        Extent3d {
            width: NOISE_SIZE,
            height: NOISE_SIZE,
            depth_or_array_layers: 1,
        },
        TextureDimension::D2,
        TextureFormat::Rgba8Unorm,
        RenderAssetUsages::RENDER_WORLD,
    );
    image.data = Some(data);
    image.texture_descriptor.mip_level_count = levels.len() as u32;
    image.sampler = ImageSampler::Descriptor(ImageSamplerDescriptor {
        address_mode_u: ImageAddressMode::Repeat,
        address_mode_v: ImageAddressMode::Repeat,
        mag_filter: ImageFilterMode::Linear,
        min_filter: ImageFilterMode::Linear,
        mipmap_filter: ImageFilterMode::Linear,
        ..default()
    });
    image
}

fn tile_fbm(x: f32, y: f32, cells: i32, octaves: u32, seed: u32) -> f32 {
    let mut sum = 0.0;
    let mut norm = 0.0;
    let mut amp = 0.5;
    let mut c = cells;
    for octave in 0..octaves {
        sum += amp * tile_noise(x, y, c, seed.wrapping_add(octave.wrapping_mul(0x9E37_79B9)));
        norm += amp;
        amp *= 0.5;
        c *= 2;
    }
    sum / norm
}

fn tile_ridged(x: f32, y: f32, cells: i32, octaves: u32, seed: u32) -> f32 {
    let mut sum = 0.0;
    let mut norm = 0.0;
    let mut amp = 0.5;
    let mut c = cells;
    for octave in 0..octaves {
        let n = 1.0
            - (2.0 * tile_noise(x, y, c, seed.wrapping_add(octave.wrapping_mul(0x85EB_CA6B)))
                - 1.0)
                .abs();
        sum += amp * n * n;
        norm += amp;
        amp *= 0.5;
        c *= 2;
    }
    sum / norm
}

fn tile_noise(x: f32, y: f32, cells: i32, seed: u32) -> f32 {
    let xf = x * cells as f32;
    let yf = y * cells as f32;
    let xi = xf.floor() as i32;
    let yi = yf.floor() as i32;
    let fx = smooth(xf - xi as f32);
    let fy = smooth(yf - yi as f32);
    let w = |i: i32| i.rem_euclid(cells);
    let c00 = hash2(w(xi), w(yi), seed);
    let c10 = hash2(w(xi + 1), w(yi), seed);
    let c01 = hash2(w(xi), w(yi + 1), seed);
    let c11 = hash2(w(xi + 1), w(yi + 1), seed);
    let c0 = c00 + (c10 - c00) * fx;
    let c1 = c01 + (c11 - c01) * fx;
    c0 + (c1 - c0) * fy
}

/// Tileable cellular noise: nearest feature point of the 3×3 neighbourhood.
fn tile_worley(x: f32, y: f32, cells: i32, seed: u32) -> f32 {
    let xf = x * cells as f32;
    let yf = y * cells as f32;
    let xi = xf.floor() as i32;
    let yi = yf.floor() as i32;
    let mut best = f32::MAX;
    for dz in -1..=1 {
        for dx in -1..=1 {
            let cx = xi + dx;
            let cy = yi + dz;
            let wx = cx.rem_euclid(cells);
            let wy = cy.rem_euclid(cells);
            let px = cx as f32 + hash2(wx, wy, seed);
            let py = cy as f32 + hash2(wx, wy, seed ^ 0x51ED_2701);
            let d = ((xf - px).powi(2) + (yf - py).powi(2)).sqrt();
            best = best.min(d);
        }
    }
    (1.0 - best).clamp(0.0, 1.0)
}

fn hash2(x: i32, z: i32, seed: u32) -> f32 {
    let mut h = seed
        .wrapping_add((x as u32).wrapping_mul(0x8DA6_B343))
        .wrapping_add((z as u32).wrapping_mul(0xD816_3841));
    h ^= h >> 13;
    h = h.wrapping_mul(0x5BD1_E995);
    h ^= h >> 15;
    h = h.wrapping_mul(0x27D4_EB2D);
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
    fn baked_heights_answer_finite_and_shape_the_arena() {
        let mut sea = 0;
        let mut high = 0;
        let samples = 40 * 40;
        for iz in 0..40 {
            for ix in 0..40 {
                let x = -ARENA_HALF + ix as f32 * ARENA_HALF / 20.0;
                let z = -ARENA_HALF + iz as f32 * ARENA_HALF / 20.0;
                let h = ground_height(x, z);
                assert!(h.is_finite(), "height at ({x}, {z}) = {h}");
                sea += (h < 0.0) as i32;
                high += (h > 120.0) as i32;
            }
        }
        assert!(sea > samples / 10, "expected open sea, got {sea}/{samples}");
        assert!(high > samples / 50, "expected islands, got {high}/{samples}");
    }
}
