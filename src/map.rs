//! Arcade mission maps: a procedural heightfield, a matching water plane and
//! the terrain-avoidance assists that keep an always-flying F-22 in the air.
//!
//! The arena is a 96 km square centred on the world origin. A mission spawns
//! the jet at the origin, so the terrain is a fixed patch rather than an
//! endlessly streamed world: this is the Ace Combat model — a bounded, hand-
//! tuned arena that reads as vast because of haze and cloud, not a live
//! planet. Heights are world-space functions, so the mesher, the terrain shader
//! and the per-step avoidance assist all sample the same surface.

use bevy::asset::RenderAssetUsages;
use bevy::image::{ImageAddressMode, ImageFilterMode, ImageSampler, ImageSamplerDescriptor};
use bevy::mesh::{Indices, PrimitiveTopology};
use bevy::prelude::*;
use bevy::render::render_resource::{
    AsBindGroup, Extent3d, ShaderType, TextureDimension, TextureFormat,
};
use bevy::shader::ShaderRef;

use crate::app_state::AppState;
use crate::sim::StartFlight;
use crate::sky::{coverage_for, CloudField};

/// Half-width of the square arena, metres. The terrain spans ±48 km.
pub const ARENA_HALF: f32 = 48_000.0;
/// Terrain mesh spacing, metres. 400 quads per side keeps the mesh at ~320k
/// triangles (one draw call) while still reading as sculpted relief.
const GRID_STEP: f32 = 240.0;
/// Distance from the centre where the arena starts banking the jet back.
pub const RETURN_RADIUS: f32 = 40_000.0;
/// Aerial haze density, 1/m. Light enough to keep mid-field relief legible,
/// dense enough at ~48 km to fold the mesh edge into the horizon.
const HAZE: f32 = 0.000032;
/// Terrain that must be cleared ahead of the jet, metres below the nose.
const CLEARANCE: f32 = 700.0;
/// Height the assist holds over open water, metres: the mission minimum over
/// the sea. Land keeps the full `CLEARANCE`.
const SEA_CLEARANCE: f32 = 500.0;
/// Hard floor above the sea surface, metres. The FDM stop catches a dive the
/// assist could not recover in time.
pub const SEA_FLOOR_M: f32 = 500.0;
/// Where the terrain look-ahead samples sit, metres ahead along the ground
/// track. At arcade cruise (~215 m/s) the far sample is ~12 s out, which gives
/// the assist room to ease the nose up instead of snatching at the ridge.
const AHEAD: [f32; 3] = [700.0, 1_500.0, 2_500.0];
/// Spawn cruise speed for every mission, knots.
const MISSION_KT: f64 = 420.0;
/// Water plane is larger than the arena so its own edge stays beyond the haze.
const WATER_SIZE: f32 = 140_000.0;
/// Resolution of the tileable detail texture the terrain shader uses.
const NOISE_SIZE: u32 = 256;

const DEFAULT_SUN: (f32, f32) = (-0.5, -0.6);

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MapId {
    Archipelago,
    Alpine,
}

pub const MAPS: [MapId; 2] = [MapId::Archipelago, MapId::Alpine];

/// Menu labels, in `MAPS` order. The menu sub-row renders these strings, so
/// they are the stable interface between the two modules.
pub const MAP_LABELS: [&str; MAPS.len()] = ["ARCHIPELAGO", "ALPINE"];

impl MapId {
    pub fn label(self) -> &'static str {
        MAP_LABELS[match self {
            MapId::Archipelago => 0,
            MapId::Alpine => 1,
        }]
    }

    pub fn from_label(label: &str) -> Option<MapId> {
        MAPS.iter().copied().find(|m| m.label() == label)
    }

    /// High-angle afternoon light over the islands; a low sun rakes the snow.
    fn sun(self) -> (f32, f32) {
        match self {
            MapId::Archipelago => (0.62, -0.65),
            MapId::Alpine => (0.40, -1.15),
        }
    }

    fn palette(self) -> Palette {
        match self {
            MapId::Archipelago => Palette {
                low: [0.52, 0.41, 0.24],
                mid: [0.045, 0.135, 0.035],
                high: [0.26, 0.24, 0.20],
                deep: [0.010, 0.030, 0.040],
                line: 420.0,
                mode: 0.0,
            },
            MapId::Alpine => Palette {
                low: [0.10, 0.185, 0.07],
                mid: [0.24, 0.22, 0.19],
                high: [0.92, 0.95, 1.0],
                deep: [0.16, 0.15, 0.14],
                line: 2_400.0,
                mode: 1.0,
            },
        }
    }

    /// Airborne spawn altitude in feet, seated clear of the terrain and the
    /// cloud layer: low enough for the archipelago to read, above the deck for
    /// the alpine peaks.
    pub fn spawn_alt_ft(self) -> f64 {
        let ground = ground_height(self, 0.0, 0.0) as f64;
        let m = match self {
            // Clear of the central ridge, inside the visual band the islands
            // read best from, and below the cloud deck.
            MapId::Archipelago => 1_200.0,
            // Above the cloud top so the opening shot looks down on the peaks.
            MapId::Alpine => (ground + 900.0).max(7_200.0),
        };
        m / 0.3048
    }

    /// Mission spawn speed, knots.
    pub fn spawn_kts(self) -> f64 {
        MISSION_KT
    }
}

struct Palette {
    low: [f32; 3],
    mid: [f32; 3],
    high: [f32; 3],
    deep: [f32; 3],
    /// Rock line (archipelago) or snow line (alpine), metres.
    line: f32,
    /// 0 = archipelago palette, 1 = alpine palette.
    mode: f32,
}

/// A mission chosen in the menu, waiting for the flight to start. `start_flight`
/// forces the F-22 and an airborne spawn when this is set; the map plugin then
/// builds the terrain and clears it.
#[derive(Resource, Default)]
pub struct PendingMission(pub Option<MapId>);

/// The map the current flight is flying over, if any. Flight systems gate their
/// arcade assists on this.
#[derive(Resource, Default)]
pub struct ActiveMap(pub Option<MapId>);

#[derive(Resource, Default)]
struct MapHandles {
    terrain: Option<Handle<TerrainMaterial>>,
    water: Option<Handle<WaterMaterial>>,
}

// --- shader parameters ------------------------------------------------------

#[derive(Clone, Copy, ShaderType, Default)]
struct TerrainParams {
    /// xyz = camera position, w = time (s).
    camera: Vec4,
    /// xyz = direction toward the sun, w = rain (0..1).
    sun: Vec4,
    /// Ground colour: sand / alpine meadow.
    low: Vec4,
    /// Vegetation / rock.
    mid: Vec4,
    /// Rock / snow.
    high: Vec4,
    /// Seabed / scree.
    deep: Vec4,
    /// Haze colour, the sky horizon under the current weather.
    fog: Vec4,
    /// x = rock/snow line, y = palette mode, z = haze density, w = unused.
    rules: Vec4,
    /// x = cloud base, y = cloud top, z = coverage, w = wind (m/s).
    cloud: Vec4,
}

#[derive(Asset, TypePath, AsBindGroup, Clone)]
struct TerrainMaterial {
    #[uniform(0)]
    params: TerrainParams,
    /// Tileable 2D fBm for colour breakup and snow-line jitter.
    #[texture(1)]
    #[sampler(2)]
    #[dependency]
    noise: Handle<Image>,
    /// The cloud volume, sampled upward along the sun to cast cloud shadows.
    #[texture(3, dimension = "3d")]
    #[sampler(4)]
    #[dependency]
    cloud_noise: Handle<Image>,
}

impl Material for TerrainMaterial {
    fn fragment_shader() -> ShaderRef {
        "shaders/terrain.wesl".into()
    }
    fn enable_prepass() -> bool {
        false
    }
    fn enable_shadows() -> bool {
        false
    }
}

#[derive(Clone, Copy, ShaderType, Default)]
struct WaterParams {
    /// xyz = camera position, w = time (s).
    camera: Vec4,
    /// xyz = direction toward the sun, w = rain (0..1).
    sun: Vec4,
    /// Deep-water colour.
    deep: Vec4,
    /// Haze colour.
    fog: Vec4,
    /// x = wave-LOD scale, y = swell gain, z = glitter gain, w = glitter exponent.
    tune: Vec4,
    /// x = cloud base, y = cloud top, z = coverage, w = wind (m/s).
    cloud: Vec4,
}

#[derive(Asset, TypePath, AsBindGroup, Clone)]
struct WaterMaterial {
    #[uniform(0)]
    params: WaterParams,
    /// The cloud volume, sampled upward along the sun to cast cloud shadows.
    #[texture(1, dimension = "3d")]
    #[sampler(2)]
    #[dependency]
    cloud_noise: Handle<Image>,
}

impl Material for WaterMaterial {
    fn fragment_shader() -> ShaderRef {
        "shaders/water.wesl".into()
    }
    fn enable_prepass() -> bool {
        false
    }
    fn enable_shadows() -> bool {
        false
    }
}

// --- plugin -----------------------------------------------------------------

pub struct MapPlugin;

impl Plugin for MapPlugin {
    fn build(&self, app: &mut App) {
        app.add_plugins((
            MaterialPlugin::<TerrainMaterial>::default(),
            MaterialPlugin::<WaterMaterial>::default(),
        ))
        .init_resource::<PendingMission>()
        .init_resource::<ActiveMap>()
        .init_resource::<MapHandles>()
        .add_systems(OnEnter(AppState::Flying), enter_mission.after(StartFlight))
        .add_systems(OnExit(AppState::Flying), leave_mission)
        .add_systems(Update, update_map_params.run_if(in_state(AppState::Flying)));
    }
}

/// Build the terrain (and sea) for a mission. Runs after `start_flight`, which
/// leaves the pending map in place so this can read it.
fn enter_mission(
    mut commands: Commands,
    mut pending: ResMut<PendingMission>,
    mut active: ResMut<ActiveMap>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut images: ResMut<Assets<Image>>,
    mut terrain_mats: ResMut<Assets<TerrainMaterial>>,
    mut water_mats: ResMut<Assets<WaterMaterial>>,
    weather: Res<crate::weather::Weather>,
    cloud: Res<CloudField>,
    mut lights: Query<&mut Transform, With<DirectionalLight>>,
) {
    let Some(map) = pending.0.take() else {
        return;
    };
    active.0 = Some(map);

    let pal = map.palette();
    let fog = fog_color(0.0);
    let cloud_params = Vec4::new(
        cloud.base,
        cloud.top,
        coverage_for(weather.rain),
        cloud.wind,
    );
    let material = terrain_mats.add(TerrainMaterial {
        params: TerrainParams {
            camera: Vec4::ZERO,
            sun: Vec4::new(0.0, 1.0, 0.0, 0.0),
            low: vec4_rgb(pal.low),
            mid: vec4_rgb(pal.mid),
            high: vec4_rgb(pal.high),
            deep: vec4_rgb(pal.deep),
            fog: vec4_rgb(fog),
            rules: Vec4::new(pal.line, pal.mode, HAZE, 0.0),
            cloud: cloud_params,
        },
        noise: images.add(noise_image()),
        cloud_noise: cloud.noise.clone(),
    });
    commands.spawn((
        Mesh3d(meshes.add(terrain_mesh(map))),
        MeshMaterial3d(material.clone()),
        Transform::default(),
        DespawnOnExit(AppState::Flying),
    ));

    if map == MapId::Archipelago {
        let water = water_mats.add(WaterMaterial {
            params: WaterParams {
                camera: Vec4::ZERO,
                sun: Vec4::new(0.0, 1.0, 0.0, 0.0),
                deep: Vec4::new(0.006, 0.028, 0.052, 1.0),
                fog: Vec4::new(fog[0], fog[1], fog[2], HAZE),
                // Fine waves survive ~1.7x farther out than the raw footprint
                // LOD, so the sea keeps the detail of the near-surface renders.
                tune: Vec4::new(0.6, 1.05, 1.1, 65.0),
                cloud: cloud_params,
            },
            cloud_noise: cloud.noise.clone(),
        });
        commands.spawn((
            Mesh3d(meshes.add(Plane3d::default().mesh().size(WATER_SIZE, WATER_SIZE))),
            MeshMaterial3d(water.clone()),
            Transform::default(),
            DespawnOnExit(AppState::Flying),
        ));
        commands.insert_resource(MapHandles {
            terrain: Some(material),
            water: Some(water),
        });
    } else {
        commands.insert_resource(MapHandles {
            terrain: Some(material),
            water: None,
        });
    }

    let (el, az) = map.sun();
    for mut t in &mut lights {
        t.rotation = Quat::from_euler(EulerRot::XYZ, -el, az, 0.0);
    }
}

fn leave_mission(
    mut active: ResMut<ActiveMap>,
    mut handles: ResMut<MapHandles>,
    mut lights: Query<&mut Transform, With<DirectionalLight>>,
) {
    active.0 = None;
    *handles = MapHandles::default();
    for mut t in &mut lights {
        let (el, az) = (DEFAULT_SUN.0, DEFAULT_SUN.1);
        t.rotation = Quat::from_euler(EulerRot::XYZ, -el, az, 0.0);
    }
}

fn update_map_params(
    time: Res<Time>,
    weather: Res<crate::weather::Weather>,
    camera: Query<&GlobalTransform, (With<Camera3d>, Without<crate::cloud_camera::CloudCamera>)>,
    sun: Query<&GlobalTransform, With<DirectionalLight>>,
    handles: Res<MapHandles>,
    cloud: Res<CloudField>,
    mut terrain_mats: ResMut<Assets<TerrainMaterial>>,
    mut water_mats: ResMut<Assets<WaterMaterial>>,
) {
    let (Ok(cam), Ok(sun)) = (camera.single(), sun.single()) else {
        return;
    };
    let pos = cam.translation();
    let sun_dir = (sun.rotation() * Vec3::Z).normalize_or_zero();
    let rain = weather.rain as f32;
    let fog = fog_color(rain);
    let t = time.elapsed_secs();
    let cloud_params = Vec4::new(cloud.base, cloud.top, coverage_for(rain), cloud.wind);

    if let Some(handle) = &handles.terrain {
        if let Some(mut mat) = terrain_mats.get_mut(handle) {
            mat.params.camera = Vec4::new(pos.x, pos.y, pos.z, t);
            mat.params.sun = Vec4::new(sun_dir.x, sun_dir.y, sun_dir.z, rain);
            mat.params.fog = vec4_rgb(fog);
            mat.params.cloud = cloud_params;
        }
    }
    if let Some(handle) = &handles.water {
        if let Some(mut mat) = water_mats.get_mut(handle) {
            mat.params.camera = Vec4::new(pos.x, pos.y, pos.z, t);
            mat.params.sun = Vec4::new(sun_dir.x, sun_dir.y, sun_dir.z, rain);
            mat.params.fog = Vec4::new(fog[0], fog[1], fog[2], HAZE);
            mat.params.cloud = cloud_params;
        }
    }
}

fn vec4_rgb(rgb: [f32; 3]) -> Vec4 {
    Vec4::new(rgb[0], rgb[1], rgb[2], 1.0)
}

/// Linear-light horizon colour for the current weather, matching
/// `sky_color` in `cloud_sky.wesl` so haze and sky meet seamlessly.
fn fog_color(rain: f32) -> [f32; 3] {
    let horizon = Vec3::new(0.55, 0.70, 0.87);
    let l = 0.299 * horizon.x + 0.587 * horizon.y + 0.114 * horizon.z;
    let grey = Vec3::new(l * 0.96, l, l * 1.04);
    (horizon.lerp(grey, 0.85 * rain) * (1.0 - 0.38 * rain)).to_array()
}

// --- terrain height field ---------------------------------------------------

/// Ground height (metres above sea level) at world x/z.
pub fn ground_height(map: MapId, x: f32, z: f32) -> f32 {
    match map {
        MapId::Archipelago => archipelago_height(x, z),
        MapId::Alpine => alpine_height(x, z),
    }
}

/// Scattered islands over open sea: a domain-warped continent field decides
/// where land exists, ridged noise builds volcanic interiors, and everything
/// under the sea is pushed into a shelf.
fn archipelago_height(x: f32, z: f32) -> f32 {
    let (wx, wz) = warp(x, z, 0.00016, 3_200.0, 1011);
    let cont = fbm(wx * 0.00006, wz * 0.00006, 4, 2021);
    let r = ((x * x + z * z).sqrt() / ARENA_HALF).clamp(0.0, 1.0);
    // The main landmass sits near the spawn; the edges open into empty sea.
    let central = (1.0 - r).powi(2) * 0.16;
    let land = smoothstep(0.51, 0.60, cont + central);

    let ridge = ridged(wx * 0.00030, wz * 0.00030, 5, 3031);
    // A second, tighter ridge set gives the volcanic flanks their spines.
    let crags = ridged(wx * 0.00070, wz * 0.00070, 4, 3121);
    let hills = fbm(wx * 0.0011, wz * 0.0011, 3, 4041);
    let fine = fbm(x * 0.004, z * 0.004, 2, 5051);
    let inland =
        ridge * ridge * 760.0 + crags.powf(2.0) * 90.0 + hills * 90.0 + (fine - 0.5) * 12.0;
    land * inland - (1.0 - land) * 55.0
}

/// Snow-capped massif over a high plateau: broad mass plus sharp ridged peaks,
/// with a low-frequency valley term so the ridges are cut by passes.
fn alpine_height(x: f32, z: f32) -> f32 {
    let (wx, wz) = warp(x, z, 0.00012, 3_600.0, 6067);
    let mass = fbm(wx * 0.00005, wz * 0.00005, 4, 7077);
    let ridge = ridged(wx * 0.00016, wz * 0.00016, 5, 8087);
    // Finer ridged octave: sharp aretes and couloirs on the big faces.
    let crags = ridged(wx * 0.00065, wz * 0.00065, 4, 8187);
    let detail = fbm(wx * 0.0009, wz * 0.0009, 3, 9097);
    let valleys = (fbm(wx * 0.00022, wz * 0.00022, 2, 1117) - 0.5) * 500.0;

    let base = 500.0 + 900.0 * mass;
    let peaks = ridge.powf(1.7) * (1_500.0 + 1_100.0 * mass) + crags.powf(2.0) * 220.0;
    (base + peaks + valleys + (detail - 0.5) * 180.0).max(20.0)
}

fn terrain_mesh(map: MapId) -> Mesh {
    let n = (ARENA_HALF * 2.0 / GRID_STEP) as usize;
    let stride = n + 1;
    let mut heights = Vec::with_capacity(stride * stride);
    let mut positions = Vec::with_capacity(stride * stride);

    for iz in 0..stride {
        let z = -ARENA_HALF + iz as f32 * GRID_STEP;
        for ix in 0..stride {
            let x = -ARENA_HALF + ix as f32 * GRID_STEP;
            let h = ground_height(map, x, z);
            heights.push(h);
            positions.push([x, h, z]);
        }
    }

    // Normals from the shared height grid: no extra height evaluations, and
    // neighbouring quads agree on the slope across their seam.
    let mut normals = Vec::with_capacity(stride * stride);
    for iz in 0..stride {
        for ix in 0..stride {
            let left = heights[iz * stride + ix.saturating_sub(1)];
            let right = heights[iz * stride + (ix + 1).min(n)];
            let back = heights[iz.saturating_sub(1) * stride + ix];
            let front = heights[(iz + 1).min(n) * stride + ix];
            let dhdx = (right - left) / (2.0 * GRID_STEP);
            let dhdz = (front - back) / (2.0 * GRID_STEP);
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
    mesh
}

// --- arcade assists ---------------------------------------------------------

/// Nose-up elevator demand (0..1) for a jet at `alt`, with `ground` directly
/// below and `ahead` the highest terrain over the look-ahead samples. Pulls
/// gently at the clearance boundary and hard when the ground is about to win.
pub fn avoid_pull(alt: f32, ground: f32, ahead: f32, vertical_speed_ms: f32) -> f32 {
    let sink = (-vertical_speed_ms).max(0.0) * 3.0;
    // Over water `ground` is the seabed, not a surface to clear; the sea plane
    // is at zero. `ahead` still sees islands, so approaching land restores the
    // full terrain clearance before the coast arrives.
    let surface = ground.max(0.0);
    let surface_ahead = ahead.max(0.0);
    let clearance = (alt - surface).min(alt - surface_ahead) - sink;
    let target = if surface > 0.0 || surface_ahead > 0.0 {
        CLEARANCE
    } else {
        SEA_CLEARANCE
    };
    if clearance >= target {
        return 0.0;
    }
    let t = ((target - clearance) / target).clamp(0.0, 1.0);
    t * t
}

/// Aileron command that banks a jet beyond `RETURN_RADIUS` back toward the
/// centre. Zero inside the arena, ramping to a hard turn at the mesh edge.
pub fn arena_aileron(x: f32, z: f32, heading_deg: f64) -> f64 {
    let r = (x * x + z * z).sqrt();
    if r <= RETURN_RADIUS {
        return 0.0;
    }
    // World x is east and world -z is north, so the vector to the centre is
    // (-x east, z north); heading is clockwise from north.
    let target = (-x).atan2(z).to_degrees().rem_euclid(360.0) as f64;
    let err = (target - heading_deg + 540.0).rem_euclid(360.0) - 180.0;
    let strength = ((r - RETURN_RADIUS) / 8_000.0).clamp(0.0, 1.0);
    (err / 45.0).clamp(-1.0, 1.0) * strength as f64
}

/// Highest terrain over the jets ground track, `dir` being the normalised
/// world-space horizontal heading and `x/z` the current position.
pub fn terrain_ahead(map: MapId, x: f32, z: f32, dir: Vec2) -> f32 {
    AHEAD
        .iter()
        .map(|d| ground_height(map, x + dir.x * d, z + dir.y * d))
        .fold(f32::NEG_INFINITY, f32::max)
}

/// Terrain-avoidance and arena-boundary assist, run once per FDM step after
/// the pilot's bindings. The arcade FDM reads a positive elevator as nose-down,
/// so a pull-up is a negative command.
pub fn assist(sim: &mut crate::sim::Sim, map: MapId) {
    let (pos, _, _) = sim.pose();

    // Ground track in world space (x = east, z = -north). Near the hover the
    // velocity vector is unusable, so fall back to the nose heading.
    let ve = sim.model.get("velocities/v-east-fps");
    let vn = sim.model.get("velocities/v-north-fps");
    let (mut dx, mut dz) = (ve, -vn);
    let speed = (dx * dx + dz * dz).sqrt();
    if speed < 1.0 {
        let psi = sim.model.get("attitude/psi-rad");
        (dx, dz) = (psi.sin(), -psi.cos());
    }
    let len = (dx * dx + dz * dz).sqrt().max(1e-6);
    let dir = Vec2::new((dx / len) as f32, (dz / len) as f32);

    let ground = ground_height(map, pos.x, pos.z);
    let ahead = terrain_ahead(map, pos.x, pos.z, dir);
    let vs_ms = sim.model.get("velocities/h-dot-fps") * 0.3048;
    let pull = avoid_pull(pos.y, ground, ahead, vs_ms as f32);

    if pull > 0.0 {
        let want = -(pull as f64);
        // Respect a stronger pull from the pilot; only add nose-up.
        if sim.model.get("fcs/elevator-cmd-norm") > want {
            sim.model.set("fcs/elevator-cmd-norm", want);
        }
        // Level the wings while the assist has the jet: a knife-edge bank turns
        // a pull-up into a horizontal turn into the hillside.
        let roll = sim.model.get("attitude/phi-deg");
        let level = (-(roll / 45.0)).clamp(-1.0, 1.0) * pull as f64;
        sim.model.set("fcs/aileron-cmd-norm", level);
    }

    // Outside the arena, bank back toward the centre. This wins over the
    // leveling above: by the time the edge matters, turning home is the point.
    let bank = arena_aileron(pos.x, pos.z, sim.model.get("attitude/psi-deg"));
    if bank != 0.0 {
        sim.model.set("fcs/aileron-cmd-norm", bank);
    }
}

// --- tileable detail noise --------------------------------------------------

fn noise_image() -> Image {
    let n = NOISE_SIZE;
    let mut data = Vec::with_capacity((n * n) as usize);
    for y in 0..n {
        for x in 0..n {
            let v = noise_fbm(x as f32 / n as f32, y as f32 / n as f32);
            data.push((v.clamp(0.0, 1.0) * 255.0).round() as u8);
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
        RenderAssetUsages::RENDER_WORLD,
    );
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

fn noise_fbm(x: f32, y: f32) -> f32 {
    let mut sum = 0.0;
    let mut norm = 0.0;
    let mut amp = 0.5;
    let mut cells = 4;
    for octave in 0..5u32 {
        sum += amp * tile_noise(x, y, cells, 0x5EED_1234 + octave);
        norm += amp;
        amp *= 0.5;
        cells *= 2;
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

// --- world-space noise ------------------------------------------------------

fn warp(x: f32, z: f32, scale: f32, amp: f32, seed: u32) -> (f32, f32) {
    let wx = x + (fbm(x * scale, z * scale, 2, seed) - 0.5) * amp;
    let wz = z + (fbm(x * scale + 5.2, z * scale + 1.3, 2, seed + 917) - 0.5) * amp;
    (wx, wz)
}

fn fbm(x: f32, z: f32, octaves: u32, seed: u32) -> f32 {
    let mut sum = 0.0;
    let mut norm = 0.0;
    let mut amp = 0.5;
    let mut freq = 1.0;
    for o in 0..octaves {
        sum += amp * value_noise(x * freq, z * freq, seed.wrapping_add(o * 37));
        norm += amp;
        amp *= 0.5;
        freq *= 2.0;
    }
    sum / norm
}

fn ridged(x: f32, z: f32, octaves: u32, seed: u32) -> f32 {
    let mut sum = 0.0;
    let mut norm = 0.0;
    let mut amp = 0.5;
    let mut freq = 1.0;
    for o in 0..octaves {
        let n =
            1.0 - (2.0 * value_noise(x * freq, z * freq, seed.wrapping_add(o * 53)) - 1.0).abs();
        sum += amp * n * n;
        norm += amp;
        amp *= 0.5;
        freq *= 2.0;
    }
    sum / norm
}

fn value_noise(x: f32, z: f32, seed: u32) -> f32 {
    let xi = x.floor();
    let zi = z.floor();
    let fx = smooth(x - xi);
    let fz = smooth(z - zi);
    let (xi, zi) = (xi as i32, zi as i32);
    let c00 = hash2(xi, zi, seed);
    let c10 = hash2(xi + 1, zi, seed);
    let c01 = hash2(xi, zi + 1, seed);
    let c11 = hash2(xi + 1, zi + 1, seed);
    let c0 = c00 + (c10 - c00) * fx;
    let c1 = c01 + (c11 - c01) * fx;
    c0 + (c1 - c0) * fz
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

fn smoothstep(e0: f32, e1: f32, x: f32) -> f32 {
    let t = ((x - e0) / (e1 - e0)).clamp(0.0, 1.0);
    t * t * (3.0 - 2.0 * t)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn heights_are_finite_across_both_arenas() {
        for map in MAPS {
            for iz in 0..20 {
                for ix in 0..20 {
                    let x = -ARENA_HALF + ix as f32 * ARENA_HALF / 10.0;
                    let z = -ARENA_HALF + iz as f32 * ARENA_HALF / 10.0;
                    let h = ground_height(map, x, z);
                    assert!(h.is_finite(), "{map:?} at ({x}, {z}) = {h}");
                    assert!((-200.0..6_000.0).contains(&h), "{map:?} height {h}");
                }
            }
        }
    }

    #[test]
    fn archipelago_has_open_sea_and_tall_islands() {
        let mut sea = 0;
        let mut high = 0;
        let samples = 40 * 40;
        for iz in 0..40 {
            for ix in 0..40 {
                let x = -ARENA_HALF + ix as f32 * ARENA_HALF / 20.0;
                let z = -ARENA_HALF + iz as f32 * ARENA_HALF / 20.0;
                let h = ground_height(MapId::Archipelago, x, z);
                sea += (h < 0.0) as i32;
                high += (h > 120.0) as i32;
            }
        }
        assert!(sea > samples / 10, "expected open sea, got {sea}/{samples}");
        assert!(
            high > samples / 50,
            "expected islands, got {high}/{samples}"
        );
    }

    #[test]
    fn alpine_reaches_snow_and_drops_to_valleys() {
        let mut min = f32::INFINITY;
        let mut max = f32::NEG_INFINITY;
        for iz in 0..40 {
            for ix in 0..40 {
                let x = -ARENA_HALF + ix as f32 * ARENA_HALF / 20.0;
                let z = -ARENA_HALF + iz as f32 * ARENA_HALF / 20.0;
                let h = ground_height(MapId::Alpine, x, z);
                min = min.min(h);
                max = max.max(h);
            }
        }
        assert!(max > 2_700.0, "snow line needs peaks, max {max}");
        assert!(min < 1_800.0, "needs valleys, min {min}");
    }

    #[test]
    fn terrain_mesh_has_upward_normals_and_consistent_buffers() {
        let mesh = terrain_mesh(MapId::Alpine);
        let n = (ARENA_HALF * 2.0 / GRID_STEP) as usize;
        assert_eq!(mesh.count_vertices(), (n + 1) * (n + 1));
        assert_eq!(mesh.indices().expect("indexed").len(), n * n * 6);
        let Some(bevy::mesh::VertexAttributeValues::Float32x3(normals)) =
            mesh.attribute(Mesh::ATTRIBUTE_NORMAL)
        else {
            panic!("normals missing");
        };
        for normal in normals.iter().step_by(97) {
            let n = Vec3::from_array(*normal);
            assert!((n.length() - 1.0).abs() < 1e-3, "normal not unit: {n}");
            assert!(n.y > 0.0, "terrain normal must face up: {n}");
        }
    }

    #[test]
    fn terrain_pull_is_off_when_clear_and_hard_when_low() {
        assert_eq!(avoid_pull(3_000.0, 1_000.0, 1_000.0, 0.0), 0.0);
        assert_eq!(avoid_pull(3_000.0, 3_000.0, 3_000.0, 0.0), 1.0);
        assert!(avoid_pull(3_000.0, 2_980.0, 2_980.0, 0.0) > 0.9);
        // A descending jet pulls earlier than a level one.
        let level = avoid_pull(1_300.0, 1_000.0, 1_000.0, 0.0);
        let diving = avoid_pull(1_300.0, 1_000.0, 1_000.0, -60.0);
        assert!(diving > level, "{diving} vs {level}");
        // A ridge ahead pulls even with sea directly below.
        let ahead = avoid_pull(1_300.0, 0.0, 1_250.0, 0.0);
        assert!(ahead > 0.0);
        // Open water clears against the sea surface, not the seabed: 525 m
        // above the waves is free even with the bottom 55 m down, but 300 m is
        // already below the 500 m minimum and pulls.
        assert_eq!(avoid_pull(525.0, -55.0, -50.0, 0.0), 0.0);
        assert!(avoid_pull(300.0, -55.0, -50.0, 0.0) > 0.1);
    }

    #[test]
    fn arena_turns_the_jet_back_toward_the_centre() {
        // Inside the arena: no intervention.
        assert_eq!(arena_aileron(0.0, 0.0, 12.0), 0.0);
        // Beyond the north edge, pointing north (away): bank to reverse.
        assert!(arena_aileron(0.0, -45_000.0, 0.0).abs() > 0.0);
        // Same spot but already heading south (home): no turn.
        assert_eq!(arena_aileron(0.0, -45_000.0, 180.0), 0.0);
        // Beyond the east edge, heading east (away): bank to reverse.
        assert!(arena_aileron(45_000.0, 0.0, 90.0).abs() > 0.0);
        // At the edge with a perpendicular heading, the turn is capped hard.
        assert!(arena_aileron(-48_000.0, 0.0, 0.0) <= 1.0);
    }

    #[test]
    fn map_labels_round_trip() {
        for map in MAPS {
            assert_eq!(MapId::from_label(map.label()), Some(map));
        }
        assert_eq!(MapId::from_label("nowhere"), None);
    }

    #[test]
    fn detail_noise_tiles_without_a_seam() {
        for edge in [0.0f32, 1.0] {
            for t in 0..32 {
                let s = t as f32 / 32.0;
                let a = noise_fbm(edge, s);
                let b = noise_fbm(edge + 1.0, s);
                assert!((a - b).abs() < 1e-5, "seam at {edge}: {a} vs {b}");
            }
        }
    }
}
