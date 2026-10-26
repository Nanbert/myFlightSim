//! Fixed, depth-tested cloud cards.
//!
//! Clouds used to be raymarched into a texture and composited onto the sky
//! dome. That made them a background layer: looking down from above the deck,
//! the terrain (closer to the camera) depth-rejected the whole cloud layer.
//! This module places a small set of translucent cards in world space instead.
//! They are ordinary transparent geometry, so terrain and aircraft occlude
//! them correctly from every direction, and each map owns a fixed layout.

use bevy::asset::RenderAssetUsages;
use bevy::image::{ImageAddressMode, ImageFilterMode, ImageSampler, ImageSamplerDescriptor};
use bevy::prelude::*;
use bevy::render::render_resource::{
    AsBindGroup, Extent3d, ShaderType, TextureDimension, TextureFormat,
};
use bevy::shader::ShaderRef;

use crate::app_state::AppState;
use crate::map::{ActiveMap, MapId};
use crate::weather::Weather;

/// Card texture resolution.
const CLOUD_TEX: u32 = 256;
/// Cards per cluster. Enough overlapping puffs to read as a cloud, few enough
/// that the transparent queue stays cheap.
const CARDS_PER_CLUSTER: usize = 5;

/// One map's fixed cloud layout.
#[derive(Clone, Copy)]
struct CloudPreset {
    seed: u32,
    /// Grid cells per axis across the 96 km arena.
    cells: i32,
    /// Fraction of cells that receive a cloud cluster.
    coverage: f32,
    base_y: f32,
    top_y: f32,
    tint: Vec3,
    opacity: f32,
}

fn preset(map: Option<MapId>) -> CloudPreset {
    match map {
        Some(MapId::Alpine) => CloudPreset {
            seed: 0xA1F1_0002,
            cells: 6,
            coverage: 0.90,
            base_y: 2_600.0,
            top_y: 6_800.0,
            tint: Vec3::new(0.88, 0.92, 0.98),
            opacity: 0.62,
        },
        Some(MapId::Glacial) => CloudPreset {
            seed: 0xA1F1_0003,
            cells: 6,
            coverage: 0.75,
            base_y: 3_200.0,
            top_y: 7_200.0,
            tint: Vec3::new(0.82, 0.88, 0.96),
            opacity: 0.46,
        },
        // Archipelago, free flight without a pending mission, and the menu.
        _ => CloudPreset {
            seed: 0xA1F1_0001,
            cells: 6,
            coverage: 0.88,
            base_y: 1_800.0,
            top_y: 5_400.0,
            tint: Vec3::new(0.93, 0.96, 1.0),
            opacity: 0.58,
        },
    }
}

fn menu_preset() -> CloudPreset {
    CloudPreset {
        seed: 0xA1F1_00FF,
        cells: 2,
        coverage: 0.8,
        base_y: 1_400.0,
        top_y: 4_200.0,
        tint: Vec3::new(0.90, 0.94, 1.0),
        opacity: 0.30,
    }
}

#[derive(Clone, Copy, ShaderType)]
struct CloudCardParams {
    /// xyz = camera position, w = time (s).
    camera: Vec4,
    /// xyz = direction toward the sun, w = rain (0..1).
    sun: Vec4,
    /// rgb = cloud tint, a = card opacity.
    tint: Vec4,
    /// x = per-card seed, y = night, z = time (s), w = horizontal top card.
    anim: Vec4,
    /// rgb = haze colour, w = haze density.
    fog: Vec4,
}

#[derive(Asset, TypePath, AsBindGroup, Clone)]
struct CloudCardMaterial {
    #[uniform(0)]
    params: CloudCardParams,
    #[texture(1)]
    #[sampler(2)]
    #[dependency]
    texture: Handle<Image>,
}

impl Material for CloudCardMaterial {
    fn fragment_shader() -> ShaderRef {
        "shaders/cloud_card.wesl".into()
    }

    fn alpha_mode(&self) -> AlphaMode {
        AlphaMode::Blend
    }

    fn enable_prepass() -> bool {
        false
    }

    fn enable_shadows() -> bool {
        false
    }

    /// Camera-facing cards can present either winding depending on the view;
    /// render both sides so a card never disappears when the camera crosses
    /// its plane.
    fn specialize(
        _pipeline: &bevy::pbr::MaterialPipeline,
        descriptor: &mut bevy::render::render_resource::RenderPipelineDescriptor,
        _layout: &bevy::mesh::MeshVertexBufferLayoutRef,
        _key: bevy::pbr::MaterialPipelineKey<Self>,
    ) -> Result<(), bevy::render::render_resource::SpecializedMeshPipelineError> {
        descriptor.primitive.cull_mode = None;
        Ok(())
    }
}

#[derive(Component)]
struct CloudCard {
    base_opacity: f32,
}

/// A horizontal cloud-top card. Vertical cards flatten into a white sheet when
/// seen from above; these carry the deck's top surface instead.
#[derive(Component)]
struct CloudTop {
    base_opacity: f32,
}

#[derive(Component)]
struct CloudFogOverlay;

#[derive(Clone, Copy)]
struct CloudFogVolume {
    center: Vec3,
    radii: Vec3,
}

#[derive(Resource, Default)]
struct ActiveCloudFog {
    volumes: Vec<CloudFogVolume>,
    strength: f32,
    /// Deck bounds for the current map, used to swap vertical cards for the
    /// horizontal top layer as the camera crosses the cloud top.
    base_y: f32,
    top_y: f32,
}

#[derive(Resource)]
struct CloudAssets {
    texture: Handle<Image>,
    quad: Handle<Mesh>,
}

pub struct CloudPlugin;

impl Plugin for CloudPlugin {
    fn build(&self, app: &mut App) {
        app.add_plugins(MaterialPlugin::<CloudCardMaterial>::default())
            .init_resource::<ActiveCloudFog>()
            .add_systems(Startup, setup_cloud_assets)
            .add_systems(
                OnEnter(AppState::Flying),
                (spawn_fog_overlay, spawn_flight_clouds.after(crate::map::EnterMissionSet)),
            )
            .add_systems(OnEnter(AppState::Menu), spawn_menu_clouds)
            .add_systems(OnExit(AppState::Flying), clear_flight_fog)
            .add_systems(
                Update,
                (billboard_clouds, update_cloud_card_params, update_cloud_fog),
            );
    }
}

fn setup_cloud_assets(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut images: ResMut<Assets<Image>>,
) {
    commands.insert_resource(CloudAssets {
        texture: images.add(cloud_texture()),
        quad: meshes.add(Rectangle::new(2.0, 2.0)),
    });
}

fn spawn_flight_clouds(
    mut commands: Commands,
    assets: Option<Res<CloudAssets>>,
    mut materials: ResMut<Assets<CloudCardMaterial>>,
    active: Res<ActiveMap>,
    mut fog: ResMut<ActiveCloudFog>,
) {
    let Some(assets) = assets else {
        return;
    };
    fog.volumes.clear();
    fog.strength = 0.0;
    let preset = preset(active.0);
    fog.base_y = preset.base_y;
    fog.top_y = preset.top_y;
    fog.volumes = spawn_preset(
        &mut commands,
        &assets,
        &mut materials,
        preset,
        AppState::Flying,
    );
}

fn spawn_menu_clouds(
    mut commands: Commands,
    assets: Option<Res<CloudAssets>>,
    mut materials: ResMut<Assets<CloudCardMaterial>>,
    mut fog: ResMut<ActiveCloudFog>,
) {
    let Some(assets) = assets else {
        return;
    };
    let preset = menu_preset();
    fog.base_y = preset.base_y;
    fog.top_y = preset.top_y;
    fog.volumes = spawn_preset(&mut commands, &assets, &mut materials, preset, AppState::Menu);
}

fn spawn_preset(
    commands: &mut Commands,
    assets: &CloudAssets,
    materials: &mut Assets<CloudCardMaterial>,
    preset: CloudPreset,
    state: AppState,
) -> Vec<CloudFogVolume> {
    let mut volumes = Vec::new();
    let mut card_count = 0usize;
    let cells = preset.cells.max(1);
    let cell = (crate::map::ARENA_HALF * 2.0) / cells as f32;

    for gz in 0..cells {
        for gx in 0..cells {
            let pick = hash01(preset.seed, gx, gz);
            if pick > preset.coverage {
                continue;
            }
            let hx = hash01(preset.seed ^ 0x51ED_2701, gx, gz);
            let hz = hash01(preset.seed ^ 0x9E37_79B1, gx, gz);
            let hy = hash01(preset.seed ^ 0x85EB_CA77, gx, gz);
            let radius = cell * (0.24 + 0.10 * hx);
            let center = Vec3::new(
                -crate::map::ARENA_HALF + (gx as f32 + 0.5 + (hx - 0.5) * 0.45) * cell,
                preset.base_y + (preset.top_y - preset.base_y) * (0.30 + 0.45 * hy),
                -crate::map::ARENA_HALF + (gz as f32 + 0.5 + (hz - 0.5) * 0.45) * cell,
            );
            volumes.push(CloudFogVolume {
                center,
                radii: Vec3::new(radius * 1.8, (preset.top_y - preset.base_y) * 0.42, radius * 1.8),
            });

            for i in 0..CARDS_PER_CLUSTER {
                let s = i as u32;
                // Hash mixes are wrapping by design; plain `*` overflows u32
                // in debug builds for the second card onward.
                let a = hash01(
                    preset.seed ^ 0x1234_5678u32.wrapping_add(s.wrapping_mul(0x9E37_79B9)),
                    gx,
                    gz,
                );
                let b = hash01(
                    preset.seed ^ 0x0BAD_F00Du32.wrapping_add(s.wrapping_mul(0x85EB_CA77)),
                    gx,
                    gz,
                );
                let c = hash01(
                    preset.seed ^ 0xFEED_5678u32.wrapping_add(s.wrapping_mul(0xC2B2_AE3D)),
                    gx,
                    gz,
                );
                let d = hash01(
                    preset.seed ^ 0xA5A5_5A5Au32.wrapping_add(s.wrapping_mul(0x27D4_EB2D)),
                    gx,
                    gz,
                );
                let offset = Vec3::new(
                    (a - 0.5) * radius * 1.4,
                    (b - 0.5) * (preset.top_y - preset.base_y) * 0.34,
                    (c - 0.5) * radius * 1.4,
                );
                let layer_height = (preset.top_y - preset.base_y).max(1.0);
                let height = (1_600.0 + a * 3_400.0).min(layer_height);
                // Keep the whole card inside the deck. Cards whose bottom used
                // to hang below cloud base were depth-tested against the sea and
                // read as a white film lying on the water from above.
                let y = (center.y + offset.y)
                    .clamp(preset.base_y + height * 0.5, preset.top_y - height * 0.5);
                let pos = Vec3::new(center.x + offset.x, y, center.z + offset.z);
                let size = Vec2::new(2_600.0 + d * 5_800.0, height);
                let opacity = preset.opacity * (0.55 + 0.45 * b);
                let material = materials.add(CloudCardMaterial {
                    params: CloudCardParams {
                        camera: Vec4::ZERO,
                        sun: Vec4::new(0.0, 1.0, 0.0, 0.0),
                        tint: Vec4::new(preset.tint.x, preset.tint.y, preset.tint.z, opacity),
                        anim: Vec4::new(d, 0.0, 0.0, 0.0),
                        fog: Vec4::new(0.55, 0.70, 0.87, 0.000018),
                    },
                    texture: assets.texture.clone(),
                });
                commands.spawn((
                    CloudCard {
                        base_opacity: opacity,
                    },
                    Mesh3d(assets.quad.clone()),
                    MeshMaterial3d(material),
                    Transform::from_translation(pos)
                        .with_scale(Vec3::new(size.x, size.y, 1.0)),
                    DespawnOnExit(state),
                ));
                card_count += 1;
            }
        }
    }

    // A second, horizontal layer carries the deck's top surface. Without it the
    // only thing visible from above is the vertical cards seen edge-on: a flat
    // white sheet over the sea and land.
    let mut top_count = 0usize;
    for gz in 0..cells {
        for gx in 0..cells {
            // The top deck is deliberately sparser than the vertical body: it
            // should read as cloud masses with sea/land gaps, not an overcast
            // white sheet.
            if hash01(preset.seed ^ 0x7011_0000, gx, gz) > preset.coverage * 0.82 {
                continue;
            }
            for i in 0..2 {
                let s = i as u32;
                let a = hash01(
                    preset.seed ^ 0x7011_0001u32.wrapping_add(s.wrapping_mul(0x9E37_79B9)),
                    gx,
                    gz,
                );
                let b = hash01(
                    preset.seed ^ 0x7011_0002u32.wrapping_add(s.wrapping_mul(0x85EB_CA77)),
                    gx,
                    gz,
                );
                let c = hash01(
                    preset.seed ^ 0x7011_0003u32.wrapping_add(s.wrapping_mul(0xC2B2_AE3D)),
                    gx,
                    gz,
                );
                let d = hash01(
                    preset.seed ^ 0x7011_0004u32.wrapping_add(s.wrapping_mul(0x27D4_EB2D)),
                    gx,
                    gz,
                );
                let size = cell * (0.58 + 0.22 * a);
                let x = -crate::map::ARENA_HALF + (gx as f32 + a) * cell;
                let z = -crate::map::ARENA_HALF + (gz as f32 + b) * cell;
                let y = preset.top_y - 80.0 + c * 220.0;
                let rotation = Quat::from_rotation_y(d * std::f32::consts::TAU)
                    * Quat::from_rotation_x(-std::f32::consts::FRAC_PI_2);
                let opacity = preset.opacity * (0.26 + 0.16 * b);
                let tint = preset.tint * 0.88;
                let material = materials.add(CloudCardMaterial {
                    params: CloudCardParams {
                        camera: Vec4::ZERO,
                        sun: Vec4::new(0.0, 1.0, 0.0, 0.0),
                        tint: Vec4::new(tint.x, tint.y, tint.z, opacity),
                        // w = 1 marks a horizontal top card for the shader's
                        // near-fade and flat-sheet shading branch.
                        anim: Vec4::new(d, 0.0, 0.0, 1.0),
                        fog: Vec4::new(0.55, 0.70, 0.87, 0.000018),
                    },
                    texture: assets.texture.clone(),
                });
                commands.spawn((
                    CloudTop {
                        base_opacity: opacity,
                    },
                    Mesh3d(assets.quad.clone()),
                    MeshMaterial3d(material),
                    Transform::from_translation(Vec3::new(x, y, z))
                        .with_rotation(rotation)
                        .with_scale(Vec3::new(size, size, 1.0)),
                    DespawnOnExit(state),
                ));
                top_count += 1;
            }
        }
    }
    info!(
        "cloud preset spawned {card_count} cards, {top_count} top cards, {} fog volumes",
        volumes.len()
    );
    volumes
}

fn spawn_fog_overlay(mut commands: Commands) {
    commands.spawn((
        CloudFogOverlay,
        Node {
            position_type: PositionType::Absolute,
            width: Val::Percent(100.0),
            height: Val::Percent(100.0),
            ..default()
        },
        BackgroundColor(Color::srgba(0.82, 0.86, 0.90, 0.0)),
        // HUD lives at GlobalZIndex 20; keep the veil below it.
        GlobalZIndex(1),
        DespawnOnExit(AppState::Flying),
    ));
}

fn clear_flight_fog(mut fog: ResMut<ActiveCloudFog>) {
    fog.volumes.clear();
    fog.strength = 0.0;
    fog.base_y = 0.0;
    fog.top_y = 0.0;
}

fn billboard_clouds(
    camera: Query<&GlobalTransform, (With<Camera3d>, Without<crate::cloud_camera::CloudCamera>)>,
    mut cards: Query<&mut Transform, With<CloudCard>>,
) {
    let Ok(cam) = camera.single() else {
        return;
    };
    let rotation = cam.rotation();
    for mut t in &mut cards {
        t.rotation = rotation;
    }
}

fn update_cloud_card_params(
    time: Res<Time>,
    weather: Res<Weather>,
    deck: Res<ActiveCloudFog>,
    camera: Query<&GlobalTransform, (With<Camera3d>, Without<crate::cloud_camera::CloudCamera>)>,
    sun: Query<&GlobalTransform, With<DirectionalLight>>,
    cards: Query<(&MeshMaterial3d<CloudCardMaterial>, &CloudCard)>,
    tops: Query<(&MeshMaterial3d<CloudCardMaterial>, &CloudTop)>,
    mut materials: ResMut<Assets<CloudCardMaterial>>,
) {
    let (Ok(cam), Ok(sun)) = (camera.single(), sun.single()) else {
        return;
    };
    let sun_dir = (sun.rotation() * Vec3::Z).normalize_or_zero();
    let pos = cam.translation();
    let rain = weather.rain;
    let night = weather.night_factor();
    let haze = crate::map::fog_color(rain, night);

    // Vertical cards are the cloud body seen from below/inside. From above they
    // collapse into a flat white sheet, so hand the top view to the horizontal
    // cards. The two crossover bands keep the swap from popping.
    let vertical_visible = 1.0 - smoothstep(deck.top_y - 400.0, deck.top_y + 300.0, pos.y);
    let top_visible = smoothstep(deck.top_y - 900.0, deck.top_y + 100.0, pos.y);

    for (material, card) in &cards {
        if let Some(mut m) = materials.get_mut(&material.0) {
            m.params.camera = Vec4::new(pos.x, pos.y, pos.z, time.elapsed_secs());
            m.params.sun = Vec4::new(sun_dir.x, sun_dir.y, sun_dir.z, rain);
            m.params.tint.w = card.base_opacity * vertical_visible;
            m.params.anim.y = night;
            m.params.anim.z = time.elapsed_secs();
            m.params.fog = Vec4::new(haze[0], haze[1], haze[2], 0.000018);
        }
    }
    for (material, top) in &tops {
        if let Some(mut m) = materials.get_mut(&material.0) {
            m.params.camera = Vec4::new(pos.x, pos.y, pos.z, time.elapsed_secs());
            m.params.sun = Vec4::new(sun_dir.x, sun_dir.y, sun_dir.z, rain);
            m.params.tint.w = top.base_opacity * top_visible;
            m.params.anim.y = night;
            m.params.anim.z = time.elapsed_secs();
            m.params.fog = Vec4::new(haze[0], haze[1], haze[2], 0.000018);
        }
    }
}

fn update_cloud_fog(
    camera: Query<&GlobalTransform, (With<Camera3d>, Without<crate::cloud_camera::CloudCamera>)>,
    mut fog: ResMut<ActiveCloudFog>,
    mut overlay: Query<&mut BackgroundColor, With<CloudFogOverlay>>,
) {
    let Ok(cam) = camera.single() else {
        return;
    };
    let pos = cam.translation();
    let mut density: f32 = 0.0;
    for volume in &fog.volumes {
        let d = ((pos - volume.center) / volume.radii.max(Vec3::splat(1.0))).length();
        density = density.max((1.0 - d).clamp(0.0, 1.0));
    }
    // The veil is an inside-the-cloud effect. Above the deck the horizontal top
    // cards carry the look; a full-screen white overlay there would wash out the
    // sea and terrain.
    let altitude_fade = 1.0 - smoothstep(fog.top_y - 300.0, fog.top_y + 500.0, pos.y);
    let strength = ((density * 1.35).clamp(0.0, 0.82) * altitude_fade).clamp(0.0, 0.82);
    fog.strength = strength;
    for mut bg in &mut overlay {
        bg.0 = Color::srgba(0.82, 0.86, 0.90, strength);
    }
}

fn cloud_texture() -> Image {
    let n = CLOUD_TEX as usize;
    let mut data = Vec::with_capacity(n * n * 4);
    for y in 0..n {
        for x in 0..n {
            let u = x as f32 / n as f32;
            let v = y as f32 / n as f32;
            let p = Vec2::new(u * 2.0 - 1.0, v * 2.0 - 1.0);
            let radial = 1.0 - smoothstep(0.25, 1.0, p.length());
            let nv = fbm2(u * 4.0, v * 4.0, 0xC10D_5EED);
            let a = smoothstep(0.30, 0.72, radial * 0.65 + nv * 0.55) * radial;
            let shade = 220.0 + 35.0 * (0.6 * nv + 0.4 * (1.0 - p.length()));
            data.extend_from_slice(&[
                shade.min(255.0) as u8,
                shade.min(255.0) as u8,
                255,
                (a.clamp(0.0, 1.0) * 255.0) as u8,
            ]);
        }
    }

    let mut image = Image::new(
        Extent3d {
            width: CLOUD_TEX,
            height: CLOUD_TEX,
            depth_or_array_layers: 1,
        },
        TextureDimension::D2,
        data,
        TextureFormat::Rgba8Unorm,
        RenderAssetUsages::RENDER_WORLD,
    );
    image.sampler = ImageSampler::Descriptor(ImageSamplerDescriptor {
        address_mode_u: ImageAddressMode::ClampToEdge,
        address_mode_v: ImageAddressMode::ClampToEdge,
        mag_filter: ImageFilterMode::Linear,
        min_filter: ImageFilterMode::Linear,
        mipmap_filter: ImageFilterMode::Nearest,
        ..default()
    });
    image
}

fn hash01(seed: u32, x: i32, z: i32) -> f32 {
    let mut h = seed
        .wrapping_add((x as u32).wrapping_mul(0x9E37_79B1))
        .wrapping_add((z as u32).wrapping_mul(0x85EB_CA77));
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

fn smoothstep(edge0: f32, edge1: f32, x: f32) -> f32 {
    smooth(((x - edge0) / (edge1 - edge0).max(1.0e-6)).clamp(0.0, 1.0))
}

fn value_noise(x: f32, y: f32, seed: u32) -> f32 {
    let xi = x.floor() as i32;
    let yi = y.floor() as i32;
    let fx = smooth(x - xi as f32);
    let fy = smooth(y - yi as f32);
    let a = hash01(seed, xi, yi);
    let b = hash01(seed, xi + 1, yi);
    let c = hash01(seed, xi, yi + 1);
    let d = hash01(seed, xi + 1, yi + 1);
    let ab = a + (b - a) * fx;
    let cd = c + (d - c) * fx;
    ab + (cd - ab) * fy
}

fn fbm2(x: f32, y: f32, seed: u32) -> f32 {
    let mut sum = 0.0;
    let mut norm = 0.0;
    let mut amp = 0.5;
    let mut p = Vec2::new(x, y);
    for i in 0..4 {
        // Wrapping is part of the hash; the multipliers overflow u32.
        let octave = seed.wrapping_add((i as u32).wrapping_mul(0x9E37_79B9));
        sum += amp * value_noise(p.x, p.y, octave);
        norm += amp;
        amp *= 0.5;
        p = Vec2::new(p.x * 2.03 + 17.1, p.y * 2.03 + 9.2);
    }
    sum / norm
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn cloud_texture_builds_without_arithmetic_overflow() {
        // Debug builds used to panic in `fbm2`: `seed + i * 0x9E37_79B9`
        // overflows u32 from the second octave onward.
        let image = cloud_texture();
        let data = image.data.as_ref().expect("cloud texture data");
        assert_eq!(data.len(), (CLOUD_TEX * CLOUD_TEX * 4) as usize);
    }
}
