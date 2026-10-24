//! Bake the 737 MCP's 3D model to `assets/ui/737_mcp_3d.png`.
//!
//! The runtime panel is 2D: a pre-rendered orthographic image plus UI labels,
//! digits and hotspots (`mcp_layout` owns the shared coordinates). This module
//! is the only place that still builds the 3D meshes. It is compiled only with
//! the `shots` feature and driven by `examples/mcp_bake.rs`.
//!
//! The camera is orthographic and its projection is exactly the plate's
//! 0.55 m x 0.10 m face, so the PNG is a linear screen-to-asset-pixel map. The
//! world is not part of the bake app: three stage lights plus a low warm
//! ambient define the look, and the clear colour matches the runtime backdrop.
//! Digits and labels are not baked; they are live UI overlays.

use std::f32::consts::{FRAC_PI_2, TAU};
use std::path::PathBuf;

use bevy::camera::{Hdr, RenderTarget, ScalingMode};
use bevy::core_pipeline::tonemapping::Tonemapping;
use bevy::post_process::bloom::{Bloom, BloomCompositeMode, BloomPrefilter};
use bevy::prelude::*;
use bevy::render::render_resource::TextureFormat;
use bevy::render::view::window::screenshot::{save_to_disk, Screenshot};

use crate::mcp_layout::{
    PRect, AT_SWITCH, BUTTONS, COURSE_L_KNOB, COURSE_L_WINDOW, COURSE_R_KNOB, COURSE_R_WINDOW,
    CROP_H, CROP_W, DECOR_CAPS, DECOR_ROUND, DISENGAGE, FD_L, FD_R, KNOBS, VS_WHEEL, WINDOWS,
};

/// The baked PNG's size: 2x the 2470x449 layout space.
pub const BAKE_SIZE: (u32, u32) = (4940, 898);

/// The plate, in metres.
const PANEL_W: f32 = 0.55;
const PANEL_H: f32 = PANEL_W * CROP_H / CROP_W;
const PX: f32 = PANEL_W / CROP_W;

/// Matches the runtime backdrop and the old 3D exhibit's backdrop.
const BG: Color = Color::srgb(0.016, 0.015, 0.014);

pub struct McpBakePlugin {
    path: PathBuf,
}

impl McpBakePlugin {
    pub fn new(path: impl Into<PathBuf>) -> Self {
        Self { path: path.into() }
    }
}

#[derive(Resource)]
struct BakeConfig {
    path: PathBuf,
}

#[derive(Resource)]
struct BakeTarget(Handle<Image>);

#[derive(Resource, Default)]
struct BakeState {
    requested: bool,
}

impl Plugin for McpBakePlugin {
    fn build(&self, app: &mut App) {
        app.insert_resource(BakeConfig {
            path: self.path.clone(),
        })
        .insert_resource(ClearColor(BG))
        .init_resource::<BakeState>()
        .add_systems(Startup, spawn_bake_scene)
        .add_systems(Update, drive_bake);
    }
}

/// Photo pixel -> stage-local metres. +Y is up, +Z is out of the panel.
fn local(x: f32, y: f32, z: f32) -> Vec3 {
    Vec3::new(
        (x / CROP_W - 0.5) * PANEL_W,
        (0.5 - y / CROP_H) * PANEL_H,
        z,
    )
}

struct Kit {
    cube: Handle<Mesh>,
    cyl: Handle<Mesh>,
}

struct Mats {
    plate: Handle<StandardMaterial>,
    rim: Handle<StandardMaterial>,
    bezel: Handle<StandardMaterial>,
    cap: Handle<StandardMaterial>,
    cap_functional: Handle<StandardMaterial>,
    knob: Handle<StandardMaterial>,
    knob_body: Handle<StandardMaterial>,
    knob_dark: Handle<StandardMaterial>,
    lever: Handle<StandardMaterial>,
    wheel: Handle<StandardMaterial>,
    glass: Handle<StandardMaterial>,
}

fn spawn_bake_scene(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut images: ResMut<Assets<Image>>,
) {
    let kit = Kit {
        cube: meshes.add(Cuboid::new(1.0, 1.0, 1.0)),
        cyl: meshes.add(Cylinder::new(0.5, 1.0)),
    };
    let mats = Mats {
        plate: materials.add(StandardMaterial {
            base_color: Color::srgb(0.20, 0.13, 0.070),
            metallic: 0.0,
            perceptual_roughness: 0.68,
            ..default()
        }),
        rim: materials.add(StandardMaterial {
            base_color: Color::srgb(0.42, 0.285, 0.125),
            metallic: 0.55,
            perceptual_roughness: 0.38,
            ..default()
        }),
        bezel: materials.add(StandardMaterial {
            base_color: Color::srgb(0.070, 0.048, 0.026),
            metallic: 0.30,
            perceptual_roughness: 0.54,
            ..default()
        }),
        cap: materials.add(StandardMaterial {
            base_color: Color::srgb(0.050, 0.043, 0.034),
            metallic: 0.12,
            perceptual_roughness: 0.68,
            ..default()
        }),
        cap_functional: materials.add(StandardMaterial {
            base_color: Color::srgb(0.042, 0.036, 0.028),
            metallic: 0.2,
            perceptual_roughness: 0.62,
            ..default()
        }),
        knob: materials.add(StandardMaterial {
            base_color: Color::srgb(0.32, 0.21, 0.10),
            metallic: 0.35,
            perceptual_roughness: 0.42,
            ..default()
        }),
        knob_body: materials.add(StandardMaterial {
            base_color: Color::srgb(0.26, 0.19, 0.11),
            metallic: 0.75,
            perceptual_roughness: 0.32,
            ..default()
        }),
        knob_dark: materials.add(StandardMaterial {
            base_color: Color::srgb(0.055, 0.042, 0.028),
            metallic: 0.30,
            perceptual_roughness: 0.52,
            ..default()
        }),
        lever: materials.add(StandardMaterial {
            base_color: Color::srgb(0.34, 0.26, 0.16),
            metallic: 0.8,
            perceptual_roughness: 0.30,
            ..default()
        }),
        wheel: materials.add(StandardMaterial {
            base_color: Color::srgb(0.34, 0.31, 0.27),
            metallic: 0.75,
            perceptual_roughness: 0.34,
            ..default()
        }),
        glass: materials.add(StandardMaterial {
            base_color: Color::srgb(0.010, 0.009, 0.008),
            metallic: 0.0,
            perceptual_roughness: 0.16,
            ..default()
        }),
    };

    commands
        .spawn((Transform::default(), Visibility::Inherited))
        .with_children(|root| build_panel(root, &kit, &mats));

    // The old exhibit's key, fill and warm bounce. The world is absent, so
    // these three plus the ambient are the whole light rig.
    commands.spawn((
        DirectionalLight {
            color: Color::srgb(1.0, 0.88, 0.68),
            illuminance: 8000.0,
            shadow_maps_enabled: false,
            ..default()
        },
        Transform::from_rotation(Quat::from_euler(EulerRot::XYZ, -0.55, -0.55, 0.0)),
    ));
    commands.spawn((
        DirectionalLight {
            color: Color::srgb(1.0, 0.78, 0.52),
            illuminance: 2500.0,
            shadow_maps_enabled: false,
            ..default()
        },
        Transform::from_rotation(Quat::from_euler(EulerRot::XYZ, -0.10, 2.4, 0.0)),
    ));
    commands.spawn((
        PointLight {
            color: Color::srgb(1.0, 0.65, 0.35),
            intensity: 60.0,
            range: 1.2,
            ..default()
        },
        Transform::from_xyz(0.0, -0.18, 0.22),
    ));

    let target = images.add(Image::new_target_texture(
        BAKE_SIZE.0,
        BAKE_SIZE.1,
        TextureFormat::Rgba8UnormSrgb,
        None,
    ));
    commands.insert_resource(BakeTarget(target.clone()));

    // Same pipeline shape as the main camera: HDR, a gentle bloom for the
    // metal's speculars, and no tonemapping so the authored warm palette
    // survives. `Fixed` makes the projection exactly the plate's face, so the
    // PNG is a linear asset-pixel map.
    commands.spawn((
        Camera3d::default(),
        Camera {
            clear_color: ClearColorConfig::Custom(BG),
            ..default()
        },
        RenderTarget::Image(target.into()),
        AmbientLight {
            brightness: 40.0,
            ..default()
        },
        Projection::Orthographic(OrthographicProjection {
            scaling_mode: ScalingMode::Fixed {
                width: PANEL_W,
                height: PANEL_H,
            },
            near: 0.01,
            far: 10.0,
            ..OrthographicProjection::default_3d()
        }),
        Transform::from_xyz(0.0, 0.0, 1.0),
        Tonemapping::None,
        Hdr,
        Bloom {
            intensity: 0.5,
            low_frequency_boost: 0.35,
            high_pass_frequency: 1.0,
            prefilter: BloomPrefilter {
                threshold: 1.5,
                threshold_softness: 0.5,
            },
            composite_mode: BloomCompositeMode::Additive,
            ..Bloom::NATURAL
        },
    ));
}

fn drive_bake(
    time: Res<Time>,
    mut state: ResMut<BakeState>,
    target: Option<Res<BakeTarget>>,
    config: Res<BakeConfig>,
    mut commands: Commands,
    mut exit: MessageWriter<AppExit>,
) {
    let elapsed = time.elapsed_secs();
    if !state.requested && elapsed > 1.0 {
        if let Some(target) = target {
            state.requested = true;
            commands
                .spawn(Screenshot::image(target.0.clone()))
                .observe(save_to_disk(config.path.clone()));
            info!("mcp bake -> {}", config.path.display());
        }
    }
    if elapsed > 3.0 {
        exit.write(AppExit::Success);
    }
}

fn build_panel(stage: &mut ChildSpawnerCommands, kit: &Kit, mats: &Mats) {
    // Plate, rim and corner screws.
    add_box(
        stage,
        kit,
        &mats.plate,
        Vec3::new(0.0, 0.0, -0.008),
        Vec3::new(PANEL_W, PANEL_H, 0.016),
    );
    let t = 0.0035;
    for (cx, cy, sx, sy) in [
        (0.0, PANEL_H * 0.5 - t * 0.5, PANEL_W, t),
        (0.0, -PANEL_H * 0.5 + t * 0.5, PANEL_W, t),
        (-PANEL_W * 0.5 + t * 0.5, 0.0, t, PANEL_H),
        (PANEL_W * 0.5 - t * 0.5, 0.0, t, PANEL_H),
    ] {
        add_box(
            stage,
            kit,
            &mats.rim,
            Vec3::new(cx, cy, 0.0005),
            Vec3::new(sx, sy, 0.004),
        );
    }
    for (x, y) in [
        (40.0, 30.0),
        (CROP_W - 40.0, 30.0),
        (40.0, CROP_H - 30.0),
        (CROP_W - 40.0, CROP_H - 30.0),
        (CROP_W * 0.5, 22.0),
    ] {
        add_cyl(
            stage,
            kit,
            &mats.knob_dark,
            local(x, y, 0.002),
            0.0032,
            0.003,
            Quat::from_rotation_x(FRAC_PI_2),
        );
    }

    for &(rect, _, _) in WINDOWS {
        spawn_window(stage, kit, mats, rect);
    }
    for &(rect, _, _) in BUTTONS {
        spawn_button(stage, kit, mats, rect, true);
    }
    spawn_switch(stage, kit, mats, AT_SWITCH, true);
    for &(rect, _) in KNOBS {
        spawn_knob(stage, kit, mats, rect);
    }
    spawn_wheel(stage, kit, mats);

    for &(rect, _) in DECOR_CAPS {
        spawn_button(stage, kit, mats, rect, false);
    }
    for &(rect, _) in DECOR_ROUND {
        spawn_round(stage, kit, mats, rect);
    }
    spawn_bar(stage, kit, mats, DISENGAGE);
    spawn_switch(stage, kit, mats, FD_L, false);
    spawn_switch(stage, kit, mats, FD_R, false);
    spawn_course_window(stage, kit, mats, COURSE_L_WINDOW);
    spawn_course_window(stage, kit, mats, COURSE_R_WINDOW);
    spawn_decor_knob(stage, kit, mats, COURSE_L_KNOB);
    spawn_decor_knob(stage, kit, mats, COURSE_R_KNOB);
}

fn spawn_window(stage: &mut ChildSpawnerCommands, kit: &Kit, mats: &Mats, rect: PRect) {
    // Recessed bezel: four thin bars around the glass.
    let b = 9.0;
    for r in [
        PRect::new(rect.x0 - b, rect.y0 - b, rect.x1 + b, rect.y0),
        PRect::new(rect.x0 - b, rect.y1, rect.x1 + b, rect.y1 + b),
        PRect::new(rect.x0 - b, rect.y0, rect.x0, rect.y1),
        PRect::new(rect.x1, rect.y0, rect.x1 + b, rect.y1),
    ] {
        add_box(
            stage,
            kit,
            &mats.bezel,
            local(r.cx(), r.cy(), 0.004),
            Vec3::new(r.w() * PX, r.h() * PX, 0.008),
        );
    }
    add_box(
        stage,
        kit,
        &mats.glass,
        local(rect.cx(), rect.cy(), 0.001),
        Vec3::new(rect.w() * PX, rect.h() * PX, 0.003),
    );
}

fn spawn_button(
    stage: &mut ChildSpawnerCommands,
    kit: &Kit,
    mats: &Mats,
    rect: PRect,
    functional: bool,
) {
    add_box(
        stage,
        kit,
        &mats.bezel,
        local(rect.cx(), rect.cy(), 0.0005),
        Vec3::new(rect.w() * PX * 1.04, rect.h() * PX * 1.04, 0.003),
    );
    let mat = if functional {
        &mats.cap_functional
    } else {
        &mats.cap
    };
    stage.spawn((
        Mesh3d(kit.cube.clone()),
        MeshMaterial3d(mat.clone()),
        Transform {
            translation: local(rect.cx(), rect.cy(), 0.0055),
            scale: Vec3::new(rect.w() * PX * 0.94, rect.h() * PX * 0.94, 0.008),
            ..default()
        },
    ));
}

fn spawn_round(stage: &mut ChildSpawnerCommands, kit: &Kit, mats: &Mats, rect: PRect) {
    let r = rect.w().min(rect.h()) * PX * 0.5;
    add_cyl(
        stage,
        kit,
        &mats.bezel,
        local(rect.cx(), rect.cy(), 0.002),
        r * 1.12,
        0.004,
        Quat::from_rotation_x(FRAC_PI_2),
    );
    add_cyl(
        stage,
        kit,
        &mats.cap,
        local(rect.cx(), rect.cy(), 0.005),
        r,
        0.007,
        Quat::from_rotation_x(FRAC_PI_2),
    );
}

fn spawn_bar(stage: &mut ChildSpawnerCommands, kit: &Kit, mats: &Mats, rect: PRect) {
    add_box(
        stage,
        kit,
        &mats.bezel,
        local(rect.cx(), rect.cy(), 0.0015),
        Vec3::new(rect.w() * PX, rect.h() * PX, 0.003),
    );
    add_box(
        stage,
        kit,
        &mats.rim,
        local(rect.cx(), rect.cy() - rect.h() * 0.08, 0.006),
        Vec3::new(rect.w() * PX * 0.92, rect.h() * PX * 0.55, 0.010),
    );
}

fn spawn_switch(
    stage: &mut ChildSpawnerCommands,
    kit: &Kit,
    mats: &Mats,
    rect: PRect,
    functional: bool,
) {
    add_box(
        stage,
        kit,
        &mats.bezel,
        local(rect.cx(), rect.cy(), 0.0015),
        Vec3::new(rect.w() * PX * 0.62, rect.h() * PX * 0.7, 0.004),
    );
    let tilt = if functional { -0.45 } else { 0.45 };
    let len = rect.h() * PX * 0.32;
    let mat = if functional { &mats.lever } else { &mats.knob };
    stage.spawn((
        Mesh3d(kit.cube.clone()),
        MeshMaterial3d(mat.clone()),
        Transform {
            translation: local(rect.cx(), rect.cy(), 0.012),
            rotation: Quat::from_rotation_x(tilt),
            scale: Vec3::new(0.0045, len, 0.0045),
            ..default()
        },
    ));
}

fn spawn_course_window(stage: &mut ChildSpawnerCommands, kit: &Kit, mats: &Mats, rect: PRect) {
    add_box(
        stage,
        kit,
        &mats.bezel,
        local(rect.cx(), rect.cy(), 0.001),
        Vec3::new((rect.w() + 14.0) * PX, (rect.h() + 14.0) * PX, 0.006),
    );
    add_box(
        stage,
        kit,
        &mats.glass,
        local(rect.cx(), rect.cy(), 0.003),
        Vec3::new(rect.w() * PX, rect.h() * PX, 0.002),
    );
}

fn spawn_knob(stage: &mut ChildSpawnerCommands, kit: &Kit, mats: &Mats, rect: PRect) {
    let r = rect.w().min(rect.h()) * PX * 0.5 * 0.92;
    let centre = local(rect.cx(), rect.cy(), 0.0);
    add_cyl(
        stage,
        kit,
        &mats.bezel,
        centre + Vec3::new(0.0, 0.0, 0.002),
        r * 1.12,
        0.004,
        Quat::from_rotation_x(FRAC_PI_2),
    );
    stage.spawn((
        Mesh3d(kit.cyl.clone()),
        MeshMaterial3d(mats.knob_body.clone()),
        Transform {
            translation: centre + Vec3::new(0.0, 0.0, 0.010),
            rotation: Quat::from_rotation_x(FRAC_PI_2),
            scale: Vec3::new(r * 2.0, 0.016, r * 2.0),
            ..default()
        },
    ));
    for i in 0..28 {
        let a = i as f32 / 28.0 * TAU;
        stage.spawn((
            Mesh3d(kit.cube.clone()),
            MeshMaterial3d(mats.knob.clone()),
            Transform {
                translation: centre + Vec3::new(r * 0.93 * a.cos(), r * 0.93 * a.sin(), 0.010),
                scale: Vec3::new(0.0014, 0.0014, 0.006),
                ..default()
            },
        ));
    }
    stage.spawn((
        Mesh3d(kit.cube.clone()),
        MeshMaterial3d(mats.knob_dark.clone()),
        Transform {
            translation: centre + Vec3::new(0.0, r * 0.30, 0.0185),
            scale: Vec3::new(0.0022, r * 0.55, 0.0012),
            ..default()
        },
    ));
}

fn spawn_decor_knob(stage: &mut ChildSpawnerCommands, kit: &Kit, mats: &Mats, rect: PRect) {
    let r = rect.w().min(rect.h()) * PX * 0.5 * 0.92;
    let centre = local(rect.cx(), rect.cy(), 0.0);
    add_cyl(
        stage,
        kit,
        &mats.bezel,
        centre + Vec3::new(0.0, 0.0, 0.002),
        r * 1.12,
        0.004,
        Quat::from_rotation_x(FRAC_PI_2),
    );
    add_cyl(
        stage,
        kit,
        &mats.knob,
        centre + Vec3::new(0.0, 0.0, 0.010),
        r,
        0.016,
        Quat::from_rotation_x(FRAC_PI_2),
    );
    for i in 0..24 {
        let a = i as f32 / 24.0 * TAU;
        stage.spawn((
            Mesh3d(kit.cube.clone()),
            MeshMaterial3d(mats.knob.clone()),
            Transform {
                translation: centre + Vec3::new(r * 0.93 * a.cos(), r * 0.93 * a.sin(), 0.010),
                scale: Vec3::new(0.0014, 0.0014, 0.006),
                ..default()
            },
        ));
    }
}

fn spawn_wheel(stage: &mut ChildSpawnerCommands, kit: &Kit, mats: &Mats) {
    let rect = VS_WHEEL;
    let centre = local(rect.cx(), rect.cy(), 0.0);
    add_box(
        stage,
        kit,
        &mats.bezel,
        centre + Vec3::new(0.0, 0.0, 0.001),
        Vec3::new(rect.w() * PX * 1.05, rect.h() * PX, 0.004),
    );
    let r = rect.w() * PX * 0.48;
    stage.spawn((
        Mesh3d(kit.cyl.clone()),
        MeshMaterial3d(mats.wheel.clone()),
        Transform {
            translation: centre + Vec3::new(0.0, 0.0, 0.010),
            rotation: Quat::from_rotation_z(-FRAC_PI_2),
            scale: Vec3::new(r * 2.0, 0.011, r * 2.0),
            ..default()
        },
    ));
    for i in 0..22 {
        let a = i as f32 / 22.0 * TAU;
        stage.spawn((
            Mesh3d(kit.cube.clone()),
            MeshMaterial3d(mats.knob.clone()),
            Transform {
                translation: centre
                    + Vec3::new(0.0, r * 0.94 * a.cos(), 0.010 + r * 0.94 * a.sin()),
                scale: Vec3::new(0.010, 0.0012, 0.0012),
                ..default()
            },
        ));
    }
}

fn add_box(
    parent: &mut ChildSpawnerCommands,
    kit: &Kit,
    mat: &Handle<StandardMaterial>,
    centre: Vec3,
    size: Vec3,
) {
    parent.spawn((
        Mesh3d(kit.cube.clone()),
        MeshMaterial3d(mat.clone()),
        Transform {
            translation: centre,
            scale: size,
            ..default()
        },
    ));
}

fn add_cyl(
    parent: &mut ChildSpawnerCommands,
    kit: &Kit,
    mat: &Handle<StandardMaterial>,
    centre: Vec3,
    radius: f32,
    height: f32,
    rot: Quat,
) {
    parent.spawn((
        Mesh3d(kit.cyl.clone()),
        MeshMaterial3d(mat.clone()),
        Transform {
            translation: centre,
            rotation: rot,
            scale: Vec3::new(radius * 2.0, height, radius * 2.0),
            ..default()
        },
    ));
}
