//! Bake the 737 control stand to `assets/ui/737_pedestal_plate.png`.
//!
//! The runtime strip is 2D: this pre-rendered orthographic plate plus live
//! levers, lights and text (`pedestal_ui`). Only the fixed hardware — plate,
//! gate frames, slots, indicator well, lamp bezels — is baked. The handles are
//! drawn at runtime because they move.
//!
//! Compiled only with the `shots` feature and driven by
//! `examples/pedestal_bake.rs`, like `mcp_bake`.

use std::f32::consts::FRAC_PI_2;
use std::path::PathBuf;

use bevy::camera::{Hdr, RenderTarget, ScalingMode};
use bevy::core_pipeline::tonemapping::Tonemapping;
use bevy::post_process::bloom::{Bloom, BloomCompositeMode, BloomPrefilter};
use bevy::prelude::*;
use bevy::render::render_resource::TextureFormat;
use bevy::render::view::window::screenshot::{save_to_disk, Screenshot};

use crate::panel_geom::PRect;
use crate::pedestal_layout::{
    lever, Lever, LeverId, LightSpec, CROP_H, CROP_W, FLAP_INDICATOR, FLAP_STOPS, LEVERS, LIGHTS,
    N2_TRACKS, REVERSE_BAR,
};

/// The baked PNG's size: 2x the 2470x540 layout space.
pub const BAKE_SIZE: (u32, u32) = (4940, 1080);

/// The plate, in metres. Same physical width as the MCP so the two strips read
/// as one console.
const PANEL_W: f32 = 0.55;
const PANEL_H: f32 = PANEL_W * CROP_H / CROP_W;
const PX: f32 = PANEL_W / CROP_W;

/// Matches the runtime backdrop and the MCP bake.
const BG: Color = Color::srgb(0.016, 0.015, 0.014);

pub struct PedestalBakePlugin {
    path: PathBuf,
}

impl PedestalBakePlugin {
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

impl Plugin for PedestalBakePlugin {
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

/// `(centre, size)` for a PRect, in metres.
fn rect_box(r: PRect, z: f32, depth: f32) -> (Vec3, Vec3) {
    (
        local(r.cx(), r.cy(), z),
        Vec3::new(r.w() * PX, r.h() * PX, depth),
    )
}

struct Kit {
    cube: Handle<Mesh>,
    cyl: Handle<Mesh>,
}

struct Mats {
    /// The main grey plate.
    plate: Handle<StandardMaterial>,
    /// Raised frames and housings, a touch lighter than the plate.
    panel: Handle<StandardMaterial>,
    /// The bright machined rim.
    rim: Handle<StandardMaterial>,
    /// A non-metallic highlight for painted chamfers and scale ticks.
    edge: Handle<StandardMaterial>,
    /// Recessed bezels and slot walls.
    bezel: Handle<StandardMaterial>,
    /// Slot glass / lamp glass: almost black.
    glass: Handle<StandardMaterial>,
    /// Screw heads.
    screw: Handle<StandardMaterial>,
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
            base_color: Color::srgb(0.158, 0.157, 0.150),
            metallic: 0.0,
            perceptual_roughness: 0.74,
            ..default()
        }),
        panel: materials.add(StandardMaterial {
            base_color: Color::srgb(0.240, 0.238, 0.228),
            metallic: 0.18,
            perceptual_roughness: 0.58,
            ..default()
        }),
        rim: materials.add(StandardMaterial {
            base_color: Color::srgb(0.42, 0.405, 0.375),
            metallic: 0.72,
            perceptual_roughness: 0.30,
            ..default()
        }),
        edge: materials.add(StandardMaterial {
            base_color: Color::srgb(0.50, 0.49, 0.46),
            metallic: 0.0,
            perceptual_roughness: 0.42,
            ..default()
        }),
        bezel: materials.add(StandardMaterial {
            base_color: Color::srgb(0.045, 0.043, 0.040),
            metallic: 0.30,
            perceptual_roughness: 0.52,
            ..default()
        }),
        glass: materials.add(StandardMaterial {
            // Not pure black: a slot that reads as a hole loses the hardware.
            base_color: Color::srgb(0.030, 0.031, 0.034),
            metallic: 0.0,
            perceptual_roughness: 0.22,
            ..default()
        }),
        screw: materials.add(StandardMaterial {
            base_color: Color::srgb(0.30, 0.29, 0.27),
            metallic: 0.85,
            perceptual_roughness: 0.28,
            ..default()
        }),
    };

    commands
        .spawn((Transform::default(), Visibility::Inherited))
        .with_children(|root| build_stand(root, &kit, &mats));

    // The MCP bake's key/fill/bounce, warmed a little less: the stand is grey
    // metal, not the brown overhead panel.
    commands.spawn((
        DirectionalLight {
            color: Color::srgb(1.0, 0.96, 0.90),
            illuminance: 11000.0,
            shadow_maps_enabled: false,
            ..default()
        },
        Transform::from_rotation(Quat::from_euler(EulerRot::XYZ, -0.55, -0.55, 0.0)),
    ));
    commands.spawn((
        DirectionalLight {
            color: Color::srgb(0.86, 0.90, 1.0),
            illuminance: 2200.0,
            shadow_maps_enabled: false,
            ..default()
        },
        Transform::from_rotation(Quat::from_euler(EulerRot::XYZ, -0.10, 2.4, 0.0)),
    ));
    commands.spawn((
        PointLight {
            color: Color::srgb(1.0, 0.86, 0.70),
            intensity: 60.0,
            range: 1.1,
            ..default()
        },
        Transform::from_xyz(0.0, -0.16, 0.22),
    ));

    let target = images.add(Image::new_target_texture(
        BAKE_SIZE.0,
        BAKE_SIZE.1,
        TextureFormat::Rgba8UnormSrgb,
        None,
    ));
    commands.insert_resource(BakeTarget(target.clone()));

    commands.spawn((
        Camera3d::default(),
        Camera {
            clear_color: ClearColorConfig::Custom(BG),
            ..default()
        },
        RenderTarget::Image(target.into()),
        AmbientLight {
            brightness: 22.0,
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
            intensity: 0.45,
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
            info!("pedestal bake -> {}", config.path.display());
        }
    }
    if elapsed > 3.0 {
        exit.write(AppExit::Success);
    }
}

fn build_stand(stage: &mut ChildSpawnerCommands, kit: &Kit, mats: &Mats) {
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

    // A raised back rail and front lip, so the strip reads as a unit sitting
    // over the MCP rather than a picture pasted on the screen.
    for (y0, y1) in [(0.0, 15.0), (CROP_H - 15.0, CROP_H)] {
        let (c, s) = rect_box(PRect::new(0.0, y0, CROP_W, y1), 0.004, 0.012);
        add_box(stage, kit, &mats.panel, c, s);
    }
    for y in [15.0, CROP_H - 18.0] {
        let (c, s) = rect_box(PRect::new(0.0, y, CROP_W, y + 3.0), 0.009, 0.004);
        add_box(stage, kit, &mats.rim, c, s);
    }

    // Group dividers: shallow grooves between the stand's four clusters.
    for x in [480.0, 880.0, 1620.0, 2130.0] {
        let (c, s) = rect_box(PRect::new(x, 26.0, x + 3.0, CROP_H - 30.0), 0.0005, 0.002);
        add_box(stage, kit, &mats.bezel, c, s);
    }

    for x in [30.0, CROP_W - 30.0] {
        for y in [30.0, CROP_H - 30.0] {
            add_cyl(
                stage,
                kit,
                &mats.screw,
                local(x, y, 0.0015),
                0.0032,
                0.003,
                Quat::from_rotation_x(FRAC_PI_2),
            );
        }
    }
    add_cyl(
        stage,
        kit,
        &mats.screw,
        local(CROP_W * 0.5, 30.0, 0.0015),
        0.0032,
        0.003,
        Quat::from_rotation_x(FRAC_PI_2),
    );

    for l in LEVERS {
        spawn_gate(stage, kit, mats, l);
    }
    spawn_flap_indicator(stage, kit, mats);
    for (_, track) in N2_TRACKS {
        spawn_n2(stage, kit, mats, *track);
    }
    for spec in LIGHTS {
        spawn_lamp(stage, kit, mats, spec);
    }
    spawn_reverse(stage, kit, mats);
}

/// A lever gate: a raised frame around a dark slot. The flap gate also gets a
/// notch at each of its nine stops and a metal tick pointing at the label.
fn spawn_gate(stage: &mut ChildSpawnerCommands, kit: &Kit, mats: &Mats, l: &Lever) {
    spawn_frame(stage, kit, mats, l.channel, 10.0, 0.003, 0.006, &mats.panel);
    let (c, s) = rect_box(l.channel, -0.0005, 0.002);
    add_box(stage, kit, &mats.glass, c, s);
    spawn_slot_edges(stage, kit, mats, l.channel);

    if l.id == LeverId::Flap {
        for (_, norm) in FLAP_STOPS {
            let y = l.y(*norm);
            // Detents are cut into the gate walls, not across the opening: a
            // full-width bar would read as a ladder and hide the lever stem.
            for notch in [
                PRect::new(l.channel.x0 - 5.0, y - 2.5, l.channel.x0 + 11.0, y + 2.5),
                PRect::new(l.channel.x1 - 11.0, y - 2.5, l.channel.x1 + 5.0, y + 2.5),
            ] {
                let (c, s) = rect_box(notch, 0.0015, 0.002);
                add_box(stage, kit, &mats.bezel, c, s);
            }
            // The pointer from the stop's caption to the gate.
            let tick = PRect::new(l.channel.x0 - 26.0, y - 2.0, l.channel.x0 - 12.0, y + 2.0);
            let (c, s) = rect_box(tick, 0.0025, 0.003);
            add_box(stage, kit, &mats.edge, c, s);
        }
    }
}

/// Lit near wall and shadowed far wall, so a black slot reads as a recess.
fn spawn_slot_edges(stage: &mut ChildSpawnerCommands, kit: &Kit, mats: &Mats, rect: PRect) {
    for (r, mat) in [
        (
            PRect::new(rect.x0, rect.y0, rect.x0 + 3.0, rect.y1),
            &mats.edge,
        ),
        (
            PRect::new(rect.x0, rect.y0, rect.x1, rect.y0 + 3.0),
            &mats.edge,
        ),
        (
            PRect::new(rect.x1 - 3.0, rect.y0, rect.x1, rect.y1),
            &mats.bezel,
        ),
    ] {
        let (c, s) = rect_box(r, 0.0002, 0.003);
        add_box(stage, kit, mat, c, s);
    }
}

/// The actual-flap indicator well, with a tick on its right bezel at each stop.
fn spawn_flap_indicator(stage: &mut ChildSpawnerCommands, kit: &Kit, mats: &Mats) {
    spawn_frame(stage, kit, mats, FLAP_INDICATOR, 9.0, 0.002, 0.007, &mats.panel);
    let (c, s) = rect_box(FLAP_INDICATOR, -0.0005, 0.002);
    add_box(stage, kit, &mats.glass, c, s);
    let flap = lever(LeverId::Flap);
    for (_, norm) in FLAP_STOPS {
        let y = flap.y(*norm);
        let tick = PRect::new(
            FLAP_INDICATOR.x1,
            y - 2.5,
            FLAP_INDICATOR.x1 + 9.0,
            y + 2.5,
        );
        let (c, s) = rect_box(tick, 0.0065, 0.002);
        add_box(stage, kit, &mats.edge, c, s);
    }
}

/// An N2 bar: a thin recessed track with reference ticks at idle and cruise.
fn spawn_n2(stage: &mut ChildSpawnerCommands, kit: &Kit, mats: &Mats, track: PRect) {
    spawn_frame(stage, kit, mats, track, 6.0, 0.0015, 0.005, &mats.panel);
    let (c, s) = rect_box(track, -0.0005, 0.002);
    add_box(stage, kit, &mats.glass, c, s);
    for frac in [0.60, 0.80] {
        let y = track.y1 - frac * track.h();
        let tick = PRect::new(track.x0, y - 2.0, track.x1, y + 2.0);
        let (c, s) = rect_box(tick, 0.0012, 0.0015);
        add_box(stage, kit, &mats.edge, c, s);
    }
}

/// A lamp: raised bezel and a dark glass lens. The live layer paints the lens.
fn spawn_lamp(stage: &mut ChildSpawnerCommands, kit: &Kit, mats: &Mats, spec: &LightSpec) {
    let r = spec.lens.w().min(spec.lens.h()) * 0.5;
    if spec.round {
        let centre = local(spec.lens.cx(), spec.lens.cy(), 0.0015);
        add_cyl(
            stage,
            kit,
            &mats.panel,
            centre,
            (r + 5.0) * PX,
            0.006,
            Quat::from_rotation_x(FRAC_PI_2),
        );
        add_cyl(
            stage,
            kit,
            &mats.glass,
            centre + Vec3::new(0.0, 0.0, 0.0045),
            r * PX,
            0.003,
            Quat::from_rotation_x(FRAC_PI_2),
        );
    } else {
        spawn_frame(stage, kit, mats, spec.lens, 5.0, 0.0015, 0.005, &mats.panel);
        let (c, s) = rect_box(spec.lens, 0.0012, 0.002);
        add_box(stage, kit, &mats.glass, c, s);
    }
}

/// The reverse latch under the throttle pair.
fn spawn_reverse(stage: &mut ChildSpawnerCommands, kit: &Kit, mats: &Mats) {
    // A raised handle body with a groove down its middle, so the latch reads as
    // something to push rather than another slot.
    let body = REVERSE_BAR.expand(2.0);
    let (c, s) = rect_box(body, 0.004, 0.008);
    add_box(stage, kit, &mats.panel, c, s);
    let groove = PRect::new(body.x0 + 8.0, body.cy() - 4.0, body.x1 - 8.0, body.cy() + 4.0);
    let (c, s) = rect_box(groove, 0.009, 0.002);
    add_box(stage, kit, &mats.glass, c, s);
}

/// Four raised bars around `rect`, forming a frame with its opening exactly on
/// the rect.
fn spawn_frame(
    stage: &mut ChildSpawnerCommands,
    kit: &Kit,
    mats: &Mats,
    rect: PRect,
    bar: f32,
    z: f32,
    depth: f32,
    mat: &Handle<StandardMaterial>,
) {
    for r in [
        PRect::new(rect.x0 - bar, rect.y0 - bar, rect.x1 + bar, rect.y0),
        PRect::new(rect.x0 - bar, rect.y1, rect.x1 + bar, rect.y1 + bar),
        PRect::new(rect.x0 - bar, rect.y0, rect.x0, rect.y1),
        PRect::new(rect.x1, rect.y0, rect.x1 + bar, rect.y1),
    ] {
        let (c, s) = rect_box(r, z, depth);
        add_box(stage, kit, mat, c, s);
    }
    // The frame's top face is coplanar with the light, so the 3D sides are
    // edge-on and invisible in an orthographic front view. Paint the chamfer:
    // a light line on the top edge, a shadow on the bottom.
    let top = z + depth * 0.5 + 0.0007;
    let light = PRect::new(rect.x0 - bar, rect.y0 - bar, rect.x1 + bar, rect.y0 - bar + 2.5);
    let (c, s) = rect_box(light, top, 0.0012);
    add_box(stage, kit, &mats.edge, c, s);
    let dark = PRect::new(rect.x0 - bar, rect.y1 + bar - 2.5, rect.x1 + bar, rect.y1 + bar);
    let (c, s) = rect_box(dark, top, 0.0012);
    add_box(stage, kit, &mats.bezel, c, s);
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
