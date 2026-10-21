//! Engine-side preview of the AH-64D pilot-station cockpit asset.
//!
//! The GLB is authored around the pilot's eye (origin) with nose +Z. This
//! example renders a fixed 1600x900 frame off-screen (so the result does not
//! depend on the desktop's window shape), pins the camera at the eye, looks
//! down the nose, and writes the frame to disk.
//!
//! Run: `cargo run --features shots --example ah64_cockpit_shot`
//! Open `/tmp/opencode/ah64_cockpit_bevy.png`.

use bevy::asset::RenderAssetUsages;
use bevy::camera::{Camera, ClearColorConfig, RenderTarget};
use bevy::image::Image;
use bevy::prelude::*;
use bevy::render::render_resource::{Extent3d, TextureDimension, TextureFormat, TextureUsages};
use bevy::render::view::window::screenshot::{save_to_disk, Screenshot};
use bevy::winit::WinitSettings;

#[derive(Resource)]
struct ShotTarget(Handle<Image>);

#[derive(Resource, Default)]
struct Frame(u32);

fn main() {
    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "AH-64D cockpit preview".into(),
                resolution: (1280, 720).into(),
                ..default()
            }),
            ..default()
        }))
        .insert_resource(ClearColor(Color::srgb(0.36, 0.55, 0.80)))
        .insert_resource(WinitSettings::continuous())
        .insert_resource(Frame::default())
        .add_systems(Startup, setup)
        .add_systems(Update, (shot, quit))
        .run();
}

fn setup(
    mut commands: Commands,
    assets: Res<AssetServer>,
    mut images: ResMut<Assets<Image>>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    // Fixed-size off-screen target: the preview is always 16:9, whatever the
    // desktop window manager does with the actual window.
    let size = Extent3d {
        width: 1600,
        height: 900,
        depth_or_array_layers: 1,
    };
    let mut image = Image::new_fill(
        size,
        TextureDimension::D2,
        &[28, 40, 56, 255],
        TextureFormat::Rgba8UnormSrgb,
        RenderAssetUsages::default(),
    );
    image.texture_descriptor.usage |= TextureUsages::RENDER_ATTACHMENT | TextureUsages::COPY_SRC;
    let target = images.add(image);
    commands.insert_resource(ShotTarget(target.clone()));

    // Asset frame: +X left, +Y up, +Z nose; eye at the origin.
    commands.spawn((
        Transform::default(),
        Visibility::default(),
        WorldAssetRoot(assets.load(GltfAssetLabel::Scene(0).from_asset("ah64d_cockpit.glb"))),
    ));

    // Camera at the pilot's eye, looking down the nose (+Z). Bevy cameras look
    // down -Z, hence the 180° yaw.
    commands.spawn((
        Camera3d::default(),
        Camera {
            clear_color: ClearColorConfig::Custom(Color::srgb(0.36, 0.55, 0.80)),
            ..default()
        },
        RenderTarget::Image(target.into()),
        Projection::Perspective(PerspectiveProjection {
            fov: 55_f32.to_radians(),
            near: 0.01,
            far: 20_000.0,
            ..default()
        }),
        bevy::core_pipeline::tonemapping::Tonemapping::None,
        Transform::from_rotation(Quat::from_rotation_y(std::f32::consts::PI)),
    ));

    // A neutral outdoor environment outside the canopy.
    commands.spawn((
        DirectionalLight {
            illuminance: 12_000.0,
            shadow_maps_enabled: false,
            ..default()
        },
        Transform::from_rotation(Quat::from_euler(EulerRot::XYZ, -0.55, 0.65, 0.0)),
    ));
    commands.spawn(AmbientLight {
        color: Color::srgb(0.65, 0.72, 0.85),
        brightness: 420.0,
        ..default()
    });
    // Cockpit fill, slightly behind the eye so the panel reads in shadow.
    commands.spawn((
        PointLight {
            intensity: 22_000.0,
            range: 4.0,
            shadow_maps_enabled: false,
            color: Color::srgb(1.0, 0.94, 0.86),
            ..default()
        },
        Transform::from_xyz(0.0, 0.55, -0.35),
    ));
    // A soft fill in front of the panel, so the lower half of the cockpit reads
    // in the preview. The game's own cockpit lighting can replace this.
    commands.spawn((
        PointLight {
            intensity: 9_000.0,
            range: 2.5,
            shadow_maps_enabled: false,
            color: Color::srgb(0.86, 0.92, 1.0),
            ..default()
        },
        Transform::from_xyz(0.0, -0.15, 0.20),
    ));

    commands.spawn((
        Mesh3d(meshes.add(Plane3d::default().mesh().size(8_000.0, 8_000.0))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.34, 0.31, 0.22),
            perceptual_roughness: 0.95,
            ..default()
        })),
        Transform::from_xyz(0.0, -60.0, 0.0),
    ));

    info!("AH-64D cockpit preview loaded");
}

fn shot(
    mut commands: Commands,
    target: Res<ShotTarget>,
    mut frames: ResMut<Frame>,
    mut exit: MessageWriter<AppExit>,
) {
    frames.0 += 1;
    if frames.0 == 480 {
        commands
            .spawn(Screenshot::image(target.0.clone()))
            .observe(save_to_disk("/tmp/opencode/ah64_cockpit_bevy.png"));
        info!("screenshot -> /tmp/opencode/ah64_cockpit_bevy.png");
    }
    if frames.0 >= 560 {
        exit.write(AppExit::Success);
    }
}

fn quit(keys: Res<ButtonInput<KeyCode>>, mut exit: MessageWriter<AppExit>) {
    if keys.just_pressed(KeyCode::Escape) {
        exit.write(AppExit::Success);
    }
}
