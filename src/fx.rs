//! F-22 engine effects.
//!
//! Two effects, both gated to the F-22 (`sim.name == "f22"`), because only that
//! model ships the geometry (measured nozzle anchors) and only its FDM publishes
//! the signals they are driven from:
//!
//! - **Afterburner flame** — one additive cone per nozzle ([`FlameMaterial`] +
//!   `assets/shaders/flame.wesl`), its brightness following the throttle. The
//!   camera's bloom pass (see `view.rs`) turns the >1 hot core into a glow.
//! - **Exhaust heat haze** — a screen-space refraction behind the nozzles
//!   ([`ExhaustHaze`] + `assets/shaders/exhaust_haze.wesl`), riding on Bevy's
//!   `FullscreenMaterial` exactly like the rain-on-lens pass in `lens.rs`.
//!
//! The plugin is self-contained: it spawns its entities once `AircraftRoot`
//! exists (see [`spawn_fx`]) rather than hooking the flight spawn, so examples
//! that build their own scene (and never add this plugin) are unaffected, and
//! there is no cross-plugin ordering to get wrong. Every entity carries
//! `DespawnOnExit` so a fresh flight starts clean.

use bevy::core_pipeline::fullscreen_material::{FullscreenMaterial, FullscreenMaterialPlugin};
use bevy::prelude::*;
use bevy::render::extract_component::ExtractComponent;
use bevy::render::mesh::ConeAnchor;
use bevy::render::render_resource::{AsBindGroup, ShaderType};
use bevy::render::RenderApp;
use bevy::shader::ShaderRef;

use crate::app_state::AppState;
use crate::sim::{AircraftState, Sim};
use crate::view::{AircraftRoot, model_to_body};

/// Engine nozzle exits in the F-22's own glTF frame (metres, +X left / +Y up /
/// +Z nose), measured from the model geometry. Left is +X in that frame.
const F22_NOZZLE_L: Vec3 = Vec3::new(0.16, 0.145, -1.28);
const F22_NOZZLE_R: Vec3 = Vec3::new(-0.16, 0.145, -1.28);

/// Flame cone: base sits on the nozzle, apex trails aft. `Transform::scale.y`
/// stretches it, so this is the nominal (unscaled) size.
const FLAME_RADIUS: f32 = 0.26;
const FLAME_LENGTH: f32 = 3.4;

/// These effects exist for the F-22 alone.
fn is_f22(sim: &Sim) -> bool {
    sim.name == "f22"
}

// --- afterburner flame -----------------------------------------------------

/// Uniform for `assets/shaders/flame.wesl`. Three `vec4`s: a multiple of 16
/// bytes, as the uniform address space requires.
#[derive(Clone, Copy, ShaderType, Default)]
struct FlameParams {
    /// rgb = hot core colour, w = unused.
    hot: Vec4,
    /// rgb = cool tail colour, w = unused.
    cool: Vec4,
    /// x = time (s), y = throttle 0..1, z = intensity gain, w = turbulence.
    anim: Vec4,
}

#[derive(Asset, TypePath, AsBindGroup, Clone)]
struct FlameMaterial {
    #[uniform(0)]
    params: FlameParams,
}

impl Material for FlameMaterial {
    fn fragment_shader() -> ShaderRef {
        "shaders/flame.wesl".into()
    }

    fn alpha_mode(&self) -> AlphaMode {
        AlphaMode::Add
    }

    // A pure additive glow: nothing to write to a depth prepass, no shadows.
    fn enable_prepass() -> bool {
        false
    }

    fn enable_shadows() -> bool {
        false
    }
}

/// Marks a spawned flame cone, so `drive_flames` can find its material.
#[derive(Component)]
struct Flame;

// --- exhaust heat haze -----------------------------------------------------

/// Per-frame parameters for `assets/shaders/exhaust_haze.wesl`. Plain `f32`s
/// (like [`crate::lens::RainLens`]), four to a row, so the hand-written WGSL
/// struct matches the uniform layout byte for byte.
///
/// Nozzle screen positions and directions are in **aspect-corrected UV**: the
/// UV offset from the screen centre with x multiplied by the aspect ratio, so
/// the shader's lobes stay round on a wide window.
#[derive(Component, Clone, Copy, Default, ShaderType, ExtractComponent)]
#[extract_app(RenderApp)]
struct ExhaustHaze {
    uv0x: f32,
    uv0y: f32,
    dir0x: f32,
    dir0y: f32,
    str0: f32,
    rad0: f32,
    len0: f32,
    pad0: f32,
    uv1x: f32,
    uv1y: f32,
    dir1x: f32,
    dir1y: f32,
    str1: f32,
    rad1: f32,
    len1: f32,
    pad1: f32,
    time: f32,
    intensity: f32,
    pad2: f32,
    pad3: f32,
}

impl FullscreenMaterial for ExhaustHaze {
    fn fragment_shader() -> ShaderRef {
        "shaders/exhaust_haze.wesl".into()
    }
}

// --- plugin ----------------------------------------------------------------

pub struct FxPlugin;

impl Plugin for FxPlugin {
    fn build(&self, app: &mut App) {
        app.add_plugins((
            MaterialPlugin::<FlameMaterial>::default(),
            FullscreenMaterialPlugin::<ExhaustHaze>::default(),
        ))
        .add_systems(
            Update,
            (spawn_fx, drive_flames)
                .chain()
                .run_if(in_state(AppState::Flying)),
        )
        // The haze needs the final airframe/camera pose, so it runs after the
        // view systems have placed everything.
        .add_systems(
            PostUpdate,
            manage_haze.run_if(in_state(AppState::Flying)),
        );
    }
}

/// Spawn the F-22's effect entities once the flown aircraft exists. Idempotent:
/// it bails as soon as a flame is present, so it fires exactly once per flight.
///
/// The cones are children of `AircraftRoot`, so they inherit the interpolated
/// airframe pose.
fn spawn_fx(
    mut commands: Commands,
    sim: Res<Sim>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut flame_mats: ResMut<Assets<FlameMaterial>>,
    roots: Query<Entity, With<AircraftRoot>>,
    flames: Query<(), With<Flame>>,
) {
    if !is_f22(&sim) || !flames.is_empty() {
        return;
    }
    let Ok(root) = roots.single() else {
        return;
    };

    // A cone with its base at the origin and its apex at +Y. `ConeAnchor::Base`
    // puts the wide base on the entity origin, and the +90° X rotation maps the
    // cone's +Y axis onto +Z, so the base sits on the nozzle and the plume
    // tapers to a point trailing aft. The wide (nozzle) end catches the eye and
    // the thin tail fades out, with no open end for the viewer to see into.
    let cone = Cone {
        radius: FLAME_RADIUS,
        height: FLAME_LENGTH,
    }
    .mesh()
    .resolution(20)
    .anchor(ConeAnchor::Base)
    .build();
    let cone = meshes.add(cone);
    let flame_mat = flame_mats.add(FlameMaterial {
        params: flame_params(0.0, 0.0),
    });

    let orient = Quat::from_rotation_x(std::f32::consts::FRAC_PI_2);
    for nozzle in [F22_NOZZLE_L, F22_NOZZLE_R] {
        commands.spawn((
            Flame,
            Mesh3d(cone.clone()),
            MeshMaterial3d(flame_mat.clone()),
            Transform {
                translation: model_to_body(&sim.name, nozzle),
                rotation: orient,
                scale: Vec3::ONE,
            },
            DespawnOnExit(AppState::Flying),
            ChildOf(root),
        ));
    }
}

/// The flame's material parameters for a given time and throttle. Colours are
/// pushed past 1.0 at the core so the camera's bloom (prefilter threshold 1.5)
/// lights up.
fn flame_params(time: f32, throttle: f32) -> FlameParams {
    FlameParams {
        hot: Vec4::new(2.6, 2.0, 1.1, 0.0),
        cool: Vec4::new(1.3, 0.45, 0.1, 0.0),
        anim: Vec4::new(time, throttle, 1.35, 0.6),
    }
}

/// Keep each flame's uniform and stretch in step with the throttle.
fn drive_flames(
    time: Res<Time>,
    state: Res<AircraftState>,
    mut materials: ResMut<Assets<FlameMaterial>>,
    mut flames: Query<(&MeshMaterial3d<FlameMaterial>, &mut Transform), With<Flame>>,
) {
    let t = time.elapsed_secs();
    let throttle = state.throttle.clamp(0.0, 1.0) as f32;
    // A lit engine always shows a little flame; the burner's roar is at the top.
    let drive = ((throttle - 0.15) / 0.85).clamp(0.0, 1.0);
    let length = 0.45 + 0.75 * drive;

    for (mat, mut tf) in &mut flames {
        if let Some(mut m) = materials.get_mut(&mat.0) {
            m.params = flame_params(t, throttle);
        }
        tf.scale = Vec3::new(1.0, length, 1.0);
    }
}

/// Refresh the camera's exhaust-haze uniform from the live nozzle screen
/// positions, attaching/detaching the pass with the throttle so a dead engine
/// costs nothing.
fn manage_haze(
    mut commands: Commands,
    time: Res<Time>,
    sim: Res<Sim>,
    state: Res<AircraftState>,
    root: Query<&Transform, With<AircraftRoot>>,
    mut cameras: Query<(Entity, &Camera, &GlobalTransform, Option<&ExhaustHaze>), With<Camera3d>>,
) {
    let throttle = state.throttle.clamp(0.0, 1.0) as f32;
    let want = is_f22(&sim) && throttle > 0.02;
    let root_tf = root.single().ok();
    let name = sim.name.as_str();

    for (e, cam, cam_gt, existing) in &mut cameras {
        // Drop the pass (no per-frame cost) when nothing is running or seen.
        let Some(root_tf) = root_tf.filter(|_| want) else {
            if existing.is_some() {
                commands.entity(e).remove::<ExhaustHaze>();
            }
            continue;
        };
        let Some(rect) = cam.logical_viewport_rect() else {
            continue;
        };
        let size = rect.size();
        if size.x < 1.0 || size.y < 1.0 {
            continue;
        }
        let aspect = size.x / size.y;

        // A nozzle's screen UV plus the on-screen direction its exhaust trails
        // toward, both in aspect-corrected UV. `None` if it is behind the camera.
        let nozzle = |model: Vec3| -> Option<(Vec2, Vec2)> {
            let world = root_tf.translation
                + root_tf.rotation * (root_tf.scale * model_to_body(name, model));
            let p = cam.world_to_viewport(cam_gt, world).ok()?;
            // The plume runs along the body's aft axis (+Z); sample a point a
            // few metres downstream to get its screen direction.
            let aft = world + root_tf.rotation * (root_tf.scale * Vec3::Z) * 4.0;
            let p2 = cam.world_to_viewport(cam_gt, aft).ok()?;
            let to_uv = |v: Vec2| ((v - rect.min) / size - Vec2::splat(0.5)) * Vec2::new(aspect, 1.0);
            let a = to_uv(p);
            let b = to_uv(p2);
            // Looking straight down the exhaust the two points nearly coincide;
            // fall back to "heat rises" so there is still a shimmer.
            let off = b - a;
            let dir = if off.length() < 1e-4 {
                Vec2::NEG_Y
            } else {
                off.normalize()
            };
            Some((a, dir))
        };
        let (Some((uv0, dir0)), Some((uv1, dir1))) = (nozzle(F22_NOZZLE_L), nozzle(F22_NOZZLE_R))
        else {
            if existing.is_some() {
                commands.entity(e).remove::<ExhaustHaze>();
            }
            continue;
        };

        // Strongest just off idle, holding through the burner.
        let intensity = 0.35 + 0.65 * throttle;
        // A subtle shimmer: the offset is a small fraction of the screen, not a
        // full warp. `rad`/`len` are the lobe's half-width and length in
        // aspect-corrected UV.
        let str = 0.011 * intensity;
        commands.entity(e).insert(ExhaustHaze {
            uv0x: uv0.x,
            uv0y: uv0.y,
            dir0x: dir0.x,
            dir0y: dir0.y,
            str0: str,
            rad0: 0.035,
            len0: 0.085,
            pad0: 0.0,
            uv1x: uv1.x,
            uv1y: uv1.y,
            dir1x: dir1.x,
            dir1y: dir1.y,
            str1: str,
            rad1: 0.035,
            len1: 0.085,
            pad1: 0.0,
            time: time.elapsed_secs(),
            intensity,
            pad2: 0.0,
            pad3: 0.0,
        });
    }
}
