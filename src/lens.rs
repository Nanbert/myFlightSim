//! Screen-space rain on the lens.
//!
//! While it rains, a fullscreen post-process lays droplets over the rendered
//! frame: a hash-driven grid of static condensation drops plus two layers of
//! sliding drops with trails. The droplet field's gradient refracts the scene
//! behind each drop and the un-cleared glass hazes over — a port of the
//! *Heartfelt* droplet field (Martijn Steinrucken / BigWings, shadertoy
//! `ltffzl`), reworked to sample the live frame instead of a raymarched scene.
//!
//! Everything is self-contained: the pass reads [`Weather`] and rides on
//! Bevy's `FullscreenMaterial` (no hand-rolled render graph). The component is
//! attached to the camera only while it rains, so a clear sky costs nothing.

use bevy::core_pipeline::fullscreen_material::{FullscreenMaterial, FullscreenMaterialPlugin};
use bevy::prelude::*;
use bevy::render::extract_component::ExtractComponent;
use bevy::render::render_resource::ShaderType;
use bevy::render::RenderApp;
use bevy::shader::ShaderRef;

use crate::weather::Weather;

/// Per-frame parameters handed to `assets/shaders/rain_lens.wesl`. The four
/// `f32`s fill one 16-byte uniform; the pads keep it aligned for WGSL's
/// uniform layout rules.
#[derive(Component, Clone, Copy, Default, ShaderType, ExtractComponent)]
#[extract_app(RenderApp)]
struct RainLens {
    /// Rain intensity, 0 (clear) … 1 (downpour).
    intensity: f32,
    /// Scene time (s), driving the drop animation.
    time: f32,
    #[doc(hidden)]
    pad0: f32,
    #[doc(hidden)]
    pad1: f32,
}

impl FullscreenMaterial for RainLens {
    fn fragment_shader() -> ShaderRef {
        "shaders/rain_lens.wesl".into()
    }
}

pub struct LensPlugin;

impl Plugin for LensPlugin {
    fn build(&self, app: &mut App) {
        app.add_plugins(FullscreenMaterialPlugin::<RainLens>::default())
            .add_systems(Update, manage_lens);
    }
}

/// Attach the effect to the camera while it rains and refresh its uniform; drop
/// it once the rain stops, so the pass (and its frame copy) is skipped entirely.
fn manage_lens(
    mut commands: Commands,
    weather: Res<Weather>,
    time: Res<Time>,
    cameras: Query<(Entity, Option<&RainLens>), With<Camera3d>>,
) {
    let rain = weather.rain;
    let t = time.elapsed_secs();
    for (e, lens) in &cameras {
        if rain > 0.001 {
            commands.entity(e).insert(RainLens {
                intensity: rain,
                time: t,
                pad0: 0.0,
                pad1: 0.0,
            });
        } else if lens.is_some() {
            commands.entity(e).remove::<RainLens>();
        }
    }
}
