//! Landmark bust on the glacial summit.
//!
//! The GLACIAL mission carries a dominant central peak at the world origin
//! (see `map::glacial_peak`, whose flat summit disc is sized for this). This
//! module plants the half-body model from `xuan.glb` — ported from the
//! terrain lab's statue — scaled to ~870 m tall on that disc. The scene's own
//! sun and ambient (the ones `view::dim_lights` sweep through the day) light
//! the standard-PBR bust, so no extra lights are needed.

use bevy::prelude::*;

use crate::app_state::AppState;
use crate::map::{ground_height, ActiveMap, MapId};

/// Scale applied to the ~1.09 m tall source model: 800x -> ~870 m tall bust.
const STATUE_SCALE: f32 = 800.0;
/// Sink the base a little so the skirt of the model meets the terrain.
const STATUE_SINK: f32 = 12.0;
/// Yaw that turns the bust to face the default southern approach (+Z).
const STATUE_YAW: f32 = 0.0;

pub struct StatuePlugin;

impl Plugin for StatuePlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Update, present_statue);
    }
}

#[derive(Component)]
struct StatueRoot;

/// Spawn the bust once the glacial mission is the active map. Leaving the
/// flight despawns it with the rest of the arena; a re-enter spawns it again.
fn present_statue(
    mut commands: Commands,
    assets: Res<AssetServer>,
    active: Res<ActiveMap>,
    spawned: Query<(), With<StatueRoot>>,
) {
    if !spawned.is_empty() || active.0 != Some(MapId::Glacial) {
        return;
    }
    let summit = ground_height(MapId::Glacial, 0.0, 0.0);
    commands.spawn((
        StatueRoot,
        WorldAssetRoot(assets.load(GltfAssetLabel::Scene(0).from_asset("xuan.glb"))),
        Transform::from_xyz(0.0, summit - STATUE_SINK, 0.0)
            .with_rotation(Quat::from_rotation_y(STATUE_YAW))
            .with_scale(Vec3::splat(STATUE_SCALE)),
        DespawnOnExit(AppState::Flying),
    ));
}
