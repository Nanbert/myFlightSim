//! The Cessna-172 attitude indicator (artificial horizon).
//!
//! Unlike the other gauges this one has no fixed dial texture: the horizon
//! slides and rolls with the aircraft, so the whole face is drawn procedurally
//! by a [`UiMaterial`] on a single square node (see `assets/shaders/attitude.wesl`).
//! The shader un-rotates each pixel by the bank angle and offsets it by the
//! pitch, which places the pixel in the *earth* frame — from there the
//! sky/ground split, the pitch ladder and the horizon line are trivial. The
//! bezel, the bank scale and the orange symbolic aircraft stay fixed to the
//! case. [`update`] feeds the shader the live pitch and roll each frame.

use bevy::asset::Asset;
use bevy::prelude::*;
use bevy::render::render_resource::AsBindGroup;
use bevy::shader::ShaderRef;
use bevy::ui_render::prelude::{MaterialNode, UiMaterial};

use super::*;
use crate::sim::AircraftState;

/// Shader parameters: `x` = pitch (deg, + nose up), `y` = roll (deg, + right
/// wing down).
#[derive(Asset, TypePath, AsBindGroup, Clone, Default)]
pub(super) struct AttitudeMaterial {
    #[uniform(0)]
    params: Vec4,
}

impl UiMaterial for AttitudeMaterial {
    fn fragment_shader() -> ShaderRef {
        "shaders/attitude.wesl".into()
    }
}

pub(super) fn setup(mut commands: Commands, mut materials: ResMut<Assets<AttitudeMaterial>>) {
    let mat = materials.add(AttitudeMaterial::default());
    let root = spawn_root_centered(&mut commands, PanelKey::Attitude, MARGIN);
    commands.entity(root).with_children(|p| {
        p.spawn((
            MaterialNode(mat),
            Node {
                position_type: PositionType::Absolute,
                left: Val::Px(0.0),
                top: Val::Px(0.0),
                width: Val::Px(NODE),
                height: Val::Px(NODE),
                ..default()
            },
        ));
    });
}

/// Feed the live attitude to the shader.
pub(super) fn update(
    state: Res<AircraftState>,
    mut materials: ResMut<Assets<AttitudeMaterial>>,
    q: Query<&MaterialNode<AttitudeMaterial>>,
) {
    let Ok(node) = q.single() else {
        return;
    };
    if let Some(mut mat) = materials.get_mut(&node.0) {
        mat.params = Vec4::new(state.pitch_deg as f32, state.roll_deg as f32, 0.0, 0.0);
    }
}
