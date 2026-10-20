//! Pilot G-strain: grey-out, tunnel vision and the sustained over-G caution.
//!
//! The F-22's HUD already prints the load factor; this module supplies the
//! physiological half that a HUD number cannot: above ~6 G the visual field
//! shrinks and drains to grey (`assets/shaders/g_strain.wesl`), and a sustained
//! pull past 9 G raises a brief OVER-G caution. Real fighters do not sound a
//! horn for this — the pilot's vision and the flight-control limits are the
//! warning — so the cue is visual, not a buzzer.
//!
//! The strain is smoothed with a fast attack and a slower recovery, so a
//! momentary spike does not flicker the overlay and the grey takes a moment to
//! clear after the pull is released.

use bevy::prelude::*;
use bevy::render::render_resource::{AsBindGroup, ShaderType};
use bevy::shader::ShaderRef;
use bevy::ui_render::prelude::{MaterialNode, UiMaterial, UiMaterialPlugin};

use crate::app_state::AppState;
use crate::sim::AircraftState;

/// Load factor where the visual field starts to close in.
const ONSET_G: f32 = 6.0;
/// Load factor where the tunnel is fully closed.
const FULL_G: f32 = 9.5;
/// Load factor held before the airframe caution appears.
const OVER_G: f32 = 9.0;
/// How long above `OVER_G` before the caution shows (s).
const OVER_G_DELAY: f32 = 0.8;
/// Grey-out onset / recovery time constants (s).
const ATTACK_TAU: f32 = 0.18;
const RECOVER_TAU: f32 = 0.9;

#[derive(Clone, Copy, ShaderType, Default)]
struct GStrainParams {
    /// x = strain 0..1; yzw unused.
    a: Vec4,
}

#[derive(Asset, TypePath, AsBindGroup, Clone, Default)]
struct GStrainMaterial {
    #[uniform(0)]
    params: GStrainParams,
}

impl UiMaterial for GStrainMaterial {
    fn fragment_shader() -> ShaderRef {
        "shaders/g_strain.wesl".into()
    }
}

#[derive(Component)]
struct GStrainOverlay;

#[derive(Component)]
struct OverGCaution;

#[derive(Resource, Default)]
struct Strain {
    /// Smoothed display value, 0..1.
    shown: f32,
    /// Time spent above `OVER_G` (s), with a faster decay below it.
    over_g_s: f32,
    /// Screenshot hook (`G_STRAIN=1`): hold the overlay fully closed.
    forced: bool,
}

pub struct GStrainPlugin;

impl Plugin for GStrainPlugin {
    fn build(&self, app: &mut App) {
        app.insert_resource(Strain {
            forced: std::env::var("G_STRAIN").is_ok_and(|v| v == "1"),
            ..default()
        })
        .add_plugins(UiMaterialPlugin::<GStrainMaterial>::default())
            .add_systems(Startup, spawn_overlay)
            .add_systems(Update, update_strain);
    }
}

fn spawn_overlay(mut commands: Commands, mut materials: ResMut<Assets<GStrainMaterial>>) {
    let mat = materials.add(GStrainMaterial::default());
    commands
        .spawn((
            GStrainOverlay,
            MaterialNode(mat),
            Node {
                position_type: PositionType::Absolute,
                left: Val::Px(0.0),
                top: Val::Px(0.0),
                width: Val::Percent(100.0),
                height: Val::Percent(100.0),
                display: Display::None,
                ..default()
            },
            GlobalZIndex(40),
        ))
        .with_children(|root| {
            root.spawn((
                OverGCaution,
                Text::new("OVER-G"),
                TextFont {
                    font_size: FontSize::Px(24.0),
                    ..default()
                },
                TextColor(Color::srgb(1.0, 0.22, 0.16)),
                TextLayout::new(Justify::Center, LineBreak::NoWrap),
                Node {
                    position_type: PositionType::Absolute,
                    top: Val::Percent(16.0),
                    left: Val::Px(0.0),
                    width: Val::Percent(100.0),
                    ..default()
                },
                Visibility::Hidden,
            ));
        });
}

fn update_strain(
    time: Res<Time>,
    app_state: Res<State<AppState>>,
    state: Res<AircraftState>,
    mut strain: ResMut<Strain>,
    mut materials: ResMut<Assets<GStrainMaterial>>,
    overlays: Query<(&MaterialNode<GStrainMaterial>, &mut Node), With<GStrainOverlay>>,
    mut caution: Query<&mut Visibility, With<OverGCaution>>,
) {
    let dt = time.delta_secs().clamp(0.0, 0.1);
    let flying = *app_state.get() == AppState::Flying;
    let g = if flying { state.g_norm as f32 } else { 1.0 };
    let target = if strain.forced {
        1.0
    } else if flying {
        strain_target(g)
    } else {
        0.0
    };

    // Fast onset, slow recovery: a hard pull greys the view at once, while the
    // field takes a beat to return after the aircraft unloads.
    let tau = if target > strain.shown {
        ATTACK_TAU
    } else {
        RECOVER_TAU
    };
    strain.shown += (target - strain.shown) * (1.0 - (-dt / tau).exp());
    if strain.shown < 0.004 {
        strain.shown = 0.0;
    }

    if flying && g > OVER_G {
        strain.over_g_s += dt;
    } else {
        strain.over_g_s = (strain.over_g_s - dt * 2.0).max(0.0);
    }
    let warn = flying && strain.over_g_s >= OVER_G_DELAY;

    for (node_mat, mut node) in overlays {
        if let Some(mut mat) = materials.get_mut(&node_mat.0) {
            mat.params = GStrainParams {
                a: Vec4::new(strain.shown, 0.0, 0.0, 0.0),
            };
        }
        let want = if strain.shown > 0.0 {
            Display::Flex
        } else {
            Display::None
        };
        if node.display != want {
            node.display = want;
        }
    }
    for mut vis in &mut caution {
        let want = if warn {
            Visibility::Inherited
        } else {
            Visibility::Hidden
        };
        if *vis != want {
            *vis = want;
        }
    }
}

/// Smooth onset: nothing below `ONSET_G`, full tunnel at `FULL_G`.
fn strain_target(g: f32) -> f32 {
    let x = ((g - ONSET_G) / (FULL_G - ONSET_G)).clamp(0.0, 1.0);
    x * x * (3.0 - 2.0 * x)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn strain_starts_at_six_g_and_saturates() {
        assert_eq!(strain_target(0.0), 0.0);
        assert_eq!(strain_target(ONSET_G), 0.0);
        assert_eq!(strain_target(FULL_G), 1.0);
        assert_eq!(strain_target(15.0), 1.0);
        assert!(strain_target(7.0) > 0.0 && strain_target(7.0) < 0.5);
        assert!(strain_target(9.0) > strain_target(8.0));
    }
}
