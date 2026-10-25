//! Crosshair interaction with the modelled MCP in the 737 cockpit.
//!
//! The cockpit view locks the mouse for free-look (see `view::cockpit_cursor`),
//! so there is no cursor to click with. A fixed crosshair sits at the centre of
//! the view, and a red frame tracks the modelled MCP board. Line the crosshair
//! up with the board and click: `mcp_panel` then raises the MCP as a centred
//! 2D panel in front of the blurred, dimmed cockpit, releases the mouse and
//! lets the individual controls be clicked and hovered directly. Both paths
//! drive the same `Mcp`/`Autopilot` resources, so the cockpit panel and the
//! chase-view strip stay in sync.

use bevy::prelude::*;
use bevy::text::FontSize;

use crate::aircraft_profile;
use crate::app_state::{unpaused, AppState};
use crate::sim::Sim;
use crate::ui_theme::centered;
use crate::view::{self, CockpitTune, PanelFocus, Ui};

/// The modelled MCP board in cockpit-model coordinates: the 737 model is
/// authored in metres with +X left / +Y up / +Z nose. The bounds cover the
/// COURSE knob through CMD A/B; the board face sits at `BOARD_Z`.
const BOARD_Z: f32 = 1.425;
const BOARD_X: (f32, f32) = (-0.15, 0.265);
const BOARD_Y: (f32, f32) = (1.99, 2.10);

/// The control stand's top surface, in the same model frame. It is a
/// horizontal plane (the pedestal top), measured with
/// `examples/pedestal_calib.rs`. Looking down through this rectangle selects
/// the stand; looking at the MCP is almost horizontal, so the two never
/// compete.
const PED_Y: f32 = 1.52;
const PED_X: (f32, f32) = (-0.18, 0.06);
const PED_Z: (f32, f32) = (1.00, 1.58);

/// Red frame around the clickable board: dim until aimed at, hot on hover.
const COLD: Color = Color::srgba(1.0, 0.26, 0.18, 0.50);
const HOT: Color = Color::srgba(1.0, 0.30, 0.20, 1.0);

/// Whether the crosshair is on the modelled MCP board or the pedestal.
#[derive(Resource, Default)]
struct BoardHover {
    hovered: bool,
    pedestal: bool,
}

#[derive(Component)]
struct Crosshair;

#[derive(Component)]
struct CrosshairTick;

#[derive(Component)]
struct CrosshairLabel;

#[derive(Component)]
struct BoardFrame;

pub struct CockpitPlugin;

impl Plugin for CockpitPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<BoardHover>()
            .add_systems(
                OnEnter(AppState::Flying),
                spawn_overlay.after(crate::sim::StartFlight),
            )
            .add_systems(
                Update,
                (board_probe, sync_overlay)
                    .chain()
                    .run_if(unpaused),
            );
    }
}

fn spawn_overlay(mut commands: Commands, sim: Res<Sim>) {
    // The board drives the MCP, so it is 737-only.
    if !aircraft_profile::profile(&sim.name).has_autopilot {
        return;
    }

    commands
        .spawn((
            Crosshair,
            Node {
                position_type: PositionType::Absolute,
                width: Val::Percent(100.0),
                height: Val::Percent(100.0),
                ..default()
            },
            GlobalZIndex(16),
            DespawnOnExit(AppState::Flying),
        ))
        .with_children(|root| {
            // Four ticks with a gap around a centre dot, like a gun cross.
            for node in [
                centered(4.0, 4.0, 0.0, 0.0),
                centered(2.0, 8.0, 0.0, -18.0),
                centered(2.0, 8.0, 0.0, 18.0),
                centered(8.0, 2.0, -18.0, 0.0),
                centered(8.0, 2.0, 18.0, 0.0),
            ] {
                root.spawn((CrosshairTick, node, BackgroundColor(COLD)));
            }
            // A fixed-width flex row keeps the auto-sized text centred under
            // the crosshair.
            root.spawn(Node {
                position_type: PositionType::Absolute,
                left: Val::Percent(50.0),
                top: Val::Percent(50.0),
                width: Val::Px(300.0),
                margin: UiRect {
                    left: Val::Px(-150.0),
                    top: Val::Px(26.0),
                    ..default()
                },
                justify_content: JustifyContent::Center,
                ..default()
            })
            .with_children(|label| {
                label.spawn((
                    CrosshairLabel,
                    Text::new(""),
                    TextFont {
                        font_size: FontSize::Px(14.0),
                        ..default()
                    },
                    TextColor(COLD),
                    TextLayout::new(Justify::Center, LineBreak::NoWrap),
                ));
            });
        });

    // The clickable MCP board outline. Its screen rect is projected each frame.
    commands.spawn((
        BoardFrame,
        Node {
            position_type: PositionType::Absolute,
            display: Display::None,
            border: UiRect::all(Val::Px(2.0)),
            border_radius: BorderRadius::all(Val::Px(4.0)),
            ..default()
        },
        BorderColor::all(COLD),
        BackgroundColor(Color::NONE),
        GlobalZIndex(15),
        DespawnOnExit(AppState::Flying),
    ));
}

/// Test the crosshair against the modelled board and project its outline.
///
/// The eye is fixed, so both are pure geometry: the camera's model-frame look
/// direction gives the hit on the board plane, and the four board corners map
/// through the same perspective projection the camera uses.
fn board_probe(
    mouse: Res<ButtonInput<MouseButton>>,
    sim: Res<Sim>,
    tune: Res<CockpitTune>,
    mut ui: ResMut<Ui>,
    mut hover: ResMut<BoardHover>,
    mut frame: Query<&mut Node, With<BoardFrame>>,
    cameras: Query<&Camera, (With<Camera3d>, Without<crate::cloud_camera::CloudCamera>)>,
) {
    hover.hovered = false;
    hover.pedestal = false;
    let autopilot = aircraft_profile::profile(&sim.name).has_autopilot;
    if !ui.cockpit || ui.has_focus() || !autopilot {
        if let Ok(mut node) = frame.single_mut() {
            node.display = Display::None;
        }
        return;
    }

    let (Some(eye), Some(yaw)) = (
        view::cockpit_eye_model(&sim.name, &tune),
        view::cockpit_model_yaw(&sim.name),
    ) else {
        return;
    };
    let eye = Vec3::from_array(eye);
    // Camera orientation in the cockpit model's frame.
    let rot = Quat::from_rotation_y(-yaw) * view::look_rotation(&ui);
    let forward = rot * Vec3::NEG_Z;

    // Does the line of sight hit the board face inside its bounds?
    if forward.z.abs() > 1e-4 {
        let t = (BOARD_Z - eye.z) / forward.z;
        if t > 0.0 {
            let hit = eye + forward * t;
            hover.hovered = BOARD_X.0 <= hit.x
                && hit.x <= BOARD_X.1
                && BOARD_Y.0 <= hit.y
                && hit.y <= BOARD_Y.1;
        }
    }

    // The pedestal is below the pilot, not in front: only a downward look can
    // hit its plane, which keeps `MCP` and `CONTROL STAND` from stacking.
    if !hover.hovered && forward.y < -1e-4 {
        let t = (PED_Y - eye.y) / forward.y;
        if t > 0.0 {
            let hit = eye + forward * t;
            hover.pedestal = PED_X.0 <= hit.x
                && hit.x <= PED_X.1
                && PED_Z.0 <= hit.z
                && hit.z <= PED_Z.1;
        }
    }

    // Both hit tests are resolved before the click is handled: a click must
    // act on this frame's aim, not on a flag left over from the last one.
    if mouse.just_pressed(MouseButton::Left) {
        if hover.hovered {
            ui.panel_focus = Some(PanelFocus::Mcp);
        } else if hover.pedestal {
            ui.panel_focus = Some(PanelFocus::Pedestal);
        }
    }

    // Project the board corners to screen space for the outline.
    let Ok(camera) = cameras.single() else {
        return;
    };
    let Some(size) = camera.logical_viewport_size() else {
        return;
    };
    let fov = tune.fov.unwrap_or(std::f32::consts::FRAC_PI_4);
    let tan_half = (fov * 0.5).tan();
    let aspect = size.x / size.y;
    let inv = rot.inverse();
    let mut min = Vec2::splat(f32::MAX);
    let mut max = Vec2::splat(f32::MIN);
    for (x, y) in [
        (BOARD_X.0, BOARD_Y.0),
        (BOARD_X.1, BOARD_Y.0),
        (BOARD_X.0, BOARD_Y.1),
        (BOARD_X.1, BOARD_Y.1),
    ] {
        let d = inv * (Vec3::new(x, y, BOARD_Z) - eye);
        let depth = -d.z;
        if depth <= 0.05 {
            if let Ok(mut node) = frame.single_mut() {
                node.display = Display::None;
            }
            return;
        }
        let ndc = Vec2::new(d.x / (depth * tan_half * aspect), d.y / (depth * tan_half));
        let px = Vec2::new((ndc.x * 0.5 + 0.5) * size.x, (0.5 - ndc.y * 0.5) * size.y);
        min = min.min(px);
        max = max.max(px);
    }
    if let Ok(mut node) = frame.single_mut() {
        node.display = Display::Flex;
        node.left = Val::Px(min.x);
        node.top = Val::Px(min.y);
        node.width = Val::Px(max.x - min.x);
        node.height = Val::Px(max.y - min.y);
    }

}

/// Show/hide the crosshair and board outline, and reflect hover in their
/// colour. While the MCP panel is focused the crosshair gets out of the way.
fn sync_overlay(
    ui: Res<Ui>,
    hover: Res<BoardHover>,
    mut root: Query<&mut Node, With<Crosshair>>,
    mut frame_nodes: Query<&mut Node, (With<BoardFrame>, Without<Crosshair>)>,
    mut ticks: Query<&mut BackgroundColor, With<CrosshairTick>>,
    mut frame_colors: Query<&mut BorderColor, With<BoardFrame>>,
    mut label: Query<(&mut Text, &mut TextColor), With<CrosshairLabel>>,
) {
    let show = ui.cockpit && !ui.has_focus();
    let display = if show { Display::Flex } else { Display::None };
    for mut node in &mut root {
        if node.display != display {
            node.display = display;
        }
    }
    if !show {
        for mut node in &mut frame_nodes {
            node.display = Display::None;
        }
    }

    let aimed = show && (hover.hovered || hover.pedestal);
    let color = if aimed { HOT } else { COLD };
    for mut bg in &mut ticks {
        bg.0 = color;
    }
    // The frame outlines the MCP only; the pedestal has no outline to keep
    // the down-look uncluttered.
    let frame_color = if show && hover.hovered { HOT } else { COLD };
    for mut border in &mut frame_colors {
        *border = BorderColor::all(frame_color);
    }
    if let Ok((mut text, mut tint)) = label.single_mut() {
        let want = if !aimed {
            ""
        } else if hover.hovered {
            "CLICK TO OPEN MCP"
        } else {
            "CLICK TO OPEN CONTROL STAND"
        };
        if text.0 != want {
            text.0 = want.to_owned();
        }
        tint.0 = color;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The rest view must look at the board (otherwise the red frame is
    /// misleading and the click target is nowhere near the crosshair).
    #[test]
    fn rest_look_lands_inside_the_board() {
        let eye = Vec3::new(0.2801, 2.14, 0.9355);
        let yaw = std::f32::consts::PI;
        let look = Quat::from_rotation_y(-11.76_f32.to_radians())
            * Quat::from_rotation_x(-6.5_f32.to_radians());
        let rot = Quat::from_rotation_y(-yaw) * look;
        let forward = rot * Vec3::NEG_Z;
        let t = (BOARD_Z - eye.z) / forward.z;
        assert!(t > 0.0);
        let hit = eye + forward * t;
        assert!(
            BOARD_X.0 <= hit.x && hit.x <= BOARD_X.1,
            "x {} outside {:?}",
            hit.x,
            BOARD_X
        );
        assert!(
            BOARD_Y.0 <= hit.y && hit.y <= BOARD_Y.1,
            "y {} outside {:?}",
            hit.y,
            BOARD_Y
        );
    }

    /// The calibration pose (`pedestal_calib`) must land on the pedestal
    /// rectangle, and the MCP gaze must not: both panels share one focus slot,
    /// so a stray hit would open the wrong one.
    #[test]
    fn a_downward_look_lands_on_the_pedestal() {
        let eye = Vec3::new(0.2801, 2.14, 0.9355);
        let yaw = std::f32::consts::PI;
        let look = Quat::from_rotation_y(-52_f32.to_radians())
            * Quat::from_rotation_x(-60_f32.to_radians());
        let forward = Quat::from_rotation_y(-yaw) * look * Vec3::NEG_Z;
        assert!(forward.y < 0.0);
        let t = (PED_Y - eye.y) / forward.y;
        assert!(t > 0.0);
        let hit = eye + forward * t;
        assert!(
            PED_X.0 <= hit.x && hit.x <= PED_X.1,
            "x {} outside {:?}",
            hit.x,
            PED_X
        );
        assert!(
            PED_Z.0 <= hit.z && hit.z <= PED_Z.1,
            "z {} outside {:?}",
            hit.z,
            PED_Z
        );
    }

    #[test]
    fn the_mcp_gaze_does_not_hit_the_pedestal() {
        let eye = Vec3::new(0.2801, 2.14, 0.9355);
        let yaw = std::f32::consts::PI;
        let look = Quat::from_rotation_y(-11.76_f32.to_radians())
            * Quat::from_rotation_x(-6.5_f32.to_radians());
        let forward = Quat::from_rotation_y(-yaw) * look * Vec3::NEG_Z;
        let t = (PED_Y - eye.y) / forward.y;
        assert!(t > 0.0);
        let hit = eye + forward * t;
        let inside = PED_X.0 <= hit.x
            && hit.x <= PED_X.1
            && PED_Z.0 <= hit.z
            && hit.z <= PED_Z.1;
        assert!(!inside, "MCP gaze hit the stand at {hit:?}");
    }
}
