//! Cockpit instrument close-up for the C172.
//!
//! The cockpit view locks the mouse for free-look, so there is no pointer to
//! click with. Aiming uses the fixed centre crosshair instead: every live
//! instrument carries a dim red ring, and the dial under the crosshair burns
//! brighter. A left click eases the lens down to a narrow FOV — the eye does
//! not move, so nothing can clip through the panel or the yoke — and freezes
//! the head-look so the close-up stays exactly on the dial. A second click (or
//! leaving the cockpit) restores the rest lens and the entry gaze. `Esc` is
//! deliberately not used: in flight it is the "back to menu" key.

use bevy::prelude::*;
use bevy::text::FontSize;

use super::{project_dial, PanelKey, DIAL_PX};
use crate::app_state::{unpaused, AppState};
use crate::sim::Sim;
use crate::view::{AircraftRoot, CockpitTune, Ui};

/// Rest cockpit lens, matching `view::camera_feel`'s default.
const REST_FOV: f32 = std::f32::consts::FRAC_PI_4;
/// Close-up lens. The dial face is rasterised at 512 px, so a 45° → 16° zoom
/// (2.8× linear) lands near 1:1 texels instead of magnifying a soft face.
const ZOOM_FOV: f32 = 16.0 * std::f32::consts::PI / 180.0;
/// Lens ease rate, 1/s.
const FOV_RATE: f32 = 9.0;

/// Dim ring on every live dial, and the hot ring under the crosshair.
const COLD: Color = Color::srgba(1.0, 0.26, 0.18, 0.35);
const HOT: Color = Color::srgba(1.0, 0.30, 0.20, 1.0);

/// Every instrument the gauges drive live. The rings are shown for all of
/// them, not only the one being aimed at.
const ALL: [PanelKey; 9] = [
    PanelKey::Airspeed,
    PanelKey::Vsi,
    PanelKey::Altimeter,
    PanelKey::Tach,
    PanelKey::Heading,
    PanelKey::Turn,
    PanelKey::Attitude,
    PanelKey::Fuel,
    PanelKey::Cdi,
];

/// Which instrument the crosshair is on, and which one the lens is zoomed to.
#[derive(Resource, Default)]
pub struct InstrumentFocus {
    pub hovered: Option<PanelKey>,
    pub active: Option<PanelKey>,
    /// Eased 0 (rest lens) … 1 (close-up).
    pub blend: f32,
    /// Head-look (yaw, pitch) when the close-up was entered. The lens pans
    /// from here to the dial's centre and back on exit.
    entry: (f32, f32),
    /// Head-look that puts the active dial in the middle of the lens. `None`
    /// until it has been computed for this target.
    aim: Option<(f32, f32)>,
}

#[derive(Component)]
struct FocusRing(PanelKey);

#[derive(Component)]
struct AimCrosshair;

#[derive(Component)]
struct AimTick;

#[derive(Component)]
struct AimHint;

pub struct FocusPlugin;

impl Plugin for FocusPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<InstrumentFocus>()
            .add_systems(
                OnEnter(AppState::Flying),
                spawn_overlay.after(crate::sim::StartFlight),
            )
            .add_systems(
                Update,
                (update_focus, place_rings, sync_overlay)
                    .chain()
                    // Same reason as `gauge::seat_panel`: project with this
                    // frame's airframe pose, or the rings trail the panel by a
                    // frame whenever the aircraft is moving.
                    .after(crate::view::follow_camera)
                    .after(crate::view::sync_aircraft)
                    .run_if(unpaused),
            );
    }
}

fn spawn_overlay(
    mut commands: Commands,
    sim: Res<Sim>,
    ui: Res<Ui>,
    mut focus: ResMut<InstrumentFocus>,
) {
    // A fresh flight starts with no close-up, even if the last one ended while
    // zoomed in.
    *focus = InstrumentFocus::default();
    if sim.name != "c172p" {
        return;
    }

    // Development hook for screenshots: FOCUS=altimeter opens the close-up
    // straight away, so a headless pass can check the zoomed lens.
    if let Ok(name) = std::env::var("FOCUS") {
        if let Some(key) = PanelKey::from_name(&name) {
            focus.entry = (ui.look_yaw, ui.look_pitch);
            focus.active = Some(key);
            focus.blend = 1.0;
        }
    }

    // One ring per live dial. The instruments themselves are spawned at
    // startup, so these sit on top; the explicit z-index also keeps them
    // above the instrument faces and needles.
    for key in ALL {
        commands.spawn((
            FocusRing(key),
            Node {
                position_type: PositionType::Absolute,
                display: Display::None,
                width: Val::Px(2.0 * DIAL_PX),
                height: Val::Px(2.0 * DIAL_PX),
                border: UiRect::all(Val::Px(2.0)),
                border_radius: BorderRadius::all(Val::Percent(50.0)),
                ..default()
            },
            UiTransform::IDENTITY,
            BorderColor::all(COLD),
            BackgroundColor(Color::NONE),
            GlobalZIndex(14),
            DespawnOnExit(AppState::Flying),
        ));
    }

    commands
        .spawn((
            AimCrosshair,
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
            // Same gun-cross as the 737's cockpit overlay.
            for node in [
                centered(4.0, 4.0, 0.0, 0.0),
                centered(2.0, 8.0, 0.0, -18.0),
                centered(2.0, 8.0, 0.0, 18.0),
                centered(8.0, 2.0, -18.0, 0.0),
                centered(8.0, 2.0, 18.0, 0.0),
            ] {
                root.spawn((AimTick, node, BackgroundColor(COLD)));
            }
            // A fixed-width flex row keeps the auto-sized hint centred along
            // the bottom edge, clear of the dial in the close-up.
            root.spawn(Node {
                position_type: PositionType::Absolute,
                left: Val::Percent(50.0),
                bottom: Val::Px(36.0),
                width: Val::Px(400.0),
                margin: UiRect {
                    left: Val::Px(-200.0),
                    ..default()
                },
                justify_content: JustifyContent::Center,
                ..default()
            })
            .with_children(|label| {
                label.spawn((
                    AimHint,
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
}

/// A fixed-size box centred on the screen, optionally offset in px.
fn centered(w: f32, h: f32, dx: f32, dy: f32) -> Node {
    Node {
        position_type: PositionType::Absolute,
        left: Val::Percent(50.0),
        top: Val::Percent(50.0),
        width: Val::Px(w),
        height: Val::Px(h),
        margin: UiRect {
            left: Val::Px(-w * 0.5 + dx),
            top: Val::Px(-h * 0.5 + dy),
            ..default()
        },
        ..default()
    }
}

/// Pick the dial under the camera's forward ray, in the airframe body frame.
///
/// Each dial is a 3D disc ([`PanelKey::dial`]); the ray/plane hit is expressed
/// in the disc's own `u`/`v` basis, so the test is exact under free look,
/// airframe attitude and the panel's slight curvature. Nearest hit wins.
fn pick_dial(origin: Vec3, dir: Vec3, root_tf: &Transform) -> Option<PanelKey> {
    let mut best: Option<(f32, PanelKey)> = None;
    for key in ALL {
        let d = key.dial();
        let centre = root_tf.transform_point(d.anchor);
        let u = root_tf.transform_vector(d.u);
        let v = root_tf.transform_vector(d.v);
        let normal = u.cross(v);
        // The dial faces the pilot, so the ray must travel against its normal.
        let denom = dir.dot(normal);
        if denom > -1e-6 {
            continue;
        }
        let t = (centre - origin).dot(normal) / denom;
        if t <= 0.0 {
            continue;
        }
        let p = origin + dir * t - centre;
        let x = p.dot(u) / u.length_squared();
        let y = p.dot(v) / v.length_squared();
        if x * x + y * y > 1.0 {
            continue;
        }
        if best.is_none_or(|(bt, _)| t < bt) {
            best = Some((t, key));
        }
    }
    best.map(|(_, key)| key)
}

/// Aim, input and lens easing.
fn update_focus(
    time: Res<Time>,
    mouse: Res<ButtonInput<MouseButton>>,
    sim: Res<Sim>,
    tune: Res<CockpitTune>,
    mut ui: ResMut<Ui>,
    mut focus: ResMut<InstrumentFocus>,
    cams: Query<(&Camera, &Transform), (With<Camera3d>, Without<crate::cloud_camera::CloudCamera>)>,
    airframe: Query<&Transform, With<AircraftRoot>>,
) {
    let cockpit = sim.name == "c172p" && ui.cockpit && !ui.mcp_focus;
    if !cockpit {
        focus.hovered = None;
        focus.active = None;
        focus.blend = 0.0;
        focus.aim = None;
        focus.entry = (0.0, 0.0);
        ui.freeze_look = false;
        ui.zoom_fov = None;
        return;
    }

    // The centre crosshair *is* the camera's forward ray.
    let aim = match (cams.single(), airframe.single()) {
        (Ok((_, cam_tf)), Ok(root_tf)) => {
            pick_dial(cam_tf.translation, cam_tf.rotation * Vec3::NEG_Z, root_tf)
        }
        _ => None,
    };

    if focus.active.is_some() {
        // The close-up is frozen: only the click-to-return gesture is live.
        focus.hovered = None;
        if mouse.just_pressed(MouseButton::Left) {
            focus.active = None;
        }
    } else {
        focus.hovered = aim;
        if mouse.just_pressed(MouseButton::Left) {
            if let Some(key) = aim {
                focus.entry = (ui.look_yaw, ui.look_pitch);
                focus.active = Some(key);
            }
        }
    }

    // Where the lens should look to put the active dial dead centre. Pure
    // orientation (no eye movement), so nothing can clip through the panel.
    if let Some(key) = focus.active {
        if let Some(eye) = crate::view::cockpit_eye_model("c172p", &tune)
            .map(|e| crate::view::model_to_body("c172p", Vec3::from_array(e)))
        {
            focus.aim = aim_look(eye, key.dial().anchor);
        }
    }

    let target = if focus.active.is_some() { 1.0 } else { 0.0 };
    let was = focus.blend;
    focus.blend = blend_step(focus.blend, target, time.delta_secs().min(0.1));

    // Pan from the gaze that opened the close-up onto the dial and back on
    // exit. The final `a = 0` step restores the entry gaze exactly.
    if focus.blend > 0.0 || (was > 0.0 && focus.blend == 0.0) {
        if let Some((aim_yaw, aim_pitch)) = focus.aim {
            let a = smooth(focus.blend);
            ui.look_yaw = lerp_angle(focus.entry.0, aim_yaw, a);
            ui.look_pitch = focus.entry.1 + (aim_pitch - focus.entry.1) * a;
        }
    }

    // Keep the head still until the lens is fully back: a mouse move during
    // the ease-out would otherwise slide the panel away while it restores.
    ui.freeze_look = focus.blend > 0.0;
    let rest = tune.fov.unwrap_or(REST_FOV);
    ui.zoom_fov = (focus.blend > 0.0).then(|| rest + (ZOOM_FOV - rest) * smooth(focus.blend));
}

/// Interpolate an angle the short way round, so the pan never spins through
/// ±180°.
fn lerp_angle(a: f32, b: f32, t: f32) -> f32 {
    let d = (b - a + std::f32::consts::PI).rem_euclid(std::f32::consts::TAU) - std::f32::consts::PI;
    a + d * t
}

/// Head-look (yaw, pitch) that points the eye at `target`, in the airframe
/// body frame. Inverse of `view::look_rotation`'s forward vector.
fn aim_look(eye: Vec3, target: Vec3) -> Option<(f32, f32)> {
    let dir = target - eye;
    if dir.length_squared() < 1e-9 {
        return None;
    }
    let dir = dir.normalize();
    Some(((-dir.x).atan2(-dir.z), dir.y.clamp(-1.0, 1.0).asin()))
}

/// One eased step toward `target`, snapped once it is visually there.
fn blend_step(current: f32, target: f32, dt: f32) -> f32 {
    let next = current + (target - current) * (1.0 - (-FOV_RATE * dt).exp());
    if (next - target).abs() < 0.002 {
        target
    } else {
        next
    }
}

fn smooth(x: f32) -> f32 {
    let x = x.clamp(0.0, 1.0);
    x * x * (3.0 - 2.0 * x)
}

/// Track every live dial with a ring, dim by default and hot under the
/// crosshair. Hidden while a close-up is active (the ring would be scaled into
/// a thick band) and whenever the cockpit is not on screen.
fn place_rings(
    ui: Res<Ui>,
    focus: Res<InstrumentFocus>,
    cams: Query<(&Camera, &Transform), (With<Camera3d>, Without<crate::cloud_camera::CloudCamera>)>,
    airframe: Query<&Transform, With<AircraftRoot>>,
    mut rings: Query<(&FocusRing, &mut Node, &mut UiTransform, &mut BorderColor)>,
) {
    let show = ui.cockpit && !ui.mcp_focus && focus.active.is_none();
    let view = match (show, cams.single(), airframe.single()) {
        (true, Ok((camera, cam_tf)), Ok(root_tf)) => {
            Some((camera, GlobalTransform::from(*cam_tf), *root_tf))
        }
        _ => None,
    };
    let Some((camera, cam_gt, root_tf)) = view else {
        for (_, mut node, _, _) in &mut rings {
            node.display = Display::None;
        }
        return;
    };
    let Some(size) = camera.logical_viewport_size() else {
        return;
    };

    let r = DIAL_PX;
    for (ring, mut node, mut tf, mut border) in &mut rings {
        let Some(p) = project_dial(camera, &cam_gt, &root_tf, &ring.0.dial()) else {
            node.display = Display::None;
            continue;
        };
        // Cull rings whose dial centre is off screen; a partially visible dial
        // at the edge still rings, which is the useful half of the test.
        if p.centre.x < -r || p.centre.y < -r || p.centre.x > size.x + r || p.centre.y > size.y + r
        {
            node.display = Display::None;
            continue;
        }
        node.display = Display::Flex;
        tf.translation = Val2::px(p.centre.x - r, p.centre.y - r);
        tf.scale = Vec2::new(p.right.length() / r, p.up.length() / r);
        tf.rotation = Rot2::radians(p.right.y.atan2(p.right.x));
        *border = BorderColor::all(if focus.hovered == Some(ring.0) {
            HOT
        } else {
            COLD
        });
    }
}

/// Show the crosshair in the cockpit and the interaction hint under it.
fn sync_overlay(
    ui: Res<Ui>,
    focus: Res<InstrumentFocus>,
    mut root: Query<&mut Node, With<AimCrosshair>>,
    mut ticks: Query<&mut Node, (With<AimTick>, Without<AimCrosshair>)>,
    mut tick_colors: Query<&mut BackgroundColor, With<AimTick>>,
    mut hint: Query<(&mut Text, &mut TextColor), With<AimHint>>,
) {
    let cockpit = ui.cockpit && !ui.mcp_focus;
    let display = if cockpit {
        Display::Flex
    } else {
        Display::None
    };
    for mut node in &mut root {
        if node.display != display {
            node.display = display;
        }
    }
    // The crosshair itself gets out of the way for the close-up; the hint
    // stays so the exit gesture is discoverable.
    let tick_display = if cockpit && focus.active.is_none() {
        Display::Flex
    } else {
        Display::None
    };
    for mut node in &mut ticks {
        if node.display != tick_display {
            node.display = tick_display;
        }
    }
    let color = if focus.hovered.is_some() { HOT } else { COLD };
    for mut bg in &mut tick_colors {
        bg.0 = color;
    }
    if let Ok((mut text, mut tint)) = hint.single_mut() {
        let want = if !cockpit {
            ""
        } else if focus.active.is_some() {
            "CLICK TO RETURN"
        } else if focus.hovered.is_some() {
            "CLICK TO ZOOM"
        } else {
            ""
        };
        if text.0 != want {
            text.0 = want.to_owned();
        }
        // The exit hint needs to read against the panel; hover keeps the
        // crosshair's own colour.
        tint.0 = if focus.active.is_some() { HOT } else { color };
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A ray from the pilot side of `key`'s dial, straight at its centre.
    fn aimed_at(key: PanelKey) -> (Vec3, Vec3) {
        let d = key.dial();
        let normal = d.u.cross(d.v).normalize();
        (d.anchor + normal, -normal)
    }

    #[test]
    fn every_live_dial_can_be_picked() {
        let root = Transform::IDENTITY;
        for key in ALL {
            let (origin, dir) = aimed_at(key);
            assert_eq!(
                pick_dial(origin, dir, &root),
                Some(key),
                "{key:?} was not picked by its own centre ray"
            );
        }
    }

    #[test]
    fn a_ray_beside_the_dial_picks_nothing() {
        let d = PanelKey::Altimeter.dial();
        let normal = d.u.cross(d.v).normalize();
        // Six dial radii above the top row: no face there.
        let origin = d.anchor + normal + d.v.normalize() * d.v.length() * 6.0;
        assert_eq!(pick_dial(origin, -normal, &Transform::IDENTITY), None);
    }

    #[test]
    fn a_ray_from_behind_the_panel_picks_nothing() {
        let d = PanelKey::Airspeed.dial();
        let normal = d.u.cross(d.v).normalize();
        // Looking at the back of the panel must not hit the face.
        assert_eq!(
            pick_dial(d.anchor - normal, normal, &Transform::IDENTITY),
            None
        );
    }

    #[test]
    fn the_lens_eases_and_settles_on_both_ends() {
        let mut blend = 0.0;
        for _ in 0..120 {
            blend = blend_step(blend, 1.0, 1.0 / 60.0);
        }
        assert_eq!(blend, 1.0);
        for _ in 0..120 {
            blend = blend_step(blend, 0.0, 1.0 / 60.0);
        }
        assert_eq!(blend, 0.0);
        assert_eq!(smooth(0.0), 0.0);
        assert_eq!(smooth(1.0), 1.0);
    }

    #[test]
    fn names_round_trip_for_the_screenshot_harness() {
        for key in ALL {
            let name = format!("{key:?}").to_ascii_lowercase();
            assert_eq!(PanelKey::from_name(&name), Some(key));
        }
        assert_eq!(PanelKey::from_name("nope"), None);
    }

    #[test]
    fn the_aim_points_the_camera_at_the_dial() {
        // The C172's calibrated pilot eye, mapped into the airframe frame.
        let eye = crate::view::model_to_body("c172p", Vec3::new(0.209, 1.835, -0.350));
        for key in ALL {
            let (yaw, pitch) = aim_look(eye, key.dial().anchor).expect("a dial aim");
            // `view::look_rotation`'s forward vector for that head-look.
            let forward = Vec3::new(
                -yaw.sin() * pitch.cos(),
                pitch.sin(),
                -yaw.cos() * pitch.cos(),
            );
            let want = (key.dial().anchor - eye).normalize();
            assert!(
                forward.dot(want) > 0.999,
                "{key:?} is not centred: dot {}",
                forward.dot(want)
            );
        }
    }

    #[test]
    fn angle_pans_the_short_way_round() {
        let a = 170.0_f32.to_radians();
        let b = -170.0_f32.to_radians();
        let mid = lerp_angle(a, b, 0.5);
        assert!(
            (mid.abs() - std::f32::consts::PI).abs() < 1e-4,
            "expected ±180°, got {}",
            mid.to_degrees()
        );
    }
}
