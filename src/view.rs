//! Scene setup and per-frame visualisation: aircraft mesh, ground grid,
//! chase camera and an on-screen HUD.

use bevy::core_pipeline::tonemapping::Tonemapping;
use bevy::prelude::*;
use bevy::text::FontSize;
use std::collections::HashMap;

use crate::sim::{AircraftState, RenderPose};
use crate::weather::Weather;

/// The only model in `assets/` with a control-surface bone rig (see session
/// notes / `examples/confirm_controls.rs`).
const MODEL: &str = "cessna_172.glb";

/// The model's origin sits at the belly (bind-pose y ≈ 0), but JSBSim's
/// reference point is the CG, ~4.7 ft up when parked. Dropping the mesh by
/// that much puts the wheels on the ground at rest (and keeps it honest in
/// the air, since the offset just tracks CG-above-belly).
const MODEL_DROP: f32 = 1.4326;

/// Hinge axis of each control bone, in the bone's LOCAL (bind-pose) frame.
/// Each is the surface's leading-edge direction (its span, found from the
/// bind-pose geometry) mapped into the bone's local frame via `Rᵀ·a`, where
/// `R` is the bone's bind orientation. The rudder's hinge is its *swept*
/// leading edge (~22° off vertical) — rotating about a plain vertical axis
/// slides the whole rudder instead of hinging it, which reads as the rudder
/// "rotating". The prop spins about the disc normal (model ±X).
const AILERON_L: [f32; 3] = [0.0419, -0.0981, 0.9943];
const AILERON_R: [f32; 3] = [-0.0388, 0.1054, 0.9937];
const ELEVATOR: [f32; 3] = [0.0, 0.0, 1.0];
const RUDDER: [f32; 3] = [-1.0, 0.018, 0.004];
const FLAPS: [f32; 3] = [0.0, 0.0, 1.0];
const PROP: [f32; 3] = [0.0, 0.0, -1.0];

/// Deflection at full command, radians (~20°).
const MAX_DEFLECT: f32 = 0.35;
/// Visual prop-spin rate per RPM. Tuned so cruise reads as a fast blur.
const PROP_SPIN: f32 = 0.01;

#[derive(Component)]
struct AircraftRoot;

#[derive(Component)]
struct Hud;

/// HUD toggles. `H` reveals the control-input overlay.
#[derive(Resource, Default)]
struct Ui {
    show_inputs: bool,
}

/// Bind-pose rotations of the control bones, captured once the glTF scene has
/// loaded. Each frame we re-apply a hinge delta on top of these.
#[derive(Resource, Default)]
struct ControlRig {
    rest: HashMap<Entity, Quat>,
    /// Accumulated visual prop rotation (radians), integrated from rpm so the
    /// spin rate tracks the *instantaneous* rpm.
    prop_phase: f32,
}

pub struct ViewPlugin;

impl Plugin for ViewPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<Ui>()
            .init_resource::<ControlRig>()
            .add_systems(Startup, (setup_scene, setup_hud))
            .add_systems(
                Update,
                (
                    sync_aircraft,
                    follow_camera,
                    update_hud,
                    draw_grid,
                    toggle_ui,
                    dim_lights,
                    capture_rig,
                    drive_surfaces,
                ),
            );
    }
}

fn setup_scene(mut commands: Commands, assets: Res<AssetServer>) {
    let root = commands
        .spawn((AircraftRoot, Transform::default(), Visibility::default()))
        .id();

    // Rigged aircraft, parented to the sim-driven root. The model frame is
    // +X fwd / +Y up / +Z right; Bevy is -Z fwd, so yaw +90° to align.
    commands.spawn((
        WorldAssetRoot(assets.load(GltfAssetLabel::Scene(0).from_asset(MODEL))),
        Transform {
            translation: Vec3::new(0.0, -MODEL_DROP, 0.0),
            rotation: Quat::from_rotation_y(std::f32::consts::FRAC_PI_2),
            scale: Vec3::ONE,
        },
        ChildOf(root),
    ));

    commands.spawn((
        Camera3d::default(),
        // Far plane must clear the sky dome (radius 8000, see `sky.rs`) and the
        // ground grid (half-extent 3000) so neither is clipped.
        Projection::Perspective(PerspectiveProjection {
            far: 20_000.0,
            ..default()
        }),
        // Keep the sky's authored colours (the default filmic tonemapper washes
        // the gradient toward grey).
        Tonemapping::None,
        Transform::from_xyz(-18.0, 8.0, 22.0).looking_at(Vec3::ZERO, Vec3::Y),
        AmbientLight {
            color: Color::WHITE,
            brightness: 250.0,
            ..default()
        },
    ));

    commands.spawn((
        DirectionalLight {
            illuminance: 8_000.0,
            shadow_maps_enabled: false,
            ..default()
        },
        Transform::from_rotation(Quat::from_euler(EulerRot::XYZ, -0.5, -0.6, 0.0)),
    ));
}

fn setup_hud(mut commands: Commands) {
    commands.spawn((
        Text::new(""),
        TextFont {
            font_size: FontSize::Px(16.0),
            ..default()
        },
        TextColor(Color::WHITE),
        Node {
            position_type: PositionType::Absolute,
            top: Val::Px(8.0),
            left: Val::Px(8.0),
            ..default()
        },
        Hud,
    ));
}

/// Place the aircraft between the two latest FDM steps so its motion is smooth
/// regardless of how the render rate divides into the 120 Hz sim.
fn sync_aircraft(
    fixed: Res<Time<Fixed>>,
    rp: Res<RenderPose>,
    mut q: Query<&mut Transform, With<AircraftRoot>>,
) {
    let Ok(mut t) = q.single_mut() else {
        return;
    };
    if !rp.init {
        return;
    }
    let a = fixed.overstep_fraction();
    t.translation = rp.prev_pos.lerp(rp.curr_pos, a);
    t.rotation = rp.prev_rot.slerp(rp.curr_rot, a);
}

/// Grab each control bone's bind-pose rotation the first frame it exists
/// (the glTF scene loads asynchronously, so this can't be done in `Startup`).
fn capture_rig(names: Query<(Entity, &Name, &Transform)>, mut rig: ResMut<ControlRig>) {
    if !rig.rest.is_empty() {
        return;
    }
    for (e, n, t) in &names {
        if is_control_bone(n.as_str()) {
            rig.rest.insert(e, t.rotation);
        }
    }
}

fn is_control_bone(name: &str) -> bool {
    matches!(
        name,
        "aileron-left_3" | "aileron-right_2" | "elevator_0" | "rudder_4" | "flaps_1" | "prop_5"
    )
}

/// Rotate the control bones from the live JSBSim surface commands. Angles are
/// applied on top of the captured bind pose, so surfaces deflect about their
/// real hinges rather than snapping to an absolute orientation.
fn drive_surfaces(
    time: Res<Time>,
    state: Res<AircraftState>,
    mut rig: ResMut<ControlRig>,
    mut q: Query<(&Name, Entity, &mut Transform)>,
) {
    if rig.rest.is_empty() {
        return;
    }
    // Integrate the prop's phase rather than using an absolute angle
    // (`elapsed * rpm`): an absolute phase has rate `rpm + t*d(rpm)/dt`, so it
    // jumps — even spins backward — whenever rpm changes (throttle cut,
    // spool-up), and worse as t grows. Accumulating per frame keeps the visual
    // rate equal to the instantaneous rpm.
    rig.prop_phase = (rig.prop_phase + state.rpm as f32 * PROP_SPIN * time.delta_secs())
        .rem_euclid(std::f32::consts::TAU);
    let prop_phase = rig.prop_phase;
    for (name, e, mut tf) in &mut q {
        let Some(base) = rig.rest.get(&e) else { continue };
        // Signs are set so each surface moves the way JSBSim's command means:
        // a positive angle swings that bone's trailing edge DOWN for the
        // ailerons/elevator/flaps and toward +Z (right) for the rudder
        // (verified by skinning math against the bind pose).
        let (axis, angle) = match name.as_str() {
            // Roll right: right aileron up, left aileron down.
            "aileron-left_3" => (AILERON_L, state.aileron as f32 * MAX_DEFLECT),
            "aileron-right_2" => (AILERON_R, -state.aileron as f32 * MAX_DEFLECT),
            "elevator_0" => (ELEVATOR, state.elevator as f32 * MAX_DEFLECT),
            // +rudder yaws left, so swing the trailing edge toward -Z (left).
            "rudder_4" => (RUDDER, state.rudder as f32 * MAX_DEFLECT),
            // Flap position already comes in degrees (0..~30).
            "flaps_1" => (FLAPS, (state.flap_deg as f32).to_radians()),
            "prop_5" => (PROP, prop_phase),
            _ => continue,
        };
        tf.rotation = *base * Quat::from_axis_angle(Vec3::from_array(axis).normalize(), angle);
    }
}

/// Chase-camera catch-up rate, per second. Higher = tighter / shakier.
const CAM_FOLLOW_RATE: f32 = 4.0;

fn follow_camera(
    time: Res<Time>,
    fixed: Res<Time<Fixed>>,
    rp: Res<RenderPose>,
    mut q: Query<&mut Transform, With<Camera3d>>,
) {
    let Ok(mut t) = q.single_mut() else {
        return;
    };
    if !rp.init {
        return;
    }
    // Track the interpolated pose (not the raw step), or the camera inherits the
    // same hold/jump jitter it's meant to sit behind.
    let a = fixed.overstep_fraction();
    let pos = rp.prev_pos.lerp(rp.curr_pos, a);
    let heading = rp.prev_heading + (rp.curr_heading - rp.prev_heading) * a;

    // Yaw-only offset keeps the horizon level instead of rolling with the plane.
    let yaw = Quat::from_rotation_y(-heading);
    let target = pos + yaw * Vec3::new(0.0, 6.0, 22.0);

    // Ease toward the target so heading changes read as a smooth swing rather
    // than an instant snap. Frame-rate independent exponential smoothing.
    let alpha = 1.0 - (-CAM_FOLLOW_RATE * time.delta_secs()).exp();
    t.translation = t.translation.lerp(target, alpha);
    t.look_at(pos, Vec3::Y);
}

fn toggle_ui(keys: Res<ButtonInput<KeyCode>>, mut ui: ResMut<Ui>) {
    if keys.just_pressed(KeyCode::KeyH) {
        ui.show_inputs = !ui.show_inputs;
    }
}

/// Overcast weather dims the sun and ambient fill (the sky texture greys out on
/// its own, in `sky.rs`). Idempotent, so it just runs every frame.
fn dim_lights(
    weather: Res<Weather>,
    mut sun: Query<&mut DirectionalLight>,
    mut ambient: Query<&mut AmbientLight>,
) {
    let k = 1.0 - 0.55 * weather.rain;
    for mut light in &mut sun {
        light.illuminance = 8_000.0 * k;
    }
    for mut amb in &mut ambient {
        amb.brightness = 250.0 * k;
    }
}

/// Bar filling outward from a centre marker, for a value in [-1, 1].
/// ASCII only, so it renders with Bevy's default font.
fn signed_bar(v: f64) -> String {
    const HALF: usize = 4;
    let v = v.clamp(-1.0, 1.0);
    let n = (v.abs() * HALF as f64).round() as usize;
    let mut cells = ['-'; 2 * HALF + 1];
    cells[HALF] = '|';
    for k in 1..=n {
        cells[if v >= 0.0 { HALF + k } else { HALF - k }] = '#';
    }
    cells.iter().collect()
}

/// Bar filling left to right, for a value in [0, 1].
fn bar(v: f64) -> String {
    const N: usize = 9;
    let n = (v.clamp(0.0, 1.0) * N as f64).round() as usize;
    (0..N).map(|i| if i < n { '#' } else { '-' }).collect()
}

fn update_hud(
    state: Res<AircraftState>,
    ui: Res<Ui>,
    weather: Res<Weather>,
    mut q: Query<&mut Text, With<Hud>>,
) {
    let Ok(mut text) = q.single_mut() else {
        return;
    };
    let mode = if state.autopilot {
        format!("AUTO: {}", state.phase)
    } else {
        "MANUAL".to_string()
    };

    // Instrument readings — what a real panel shows (RPM, not "throttle %").
    let mut out = format!(
        "myFlightSim — JSBSim c172p   [{mode}]   WX: {}\n\
         ALT {:>4.0} ft   IAS {:>3.0} kt   RPM {:>4.0}   FLAP {:>4.1}°   TRIM {:+.2}\n\
         HDG {:>3.0}°   PITCH {:>5.1}°   ROLL {:>5.1}°",
        weather.label(),
        state.alt_ft,
        state.cas_kts,
        state.rpm,
        state.flap_deg,
        state.elevator_trim,
        state.heading_deg.rem_euclid(360.0),
        state.pitch_deg,
        state.roll_deg,
    );

    // Optional control-input overlay. A real cockpit hides this because the
    // pilot feels the forces; a keyboard has no force feedback, so we show it.
    if ui.show_inputs {
        out.push_str(&format!(
            "\n\n\
             ELEV {} {:+.2}   AIL {} {:+.2}   RUD {} {:+.2}\n\
             THR  {}  {:.2}   TRM {} {:+.2}   FLP {}  {:.2}",
            signed_bar(state.elevator), state.elevator,
            signed_bar(state.aileron), state.aileron,
            signed_bar(state.rudder), state.rudder,
            bar(state.throttle), state.throttle,
            signed_bar(state.elevator_trim), state.elevator_trim,
            bar(state.flap_deg / 30.0), state.flap_deg / 30.0,
        ));
    }

    out.push_str(
        "\n\n\
         W/S pitch   A/D roll   Q/E yaw   Shift/Ctrl throttle   X/Z trim\n\
         F/V flaps   B brakes   G autopilot   R reset   H inputs   T weather",
    );

    text.0 = out;
}

fn draw_grid(state: Res<AircraftState>, mut gizmos: Gizmos) {
    const STEP: f32 = 100.0;
    const HALF: f32 = 3000.0;
    const N: i32 = (HALF / STEP) as i32;
    let color = Color::srgba(0.45, 0.55, 0.45, 0.35);

    // Snap the grid to the aircraft's XZ position so it reads as ground.
    let cx = (state.pos.x / STEP).round() * STEP;
    let cz = (state.pos.z / STEP).round() * STEP;
    for i in -N..=N {
        let o = i as f32 * STEP;
        gizmos.line(
            Vec3::new(cx - HALF, 0.0, cz + o),
            Vec3::new(cx + HALF, 0.0, cz + o),
            color,
        );
        gizmos.line(
            Vec3::new(cx + o, 0.0, cz - HALF),
            Vec3::new(cx + o, 0.0, cz + HALF),
            color,
        );
    }
}

#[cfg(test)]
mod tests {
    use super::{bar, signed_bar};

    #[test]
    fn signed_bar_is_centred_and_empty_at_zero() {
        assert_eq!(signed_bar(0.0), "----|----");
    }

    #[test]
    fn signed_bar_fills_right_for_positive_left_for_negative() {
        assert_eq!(signed_bar(1.0), "----|####");
        assert_eq!(signed_bar(-1.0), "####|----");
        assert_eq!(signed_bar(0.5), "----|##--");
    }

    #[test]
    fn signed_bar_clamps_out_of_range_values() {
        assert_eq!(signed_bar(2.0), signed_bar(1.0));
        assert_eq!(signed_bar(-2.0), signed_bar(-1.0));
    }

    #[test]
    fn bar_fills_left_to_right() {
        assert_eq!(bar(0.0), "---------");
        assert_eq!(bar(1.0), "#########");
        assert_eq!(bar(0.5), "#####----");
        assert_eq!(bar(2.0), bar(1.0));
    }
}
