//! Scene setup and per-frame visualisation: aircraft mesh, ground grid,
//! chase camera and an on-screen HUD.

use bevy::core_pipeline::tonemapping::Tonemapping;
use bevy::prelude::*;
use bevy::text::FontSize;
use std::collections::HashMap;

use crate::sim::{AircraftState, RenderPose, Sim};
use crate::weather::Weather;

/// Per-aircraft visual config: which mesh to load and how to seat it on the
/// sim's CG (the root the mesh is parented to). All models here use the same
/// mesh axes — +X fwd / +Y up / +Z right — so the fixed +90° yaw aligns every
/// one; only scale and the origin drop differ.
struct ModelSpec {
    /// glTF file in `assets/`.
    file: &'static str,
    /// Uniform scale. Free models come at arbitrary authoring scales (the
    /// rigged F-22 is authored at ~1/6.8; the c172 reads as metres already).
    scale: f32,
    /// Mesh-origin drop below the CG reference (m): the child is translated by
    /// `(0, -drop, 0)`. Set so the model sits on its wheels at rest.
    drop: f32,
    /// Yaw that seats the model's authored forward onto Bevy's -Z. The c172 is
    /// +X-forward (+90°); the rigged F-22 is +Z-forward (+180°). Yaw alone
    /// suffices — both models share the same handedness (+Y up), so no mirror.
    yaw: f32,
}

fn model_spec(name: &str) -> ModelSpec {
    match name {
        // Rigged F-22 (model A): per-surface pivots, driven in `drive_surfaces`.
        // `drop` seats the deployed wheels on the runway: the FDM settles the CG
        // at ~5.56 ft, and the model's lowest gear vertex sits at −0.116 model
        // units, so drop = 1.694 − 6.8·0.116 ≈ 0.90.
        "f22" => ModelSpec {
            file: "f22.glb",
            scale: 6.8,
            drop: 0.90,
            yaw: std::f32::consts::PI,
        },
        // Mi-24 Hind (AH-1S physics stand-in). Authored +Z-forward, so the same
        // 180° yaw as the F-22 seats it on Bevy's -Z. `scale`/`drop` are fitted
        // from the model's geometry (the whole tree is scaled 0.66 by its `Body`
        // node; the skids sit ~0.037 model-units above the origin): drop =
        // rest_h(6 ft) + 0.037·scale ≈ 1.87. Rotor spin lives in `drive_surfaces`.
        "ah1s" => ModelSpec {
            file: "mi-24_hind.glb",
            scale: 1.0,
            drop: 1.87,
            yaw: std::f32::consts::PI,
        },
        // Boeing 737 (`b737.glb`): a Sketchfab model authored in metres (so
        // scale 1.0), nose along -X and origin at the wheels. Its wing,
        // stabilizer and fin are each one smooth shell with no separate
        // control-surface geometry, so ailerons/elevators/rudder don't animate.
        // `drop` = the 737's reset height (49 in = 4.083 ft ≈ 1.2446 m) seats
        // it on the runway; the -90° yaw turns its -X-forward nose onto -Z.
        "737" | "b737" => ModelSpec {
            file: "b737.glb",
            scale: 1.0,
            drop: 1.2446,
            yaw: -std::f32::consts::FRAC_PI_2,
        },
        // Default: the c172, whose control-surface bone rig is driven in
        // `drive_surfaces` (names in `is_control_bone`).
        _ => ModelSpec {
            file: "cessna_172.glb",
            scale: 1.0,
            drop: 1.4326,
            yaw: std::f32::consts::FRAC_PI_2,
        },
    }
}

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

// --- AH-1S / Mi-24 rotor spin ---
// Each rotor spins about its hub axis in the pivot node's LOCAL frame (found
// from the mesh geometry: the main disc lies in XZ so its normal is local Y; the
// tail disc lies in YZ so its normal is local X).
const ROTOR_MAIN_AXIS: [f32; 3] = [0.0, 1.0, 0.0];
const ROTOR_TAIL_AXIS: [f32; 3] = [1.0, 0.0, 0.0];
/// Visual rotor-spin rate per RPM, same convention as `PROP_SPIN`. The rotor
/// turns at ~324 rpm and the tail at ~1660, so the tail visibly outruns it.
const ROTOR_SPIN: f32 = 0.04;

// --- F-22 (`f22.glb`, model A) control pivots ---
// The model authors +X left / +Y up / +Z nose, and its control surfaces hang
// off empty pivot nodes whose names carry Blender's export suffix. Each entry
// is `(hinge axis in the pivot's bind frame, sign)`, the sign folding in the
// per-side mirroring so that `sign * cmd` gives the physically correct motion:
// trailing edge DOWN for a positive elevator/aileron, trailing edge LEFT (+X)
// for a positive rudder — verified from the bind geometry (hinge axis × the
// surface's offset-from-hinge). The pivot names are otherwise opaque; they map
// to roles only by position (`Object_NN` mesh nodes sit under them).
const F22_STAB_L: ([f32; 3], f32) = ([-1.0, 0.0, 0.0], 1.0); // Cube.043_66  (x=+0.47)
const F22_STAB_R: ([f32; 3], f32) = ([1.0, 0.0, 0.0], -1.0); // Cube.044_79  (x=-0.47)
const F22_AIL_LI: ([f32; 3], f32) = ([1.0, 0.0, 0.0], -1.0); // Empty.009_39 (left inboard)
const F22_AIL_LO: ([f32; 3], f32) = ([-1.0, 0.0, 0.0], 1.0); // Empty.010_41 (left outboard)
const F22_AIL_RI: ([f32; 3], f32) = ([1.0, 0.0, 0.0], 1.0); // Empty.011_43  (right inboard)
const F22_AIL_RO: ([f32; 3], f32) = ([-1.0, 0.0, 0.0], -1.0); // Empty.012_45 (right outboard)
const F22_RUD_L: ([f32; 3], f32) = ([-0.05, 0.90, 0.43], -1.0); // Empty.008_37 (left v-tail)
const F22_RUD_R: ([f32; 3], f32) = ([-0.05, -0.91, -0.41], 1.0); // Empty.007_35 (right v-tail)

/// One landing-gear moving part: the node's local pose fully down and fully up,
/// read from the model's gear-cycle animation (its first key = gear down, last
/// key = gear up). `drive_gear` slerps/lerps between them by the live gear
/// position, so the doors and struts fold exactly as the model's author drew
/// them. The model's single clip also opens the weapon bays and swings the
/// boarding ladder, so only the gear-subset nodes appear here.
struct GearPart {
    name: &'static str,
    /// Local rotation (quaternion x,y,z,w) with the gear down.
    down_rot: [f32; 4],
    /// Local rotation with the gear up.
    up_rot: [f32; 4],
    /// Local translation with the gear down.
    down_trans: [f32; 3],
    /// Local translation with the gear up.
    up_trans: [f32; 3],
}

/// F-22 model-A gear nodes: nose strut + doors, both main struts + doors, and
/// the main gear-bay doors. (The weapon-bay doors `Cube.058/061/062/064` and the
/// missiles `Cube.068/069` are deliberately omitted.)
const F22_GEAR: &[GearPart] = &[
    // Nose gear strut and its doors.
    GearPart { name: "Empty.003_12", down_rot: [0.0, 0.0, 0.0, 1.0], up_rot: [-0.766, 0.0, 0.0, 0.643], down_trans: [0.0, 0.045, 0.466], up_trans: [0.0, 0.045, 0.466] },
    GearPart { name: "Empty.004_16", down_rot: [0.0, 0.0, 0.0, 1.0], up_rot: [-0.200, 0.0, 0.0, 0.980], down_trans: [0.0, -0.042, 0.452], up_trans: [0.0, 0.055, 0.552] },
    GearPart { name: "Empty.005_18", down_rot: [0.0, 0.0, 0.0, 1.0], up_rot: [-0.767, 0.0, 0.0, 0.642], down_trans: [0.0, 0.057, 0.426], up_trans: [0.0, 0.025, 0.441] },
    GearPart { name: "Cube.048_91", down_rot: [0.0, 0.0, -0.468, 0.884], up_rot: [0.0, 0.0, 0.0, 1.0], down_trans: [-0.027, 0.032, 0.509], up_trans: [-0.027, 0.032, 0.509] },
    GearPart { name: "Cube.049_92", down_rot: [0.0, 0.0, 0.468, 0.884], up_rot: [0.0, 0.0, 0.0, 1.0], down_trans: [0.027, 0.032, 0.509], up_trans: [0.027, 0.032, 0.509] },
    // Main gear struts.
    GearPart { name: "Cylinder.013_25", down_rot: [0.0, 0.0, 0.0, 1.0], up_rot: [0.0, 0.0, 0.869, 0.494], down_trans: [0.161, 0.037, -0.448], up_trans: [0.161, 0.037, -0.448] },
    GearPart { name: "Cylinder.018_31", down_rot: [0.0, 0.0, 0.0, 1.0], up_rot: [0.0, 0.0, -0.869, 0.494], down_trans: [-0.161, 0.037, -0.448], up_trans: [-0.161, 0.037, -0.448] },
    // Main gear doors.
    GearPart { name: "Cube.055_27", down_rot: [0.0, 0.0, 0.183, 0.983], up_rot: [0.0, 0.0, -0.494, 0.869], down_trans: [0.197, 0.041, -0.448], up_trans: [0.139, 0.066, -0.448] },
    GearPart { name: "Cube.056_28", down_rot: [0.0, 0.0, 0.0, 1.0], up_rot: [0.0, 0.0, -0.667, 0.745], down_trans: [0.180, 0.089, -0.448], up_trans: [0.180, 0.089, -0.448] },
    GearPart { name: "Cube.059_32", down_rot: [0.0, 0.0, 0.193, 0.981], up_rot: [0.0, 0.0, 0.783, 0.622], down_trans: [-0.197, 0.040, -0.448], up_trans: [-0.139, 0.067, -0.448] },
    GearPart { name: "Cube.060_33", down_rot: [0.0, 0.0, -0.790, 0.614], up_rot: [0.0, 0.0, -0.214, 0.977], down_trans: [-0.179, 0.090, -0.448], up_trans: [-0.179, 0.090, -0.448] },
    GearPart { name: "Cube.016_68", down_rot: [0.0, 0.0, 0.906, 0.423], up_rot: [0.0, 0.0, 0.0, 1.0], down_trans: [-0.172, 0.014, -0.444], up_trans: [-0.172, 0.014, -0.444] },
    GearPart { name: "Cube.051_70", down_rot: [0.0, 0.0, -0.906, 0.423], up_rot: [0.0, 0.0, 0.0, 1.0], down_trans: [0.172, 0.014, -0.444], up_trans: [0.172, 0.014, -0.444] },
    GearPart { name: "Cube.050_69", down_rot: [0.0, 0.0, -0.537, 0.843], up_rot: [0.0, 0.0, 0.0, 1.0], down_trans: [-0.300, 0.136, -0.420], up_trans: [-0.300, 0.136, -0.420] },
    GearPart { name: "Cube.052_71", down_rot: [0.0, 0.0, 0.537, 0.843], up_rot: [0.0, 0.0, 0.0, 1.0], down_trans: [0.300, 0.136, -0.420], up_trans: [0.300, 0.136, -0.420] },
];

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
    /// Same, for the helicopter's main and tail rotors (which spin at very
    /// different rates, so each gets its own accumulator).
    main_rotor_phase: f32,
    tail_rotor_phase: f32,
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
                    drive_gear,
                    hide_scene_junk,
                ),
            );
    }
}

fn setup_scene(mut commands: Commands, assets: Res<AssetServer>, sim: Res<Sim>) {
    let spec = model_spec(&sim.name);
    let root = commands
        .spawn((AircraftRoot, Transform::default(), Visibility::default()))
        .id();

    // Aircraft mesh, parented to the sim-driven root. Each model authors its
    // own forward axis (+Y is up for all of them), so the yaw in `spec` seats
    // it onto Bevy's -Z forward.
    commands.spawn((
        WorldAssetRoot(assets.load(GltfAssetLabel::Scene(0).from_asset(spec.file))),
        Transform {
            translation: Vec3::new(0.0, -spec.drop, 0.0),
            rotation: Quat::from_rotation_y(spec.yaw),
            scale: Vec3::splat(spec.scale),
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
        // c172 bone rig (skinned).
        "aileron-left_3" | "aileron-right_2" | "elevator_0" | "rudder_4" | "flaps_1" | "prop_5"
        // F-22 model-A pivots (plain node transforms).
        | "Cube.043_66" | "Cube.044_79"
        | "Empty.009_39" | "Empty.010_41" | "Empty.011_43" | "Empty.012_45"
        | "Empty.007_35" | "Empty.008_37"
        // Mi-24 rotor hubs (spun in `drive_surfaces`).
        | "Top_Rotor" | "Tail_Rotor"
    )
}

/// The rigged F-22 glTF ships a few props that aren't the aircraft: a 1.8 m
/// ground-grid quad (`Object_113`) and a boarding ladder (`Object_89`) parked
/// off the left side. Hide them so they don't fly with the jet. Idempotent.
fn hide_scene_junk(mut q: Query<(&Name, &mut Visibility)>) {
    const JUNK: [&str; 2] = ["Object_113", "Object_89"];
    for (name, mut vis) in &mut q {
        if JUNK.contains(&name.as_str()) && *vis != Visibility::Hidden {
            *vis = Visibility::Hidden;
        }
    }
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
    // Rotors spin likewise, each from its own rotor-rpm signal.
    rig.main_rotor_phase =
        (rig.main_rotor_phase + state.main_rotor_rpm as f32 * ROTOR_SPIN * time.delta_secs())
            .rem_euclid(std::f32::consts::TAU);
    rig.tail_rotor_phase =
        (rig.tail_rotor_phase + state.tail_rotor_rpm as f32 * ROTOR_SPIN * time.delta_secs())
            .rem_euclid(std::f32::consts::TAU);
    let main_rotor_phase = rig.main_rotor_phase;
    let tail_rotor_phase = rig.tail_rotor_phase;
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

            // F-22 model-A pivots. Stabilators take the elevator command; the
            // four flaperons take aileron (per-side sign already in the const,
            // so left drops and right lifts for a positive roll); both v-tails
            // take rudder. (The real jet also rolls with differential tail and
            // vectors with TVC — neither is shown on the mesh.)
            "Cube.043_66" => (F22_STAB_L.0, F22_STAB_L.1 * state.elevator as f32 * MAX_DEFLECT),
            "Cube.044_79" => (F22_STAB_R.0, F22_STAB_R.1 * state.elevator as f32 * MAX_DEFLECT),
            "Empty.009_39" => (F22_AIL_LI.0, F22_AIL_LI.1 * state.aileron as f32 * MAX_DEFLECT),
            "Empty.010_41" => (F22_AIL_LO.0, F22_AIL_LO.1 * state.aileron as f32 * MAX_DEFLECT),
            "Empty.011_43" => (F22_AIL_RI.0, F22_AIL_RI.1 * state.aileron as f32 * MAX_DEFLECT),
            "Empty.012_45" => (F22_AIL_RO.0, F22_AIL_RO.1 * state.aileron as f32 * MAX_DEFLECT),
            "Empty.007_35" => (F22_RUD_R.0, F22_RUD_R.1 * state.rudder as f32 * MAX_DEFLECT),
            "Empty.008_37" => (F22_RUD_L.0, F22_RUD_L.1 * state.rudder as f32 * MAX_DEFLECT),

            // Mi-24 rotors: a constant spin about each hub's local axis.
            "Top_Rotor" => (ROTOR_MAIN_AXIS, main_rotor_phase),
            "Tail_Rotor" => (ROTOR_TAIL_AXIS, tail_rotor_phase),

            _ => continue,
        };
        tf.rotation = *base * Quat::from_axis_angle(Vec3::from_array(axis).normalize(), angle);
    }
}

/// Fold the landing gear between the model's authored down and up poses by the
/// live gear position. The FDM rate-limits the travel over ~5 s, so the blend
/// reads as a smooth cycle without any easing here.
fn drive_gear(state: Res<AircraftState>, mut q: Query<(&Name, &mut Transform)>) {
    if !state.gear_available {
        return;
    }
    // 0 at gear-down, 1 at gear-up.
    let t = (1.0 - state.gear_pos as f32).clamp(0.0, 1.0);
    for (name, mut tf) in &mut q {
        let Some(p) = F22_GEAR.iter().find(|p| p.name == name.as_str()) else {
            continue;
        };
        tf.rotation = Quat::from_array(p.down_rot).slerp(Quat::from_array(p.up_rot), t);
        tf.translation = Vec3::from_array(p.down_trans).lerp(Vec3::from_array(p.up_trans), t);
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
    sim: Res<Sim>,
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

    // Thrust-vectoring status, shown only for aircraft that have it (F-22).
    let tvc = if state.tvc_available {
        if state.tvc_on {
            "   TVC ON"
        } else {
            "   TVC OFF"
        }
    } else {
        ""
    };

    // Landing-gear state, shown only for aircraft whose gear retracts (F-22).
    let gear = if state.gear_available {
        if state.gear_pos > 0.99 {
            "   GEAR DOWN"
        } else if state.gear_pos < 0.01 {
            "   GEAR UP"
        } else {
            "   GEAR ..."
        }
    } else {
        ""
    };

    // Instrument readings — what a real panel shows (RPM, not "throttle %").
    let mut out = format!(
        "myFlightSim — {}   [{mode}]{tvc}{gear}   WX: {}\n\
         ALT {:>4.0} ft   IAS {:>3.0} kt   RPM {:>4.0}   FLAP {:>4.1}°   TRIM {:+.2}\n\
         HDG {:>3.0}°   PITCH {:>5.1}°   ROLL {:>5.1}°",
        sim.label,
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

    // Rotor instrument line, shown only for rotorcraft (the AH-1S).
    if state.rotor_available {
        out.push_str(&format!(
            "\nROTOR {:.0} rpm   TAIL {:.0} rpm   COLL {:.2}   TRQ {:+.0} ft-lb",
            state.main_rotor_rpm, state.tail_rotor_rpm, state.collective, state.torque_lbsft,
        ));
    }

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
        if state.rotor_available {
            out.push_str(&format!(
                "\nCOLL {} {:.2}",
                bar(state.collective), state.collective,
            ));
        }
    }

    out.push_str(
        "\n\n\
         W/S pitch   A/D roll   Q/E yaw   Shift/Ctrl throttle/collective   X/Z trim\n\
         F/V flaps   B brakes   G autopilot   J TVC   U gear   R reset   H inputs   T weather\n\
         Space trim (rotorcraft)",
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
