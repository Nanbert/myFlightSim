//! Scene setup and per-frame visualisation: aircraft mesh, ground grid,
//! chase camera and an on-screen HUD.

use bevy::camera::Hdr;
use bevy::core_pipeline::tonemapping::Tonemapping;
use bevy::input::mouse::AccumulatedMouseMotion;
use bevy::post_process::bloom::{Bloom, BloomCompositeMode, BloomPrefilter};
use bevy::post_process::motion_blur::MotionBlur;
use bevy::prelude::*;
use bevy::text::FontSize;
use bevy::window::{CursorGrabMode, CursorOptions};
use std::collections::{HashMap, HashSet};

use crate::aircraft_profile;
use crate::app_state::{unpaused, AppState};
use crate::autopilot::Autopilot;
use crate::input::Bindings;
use crate::sim::{AircraftState, RenderPose, Sim};
use crate::ui_theme;
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
    /// Extra translation in the parent (aircraft body) frame, before the
    /// scale/yaw. Lines up a model whose origin isn't at the CG; zero for the
    /// exterior models, whose origins already sit there.
    offset: [f32; 3],
    /// Yaw that seats the model's authored forward onto Bevy's -Z. The c172 is
    /// +X-forward (+90°); the rigged F-22 is +Z-forward (+180°). Yaw alone
    /// suffices — both models share the same handedness (+Y up), so no mirror.
    yaw: f32,
    /// Pilot-eye position in the model's own frame (+Y up, nose +Z, in the
    /// same units as the glTF, i.e. scaled by `scale` like the mesh). Used by
    /// the cockpit view. `None` for models with no modelled interior; cockpit
    /// mode then just keeps the chase camera.
    eye: Option<[f32; 3]>,
    /// Resting cockpit head-look yaw/pitch (radians) relative to straight
    /// ahead, applied when entering the cockpit and when recentring. Lets a
    /// dialled-in eye keep the panel/HUD framed instead of staring at the
    /// horizon. Positive yaw looks left; positive pitch looks up.
    eye_yaw: f32,
    eye_pitch: f32,
}

/// Does `name`'s model carry a pilot-eye cockpit view? Mirrors the check
/// `follow_camera` makes, so callers can line up with the camera actually used.
pub fn has_cockpit(name: &str) -> bool {
    cockpit_spec(name).is_some()
}

/// Map a point from `name`'s model frame (+X left / +Y up / +Z nose) into the
/// `AircraftRoot` body frame, applying the same scale/yaw/drop `spawn_model_child`
/// uses on the mesh. Lets effect code (see `fx.rs`) place emitters on the
/// airframe using model-space coordinates measured from the glTF.
pub fn model_to_body(name: &str, p: Vec3) -> Vec3 {
    let spec = model_spec(name);
    Quat::from_rotation_y(spec.yaw) * (p * spec.scale) + Vec3::new(0.0, -spec.drop, 0.0)
}

fn model_spec(name: &str) -> ModelSpec {
    match name {
        // Rigged F-22 (model A): per-surface pivots, driven in `drive_surfaces`.
        // `drop` seats the deployed wheels on the runway: the FDM settles the CG
        // at ~5.56 ft, and the model's lowest gear vertex sits at −0.116 model
        // units, so drop = 1.694 − 6.8·0.116 ≈ 0.90. It has no interior; the
        // cockpit view uses a second model (see `cockpit_spec`).
        "f22" => ModelSpec {
            file: "f22.glb",
            scale: 6.8,
            drop: 0.90,
            offset: [0.0; 3],
            yaw: std::f32::consts::PI,
            eye: None,
            eye_yaw: 0.0,
            eye_pitch: 0.0,
        },
        // AH-64D Apache (AH-1S physics stand-in for now). The downloaded model
        // is authored nose +X / up +Z, so it takes the F-22's +90° yaw. It is
        // scaled 7.1× from the source units; `offset.z` puts the rotor mast on
        // the FDM CG and `drop` seats the wheels at the AH-1S rest height
        // (6 ft). `tools/prepare_apache.py` split the main/tail rotors out as
        // `AH64_MainRotor` / `AH64_TailRotor` for `drive_surfaces`.
        "ah1s" => ModelSpec {
            file: "ah64d.glb",
            scale: 7.1,
            drop: -0.599,
            offset: [0.0, 0.0, 1.917],
            yaw: std::f32::consts::FRAC_PI_2,
            eye: None,
            eye_yaw: 0.0,
            eye_pitch: 0.0,
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
            offset: [0.0; 3],
            yaw: -std::f32::consts::FRAC_PI_2,
            eye: None,
            eye_yaw: 0.0,
            eye_pitch: 0.0,
        },
        // Default: the Cessna 172, using the full model (`c172_full.glb`) — the
        // one that carries interior **and** exterior, and so the cockpit view.
        //
        // Authored nose +Z / up +Y (metres) → the F-22's 180° yaw. `scale`
        // shrinks its 12.6 m span to the real 11.0 m; `drop` seats the wheels
        // (lowest vertex −0.060 model units) on the runway at the FDM's rest CG
        // height (1.4326 m = the c172p `root_y`): rest_h + lowest·scale. `eye`
        // is the pilot's eye, dialled in with `examples/cockpit_shot.rs`.
        _ => ModelSpec {
            file: "c172_full.glb",
            scale: 11.0 / 12.598,
            drop: 1.4326 - 0.060 * (11.0 / 12.598),
            offset: [0.0; 3],
            yaw: std::f32::consts::PI,
            // Dialled in live with the `COCKPIT_EYE`/`COCKPIT_LOOK` overrides
            // (raised +10 cm, seat 13 cm forward, 4° rest pitch down): parked
            // level, the cowl keeps a runway/horizon slice above the panel
            // instead of blanketing the whole forward view.
            eye: Some([0.209, 2.0, -0.20]),
            eye_yaw: 0.0,
            eye_pitch: -4.0_f32.to_radians(),
        },
    }
}

/// The model used by the pilot-eye cockpit view. `None` = the aircraft has no
/// modelled interior. Models that carry their interior in the exterior file
/// (the C172) return that same spec; the F-22's exterior is interior-less, so
/// its cockpit view uses a separate download.
fn cockpit_spec(name: &str) -> Option<ModelSpec> {
    match name {
        // F-22 Raptor (bohmerang): authored +X nose / +Y up at ~10 units/m,
        // with separate cockpit, canopy and HUD meshes. Only shown in the
        // cockpit view, so its single welded airframe (no control-surface
        // pivots) can't be seen moving from the chase view. `eye` and
        // `eye_pitch` are in this model's own frame (unlike the metre-authored
        // C172), dialled in with `examples/f22_cockpit_shot.rs`.
        "f22" => Some(ModelSpec {
            file: "f22_cockpit.glb",
            scale: 0.1,
            drop: 0.0,
            // This model's origin sits 4.77 m behind the old model's, so the
            // body is shifted aft to put the two noses (and canopies) in the
            // same place relative to the CG.
            offset: [0.0, 0.0, 4.77],
            yaw: std::f32::consts::FRAC_PI_2,
            eye: Some([96.09, 11.22, 0.12]),
            eye_yaw: -2.1_f32.to_radians(),
            eye_pitch: -3.9_f32.to_radians(),
        }),
        // Boeing 737NG flight deck (sketchfab). Authored nose +Z / up +Y, so
        // the same 180° yaw as the C172. The eye is the captain's, dialled in
        // with `examples/cockpit_eye_calib.rs`: the seat is forward enough that
        // the panel fills the view, with a slight right/level head-turn framing
        // the PFD and MCP together.
        // AH-64D pilot station. The cockpit GLB is authored eye-at-origin,
        // nose +Z; `offset` is the pilot eye in the aircraft body frame
        // (forward ~2 m of the rotor-mast CG, ~1 m above it), and the standard
        // 180° yaw maps +Z onto the body -Z nose.
        "ah1s" => Some(ModelSpec {
            file: "ah64d_cockpit.glb",
            scale: 1.0,
            drop: 0.0,
            offset: [0.0, 1.05, -2.0],
            yaw: std::f32::consts::PI,
            // Eye 0.18 m above the authored origin: enough to see the
            // windscreen/skylights without craning, while the MPDs stay in
            // view when looking down.
            eye: Some([0.0, 0.18, 0.0]),
            eye_yaw: 0.0,
            eye_pitch: -5.0_f32.to_radians(),
        }),
        // `drop` lowers the 737 section so the eye sits ~2.9 m above the runway.
        "737" | "b737" => Some(ModelSpec {
            file: "b737_cockpit.glb",
            scale: 1.0,
            drop: 0.4,
            offset: [0.0; 3],
            yaw: std::f32::consts::PI,
            // The eye sits a hand taller and the rest pose pitches down
            // enough to hold the runway over the glare shield on the ground
            // (the panel still fills the lower view); fine-tune live with
            // `COCKPIT_EYE`/`COCKPIT_LOOK` and bake the numbers here.
            eye: Some([0.2801, 2.14, 0.9355]),
            eye_yaw: -11.76_f32.to_radians(),
            eye_pitch: -6.5_f32.to_radians(),
        }),
        _ => {
            let spec = model_spec(name);
            spec.eye.map(|_| spec)
        }
    }
}

/// Deflection at full command, radians (~20°).
const MAX_DEFLECT: f32 = 0.35;
/// How much of the aileron command the F-22's differential stabilators take.
/// The real jet blends all-moving tail into roll; this is a visual share, not a
/// flight-model term, so it is deliberately below the flaperons' full authority.
const STAB_ROLL_SHARE: f32 = 0.65;
/// Visual prop-spin rate per RPM. Tuned so cruise reads as a fast blur.
const PROP_SPIN: f32 = 0.01;

// --- Cessna 172 (full model, `c172_full.glb`) control surfaces ---
//
// That model has no skeleton: each control surface is a separate glTF node, but
// its geometry is baked in the shared `Cessna_Exterior` frame, so every node's
// origin sits at the model centre — a plain node rotation would spin the surface
// about the middle of the aircraft. Instead each surface is hinged *about its own
// leading edge* with `Transform::rotate_around`, using the line below.
//
// The line lives in the node's parent frame, which for these nodes equals the
// raw glTF/authoring frame: `+X = span (left wing)`, `+Y = DOWN`, `+Z = aft`
// (the parent `Cessna-172` node maps it onto Bevy's +Y-up / +Z-nose frame).
// Measured from the bind-pose vertices (leading-edge centroid + PCA axis).
//
// `sign` folds the polarity so a positive command means "trailing edge DOWN" for
// the flat surfaces, and "trailing edge toward +X (model left)" for the rudder —
// JSBSim's convention. The aileron signs encode the differential: a positive
// `aileron` (roll right) drives the left aileron down and the right aileron up.
struct C172Surface {
    node: &'static str,
    /// A point on the hinge line (parent frame).
    hinge: [f32; 3],
    /// Unit hinge-line direction (parent frame).
    axis: [f32; 3],
    sign: f32,
    src: SurfaceSrc,
}

#[derive(Clone, Copy)]
enum SurfaceSrc {
    Elevator,
    Aileron,
    Rudder,
    /// Already an angle in degrees (0..~30).
    Flap,
    /// The integrated prop phase (radians), not a control command.
    Prop,
}

const C172_SURFACES: &[C172Surface] = &[
    // Rudder: hinge is swept back ~6° off vertical (per the measured axis).
    C172Surface {
        node: "Cessna_Exterior_Body_MAT_0.008",
        hinge: [0.0, -16.254, 470.741],
        axis: [0.0, 0.9941, -0.1077],
        sign: 1.0,
        src: SurfaceSrc::Rudder,
    },
    // Elevators: hinge line along the span at the stabiliser trailing edge.
    C172Surface {
        node: "Cessna_Exterior_Body_MAT_0.013",
        hinge: [0.0, -26.203, 452.426],
        axis: [1.0, 0.0, 0.0],
        sign: -1.0,
        src: SurfaceSrc::Elevator,
    }, // left
    C172Surface {
        node: "Cessna_Exterior_Body_MAT_0.012",
        hinge: [0.0, -26.206, 452.184],
        axis: [1.0, 0.0, 0.0],
        sign: -1.0,
        src: SurfaceSrc::Elevator,
    }, // right
    // Ailerons (outboard) and flaps (inboard) on the wing trailing edge.
    C172Surface {
        node: "Cessna_Exterior_Body_MAT_0.014",
        hinge: [0.0, -84.829, 41.878],
        axis: [1.0, 0.0, 0.0],
        sign: -1.0,
        src: SurfaceSrc::Aileron,
    }, // left
    C172Surface {
        node: "Cessna_Exterior_Body_MAT_0.015",
        hinge: [0.0, -84.463, 41.878],
        axis: [1.0, 0.0, 0.0],
        sign: 1.0,
        src: SurfaceSrc::Aileron,
    }, // right
    C172Surface {
        node: "Cessna_Exterior_Body_MAT_0.016",
        hinge: [0.0, -68.070, 56.418],
        axis: [1.0, 0.0, 0.0],
        sign: -1.0,
        src: SurfaceSrc::Flap,
    }, // left
    C172Surface {
        node: "Cessna_Exterior_Body_MAT_0.017",
        hinge: [0.0, -68.314, 58.158],
        axis: [1.0, 0.0, 0.0],
        sign: -1.0,
        src: SurfaceSrc::Flap,
    }, // right
    // Propeller: spins about the disc normal (raw +Z) through the hub centre.
    C172Surface {
        node: "Cessna_Exterior_Body_MAT_0.001",
        hinge: [0.0, 0.0, 0.0],
        axis: [0.0, 0.0, 1.0],
        sign: 1.0,
        src: SurfaceSrc::Prop,
    },
];

// --- AH-64D rotor spin ---
// Each rotor spins about its hub in the pivot node's LOCAL frame. The main
// disc lies in the model XZ plane (normal local Y). The tail rotor is mounted
// on the side of the fin: its disc lies in the model XY plane, so its normal is
// local Z. The tail pivot is at y = +0.10 in model units, not on the fin
// centreline; `tools/prepare_apache.py` puts the node origin there.
const ROTOR_MAIN_AXIS: [f32; 3] = [0.0, 1.0, 0.0];
const ROTOR_TAIL_AXIS: [f32; 3] = [0.0, 0.0, 1.0];
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
    GearPart {
        name: "Empty.003_12",
        down_rot: [0.0, 0.0, 0.0, 1.0],
        up_rot: [-0.766, 0.0, 0.0, 0.643],
        down_trans: [0.0, 0.045, 0.466],
        up_trans: [0.0, 0.045, 0.466],
    },
    GearPart {
        name: "Empty.004_16",
        down_rot: [0.0, 0.0, 0.0, 1.0],
        up_rot: [-0.200, 0.0, 0.0, 0.980],
        down_trans: [0.0, -0.042, 0.452],
        up_trans: [0.0, 0.055, 0.552],
    },
    GearPart {
        name: "Empty.005_18",
        down_rot: [0.0, 0.0, 0.0, 1.0],
        up_rot: [-0.767, 0.0, 0.0, 0.642],
        down_trans: [0.0, 0.057, 0.426],
        up_trans: [0.0, 0.025, 0.441],
    },
    GearPart {
        name: "Cube.048_91",
        down_rot: [0.0, 0.0, -0.468, 0.884],
        up_rot: [0.0, 0.0, 0.0, 1.0],
        down_trans: [-0.027, 0.032, 0.509],
        up_trans: [-0.027, 0.032, 0.509],
    },
    GearPart {
        name: "Cube.049_92",
        down_rot: [0.0, 0.0, 0.468, 0.884],
        up_rot: [0.0, 0.0, 0.0, 1.0],
        down_trans: [0.027, 0.032, 0.509],
        up_trans: [0.027, 0.032, 0.509],
    },
    // Main gear struts.
    GearPart {
        name: "Cylinder.013_25",
        down_rot: [0.0, 0.0, 0.0, 1.0],
        up_rot: [0.0, 0.0, 0.869, 0.494],
        down_trans: [0.161, 0.037, -0.448],
        up_trans: [0.161, 0.037, -0.448],
    },
    GearPart {
        name: "Cylinder.018_31",
        down_rot: [0.0, 0.0, 0.0, 1.0],
        up_rot: [0.0, 0.0, -0.869, 0.494],
        down_trans: [-0.161, 0.037, -0.448],
        up_trans: [-0.161, 0.037, -0.448],
    },
    // Main gear doors.
    GearPart {
        name: "Cube.055_27",
        down_rot: [0.0, 0.0, 0.183, 0.983],
        up_rot: [0.0, 0.0, -0.494, 0.869],
        down_trans: [0.197, 0.041, -0.448],
        up_trans: [0.139, 0.066, -0.448],
    },
    GearPart {
        name: "Cube.056_28",
        down_rot: [0.0, 0.0, 0.0, 1.0],
        up_rot: [0.0, 0.0, -0.667, 0.745],
        down_trans: [0.180, 0.089, -0.448],
        up_trans: [0.180, 0.089, -0.448],
    },
    GearPart {
        name: "Cube.059_32",
        down_rot: [0.0, 0.0, 0.193, 0.981],
        up_rot: [0.0, 0.0, 0.783, 0.622],
        down_trans: [-0.197, 0.040, -0.448],
        up_trans: [-0.139, 0.067, -0.448],
    },
    GearPart {
        name: "Cube.060_33",
        down_rot: [0.0, 0.0, -0.790, 0.614],
        up_rot: [0.0, 0.0, -0.214, 0.977],
        down_trans: [-0.179, 0.090, -0.448],
        up_trans: [-0.179, 0.090, -0.448],
    },
    GearPart {
        name: "Cube.016_68",
        down_rot: [0.0, 0.0, 0.906, 0.423],
        up_rot: [0.0, 0.0, 0.0, 1.0],
        down_trans: [-0.172, 0.014, -0.444],
        up_trans: [-0.172, 0.014, -0.444],
    },
    GearPart {
        name: "Cube.051_70",
        down_rot: [0.0, 0.0, -0.906, 0.423],
        up_rot: [0.0, 0.0, 0.0, 1.0],
        down_trans: [0.172, 0.014, -0.444],
        up_trans: [0.172, 0.014, -0.444],
    },
    GearPart {
        name: "Cube.050_69",
        down_rot: [0.0, 0.0, -0.537, 0.843],
        up_rot: [0.0, 0.0, 0.0, 1.0],
        down_trans: [-0.300, 0.136, -0.420],
        up_trans: [-0.300, 0.136, -0.420],
    },
    GearPart {
        name: "Cube.052_71",
        down_rot: [0.0, 0.0, 0.537, 0.843],
        up_rot: [0.0, 0.0, 0.0, 1.0],
        down_trans: [0.300, 0.136, -0.420],
        up_trans: [0.300, 0.136, -0.420],
    },
];

#[derive(Component)]
pub struct AircraftRoot;

/// Marks a model whose glTF materials should be matted once its scene spawns.
/// The F-22 ships glossy, partly metallic paint that glares under the sun; see
/// `tame_airframe_materials`.
#[derive(Component)]
struct MatteAirframe;

/// The aircraft's exterior model. Hidden while the cockpit view shows a
/// separate interior-equipped model (see `sync_view_models`).
#[derive(Component)]
struct ExteriorModel;

/// Separate interior-equipped model, shown only in the cockpit view.
#[derive(Component)]
pub struct CockpitModel;

/// The 737 cockpit's captain-side yoke sits at eye level and covers most of
/// the panel from the modelled eye. The sim doesn't draw or animate it (there
/// is no control-wheel input), so hide that glTF node once the scene loads.
#[derive(Component)]
struct Hide737Yoke;

#[derive(Component)]
struct Hud;

/// Which 2D panel is raised over the cockpit. The MCP and the control stand
/// share one slot: only one can hold the mouse and the focus keys at a time.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PanelFocus {
    Mcp,
    Pedestal,
}

/// HUD/toggle state. `H` reveals the control-input overlay; `C` switches the
/// camera between the chase view and (for models that have an interior) the
/// pilot-eye cockpit view.
#[derive(Resource, Default)]
pub struct Ui {
    pub show_inputs: bool,
    pub cockpit: bool,
    /// Cockpit head-look, relative to the airframe: yaw about the body's up
    /// axis and pitch about its lateral axis. Zero looks straight ahead. Only
    /// the camera's *orientation* follows it — the pilot's eye stays put.
    pub look_yaw: f32,
    pub look_pitch: f32,
    /// Seconds left to ignore mouse motion after entering the cockpit. Locking
    /// the cursor can warp it once, which otherwise snaps the view to the roof.
    pub look_settle: f32,
    /// Drop the first non-zero mouse delta after the lock, too; some compositors
    /// deliver the warp a little later than the grab.
    pub look_ignore: bool,
    /// Cockpit only: a 2D panel is raised over the cockpit, which its hero
    /// plugin dims (not clears) so the PFDs stay readable. The mouse is
    /// released for direct clicks, and free-look and the crosshair are
    /// suspended while this is set.
    pub panel_focus: Option<PanelFocus>,
    /// Cockpit only, C172: the head-look is held still while an instrument is
    /// zoomed in, so the close-up stays exactly on the dial.
    pub freeze_look: bool,
    /// Cockpit only: vertical FOV override while the instrument close-up is
    /// eased in. `None` uses the rest lens.
    pub zoom_fov: Option<f32>,
}

impl Ui {
    /// True while any 2D panel holds the cockpit.
    pub fn has_focus(&self) -> bool {
        self.panel_focus.is_some()
    }

    /// True while `panel` is the raised one.
    pub fn is_focused(&self, panel: PanelFocus) -> bool {
        self.panel_focus == Some(panel)
    }
}

/// Live overrides for dialling in the cockpit view, driven by
/// `examples/cockpit_eye_calib.rs`. `None` uses the model spec, so the game
/// behaves exactly as before unless a calibration demo writes to it.
#[derive(Resource, Default)]
pub struct CockpitTune {
    /// Pilot eye in the cockpit model's own frame, same convention as
    /// `ModelSpec::eye`.
    pub eye: Option<[f32; 3]>,
    /// Vertical field of view for the cockpit lens, radians.
    pub fov: Option<f32>,
}

/// Bind-pose transforms of the control surfaces, captured once the glTF scene
/// has loaded. Each frame we re-apply a hinge delta on top of these (a bone
/// rotation for the rigged models, a rotate-about-a-line for the baked ones).
#[derive(Resource, Default)]
pub struct ControlRig {
    /// Control nodes in their bind pose, with their names so the per-frame
    /// driver can dispatch on the surface without scanning every named entity.
    rest: HashMap<Entity, (Name, Transform)>,
    /// F-22 landing-gear nodes, each with its index into [`F22_GEAR`].
    gear: Vec<(Entity, usize)>,
    /// Accumulated visual prop rotation (radians), integrated from rpm so the
    /// spin rate tracks the *instantaneous* rpm.
    prop_phase: f32,
    /// Same, for the helicopter's main and tail rotors (which spin at very
    /// different rates, so each gets its own accumulator).
    main_rotor_phase: f32,
    tail_rotor_phase: f32,
}

/// Smoothed chase-camera offset from the aircraft, in world space. Seated on the
/// first frame the pose exists, then eased toward the desired offset so a
/// heading change swings the view round instead of snapping.
#[derive(Resource, Default)]
pub struct Chase {
    offset: Option<Vec3>,
}

pub struct ViewPlugin;

impl Plugin for ViewPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<Ui>()
            .init_resource::<ControlRig>()
            .init_resource::<Chase>()
            .init_resource::<CockpitTune>()
            .add_systems(Startup, (setup_scene, setup_hud))
            .add_systems(OnExit(AppState::Flying), reset_camera_feel)
            // Enter the chase pose *pinned*: without this the offset left over
            // from the previous flight eases back to the standard framing over
            // the first second — a lazy swing that reads as the camera wobbling
            // right at scene entry, on a plane that hasn't moved.
            .add_systems(OnEnter(AppState::Flying), clear_chase_offset)
            .add_systems(
                Update,
                (
                    sync_aircraft.run_if(unpaused),
                    cockpit_look
                        .run_if(unpaused)
                        .before(follow_camera),
                    (follow_camera, camera_feel)
                        .chain()
                        .run_if(unpaused),
                    update_hud.run_if(unpaused),
                    toggle_ui.run_if(unpaused),
                    sync_view_models.run_if(unpaused),
                    hide_737_yoke.run_if(unpaused),
                    capture_rig.run_if(unpaused),
                    drive_surfaces.run_if(unpaused),
                    drive_gear.run_if(unpaused),
                    draw_grid,
                    dim_lights,
                    hide_scene_junk,
                    // Ungated: the menu showcase flies the same materials.
                    tame_airframe_materials,
                    sync_hud_visibility,
                    // Ungated: it must also run in the menu to release the
                    // cursor when a flight ends.
                    cockpit_cursor,
                ),
            );
    }
}

/// Spawn the model's glTF scene as a child of `parent`, scaled/yawed/dropped per
/// `model_spec`. Shared by the flown aircraft and the menu's showcase row.
pub fn spawn_model_child(
    commands: &mut Commands,
    assets: &AssetServer,
    parent: Entity,
    id: &str,
) -> Entity {
    let model = spawn_model(
        commands,
        assets,
        parent,
        &model_spec(id),
        Visibility::Inherited,
    );
    if id == "f22" {
        commands.entity(model).insert(MatteAirframe);
    }
    model
}

/// Spawn one glTF scene under `parent` with `spec`'s scale/yaw/drop.
fn spawn_model(
    commands: &mut Commands,
    assets: &AssetServer,
    parent: Entity,
    spec: &ModelSpec,
    visibility: Visibility,
) -> Entity {
    commands
        .spawn((
            WorldAssetRoot(assets.load(GltfAssetLabel::Scene(0).from_asset(spec.file))),
            Transform {
                translation: Vec3::from_array(spec.offset) + Vec3::new(0.0, -spec.drop, 0.0),
                rotation: Quat::from_rotation_y(spec.yaw),
                scale: Vec3::splat(spec.scale),
            },
            visibility,
            ChildOf(parent),
        ))
        .id()
}

/// Replace the F-22's glossy paint with a matte finish once the glTF scene has
/// spawned. The materials ship at roughness ~0.3 and partly metallic; under the
/// 8000-lux sun that throws a hard specular glare across the whole canopy area,
/// and bloom turns it into a white smear.
///
/// The scalars alone are not enough: glTF's metallic/roughness factors are
/// *multipliers* on the material's texture, so a glossy texture survives them.
/// Dropping the texture lets the matte scalars take full effect. Glass is left
/// alone — the canopy should still read as glass.
fn tame_airframe_materials(
    mut commands: Commands,
    roots: Query<Entity, With<MatteAirframe>>,
    children: Query<&Children>,
    mesh_materials: Query<&MeshMaterial3d<StandardMaterial>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut matted: Local<HashSet<AssetId<StandardMaterial>>>,
) {
    for root in &roots {
        let mut stack = vec![root];
        let mut found = false;
        while let Some(e) = stack.pop() {
            if let Ok(kids) = children.get(e) {
                stack.extend(kids.iter());
            }
            let Ok(handle) = mesh_materials.get(e) else {
                continue;
            };
            found = true;
            if !matted.insert(handle.0.id()) {
                continue;
            }
            let Some(mut mat) = materials.get_mut(&handle.0) else {
                continue;
            };
            if mat.alpha_mode != AlphaMode::Opaque {
                continue;
            }
            mat.metallic = 0.05;
            mat.perceptual_roughness = 0.65;
            mat.metallic_roughness_texture = None;
        }
        // The scene spawns atomically, so one mesh means all of them are here.
        if found {
            commands.entity(root).remove::<MatteAirframe>();
        }
    }
}

/// Spawn the flown aircraft: one `AircraftRoot` (driven by `RenderPose`) with
/// the sim's current model under it. Runs on entering `Flying`; torn down on
/// exit so it never coexists with the menu's showcase row.
pub fn spawn_flight_aircraft(
    mut commands: Commands,
    assets: Res<AssetServer>,
    sim: Res<Sim>,
    mut ui: ResMut<Ui>,
    mut tune: ResMut<CockpitTune>,
) {
    // A fresh flight opens with the cockpit head-look at its rest pose, and no
    // MCP focus left over from a previous cockpit visit.
    (ui.look_yaw, ui.look_pitch) = rest_look(&sim.name);
    // Development override for screenshot passes: COCKPIT_LOOK=yaw,pitch
    // (degrees), the same fields the mouse drives.
    if let Ok(v) = std::env::var("COCKPIT_LOOK") {
        let p: Vec<f32> = v.split(',').filter_map(|n| n.trim().parse().ok()).collect();
        if p.len() == 2 {
            (ui.look_yaw, ui.look_pitch) = (p[0].to_radians(), p[1].to_radians());
        }
    }
    ui.cockpit = std::env::var("COCKPIT").is_ok_and(|v| v == "1");
    ui.look_settle = if ui.cockpit { 1.0 } else { 0.0 };
    ui.look_ignore = ui.cockpit;
    ui.panel_focus = None;
    // No instrument close-up carries across flights.
    ui.freeze_look = false;
    ui.zoom_fov = None;
    // Development overrides for dialling the eye/FOV from a screenshot shell:
    // COCKPIT_EYE=x,y,z (model frame, metres) and COCKPIT_FOV=degrees.
    tune.eye = std::env::var("COCKPIT_EYE").ok().and_then(|v| {
        let p: Vec<f32> = v.split(',').filter_map(|n| n.trim().parse().ok()).collect();
        (p.len() == 3).then(|| [p[0], p[1], p[2]])
    });
    tune.fov = std::env::var("COCKPIT_FOV")
        .ok()
        .and_then(|v| v.parse::<f32>().ok())
        .map(f32::to_radians)
        // FOV defaults per airframe: an in-cockpit pilot takes in far more
        // world than Bevy's 45° telephoto default, which makes the C172's
        // panel loom over the few degrees the eye has to spare over it.
        .or_else(|| {
            let fov_deg = match crate::aircraft_profile::normalized(&sim.name) {
                "ah1s" => 65.0_f32,
                "c172" | "c172p" => 60.0,
                _ => return None,
            };
            Some(fov_deg.to_radians())
        });
    let root = commands
        .spawn((
            AircraftRoot,
            Transform::default(),
            Visibility::default(),
            DespawnOnExit(AppState::Flying),
        ))
        .id();
    let exterior = spawn_model_child(&mut commands, &assets, root, &sim.name);
    commands.entity(exterior).insert(ExteriorModel);
    if let Some(spec) = cockpit_spec(&sim.name) {
        // Models whose exterior file already carries the interior (the C172)
        // need no second spawn; the F-22's does.
        if spec.file != model_spec(&sim.name).file {
            let cockpit = spawn_model(&mut commands, &assets, root, &spec, Visibility::Hidden);
            commands.entity(cockpit).insert(CockpitModel);
            if sim.name == "737" {
                commands.entity(cockpit).insert(Hide737Yoke);
            }
        }
    }
}

/// Show one model per view: the cockpit-view model replaces the exterior one
/// while the camera sits inside (models with a built-in interior have no
/// separate cockpit model, so their exterior stays visible).
fn sync_view_models(
    ui: Res<Ui>,
    mut exterior: Query<&mut Visibility, (With<ExteriorModel>, Without<CockpitModel>)>,
    mut cockpit: Query<&mut Visibility, (With<CockpitModel>, Without<ExteriorModel>)>,
) {
    let split = !cockpit.is_empty();
    for mut visibility in &mut exterior {
        *visibility = if ui.cockpit && split {
            Visibility::Hidden
        } else {
            Visibility::Inherited
        };
    }
    for mut visibility in &mut cockpit {
        // The cockpit stays on screen while the MCP panel is up: `mcp_panel`
        // dims it with a translucent scrim, so the PFDs stay readable behind
        // the panel rather than swapping to a black room.
        *visibility = if ui.cockpit {
            Visibility::Inherited
        } else {
            Visibility::Hidden
        };
    }
}

/// Hide the 737 cockpit's captain yoke once its glTF subtree has spawned.
/// The glTF scene loads asynchronously as a child of the marker entity, so
/// this walks the subtree each frame and clears the marker when it lands.
fn hide_737_yoke(
    mut commands: Commands,
    roots: Query<(Entity, &Children), With<Hide737Yoke>>,
    mut nodes: Query<(&Name, &mut Visibility)>,
    children: Query<&Children>,
) {
    for (root, kids) in &roots {
        let mut stack: Vec<Entity> = kids.iter().collect();
        let mut found = false;
        while let Some(e) = stack.pop() {
            if let Ok((name, mut visibility)) = nodes.get_mut(e) {
                if name.as_str() == "Cube.488" {
                    *visibility = Visibility::Hidden;
                    found = true;
                }
            }
            if let Ok(kids) = children.get(e) {
                stack.extend(kids.iter());
            }
        }
        if found {
            commands.entity(root).remove::<Hide737Yoke>();
        }
    }
}

/// Drop the captured bind poses so the next aircraft's control bones are captured
/// fresh (the glTF scene loads async, so `capture_rig` can't run on spawn).
pub fn reset_control_rig(mut rig: ResMut<ControlRig>) {
    *rig = ControlRig::default();
}

/// The camera and lights are the one fixed part of the scene, shared by the menu
/// and the flight. The aircraft itself is spawned per state (see
/// `spawn_flight_aircraft` and `menu::spawn_hangar`).
fn setup_scene(mut commands: Commands) {
    commands.spawn((
        Camera3d::default(),
        // WebGL2 has no multisampled immutable textures — wgpu's glow backend
        // panics the tab the moment one is created (the 4× engine default
        // does it) — so the web build flies with MSAA off. The cloud camera
        // already opts out unconditionally (see `sky.rs`).
        #[cfg(target_arch = "wasm32")]
        bevy::render::view::Msaa::Off,
        // Far plane must clear the sky dome (radius 60 km, see `sky.rs`) and
        // the 96 km mission terrain, which is hazed out well before its edge.
        Projection::Perspective(PerspectiveProjection {
            far: 80_000.0,
            ..default()
        }),
        // Keep the sky's authored colours (the default filmic tonemapper washes
        // the gradient toward grey).
        Tonemapping::None,
        // Speed cue: `camera_feel` drives the shutter angle from airspeed. At
        // rest it is 0 (no blur); the component stays put so the render pipeline
        // never changes shape as the jet accelerates.
        MotionBlur {
            shutter_angle: 0.0,
            samples: 1,
        },
        // HDR keeps the exhaust's emissive values above 1 all the way to the
        // bloom pass (without it the flame clips to flat white before it can
        // spread). The prefilter threshold sits above 1 so only the flame's
        // >1 hot core glows — the sky and the sunlit airframe stay untouched.
        // The F-22's paint is matted at runtime (see `tame_airframe_materials`),
        // so the remaining intensity is a gentle halo, not a screen wash.
        Hdr,
        Bloom {
            intensity: 0.5,
            low_frequency_boost: 0.35,
            high_pass_frequency: 1.0,
            prefilter: BloomPrefilter {
                threshold: 1.5,
                threshold_softness: 0.5,
            },
            composite_mode: BloomCompositeMode::Additive,
            ..Bloom::NATURAL
        },
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

/// Show the HUD only while flying; hide it in the menu (otherwise the last
/// frame's text sits over the menu).
fn sync_hud_visibility(state: Res<State<AppState>>, q: Query<&mut Node, With<Hud>>) {
    ui_theme::set_display(
        q,
        if *state.get() == AppState::Flying {
            Display::Flex
        } else {
            Display::None
        },
    );
}

/// Place the aircraft between the two latest FDM steps so its motion is smooth
/// regardless of how the render rate divides into the 120 Hz sim.
pub fn sync_aircraft(
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

/// Grab each control surface's bind-pose transform the first frame it exists
/// (the glTF scene loads asynchronously, so this can't be done in `Startup`).
fn capture_rig(names: Query<(Entity, &Name, &Transform)>, mut rig: ResMut<ControlRig>) {
    if !rig.rest.is_empty() {
        return;
    }
    for (e, n, t) in &names {
        if is_control_bone(n.as_str()) {
            rig.rest.insert(e, (n.clone(), *t));
        }
        if let Some(i) = F22_GEAR.iter().position(|p| p.name == n.as_str()) {
            rig.gear.push((e, i));
        }
    }
}

fn is_control_bone(name: &str) -> bool {
    matches!(
        name,
        // F-22 model-A pivots (plain node transforms).
        "Cube.043_66" | "Cube.044_79"
        | "Empty.009_39" | "Empty.010_41" | "Empty.011_43" | "Empty.012_45"
        | "Empty.007_35" | "Empty.008_37"
        // AH-64D rotor hubs (spun in `drive_surfaces`).
        | "AH64_MainRotor" | "AH64_TailRotor"
    ) || C172_SURFACES.iter().any(|s| s.node == name)
}

/// The rigged F-22 glTF ships a few props that aren't the aircraft: a 1.8 m
/// ground-grid quad (`Object_113`) and a boarding ladder (`Object_89`) parked
/// off the left side. Hide them so they don't fly with the jet. Idempotent.
fn hide_scene_junk(mut q: Query<(&Name, &mut Visibility), Added<Name>>) {
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
    mut q: Query<&mut Transform>,
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
    rig.main_rotor_phase = (rig.main_rotor_phase
        + state.main_rotor_rpm as f32 * ROTOR_SPIN * time.delta_secs())
    .rem_euclid(std::f32::consts::TAU);
    rig.tail_rotor_phase = (rig.tail_rotor_phase
        + state.tail_rotor_rpm as f32 * ROTOR_SPIN * time.delta_secs())
    .rem_euclid(std::f32::consts::TAU);
    let main_rotor_phase = rig.main_rotor_phase;
    let tail_rotor_phase = rig.tail_rotor_phase;
    // Differential stabilator: pitch and roll share the same all-moving tail.
    // A positive aileron command (right roll) lifts the right tail surface and
    // drops the left one, matching the flaperons.
    let stab_l = (state.elevator as f32 + STAB_ROLL_SHARE * state.aileron as f32).clamp(-1.0, 1.0);
    let stab_r = (state.elevator as f32 - STAB_ROLL_SHARE * state.aileron as f32).clamp(-1.0, 1.0);
    for (e, (name, base)) in &rig.rest {
        let Ok(mut tf) = q.get_mut(*e) else {
            continue;
        };
        let name = name.as_str();

        // Baked (unrigged) models: hinge about a measured line, not the node's
        // origin, so the translation moves too.
        if let Some(s) = C172_SURFACES.iter().find(|s| s.node == name) {
            let angle = match s.src {
                // The C172's elevator reads the actual surface angle, so a
                // pitch trim that leaves the yoke centred still shows on the
                // tail. Aileron/rudder have no cockpit trim on the real 172.
                SurfaceSrc::Elevator => s.sign * state.elevator_pos_rad as f32,
                SurfaceSrc::Aileron => s.sign * state.aileron as f32 * MAX_DEFLECT,
                SurfaceSrc::Rudder => s.sign * state.rudder as f32 * MAX_DEFLECT,
                SurfaceSrc::Flap => s.sign * (state.flap_deg as f32).to_radians(),
                SurfaceSrc::Prop => s.sign * prop_phase,
            };
            let axis = Vec3::from_array(s.axis).normalize();
            let hinge = Vec3::from_array(s.hinge);
            let mut t = *base;
            t.rotate_around(hinge, Quat::from_axis_angle(axis, angle));
            *tf = t;
            continue;
        }

        // Rigged models: each bone's origin is its hinge, so a plain rotation on
        // top of the bind pose is enough. The F-22's per-surface signs live in
        // the axis consts (see `F22_*`).
        let (axis, angle) = match name {
            // F-22 model-A pivots. Stabilators take pitch plus a share of roll
            // (the real jet's differential tail); the four flaperons take the
            // rest of the aileron command (per-side sign already in the const,
            // so left drops and right lifts for a positive roll); both v-tails
            // take rudder. TVC is animated on the nozzles in `fx.rs`.
            "Cube.043_66" => (F22_STAB_L.0, F22_STAB_L.1 * stab_l * MAX_DEFLECT),
            "Cube.044_79" => (F22_STAB_R.0, F22_STAB_R.1 * stab_r * MAX_DEFLECT),
            "Empty.009_39" => (
                F22_AIL_LI.0,
                F22_AIL_LI.1 * state.aileron as f32 * MAX_DEFLECT,
            ),
            "Empty.010_41" => (
                F22_AIL_LO.0,
                F22_AIL_LO.1 * state.aileron as f32 * MAX_DEFLECT,
            ),
            "Empty.011_43" => (
                F22_AIL_RI.0,
                F22_AIL_RI.1 * state.aileron as f32 * MAX_DEFLECT,
            ),
            "Empty.012_45" => (
                F22_AIL_RO.0,
                F22_AIL_RO.1 * state.aileron as f32 * MAX_DEFLECT,
            ),
            "Empty.007_35" => (F22_RUD_R.0, F22_RUD_R.1 * state.rudder as f32 * MAX_DEFLECT),
            "Empty.008_37" => (F22_RUD_L.0, F22_RUD_L.1 * state.rudder as f32 * MAX_DEFLECT),

            // AH-64D rotors: a constant spin about each hub's local axis.
            "AH64_MainRotor" => (ROTOR_MAIN_AXIS, main_rotor_phase),
            "AH64_TailRotor" => (ROTOR_TAIL_AXIS, tail_rotor_phase),

            _ => continue,
        };
        tf.rotation =
            base.rotation * Quat::from_axis_angle(Vec3::from_array(axis).normalize(), angle);
    }
}

/// Fold the landing gear between the model's authored down and up poses by the
/// live gear position. The FDM rate-limits the travel over ~5 s, so the blend
/// reads as a smooth cycle without any easing here.
fn drive_gear(
    state: Res<AircraftState>,
    rig: Res<ControlRig>,
    mut q: Query<&mut Transform>,
) {
    if !state.gear_available {
        return;
    }
    // 0 at gear-down, 1 at gear-up.
    let t = (1.0 - state.gear_pos as f32).clamp(0.0, 1.0);
    for &(e, i) in &rig.gear {
        let Ok(mut tf) = q.get_mut(e) else {
            continue;
        };
        let p = &F22_GEAR[i];
        tf.rotation = Quat::from_array(p.down_rot).slerp(Quat::from_array(p.up_rot), t);
        tf.translation = Vec3::from_array(p.down_trans).lerp(Vec3::from_array(p.up_trans), t);
    }
}

/// Chase-camera catch-up rate, per second. Higher = tighter / shakier.
const CAM_FOLLOW_RATE: f32 = 4.0;

/// How much of the airframe's bank the chase camera takes on. Keeping the
/// horizon level is what makes a hard roll read as the *world* pivoting; a
/// little coupling makes it read as the jet rolling. Blended toward world-up, so
/// inverted flight never flips the view.
const CAM_BANK_COUPLING: f32 = 0.25;

/// Airspeed (kt) the camera buffet starts at and is full at. The airframe — and
/// so the view — shakes as dynamic pressure builds.
const SHAKE0_KT: f32 = 120.0;
const SHAKE1_KT: f32 = 420.0;
/// Airspeed (kt) the buffet's angle-of-attack share gates on. Below this band
/// the wing carries no load worth shaking over, and alpha is only drift noise.
const AIRFLOW0_KT: f32 = 30.0;
/// Airspeed (kt) where the alpha share is fully trusted.
const AIRFLOW1_KT: f32 = 70.0;
/// Peak buffet amplitude at full weight: metres of camera position and degrees
/// of camera rotation. Small on purpose — a shake you notice, not one you fight.
const SHAKE_POS_M: f32 = 0.30;
const SHAKE_ROT_DEG: f32 = 0.40;
/// Degrees the vertical FOV widens at speed, on a slight ease. Makes the world
/// sweep past faster without the fisheye look of a big boost.
const FOV_BOOST_DEG: f32 = 5.0;
/// Peak motion-blur shutter angle at speed (0 = none). The camera is pinned to
/// the jet, so this mostly streaks the ground past underneath — the clearest
/// low-level speed cue.
const BLUR_MAX: f32 = 0.35;

/// Cockpit head-look sensitivity, in degrees of head rotation per pixel of
/// mouse motion.
const LOOK_DEG_PER_PX: f32 = 0.12;

/// How far the pilot can look up/down before the neck stops (radians). Yaw is
/// unlimited, so you can turn right around and watch your own tail.
const LOOK_PITCH_LIMIT: f32 = 80.0_f32.to_radians();

/// Cockpit head-look as a rotation in the airframe body frame: yaw about the
/// body's up axis, then pitch about its lateral axis. Zero = straight ahead.
/// Composed *after* the airframe attitude (see `follow_camera`), so a head-turn
/// reads relative to the cockpit and rolls with the plane.
pub fn look_rotation(ui: &Ui) -> Quat {
    Quat::from_rotation_y(ui.look_yaw) * Quat::from_rotation_x(ui.look_pitch)
}

/// Fold one frame of mouse motion into the cockpit head-look. Screen-right
/// turns the view right (a negative yaw about body-up, since the camera looks
/// down its own -Z); screen-up pitches the view up (mouse y grows downward).
/// Pitch is clamped to the neck's range; yaw is unlimited.
fn apply_look_delta(ui: &mut Ui, delta: Vec2) {
    let k = LOOK_DEG_PER_PX.to_radians();
    ui.look_yaw -= delta.x * k;
    ui.look_pitch = (ui.look_pitch - delta.y * k).clamp(-LOOK_PITCH_LIMIT, LOOK_PITCH_LIMIT);
}

/// True while a cockpit panel is focused: Esc belongs to the panel, so the
/// pause menu stands aside until it closes.
pub fn panel_focused(ui: Option<&Ui>) -> bool {
    ui.is_some_and(|ui| ui.has_focus())
}

/// Cockpit free-look: mouse motion turns the pilot's head relative to the
/// airframe. Position is untouched (that stays on `follow_camera`'s eye), so
/// this only changes where you're looking. Inert unless in the cockpit view.
fn cockpit_look(time: Res<Time>, motion: Res<AccumulatedMouseMotion>, mut ui: ResMut<Ui>) {
    if !ui.cockpit || ui.has_focus() || ui.freeze_look {
        return;
    }
    // Development screenshot hook: hold the rest gaze while the desktop mouse
    // is being used by other work.
    if std::env::var("LOOK_LOCK").is_ok() {
        return;
    }
    if ui.look_settle > 0.0 {
        ui.look_settle = (ui.look_settle - time.delta_secs()).max(0.0);
        return;
    }
    let d = motion.delta;
    if d != Vec2::ZERO {
        if ui.look_ignore {
            ui.look_ignore = false;
        } else {
            apply_look_delta(&mut ui, d);
        }
    }
}

/// Keep the mouse cursor out of the way in the cockpit view: hidden and locked
/// to the window so free-look gets unbounded motion. Released everywhere else
/// (menu, chase view, focused MCP, or an unfocused window, where a lock would
/// strand it).
fn cockpit_cursor(
    state: Res<State<AppState>>,
    mut ui: ResMut<Ui>,
    mut windows: Query<(&mut CursorOptions, &Window)>,
) {
    let cockpit = *state.get() == AppState::Flying && ui.cockpit;
    let free_look = cockpit && !ui.has_focus();
    for (mut cursor, window) in &mut windows {
        let grab = if free_look && window.focused {
            CursorGrabMode::Locked
        } else {
            CursorGrabMode::None
        };
        let visible = !free_look;
        if cursor.grab_mode != grab || cursor.visible != visible {
            // The compositor warps the pointer when it locks it. Start the
            // settle window here, even if the camera was already in cockpit
            // mode for a frame or two.
            if grab == CursorGrabMode::Locked {
                ui.look_settle = ui.look_settle.max(1.0);
                ui.look_ignore = true;
            }
            cursor.grab_mode = grab;
            cursor.visible = visible;
        }
    }
}

/// Pilot-eye pose for the cockpit view: convert the model-frame eye through the
/// same scale/yaw/drop transform `spawn_model_child` applies, giving the offset
/// in the `AircraftRoot` body frame (so the camera inherits the airframe's full
/// attitude and rolls with the plane).
fn cockpit_eye(spec: &ModelSpec, eye: Option<[f32; 3]>) -> Option<Vec3> {
    eye.or(spec.eye).map(|e| {
        Vec3::from_array(spec.offset)
            + Quat::from_rotation_y(spec.yaw) * (Vec3::from_array(e) * spec.scale)
            + Vec3::new(0.0, -spec.drop, 0.0)
    })
}

/// Cockpit head-look at rest for `name`: the yaw/pitch the pilot-eye pose was
/// dialled in with.
fn rest_look(name: &str) -> (f32, f32) {
    cockpit_spec(name).map_or((0.0, 0.0), |spec| (spec.eye_yaw, spec.eye_pitch))
}

/// Pilot eye in the cockpit model's own frame, honouring the calibration
/// override. Used by cockpit picking code that works in model space.
pub fn cockpit_eye_model(name: &str, tune: &CockpitTune) -> Option<[f32; 3]> {
    cockpit_spec(name).and_then(|spec| tune.eye.or(spec.eye))
}

/// The cockpit model's mounted yaw (model frame -> body frame). Picking code
/// needs it to express the camera's body-frame look in model space.
pub fn cockpit_model_yaw(name: &str) -> Option<f32> {
    cockpit_spec(name).map(|spec| spec.yaw)
}

pub fn follow_camera(
    time: Res<Time>,
    fixed: Res<Time<Fixed>>,
    rp: Res<RenderPose>,
    sim: Res<Sim>,
    ui: Res<Ui>,
    tune: Res<CockpitTune>,
    mut chase: ResMut<Chase>,
    mut q: Query<&mut Transform, (With<Camera3d>, Without<crate::cloud_camera::CloudCamera>)>,
) {
    let Ok(mut t) = q.single_mut() else {
        return;
    };
    if !rp.init {
        return;
    }
    let a = fixed.overstep_fraction();
    let pos = rp.prev_pos.lerp(rp.curr_pos, a);
    let rot = rp.prev_rot.slerp(rp.curr_rot, a);

    // Cockpit view: pin the camera at the modelled pilot eye and copy the
    // airframe's interpolated attitude, so the panel and horizon move with the
    // plane. Falls through to the chase view when the model has no interior.
    if ui.cockpit {
        if let Some(eye) = cockpit_spec(&sim.name).and_then(|spec| cockpit_eye(&spec, tune.eye)) {
            t.translation = pos + rot * eye;
            // Head-look turns the view *relative to the airframe*; the eye
            // offset above is unrotated by it, so the camera position is fixed.
            t.rotation = rot * look_rotation(&ui);
            return;
        }
    }

    // Track the interpolated pose (not the raw step), or the camera inherits the
    // same hold/jump jitter it's meant to sit behind.
    let heading = rp.prev_heading + (rp.curr_heading - rp.prev_heading) * a;

    // Yaw-only offset keeps the horizon level instead of rolling with the plane.
    let yaw = Quat::from_rotation_y(-heading);
    let want = yaw * Vec3::new(0.0, 6.0, 22.0);

    // Ease the *offset*, not the camera's world position. Smoothing the position
    // leaves the camera trailing its target by `speed / CAM_FOLLOW_RATE` — a lag
    // that dwarfs the 22 m the framing is built from once the jet is fast, so
    // adding power visibly pulls the view back and holds it there. Easing the
    // offset instead pins the camera to the aircraft's interpolated pose, so the
    // framing survives any speed, while a heading change still swings round
    // smoothly rather than snapping.
    let alpha = 1.0 - (-CAM_FOLLOW_RATE * time.delta_secs()).exp();
    let offset = match chase.offset {
        Some(prev) => prev.lerp(want, alpha),
        None => want,
    };
    chase.offset = Some(offset);
    t.translation = pos + offset;
    // Take on a fraction of the airframe's bank, blended toward world-up so an
    // inverted jet still gives a usable view. Without any coupling a hard roll
    // reads as the world pivoting around a static jet; with all of it the view
    // is just the cockpit again.
    let up = (Vec3::Y + (rot * Vec3::Y - Vec3::Y) * CAM_BANK_COUPLING).normalize();
    t.look_at(pos, up);
}

/// How hard the airframe — and so the view — is being buffeted, 0..1.
///
/// Dynamic pressure is the dominant term: the ramp from `SHAKE0_KT` to
/// `SHAKE1_KT`. High angle of attack adds its own share (stalled / separated
/// flow looks far rougher than the airspeed alone suggests), but capped below
/// full so a slow high-alpha hold still reads as speed-related. Take the worse
/// of the two: either one alone can shake the jet.
///
/// The alpha share is *gated on airflow*. Near a standstill the FDM's
/// `aero/alpha` is `w.atan2(u)` of drift noise — a parked plane settling and
/// creeping metres-millimetres-per-second flips the reading around ±90°/±180°
/// and sweeps through the stall band, and the raw alpha term then eased the
/// chase lens between its rest and boosted FOV for minutes: the C172's
/// cold-start sit grew a slow zoom that came and went for no visible reason.
/// The gate ramps in over the gentle-approach band (`AIRFLOW0_KT`..full by
/// `AIRFLOW1_KT`) so a real low-speed stall still bites, while taxiing and
/// parking — where the wing carries no load — can't.
fn buffet_weight(cas_kts: f64, alpha_deg: f64) -> f32 {
    let speed01 = ((cas_kts as f32 - SHAKE0_KT) / (SHAKE1_KT - SHAKE0_KT)).clamp(0.0, 1.0);
    let airflow01 = ((cas_kts as f32 - AIRFLOW0_KT) / (AIRFLOW1_KT - AIRFLOW0_KT)).clamp(0.0, 1.0);
    let alpha01 = (((alpha_deg as f32) - 12.0) / 18.0).clamp(0.0, 1.0) * 0.6 * airflow01;
    speed01.max(alpha01)
}

/// Clear the eased chase offset so the next flight's first frame sits at the
/// standard framing instead of gliding there from wherever the last one left it.
fn clear_chase_offset(mut chase: ResMut<Chase>) {
    chase.offset = None;
}

/// Undo the parts of `camera_feel` that outlive a flight — the widened lens and
/// the motion blur — so the menu isn't left looking like it's at 400 kt. The
/// shake itself needs no cleanup: `follow_camera` rewrites the pose every frame.
fn reset_camera_feel(
    mut q: Query<
        (&mut Projection, Option<&mut MotionBlur>),
        (With<Camera3d>, Without<crate::cloud_camera::CloudCamera>),
    >,
) {
    let Ok((mut projection, blur)) = q.single_mut() else {
        return;
    };
    if let Projection::Perspective(p) = &mut *projection {
        p.fov = std::f32::consts::FRAC_PI_4;
    }
    if let Some(mut blur) = blur {
        blur.shutter_angle = 0.0;
    }
}

/// Camera "feel": buffet, a wider lens and motion blur, layered on the pose
/// `follow_camera` just set. Runs *after* it (and reads the final pose), so the
/// shake never feeds back into `Chase::offset` or the aircraft's framing.
///
/// The buffet is a deterministic sum of sines — no state, the same every run —
/// and rises with dynamic pressure and with angle of attack, the two things that
/// actually shake an airframe. It is squared so the view stays calm in cruise
/// and comes alive only as the jet gets busy. The shake belongs to the F-22
/// alone: a fighter riding high q *should* rattle, but the airliner and light
/// planes fly smooth air — on them the feel is the wider lens and the blur
/// only, no wobble.
///
/// This is all a chase-camera trick. In the cockpit the pilot's eye is pinned to
/// the airframe and the lens is the eye, so the view gets none of it: the panel
/// and combiner glass stay rock-solid, with no synthetic buffet, FOV boost or
/// blur smearing them.
fn camera_feel(
    time: Res<Time>,
    state: Res<AircraftState>,
    sim: Res<Sim>,
    ui: Res<Ui>,
    tune: Res<CockpitTune>,
    mut q: Query<
        (&mut Transform, &mut Projection, Option<&mut MotionBlur>),
        (With<Camera3d>, Without<crate::cloud_camera::CloudCamera>),
    >,
) {
    let Ok((mut t, mut projection, blur)) = q.single_mut() else {
        return;
    };
    if ui.cockpit {
        // Leave the lens at its rest setting; `reset_camera_feel` does the same
        // on exiting a flight. The calibration demo can override the rest FOV,
        // and the C172's instrument close-up eases it narrower on top.
        if let Projection::Perspective(p) = &mut *projection {
            p.fov = ui
                .zoom_fov
                .unwrap_or_else(|| tune.fov.unwrap_or(std::f32::consts::FRAC_PI_4));
        }
        if let Some(mut blur) = blur {
            blur.shutter_angle = 0.0;
        }
        return;
    }
    let dt = time.delta_secs();
    let now = time.elapsed_secs();

    let feel = buffet_weight(state.cas_kts, state.alpha_deg);

    if aircraft_profile::normalized(&sim.name) == "f22" {
        // High load adds an airframe tremor of its own; a hard pull at low
        // speed shakes even when dynamic pressure alone is calm.
        let load = ((state.g_norm as f32 - 6.0) / 3.0).clamp(0.0, 1.0);
        let amp = (feel * feel).max(load * load);
        // Three incommensurate frequencies per axis, seeded apart, so the shake
        // never repeats on a visible beat. 2–6 Hz: buffet, not electrical noise
        // (which would alias badly at 60 fps).
        let noise = |seed: f32| {
            (now * 12.9 + seed).sin() * 0.55
                + (now * 23.3 + seed * 1.7).sin() * 0.30
                + (now * 37.1 + seed * 2.3).sin() * 0.15
        };
        t.translation += Vec3::new(noise(0.0), noise(11.3), noise(21.7)) * (SHAKE_POS_M * amp);
        let rot =
            Vec3::new(noise(31.1), noise(41.9), noise(51.7)) * (SHAKE_ROT_DEG.to_radians() * amp);
        // `YXZ` takes (yaw, pitch, roll).
        t.rotation *= Quat::from_euler(EulerRot::YXZ, rot.y, rot.x, rot.z);
    }

    if let Projection::Perspective(p) = &mut *projection {
        let want = std::f32::consts::FRAC_PI_4 + FOV_BOOST_DEG.to_radians() * feel;
        p.fov += (want - p.fov) * (1.0 - (-6.0 * dt).exp());
    }
    // Kept on the camera at all times (shutter 0 when idle) so its presence in
    // the render pipeline never changes with speed.
    if let Some(mut blur) = blur {
        blur.shutter_angle = BLUR_MAX * feel;
    }
}

fn toggle_ui(keys: Res<ButtonInput<KeyCode>>, sim: Res<Sim>, mut ui: ResMut<Ui>) {
    if keys.just_pressed(KeyCode::KeyH) {
        ui.show_inputs = !ui.show_inputs;
    }
    if keys.just_pressed(KeyCode::KeyC) {
        ui.panel_focus = None;
        ui.freeze_look = false;
        ui.zoom_fov = None;
        ui.cockpit = !ui.cockpit;
        // Every entry into the cockpit starts looking at its rest pose. Give
        // the cursor grab a moment to settle before accepting mouse motion.
        (ui.look_yaw, ui.look_pitch) = rest_look(&sim.name);
        ui.look_settle = if ui.cockpit { 1.0 } else { 0.0 };
        ui.look_ignore = ui.cockpit;
    }
    // Recentre the cockpit head-look (a no-op in the chase view).
    if keys.just_pressed(KeyCode::KeyN) {
        (ui.look_yaw, ui.look_pitch) = rest_look(&sim.name);
    }
}

/// Overcast weather dims the sun and ambient fill (the sky shader greys out on
/// its own, in `sky.rs`). Idempotent, so it just runs every frame.
fn dim_lights(
    weather: Res<Weather>,
    pose: Res<crate::map::SunPose>,
    mut sun: Query<(&mut DirectionalLight, &mut Transform), With<DirectionalLight>>,
    mut ambient: Query<&mut AmbientLight>,
) {
    // Rain keeps its daytime dimming; the time-of-day preset sets the sun's
    // colour, strength and pose. DAY keeps the mission's own elevation from
    // `SunPose`; below DAY the elevation drops to the preset's horizon line
    // while the azimuth rides along, so the sun keeps its quarter.
    let (elev, color, lux, amb_bright, amb_color) = match weather.time_level {
        0 => (pose.rot.0, Color::WHITE, 8000.0, 250.0, Color::WHITE),
        1 => (
            -0.085, // sun 5 deg above the horizon: sunset glow in the shaders
            Color::srgb(1.0, 0.72, 0.50),
            4000.0,
            170.0,
            Color::srgb(1.0, 0.90, 0.82),
        ),
        _ => (
            -0.95, // moonlight from high up
            Color::srgb(0.62, 0.72, 0.95),
            80.0,
            30.0,
            Color::srgb(0.60, 0.68, 1.0),
        ),
    };
    let k = (1.0 - 0.55 * weather.rain).max(0.15);
    for (mut light, mut tf) in &mut sun {
        light.illuminance = lux * k;
        light.color = color;
        tf.rotation = Quat::from_euler(EulerRot::XYZ, elev, pose.rot.1, 0.0);
    }
    for mut amb in &mut ambient {
        amb.brightness = amb_bright * k;
        amb.color = amb_color;
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

/// Per-aircraft key hints, generated from the active control set so they can't
/// drift from the profile. Duplicates collapse (both brakes share one entry),
/// and the list wraps a few entries per row.
fn control_hints(
    bindings: &Bindings,
    cockpit: bool,
    f22_hud: bool,
    mcp: bool,
    c172: bool,
) -> String {
    use crate::aircraft_profile::key_name;
    let mut entries: Vec<String> = Vec::new();
    for b in &bindings.list {
        let keys = match b.neg {
            Some(neg) => format!("{}/{}", key_name(b.pos), key_name(neg)),
            None => key_name(b.pos),
        };
        let entry = format!("{keys} {}", b.action.label());
        if !entries.contains(&entry) {
            entries.push(entry);
        }
    }
    for t in &bindings.toggles {
        entries.push(format!("{} {}", key_name(t.key), t.action.label()));
    }
    entries.push("H inputs".to_string());
    if c172 {
        entries.push("Space fine".to_string());
    }
    // Spawns live in the menu (lesson rows / map missions); no fly-time keys.
    entries.push("C view".to_string());
    if f22_hud {
        entries.push("O hud".to_string());
    }
    if mcp {
        entries.push("Left/Right hdg".to_string());
        entries.push("Up/Down alt".to_string());
        entries.push("PgUp/PgDn v/s".to_string());
        entries.push("[ ] speed".to_string());
        entries.push("; stand".to_string());
    }
    if cockpit {
        // Free-look only does anything from the pilot's seat.
        entries.push("N recentre".to_string());
        entries.push("mouse look".to_string());
    }

    const PER_LINE: usize = 4;
    entries
        .chunks(PER_LINE)
        .map(|row| row.join("   "))
        .collect::<Vec<_>>()
        .join("\n")
}

fn update_hud(
    state: Res<AircraftState>,
    ui: Res<Ui>,
    weather: Res<Weather>,
    sim: Res<Sim>,
    bindings: Res<Bindings>,
    ap: Res<Autopilot>,
    map: Res<crate::map::ActiveMap>,
    airfield: Res<crate::airfield::Airfield>,
    lesson: Res<crate::mission::ActiveLesson>,
    taxi: Res<crate::taxi::Taximeter>,
    mut hints: Local<(String, (bool, bool, bool, bool))>,
    mut q: Query<&mut Text, With<Hud>>,
) {
    let Ok(mut text) = q.single_mut() else {
        return;
    };
    // Flight-mode annunciation, only for aircraft that have an autopilot (the
    // 737); everything else flies manual and shows nothing.
    let mode = if state.autopilot_available {
        format!("   [{}]", ap.fma())
    } else {
        String::new()
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

    // The 737's stand carries the gear lights, N2 and flap pointer, so its
    // text HUD keeps only the numbers the stand does not show.
    let stand = aircraft_profile::normalized(&sim.name) == "737";

    // Landing-gear state, shown only for aircraft whose gear retracts (F-22).
    let gear = if state.gear_available && !stand {
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

    // Mission name, or the home field for the default C172 scene. A lesson
    // mission appends its tag so the player knows which spawn they re-flew.
    let place = match map.0 {
        Some(m) => format!("   [{}]", m.label()),
        None => {
            let default_spec = crate::airfield::AirfieldSpec::myfs();
            let spec = airfield.0.as_ref().unwrap_or(&default_spec);
            match lesson.0 {
                Some(lesson) => {
                    format!("   [{} RWY {} | {}]", spec.icao, spec.rwy_id, lesson.label())
                }
                None => format!("   [{} RWY {}]", spec.icao, spec.rwy_id),
            }
        }
    };

    // Instrument readings — what a real panel shows (RPM, not "throttle %").
    let wind = if weather.wind_kts > 0.5 {
        format!(
            "  WIND {:.0} KT FROM {:.0}",
            weather.wind_kts,
            crate::weather::WIND_FROM_DEG
        )
    } else {
        String::new()
    };
    let instruments = if stand {
        format!(
            "ALT {:>4.0} ft   IAS {:>3.0} kt   TRIM {:+.2}",
            state.alt_ft, state.cas_kts, state.elevator_trim,
        )
    } else {
        format!(
            "ALT {:>4.0} ft   IAS {:>3.0} kt   RPM {:>4.0}   FLAP {:>4.1}°   TRIM {:+.2}",
            state.alt_ft, state.cas_kts, state.rpm, state.flap_deg, state.elevator_trim,
        )
    };
    let mut out = format!(
        "myFlightSim — {}  {mode}{tvc}{gear}{place}   WX: {}{wind}\n\
         {instruments}\n\
         HDG {:>3.0}°   PITCH {:>5.1}°   ROLL {:>5.1}°",
        sim.label,
        weather.label(),
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

    // Engine/fuel status, only for airframes with the fuel model (Rust C172):
    // the cold-start flow's feedback line — master, engine state and tanks.
    if state.fuel_capacity_lb > 0.0 {
        let master = if state.master_on {
            "MASTER  ON "
        } else {
            "MASTER -OFF"
        };
        let eng = if state.engine_running {
            "RUNNING"
        } else {
            "STOPPED"
        };
        let refuelling = state.fuel_left_lb + state.fuel_right_lb >= state.fuel_capacity_lb;
        let tag = if refuelling { " (FULL)" } else { "" };
        out.push_str(&format!(
            "\n{}  {eng:>7}\nFUEL L {:>4.0} lb   R {:>4.0} lb{tag}",
            master, state.fuel_left_lb, state.fuel_right_lb,
        ));
    }

    // Apron taxi guidance: the live offset while rolling the lead-in line,
    // the frozen grade once parked (see taxi.rs).
    if let Some(offset) = taxi.offset {
        out.push_str(&format!("\nTAXI {offset:+.1} m"));
    } else if let Some((avg, grade)) = taxi.grade {
        out.push_str(&format!("\nTAXI AVG {avg:.1} m  {}", grade.label()));
    }

    // Optional control-input overlay. A real cockpit hides this because the
    // pilot feels the forces; a keyboard has no force feedback, so we show it.
    if ui.show_inputs {
        out.push_str(&format!(
            "\n\n\
             ELEV {} {:+.2}   AIL {} {:+.2}   RUD {} {:+.2}\n\
             THR  {}  {:.2}   TRM {} {:+.2}   FLP {}  {:.2}",
            signed_bar(state.elevator),
            state.elevator,
            signed_bar(state.aileron),
            state.aileron,
            signed_bar(state.rudder),
            state.rudder,
            bar(state.throttle),
            state.throttle,
            signed_bar(state.elevator_trim),
            state.elevator_trim,
            bar(state.flap_deg / 30.0),
            state.flap_deg / 30.0,
        ));
        if state.rotor_available {
            out.push_str(&format!(
                "\nCOLL {} {:.2}",
                bar(state.collective),
                state.collective,
            ));
        }
    }

    // `control_hints` walks every binding and formats a string per row, but the
    // result only changes with the keymap or the view flags — not with the
    // binding *values* the input layer rewrites each frame. Cache on those
    // flags (the keymap is fixed for the duration of a flight).
    let flags = (
        ui.cockpit,
        sim.name == "f22",
        state.autopilot_available,
        aircraft_profile::normalized(&sim.name) == "c172p",
    );
    if hints.0.is_empty() || hints.1 != flags {
        hints.0 = control_hints(&bindings, flags.0, flags.1, flags.2, flags.3);
        hints.1 = flags;
    }
    out.push_str("\n\n");
    out.push_str(&hints.0);

    if text.0 != out {
        text.0 = out;
    }
}

fn draw_grid(
    state: Res<AircraftState>,
    map: Res<crate::map::ActiveMap>,
    sim: Res<Sim>,
    app: Res<State<AppState>>,
    mut gizmos: Gizmos,
) {
    // The mission terrain replaces the training grid; the C172's airfield does
    // too, so the runway does not fight grid lines at y = 0.
    if map.0.is_some()
        || (*app.get() == AppState::Flying && aircraft_profile::normalized(&sim.name) == "c172p")
    {
        return;
    }
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
    use super::{
        apply_look_delta, bar, buffet_weight, look_rotation, signed_bar, Ui, LOOK_PITCH_LIMIT,
        SHAKE0_KT, SHAKE1_KT,
    };
    use bevy::prelude::{Vec2, Vec3};

    /// The cockpit camera's forward direction (`-Z` local) in the airframe body
    /// frame, after the head-look. Body axes: forward -Z, up +Y, right +X.
    fn forward(ui: &Ui) -> Vec3 {
        look_rotation(ui) * Vec3::NEG_Z
    }

    #[test]
    fn zero_look_keeps_the_camera_forward() {
        let ui = Ui::default();
        assert!(forward(&ui).abs_diff_eq(Vec3::NEG_Z, 1e-6));
    }

    #[test]
    fn mouse_right_looks_right_and_mouse_up_looks_up() {
        // Mouse right (delta.x > 0) must swing the view toward body-right (+X).
        let mut ui = Ui {
            cockpit: true,
            ..Default::default()
        };
        apply_look_delta(&mut ui, Vec2::new(10.0, 0.0));
        assert!(forward(&ui).x > 0.0, "mouse-right should look toward +X");

        // Mouse up (delta.y < 0, since y grows downward) must pitch up (+Y).
        let mut ui = Ui::default();
        apply_look_delta(&mut ui, Vec2::new(0.0, -10.0));
        assert!(forward(&ui).y > 0.0, "mouse-up should pitch toward +Y");
    }

    #[test]
    fn pitch_clamps_at_the_neck_limit_but_yaw_is_free() {
        let mut ui = Ui::default();
        for _ in 0..10_000 {
            apply_look_delta(&mut ui, Vec2::new(0.0, -50.0));
        }
        assert!(ui.look_pitch <= LOOK_PITCH_LIMIT + 1e-6);
        assert!(ui.look_pitch > 0.0);

        // Yaw keeps accumulating (you can turn right around).
        for _ in 0..10_000 {
            apply_look_delta(&mut ui, Vec2::new(50.0, 0.0));
        }
        assert!(ui.look_yaw < -1.0);
    }

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
    fn buffet_is_calm_slow_and_full_fast() {
        assert_eq!(buffet_weight(0.0, 0.0), 0.0);
        assert_eq!(buffet_weight(SHAKE0_KT as f64, 0.0), 0.0);
        assert_eq!(buffet_weight(SHAKE1_KT as f64, 0.0), 1.0);
        assert_eq!(buffet_weight(600.0, 0.0), 1.0);
    }

    #[test]
    fn buffet_rises_with_angle_of_attack_but_below_full() {
        // A slow, high-alpha hold (jet-fighter stall speeds): the airspeed ramp
        // is barely moving, but the airflow gate is fully open, so the
        // airframe reads as busy anyway.
        let slow_high_alpha = buffet_weight(160.0, 30.0);
        assert!(slow_high_alpha > 0.0);
        assert!(
            slow_high_alpha < 0.7,
            "alpha alone must not read as full speed"
        );
        // Below the buffet alpha it does nothing.
        assert_eq!(buffet_weight(100.0, 10.0), 0.0);
        // Speed still wins when it's the worse of the two.
        assert_eq!(buffet_weight(SHAKE1_KT as f64, 30.0), 1.0);
    }

    #[test]
    fn parked_alpha_noise_cannot_breathe_the_lens() {
        // The regressed cold-start sit: the parked C172 reports ±90-180° of
        // bogus alpha while creeping a few centimetres a second. No airflow,
        // no lens movement — the FOV must rest at its authored angle even with
        // the wildest alpha readings drifting through the stall band.
        assert_eq!(buffet_weight(0.05, 170.0), 0.0);
        assert_eq!(buffet_weight(0.05, -173.9), 0.0);
        // Taxiing stays silent too: the wing still carries no load at 25 kt.
        assert_eq!(buffet_weight(25.0, 90.0), 0.0);
        // A real stall just after the gate's lower edge reads through.
        assert!(buffet_weight(40.0, 30.0) > 0.0);
    }

    #[test]
    fn bar_fills_left_to_right() {
        assert_eq!(bar(0.0), "---------");
        assert_eq!(bar(1.0), "#########");
        assert_eq!(bar(0.5), "#####----");
        assert_eq!(bar(2.0), bar(1.0));
    }
}
