//! Cockpit instruments drawn as Bevy UI widgets.
//!
//! Each instrument is the same recipe: a dial *face* rasterised once into a
//! texture at startup, numerals and captions as UI text, and one or more needle
//! textures rotated every frame from live sim state. This module holds the
//! shared rasterising/spawning helpers and the plugin; the instruments
//! themselves live in [`airspeed`] and [`altimeter`].
//!
//! Outside the cockpit view the gauges sit in the screen's bottom corners. In
//! the cockpit view of the full C172 they are instead *seated* on the model's
//! painted panel: each instrument knows where its dial lives in the airframe's
//! body frame ([`PanelDial`]) and [`seat_panel`] projects that through the live
//! camera every frame, so the gauges stay glued to the panel — including under
//! free look, roll and window resizes.

mod airspeed;
mod altimeter;
mod attitude;
mod cdi;
pub mod focus;
mod fuel;
mod heading;
mod tach;
mod turn;
mod vsi;

use bevy::asset::RenderAssetUsages;
use bevy::prelude::*;
use bevy::render::render_resource::{Extent3d, TextureDimension, TextureFormat};
use bevy::ui_render::prelude::UiMaterialPlugin;

use crate::app_state::{unpaused, AppState};
use crate::sim::Sim;
use crate::ui_theme;

/// On-screen size of one round instrument, logical px.
pub const NODE: f32 = 196.0;
/// Texture edge length, px. Supersampled, then minified to `NODE`.
pub const TEX: usize = 512;
/// Dial outer (bezel) radius, in texture px. The rest is transparent margin.
pub const DIAL_R: f32 = 248.0;

// --- shared dial rasteriser vocabulary -------------------------------------
//
// The round gauges (airspeed, altimeter, fuel, tach, vsi) and the two card
// gauges (cdi, heading) all sample a radial dial: transparent past the rim,
// then the same three-band metal bezel, then a face with tick marks / arcs in
// the middle. These constants and two helpers are that shared vocabulary;
// each instrument file keeps only its own layout and colours.

/// Dial face colour.
pub const FACE: [u8; 4] = [16, 16, 18, 255];
/// Bezel base band.
pub const BEZEL: [u8; 4] = [14, 14, 16, 255];
/// Bezel bright band.
pub const BEZEL_HI: [u8; 4] = [46, 48, 54, 255];
/// Bezel outer rim (the metal edge).
pub const BEZEL_RIM: [u8; 4] = [104, 108, 116, 255];
/// Tick/arc colour used by the round dial face.
pub const TICK: [u8; 4] = [228, 228, 228, 255];

/// 4 buckets per degree: the quantised angle space the rasteriser paints into.
pub const BUCKETS: usize = 1440;

/// Map an angle in degrees to its bucket index (`0..BUCKETS`).
pub fn ang_bucket(ang_deg: f32) -> usize {
    (ang_deg.rem_euclid(360.0) / 360.0 * BUCKETS as f32) as usize % BUCKETS
}

/// The shared outer ring of a dial sample at radial offset `u` (normalised by
/// [`DIAL_R`]): fully transparent past `u > 1.0`, the bezel bands from `start`
/// inward. `None` means the caller is on the dial face and continues with its
/// own face logic (`start` is 0.90 for the round gauges, 0.88 for the cards).
pub fn bezel_grade(u: f32, start: f32) -> Option<[u8; 4]> {
    if u > 1.0 {
        Some([0, 0, 0, 0])
    } else if u >= start {
        Some(if u > 0.985 {
            BEZEL_RIM
        } else if u > 0.95 {
            BEZEL_HI
        } else {
            BEZEL
        })
    } else {
        None
    }
}

/// Stamp one tick (length class 2 = long/heavy, 1 = short/short) across a
/// `tick` bucket array, centred on angle `center` with half-width `half_w`
/// degrees and the same 0.06° step every instrument uses.
pub fn paint_tick(tick: &mut [u8], center: f32, class: u8, half_w: f32) {
    let mut d = -half_w;
    while d <= half_w {
        let b = ang_bucket(center + d);
        if tick[b] < class {
            tick[b] = class;
        }
        d += 0.06;
    }
}
/// Dial radius in on-screen px.
pub const DIAL_PX: f32 = NODE * 0.5 * (DIAL_R / (TEX as f32 * 0.5));

/// Instruments in each bottom trio.
pub const COLS: usize = 3;
/// Margin from the screen edges, and the gap between instruments, px.
pub const MARGIN: f32 = 37.0;
pub const GAP: f32 = 6.0;

/// Total width of one screen-corner trio, including its outer margin. The
/// control-position discs use this to sit in the gaps either side of the
/// centred attitude indicator.
pub const CLUSTER_W: f32 = MARGIN + COLS as f32 * NODE + (COLS as f32 - 1.0) * GAP;

/// Text was tuned at a 300 px gauge; scale label sizes with the current `NODE`.
const FS: f32 = NODE / 300.0;

/// Marks an instrument's root node, so one system can show/hide the cluster and
/// another can move it onto the model's painted panel in the cockpit view.
#[derive(Component)]
pub struct Instrument {
    /// Which painted dial this instrument replaces in the cockpit view.
    key: PanelKey,
    /// The screen-corner layout used outside the cockpit view, restored on exit.
    home: Node,
    /// Whether the panel layout is currently applied (vs. `home`).
    seated: bool,
}

/// The painted dials on the full C172 model's instrument panel, one per live
/// instrument. `Airspeed`…`Attitude` name our widgets; the dials they cover are
/// the ones the model paints at those spots.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum PanelKey {
    Airspeed,
    Vsi,
    Altimeter,
    Tach,
    Heading,
    Turn,
    Attitude,
    Fuel,
    Cdi,
}

/// Where a painted dial sits on the panel, in the airframe (`view::AircraftRoot`)
/// body frame: `anchor` is the dial centre and `u`/`v` each span one dial radius
/// toward screen-right and screen-up. Stored in 3D — not as screen pixels — so
/// the gauges stay glued to the panel through free look, roll and window resizes.
/// Measured with `examples/panel_calib`, which ray-casts the on-screen dial
/// centres onto the panel plane.
pub struct PanelDial {
    anchor: Vec3,
    u: Vec3,
    v: Vec3,
}

impl PanelKey {
    fn dial(self) -> PanelDial {
        let (a, u, v): ([f32; 3], [f32; 3], [f32; 3]) = match self {
            PanelKey::Airspeed => (
                [-0.3102, 0.1506, -0.6831],
                [0.0389, 0.0000, 0.0000],
                [-0.0004, 0.0388, -0.0031],
            ),
            PanelKey::Vsi => (
                [-0.2227, 0.1506, -0.6831],
                [0.0389, 0.0000, 0.0000],
                [-0.0001, 0.0388, -0.0031],
            ),
            PanelKey::Altimeter => (
                [-0.1336, 0.1537, -0.6833],
                [0.0389, 0.0000, 0.0000],
                [0.0002, 0.0388, -0.0031],
            ),
            PanelKey::Tach => (
                [-0.0445, 0.1537, -0.6833],
                [0.0389, 0.0000, 0.0000],
                [0.0004, 0.0388, -0.0031],
            ),
            PanelKey::Heading => (
                [-0.1352, 0.0599, -0.6758],
                [0.0389, 0.0000, 0.0000],
                [0.0001, 0.0385, -0.0031],
            ),
            PanelKey::Turn => (
                [-0.1291, -0.0400, -0.6678],
                [0.0383, 0.0000, 0.0000],
                [0.0002, 0.0376, -0.0030],
            ),
            PanelKey::Attitude => (
                [-0.3096, -0.0306, -0.6685],
                [0.0389, 0.0000, 0.0000],
                [-0.0004, 0.0381, -0.0031],
            ),
            // Right of the tach, the model's blank panel at the top row's end.
            PanelKey::Fuel => (
                [0.0450, 0.1537, -0.6833],
                [0.0389, 0.0000, 0.0000],
                [0.0004, 0.0388, -0.0031],
            ),
            // The bottom row continues right of the turn coordinator.
            PanelKey::Cdi => (
                [-0.0370, -0.0400, -0.6678],
                [0.0383, 0.0000, 0.0000],
                [0.0002, 0.0376, -0.0030],
            ),
        };
        PanelDial {
            anchor: a.into(),
            u: u.into(),
            v: v.into(),
        }
    }

    /// The instrument a lowercase id selects: the same names `examples/
    /// panel_view_shot` uses. `None` for anything else.
    #[allow(dead_code)] // Only the screenshot examples name a dial.
    pub fn from_name(name: &str) -> Option<Self> {
        Some(match name.trim().to_ascii_lowercase().as_str() {
            "airspeed" => PanelKey::Airspeed,
            "vsi" => PanelKey::Vsi,
            "altimeter" => PanelKey::Altimeter,
            "tach" => PanelKey::Tach,
            "heading" => PanelKey::Heading,
            "turn" => PanelKey::Turn,
            "attitude" => PanelKey::Attitude,
            "fuel" => PanelKey::Fuel,
            "cdi" => PanelKey::Cdi,
            _ => return None,
        })
    }
}

/// A dial's placement on screen for the current camera: the centre in logical
/// px, plus the screen vectors spanning one dial radius right and up.
pub struct ProjectedDial {
    pub centre: Vec2,
    pub right: Vec2,
    pub up: Vec2,
}

/// Project `dial` through `camera` (positioned by `cam_gt`) into screen space.
/// `None` when any of the three points is behind the lens. Shared by the live
/// instrument seating and the focus-ring overlay so the two stay in lockstep.
pub fn project_dial(
    camera: &Camera,
    cam_gt: &GlobalTransform,
    root_tf: &Transform,
    dial: &PanelDial,
) -> Option<ProjectedDial> {
    let project = |p: Vec3| camera.world_to_viewport(cam_gt, p).ok();
    let centre = project(root_tf.transform_point(dial.anchor))?;
    let right = project(root_tf.transform_point(dial.anchor + dial.u))? - centre;
    let up = project(root_tf.transform_point(dial.anchor + dial.v))? - centre;
    Some(ProjectedDial { centre, right, up })
}

pub struct GaugePlugin;

impl Plugin for GaugePlugin {
    fn build(&self, app: &mut App) {
        app.add_plugins(UiMaterialPlugin::<attitude::AttitudeMaterial>::default())
            .add_plugins(focus::FocusPlugin)
            .add_systems(
                Startup,
                (
                    airspeed::setup,
                    altimeter::setup,
                    vsi::setup,
                    tach::setup,
                    heading::setup,
                    turn::setup,
                    attitude::setup,
                    fuel::setup,
                    cdi::setup,
                ),
            )
            .add_systems(
                Update,
                (
                    (
                        airspeed::update_needle,
                        altimeter::update_needles,
                        vsi::update_needle,
                        tach::update_needle,
                        heading::update_card,
                        turn::update,
                        attitude::update,
                        fuel::update_needles,
                        cdi::update,
                    )
                        .run_if(unpaused),
                    sync_visibility,
                    // After the camera and airframe have this frame's pose, so the
                    // panel projection matches what actually gets drawn (and will
                    // stay in step once free look can move the camera), and after
                    // `sync_visibility`, so it has the last word on which of the
                    // two layouts (screen corners vs. panel) is showing.
                    seat_panel
                        .after(sync_visibility)
                        .after(crate::view::sync_aircraft)
                        .after(crate::view::follow_camera),
                ),
            );
    }
}

/// The instruments are C172-only; hide the whole cluster otherwise.
fn sync_visibility(
    state: Res<State<AppState>>,
    sim: Res<Sim>,
    q: Query<&mut Node, With<Instrument>>,
) {
    ui_theme::set_display(
        q,
        if *state.get() == AppState::Flying && sim.name == "c172p" {
            Display::Flex
        } else {
            Display::None
        },
    );
}

/// Seat the live instruments onto the model's painted panel while the cockpit
/// view is active, and return them to the screen corners otherwise.
///
/// The panel is rigid with the airframe, so each gauge's dial is stored in 3D
/// (see [`PanelDial`]): every frame the anchor and its two radius vectors are
/// projected through the *current* camera and the whole widget — face, numerals
/// and needle together — is moved, scaled and rotated to cover the painted dial.
/// Because the placement comes from the camera rather than from fixed screen
/// coordinates, it survives free look, head-roll, resizing and any aircraft
/// attitude; when the panel is behind the camera the gauge is hidden.
fn seat_panel(
    state: Res<State<AppState>>,
    ui: Res<crate::view::Ui>,
    sim: Res<Sim>,
    cams: Query<(&Camera, &Transform), (With<Camera3d>, Without<crate::cloud_camera::CloudCamera>)>,
    airframe: Query<&Transform, With<crate::view::AircraftRoot>>,
    mut q: Query<(&mut Instrument, &mut Node, &mut UiTransform)>,
) {
    let on_panel =
        *state.get() == AppState::Flying && ui.cockpit && crate::view::has_cockpit(&sim.name);
    let view = if on_panel {
        match (cams.single(), airframe.single()) {
            (Ok((camera, cam_tf)), Ok(root_tf)) => {
                Some((camera, GlobalTransform::from(*cam_tf), *root_tf))
            }
            _ => None,
        }
    } else {
        None
    };
    let half = NODE * 0.5;

    for (mut inst, mut node, mut tf) in &mut q {
        let Some((camera, cam_gt, root_tf)) = view else {
            // Screen-corner layout: undo whatever the panel seating did, but keep
            // the `display` flag `sync_visibility` owns.
            if !inst.seated {
                continue;
            }
            inst.seated = false;
            let display = node.display;
            *node = inst.home.clone();
            node.display = display;
            *tf = UiTransform::IDENTITY;
            continue;
        };

        let d = inst.key.dial();
        let Some(p) = project_dial(camera, &cam_gt, &root_tf, &d) else {
            // Off-screen (e.g. looking away): hide rather than smear it.
            inst.seated = true;
            node.display = Display::None;
            continue;
        };

        // The dial's screen-space right and up vectors also give the size (a
        // nearer or resized panel changes them) and the on-screen roll.
        let (right, up) = (p.right, p.up);
        node.position_type = PositionType::Absolute;
        node.left = Val::Px(0.0);
        node.right = Val::Auto;
        node.top = Val::Px(0.0);
        node.bottom = Val::Auto;
        node.margin = UiRect::ZERO;
        node.width = Val::Px(NODE);
        node.height = Val::Px(NODE);
        node.display = Display::Flex;
        // The dial sits at the node's centre, so park that centre on the anchor.
        tf.translation = Val2::px(p.centre.x - half, p.centre.y - half);
        tf.scale = Vec2::new(right.length() / DIAL_PX, up.length() / DIAL_PX);
        tf.rotation = Rot2::radians(right.y.atan2(right.x));
        inst.seated = true;
    }
}

/// Which screen edge a bottom trio is pinned to. The panel is three gauges
/// against the left edge and three against the right, so the cluster spreads
/// with the window instead of huddling in the middle; the attitude indicator
/// sits centred between the two trios.
#[derive(Clone, Copy)]
pub enum Side {
    Left,
    Right,
}

/// Spawn an instrument's root node. `col` is its place within its `side`'s trio
/// and `row` its place above the cluster's bottom, both counted up/right.
pub fn spawn_root(
    commands: &mut Commands,
    key: PanelKey,
    side: Side,
    col: usize,
    row: usize,
) -> Entity {
    let offset = Val::Px(MARGIN + col as f32 * (NODE + GAP));
    // The right trio grows leftward from the right edge, so mirror `col` to keep
    // the left-to-right reading order the same on both sides.
    let (left, right) = match side {
        Side::Left => (offset, Val::Auto),
        Side::Right => (
            Val::Auto,
            Val::Px(MARGIN + (COLS - 1 - col) as f32 * (NODE + GAP)),
        ),
    };
    let home = Node {
        position_type: PositionType::Absolute,
        left,
        right,
        bottom: Val::Px(MARGIN + row as f32 * (NODE + GAP)),
        width: Val::Px(NODE),
        height: Val::Px(NODE),
        // Hidden until we're flying the C172; see `sync_visibility`.
        display: Display::None,
        ..default()
    };
    commands
        .spawn((
            Instrument {
                key,
                home: home.clone(),
                seated: false,
            },
            home,
            UiTransform::IDENTITY,
        ))
        .id()
}

/// Spawn a lone instrument centred horizontally, `bottom` px up from the
/// window's lower edge — the attitude indicator, in the bottom row's middle.
/// `left: 50%` puts the node's left edge at the middle and the negative margin
/// pulls it back by half its width, so it stays centred however wide the window.
pub fn spawn_root_centered(commands: &mut Commands, key: PanelKey, bottom: f32) -> Entity {
    let home = Node {
        position_type: PositionType::Absolute,
        left: Val::Percent(50.0),
        margin: UiRect::left(Val::Px(-0.5 * NODE)),
        bottom: Val::Px(bottom),
        width: Val::Px(NODE),
        height: Val::Px(NODE),
        // Hidden until we're flying the C172; see `sync_visibility`.
        display: Display::None,
        ..default()
    };
    commands
        .spawn((
            Instrument {
                key,
                home: home.clone(),
                seated: false,
            },
            home,
            UiTransform::IDENTITY,
        ))
        .id()
}

/// A square image node filling the instrument, with `extra` components attached
/// (markers, needle rotation).
pub fn spawn_image<B: Bundle>(p: &mut ChildSpawnerCommands, image: Handle<Image>, extra: B) {
    p.spawn((
        ImageNode::new(image),
        Node {
            position_type: PositionType::Absolute,
            left: Val::Px(0.0),
            top: Val::Px(0.0),
            width: Val::Px(NODE),
            height: Val::Px(NODE),
            ..default()
        },
        UiTransform::IDENTITY,
        extra,
    ));
}

/// A centred text label at on-screen offset `(x, y)` from the dial centre.
pub fn spawn_label(
    p: &mut ChildSpawnerCommands,
    text: &str,
    x: f32,
    y: f32,
    size: f32,
    color: Color,
) {
    let box_px = 48.0 * FS;
    p.spawn((
        Text::new(text),
        TextFont {
            font_size: FontSize::Px(size * FS),
            ..default()
        },
        TextColor(color),
        Node {
            position_type: PositionType::Absolute,
            left: Val::Px(x - box_px * 0.5),
            top: Val::Px(y - box_px * 0.5),
            width: Val::Px(box_px),
            height: Val::Px(box_px),
            justify_content: JustifyContent::Center,
            align_items: AlignItems::Center,
            ..default()
        },
    ));
}

/// True if `a` lies in the clockwise arc from `lo` to `hi` (degrees, any wrap).
pub fn cw_between(a: f32, lo: f32, hi: f32) -> bool {
    let (a, lo, hi) = (
        a.rem_euclid(360.0),
        lo.rem_euclid(360.0),
        hi.rem_euclid(360.0),
    );
    if lo <= hi {
        a >= lo && a <= hi
    } else {
        a >= lo || a <= hi
    }
}

/// Rasterise a dial face by sampling `sample(x, y)` — offsets from the dial
/// centre in texture px — 3×3 times per pixel for smooth edges.
pub fn raster_face(mut sample: impl FnMut(f32, f32) -> [u8; 4]) -> Vec<u8> {
    const SS: i32 = 3;
    let c = TEX as f32 * 0.5;
    let mut data = vec![0u8; TEX * TEX * 4];
    for py in 0..TEX {
        for px in 0..TEX {
            let mut acc = [0f32; 4];
            for sy in 0..SS {
                for sx in 0..SS {
                    let x = px as f32 + (sx as f32 + 0.5) / SS as f32 - c;
                    let y = py as f32 + (sy as f32 + 0.5) / SS as f32 - c;
                    let s = sample(x, y);
                    for i in 0..4 {
                        acc[i] += s[i] as f32;
                    }
                }
            }
            let k = 1.0 / (SS * SS) as f32;
            let o = (py * TEX + px) * 4;
            for i in 0..4 {
                data[o + i] = (acc[i] * k).round() as u8;
            }
        }
    }
    data
}

/// The shape of one needle, in units of the dial radius.
pub struct NeedleShape {
    /// Blade length from the centre.
    pub len: f32,
    /// Blade half-width at the hub and at the tip.
    pub base_w: f32,
    pub tip_w: f32,
    /// Counterweight tail length and its half-width at the far end.
    pub tail: f32,
    pub tail_w: f32,
    /// Hub radius.
    pub hub: f32,
}

/// Rasterise a needle pointing straight up from the centre, with a
/// counterweight tail and a hub. Rotated per frame by the node's `UiTransform`.
pub fn needle_pixels(shape: &NeedleShape, color: [u8; 4], hub_color: [u8; 4]) -> Vec<u8> {
    let c = TEX as f32 * 0.5;
    let (blade, tail) = (shape.len * DIAL_R, shape.tail * DIAL_R);
    let hub_r = shape.hub * DIAL_R;
    let mut data = vec![0u8; TEX * TEX * 4];
    for py in 0..TEX {
        for px in 0..TEX {
            let x = px as f32 + 0.5 - c;
            let y = py as f32 + 0.5 - c; // +y is down
            let r = (x * x + y * y).sqrt();

            let hw = if y <= 0.0 {
                let f = (-y / blade).clamp(0.0, 1.0);
                if -y > blade {
                    -1.0
                } else {
                    DIAL_R * (shape.base_w + (shape.tip_w - shape.base_w) * f)
                }
            } else {
                let f = (y / tail).clamp(0.0, 1.0);
                if y > tail {
                    -1.0
                } else {
                    DIAL_R * (shape.base_w + (shape.tail_w - shape.base_w) * f)
                }
            };

            let col = if r <= hub_r {
                hub_color
            } else if hw > 0.0 {
                let cov = (hw - x.abs() + 0.5).clamp(0.0, 1.0);
                if cov <= 0.0 {
                    [0, 0, 0, 0]
                } else {
                    [color[0], color[1], color[2], (255.0 * cov).round() as u8]
                }
            } else {
                [0, 0, 0, 0]
            };
            let o = (py * TEX + px) * 4;
            data[o..o + 4].copy_from_slice(&col);
        }
    }
    data
}

/// Wrap raw RGBA pixels in a Bevy image.
pub fn make_image(data: Vec<u8>) -> Image {
    Image::new(
        Extent3d {
            width: TEX as u32,
            height: TEX as u32,
            depth_or_array_layers: 1,
        },
        TextureDimension::D2,
        data,
        TextureFormat::Rgba8UnormSrgb,
        RenderAssetUsages::default(),
    )
}
