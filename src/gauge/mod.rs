//! Cockpit instruments drawn as Bevy UI widgets.
//!
//! Each instrument is the same recipe: a dial *face* rasterised once into a
//! texture at startup, numerals and captions as UI text, and one or more needle
//! textures rotated every frame from live sim state. This module holds the
//! shared rasterising/spawning helpers and the plugin; the instruments
//! themselves live in [`airspeed`] and [`altimeter`].

mod airspeed;
mod altimeter;
mod attitude;
mod heading;
mod tach;
mod turn;
mod vsi;

use bevy::asset::RenderAssetUsages;
use bevy::prelude::*;
use bevy::render::render_resource::{Extent3d, TextureDimension, TextureFormat};
use bevy::ui_render::prelude::UiMaterialPlugin;

use crate::app_state::AppState;
use crate::sim::Sim;

/// On-screen size of one round instrument, logical px.
pub(crate) const NODE: f32 = 196.0;
/// Texture edge length, px. Supersampled, then minified to `NODE`.
pub(crate) const TEX: usize = 512;
/// Dial outer (bezel) radius, in texture px. The rest is transparent margin.
pub(crate) const DIAL_R: f32 = 248.0;
/// Dial radius in on-screen px.
pub(crate) const DIAL_PX: f32 = NODE * 0.5 * (DIAL_R / (TEX as f32 * 0.5));

/// Instruments in each bottom trio.
const COLS: usize = 3;
/// Margin from the screen edges, and the gap between instruments, px.
const MARGIN: f32 = 37.0;
const GAP: f32 = 6.0;

/// Text was tuned at a 300 px gauge; scale label sizes with the current `NODE`.
const FS: f32 = NODE / 300.0;

/// Marks an instrument's root node, so one system can show/hide the cluster.
#[derive(Component)]
struct Instrument;

pub struct GaugePlugin;

impl Plugin for GaugePlugin {
    fn build(&self, app: &mut App) {
        app.add_plugins(UiMaterialPlugin::<attitude::AttitudeMaterial>::default())
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
                    )
                        .run_if(in_state(AppState::Flying)),
                    sync_visibility,
                ),
            );
    }
}

/// The instruments are C172-only; hide the whole cluster otherwise.
fn sync_visibility(
    state: Res<State<AppState>>,
    sim: Res<Sim>,
    mut q: Query<&mut Node, With<Instrument>>,
) {
    let want = if *state.get() == AppState::Flying && sim.name == "c172p" {
        Display::Flex
    } else {
        Display::None
    };
    for mut node in &mut q {
        if node.display != want {
            node.display = want;
        }
    }
}

/// Which screen edge a bottom trio is pinned to. The panel is three gauges
/// against the left edge and three against the right, so the cluster spreads
/// with the window instead of huddling in the middle; the attitude indicator
/// sits centred between the two trios.
#[derive(Clone, Copy)]
pub(crate) enum Side {
    Left,
    Right,
}

/// Spawn an instrument's root node. `col` is its place within its `side`'s trio
/// and `row` its place above the cluster's bottom, both counted up/right.
pub(crate) fn spawn_root(commands: &mut Commands, side: Side, col: usize, row: usize) -> Entity {
    let offset = Val::Px(MARGIN + col as f32 * (NODE + GAP));
    // The right trio grows leftward from the right edge, so mirror `col` to keep
    // the left-to-right reading order the same on both sides.
    let (left, right) = match side {
        Side::Left => (offset, Val::Auto),
        Side::Right => (Val::Auto, Val::Px(MARGIN + (COLS - 1 - col) as f32 * (NODE + GAP))),
    };
    commands
        .spawn((
            Instrument,
            Node {
                position_type: PositionType::Absolute,
                left,
                right,
                bottom: Val::Px(MARGIN + row as f32 * (NODE + GAP)),
                width: Val::Px(NODE),
                height: Val::Px(NODE),
                // Hidden until we're flying the C172; see `sync_visibility`.
                display: Display::None,
                ..default()
            },
        ))
        .id()
}

/// Spawn a lone instrument centred horizontally, `bottom` px up from the
/// window's lower edge — the attitude indicator, in the bottom row's middle.
/// `left: 50%` puts the node's left edge at the middle and the negative margin
/// pulls it back by half its width, so it stays centred however wide the window.
pub(crate) fn spawn_root_centered(commands: &mut Commands, bottom: f32) -> Entity {
    commands
        .spawn((
            Instrument,
            Node {
                position_type: PositionType::Absolute,
                left: Val::Percent(50.0),
                margin: UiRect::left(Val::Px(-0.5 * NODE)),
                bottom: Val::Px(bottom),
                width: Val::Px(NODE),
                height: Val::Px(NODE),
                // Hidden until we're flying the C172; see `sync_visibility`.
                display: Display::None,
                ..default()
            },
        ))
        .id()
}

/// A square image node filling the instrument, with `extra` components attached
/// (markers, needle rotation).
pub(crate) fn spawn_image<B: Bundle>(
    p: &mut ChildSpawnerCommands,
    image: Handle<Image>,
    extra: B,
) {
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
pub(crate) fn spawn_label(
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
pub(crate) fn cw_between(a: f32, lo: f32, hi: f32) -> bool {
    let (a, lo, hi) = (a.rem_euclid(360.0), lo.rem_euclid(360.0), hi.rem_euclid(360.0));
    if lo <= hi {
        a >= lo && a <= hi
    } else {
        a >= lo || a <= hi
    }
}

/// Rasterise a dial face by sampling `sample(x, y)` — offsets from the dial
/// centre in texture px — 3×3 times per pixel for smooth edges.
pub(crate) fn raster_face(mut sample: impl FnMut(f32, f32) -> [u8; 4]) -> Vec<u8> {
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
pub(crate) struct NeedleShape {
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
pub(crate) fn needle_pixels(shape: &NeedleShape, color: [u8; 4], hub_color: [u8; 4]) -> Vec<u8> {
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
pub(crate) fn make_image(data: Vec<u8>) -> Image {
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
