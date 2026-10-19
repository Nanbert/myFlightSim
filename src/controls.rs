//! On-screen control-position indicators for the chase view.
//!
//! Two small discs sit either side of the attitude indicator: a yoke on the
//! left and rudder pedals on the right. They show the *commands* the app is
//! feeding the FDM — the missing feedback for a keyboard, which has no stick
//! force, no control position, and no seat of the pants. They are explicitly
//! an assist, not a real C172 instrument, so they are C172-only and hidden in
//! the cockpit view.
//!
//! The discs are ordinary UI nodes: a round face, a crosshair, a moving yoke
//! (translation = pitch/roll, rotation = roll) and a seesaw pedal bar
//! (rotation = rudder). The two brake slots under the pedals show the toe-brake
//! commands. The trim marker on the yoke's pitch axis shows where the trim has
//! moved neutral.

use bevy::picking::Pickable;
use bevy::prelude::*;
use bevy::text::FontSize;
use bevy::window::PrimaryWindow;

use crate::aircraft_profile::{self, Action};
use crate::app_state::AppState;
use crate::assist::AssistState;
use crate::gauge;
use crate::input::Bindings;
use crate::sim::{AircraftState, Sim};
use crate::view::Ui;

/// Disc diameter, logical px. Small enough to fit between the 196 px gauge
/// trios and the centred attitude indicator on a 1920-wide screen.
const DISC: f32 = 140.0;
const RADIUS: f32 = DISC * 0.5;
/// Half-travel of the yoke glyph at full pitch/roll.
const MAX_X: f32 = 36.0;
const MAX_Y: f32 = 30.0;
/// Gap between a disc and the attitude indicator on a wide screen.
const DISC_GAP: f32 = 18.0;
/// Height of a brake slot's fill.
const BRAKE_H: f32 = 20.0;

const BG: Color = Color::srgba(0.015, 0.025, 0.03, 0.58);
const BEZEL: Color = Color::srgba(0.75, 0.78, 0.74, 0.42);
const INK: Color = Color::srgba(0.90, 0.92, 0.88, 0.92);
const DIM: Color = Color::srgba(0.65, 0.70, 0.66, 0.55);
const FAINT: Color = Color::srgba(0.65, 0.70, 0.66, 0.22);
const AMBER: Color = Color::srgba(0.95, 0.72, 0.35, 0.95);
const CYAN: Color = Color::srgba(0.35, 0.85, 0.90, 0.95);

#[derive(Clone, Copy, PartialEq, Eq)]
enum Side {
    Stick,
    Pedals,
}

#[derive(Component)]
struct ControlRoot(Side);

#[derive(Component)]
struct YokeGlyph;

#[derive(Component)]
struct TrimMark;

#[derive(Component)]
struct PedalBar;

#[derive(Component)]
struct BrakeFill {
    left: bool,
}

#[derive(Component)]
struct CenterDot {
    side: Side,
}

#[derive(Component)]
struct AssistLabel {
    side: Side,
}

pub struct ControlPlugin;

impl Plugin for ControlPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Startup, spawn_indicators).add_systems(
            Update,
            (
                layout_indicators,
                update_stick.run_if(in_state(AppState::Flying)),
                update_pedals.run_if(in_state(AppState::Flying)),
                update_assist_labels.run_if(in_state(AppState::Flying)),
            ),
        );
    }
}

fn spawn_indicators(mut commands: Commands) {
    spawn_disc(&mut commands, Side::Stick);
    spawn_disc(&mut commands, Side::Pedals);
}

fn spawn_disc(commands: &mut Commands, side: Side) {
    let root = commands
        .spawn((
            ControlRoot(side),
            Node {
                position_type: PositionType::Absolute,
                left: Val::Px(0.0),
                bottom: Val::Px(gauge::MARGIN),
                width: Val::Px(DISC),
                height: Val::Px(DISC),
                border: UiRect::all(Val::Px(2.0)),
                border_radius: BorderRadius::MAX,
                overflow: Overflow::clip(),
                display: Display::None,
                ..default()
            },
            BackgroundColor(BG),
            BorderColor::all(BEZEL),
            UiTransform::IDENTITY,
            GlobalZIndex(12),
            Pickable::IGNORE,
        ))
        .id();

    commands.entity(root).with_children(|disc| {
        // Crosshair.
        disc.spawn((
            Node {
                position_type: PositionType::Absolute,
                left: Val::Px(14.0),
                top: Val::Px(RADIUS - 0.5),
                width: Val::Px(DISC - 28.0),
                height: Val::Px(1.0),
                ..default()
            },
            BackgroundColor(FAINT),
            Pickable::IGNORE,
        ));
        disc.spawn((
            Node {
                position_type: PositionType::Absolute,
                left: Val::Px(RADIUS - 0.5),
                top: Val::Px(14.0),
                width: Val::Px(1.0),
                height: Val::Px(DISC - 28.0),
                ..default()
            },
            BackgroundColor(FAINT),
            Pickable::IGNORE,
        ));
        // Inner ring, for a little depth without a texture.
        disc.spawn((
            Node {
                position_type: PositionType::Absolute,
                left: Val::Px(DISC * 0.25),
                top: Val::Px(DISC * 0.25),
                width: Val::Px(DISC * 0.5),
                height: Val::Px(DISC * 0.5),
                border: UiRect::all(Val::Px(1.0)),
                border_radius: BorderRadius::MAX,
                ..default()
            },
            BorderColor::all(FAINT),
            Pickable::IGNORE,
        ));
        // Label inside the lower rim.
        disc.spawn((
            Node {
                position_type: PositionType::Absolute,
                left: Val::Px(0.0),
                top: Val::Px(DISC - 17.0),
                width: Val::Px(DISC),
                justify_content: JustifyContent::Center,
                ..default()
            },
            Pickable::IGNORE,
        ))
        .with_children(|label| {
            label.spawn((
                Text::new(match side {
                    Side::Stick => "YOKE",
                    Side::Pedals => "RUDDER",
                }),
                TextFont {
                    font_size: FontSize::Px(11.0),
                    ..default()
                },
                TextColor(DIM),
                Pickable::IGNORE,
            ));
        });

        match side {
            Side::Stick => spawn_yoke(disc),
            Side::Pedals => spawn_pedals(disc),
        }

        // Centre detent, on top of the moving parts so it stays visible when
        // the yoke/pedals are centred.
        disc.spawn((
            CenterDot { side },
            Node {
                position_type: PositionType::Absolute,
                left: Val::Px(RADIUS - 3.0),
                top: Val::Px(RADIUS - 3.0),
                width: Val::Px(6.0),
                height: Val::Px(6.0),
                border_radius: BorderRadius::MAX,
                ..default()
            },
            BackgroundColor(DIM),
            Pickable::IGNORE,
        ));

        // Assist annunciation: BANK on the yoke, COORD on the pedals. Hidden
        // until the corresponding controller is actually contributing.
        disc.spawn((
            AssistLabel { side },
            Node {
                position_type: PositionType::Absolute,
                left: Val::Px(10.0),
                top: Val::Px(6.0),
                display: Display::None,
                ..default()
            },
            Pickable::IGNORE,
        ))
        .with_children(|p| {
            p.spawn((
                Text::new(match side {
                    Side::Stick => "BANK",
                    Side::Pedals => "COORD",
                }),
                TextFont {
                    font_size: FontSize::Px(9.5),
                    ..default()
                },
                TextColor(CYAN),
                Pickable::IGNORE,
            ));
        });
    });
}

fn spawn_yoke(disc: &mut ChildSpawnerCommands) {
    // The whole glyph translates with pitch/roll; its own `UiTransform` is
    // updated every frame.
    disc.spawn((
        YokeGlyph,
        Node {
            position_type: PositionType::Absolute,
            left: Val::Px((DISC - 64.0) * 0.5),
            top: Val::Px((DISC - 40.0) * 0.5),
            width: Val::Px(64.0),
            height: Val::Px(40.0),
            ..default()
        },
        UiTransform::IDENTITY,
        Pickable::IGNORE,
    ))
    .with_children(|yoke| {
        // Horizontal wheel bar.
        yoke.spawn((
            Node {
                position_type: PositionType::Absolute,
                left: Val::Px(0.0),
                top: Val::Px(17.0),
                width: Val::Px(64.0),
                height: Val::Px(6.0),
                border_radius: BorderRadius::all(Val::Px(3.0)),
                ..default()
            },
            BackgroundColor(INK),
            Pickable::IGNORE,
        ));
        // Grips.
        for left in [0.0, 56.0] {
            yoke.spawn((
                Node {
                    position_type: PositionType::Absolute,
                    left: Val::Px(left),
                    top: Val::Px(8.0),
                    width: Val::Px(8.0),
                    height: Val::Px(24.0),
                    border_radius: BorderRadius::all(Val::Px(3.0)),
                    ..default()
                },
                BackgroundColor(INK),
                Pickable::IGNORE,
            ));
        }
        // Hub and column.
        yoke.spawn((
            Node {
                position_type: PositionType::Absolute,
                left: Val::Px(28.0),
                top: Val::Px(13.0),
                width: Val::Px(8.0),
                height: Val::Px(14.0),
                border_radius: BorderRadius::all(Val::Px(2.0)),
                ..default()
            },
            BackgroundColor(INK),
            Pickable::IGNORE,
        ));
        yoke.spawn((
            Node {
                position_type: PositionType::Absolute,
                left: Val::Px(30.0),
                top: Val::Px(26.0),
                width: Val::Px(4.0),
                height: Val::Px(12.0),
                border_radius: BorderRadius::all(Val::Px(2.0)),
                ..default()
            },
            BackgroundColor(INK),
            Pickable::IGNORE,
        ));
    });

    // Trim marker on the left edge of the pitch axis.
    disc.spawn((
        TrimMark,
        Node {
            position_type: PositionType::Absolute,
            left: Val::Px(12.0),
            top: Val::Px(RADIUS - 8.0),
            width: Val::Px(5.0),
            height: Val::Px(16.0),
            border_radius: BorderRadius::all(Val::Px(2.0)),
            ..default()
        },
        BackgroundColor(AMBER),
        UiTransform::IDENTITY,
        Pickable::IGNORE,
    ));
}

fn spawn_pedals(disc: &mut ChildSpawnerCommands) {
    // The seesaw rotates as one unit for rudder.
    disc.spawn((
        PedalBar,
        Node {
            position_type: PositionType::Absolute,
            left: Val::Px((DISC - 72.0) * 0.5),
            top: Val::Px((DISC - 40.0) * 0.5 + 2.0),
            width: Val::Px(72.0),
            height: Val::Px(40.0),
            ..default()
        },
        UiTransform::IDENTITY,
        Pickable::IGNORE,
    ))
    .with_children(|bar| {
        // Pivot bar.
        bar.spawn((
            Node {
                position_type: PositionType::Absolute,
                left: Val::Px(0.0),
                top: Val::Px(16.0),
                width: Val::Px(72.0),
                height: Val::Px(6.0),
                border_radius: BorderRadius::all(Val::Px(3.0)),
                ..default()
            },
            BackgroundColor(INK),
            Pickable::IGNORE,
        ));
        // Pedal plates.
        for left in [8.0, 46.0] {
            bar.spawn((
                Node {
                    position_type: PositionType::Absolute,
                    left: Val::Px(left),
                    top: Val::Px(21.0),
                    width: Val::Px(18.0),
                    height: Val::Px(25.0),
                    border_radius: BorderRadius::all(Val::Px(4.0)),
                    ..default()
                },
                BackgroundColor(INK),
                Pickable::IGNORE,
            ));
        }
        // Pivot dot.
        bar.spawn((
            Node {
                position_type: PositionType::Absolute,
                left: Val::Px(32.0),
                top: Val::Px(14.0),
                width: Val::Px(8.0),
                height: Val::Px(8.0),
                border_radius: BorderRadius::MAX,
                ..default()
            },
            BackgroundColor(INK),
            Pickable::IGNORE,
        ));
    });

    // Toe-brake slots, below the pedals and outside the rotating bar.
    for left in [RADIUS - 29.0, RADIUS + 17.0] {
        let is_left = left < RADIUS;
        disc.spawn((
            Node {
                position_type: PositionType::Absolute,
                left: Val::Px(left),
                top: Val::Px(DISC - 38.0),
                width: Val::Px(12.0),
                height: Val::Px(BRAKE_H),
                border: UiRect::all(Val::Px(1.0)),
                border_radius: BorderRadius::all(Val::Px(2.0)),
                ..default()
            },
            BackgroundColor(Color::srgba(0.65, 0.70, 0.66, 0.16)),
            BorderColor::all(FAINT),
            Pickable::IGNORE,
        ))
        .with_children(|slot| {
            slot.spawn((
                BrakeFill { left: is_left },
                Node {
                    position_type: PositionType::Absolute,
                    left: Val::Px(0.0),
                    bottom: Val::Px(0.0),
                    width: Val::Percent(100.0),
                    height: Val::Px(0.0),
                    border_radius: BorderRadius::all(Val::Px(1.0)),
                    ..default()
                },
                BackgroundColor(INK),
                Pickable::IGNORE,
            ));
        });
    }
}

fn yoke_offset(aileron: f32, elevator: f32) -> Vec2 {
    Vec2::new(
        aileron.clamp(-1.0, 1.0) * MAX_X,
        elevator.clamp(-1.0, 1.0) * MAX_Y,
    )
}

fn pedal_angle(rudder: f32) -> f32 {
    rudder.clamp(-1.0, 1.0) * 0.35
}

/// Where the two discs go, in logical pixels from the screen's left/bottom:
/// `(bottom, stick_left, pedals_left)`. Wide screens tuck them against the
/// attitude indicator; narrow ones put the pair above it.
fn disc_layout(width: f32) -> (f32, f32, f32) {
    let gap = (width - 2.0 * gauge::CLUSTER_W - gauge::NODE) * 0.5;
    let attitude_half = gauge::NODE * 0.5;
    let stick = width * 0.5 - attitude_half - DISC_GAP - DISC;
    let pedal = width * 0.5 + attitude_half + DISC_GAP;
    if gap >= DISC + 32.0 && stick >= gauge::CLUSTER_W + 24.0 {
        (gauge::MARGIN, stick, pedal)
    } else {
        (
            gauge::MARGIN + gauge::NODE + gauge::GAP,
            width * 0.5 - DISC - 8.0,
            width * 0.5 + 8.0,
        )
    }
}

/// Place the discs beside the attitude indicator, or above it when the window
/// is too narrow to fit them between the gauge trios.
fn layout_indicators(
    state: Res<State<AppState>>,
    sim: Res<Sim>,
    ui: Res<Ui>,
    windows: Query<&Window, With<PrimaryWindow>>,
    mut roots: Query<(&ControlRoot, &mut Node)>,
) {
    let want = *state.get() == AppState::Flying
        && aircraft_profile::normalized(&sim.name) == "c172p"
        && !ui.cockpit;
    let Ok(window) = windows.single() else {
        return;
    };
    let (bottom, stick_left, pedal_left) = disc_layout(window.width());

    for (root, mut node) in &mut roots {
        node.display = if want { Display::Flex } else { Display::None };
        node.bottom = Val::Px(bottom);
        node.left = Val::Px(match root.0 {
            Side::Stick => stick_left,
            Side::Pedals => pedal_left,
        });
    }
}

fn update_stick(
    state: Res<AircraftState>,
    bindings: Res<Bindings>,
    keys: Res<ButtonInput<KeyCode>>,
    mut yoke: Query<&mut UiTransform, With<YokeGlyph>>,
    mut trim: Query<&mut UiTransform, (With<TrimMark>, Without<YokeGlyph>)>,
    mut dots: Query<(&mut BackgroundColor, &CenterDot)>,
    mut borders: Query<&mut BorderColor, With<ControlRoot>>,
) {
    // White glyph = the pilot's key input; the assist term is shown by the
    // BANK annunciation, not by moving the pilot's yoke.
    let aileron = pilot(&bindings, Action::Aileron);
    let elevator = state.elevator as f32;

    for mut tf in &mut yoke {
        let offset = yoke_offset(aileron, elevator);
        tf.translation = Val2::px(offset.x, offset.y);
        tf.rotation = Rot2::radians(aileron * 0.22);
    }
    for mut tf in &mut trim {
        tf.translation = Val2::px(0.0, state.elevator_trim.clamp(-1.0, 1.0) as f32 * MAX_Y);
    }

    let fine = keys.pressed(KeyCode::Space)
        || keys.pressed(KeyCode::AltLeft)
        || keys.pressed(KeyCode::AltRight);
    for (mut color, dot) in &mut dots {
        let centered = match dot.side {
            Side::Stick => elevator.abs() < 0.03,
            Side::Pedals => pilot(&bindings, Action::Rudder).abs() < 0.03,
        };
        color.0 = if centered { INK } else { DIM };
    }
    let at_limit = [elevator, state.aileron as f32, state.rudder as f32]
        .iter()
        .any(|v| v.abs() > 0.98);
    for mut border in &mut borders {
        *border = BorderColor::all(if fine {
            AMBER
        } else if at_limit {
            INK
        } else {
            BEZEL
        });
    }
}

fn pilot(bindings: &Bindings, action: Action) -> f32 {
    bindings
        .list
        .iter()
        .find(|b| b.action == action)
        .map_or(0.0, |b| b.value as f32)
}

fn update_pedals(
    state: Res<AircraftState>,
    bindings: Res<Bindings>,
    mut bar: Query<&mut UiTransform, With<PedalBar>>,
    mut fills: Query<(&BrakeFill, &mut Node)>,
) {
    // White pedals = pilot rudder input; COORD is the assist annunciation.
    let rudder = pilot(&bindings, Action::Rudder);
    for mut tf in &mut bar {
        tf.rotation = Rot2::radians(pedal_angle(rudder));
    }
    for (fill, mut node) in &mut fills {
        let value = if fill.left {
            state.brake_left
        } else {
            state.brake_right
        };
        node.height = Val::Px(value.clamp(0.0, 1.0) as f32 * BRAKE_H);
    }
}

fn update_assist_labels(assist: Res<AssistState>, mut labels: Query<(&AssistLabel, &mut Node)>) {
    for (label, mut node) in &mut labels {
        let on = match label.side {
            Side::Stick => assist.bank_active,
            Side::Pedals => assist.coord_active,
        };
        node.display = if on { Display::Flex } else { Display::None };
    }
}

#[cfg(test)]
mod tests {
    use super::{disc_layout, gauge, pedal_angle, yoke_offset, DISC, MAX_X, MAX_Y};
    use bevy::math::Vec2;
    use bevy::prelude::*;

    #[test]
    fn yoke_moves_toward_the_command() {
        // D rolls right, so the yoke moves right; W is nose-up, which is a
        // negative elevator command and must move the glyph up the screen.
        let right = yoke_offset(1.0, 0.0);
        assert!((right - Vec2::new(MAX_X, 0.0)).length() < 1e-6);
        let up = yoke_offset(0.0, -1.0);
        assert!((up - Vec2::new(0.0, -MAX_Y)).length() < 1e-6);
        // And it clamps.
        let clamped = yoke_offset(4.0, -4.0);
        assert!((clamped - Vec2::new(MAX_X, -MAX_Y)).length() < 1e-6);
    }

    #[test]
    fn pedal_seesaw_turns_with_rudder() {
        assert_eq!(pedal_angle(0.0), 0.0);
        assert!(pedal_angle(1.0) > 0.0);
        assert!(pedal_angle(-1.0) < 0.0);
        assert_eq!(pedal_angle(9.0), pedal_angle(1.0));
    }

    #[test]
    fn discs_sit_beside_the_attitude_gauge_on_a_wide_screen() {
        let (bottom, stick, pedals) = disc_layout(1920.0);
        assert_eq!(bottom, gauge::MARGIN);
        // Tucked 18 px either side of the attitude indicator, still clear of
        // both gauge trios.
        assert!((stick - 704.0).abs() < 1e-3, "stick at {stick}");
        assert!((pedals - 1076.0).abs() < 1e-3, "pedals at {pedals}");
        assert!(stick >= gauge::CLUSTER_W);
        assert!(stick + DISC + 18.0 <= 960.0 - gauge::NODE * 0.5 + 1e-3);
        assert!(pedals >= 960.0 + gauge::NODE * 0.5);
        assert!(pedals + DISC <= 1920.0 - gauge::CLUSTER_W);
    }

    #[test]
    fn discs_move_above_the_attitude_gauge_when_the_gaps_collapse() {
        let (bottom, stick, pedals) = disc_layout(1600.0);
        assert!(bottom > gauge::MARGIN + gauge::NODE);
        assert!((stick + DISC + 8.0 - 800.0).abs() < 1e-3);
        assert!((pedals - 8.0 - 800.0).abs() < 1e-3);
    }

    #[test]
    fn the_stick_system_writes_the_command_into_the_ui() {
        use bevy::prelude::{App, Update, Val};

        let mut app = App::new();
        app.insert_resource(crate::sim::AircraftState {
            aileron: 0.5,
            elevator: -0.5,
            elevator_trim: 0.25,
            ..default()
        });
        let mut bindings =
            crate::input::Bindings::for_aircraft("c172p", &crate::config::Keymap::default());
        if let Some(b) = bindings
            .list
            .iter_mut()
            .find(|b| b.action == crate::aircraft_profile::Action::Aileron)
        {
            b.value = 0.5;
        }
        app.insert_resource(bindings);
        app.init_resource::<ButtonInput<KeyCode>>();
        let yoke = app
            .world_mut()
            .spawn((super::YokeGlyph, UiTransform::IDENTITY))
            .id();
        let trim = app
            .world_mut()
            .spawn((super::TrimMark, UiTransform::IDENTITY))
            .id();
        app.add_systems(Update, super::update_stick);
        app.update();

        let tf = app.world().get::<UiTransform>(yoke).unwrap();
        let Val::Px(x) = tf.translation.x else {
            panic!("x is not px: {:?}", tf.translation.x);
        };
        let Val::Px(y) = tf.translation.y else {
            panic!("y is not px: {:?}", tf.translation.y);
        };
        assert!((x - MAX_X * 0.5).abs() < 1e-6);
        assert!((y + MAX_Y * 0.5).abs() < 1e-6);

        let tf = app.world().get::<UiTransform>(trim).unwrap();
        let Val::Px(y) = tf.translation.y else {
            panic!("trim y is not px: {:?}", tf.translation.y);
        };
        assert!((y - MAX_Y * 0.25).abs() < 1e-6);
    }
}
