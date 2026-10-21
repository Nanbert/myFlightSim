//! Pause: Escape no longer throws the flight away. It freezes the sim and
//! asks, in a backlit card over the frozen scene:
//!
//! ```text
//!        EXIT TO MENU ?
//!   The flight stays where it is; OK discards it.
//!   | OK - EXIT TO MENU |
//!   [Enter] OK     [Esc] BACK
//! ```
//!
//! Enter returns to the hangar (the same semantics the old direct Esc had:
//! the flight is discarded). Esc pops straight back into the running
//! aircraft.
//!
//! Freezing is resource-driven: opening the dialog inserts [`Paused`], and
//! every flight system gates through `app_state::unpaused`, so the model,
//! camera and HUD stop on the same frame the card appears.

use bevy::prelude::*;

use crate::app_state::{flying, AppState, Paused};
use crate::ui_theme;

#[derive(Component)]
struct PauseCard;

/// One styled line of the card.
struct CardLine {
    text: &'static str,
    color: Color,
    size: f32,
    /// Extra space above (blank-line equivalents, like the page rows).
    gap: f32,
    /// Renders the current action row: lit bar + accent rail.
    is_ok_row: bool,
}

pub struct PausePlugin;

impl Plugin for PausePlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Update, toggle_pause.run_if(flying))
            // Whatever way a flight ends, the flag never leaks.
            .add_systems(OnExit(AppState::Flying), clear_pause)
            // Development hook: `PAUSE=1` boots the flight paused (with the
            // dialog up), so a screenshot pass can inspect the card without
            // driving keys.
            .add_systems(OnEnter(AppState::Flying), dev_pause);
    }
}

/// `PAUSE=1` opens the pause dialog as soon as a flight stages.
fn dev_pause(mut commands: Commands) {
    if std::env::var("PAUSE").is_err() {
        return;
    }
    commands.insert_resource(Paused);
    spawn_card(&mut commands);
}

/// Escape owns the pause state machine: open on Esc, resume on Esc, confirm
/// to the hangar on Enter. Runs whether paused or not, so the dialog always
/// answers a keypress even though every other flight system is frozen.
fn toggle_pause(
    keys: Res<ButtonInput<KeyCode>>,
    state: Res<State<AppState>>,
    paused: Option<Res<Paused>>,
    card: Query<Entity, With<PauseCard>>,
    mut next: ResMut<NextState<AppState>>,
    mut commands: Commands,
) {
    if *state.get() != AppState::Flying {
        return;
    }
    let esc = keys.just_pressed(KeyCode::Escape);
    let enter = keys.just_pressed(KeyCode::Enter) || keys.just_pressed(KeyCode::NumpadEnter);
    let open = paused.is_some();
    let mut card = card.single().ok();

    if esc && !open {
        commands.insert_resource(Paused);
        spawn_card(&mut commands);
        return;
    }
    if !open {
        return;
    }
    if esc {
        commands.remove_resource::<Paused>();
        if let Some(e) = card.take() {
            commands.entity(e).despawn();
        }
    } else if enter {
        // Reaching the hangar is the flight-ending path: `clear_pause`
        // (OnExitFlying) drops the flag, `DespawnOnExit` removes the card.
        next.set(AppState::Menu);
    }
}

/// The confirmation card: the hangar page's avionics look, sized like a
/// caution plate instead of a full page.
fn spawn_card(commands: &mut Commands) {
    let root = commands
        .spawn((
            Node {
                position_type: PositionType::Absolute,
                width: Val::Percent(100.0),
                height: Val::Percent(100.0),
                align_items: AlignItems::Center,
                justify_content: JustifyContent::Center,
                display: Display::Flex,
                ..default()
            },
            // The scene dims under the card, same as the hangar's panel.
            BackgroundColor(Color::srgba(0.0, 0.0, 0.0, 0.55)),
            PauseCard,
            DespawnOnExit(AppState::Flying),
        ))
        .id();
    let card = commands
        .spawn((
            Node {
                flex_direction: FlexDirection::Column,
                width: Val::Px(640.0),
                padding: UiRect {
                    left: Val::Px(40.0),
                    right: Val::Px(40.0),
                    top: Val::Px(28.0),
                    bottom: Val::Px(24.0),
                },
                border: UiRect::all(Val::Px(1.0)),
                border_radius: BorderRadius::all(Val::Px(10.0)),
                ..default()
            },
            BorderColor::all(ui_theme::accent_soft()),
            BackgroundColor(ui_theme::panel_bg()),
            BoxShadow::new(
                ui_theme::panel_glow(),
                Val::ZERO,
                Val::ZERO,
                Val::Px(18.0),
                Val::Px(56.0),
            ),
            ChildOf(root),
        ))
        .id();
    for line in [
        CardLine { text: "EXIT TO MENU ?", color: ui_theme::accent(), size: 26.0, gap: 0.0, is_ok_row: false },
        CardLine { text: "The flight stays where it is; OK discards it.", color: ui_theme::dim(), size: 18.0, gap: 8.0, is_ok_row: false },
        CardLine { text: "OK - EXIT TO MENU", color: ui_theme::text(), size: 22.0, gap: 18.0, is_ok_row: true },
        CardLine { text: "[Enter] OK     [Esc] BACK", color: ui_theme::dim(), size: 16.0, gap: 18.0, is_ok_row: false },
    ] {
        // The OK row borrows the page cursor look: translucent bar + rail.
        let (padding, border) = match line.is_ok_row {
            true => (UiRect::all(Val::Px(8.0)), UiRect::left(Val::Px(3.0))),
            false => (UiRect::ZERO, UiRect::ZERO),
        };
        commands.spawn((
            Node {
                margin: UiRect::top(Val::Px(line.gap)),
                padding,
                border,
                border_radius: BorderRadius::all(Val::Px(4.0)),
                ..default()
            },
            BorderColor::all(if line.is_ok_row {
                ui_theme::accent()
            } else {
                Color::NONE
            }),
            BackgroundColor(if line.is_ok_row {
                ui_theme::accent_soft()
            } else {
                Color::NONE
            }),
            Text::new(line.text.to_string()),
            TextFont {
                font_size: FontSize::Px(line.size),
                ..default()
            },
            TextColor(line.color),
            ChildOf(card),
        ));
    }
}

/// There is no pause without a flight.
fn clear_pause(mut commands: Commands) {
    commands.remove_resource::<Paused>();
}
