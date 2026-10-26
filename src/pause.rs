//! Pause: Escape no longer throws the flight away. It freezes the sim and
//! asks, in a backlit card over the frozen scene:
//!
//! ```text
//!        EXIT TO MENU ?
//!   The flight stays where it is; Enter discards it.
//!   [Enter] Back to menu        [Esc] Back to game
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
}

pub struct PausePlugin;

impl Plugin for PausePlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Update, toggle_pause.run_if(flying).run_if(no_key_owner))
            // Whatever way a flight ends, the flag never leaks.
            .add_systems(OnExit(AppState::Flying), clear_pause)
            // Development hook: `PAUSE=1` boots the flight paused (with the
            // dialog up), so a screenshot pass can inspect the card without
            // driving keys.
            // After `start_flight`: the card names the lesson being retried, and
            // `ActiveLesson` is only final by the end of the StartFlight set.
            .add_systems(OnEnter(AppState::Flying), dev_pause.after(crate::sim::StartFlight));
    }
}

/// `PAUSE=1` opens the pause dialog as soon as a flight stages.
fn dev_pause(mut commands: Commands, active: Res<crate::mission::ActiveLesson>) {
    if std::env::var("PAUSE").is_err() {
        return;
    }
    commands.insert_resource(Paused);
    spawn_card(&mut commands, active.0);
}

/// Condition for the pause menu: true when no other surface owns the flight's
/// keys.
///
/// This is fed to `run_if` **without** `not` — it already returns the runnable
/// case. Inverting it makes the menu run exactly when it must not: Esc would
/// freeze the flight while the panel is still closing, and the very next Esc
/// would find the condition false and be ignored, so the dialog sat there dead.
///
/// A lesson popup pins the flight with its own rows (see `lesson_flow`), and
/// MCP focus owns Esc while the cockpit panel is up — `mcp_ui::focus_input`
/// closes it, and only a second, fresh Esc may reach this menu. Without the
/// focus check the pause card swallowed the first press: `focus_input` is
/// gated on `unpaused`, so once `Paused` landed in the same frame it never ran.
/// `run_if` conditions read the world once per frame, so this holds whatever
/// order the two systems happen to run in.
///
/// The combat mission card owns the keys too: its Esc is "back to menu", and
/// Enter/R must not fall through to a pause dialog.
fn no_key_owner(
    card: Option<Res<crate::lesson_flow::FlowCard>>,
    ui: Option<Res<crate::view::Ui>>,
    over: Option<Res<crate::combat::MissionOver>>,
) -> bool {
    keys_free(
        crate::lesson_flow::modal(card),
        crate::view::panel_focused(ui.as_deref()),
        over.is_some(),
    )
}

/// The pause menu's key line is free while neither a lesson popup, the focused
/// MCP panel, nor the combat mission card owns the flight's keys.
fn keys_free(modal_open: bool, panel_focus: bool, mission_card: bool) -> bool {
    !modal_open && !panel_focus && !mission_card
}

/// Escape owns the pause state machine: open on Esc, resume on Esc, confirm
/// to the hangar on Enter. Runs whether paused or not, so the dialog always
/// answers a keypress even though every other flight system is frozen.
fn toggle_pause(
    keys: Res<ButtonInput<KeyCode>>,
    state: Res<State<AppState>>,
    paused: Option<Res<Paused>>,
    active: Res<crate::mission::ActiveLesson>,
    mut pending: ResMut<crate::mission::PendingLesson>,
    card: Query<Entity, With<PauseCard>>,
    mut next: ResMut<NextState<AppState>>,
    mut commands: Commands,
) {
    if *state.get() != AppState::Flying {
        return;
    }
    let esc = keys.just_pressed(KeyCode::Escape);
    let retry = keys.just_pressed(KeyCode::KeyR);
    let enter = keys.just_pressed(KeyCode::Enter) || keys.just_pressed(KeyCode::NumpadEnter);
    let open = paused.is_some();
    let mut card = card.single().ok();

    if esc && !open {
        commands.insert_resource(Paused);
        spawn_card(&mut commands, active.0);
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
    } else if retry {
        if let Some(lesson) = active.0 {
            // The lesson cards' own retry, opened up to the pause plate: the
            // lesson rides pending through the hangar, whose `OnEnter`
            // chains rebuild the flight exactly as a menu start does.
            pending.0 = Some(lesson);
            commands.remove_resource::<Paused>();
            if let Some(e) = card.take() {
                commands.entity(e).despawn();
            }
            next.set(AppState::Menu);
        }
    } else if enter {
        // Reaching the hangar is the flight-ending path: `clear_pause`
        // (OnExitFlying) drops the flag, `DespawnOnExit` removes the card.
        next.set(AppState::Menu);
    }
}

/// The confirmation card: the hangar page's avionics look, sized like a
/// caution plate instead of a full page. A lesson in progress offers its R
/// retry on the footer; free flight keeps the plain two-key plate.
fn spawn_card(commands: &mut Commands, retry: Option<crate::mission::Lesson>) {
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
            // Same layering as the lesson popups: above the cockpit overlay
            // and HUD panes (z 15-20), below combat/MCP (30+).
            GlobalZIndex(25),
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
        CardLine { text: "PAUSED", color: ui_theme::accent(), size: 26.0, gap: 0.0 },
        CardLine { text: "The flight stays where it is; Enter discards it.", color: ui_theme::dim(), size: 18.0, gap: 8.0 },
    ] {
        commands.spawn((
            Node {
                margin: UiRect::top(Val::Px(line.gap)),
                ..default()
            },
            Text::new(line.text.to_string()),
            TextFont {
                font_size: FontSize::Px(line.size),
                ..default()
            },
            TextColor(line.color),
            ChildOf(card),
        ));
    }
    // The hint footer, the same dim style as every other key row
    // (`ui_theme::key_footer`): no lit bar, just the keys.
    let mut rows = vec![("Enter", "Back to menu"), ("Esc", "Back to game")];
    if retry.is_some() {
        rows.insert(0, ("R", "Retry lesson"));
    }
    ui_theme::key_footer(commands, card, 20.0, &rows);
}

/// There is no pause without a flight.
fn clear_pause(mut commands: Commands) {
    commands.remove_resource::<Paused>();
}

#[cfg(test)]
mod tests {
    use super::keys_free;
    use crate::view::{panel_focused, PanelFocus, Ui};

    /// MCP focus owns Esc, so the pause menu must stand aside: one press
    /// closes the panel, not the flight.
    #[test]
    fn panel_focus_holds_the_flight_keys() {
        let clear = Ui::default();
        assert!(!panel_focused(Some(&clear)));

        let focused = Ui {
            panel_focus: Some(PanelFocus::Mcp),
            ..Default::default()
        };
        assert!(panel_focused(Some(&focused)));

        // No `Ui` at all (a hangar or menu frame) never blocks the menu.
        assert!(!panel_focused(None));
    }

    /// The truth table behind `no_key_owner`. It is used with `run_if` and no
    /// `not`, so it must return the runnable case — this pins that.
    #[test]
    fn the_pause_menu_waits_for_a_free_key_line() {
        assert!(keys_free(false, false, false)); // nothing else on screen: Esc pauses
        assert!(!keys_free(true, false, false)); // a lesson popup owns the keys
        assert!(!keys_free(false, true, false)); // the MCP panel owns Esc
        assert!(!keys_free(true, true, false));
        assert!(!keys_free(false, false, true)); // the mission card owns Esc
    }
}
