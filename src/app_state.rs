//! The top-level app state, shared by every plugin. It lives in its own module
//! (rather than in `menu`) so the flight systems can gate on it without
//! depending on the menu module.

use bevy::prelude::*;

/// `Menu` is the launch screen; `Flying` runs the sim. Every flight system
/// carries `run_if(unpaused())`: flying, and not paused.
#[derive(States, Default, Clone, Copy, PartialEq, Eq, Hash, Debug)]
pub enum AppState {
    #[default]
    Menu,
    Flying,
}

/// Present while the flight is paused (the Esc dialog). Its existence alone
/// freezes every flight system, since they all gate through [`unpaused`].
#[derive(Resource)]
pub struct Paused;

/// Shared gate for all flight systems: running the sim only while `Flying`
/// and there is no pause dialog. Function used as a run condition.
pub fn unpaused(state: Res<State<AppState>>, paused: Option<Res<Paused>>) -> bool {
    *state.get() == AppState::Flying && paused.is_none()
}

/// The pause dialog systems run while `Flying`, paused or not, so Esc can
/// open (and then close) the dialog no matter what.
pub fn flying(state: Res<State<AppState>>) -> bool {
    *state.get() == AppState::Flying
}
