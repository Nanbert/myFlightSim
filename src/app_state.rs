//! The top-level app state, shared by every plugin. It lives in its own module
//! (rather than in `menu`) so the flight systems can gate on it without
//! depending on the menu module.

use bevy::prelude::*;

/// `Menu` is the launch screen; `Flying` runs the sim. Every flight system
/// carries `run_if(in_state(AppState::Flying))`.
#[derive(States, Default, Clone, Copy, PartialEq, Eq, Hash, Debug)]
pub enum AppState {
    #[default]
    Menu,
    Flying,
}
