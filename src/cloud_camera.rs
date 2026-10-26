//! Legacy marker for the old offscreen cloud camera.
//!
//! Clouds are now fixed world-space cards, so no `CloudCamera` entity is
//! spawned. The marker stays because many camera queries exclude it and the
//! scene examples import this module directly with `#[path]`.

use bevy::prelude::*;

/// No entity currently carries this component; it is kept as a query filter
/// for compatibility with the old offscreen cloud camera.
#[derive(Component)]
pub struct CloudCamera;
