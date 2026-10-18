//! Marker for the offscreen camera that raymarches the clouds.
//!
//! It lives in its own dependency-free module because the scene examples
//! include `src` modules directly with `#[path]`, and every one of them needs
//! to exclude this camera from its "the camera" queries — without pulling the
//! whole sky plugin into the example.

use bevy::prelude::*;

/// The half-resolution camera that renders `assets/shaders/cloud_sky.wesl`
/// into the cloud target. It shares the main camera's pose, so any query that
/// expects exactly one `Camera3d` must exclude it.
#[derive(Component)]
pub struct CloudCamera;
