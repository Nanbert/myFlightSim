//! Geometry shared by the 2D panel layouts.
//!
//! Both the 737 MCP and the control stand are authored in a fixed asset-pixel
//! space and drawn at whatever size the window gives them. `PRect` is that
//! space's rectangle: the layouts, the bake tools and the runtime hotspots all
//! speak in the same coordinates, so a control cannot drift from its art.

use bevy::prelude::Vec2;

/// A rectangle in baked-image pixels.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PRect {
    pub x0: f32,
    pub y0: f32,
    pub x1: f32,
    pub y1: f32,
}

impl PRect {
    pub const fn new(x0: f32, y0: f32, x1: f32, y1: f32) -> Self {
        Self { x0, y0, x1, y1 }
    }

    pub fn cx(self) -> f32 {
        (self.x0 + self.x1) * 0.5
    }

    pub fn cy(self) -> f32 {
        (self.y0 + self.y1) * 0.5
    }

    pub fn w(self) -> f32 {
        self.x1 - self.x0
    }

    pub fn h(self) -> f32 {
        self.y1 - self.y0
    }

    pub fn contains(self, p: Vec2) -> bool {
        p.x >= self.x0 && p.x <= self.x1 && p.y >= self.y0 && p.y <= self.y1
    }

    pub fn expand(self, d: f32) -> Self {
        Self::new(self.x0 - d, self.y0 - d, self.x1 + d, self.y1 + d)
    }

    /// `left, top, width, height` for a panel drawn `scale` asset px per
    /// on-screen pixel.
    pub fn scaled(self, scale: f32) -> (f32, f32, f32, f32) {
        (
            self.x0 * scale,
            self.y0 * scale,
            self.w() * scale,
            self.h() * scale,
        )
    }
}
