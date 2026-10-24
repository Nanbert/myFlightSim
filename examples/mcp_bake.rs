//! Bake the 737 MCP's 3D model to `assets/ui/737_mcp_3d.png`.
//!
//! Run: `cargo run --features shots --example mcp_bake`. The window is only a
//! host for the render device; the panel is rendered to an offscreen 4940x898
//! target and saved, then the app exits.

use bevy::prelude::*;
use my_flight_sim::mcp_bake::McpBakePlugin;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "mcp_bake".into(),
                resolution: (800, 200).into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(McpBakePlugin::new("assets/ui/737_mcp_3d.png"))
        .run();
}
