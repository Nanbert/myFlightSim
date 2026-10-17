// F-22A head-up display, nav mode — full-screen `UiMaterial` overlay.
//
// The symbology itself lives in `f22_hud_sym.wesl`, shared with the 3D
// combiner-glass material. This entry just lays it out against the viewport,
// which keeps its proportions at any window shape.

import bevy_ui_render::ui_vertex_output::UiVertexOutput;
import super::f22_hud_sym::{HudParams, sym};

@group(1) @binding(0) var<uniform> u: HudParams;

@fragment
fn fragment(in: UiVertexOutput) -> @location(0) vec4<f32> {
    return sym(u, in.uv * in.size, in.size);
}
