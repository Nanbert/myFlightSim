// AH-64D IHADSS — full-screen UiMaterial overlay.
//
// The symbology lives in `ah64_hud_sym.wesl`; this entry lays it out against
// the viewport, which keeps its proportions at any window shape.

import bevy_ui_render::ui_vertex_output::UiVertexOutput;
import super::ah64_hud_sym::{Ah64Params, sym};

@group(1) @binding(0) var<uniform> u: Ah64Params;

@fragment
fn fragment(in: UiVertexOutput) -> @location(0) vec4<f32> {
    return sym(u, in.uv * in.size, in.size);
}
