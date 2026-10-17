// Boeing 737 primary flight display, drawn procedurally.
//
// One `UiMaterial` node per pilot, both fed the same uniform so the captain's
// and first officer's displays cannot disagree. The symbology lives in
// `pfd_sym.wesl`, shared with the 3D cockpit screens (`pfd3d.wesl`).

import bevy_ui_render::ui_vertex_output::UiVertexOutput;
import super::pfd_sym::{PfdParams, pfd};

@group(1) @binding(0) var<uniform> u: PfdParams;

@fragment
fn fragment(in: UiVertexOutput) -> @location(0) vec4<f32> {
    return pfd(u, in.uv * in.size, in.size);
}
