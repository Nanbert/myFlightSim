// Boeing 737 PFD, drawn on the cockpit's display units.
//
// Same symbology as the 2D corner overlays (`pfd.wesl`), mapped onto a quad
// laid over the model's black screen. The layout is proportional, so the
// virtual size only sets the resolution — the quad's physical size decides how
// large the display reads in the cockpit.

import bevy_pbr::render::forward_io::VertexOutput;
import super::pfd_sym::{PfdParams, pfd};

@group(constants::MATERIAL_BIND_GROUP) @binding(0) var<uniform> u: PfdParams;

const SIZE: vec2<f32> = vec2<f32>(400.0, 430.0);

@fragment
fn fragment(in: VertexOutput) -> @location(0) vec4<f32> {
    return pfd(u, in.uv * SIZE, SIZE);
}
