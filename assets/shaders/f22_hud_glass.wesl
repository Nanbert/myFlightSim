// F-22A head-up display — combiner-glass material.
//
// Draws the shared symbology (`f22_hud_sym.wesl`) on a quad that sits on the
// cockpit model's combiner glass, tilted with it. The layout is laid out on a
// virtual screen (`u.f.xy`) rather than the viewport, and shifted down (`u.f.z`)
// so its content centres on the glass.

import bevy_pbr::render::forward_io::VertexOutput;
import super::f22_hud_sym::{HudParams, sym};

@group(constants::MATERIAL_BIND_GROUP) @binding(0) var<uniform> u: HudParams;

@fragment
fn fragment(in: VertexOutput) -> @location(0) vec4<f32> {
    let size = u.f.xy;
    // v grows downward over the quad, matching the layout's screen convention.
    let p = in.uv * size - vec2<f32>(0.0, u.f.z * size.y);
    return sym(u, p, size);
}
