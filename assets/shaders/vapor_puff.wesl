// Wingtip condensation puff.
//
// One soft round billboard per trail sample (see `src/vapor.rs`). The quad is
// the atlas: a radial mask built from the UV, with a soft edge, so the puffs read
// as round clouds rather than squares. Opacity comes in per puff from the
// uniform, so each fades with its own age.

import bevy_pbr::render::forward_io::VertexOutput;

struct PuffParams {
    // rgb = vapor colour, a = opacity.
    color: vec4<f32>,
    // x = soft-edge threshold (0..1); y/z/w unused.
    anim: vec4<f32>,
};

@group(constants::MATERIAL_BIND_GROUP) @binding(0) var<uniform> params: PuffParams;

@fragment
fn fragment(in: VertexOutput) -> @location(0) vec4<f32> {
    // Radial distance over the quad: 0 at the centre, 1 at the edge midpoints
    // (and further at the corners, so they mask off cleanly).
    let d = length(in.uv - vec2<f32>(0.5)) * 2.0;
    let a = (1.0 - smoothstep(params.anim.x, 1.0, d)) * params.color.a;

    // Nothing to blend once the puff has faded.
    if (a < 0.004) {
        discard;
    }
    return vec4<f32>(params.color.rgb, a);
}
