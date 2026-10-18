// Wingtip condensation ribbon segment.
//
// One flow-aligned quad per trail sample (see `src/vapor.rs`). The quad's local
// X is across the ribbon and local Y runs along the flow, so the UVs map to
// width/length rather than to a radial disc. The mask is a soft-edged sheet
// with feathered ends and a little longitudinal tearing, which reads as vapor
// being pulled off the wing instead of a row of round puffs.

import bevy_pbr::render::forward_io::VertexOutput;

struct PuffParams {
    // rgb = vapor colour, a = opacity.
    color: vec4<f32>,
    // x = edge softness (0..1); y/z/w unused.
    anim: vec4<f32>,
};

@group(constants::MATERIAL_BIND_GROUP) @binding(0) var<uniform> params: PuffParams;

fn hash21(p: vec2<f32>) -> f32 {
    var q = fract(p * vec2<f32>(0.1031, 0.1030));
    q = q + dot(q, q.yx + 33.33);
    return fract(q.x * q.y);
}

fn noise2(p: vec2<f32>) -> f32 {
    let i = floor(p);
    let f = fract(p);
    let u = f * f * (3.0 - 2.0 * f);
    let a = hash21(i);
    let b = hash21(i + vec2<f32>(1.0, 0.0));
    let c = hash21(i + vec2<f32>(0.0, 1.0));
    let d = hash21(i + vec2<f32>(1.0, 1.0));
    return mix(mix(a, b, u.x), mix(c, d, u.x), u.y);
}

@fragment
fn fragment(in: VertexOutput) -> @location(0) vec4<f32> {
    let soft = 0.07 + 0.10 * params.anim.x;
    // Across the ribbon: a soft edge on both sides.
    let edge = smoothstep(0.0, soft, in.uv.x)
        * (1.0 - smoothstep(1.0 - soft, 1.0, in.uv.x));
    // Along the ribbon: feather the head, and pull the tail out into a wisp.
    let ends = smoothstep(0.0, 0.16, in.uv.y)
        * (1.0 - smoothstep(0.55, 1.0, in.uv.y));
    // A few longitudinal filaments so the sheet tears instead of reading as a
    // clean strip. Animated by the material's opacity in later stages.
    let seed = params.anim.y;
    let filament = 0.62
        + 0.38 * noise2(vec2(in.uv.x * 4.0 + seed * 7.0, in.uv.y * 14.0 + seed * 3.0));
    let a = edge * ends * filament * params.color.a;

    if (a < 0.003) {
        discard;
    }
    return vec4<f32>(params.color.rgb, a);
}
