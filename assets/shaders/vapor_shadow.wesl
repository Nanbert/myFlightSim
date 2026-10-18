// Wing-shaped soft shadow under the condensation layer. The mesh is the same
// upper-wing grid used by the old sheet, so the dark pool follows the planform
// and anchors the white layer against the white wing.

import bevy_pbr::render::forward_io::VertexOutput;

struct ShadowParams {
    rgb: vec4<f32>,
    anim: vec4<f32>,
};

@group(constants::MATERIAL_BIND_GROUP) @binding(0) var<uniform> params: ShadowParams;

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
    let uv = in.uv;
    let span = smoothstep(0.0, 0.06, uv.x) * (1.0 - smoothstep(0.78, 1.0, uv.x));
    let chord = smoothstep(0.0, 0.06, uv.y) * (1.0 - smoothstep(0.72, 1.0, uv.y));
    let n = noise2(uv * 6.0 + vec2<f32>(params.anim.y * 0.15, 0.0));
    let a = params.anim.x * span * chord * (0.55 + 0.45 * n);
    if (a < 0.003) {
        discard;
    }
    return vec4<f32>(params.rgb.rgb, a);
}
