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
    let cov = clamp(params.rgb.w, 0.25, 1.0);
    let root_bias = 1.0 - cov;
    // Low speed: a small pool near the root/LE. High speed: most of the wing.
    let cu = 0.42 - 0.24 * root_bias;
    let cv = 0.20;
    let d = length(vec2<f32>((uv.x - cu) * 1.1, (uv.y - cv) * 0.8));
    let patch = 1.0 - smoothstep(0.12, 0.38, d / cov);
    let n = noise2(uv * 6.0 + vec2<f32>(params.anim.y * 0.15, 0.0));
    let a = params.anim.x * patch * (0.55 + 0.45 * n);
    if (a < 0.003) {
        discard;
    }
    return vec4<f32>(params.rgb.rgb, a);
}
