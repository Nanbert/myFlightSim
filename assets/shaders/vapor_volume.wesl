// Wing boundary-layer volume puff.
//
// One camera-facing billboard per puff (see `src/vapor.rs`). The puffs are
// distributed through the first few tens of centimetres above the wing and
// follow the airframe, so they read as a cloud volume that gathers on the wing
// and dissipates in place. A noise-eroded radial mask plus a hemisphere normal
// keeps the individual puffs from looking like flat discs.

import bevy_pbr::render::forward_io::VertexOutput;
import bevy_pbr::render::mesh_view_bindings::view;

struct VolumeParams {
    // rgb = vapour colour, a = opacity.
    color: vec4<f32>,
    // x = seed, y = time (s), zw unused.
    anim: vec4<f32>,
    // xyz = direction toward the sun.
    light: vec4<f32>,
};

@group(constants::MATERIAL_BIND_GROUP) @binding(0) var<uniform> params: VolumeParams;

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

fn fbm2(p: vec2<f32>) -> f32 {
    var sum = 0.0;
    var amp = 0.5;
    var q = p;
    for (var i = 0; i < 4; i = i + 1) {
        sum += amp * noise2(q);
        q = q * 2.03 + vec2<f32>(17.1, 9.2);
        amp *= 0.5;
    }
    return sum;
}

@fragment
fn fragment(in: VertexOutput) -> @location(0) vec4<f32> {
    let alpha0 = params.color.a;
    if (alpha0 < 0.002) {
        discard;
    }

    let p = in.uv * 2.0 - 1.0;
    let d = length(p);
    let seed = params.anim.x;
    let t = params.anim.y;

    // Large billows plus fine breakup; the seed decorrelates every puff.
    let n = fbm2(in.uv * 4.0 + vec2<f32>(seed * 17.0, t * 0.50)) * 0.70
        + fbm2(in.uv * 8.0 - vec2<f32>(t * 0.80, seed * 3.0)) * 0.30;

    // Erode the silhouette, keep a dense core.
    let edge = 1.0 - smoothstep(0.30, 1.0, d + (n - 0.5) * 0.55);
    let core = 1.0 - smoothstep(0.0, 0.72, d);
    let mask = edge * (0.50 + 0.50 * core);
    if (mask < 0.003) {
        discard;
    }

    // The puffs are randomly oriented planes, so light the actual face normal
    // (two-sided) instead of a screen-space hemisphere.
    let nrm = normalize(in.world_normal);
    let sun = normalize(params.light.xyz);
    let V = normalize(view.world_position - in.world_position.xyz);
    let ndl = abs(dot(nrm, sun));
    let back = pow(max(dot(-V, sun), 0.0), 3.0);
    let lit = clamp(0.35 + 0.65 * ndl + 0.40 * back, 0.0, 1.6);
    let col = mix(
        vec3<f32>(0.55, 0.65, 0.78),
        vec3<f32>(1.0, 0.98, 0.94),
        clamp(ndl * 0.8 + back * 0.5, 0.0, 1.0),
    ) * lit;

    let a = alpha0 * mask;
    if (a < 0.003) {
        discard;
    }
    return vec4<f32>(params.color.rgb * col, a);
}
