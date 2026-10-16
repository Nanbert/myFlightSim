// Afterburner exhaust flame.
//
// One additive cone per engine nozzle (see `src/fx.rs`), drawn with the default
// mesh vertex shader and this fragment shader. The cone's wide base sits on the
// nozzle and its apex trails aft, so the fragment shader uses the *length of the
// cone's polar UV from the centre* as a longitudinal coordinate: 0 at the
// nozzle, 1 at the tail. Colour runs white-hot at the nozzle through orange to a
// faint red tail, and a scrolling turbulence field rips the plume up so it
// flickers instead of reading as a solid cone.
//
// Additive blending means the output rgb is summed onto the frame, and the
// values are deliberately driven past 1.0 near the nozzle so the camera's bloom
// pass blooms the core.

import bevy_pbr::render::forward_io::VertexOutput;

struct FlameParams {
    // rgb = hot core colour, w = unused.
    hot: vec4<f32>,
    // rgb = cool tail colour, w = unused.
    cool: vec4<f32>,
    // x = time (s), y = throttle 0..1, z = intensity gain, w = turbulence.
    anim: vec4<f32>,
};

@group(constants::MATERIAL_BIND_GROUP) @binding(0) var<uniform> params: FlameParams;

// 2D value hash (Dave Hoskins style), and the value noise built on it.
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
    let time = params.anim.x;
    let throttle = params.anim.y;
    let gain = params.anim.z;
    let turb = params.anim.w;

    // Longitudinal position along the cone: 0 at the nozzle (the wide base),
    // 1 at the tail (the apex), so brightness falls off downstream.
    let t = 1.0 - clamp(length(in.uv - vec2<f32>(0.5, 0.5)) * 2.0, 0.0, 1.0);

    // Core brightness: a long bright body that only starts to fall away past the
    // halfway mark, then fades smoothly to nothing at the tail so the plume has
    // no hard cut-off edge.
    var core = pow(1.0 - t, 1.15) * smoothstep(1.0, 0.5, t);

    // Turbulence: scroll a noise field downstream (and with the aircraft's own
    // motion, via world position) and let it eat into the plume.
    let wp = in.world_position.xyz;
    let n = noise2(vec2<f32>(t * 7.0 - time * 16.0, wp.x * 0.35 + wp.z * 0.35));
    core = core * mix(1.0 - 0.5 * turb, 1.0 + 0.3 * turb, n);

    // Global flicker, so the plume never sits perfectly steady.
    let flicker = 1.0 + 0.15 * (noise2(vec2<f32>(time * 11.0, 4.7)) - 0.5) * 2.0;

    // Hot -> cool along the plume. The very front is pushed blue-white, the way
    // an afterburner's shock diamonds read against the orange body.
    var col = mix(params.hot.rgb, params.cool.rgb, smoothstep(0.0, 0.8, t));
    col = mix(col, vec3<f32>(2.0, 2.1, 2.4), (1.0 - t) * (1.0 - t) * 0.75);

    // Throttle gates the whole thing; even at idle a needle of flame remains.
    let drive = 0.1 + 0.9 * throttle;
    let rgb = col * core * flicker * gain * drive;

    // Nothing to add once the plume has faded: skip the blend entirely.
    if (rgb.r + rgb.g + rgb.b < 0.0025) {
        discard;
    }
    return vec4<f32>(rgb, 1.0);
}
