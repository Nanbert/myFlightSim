// Sky dome: gradient, sun glow, stars and overcast/night treatment.
//
// Clouds are separate depth-tested cards (see `cloud_card.wesl`); this program
// deliberately contains no cloud raymarch, no 3D sampler and no temporal state.

import bevy_pbr::render::forward_io::VertexOutput;

struct SkyParams {
    // xyz = camera position, w = time (s)
    camera: vec4<f32>,
    // xyz = direction toward the sun, w = rain (0..1)
    sun: vec4<f32>,
    // x = night factor (0..1), yzw unused
    misc: vec4<f32>,
};

@group(constants::MATERIAL_BIND_GROUP) @binding(0) var<uniform> u: SkyParams;

// The night factor sweeps the gradient to deep navy, kills the sun glow and
// sprinkles a cheap hash star field (overcast knocks it out).
fn sky_color(rd: vec3<f32>, sun: vec3<f32>, rain: f32, night: f32) -> vec3<f32> {
    let zenith = vec3<f32>(0.008, 0.072, 0.49);
    let horizon = vec3<f32>(0.55, 0.70, 0.87);
    let ground = vec3<f32>(0.18, 0.20, 0.23);

    let up = clamp(rd.y, -1.0, 1.0);
    var col = mix(horizon, zenith, pow(clamp(up, 0.0, 1.0), 0.42));
    if (up < 0.0) {
        col = mix(horizon, ground, smoothstep(0.0, 0.35, -up));
    }

    let s = clamp(dot(rd, sun), 0.0, 1.0);
    let clear = (1.0 - 0.85 * rain) * (1.0 - night);
    col += vec3<f32>(1.0, 0.55, 0.22) * pow(s, 8.0) * 0.22 * clear;
    col += vec3<f32>(1.0, 0.78, 0.45) * pow(s, 256.0) * 1.1 * clear;

    let l = dot(col, vec3<f32>(0.299, 0.587, 0.114));
    let grey = vec3<f32>(l * 0.96, l, l * 1.04);
    col = mix(col, grey, 0.85 * rain) * (1.0 - 0.38 * rain);

    col = col * (1.0 - 0.90 * night) + vec3<f32>(0.010, 0.014, 0.032) * night;
    if (night > 0.02 && rain < 0.5) {
        let d2 = normalize(rd) * 130.0;
        let cell = floor(d2);
        let h = fract(sin(dot(cell, vec3<f32>(12.9898, 78.233, 37.719))) * 43758.5453);
        let twinkle = 0.75 + 0.25 * fract(sin(h * 91.17) * 43758.5453);
        col += vec3<f32>(0.90, 0.93, 1.0)
            * smoothstep(0.9965, 1.0, h) * 2.2 * twinkle * night * (1.0 - 0.9 * rain);
    }
    return col;
}

@fragment
fn fragment(in: VertexOutput) -> @location(0) vec4<f32> {
    let rd = normalize(in.world_position.xyz - u.camera.xyz);
    let sun = normalize(u.sun.xyz);
    return vec4<f32>(sky_color(rd, sun, u.sun.w, clamp(u.misc.x, 0.0, 1.0)), 1.0);
}
