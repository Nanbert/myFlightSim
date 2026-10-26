// Sky dome: gradient, sun glow, stars and overcast/night treatment.
//
// Clouds are separate depth-tested cards (see `cloud_card.wesl`); this program
// deliberately contains no cloud raymarch, no 3D sampler and no temporal state
// beyond the frame time, which only drives star twinkle.

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

// Cheap hash star field for dusk/night. Each grid cell on the dome can hold at
// most one star, placed on a jittered point *inside* the cell — lighting the
// whole cell quad instead read as torn-paper scraps. The point radius is a
// couple of cells-percent, so the dot stays 2-3 px regardless of FOV.
fn star_field(rd: vec3<f32>, night: f32, time: f32, rain: f32) -> f32 {
    let d2 = rd * 130.0;
    let cell = floor(d2);
    let h = fract(sin(dot(cell, vec3<f32>(12.9898, 78.233, 37.719))) * 43758.5453);
    // ~2.5% of cells hold a star; the hash tail grades the magnitude so most
    // are faint and a few stand out.
    if (h < 0.975) {
        return 0.0;
    }
    let m = (h - 0.975) / 0.025;
    // Jitter keeps the field off a lattice, and the 0.2..0.8 band keeps the
    // disc (radius 0.22) clear of the cell border: the ray only evaluates its
    // own cell, so a dot crossing the border would pop at the seam.
    let jitter = vec3<f32>(fract(h * 127.1), fract(h * 311.7), fract(h * 74.7));
    let star = cell + vec3<f32>(0.2) + 0.6 * jitter;
    // Angular distance from the view ray to the star point, in cell units.
    let t = dot(star, rd);
    let disc = 1.0 - smoothstep(0.05, 0.22, length(star - t * rd));
    let twinkle = 0.75 + 0.25 * sin(time * (1.0 + 2.0 * fract(h * 91.17)) + h * 628.0);
    // The field dies toward and past the horizon: stars drifted over the
    // ground half of the dome read as scraps there.
    let up = smoothstep(0.0, 0.08, rd.y);
    return (0.05 + 2.15 * m * m * m) * disc * twinkle * up * night * (1.0 - 0.9 * rain);
}

// The night factor sweeps the gradient to deep navy and kills the sun glow.
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
    return col;
}

@fragment
fn fragment(in: VertexOutput) -> @location(0) vec4<f32> {
    let rd = normalize(in.world_position.xyz - u.camera.xyz);
    let sun = normalize(u.sun.xyz);
    let night = clamp(u.misc.x, 0.0, 1.0);
    var col = sky_color(rd, sun, u.sun.w, night);
    col += vec3<f32>(0.90, 0.93, 1.0) * star_field(rd, night, u.camera.w, u.sun.w);
    return vec4<f32>(col, 1.0);
}
