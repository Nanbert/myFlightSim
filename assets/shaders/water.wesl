// Mission sea plane.
//
// A flat, opaque plane at sea level. The fragment shader builds a normal from
// a handful of long swells, reflects the same horizon/zenith gradient the sky
// uses, and adds a tight sun glint. Wave amplitude fades with distance so the
// far ocean stays a smooth mirror instead of aliasing into noise.

import bevy_pbr::render::forward_io::VertexOutput;

struct WaterParams {
    // xyz = camera position, w = time (s)
    camera: vec4<f32>,
    // xyz = direction toward the sun, w = rain (0..1)
    sun: vec4<f32>,
    // xyz = deep water colour
    deep: vec4<f32>,
    // xyz = horizon haze colour
    fog: vec4<f32>,
};

@group(constants::MATERIAL_BIND_GROUP) @binding(0) var<uniform> u: WaterParams;

// Slope (dH/dx, dH/dz) of four crossed swells, in metres per metre.
fn wave_slope(p: vec2<f32>, t: f32) -> vec2<f32> {
    let d0 = vec2<f32>(0.80, 0.60);
    let d1 = vec2<f32>(-0.42, 0.91);
    let d2 = vec2<f32>(0.20, -0.98);
    let d3 = vec2<f32>(-0.86, -0.31);

    var s = vec2<f32>(0.0);
    s += d0 * (0.020 * cos(dot(d0, p) * 0.055 - t * 1.10));
    s += d1 * (0.023 * cos(dot(d1, p) * 0.085 - t * 1.55 + 1.7));
    s += d2 * (0.028 * cos(dot(d2, p) * 0.160 - t * 2.10 + 3.1));
    s += d3 * (0.045 * cos(dot(d3, p) * 0.420 - t * 3.30 + 0.4));
    return s;
}

fn overcast(col: vec3<f32>, rain: f32) -> vec3<f32> {
    let l = dot(col, vec3<f32>(0.299, 0.587, 0.114));
    let grey = vec3<f32>(l * 0.96, l, l * 1.04);
    return mix(col, grey, 0.85 * rain) * (1.0 - 0.38 * rain);
}

@fragment
fn fragment(in: VertexOutput) -> @location(0) vec4<f32> {
    let p = in.world_position.xyz;
    let to_cam = u.camera.xyz - p;
    let dist = length(to_cam);
    let v = to_cam / max(dist, 1e-4);
    let rain = u.sun.w;
    let sun = normalize(u.sun.xyz);

    let fade = 1.0 / (1.0 + dist * 0.00006);
    let s = wave_slope(p.xz, u.camera.w) * fade;
    let n = normalize(vec3<f32>(-s.x, 1.0, -s.y));

    // Reflected sky, matching cloud_sky.wesl's gradient.
    let refl = reflect(-v, n);
    let up = clamp(refl.y, 0.0, 1.0);
    let sky = overcast(
        mix(
            vec3<f32>(0.55, 0.70, 0.87),
            vec3<f32>(0.008, 0.072, 0.49),
            pow(up, 0.42),
        ),
        rain,
    );

    // The reflected horizon is pale by nature; tint it seaward so the open
    // ocean reads blue rather than white at shallow view angles.
    let fres = 0.02 + 0.98 * pow(1.0 - max(dot(n, v), 0.0), 5.0);
    let refl_col = sky * vec3<f32>(0.62, 0.76, 0.94);
    var col = mix(u.deep.rgb, refl_col, fres);

    // A hint of light scattered up out of the water body.
    col += vec3<f32>(0.010, 0.050, 0.060) * max(dot(n, sun), 0.0) * 0.35;

    // Sun glint.
    let h = normalize(v + sun);
    let spec = pow(max(dot(n, h), 0.0), 700.0);
    col += vec3<f32>(1.0, 0.84, 0.58) * spec * 3.0 * (1.0 - 0.75 * rain);

    // Aerial perspective, same falloff as the terrain (fog.w carries density).
    let haze = 1.0 - exp(-dist * u.fog.w);
    col = mix(col, u.fog.rgb, clamp(haze, 0.0, 1.0));

    return vec4<f32>(col, 1.0);
}
