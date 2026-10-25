// Transparent sea for the archipelago — the WEB variant.
//
// Identical blending to `island_water.wesl` (the shelf thins to turquoise
// over the painted seabed, surf rides the shoreline), but the wave field is
// five travelling sine groups perturbed by the noise tile instead of the
// Seascape ray-marched height field. That ray march is what flickered on the
// web build, so `island.rs` binds this shader on wasm32 and the dynamic one
// everywhere else. The uniform block and bindings match the dynamic shader
// exactly — one WaterMaterial, two fragment programs.

import bevy_pbr::render::forward_io::VertexOutput;

struct WaterParams {
    // xyz = camera position, w = time (s)
    camera: vec4<f32>,
    // xyz = direction toward the sun, w = intensity
    sun: vec4<f32>,
    deep: vec4<f32>,
    shallow: vec4<f32>,
    // xyz = haze colour, w = density
    fog: vec4<f32>,
    // x = wave-LOD scale, y = swell height gain, z = glitter gain,
    // w = glitter exponent.
    tune: vec4<f32>,
    // x = cloud base, y = cloud top, z = coverage, w = wind (m/s)
    cloud: vec4<f32>,
    // Night factor (0 day .. 1 deep night)
    night: f32,
};

@group(constants::MATERIAL_BIND_GROUP) @binding(0) var<uniform> u: WaterParams;
@group(constants::MATERIAL_BIND_GROUP) @binding(1) var detail_tex: texture_2d<f32>;
@group(constants::MATERIAL_BIND_GROUP) @binding(2) var detail_samp: sampler;
@group(constants::MATERIAL_BIND_GROUP) @binding(3) var height_tex: texture_2d<f32>;
@group(constants::MATERIAL_BIND_GROUP) @binding(4) var height_samp: sampler;
@group(constants::MATERIAL_BIND_GROUP) @binding(5) var cloud_noise: texture_3d<f32>;
@group(constants::MATERIAL_BIND_GROUP) @binding(6) var cloud_samp: sampler;

const ARENA_HALF: f32 = 48000.0;

/// Water depth from the arena height field: 0 at the shoreline, positive
/// offshore and clamped where the terrain is above the surface.
fn water_depth(xz: vec2<f32>) -> f32 {
    let uv = xz / (2.0 * ARENA_HALF) + 0.5;
    let h = textureSampleLevel(height_tex, height_samp, uv, 0.0).r;
    return max(-h, 0.0);
}

/// One directional sine wave: returns (grad_x, height, grad_z) scaled by fade.
fn wave(p: vec2<f32>, dir: vec2<f32>, wavelength: f32, amp: f32, speed: f32, fade: f32) -> vec3<f32> {
    let d = normalize(dir);
    let k = 6.2831853 / wavelength;
    let ph = dot(d, p) * k + u.camera.w * speed;
    let grad = d * (k * amp * cos(ph));
    return vec3<f32>(grad.x, amp * sin(ph), grad.y) * fade;
}

@fragment
fn fragment(in: VertexOutput) -> @location(0) vec4<f32> {
    let p3 = in.world_position.xyz;
    let p = p3.xz;
    let t = u.camera.w;
    let dist = length(p3 - u.camera.xyz);

    // A broad swell plus shorter chop. Each wave fades out once it is smaller
    // than a pixel, which keeps the far sea smooth instead of aliased.
    var g = vec3<f32>(0.0);
    g += wave(p, vec2<f32>(1.0, 0.25), 190.0, 1.10, 0.50, clamp(1.0 - dist / 55000.0, 0.0, 1.0));
    g += wave(p, vec2<f32>(0.75, -0.65), 88.0, 0.62, 0.75, clamp(1.0 - dist / 30000.0, 0.0, 1.0));
    g += wave(p, vec2<f32>(-0.55, 0.84), 37.0, 0.34, 1.10, clamp(1.0 - dist / 12000.0, 0.0, 1.0));
    g += wave(p, vec2<f32>(0.92, 0.38), 14.0, 0.16, 1.60, clamp(1.0 - dist / 4500.0, 0.0, 1.0));
    g += wave(p, vec2<f32>(-0.2, -0.98), 5.5, 0.065, 2.20, clamp(1.0 - dist / 1600.0, 0.0, 1.0));

    // Fine ripples from the shared detail texture.
    let r1 = textureSampleLevel(detail_tex, detail_samp, p * 0.055 + vec2<f32>(t * 0.020, 0.0), 0.0).r;
    let r2 = textureSampleLevel(detail_tex, detail_samp, p * 0.11 - vec2<f32>(0.0, t * 0.026), 1.0).r;
    let rip = (r1 - r2) * 0.16 * clamp(1.0 - dist / 3000.0, 0.0, 1.0);

    // Broad, non-directional swell noise breaks up the parallel sine crests.
    let s1 = textureSampleLevel(detail_tex, detail_samp, p * 0.0022 + vec2<f32>(t * 0.002, 0.0), 1.0).r;
    let s2 = textureSampleLevel(detail_tex, detail_samp, p * 0.0048 - vec2<f32>(0.0, t * 0.003), 1.0).g;
    let swell = (s1 - s2) * 0.11 * clamp(1.0 - dist / 8000.0, 0.0, 1.0);
    var n = normalize(vec3<f32>(-g.x + rip + swell, 1.0, -g.z - rip - swell));

    let L = normalize(u.sun.xyz);
    let V = normalize(u.camera.xyz - p3);
    let ndv = max(dot(n, V), 0.0);
    let fresnel = clamp(0.02 + 0.98 * pow(1.0 - ndv, 5.0), 0.0, 0.62);

    // Sky reflection: horizon haze -> deeper zenith blue.
    let R = reflect(-V, n);
    let up = clamp(R.y, 0.0, 1.0);
    let sky = mix(vec3<f32>(0.16, 0.26, 0.40), vec3<f32>(0.012, 0.055, 0.28), pow(up, 0.45));

    var col = mix(u.deep.rgb, sky, clamp(fresnel * 0.85, 0.0, 1.0));

    // Shelf depth: turquoise and more transparent over the shallows.
    let depth = water_depth(p);
    let shoal = 1.0 - smoothstep(2.0, 34.0, depth);
    col = mix(col, u.shallow.rgb, shoal * 0.88 * (1.0 - fresnel * 0.45));

    // Sun glitter, strongest along the sun track.
    let H = normalize(L + V);
    let nh = max(dot(n, H), 0.0);
    let glitter = pow(nh, 420.0) * 3.0 + pow(nh, 48.0) * 0.12;
    col += vec3<f32>(1.0, 0.86, 0.62) * glitter * (1.0 - 0.55 * u.sun.w) * u.tune.z;

    // Sparse, small whitecaps close to the camera only.
    let cap_n = textureSampleLevel(detail_tex, detail_samp, p * 0.040 + vec2<f32>(t * 0.010, t * 0.006), 1.0).b;
    let cap = smoothstep(0.90, 0.97, cap_n) * 0.07 * clamp(1.0 - dist / 2500.0, 0.0, 1.0);
    col = mix(col, vec3<f32>(0.80, 0.85, 0.86), cap);

    // Breaking surf on the last few metres of shelf.
    let surf_n = textureSampleLevel(detail_tex, detail_samp, p * 0.020 + vec2<f32>(t * 0.015, -t * 0.009), 0.0).b;
    let surf = (1.0 - smoothstep(0.0, 3.2, depth)) * smoothstep(0.32, 0.70, surf_n);
    col = mix(col, vec3<f32>(0.88, 0.93, 0.92), surf * 0.65);

    // Night: dim the whole surface like the night sky does.
    let night = clamp(u.night, 0.0, 1.0);
    col = (col + vec3<f32>(0.004, 0.006, 0.014) * night) * (1.0 - 0.88 * night);

    // Aerial perspective; alpha rises with haze so the far sea meets the sky.
    let haze = 1.0 - exp(-dist * u.fog.w);
    col = mix(col, u.fog.rgb, clamp(haze, 0.0, 1.0));

    var alpha = mix(u.tune.w, 0.97, fresnel);
    alpha = mix(alpha, 0.30, shoal * 0.80);
    alpha = max(alpha, surf * 0.85);
    alpha = mix(alpha, 1.0, clamp(haze, 0.0, 1.0));

    return vec4<f32>(col, alpha);
}
