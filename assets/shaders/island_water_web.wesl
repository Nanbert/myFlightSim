// Web-safe sea for the archipelago.
//
// WebGL2 only: one R16Float height texture (filterable in core WebGL2), one
// 2D sampler, no 3D texture, no ray march, no implicit derivatives and no
// foam. Waves are three analytic sine swells; the plane stays flat, so the
// shoreline depth/alpha path is unchanged.

import bevy_pbr::render::forward_io::VertexOutput;

struct WaterParams {
    // xyz = camera position, w = time (s)
    camera: vec4<f32>,
    // xyz = direction toward the sun, w = rain (0..1)
    sun: vec4<f32>,
    deep: vec4<f32>,
    shallow: vec4<f32>,
    // xyz = haze colour, w = density
    fog: vec4<f32>,
    // x = swell gain, y = speed gain, z = glitter gain, w = unused
    tune: vec4<f32>,
    // x = cloud base, y = cloud top, z = coverage, w = wind (m/s)
    cloud: vec4<f32>,
    // Night factor (0 day .. 1 deep night)
    night: f32,
};

@group(constants::MATERIAL_BIND_GROUP) @binding(0) var<uniform> u: WaterParams;
@group(constants::MATERIAL_BIND_GROUP) @binding(3) var height_tex: texture_2d<f32>;
@group(constants::MATERIAL_BIND_GROUP) @binding(4) var height_samp: sampler;

const ARENA_HALF: f32 = 48000.0;

/// Water depth from the baked height field: 0 at the shoreline, positive
/// offshore, clamped where the terrain stands above the surface.
fn water_depth(xz: vec2<f32>) -> f32 {
    let uv = xz / (2.0 * ARENA_HALF) + vec2<f32>(0.5);
    let h = textureSampleLevel(height_tex, height_samp, uv, 0.0).r;
    return max(-h, 0.0);
}

/// One directional sine swell: returns (grad_x, height, grad_z) scaled by fade.
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
    let dist = length(p3 - u.camera.xyz);

    // Long swells only: short octaves turn into sub-pixel speckle from
    // altitude, which reads as noise rather than water.
    let gain = u.tune.x;
    let speed = u.tune.y;
    var g = vec3<f32>(0.0);
    g += wave(p, vec2<f32>(1.0, 0.18), 900.0, 10.0 * gain, 0.20 * speed, clamp(1.0 - dist / 80000.0, 0.0, 1.0));
    g += wave(p, vec2<f32>(0.62, -0.78), 420.0, 4.0 * gain, 0.32 * speed, clamp(1.0 - dist / 55000.0, 0.0, 1.0));
    g += wave(p, vec2<f32>(-0.52, 0.85), 180.0, 1.2 * gain, 0.50 * speed, clamp(1.0 - dist / 26000.0, 0.0, 1.0));

    let n = normalize(vec3<f32>(-g.x, 1.0, -g.z));

    let L = normalize(u.sun.xyz);
    let V = normalize(u.camera.xyz - p3);
    let ndv = max(dot(n, V), 0.0);
    let fresnel = clamp(0.02 + 0.98 * pow(1.0 - ndv, 5.0), 0.0, 0.62);

    // Sky reflection: horizon haze -> deeper zenith blue.
    let R = reflect(-V, n);
    let up = clamp(R.y, 0.0, 1.0);
    let sky = mix(
        vec3<f32>(0.16, 0.26, 0.40),
        vec3<f32>(0.012, 0.055, 0.28),
        pow(up, 0.45),
    );

    var col = mix(u.deep.rgb, sky, clamp(fresnel * 0.85, 0.0, 1.0));

    // Shelf depth: turquoise and more transparent over the shallows.
    let depth = water_depth(p);
    let shoal = 1.0 - smoothstep(2.0, 34.0, depth);
    col = mix(col, u.shallow.rgb, shoal * 0.88 * (1.0 - fresnel * 0.45));

    // A whisper of crest/trough contrast makes the long swells legible from
    // altitude without adding foam or sparkle.
    let crest = clamp(g.y / 14.0, -1.0, 1.0);
    col += vec3<f32>(0.014, 0.020, 0.022) * crest;

    // Broad, low-intensity sun lobe. No texture, no high-frequency sparkle.
    let H = normalize(L + V);
    let nh = max(dot(n, H), 0.0);
    col += vec3<f32>(1.0, 0.86, 0.62)
        * pow(nh, 64.0)
        * 0.08
        * (1.0 - 0.5 * u.sun.w)
        * u.tune.z;

    // Night: dim the whole surface like the night sky does.
    let night = clamp(u.night, 0.0, 1.0);
    col = (col + vec3<f32>(0.004, 0.006, 0.014) * night) * (1.0 - 0.88 * night);

    // Aerial perspective; alpha rises with haze so the far sea meets the sky.
    let haze = 1.0 - exp(-dist * u.fog.w);
    col = mix(col, u.fog.rgb, clamp(haze, 0.0, 1.0));

    var alpha = mix(0.92, 0.30, shoal * 0.85);
    alpha = mix(alpha, 1.0, clamp(haze, 0.0, 1.0));

    return vec4<f32>(col, alpha);
}
