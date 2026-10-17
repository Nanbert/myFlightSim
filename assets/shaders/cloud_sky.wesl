// Volumetric sky and clouds.
//
// The dome is only a canvas: from the camera→fragment direction we ray-march a
// world-space cloud slab, sampling a tileable 3D fBm volume. A low-frequency
// coverage field breaks the layer into scattered clouds, and every dense sample
// is re-sampled toward the sun to fake self-shadowing. The background sky is a
// small physical-ish gradient with a sun glow.

import bevy_pbr::render::forward_io::VertexOutput;

struct CloudParams {
    // xyz = camera position, w = time (s)
    camera: vec4<f32>,
    // xyz = direction toward the sun, w = rain (0..1)
    sun: vec4<f32>,
    // x = cloud base, y = cloud top, z = coverage, w = wind (m/s)
    layer: vec4<f32>,
};

@group(constants::MATERIAL_BIND_GROUP) @binding(0) var<uniform> u: CloudParams;
@group(constants::MATERIAL_BIND_GROUP) @binding(1) var noise_tex: texture_3d<f32>;
@group(constants::MATERIAL_BIND_GROUP) @binding(2) var noise_samp: sampler;

// Long enough for the cloud deck to reach the horizon of the 96 km mission
// terrain; the march still breaks early on opacity.
const MAX_DIST: f32 = 30000.0;
const STEPS: i32 = 44;

fn hash12(p: vec2<f32>) -> f32 {
    let h = dot(p, vec2<f32>(127.1, 311.7));
    return fract(sin(h) * 43758.5453123);
}

// One fBm sample at the volume's native frequency, plus a higher-frequency
// offset sample so the 64³ texture does not read as a repeating grid.
fn sample_fbm(p: vec3<f32>) -> f32 {
    let a = textureSampleLevel(noise_tex, noise_samp, p, 0.0).r;
    let b = textureSampleLevel(
        noise_tex,
        noise_samp,
        p * 3.17 + vec3<f32>(0.37, 0.11, 0.71),
        0.0,
    ).r;
    return a * 0.72 + b * 0.28;
}

// Low-frequency weather field. Returns 0 for clear gaps and 1 for solid cloud.
fn coverage_at(p: vec3<f32>) -> f32 {
    let cover = u.layer.z;
    let q = p + vec3<f32>(u.camera.w * u.layer.w, 0.0, u.camera.w * u.layer.w * 0.35);
    let a = textureSampleLevel(noise_tex, noise_samp, q / 18000.0, 0.0).r;
    let b = textureSampleLevel(
        noise_tex,
        noise_samp,
        q / 5000.0 + vec3<f32>(0.53, 0.19, 0.91),
        0.0,
    ).r;
    let weather = a * 0.55 + b * 0.45;
    let threshold = mix(0.62, 0.26, cover);
    return smoothstep(threshold - 0.08, threshold + 0.16, weather);
}

fn height_profile(h: f32) -> f32 {
    let fade_in = smoothstep(0.0, 0.09, h);
    let fade_out = 1.0 - smoothstep(0.55, 0.98, h);
    return fade_in * fade_out;
}

// Density at a world-space point. `cover` is the coverage sampled once at the
// ray's entry into the layer (it varies slowly, so this saves two samples per
// march step).
fn density_at(p: vec3<f32>, cover: f32) -> f32 {
    let base = u.layer.x;
    let top = u.layer.y;
    let h = (p.y - base) / (top - base);
    if (h <= 0.0 || h >= 1.0 || cover <= 0.001) {
        return 0.0;
    }

    let q = p + vec3<f32>(u.camera.w * u.layer.w, 0.0, u.camera.w * u.layer.w * 0.35);
    let shape = sample_fbm(q / 2600.0);
    let detail = sample_fbm(q / 850.0 + vec3<f32>(0.17, 0.31, 0.53));
    let f = shape * 0.78 + detail * 0.22;

    // Puffy tops over a flatter base; coverage gates the whole column.
    let d = (f * 2.2 - h * 0.85 - (1.0 - cover) * 0.45 - 0.30)
        * height_profile(h);
    return clamp(d, 0.0, 1.0);
}

fn sky_color(rd: vec3<f32>, sun: vec3<f32>, rain: f32) -> vec3<f32> {
    // Linear-light approximations of the old sRGB gradient.
    let zenith = vec3<f32>(0.008, 0.072, 0.49);
    let horizon = vec3<f32>(0.55, 0.70, 0.87);
    let ground = vec3<f32>(0.18, 0.20, 0.23);

    let up = clamp(rd.y, -1.0, 1.0);
    var col = mix(horizon, zenith, pow(clamp(up, 0.0, 1.0), 0.42));
    if (up < 0.0) {
        col = mix(horizon, ground, smoothstep(0.0, 0.35, -up));
    }

    // Sun glow and a small disk; both wash out in overcast.
    let s = clamp(dot(rd, sun), 0.0, 1.0);
    let clear = 1.0 - 0.85 * rain;
    col += vec3<f32>(1.0, 0.55, 0.22) * pow(s, 8.0) * 0.22 * clear;
    col += vec3<f32>(1.0, 0.78, 0.45) * pow(s, 256.0) * 1.1 * clear;

    // Overcast: desaturate toward luminance, then darken.
    let l = dot(col, vec3<f32>(0.299, 0.587, 0.114));
    let grey = vec3<f32>(l * 0.96, l, l * 1.04);
    col = mix(col, grey, 0.85 * rain) * (1.0 - 0.38 * rain);
    return col;
}

fn cloud_color(light: f32, forward: f32, rain: f32) -> vec3<f32> {
    let shadow = mix(
        vec3<f32>(0.20, 0.23, 0.29),
        vec3<f32>(0.16, 0.18, 0.21),
        rain,
    );
    let lit = mix(
        vec3<f32>(0.95, 0.97, 1.0),
        vec3<f32>(0.72, 0.74, 0.78),
        rain,
    );
    var c = mix(shadow, lit, light);
    // Silver lining where the sun is behind the cloud.
    c += vec3<f32>(1.0, 0.82, 0.55)
        * pow(forward, 5.0)
        * (1.0 - light)
        * 0.55
        * (1.0 - 0.7 * rain);
    return c;
}

@fragment
fn fragment(in: VertexOutput) -> @location(0) vec4<f32> {
    let ro = u.camera.xyz;
    let rd = normalize(in.world_position.xyz - ro);
    let sun = normalize(u.sun.xyz);
    let rain = u.sun.w;
    let sky = sky_color(rd, sun, rain);
    var col = sky;

    // Slab intersection in world space. If the ray never crosses the layer,
    // there is nothing to march.
    let base = u.layer.x;
    let top = u.layer.y;
    var t0 = 0.0;
    var t1 = MAX_DIST;
    if (abs(rd.y) > 1e-4) {
        let a = (base - ro.y) / rd.y;
        let b = (top - ro.y) / rd.y;
        t0 = max(t0, min(a, b));
        t1 = min(t1, max(a, b));
    } else if (ro.y < base || ro.y > top) {
        t1 = t0;
    }

    if (t1 > t0) {
        let entry = ro + rd * t0;
        let cover = coverage_at(entry);
        if (cover > 0.001) {
            // One stratified sample per pixel; no time term, so it does not
            // shimmer while the camera is still.
            var t = t0 + hash12(in.position.xy) * 60.0;
            var step = 70.0;
            var trans = 1.0;
            var acc = vec3<f32>(0.0);

            for (var i = 0; i < STEPS; i++) {
                if (t > t1 || trans < 0.015) {
                    break;
                }
                let p = ro + rd * t;
                let d = density_at(p, cover);
                if (d > 0.004) {
                    let sun_d = density_at(p + sun * 300.0, cover);
                    let light = exp(-sun_d * 6.0);
                    let forward = pow(clamp(dot(rd, sun), 0.0, 1.0), 4.0);
                    let alpha = 1.0 - exp(-d * step * 0.0045);
                    let cc = cloud_color(light, forward, rain);
                    // Aerial perspective: distant cloud fades into the sky.
                    let haze = 1.0 - exp(-t * 0.000045);
                    let c = mix(cc, sky, haze * 0.45);
                    acc += c * alpha * trans;
                    trans *= 1.0 - alpha;
                }
                t += step;
                step *= 1.045;
            }
            col = sky * trans + acc;
        }
    }

    // Fly-through veil: if the camera itself sits in dense cloud, wash the
    // whole view toward a cool grey instead of letting the layer clip through.
    let cam_cover = coverage_at(ro);
    let cam_d = density_at(ro, cam_cover);
    let veil = 1.0 - exp(-cam_d * 1.6);
    col = mix(col, vec3<f32>(0.55, 0.60, 0.68), veil * 0.75);

    return vec4<f32>(col, 1.0);
}
