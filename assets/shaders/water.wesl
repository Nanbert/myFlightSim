// Mission sea plane.
//
// The wave field and light model are ported from "Seascape" by Alexander
// Alekseev aka TDM (CC BY-NC-SA 3.0 — see CREDITS.md). The reference reads as
// real water because of three things: a choppy multi-octave height field (two
// travelling directions per octave, `sea_octave`), a Fresnel-mixed sky
// reflection with a capped reflectance, and a normalised specular lobe whose
// glitter follows the wave normals. This shader keeps all three.
//
// The plane itself stays flat (one world-scale quad), so wave heights only
// shape the normal. Each octave fades once its wavelength drops below a few
// pixels at the fragment's footprint, which keeps the far sea smooth instead of
// shimmering; aerial haze then folds it into the horizon.

import bevy_pbr::render::forward_io::VertexOutput;

struct WaterParams {
    // xyz = camera position, w = time (s)
    camera: vec4<f32>,
    // xyz = direction toward the sun, w = rain (0..1)
    sun: vec4<f32>,
    // xyz = deep water colour
    deep: vec4<f32>,
    // xyz = horizon haze colour, w = haze density
    fog: vec4<f32>,
    // x = wave-LOD scale (1 = baseline; < 1 keeps fine octaves farther out),
    // y = swell height gain, z = sun-glitter gain, w = glitter exponent.
    tune: vec4<f32>,
};

@group(constants::MATERIAL_BIND_GROUP) @binding(0) var<uniform> u: WaterParams;

const PI: f32 = 3.14159265;
const SEA_HEIGHT: f32 = 0.6;
const SEA_CHOPPY: f32 = 4.0;
const SEA_FREQ: f32 = 0.16;
// Phase speed of the longest swell in uv units/s. The reference's 0.8 came
// from screen-space coordinates; read as metres it left 39 m swells crawling
// at ~1 m/s. Deep-water dispersion slows the shorter octaves below this.
const SEA_SPEED: f32 = 7.0;
// Linear-light equivalents of Seascape's sRGB palette.
const SEA_WATER_COLOR: vec3<f32> = vec3<f32>(0.195, 0.253, 0.107);
const OCTAVES: i32 = 5;

// World-space size of one pixel on the sea at the fragment's distance.
const PIXEL: f32 = 0.0011;
// Rays hitting the sea inside this range are solved against the height field;
// beyond it the flat-plane normal is shaded. The two blend over the last
// `MARCH_FADE` metres, where both agree, so the transition leaves no ring.
const MARCH_RANGE: f32 = 2200.0;
const MARCH_FADE: f32 = 600.0;
// Bisection halves the search band each pass; four passes put the hit within a
// few metres even at a grazing 20 m altitude. Each pass costs one height-field
// evaluation, so this stays cheap enough for integrated GPUs.
const MARCH_STEPS: i32 = 4;

// A world-scale-safe hash (Dave Hoskins' hash13). Seascape's sin hash loses
// precision once the world coordinates reach tens of kilometres.
fn hash(p: vec2<f32>) -> f32 {
    var p3 = fract(vec3<f32>(p.x, p.y, p.x) * 0.1031);
    p3 += dot(p3, vec3<f32>(p3.y, p3.z, p3.x) + 33.33);
    return fract((p3.x + p3.y) * p3.z);
}

fn noise(p: vec2<f32>) -> f32 {
    let i = floor(p);
    let f = fract(p);
    let w = f * f * (3.0 - 2.0 * f);
    let a = hash(i);
    let b = hash(i + vec2<f32>(1.0, 0.0));
    let c = hash(i + vec2<f32>(0.0, 1.0));
    let d = hash(i + vec2<f32>(1.0, 1.0));
    return -1.0 + 2.0 * mix(mix(a, b, w.x), mix(c, d, w.x), w.y);
}

// One octave of wind chop: warped sine ridges, sharpened by `choppy`.
fn sea_octave(uv: vec2<f32>, choppy: f32) -> f32 {
    let p = uv + vec2<f32>(noise(uv));
    let wv = 1.0 - abs(sin(p));
    let swv = abs(cos(p));
    let m = mix(wv, swv, wv);
    return pow(1.0 - pow(m.x * m.y, 0.65), choppy);
}

// Wave height at world xz, metres. `foot` is the world-space size of one pixel
// (metres); octaves whose wavelength shrinks below a few pixels fade out so the
// far/grazing sea cannot alias into moire.
fn sea_height(xz: vec2<f32>, t: f32, foot: f32) -> f32 {
    var uv = vec2<f32>(xz.x * 0.75, xz.y);

    var freq = SEA_FREQ;
    var amp = SEA_HEIGHT * max(u.tune.y, 0.0);
    var choppy = SEA_CHOPPY;
    var h = 0.0;

    for (var i: i32 = 0; i < OCTAVES; i = i + 1) {
        let wavelength = 2.0 * PI / freq;
        let lod = smoothstep(0.0, 1.0, wavelength / max(foot, 1e-4));
        // Higher octaves fade first, so there is nothing left to add once one
        // goes subpixel.
        if (lod <= 0.001) {
            break;
        }
        // Deep-water dispersion: phase speed falls with the square root of
        // the wavenumber, so the long swell rolls while the chop ripples.
        let sea_t = 1.0 + t * SEA_SPEED * sqrt(SEA_FREQ / freq);
        let d = sea_octave((uv + vec2<f32>(sea_t)) * freq, choppy)
            + sea_octave((uv - vec2<f32>(sea_t)) * freq, choppy);
        h += d * amp * lod;

        // Rotate ~37 degrees and double in frequency each octave. The height
        // decay is gentler than Seascape's 0.22: a flat plane has no crest
        // silhouettes, so the finer octaves must carry more slope to keep the
        // surface reading as water from altitude.
        uv = vec2<f32>(1.6 * uv.x + 1.2 * uv.y, -1.2 * uv.x + 1.6 * uv.y);
        freq = freq * 1.9;
        amp = amp * 0.32;
        choppy = mix(choppy, 1.0, 0.2);
    }
    return h;
}

struct Surface {
    normal: vec3<f32>,
    height: f32,
};

fn sea_surface(xz: vec2<f32>, t: f32, foot: f32) -> Surface {
    // Span the difference over at least a pixel so the slope itself does not
    // alias at grazing angles.
    let eps = max(0.12, foot * 1.5);
    let h0 = sea_height(xz, t, foot);
    let hx = sea_height(xz + vec2<f32>(eps, 0.0), t, foot);
    let hz = sea_height(xz + vec2<f32>(0.0, eps), t, foot);
    let n = normalize(vec3<f32>(-(hx - h0) / eps, 1.0, -(hz - h0) / eps));
    return Surface(n, h0);
}

// World-space size of one pixel at distance `dist`, with the eye `eye_height`
// above the shaded point. Same formula the flat path uses, evaluated at the
// marched hit so the wave LOD follows the actual distance.
fn pixel_foot(dist: f32, eye_height: f32) -> f32 {
    let across = dist * PIXEL;
    let along = dist * dist * PIXEL / max(eye_height, 1.0);
    return sqrt(across * along) * max(u.tune.x, 0.05);
}

// Intersection of the eye ray with the wave height field, used to give the near
// sea real parallax: shading follows the swell the ray would actually meet, so
// crests shift against troughs with distance instead of only bending a flat
// normal.
//
// The height field is non-negative, so the surface hit lies between the flat
// plane hit and `h_max / fall` metres closer to the eye. Bisection over that
// band is unconditionally stable; a Newton step would diverge at the grazing
// angles that dominate a 20 m pass.
fn march_sea(rd: vec3<f32>, plane_dist: f32, time: f32) -> vec3<f32> {
    let ro = u.camera.xyz;
    let flat = ro + rd * plane_dist;
    let hlen = length(rd.xz);
    // Vertical drop per horizontal metre along the ray.
    let fall = -rd.y / max(hlen, 1e-3);
    if (hlen < 1e-3 || fall < 1e-4) {
        return flat;
    }
    let dir = rd.xz / hlen;
    let s0 = max(ro.y, 0.0) / fall;
    // `sea_octave` spans [0, 1] and each octave sums two of them, so 3.2x the
    // base swell bounds the full five-octave field with margin.
    let hmax = SEA_HEIGHT * max(u.tune.y, 0.0) * 3.2;
    var lo = max(s0 - hmax / fall, 0.0);
    var hi = s0;
    for (var i: i32 = 0; i < MARCH_STEPS; i = i + 1) {
        let s = 0.5 * (lo + hi);
        let t = s / hlen;
        let xz = ro.xz + dir * s;
        let eye = max(-rd.y * t, 1.0);
        let h = sea_height(xz, time, pixel_foot(t, eye));
        if (ro.y - fall * s > h) {
            lo = s;
        } else {
            hi = s;
        }
    }
    return ro + rd * (0.5 * (lo + hi) / hlen);
}

fn diffuse(n: vec3<f32>, l: vec3<f32>, p: f32) -> f32 {
    return pow(dot(n, l) * 0.4 + 0.6, p);
}

fn specular(n: vec3<f32>, l: vec3<f32>, v: vec3<f32>, s: f32) -> f32 {
    let nrm = (s + 8.0) / (PI * 8.0);
    return pow(max(dot(reflect(-v, n), l), 0.0), s) * nrm;
}

/// Reflected sky, matching cloud_sky.wesl's gradient so sea and sky meet.
fn sky_color(rd: vec3<f32>, sun: vec3<f32>, rain: f32) -> vec3<f32> {
    let zenith = vec3<f32>(0.008, 0.072, 0.49);
    let horizon = vec3<f32>(0.55, 0.70, 0.87);
    let up = clamp(rd.y, 0.0, 1.0);
    var col = mix(horizon, zenith, pow(up, 0.42));

    // Soft glow around the sun; the sharp glitter comes from `specular`.
    let s = clamp(dot(rd, sun), 0.0, 1.0);
    col += vec3<f32>(1.0, 0.55, 0.22) * pow(s, 8.0) * 0.22;

    let l = dot(col, vec3<f32>(0.299, 0.587, 0.114));
    let grey = vec3<f32>(l * 0.96, l, l * 1.04);
    return mix(col, grey, 0.85 * rain) * (1.0 - 0.38 * rain);
}

@fragment
fn fragment(in: VertexOutput) -> @location(0) vec4<f32> {
    let p = in.world_position.xyz;
    let ro = u.camera.xyz;
    let sun = normalize(u.sun.xyz);
    let rain = u.sun.w;
    let t = u.camera.w;

    // Near field: shade the ray/height-field intersection instead of the flat
    // plane. The blend back to the plane point over the last `MARCH_FADE` keeps
    // the shading continuous at the range boundary.
    let plane_dist = length(p - ro);
    let rd = (p - ro) / max(plane_dist, 1e-4);
    var hit = p;
    var dist = plane_dist;
    if (plane_dist < MARCH_RANGE && u.camera.y > 1.0) {
        let fade = smoothstep(MARCH_RANGE - MARCH_FADE, MARCH_RANGE, plane_dist);
        hit = mix(march_sea(rd, plane_dist, t), p, fade);
        dist = length(hit - ro);
    }
    let v = (ro - hit) / max(dist, 1e-4);

    // World-space size of one pixel, for the wave-LOD. Across-view it grows
    // with distance; a grazing ray stretches it along-view by another
    // `dist / eye-height`. The geometric mean keeps the swell readable at
    // altitude without letting finer octaves alias. (Screen derivatives would
    // jump on the plane's triangle edge and draw a seam.)
    let foot = pixel_foot(dist, max(ro.y - hit.y, 1.0));
    let surface = sea_surface(hit.xz, t, foot);
    let n = surface.normal;

    // Seascape's colour model. Reflectance caps at 0.5, so the deep body of
    // the water stays visible at grazing angles instead of turning to mirror.
    let fresnel = min(pow(clamp(1.0 - dot(n, v), 0.0, 1.0), 3.0), 0.5);
    let reflected = sky_color(reflect(-v, n), sun, rain);
    let refracted = u.deep.rgb + diffuse(n, sun, 80.0) * SEA_WATER_COLOR * 0.12;
    var col = mix(refracted, reflected, fresnel);

    // Crests catch a little upwelling light; troughs darken.
    let atten = exp(-dist * 0.00005);
    col += SEA_WATER_COLOR * (surface.height - SEA_HEIGHT * u.tune.y) * 0.18 * atten;

    let sun_col = vec3<f32>(1.0, 0.95, 0.86) * (1.0 - 0.55 * rain);
    let glitter = specular(n, sun, v, max(u.tune.w, 1.0)) * max(u.tune.z, 0.0);
    col += sun_col * glitter * (1.0 - 0.75 * rain);

    // Aerial perspective, same falloff as the terrain (fog.w carries density).
    let haze = 1.0 - exp(-dist * u.fog.w);
    col = mix(col, u.fog.rgb, clamp(haze, 0.0, 1.0));

    return vec4<f32>(col, 1.0);
}
