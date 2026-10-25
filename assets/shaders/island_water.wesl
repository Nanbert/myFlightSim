// Transparent sea for the ported archipelago.
//
// The wave field and light model are the flight sim's old mission sea
// (ported from "Seascape" by Alexander Alekseev aka TDM, CC BY-NC-SA 3.0 —
// see CREDITS.md): a choppy multi-octave height field with near-field
// ray/height parallax, a Fresnel-mixed sky reflection and a normalised
// glitter lobe, plus cloud shadows and fly-through whiteout from the sky's
// 3D weather field. On top of that this version samples the baked height
// field for **water depth**: over the shallows the surface thins to a
// translucent turquoise and the painted seabed shows through. That alpha
// transition is what kills the old shoreline flicker — the sand/sea boundary
// is a blend now, not two opaque surfaces fighting for depth. Waves read as
// deep-water chop only where the water is actually deep: past the shelf the
// old alpha-1 look returns untouched.

import bevy_pbr::render::forward_io::VertexOutput;

struct WaterParams {
    // xyz = camera position, w = time (s)
    camera: vec4<f32>,
    // xyz = direction toward the sun, w = rain (0..1)
    sun: vec4<f32>,
    // xyz = deep water colour
    deep: vec4<f32>,
    // xyz = shallow-water tint
    shallow: vec4<f32>,
    // xyz = horizon haze colour, w = haze density
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
@group(constants::MATERIAL_BIND_GROUP) @binding(1) var noise_tex: texture_2d<f32>;
@group(constants::MATERIAL_BIND_GROUP) @binding(2) var noise_samp: sampler;
@group(constants::MATERIAL_BIND_GROUP) @binding(3) var height_tex: texture_2d<f32>;
@group(constants::MATERIAL_BIND_GROUP) @binding(4) var height_samp: sampler;
@group(constants::MATERIAL_BIND_GROUP) @binding(5) var cloud_noise: texture_3d<f32>;
@group(constants::MATERIAL_BIND_GROUP) @binding(6) var cloud_samp: sampler;

const ARENA_HALF: f32 = 48000.0;

/// Water depth from the baked height field: 0 at the shoreline, positive
/// offshore, clamped where the terrain stands above the surface.
fn water_depth(xz: vec2<f32>) -> f32 {
    let uv = xz / (2.0 * ARENA_HALF) + vec2<f32>(0.5);
    let h = textureSampleLevel(height_tex, height_samp, uv, 0.0).r;
    return max(-h, 0.0);
}

// The sun's direct path is occluded where a cloud sits between it and the sea.
// Same projection and weather field as the terrain shadow.
fn cloud_shadow(p: vec3<f32>, sun: vec3<f32>) -> f32 {
    if (sun.y <= 0.03) {
        return 1.0;
    }
    let t = (u.cloud.x - p.y) / sun.y;
    if (t <= 0.0) {
        return 1.0;
    }
    let q = p + sun * t
        + vec3<f32>(u.camera.w * u.cloud.w, 0.0, u.camera.w * u.cloud.w * 0.35);
    let a = textureSampleLevel(cloud_noise, cloud_samp, q / 18000.0, 0.0).r;
    let b = textureSampleLevel(
        cloud_noise,
        cloud_samp,
        q / 5000.0 + vec3<f32>(0.53, 0.19, 0.91),
        0.0,
    ).r;
    let threshold = mix(0.72, 0.30, u.cloud.z);
    let cover = smoothstep(threshold - 0.06, threshold + 0.06, a * 0.55 + b * 0.45);

    let mid = vec3<f32>(q.x, (u.cloud.x + u.cloud.y) * 0.5, q.z);
    let raw = textureSampleLevel(
        cloud_noise,
        cloud_samp,
        vec3<f32>(mid.x / 2600.0, mid.y / 780.0, mid.z / 2600.0),
        0.0,
    ).r;
    let billow = 1.0 - abs(raw * 2.0 - 1.0);
    let body = smoothstep(0.45, 0.70, mix(raw, billow, 0.5));
    return 1.0 - 0.55 * cover * body;
}

// Whiteout when the camera is inside the layer. The dome is drawn at the back,
// so without this the sea stays crisp behind the cloud the aircraft is flying
// through. Five taps over the first ~560 m of the view ray, matching the dome's
// fly-through integral; the altitude gate makes it free when clear of the layer.
fn cam_veil(rd: vec3<f32>) -> f32 {
    if (u.camera.y < u.cloud.x - 200.0 || u.camera.y > u.cloud.y + 200.0) {
        return 0.0;
    }
    var tau = 0.0;
    var walk = 0.0;
    var seg = 10.0;
    for (var i = 0; i < 5; i = i + 1) {
        let p = u.camera.xyz + rd * walk;
        let q = p + vec3<f32>(u.camera.w * u.cloud.w, 0.0, u.camera.w * u.cloud.w * 0.35);
        let a = textureSampleLevel(cloud_noise, cloud_samp, q / 18000.0, 0.0).r;
        let b = textureSampleLevel(
            cloud_noise,
            cloud_samp,
            q / 5000.0 + vec3<f32>(0.53, 0.19, 0.91),
            0.0,
        ).r;
        let threshold = mix(0.72, 0.30, u.cloud.z);
        let cover = smoothstep(threshold - 0.06, threshold + 0.06, a * 0.55 + b * 0.45);

        let h = clamp((p.y - u.cloud.x) / (u.cloud.y - u.cloud.x), 0.0, 1.0);
        let raw = textureSampleLevel(
            cloud_noise,
            cloud_samp,
            vec3<f32>(q.x / 2600.0, q.y / 780.0, q.z / 2600.0),
            0.0,
        ).r;
        let billow = 1.0 - abs(raw * 2.0 - 1.0);
        let body = smoothstep(0.45, 0.70, mix(raw, billow, 0.5));
        let prof = smoothstep(0.0, 0.06, h) * (1.0 - smoothstep(0.85, 1.0, h));

        tau += cover * body * prof * seg * 0.0045;
        walk += seg;
        seg *= 2.4;
    }
    return 1.0 - exp(-tau);
}

const PI: f32 = 3.14159265;
const SEA_HEIGHT: f32 = 0.6;
// Lower than Seascape's 4: soft, cloth-like ridges instead of sharp chop.
const SEA_CHOPPY: f32 = 3.0;
// Base spatial frequency. Lower than Seascape's 0.16, so the longest swell is
// ~1.6x wider; combined with the gentler chop this reads as slow undulation
// rather than a choppy sea.
const SEA_FREQ: f32 = 0.10;
// Phase speed of the longest swell in uv units/s. The reference's 0.8 came
// from screen-space coordinates; read as metres it left 39 m swells crawling
// at ~1 m/s. Deep-water dispersion slows the shorter octaves below this.
// 3.0 keeps even the reference-scale swell visibly rolling, but slowly.
const SEA_SPEED: f32 = 3.0;
// Linear-light equivalents of Seascape's sRGB palette.
const SEA_WATER_COLOR: vec3<f32> = vec3<f32>(0.195, 0.253, 0.107);
const OCTAVES: i32 = 5;

// The wave field is drawn at true scale at this eye height. Above it every
// octave's wavelength, height and travel speed scale by the same factor, so the
// sea keeps the reference look instead of flattening into haze. Physically the
// swell would be invisible from 500 m, but the mission minimum is 500 m and the
// brief is that altitude's sea should read like the reference (a ~10 m camera),
// so this is deliberately non-physical. The cap keeps the far sea from turning
// into continental-scale ridges if the jet climbs.
const WAVE_REF_HEIGHT: f32 = 10.0;
const WAVE_MAX_SCALE: f32 = 60.0;
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

// Stretch applied to the whole wave field, 1 at the reference eye height and
// proportional to altitude above it.
fn wave_scale() -> f32 {
    return clamp(u.camera.y / WAVE_REF_HEIGHT, 1.0, WAVE_MAX_SCALE);
}

// Reference wave height at reference-scale xz, metres. `foot` is the size of one
// pixel in those same units; octaves whose wavelength shrinks below a few pixels
// fade out so the far/grazing sea cannot alias into moire.
fn sea_height_ref(xz: vec2<f32>, t: f32, foot: f32) -> f32 {
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

// World-scale wave height. Scaling both the sample position and the result
// keeps the slope field identical, so the sea's shading is preserved at any
// altitude; leaving time unscaled also stretches the travel speed by the same
// factor, which is what makes the motion look unchanged rather than slow.
fn sea_height(xz: vec2<f32>, t: f32, foot: f32) -> f32 {
    let s = wave_scale();
    return s * sea_height_ref(xz / s, t, foot / s);
}

struct Surface {
    normal: vec3<f32>,
    height: f32,
};

fn sea_surface(xz: vec2<f32>, t: f32, foot: f32) -> Surface {
    // Span the difference over at least a pixel so the slope itself does not
    // alias at grazing angles. The floor is in reference-scale metres, so it
    // grows with the field.
    let eps = max(0.12 * wave_scale(), foot * 1.5);
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
    let hmax = SEA_HEIGHT * max(u.tune.y, 0.0) * 3.2 * wave_scale();
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

    let foot = pixel_foot(dist, max(ro.y - hit.y, 1.0));
    let surface = sea_surface(hit.xz, t, foot);
    let n = surface.normal;

    // Water depth drives the whole shoreline treatment. Waves are a deep-water
    // look: their amplitude folds away as the shelf rises, so the chop dies
    // before it can climb the beach.
    let depth = water_depth(hit.xz);
    let shoal = 1.0 - smoothstep(1.5, 26.0, depth);

    // Seascape's colour model. Reflectance caps at 0.5, so the deep body of
    // the water stays visible at grazing angles instead of turning to mirror.
    let fresnel = min(pow(clamp(1.0 - dot(n, v), 0.0, 1.0), 3.0), 0.5);
    let reflected = sky_color(reflect(-v, n), sun, rain);
    let shadow = cloud_shadow(hit, sun);
    let refracted = mix(u.deep.rgb, u.shallow.rgb, shoal * 0.75)
        + diffuse(n, sun, 80.0) * SEA_WATER_COLOR * 0.12 * shadow;
    var col = mix(refracted, reflected, fresnel);

    // Crests catch a little upwelling light; troughs darken. Height is divided
    // by the wave scale so the contrast stays constant as the field grows.
    let atten = exp(-dist * 0.00005);
    col += SEA_WATER_COLOR * (surface.height / wave_scale() - SEA_HEIGHT * u.tune.y) * 0.18 * atten * (1.0 - shoal);

    let sun_col = vec3<f32>(1.0, 0.95, 0.86) * (1.0 - 0.55 * rain);
    let glitter = specular(n, sun, v, max(u.tune.w, 1.0)) * max(u.tune.z, 0.0);
    col += sun_col * glitter * shadow * (1.0 - 0.75 * rain) * (1.0 - shoal);

    // Night: dim the whole surface like the night sky does.
    let night = clamp(u.night, 0.0, 1.0);
    col = (col + vec3<f32>(0.004, 0.006, 0.014) * night) * (1.0 - 0.88 * night);

    // Aerial perspective, same falloff as the terrain (fog.w carries density).
    let haze = 1.0 - exp(-dist * u.fog.w);
    col = mix(col, u.fog.rgb, clamp(haze, 0.0, 1.0));

    // Fly-through whiteout: the sea disappears into the cloud too.
    let veil = cam_veil((hit - ro) / max(dist, 1e-4));
    col = mix(col, vec3<f32>(0.90, 0.94, 1.0) * (1.0 - 0.22 * rain), veil * 0.85);

    // Breaking surf on the last few metres of shelf, over the same noise tile
    // the terrain material uses.
    let f1 = textureSampleLevel(noise_tex, noise_samp, hit.xz * 0.013 + vec2<f32>(t * 0.012, -t * 0.007), 1.0).b;
    let f2 = textureSampleLevel(noise_tex, noise_samp, hit.xz * 0.031 - vec2<f32>(t * 0.020, t * 0.011), 2.0).a;
    let foam_noise = f1 * 0.62 + f2 * 0.38;
    let surf = (1.0 - smoothstep(0.0, 3.0, depth)) * smoothstep(0.40, 0.72, foam_noise);
    col = mix(col, vec3<f32>(0.90, 0.94, 0.92), surf * 0.70);

    // Alpha is the shoreline fix: opaque deep water, translucent shelf. The
    // painted seabed (terrain triangles) shows through the thin water, and
    // where the beach rises past the surface the depth test simply wins.
    var alpha = mix(1.0, 0.30, shoal * 0.85);
    alpha = max(alpha, surf * 0.85);
    alpha = mix(alpha, 1.0, clamp(haze, 0.0, 1.0));
    return vec4<f32>(col, alpha);
}
