// Volumetric sky and clouds.
//
// The dome is only a canvas: from the camera→fragment direction we ray-march a
// world-space cloud slab, sampling a tileable 3D fBm volume. A low-frequency
// coverage field breaks the layer into scattered clouds. Dense samples march
// toward the sun to build optical depth, then shade with a multiple-scattering
// approximation — without that integral every cloud reads as a flat fog bank.
//
// Everything here is fragment-only: no compute, no storage buffers, no 3D
// texture larger than 64³. The same shader runs on WebGL2 and WebGPU.

import bevy_pbr::render::forward_io::VertexOutput;

struct CloudParams {
    // xyz = camera position, w = time (s)
    camera: vec4<f32>,
    // xyz = direction toward the sun, w = rain (0..1)
    sun: vec4<f32>,
    // x = cloud base, y = cloud top, z = coverage, w = wind (m/s)
    layer: vec4<f32>,
    // x = mode (0 raymarch, 1 composite), y = temporal weight of this frame,
    // z = frame index (dither rotation), w = history valid
    misc: vec4<f32>,
    // xy = full-resolution target size in physical pixels,
    //     z = night factor (0 day .. 1 deep night), w unused
    screen: vec4<f32>,
    // Previous frame's clip-from-world, for reprojecting the cloud target.
    prev_clip_from_world: mat4x4<f32>,
};

@group(constants::MATERIAL_BIND_GROUP) @binding(0) var<uniform> u: CloudParams;
@group(constants::MATERIAL_BIND_GROUP) @binding(1) var noise_tex: texture_3d<f32>;
@group(constants::MATERIAL_BIND_GROUP) @binding(2) var noise_samp: sampler;
// Half-resolution cloud buffer: rgb = premultiplied in-scattered radiance,
// a = transmittance. The march material reads the previous frame from it; the
// composite material reads the frame just rendered.
@group(constants::MATERIAL_BIND_GROUP) @binding(3) var cloud_tex: texture_2d<f32>;
@group(constants::MATERIAL_BIND_GROUP) @binding(4) var cloud_samp: sampler;

// Long enough for the cloud deck to reach the horizon of the 96 km mission
// terrain; the march still breaks early on opacity.
const MAX_DIST: f32 = 30000.0;
// Forty view samples is the web budget. The march runs at half resolution and
// is accumulated over frames, which buys the extra sun taps below. The light
// march only runs on dense samples, so most rays pay for the view march alone.
const STEPS: i32 = 40;
const SUN_STEPS: i32 = 9;
const PI: f32 = 3.14159265;
// Extinction per metre at density 1. View opacity and sun optical depth share
// this constant so a cloud that looks opaque also shadows itself.
const EXTINCTION: f32 = 0.0045;

// Interleaved gradient noise (Jimenez). A screen-stable dither with an even
// local distribution: white noise flickers between neighbouring pixels, which
// reads as boiling while the aircraft manoeuvres.
fn dither(p: vec2<f32>) -> f32 {
    return fract(52.9829189 * fract(dot(p, vec2<f32>(0.06711056, 0.00583715))));
}

// One fBm sample at the volume's native frequency, plus a higher-frequency
// offset sample so the 64³ texture does not read as a repeating grid.
// `lod` is chosen from the world-space sample spacing; the 3.17x tap needs
// log2(3.17) more.
fn sample_fbm(p: vec3<f32>, lod: f32) -> f32 {
    let a = textureSampleLevel(noise_tex, noise_samp, p, lod).r;
    let b = textureSampleLevel(
        noise_tex,
        noise_samp,
        p * 3.17 + vec3<f32>(0.37, 0.11, 0.71),
        lod + 1.664,
    ).r;
    return a * 0.72 + b * 0.28;
}

// Mip level whose texels match a `foot`-metre sample spacing at `scale` metres
// per volume repeat.
fn lod_for(foot: f32, scale: f32) -> f32 {
    return log2(max(foot * 64.0 / scale, 1.0));
}

// Value-noise fBm grows soft, smoke-like blobs. Folding it around 0.5 turns
// the ridges into rounded lobes, which is what makes cumulus read as
// cauliflower instead of fog.
fn billow(v: f32) -> f32 {
    return 1.0 - abs(v * 2.0 - 1.0);
}

// Horizontal features are kilometres wide; vertical ones are only a few
// hundred metres. An isotropic sample has no variation across a 1.9 km slab,
// which is what makes the deck read as a flat fog sheet.
fn shape_pos(q: vec3<f32>, scale: f32) -> vec3<f32> {
    return vec3<f32>(q.x / scale, q.y / (scale * 0.30), q.z / scale);
}

// Low-frequency weather field, raw 0..1. The raw value also drives each
// column's top height, so towers grow where the weather is dense instead of
// every cloud sharing one ceiling.
fn weather_at(p: vec3<f32>, foot: f32) -> f32 {
    let q = p + vec3<f32>(u.camera.w * u.layer.w, 0.0, u.camera.w * u.layer.w * 0.35);
    let a = textureSampleLevel(noise_tex, noise_samp, q / 18000.0, lod_for(foot, 18000.0)).r;
    let b = textureSampleLevel(
        noise_tex,
        noise_samp,
        q / 5000.0 + vec3<f32>(0.53, 0.19, 0.91),
        lod_for(foot, 5000.0),
    ).r;
    return a * 0.55 + b * 0.45;
}

// Flat base, bulbous top. The old profile tapered the upper half away, which
// rounded every cloud into the same smooth lozenge.
fn height_profile(h: f32) -> f32 {
    let fade_in = smoothstep(0.0, 0.05, h);
    let fade_out = 1.0 - smoothstep(0.85, 1.0, h);
    return fade_in * fade_out;
}

// Density at a world-space point. `weather` is the raw field sampled at that
// point; the gate and the column top are both derived from it.
fn density_at(p: vec3<f32>, weather: f32, foot: f32) -> f32 {
    let base = u.layer.x;
    let top = u.layer.y;
    let h = (p.y - base) / (top - base);
    if (h <= 0.0 || h >= 1.0) {
        return 0.0;
    }
    // The gate is deliberately narrow: a wide smoothstep leaves most of the
    // slab at half density, and at this extinction even 0.25 density is opaque
    // over 1.9 km, so every cloud merges into one sheet.
    let weather_threshold = mix(0.72, 0.30, u.layer.z);
    let cover = smoothstep(
        weather_threshold - 0.06,
        weather_threshold + 0.06,
        weather,
    );
    if (cover <= 0.001) {
        return 0.0;
    }

    let q = p + vec3<f32>(u.camera.w * u.layer.w, 0.0, u.camera.w * u.layer.w * 0.35);
    let raw = sample_fbm(shape_pos(q, 2600.0), lod_for(foot, 2600.0));
    let shape = mix(raw, billow(raw), 0.5);
    let detail = sample_fbm(
        shape_pos(q, 850.0) + vec3<f32>(0.17, 0.31, 0.53),
        lod_for(foot, 850.0),
    );

    let threshold = mix(0.56, 0.34, cover);
    var d = (shape - threshold - h * 0.10) * 4.5;
    // Nibble the isosurface so the silhouette breaks into lobes. The detail
    // only acts near d = 0; interiors stay solid.
    let edge = clamp(1.0 - abs(d), 0.0, 1.0);
    d -= (detail - 0.5) * 0.8 * edge;
    // Each column tops out at its own height: dense weather builds towers,
    // sparse weather only a shallow puff. Without this every cloud shares the
    // same flat ceiling and can never shadow its neighbours.
    let local_top = mix(0.30, 1.0, weather);
    // A hard ceiling shared by a whole column reads as a flat sheet from
    // above. Let the fine octave raise and lower the cap so the plateau
    // breaks into lobes.
    let cap = 1.0 - smoothstep(
        local_top - 0.10,
        local_top,
        h + (detail - 0.5) * 0.05,
    );
    // Scale by the gate itself: a soft edge should be a wisp, not a dim bank.
    return clamp(d, 0.0, 1.0) * cap * height_profile(h) * cover;
}

// Shape-only density for the light march: skipping the detail octave halves
// the cost per shadow tap and the difference is invisible at this frequency.
fn shadow_density_at(p: vec3<f32>, weather: f32, foot: f32) -> f32 {
    let base = u.layer.x;
    let top = u.layer.y;
    let h = (p.y - base) / (top - base);
    if (h <= 0.0 || h >= 1.0) {
        return 0.0;
    }
    let weather_threshold = mix(0.72, 0.30, u.layer.z);
    let cover = smoothstep(
        weather_threshold - 0.06,
        weather_threshold + 0.06,
        weather,
    );
    if (cover <= 0.001) {
        return 0.0;
    }

    let q = p + vec3<f32>(u.camera.w * u.layer.w, 0.0, u.camera.w * u.layer.w * 0.35);
    let raw = textureSampleLevel(
        noise_tex,
        noise_samp,
        shape_pos(q, 2600.0),
        lod_for(foot, 2600.0),
    ).r;
    let shape = mix(raw, billow(raw), 0.5);
    let threshold = mix(0.56, 0.34, cover);
    let d = (shape - threshold - h * 0.10) * 4.5;
    let local_top = mix(0.30, 1.0, weather);
    let cap = 1.0 - smoothstep(local_top - 0.12, local_top, h);
    return clamp(d, 0.0, 1.0) * cap * height_profile(h) * cover;
}

// Optical depth from `p` toward the sun, normalised to [0, 1] transmittance.
// The taps start close to the sample so the thin skin of a cloud top resolves
// into a gradient instead of one flat shadow, then grow quickly to cross the
// slab. Dense samples early-out: once light is extinguished, more taps cannot
// change the result, so the expensive rays are the thin ones that need them.
fn sun_transmittance(p: vec3<f32>, weather: f32, foot: f32, sun: vec3<f32>) -> f32 {
    var tau = 0.0;
    var dist = 0.0;
    var seg = 80.0;
    for (var i = 0; i < SUN_STEPS; i = i + 1) {
        dist += seg;
        tau += shadow_density_at(p + sun * dist, weather, foot) * seg * EXTINCTION;
        if (tau > 4.6) {
            break;
        }
        seg *= 1.85;
    }
    return exp(-tau);
}

// Three-octave approximation to multiple scattering. Single scattering alone
// makes cloud interiors collapse to the exponential; photons that bounce
// survive deeper, so the shadowed side stays bright and bluish.
fn multi_scatter(trans: f32) -> f32 {
    // The thick limit is not zero: with an albedo near 0.9, diffuse photons
    // keep a cloud interior at a sizeable fraction of the lit surface. Without
    // this floor every back-lit bank collapses to black.
    return 0.12
        + 0.50 * trans
        + 0.28 * pow(trans, 0.5)
        + 0.10 * pow(trans, 0.25);
}

fn henyey_greenstein(c: f32, g: f32) -> f32 {
    let g2 = g * g;
    return (1.0 - g2) / (4.0 * PI * pow(max(1.0 + g2 - 2.0 * g * c, 1e-4), 1.5));
}

// Normalised so an isotropic distribution has mean 1. The forward lobe gives
// the silver lining; the weak back lobe keeps the anti-solar side from going
// flat. Photons that have scattered many times forget their original
// direction, so a thick interior tends to isotropic — without this everything
// viewed away from the sun collapses to a grey silhouette.
fn phase(c: f32, trans: f32) -> f32 {
    let forward = (0.7 * henyey_greenstein(c, 0.5) + 0.3 * henyey_greenstein(c, -0.2))
        * 4.0 * PI;
    let thick = clamp(1.0 - trans, 0.0, 1.0);
    return mix(forward, 1.0, thick * 0.8);
}

// Radiance of the medium when the camera is inside it. The direct sun is
// extinguished within metres, but multiple scattering keeps a dense interior
// bright. Deriving this from the local sun transmittance alone collapses the
// veil to the black of single scattering, which reads as a dark grey wall.
fn in_cloud_color(h: f32, rain: f32, toward_sun: f32, day: f32) -> vec3<f32> {
    let trans = 0.35;
    let sun_col = vec3<f32>(1.0, 0.93, 0.80) * 3.2 * (1.0 - 0.45 * rain);
    let amb = mix(
        vec3<f32>(0.12, 0.16, 0.24),
        vec3<f32>(0.32, 0.44, 0.62),
        smoothstep(0.0, 0.85, h),
    ) * (0.75 - 0.15 * rain);
    var c = sun_col * (multi_scatter(trans) * phase(toward_sun, trans) * 0.55) + amb;
    // Overcast: desaturate and darken, matching the deck above the camera.
    let l = dot(c, vec3<f32>(0.299, 0.587, 0.114));
    c = mix(c, vec3<f32>(l), 0.45 * rain) * (1.0 - 0.18 * rain);
    return c * day;
}

// The scattering term is HDR: a forward-lit edge reaches ~10x white. The game
// composites with `Tonemapping::None`, so anything over 1 clips to paper white
// and the silver lining loses all structure. A Reinhard shoulder on the cloud
// radiance keeps the sun-facing side legible; the sky bypasses it.
fn tonemap_cloud(c: vec3<f32>) -> vec3<f32> {
    // 2x exposure keeps lit faces bright while the shoulder still separates
    // the forward-scatter peak from the body of the cloud.
    let x = c * 2.0;
    return x / (vec3<f32>(1.0) + x);
}

fn cloud_color(trans: f32, ph: f32, h: f32, rain: f32, day: f32) -> vec3<f32> {
    let ms = multi_scatter(trans);
    let sun_col = vec3<f32>(1.0, 0.93, 0.80) * 3.2 * (1.0 - 0.45 * rain);
    // Sky light leaks into the shadowed side: dim and blue at the base, bright
    // at the sunlit crowns.
    let amb_base = vec3<f32>(0.12, 0.16, 0.24);
    let amb_top = vec3<f32>(0.32, 0.44, 0.62);
    let amb = mix(amb_base, amb_top, smoothstep(0.0, 0.85, h)) * (0.55 - 0.20 * rain);
    var c = sun_col * (ms * ph * 0.85) + amb;

    // Overcast: desaturate toward luminance, then darken.
    let l = dot(c, vec3<f32>(0.299, 0.587, 0.114));
    c = mix(c, vec3<f32>(l), 0.55 * rain) * (1.0 - 0.22 * rain);
    return c * day;
}

// The night factor sweeps the gradient to deep navy, kills the sun glow and
// sprinkles a cheap hash star field (overcast knocks it out).
fn sky_color(rd: vec3<f32>, sun: vec3<f32>, rain: f32, night: f32) -> vec3<f32> {
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
    let clear = (1.0 - 0.85 * rain) * (1.0 - night);
    col += vec3<f32>(1.0, 0.55, 0.22) * pow(s, 8.0) * 0.22 * clear;
    col += vec3<f32>(1.0, 0.78, 0.45) * pow(s, 256.0) * 1.1 * clear;

    // Overcast: desaturate toward luminance, then darken.
    let l = dot(col, vec3<f32>(0.299, 0.587, 0.114));
    let grey = vec3<f32>(l * 0.96, l, l * 1.04);
    col = mix(col, grey, 0.85 * rain) * (1.0 - 0.38 * rain);

    // Night: toward navy, then stars on the clear eyeball cells.
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

// Composite the half-resolution cloud buffer over the full-resolution sky.
// The dome still supplies the depth test, so terrain and the cockpit occlude
// clouds exactly as before; only the raymarch moved to the offscreen target.
fn composite(rd: vec3<f32>, uv: vec2<f32>, sun: vec3<f32>, rain: f32, night: f32) -> vec4<f32> {
    let cloud = textureSampleLevel(cloud_tex, cloud_samp, uv, 0.0);
    let trans = clamp(cloud.a, 0.0, 1.0);
    return vec4<f32>(cloud.rgb + sky_color(rd, sun, rain, night) * trans, 1.0);
}

// Reproject the previous frame's cloud radiance. The march is jittered, so
// even a still camera needs the history to average the dither; under motion
// the representative depth is the first hit, which is where the visible
// cloud actually is. History is clamped to a band around this frame's value
// and dropped when the reprojected pixel disagrees, so a disocclusion ghosts
// for one frame instead of smearing.
fn temporally_stabilize(
    acc: vec3<f32>,
    trans: f32,
    hit_t: f32,
    ro: vec3<f32>,
    rd: vec3<f32>,
    uv_px: vec2<f32>,
) -> vec4<f32> {
    if (u.misc.w < 0.5 || hit_t <= 0.0) {
        return vec4<f32>(acc, trans);
    }
    let wp = ro + rd * hit_t;
    let clip = u.prev_clip_from_world * vec4<f32>(wp, 1.0);
    if (clip.w <= 1e-4) {
        return vec4<f32>(acc, trans);
    }
    let ndc = clip.xyz / clip.w;
    let uv = vec2<f32>(ndc.x * 0.5 + 0.5, 0.5 - ndc.y * 0.5);
    if (uv.x < 0.0 || uv.x > 1.0 || uv.y < 0.0 || uv.y > 1.0) {
        return vec4<f32>(acc, trans);
    }

    let hist = textureSampleLevel(cloud_tex, cloud_samp, uv, 0.0);
    let cur_uv = uv_px / vec2<f32>(textureDimensions(cloud_tex, 0));
    let motion = distance(uv, cur_uv);
    // A large transmittance mismatch means the reprojection landed on
    // different cloud (or sky) than last frame.
    var conf = 1.0 - smoothstep(0.12, 0.45, abs(hist.a - trans));
    conf *= 1.0 - smoothstep(0.02, 0.12, motion);
    let w = mix(0.5, u.misc.y, conf);

    let hrgb = clamp(
        hist.rgb,
        acc - vec3<f32>(0.18),
        acc + vec3<f32>(0.18),
    );
    let ha = clamp(hist.a, trans - 0.15, trans + 0.15);
    return vec4<f32>(mix(hrgb, acc, w), mix(ha, trans, w));
}

@fragment
fn fragment(in: VertexOutput) -> @location(0) vec4<f32> {
    let ro = u.camera.xyz;
    let rd = normalize(in.world_position.xyz - ro);
    let sun = normalize(u.sun.xyz);
    let rain = u.sun.w;
    let night = clamp(u.screen.z, 0.0, 1.0);
    // Cloud day factor: night sweeps to near-black, and a low sun (dusk)
    // half-lights the deck exactly like it half-lights the terrain.
    let day = (0.55 + 0.45 * clamp(2.0 * sun.y, 0.0, 1.0)) * mix(1.0, 0.06, night);

    // The main camera's sky dome samples the half-resolution target; the
    // offscreen camera (same pose) runs the march below.
    if (u.misc.x > 0.5) {
        let screen = max(u.screen.xy, vec2<f32>(1.0));
        return composite(rd, in.position.xy / screen, sun, rain, night);
    }

    let sky = sky_color(rd, sun, rain, night);
    let frame = u.misc.z;

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

    var trans = 1.0;
    var acc = vec3<f32>(0.0);
    var hit_t = -1.0;

    if (t1 > t0) {
        // One stratified sample per pixel, rotated every frame so the
        // temporal filter can average the dither. Steps grow fast enough that
        // 40 samples still span the 30 km to the horizon: 80 m near the eye
        // where silhouettes are resolved, ~2 km at the deck's far edge.
        let jitter_seed = vec2<f32>(frame * 5.588238, frame * 3.0);
        var t = t0 + dither(in.position.xy + jitter_seed) * 80.0;
        var step = 80.0;

        for (var i = 0; i < STEPS; i++) {
            if (t > t1 || trans < 0.015) {
                break;
            }
            // Jitter every step inside its own interval. A single start offset
            // still samples the growing steps in phase with the noise lattice,
            // which shows up as horizontal ripples where the deck is seen
            // edge-on. The per-step offset turns that structure into dither.
            let jitter = dither(
                in.position.xy + jitter_seed
                    + vec2<f32>(f32(i) * 5.588238, f32(i) * 3.0),
            );
            let p = ro + rd * (t + jitter * step);
            // Mip level from the visible footprint: one pixel across the view
            // at this distance, plus a share of the march spacing, since a
            // coarse step cannot resolve detail finer than its own stride.
            let foot = max(t * 0.0012, step * 0.15);
            // Coverage varies along the ray, not just at the slab entry: a
            // single entry sample turns every cloud into a vertical curtain.
            let weather = weather_at(p, foot);
            let d = density_at(p, weather, foot);
            if (d > 0.004) {
                if (hit_t < 0.0) {
                    hit_t = t + jitter * step;
                }
                let h = clamp((p.y - base) / (top - base), 0.0, 1.0);
                let sun_trans = sun_transmittance(p, weather, foot, sun);
                // Powder fades the direct term at the thin lit skirts;
                // multiple scattering + ambient carry the rest.
                let powder = 1.0 - exp(-2.0 * d);
                let ph = phase(dot(rd, sun), sun_trans) * mix(1.0, powder, 0.25);
                let alpha = 1.0 - exp(-d * step * EXTINCTION);
                let cc = tonemap_cloud(cloud_color(sun_trans, ph, h, rain, day));
                // Aerial perspective: distant cloud fades into the sky.
                let haze = 1.0 - exp(-t * 0.000045);
                let c = mix(cc, sky, haze * 0.45);
                acc += c * alpha * trans;
                trans *= 1.0 - alpha;
            }
            t += step;
            step *= 1.095;
        }
    }

    // Fly-through whiteout: integrate the medium along the first ~560 m of the
    // view ray rather than tinting by the density at one point. Sampling ahead
    // means looking into a cloud reads denser than looking into a gap, and the
    // veil slides as the aircraft moves instead of switching on at one
    // position. The altitude gate is uniform across the frame, so the extra
    // taps cost nothing while the aircraft is clear of the layer.
    if (ro.y > base - 200.0 && ro.y < top + 200.0) {
        var tau = 0.0;
        var h_sum = 0.0;
        var d_sum = 0.0;
        var walk = 0.0;
        var seg = 10.0;
        for (var i = 0; i < 5; i = i + 1) {
            let p = ro + rd * walk;
            let foot = max(walk * 0.0012, 1.0);
            let w = weather_at(p, foot);
            let d = density_at(p, w, foot);
            tau += d * seg * EXTINCTION;
            let hh = clamp((p.y - base) / (top - base), 0.0, 1.0);
            h_sum += hh * d;
            d_sum += d;
            walk += seg;
            seg *= 2.4;
        }
        let veil = 1.0 - exp(-tau);
        if (veil > 0.002) {
            let veil_h = h_sum / max(d_sum, 1e-4);
            var veil_col = in_cloud_color(veil_h, rain, dot(rd, sun), day);
            // Near-field wisps: a dense interior would otherwise be one flat
            // colour. This octave is small enough to sweep past the aircraft,
            // which is what sells the speed once the camera is inside.
            let wisp = sample_fbm(
                shape_pos(ro + rd * 120.0, 500.0) + vec3<f32>(0.71, 0.23, 0.45),
                0.0,
            );
            veil_col *= mix(0.62, 1.25, smoothstep(0.35, 0.75, wisp));
            // The old shader mixed the sky and cloud radiance together; split
            // into premultiplied radiance + transmittance, the veil attenuates
            // both. The composite adds the full-resolution sky back.
            let v = veil * 0.9;
            acc = acc * (1.0 - v) + veil_col * v;
            trans *= 1.0 - v;
        }
    }

    return temporally_stabilize(acc, trans, hit_t, ro, rd, in.position.xy);
}
