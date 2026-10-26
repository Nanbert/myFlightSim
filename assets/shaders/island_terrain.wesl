// Reworked archipelago land material.
//
// The flight sim shaded the mesh from height and slope alone. This version
// paints a proper coastline (wet sand -> dry beach -> scrub -> jungle -> rock),
// adds triplanar rock detail, micro-relief normals, animated surf, cavity
// shading and soft cloud shadows, then fades into the same horizon haze the
// sky and sea use.

import bevy_pbr::render::forward_io::VertexOutput;

struct TerrainParams {
    // xyz = camera position, w = time (s)
    camera: vec4<f32>,
    // xyz = direction toward the sun, w = rain (0..1)
    sun: vec4<f32>,
    // dry sand
    low: vec4<f32>,
    // vegetation
    mid: vec4<f32>,
    // rock
    high: vec4<f32>,
    // seabed
    deep: vec4<f32>,
    // haze colour
    fog: vec4<f32>,
    // x = rock line, y = sea level, z = haze density, w = detail strength
    rules: vec4<f32>,
    // x = cloud coverage, y = wind, z = shadow strength, w = surf strength
    cloud: vec4<f32>,
    // The sky's cloud deck: x = base, y = top, z = coverage, w = wind (m/s).
    deck: vec4<f32>,
};

@group(constants::MATERIAL_BIND_GROUP) @binding(0) var<uniform> u: TerrainParams;
@group(constants::MATERIAL_BIND_GROUP) @binding(1) var detail_tex: texture_2d<f32>;
@group(constants::MATERIAL_BIND_GROUP) @binding(2) var detail_samp: sampler;
@group(constants::MATERIAL_BIND_GROUP) @binding(3) var height_tex: texture_2d<f32>;
@group(constants::MATERIAL_BIND_GROUP) @binding(4) var height_samp: sampler;
@group(constants::MATERIAL_BIND_GROUP) @binding(5) var cloud_noise: texture_3d<f32>;
@group(constants::MATERIAL_BIND_GROUP) @binding(6) var cloud_samp: sampler;

const ARENA_HALF: f32 = 48000.0;

/// Whiteout when the camera is inside the layer — the same fly-through
/// integral the sea shader runs, ported so the islands dissolve into the
/// deck exactly when the water around them does. Without it the land stayed
/// crisp behind the cloud the aircraft is flying through. Five taps over the
/// first ~560 m of the view ray; the altitude gate makes it free while the
/// aircraft is clear of the layer.
fn cam_veil(rd: vec3<f32>) -> f32 {
    if (u.camera.y < u.deck.x - 200.0 || u.camera.y > u.deck.y + 200.0) {
        return 0.0;
    }
    var tau = 0.0;
    var walk = 0.0;
    var seg = 10.0;
    for (var i = 0; i < 5; i = i + 1) {
        let p = u.camera.xyz + rd * walk;
        let q = p + vec3<f32>(u.camera.w * u.deck.w, 0.0, u.camera.w * u.deck.w * 0.35);
        let a = textureSampleLevel(cloud_noise, cloud_samp, q / 18000.0, 0.0).r;
        let b = textureSampleLevel(
            cloud_noise,
            cloud_samp,
            q / 5000.0 + vec3<f32>(0.53, 0.19, 0.91),
            0.0,
        ).r;
        let threshold = mix(0.72, 0.30, u.deck.z);
        let cover = smoothstep(threshold - 0.06, threshold + 0.06, a * 0.55 + b * 0.45);

        let h = clamp((p.y - u.deck.x) / (u.deck.y - u.deck.x), 0.0, 1.0);
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

/// Bilinear height sample from the arena height field.
fn terrain_height(xz: vec2<f32>) -> f32 {
    let uv = xz / (2.0 * ARENA_HALF) + 0.5;
    return textureSampleLevel(height_tex, height_samp, uv, 0.0).r;
}

/// Cheap sky occlusion from the height field: nearby ground above the point
/// darkens valley floors and the base of slopes.
fn terrain_ao(p: vec3<f32>) -> f32 {
    var occ = 0.0;
    for (var i = 0; i < 6; i = i + 1) {
        let a = f32(i) * 1.0472 + 0.4;
        let dir = vec2<f32>(cos(a), sin(a));
        let r1 = 70.0;
        let r2 = 220.0;
        let d1 = terrain_height(p.xz + dir * r1) - p.y;
        let d2 = terrain_height(p.xz + dir * r2) - p.y;
        occ += max(d1, 0.0) / r1;
        occ += max(d2, 0.0) / r2;
    }
    let strength = mix(0.16, 0.09, u.rules.y);
    return clamp(1.0 - occ * strength, 0.55, 1.0);
}

/// March the height field toward the sun: 0 in cast shadow, 1 in the open.
/// Steps grow geometrically so the first few metres are resolved while the
/// march still reaches a few kilometres.
fn sun_visibility(p: vec3<f32>, n: vec3<f32>, L: vec3<f32>) -> f32 {
    if (L.y <= 0.02) {
        return 1.0;
    }
    let origin = p + n * 2.0 + L * 10.0;
    var t = 26.0;
    for (var i = 0; i < 14; i = i + 1) {
        let q = origin + L * t;
        if (q.x < -ARENA_HALF || q.x > ARENA_HALF || q.z < -ARENA_HALF || q.z > ARENA_HALF) {
            break;
        }
        let h = terrain_height(q.xz);
        if (q.y < h - 0.5) {
            // Soft edge: a hard binary shadow reads as polygons from the
            // height-field step at grazing light.
            return clamp(1.0 - (h - q.y) / 45.0, 0.0, 1.0);
        }
        t *= 1.55;
    }
    return 1.0;
}

fn detail(uv: vec2<f32>, lod: f32) -> vec4<f32> {
    return textureSampleLevel(detail_tex, detail_samp, uv, lod);
}

/// Automatic-LOD sample for colour layers: the hardware accounts for the
/// grazing-angle footprint, which keeps slopes from smearing.
fn detail_auto(uv: vec2<f32>) -> vec4<f32> {
    return textureSample(detail_tex, detail_samp, uv);
}

/// Mip level for a detail tile of `tile_m` metres seen from `dist` metres.
/// Heading off moire at altitude matters more than preserving every octave:
/// a 25 m tile at 20 km should collapse to its mean, not speckle.
fn tex_lod(dist: f32, tile_m: f32) -> f32 {
    return clamp(log2(max(dist, 1.0) * 0.0016 * 512.0 / tile_m), 0.0, 9.0);
}

/// Triplanar blend of the broad detail octave: top faces get the xz plane,
/// cliffs get the zy/xy planes so the noise does not smear vertically.
fn triplanar(p: vec3<f32>, n: vec3<f32>, scale: f32, lod: f32) -> vec4<f32> {
    let w = pow(abs(n), vec3<f32>(2.0));
    let ws = w / max(w.x + w.y + w.z, 0.0001);
    let x = detail_auto(p.zy * scale + vec2<f32>(3.1, 7.7));
    let y = detail_auto(p.xz * scale);
    let z = detail_auto(p.xy * scale + vec2<f32>(11.3, 1.9));
    return x * ws.x + y * ws.y + z * ws.z;
}

/// Offset the smooth mesh normal with the fine detail height field. Fades out
/// with distance so the 120 m mesh does not glitter at the horizon.
fn perturb(n: vec3<f32>, p: vec3<f32>, dist: f32, lod: f32, strength_scale: f32) -> vec3<f32> {
    let s = 0.012;
    let e = 2.0;
    let h0 = detail(p.xz * s, lod).r;
    let hx = detail((p.xz + vec2<f32>(e, 0.0)) * s, lod).r;
    let hz = detail((p.xz + vec2<f32>(0.0, e)) * s, lod).r;
    let grad = vec2<f32>((hx - h0) / e, (hz - h0) / e);
    let strength = 10.0 * strength_scale * clamp(1.0 - dist / 22000.0, 0.0, 1.0);
    let bump = vec3<f32>(-grad.x, 0.0, -grad.y) * strength;
    return normalize(n + bump * clamp(n.y, 0.12, 1.0));
}

@fragment
fn fragment(in: VertexOutput) -> @location(0) vec4<f32> {
    let p = in.world_position.xyz;
    let t = u.camera.w;
    let dist = length(p - u.camera.xyz);
    let n0 = normalize(in.world_normal);
    let slope = 1.0 - clamp(n0.y, 0.0, 1.0);
    let lod_b = tex_lod(dist, 1100.0);
    let lod_m = tex_lod(dist, 125.0);
    let lod_f = tex_lod(dist, 222.0);
    let lod_u = tex_lod(dist, 25.0);

    // Large, rotated colour field: stops the island reading as one flat green.
    let rot = vec2<f32>(p.x * 0.866 - p.z * 0.5, p.x * 0.5 + p.z * 0.866);
    let macro_n = detail(p.xz * 0.00011, 2.0).r * 0.62
        + detail(rot * 0.00019 + vec2<f32>(0.37, 0.11), 2.0).r * 0.38;

    // Warp the 1.1 km detail tile so its repetition does not read as a grid
    // from altitude (the low-frequency warp is itself tile-free at this scale).
    let warp = detail(p.xz * 0.00013 + vec2<f32>(0.37, 0.71), 2.0).rg - 0.5;
    let wp = vec3<f32>(p.x + warp.x * 2600.0, p.y, p.z + warp.y * 2600.0);
    let broad = triplanar(wp, n0, 0.0009, lod_b);
    let fine = detail_auto(p.xz * 0.0045);
    // High-frequency layers fade to their mean once they are sub-pixel.
    let near = clamp(1.0 - (dist - 2500.0) / 12000.0, 0.0, 1.0);
    let meso = triplanar(p, n0, 0.008, lod_m);
    let micro = triplanar(p, n0, 0.040, lod_u);
    let broad_c = clamp((broad.r - 0.5) * 1.9 + 0.5, 0.0, 1.0);
    let clump = smoothstep(0.32, 0.70, meso.r);

    // Shared between the two palettes for AO and specular below.
    var rock = 0.0;
    var wet = 0.0;
    var snow = 0.0;
    var col: vec3<f32>;

    if (u.rules.y < 0.5) {
    // --- coastline ---------------------------------------------------------
    // The beach is a soft, noise-jittered band instead of a fixed height.
    let jitter = (broad.b - 0.5) * 8.0 + (macro_n - 0.5) * 11.0;
    let beach_top = 5.0 + (macro_n - 0.5) * 7.0;
    let veg = smoothstep(beach_top - 1.0, beach_top + 9.0, p.y + jitter);
    wet = 1.0 - smoothstep(0.2, 3.0, p.y);

    let sand_dry = u.low.rgb * (0.72 + 0.48 * broad_c) * mix(1.0, 0.92 + 0.16 * micro.r, near);
    let sand_wet = u.low.rgb * 0.42 + u.mid.rgb * 0.05;
    let sand = mix(sand_wet, sand_dry, smoothstep(0.0, 2.0, p.y));

    // --- vegetation --------------------------------------------------------
    let grass_dark = u.mid.rgb * 0.32 + vec3<f32>(0.0, 0.006, 0.002);
    let grass_light = u.mid.rgb * 2.10 + vec3<f32>(0.045, 0.080, 0.014);
    let grass_mix = clamp(broad_c * 0.32 + macro_n * 0.38 + clump * 0.30, 0.0, 1.0);
    var grass = mix(grass_dark, grass_light, grass_mix);
    // Broken canopy: deep gaps between crowns, bright lit leaves on top.
    let canopy = smoothstep(0.28, 0.72, meso.r);
    grass *= mix(1.0, 0.50 + 0.95 * canopy, clamp(1.0 - (dist - 3000.0) / 14000.0, 0.0, 1.0)) * mix(1.0, 0.86 + 0.28 * micro.r, near);
    // Sun-bleached scrub on the dry, high shelves and windward ridges.
    grass = mix(grass, vec3<f32>(0.23, 0.215, 0.095), smoothstep(0.55, 0.92, macro_n) * 0.38);

    // --- rock --------------------------------------------------------------
    let rock_slope = smoothstep(0.30, 0.56, slope + (broad.g - 0.5) * 0.26);
    let rock_alt = smoothstep(u.rules.x - 60.0, u.rules.x + 150.0, p.y + (macro_n - 0.5) * 180.0);
    rock = clamp(rock_slope * 0.95 + rock_alt * 0.78, 0.0, 1.0);
    // Ridged veins, wall striation on near-vertical faces, and dark crevices.
    let wall = detail(vec2<f32>(p.y * 0.010, (p.x + p.z) * 0.004), lod_m).g;
    let vein = smoothstep(0.58, 0.92, micro.g);
    var rock_col = mix(u.high.rgb * 0.55, u.high.rgb * 1.55, broad.g * 0.40 + fine.g * 0.60);
    rock_col = mix(rock_col, vec3<f32>(0.27, 0.21, 0.15), broad.b * 0.40);
    rock_col = mix(rock_col, rock_col * 0.55, vein * 0.40);
    rock_col = mix(rock_col, rock_col * 1.20, wall * (1.0 - n0.y) * 0.30);
    rock_col *= mix(1.0, 0.84 + 0.30 * micro.r, near);

    col = mix(sand, grass, veg);
    col = mix(col, rock_col, rock);
    // Ridged outcrops break up the green fields on steep ground.
    let outcrop = smoothstep(0.76, 0.95, broad.g) * smoothstep(0.22, 0.52, slope);
    col = mix(col, rock_col, outcrop * 0.55);

    // --- seabed (seen through the transparent water) -----------------------
    let shelf = smoothstep(-48.0, -3.0, p.y);
    let seabed = mix(u.deep.rgb, u.low.rgb * 0.42 + u.mid.rgb * 0.05, shelf);
    col = mix(seabed, col, smoothstep(-2.0, 1.5, p.y));

    // Wet sand reads darker and a little glossier.
    col *= 1.0 - 0.42 * wet * smoothstep(0.0, 1.2, p.y);

    // --- surf --------------------------------------------------------------
    let f1 = detail(p.xz * 0.013 + vec2<f32>(t * 0.012, -t * 0.007), 1.0).b;
    let f2 = detail(p.xz * 0.031 - vec2<f32>(t * 0.020, t * 0.011), 2.0).a;
    let foam_noise = f1 * 0.62 + f2 * 0.38;
    let wash = 0.7 + 0.5 * sin(t * 1.25 + dot(p.xz, vec2<f32>(0.011, 0.017)));
    let band = 1.0 - smoothstep(0.0, 2.0, abs(p.y - 0.25 - wash * 0.55));
    let foam = band * smoothstep(0.40, 0.72, foam_noise) * u.cloud.w
        * (1.0 - smoothstep(12000.0, 36000.0, dist));
    col = mix(col, vec3<f32>(0.92, 0.95, 0.93), foam * 0.80);
    } else if (u.rules.y < 1.5) {
        // --- glacial valleys ------------------------------------------------
        // Blue ice in the troughs, dark rock on the arêtes, snow on the high
        // shelves. The NscGWl height field already carries the fluting, so the
        // masks only need broad noise to break up the elevation bands.
        let valley = smoothstep(150.0, 750.0, p.y + (macro_n - 0.5) * 500.0);
        let alt = p.y + (macro_n - 0.5) * 600.0;
        snow = smoothstep(u.rules.x - 250.0, u.rules.x + 350.0, alt);
        rock = smoothstep(0.20, 0.48, slope + (broad.g - 0.5) * 0.22);
        snow *= 1.0 - rock * 0.65;

        var ice = mix(u.deep.rgb, u.low.rgb, valley);
        ice *= 0.88 + 0.20 * broad.g;
        var glacier_rock = mix(
            u.mid.rgb * 0.60,
            u.mid.rgb * 1.55,
            broad.g * 0.50 + micro.r * 0.50,
        );
        glacier_rock = mix(glacier_rock, glacier_rock * 0.55, smoothstep(0.55, 0.90, micro.g) * 0.50);
        col = mix(ice, glacier_rock, rock);
        col = mix(col, u.high.rgb * (0.95 + 0.06 * micro.r), snow);
    } else {
        // --- highlands -------------------------------------------------------
        // The Terrain Tubes landform with the tubes left out: grass and dry
        // earth low down, rock on the steeps, snow on the highest shelves.
        let alt = p.y + (macro_n - 0.5) * 180.0;
        rock = smoothstep(0.18, 0.45, slope + (broad.g - 0.5) * 0.28);
        rock = max(rock, smoothstep(u.rules.x - 550.0, u.rules.x - 30.0, alt) * 0.80);
        snow = smoothstep(u.rules.x - 150.0, u.rules.x + 250.0, alt)
            * (1.0 - smoothstep(0.40, 0.75, slope));

        var ground = mix(u.deep.rgb, u.low.rgb, smoothstep(250.0, 650.0, p.y));
        ground = mix(ground, u.mid.rgb, rock);
        ground = mix(ground, vec3<f32>(0.26, 0.20, 0.12), smoothstep(0.55, 0.90, macro_n) * 0.35);
        ground = mix(ground, u.high.rgb, snow);
        ground *= 0.90 + 0.18 * micro.r;
        col = ground;
    }

    // --- lighting ----------------------------------------------------------
    let n = perturb(n0, p, dist, lod_f, mix(1.0, 0.35, clamp(u.rules.y, 0.0, 1.0)));
    let L = normalize(u.sun.xyz);
    let V = normalize(u.camera.xyz - p);
    let ndl = max(dot(n, L), 0.0);
    let sky = mix(
        vec3<f32>(0.11, 0.15, 0.21),
        vec3<f32>(0.33, 0.44, 0.62),
        clamp(n.y * 0.5 + 0.5, 0.0, 1.0),
    );
    let ao = terrain_ao(p) * (1.0 - 0.22 * rock * slope - 0.10 * (1.0 - broad_c));

    // Soft, drifting cloud shadow from the same detail field.
    let cloud_n = detail(p.xz * 0.000018 + vec2<f32>(t * 0.0006, t * 0.00035), 2.5).r;
    let cover = smoothstep(0.50 - u.cloud.x * 0.10, 0.72 - u.cloud.x * 0.08, cloud_n);
    let shadow = 1.0 - u.cloud.z * cover;

    let sun_col = vec3<f32>(1.04, 0.95, 0.82);
    let sun_vis = sun_visibility(p, n0, L);
    // Rain wets everything down and dulls the direct light.
    let direct = (0.06 + 1.15 * ndl) * mix(1.0, shadow, 0.90) * mix(0.12, 1.0, sun_vis)
        * (1.0 - 0.30 * u.sun.w);
    let ambient = sky * ao * 0.40;
    let spec = pow(max(dot(n, normalize(L + V)), 0.0), 42.0)
        * (wet * 0.45 + rock * 0.12 + snow * 0.18) * 0.9;
    col = col * (sun_col * direct + ambient) + sun_col * spec * shadow;
    // Wet sheen: the rain-soaked ground picks up the sky like the wet sand does.
    col = mix(col, col * 0.85 + sky * 0.15, u.sun.w * 0.6 * wet);

    // Night: the same scalar the sky sweeps by wipes the palette toward a
    // navy floor that keeps a whisper of relief under the "moonlight".
    let night = clamp(u.rules.w, 0.0, 1.0);
    col = (col + vec3<f32>(0.006, 0.010, 0.020) * night) * (1.0 - 0.88 * night);

    // --- aerial perspective ------------------------------------------------
    let height_fade = mix(1.0, 0.78, smoothstep(0.0, 2500.0, p.y));
    // Looking down from altitude crosses less of the boundary layer.
    let alt_fade = clamp(1.0 - (u.camera.y - 600.0) / 14000.0, 0.55, 1.0);
    let haze = 1.0 - exp(-dist * u.rules.z * height_fade * alt_fade);
    col = mix(col, u.fog.rgb, clamp(haze, 0.0, 1.0));

    // Fly-through whiteout, applied last so the ground dissolves into the
    // cloud rather than into the horizon fog — the same veil and the same
    // white the sea shader mixes toward, swept to the deck's dark blue-grey
    // at night by the same night factor the palette uses.
    let veil = cam_veil((p - u.camera.xyz) / max(dist, 1e-4));
    let veil_col = mix(
        vec3<f32>(0.90, 0.94, 1.0) * (1.0 - 0.22 * u.sun.w),
        vec3<f32>(0.13, 0.14, 0.16),
        night,
    );
    col = mix(col, veil_col, veil * 0.85);

    // 8-bit dither against banding in the big gradients.
    let dither = fract(sin(dot(p.xz, vec2<f32>(12.9898, 78.233))) * 43758.5453) - 0.5;
    col += dither / 320.0;

    return vec4<f32>(col, 1.0);
}
