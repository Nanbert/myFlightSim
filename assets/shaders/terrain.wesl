// Procedural mission terrain.
//
// The mesh carries world-space positions and normals; this shader paints them
// from height and slope (beach/vegetation/rock for the archipelago, meadow/rock
// snow for the alpine map), adds tileable detail breakup, lights the surface
// against the same sun the sky and sea use, and fades the far field into the
// horizon colour so the 96 km mesh edge never reads as an edge.

import bevy_pbr::render::forward_io::VertexOutput;

struct TerrainParams {
    // xyz = camera position, w = time (s)
    camera: vec4<f32>,
    // xyz = direction toward the sun, w = rain (0..1)
    sun: vec4<f32>,
    low: vec4<f32>,
    mid: vec4<f32>,
    high: vec4<f32>,
    deep: vec4<f32>,
    fog: vec4<f32>,
    // x = rock/snow line, y = mode (0 islands, 1 alpine), z = haze density
    rules: vec4<f32>,
    // x = cloud base, y = cloud top, z = coverage, w = wind (m/s)
    cloud: vec4<f32>,
};

@group(constants::MATERIAL_BIND_GROUP) @binding(0) var<uniform> u: TerrainParams;
@group(constants::MATERIAL_BIND_GROUP) @binding(1) var detail_tex: texture_2d<f32>;
@group(constants::MATERIAL_BIND_GROUP) @binding(2) var detail_samp: sampler;
@group(constants::MATERIAL_BIND_GROUP) @binding(3) var cloud_noise: texture_3d<f32>;
@group(constants::MATERIAL_BIND_GROUP) @binding(4) var cloud_samp: sampler;

// Project the shaded point up to the cloud base along the sun and evaluate the
// same weather field the dome marches. The mid-layer shape decides whether the
// column actually holds a cloud, so the ground shadow follows the same gaps.
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
// so without this the terrain stays crisp behind the cloud the aircraft is
// flying through. Five taps over the first ~560 m of the view ray, matching the
// dome's fly-through integral. The altitude gate is uniform across the frame,
// so the taps cost nothing while the aircraft is clear of the layer.
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

@fragment
fn fragment(in: VertexOutput) -> @location(0) vec4<f32> {
    let p = in.world_position.xyz;
    let rain = u.sun.w;

    // Two scales of tileable fBm break up the flat vertex colours.
    let d1 = textureSampleLevel(detail_tex, detail_samp, p.xz * 0.00085, 0.0).r;
    let d2 = textureSampleLevel(detail_tex, detail_samp, p.xz * 0.0042, 0.0).r;
    let detail = d1 * 0.62 + d2 * 0.38;

    // Micro relief: the broad detail octave tilts the surface normal, giving
    // slopes shadowed wrinkles the 240 m mesh cannot carry. The offset is large
    // enough (tens of metres) that the result reads as relief, not grain.
    let e = 30.0;
    let h0 = textureSampleLevel(detail_tex, detail_samp, p.xz * 0.00085, 0.0).r;
    let hx = textureSampleLevel(detail_tex, detail_samp, (p.xz + vec2<f32>(e, 0.0)) * 0.00085, 0.0).r;
    let hz = textureSampleLevel(detail_tex, detail_samp, (p.xz + vec2<f32>(0.0, e)) * 0.00085, 0.0).r;
    let grad = vec2<f32>(hx - h0, hz - h0) / e;
    let n = normalize(normalize(in.world_normal) + vec3<f32>(-grad.x, 0.0, -grad.y) * 60.0);
    let slope = 1.0 - clamp(n.y, 0.0, 1.0);

    var col: vec3<f32>;
    if (u.rules.y < 0.5) {
        // Archipelago: seabed -> sand -> jungle, rock on the steep volcanic
        // flanks. The seabed is only ever seen where terrain pokes out of the
        // water, but keeping it dark makes the shoreline read.
        let shore = smoothstep(-6.0, 16.0, p.y);
        let veg = smoothstep(8.0, 90.0, p.y + (detail - 0.5) * 40.0);
        let rock = smoothstep(u.rules.x - 250.0, u.rules.x + 250.0, p.y)
            * smoothstep(0.20, 0.46, slope + (detail - 0.5) * 0.22);
        col = mix(u.deep.rgb, u.low.rgb, shore);
        col = mix(col, u.mid.rgb, veg * (1.0 - rock));
        col = mix(col, u.high.rgb, rock);
    } else {
        // Alpine: meadow low down, rock on any steep face, snow above the snow
        // line except where the slope is too steep to hold it.
        let alt = smoothstep(1100.0, 2100.0, p.y + (detail - 0.5) * 300.0);
        let rock = smoothstep(0.10, 0.36, slope);
        col = mix(u.low.rgb, u.mid.rgb, max(alt, rock));

        let snow_h = smoothstep(
            u.rules.x - 220.0,
            u.rules.x + 220.0,
            p.y + (detail - 0.5) * 260.0,
        );
        let snow = snow_h * (1.0 - smoothstep(0.30, 0.72, slope));
        col = mix(col, u.high.rgb, snow);
    }

    // Grain: keeps large colour fields from looking like flat paint. The
    // dappled octave reads as scrub/forest texture, the fine one as rock grain.
    col *= 0.80 + 0.28 * detail + 0.06 * d2;

    // Cheap directional + sky lighting, matching the sun the clouds use. A
    // strong key against a cool sky keeps the relief legible through the haze.
    let sun = normalize(u.sun.xyz);
    let ndl = max(dot(n, sun), 0.0);
    let sun_col = vec3<f32>(1.0, 0.95, 0.86) * (1.0 - 0.55 * rain);
    let amb = mix(vec3<f32>(0.22, 0.30, 0.44), vec3<f32>(0.34, 0.36, 0.40), rain);
    let shadow = cloud_shadow(p, sun);
    col *= sun_col * (0.22 + 1.05 * ndl * shadow) + amb * 0.34;

    // Night: the same scalar the sky sweeps by wipes the palette toward a
    // navy floor that keeps a whisper of relief under the "moonlight".
    let night = clamp(u.rules.w, 0.0, 1.0);
    col = (col + vec3<f32>(0.006, 0.010, 0.020) * night) * (1.0 - 0.88 * night);

    // Aerial perspective into the sky horizon.
    let dist = length(p - u.camera.xyz);
    let haze = 1.0 - exp(-dist * u.rules.z);
    col = mix(col, u.fog.rgb, clamp(haze, 0.0, 1.0));

    // Fly-through whiteout, applied last so the ground dissolves into the cloud
    // rather than into the horizon fog.
    let veil = cam_veil((p - u.camera.xyz) / max(dist, 1e-4));
    col = mix(col, vec3<f32>(0.90, 0.94, 1.0) * (1.0 - 0.22 * rain), veil * 0.85);

    return vec4<f32>(col, 1.0);
}
