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
};

@group(constants::MATERIAL_BIND_GROUP) @binding(0) var<uniform> u: TerrainParams;
@group(constants::MATERIAL_BIND_GROUP) @binding(1) var detail_tex: texture_2d<f32>;
@group(constants::MATERIAL_BIND_GROUP) @binding(2) var detail_samp: sampler;

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
    col *= sun_col * (0.22 + 1.05 * ndl) + amb * 0.34;

    // Aerial perspective into the sky horizon.
    let dist = length(p - u.camera.xyz);
    let haze = 1.0 - exp(-dist * u.rules.z);
    col = mix(col, u.fog.rgb, clamp(haze, 0.0, 1.0));

    return vec4<f32>(col, 1.0);
}
