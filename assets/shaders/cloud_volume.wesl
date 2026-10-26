// Volumetric clouds: one full-screen ray march through a world-space domain.
//
// Density is a 3D Perlin-Worley shape eroded by a second octave of the same
// volume, masked by a world-space coverage map and a cumulus height profile.
// Lighting is a short sun march (the self-shadowing the built-in Bevy fog pass
// lacks), a dual-lobe Henyey-Greenstein phase, a powder term and a sky-tinted
// ambient. The output is premultiplied in-scatter plus opacity, so it blends
// over the HDR target with (One, OneMinusSrcAlpha) and works with or without
// geometry behind it.

struct CloudParams {
    inv_view_proj: mat4x4<f32>,
    // xyz = camera position, w = time (s)
    camera_pos: vec4<f32>,
    // xyz = direction toward the sun, w = density scale (1/m at full density)
    sun_dir: vec4<f32>,
    // rgb = sun radiance, w = extinction scale
    sun_color: vec4<f32>,
    // xyz = domain min, w = coverage bias (-1..1)
    domain_min: vec4<f32>,
    // xyz = domain max, w = unused
    domain_max: vec4<f32>,
    // rgb = zenith ambient, w = ambient strength
    zenith: vec4<f32>,
    // rgb = horizon ambient, w = unused
    horizon: vec4<f32>,
    // x = march steps, y = frame index, z = wind (m/s), w = detail strength
    params: vec4<f32>,
};

@group(0) @binding(0) var<uniform> u: CloudParams;
@group(1) @binding(0) var depth_tex: texture_depth_multisampled_2d;
@group(1) @binding(1) var shape_tex: texture_3d<f32>;
@group(1) @binding(2) var shape_sampler: sampler;
@group(1) @binding(3) var weather_tex: texture_2d<f32>;
@group(1) @binding(4) var weather_sampler: sampler;

const PI: f32 = 3.14159265;

@vertex
fn vertex(@builtin(vertex_index) index: u32) -> @builtin(position) vec4<f32> {
    // Oversized triangle: one draw, no vertex buffer.
    let uv = vec2<f32>(f32((index << 1u) & 2u), f32(index & 2u));
    return vec4<f32>(uv * 2.0 - 1.0, 0.0, 1.0);
}

/// Interleaved gradient noise: cheap per-pixel jitter for the march start.
fn ign(p: vec2<f32>) -> f32 {
    return fract(52.9829189 * fract(dot(p, vec2<f32>(0.06711056, 0.00583715))));
}

fn intersect_aabb(ro: vec3<f32>, rd: vec3<f32>, bmin: vec3<f32>, bmax: vec3<f32>) -> vec2<f32> {
    let inv = 1.0 / rd;
    let a = (bmin - ro) * inv;
    let b = (bmax - ro) * inv;
    let lo = min(a, b);
    let hi = max(a, b);
    return vec2<f32>(max(max(lo.x, lo.y), lo.z), min(min(hi.x, hi.y), hi.z));
}

/// Cloud density at a world position, in [0, 1].
///
/// The height profile is offset by high-frequency noise so the underside is
/// ragged instead of a cut plane; three octaves of the same Perlin-Worley
/// volume carry mass, clumps and the cauliflower edge erosion.
fn density(p: vec3<f32>) -> f32 {
    let size = u.domain_max.xyz - u.domain_min.xyz;
    let uvw = (p - u.domain_min.xyz) / size;
    if (any(uvw < vec3<f32>(0.0)) || any(uvw > vec3<f32>(1.0))) {
        return 0.0;
    }

    let wind = vec2<f32>(u.params.z, 0.0) * u.camera_pos.w / 8000.0;
    let drift = vec3<f32>(wind.x * 1.6, 0.0, 0.0) + vec3<f32>(0.0, 0.0, wind.y);

    // Detail first: it both erodes the edges and wobbles the cloud base.
    let fine = textureSampleLevel(shape_tex, shape_sampler, uvw * 9.0 + drift * 2.0, 0.0).r;

    // Weather: 8 km cells in world space, advected by the wind, with a hard
    // contrast curve so there are gaps between cloud masses.
    let weather = textureSampleLevel(weather_tex, weather_sampler, p.xz / 8000.0 + wind, 0.0).r;
    let coverage = clamp((weather - 0.40) * 2.6 + u.domain_min.w, 0.0, 1.0);
    if (coverage < 0.02) {
        return 0.0;
    }

    // The cloud base is not a plane: it hangs lower in dense weather cells, so
    // looking up from underneath shows masses at different heights instead of
    // one flat ceiling.
    let h = uvw.y - (fine - 0.5) * 0.07 - (weather - 0.5) * 0.28;
    if (h <= 0.0 || h >= 1.0) {
        return 0.0;
    }

    // Cumulus profile: a soft base and a rounded top, so the deck reads as
    // cloud from above instead of a fog slab.
    let profile = smoothstep(0.0, 0.07, h) * (1.0 - smoothstep(0.55, 1.0, h));

    // Shape: shear the domain with height so the towers lean, then stack three
    // octaves for mass, clumps and the fine cauliflower.
    let shear = vec3<f32>(h * 0.05, 0.0, -h * 0.03);
    let uv = uvw * vec3<f32>(2.4, 3.4, 2.4) + shear + drift;
    let base = textureSampleLevel(shape_tex, shape_sampler, uv, 0.0).r;
    let mid = textureSampleLevel(
        shape_tex, shape_sampler, uv * 2.7 + vec3<f32>(0.31, 0.17, 0.73), 0.0
    ).r;
    var shape = base * 0.60 + mid * 0.28 + fine * 0.12;
    shape -= (1.0 - fine) * 0.22 * (1.0 - shape);

    let mass = shape * profile;
    var d = saturate((mass - (1.0 - coverage)) / max(coverage * 0.70, 0.05));
    // Erode the base and the thin edges, more where the cloud is wispy.
    d *= smoothstep(0.0, 0.05, h + (fine - 0.5) * 0.05);
    d *= 1.0 - 0.35 * (1.0 - fine) * (1.0 - d);
    // Kill the sub-visible wisps: without this, a long grazing ray through the
    // deck integrates them into an opaque wall and the gaps never open up.
    d = max(d - 0.07, 0.0) / 0.93;

    // Fade the domain's XZ rim: the box follows the camera, so without this
    // the far wall reads as a straight cut in the sky.
    let rim = smoothstep(0.0, 0.10, uvw.x) * smoothstep(0.0, 0.10, 1.0 - uvw.x)
        * smoothstep(0.0, 0.10, uvw.z) * smoothstep(0.0, 0.10, 1.0 - uvw.z);
    return d * rim;
}

/// Henyey-Greenstein phase function.
fn hg(cos_theta: f32, g: f32) -> f32 {
    let g2 = g * g;
    return (1.0 - g2) / (4.0 * PI * pow(max(1.0 + g2 - 2.0 * g * cos_theta, 1e-4), 1.5));
}

/// Transmittance from `p` toward the sun: the short march that gives clouds
/// their lit tops and dark bases.
fn sun_transmittance(p: vec3<f32>, jitter: f32) -> f32 {
    var tau = 0.0;
    var q = p + u.sun_dir.xyz * jitter * 90.0;
    let step = 90.0;
    for (var i = 0; i < 8; i += 1) {
        q += u.sun_dir.xyz * step;
        tau += density(q) * step;
    }
    return exp(-tau * u.sun_color.w);
}

@fragment
fn fragment(@builtin(position) frag: vec4<f32>) -> @location(0) vec4<f32> {
    let size = vec2<f32>(textureDimensions(depth_tex));
    let uv = frag.xy / size;
    let ndc = vec2<f32>(uv.x * 2.0 - 1.0, 1.0 - uv.y * 2.0);
    let depth = textureLoad(depth_tex, vec2<i32>(frag.xy), 0);

    // Bevy uses an infinite reverse-Z projection: depth 0 is the far plane at
    // infinity, so unprojecting it divides by w == 0. Build the view ray from
    // two finite depths instead, then clamp the march to the scene depth.
    let ro = u.camera_pos.xyz;
    var near_clip = u.inv_view_proj * vec4<f32>(ndc, 1.0, 1.0);
    var far_clip = u.inv_view_proj * vec4<f32>(ndc, 1e-4, 1.0);
    let rd = normalize(far_clip.xyz / far_clip.w - near_clip.xyz / near_clip.w);

    var scene_t = 1e9;
    if (depth > 1e-6) {
        var clip = u.inv_view_proj * vec4<f32>(ndc, depth, 1.0);
        scene_t = distance(clip.xyz / clip.w, ro);
    }

    let hit = intersect_aabb(ro, rd, u.domain_min.xyz, u.domain_max.xyz);
    let t0 = max(hit.x, 0.0);
    // Stop at the opaque scene so terrain and aircraft occlude the deck.
    let t1 = min(hit.y, scene_t);
    if (t1 <= t0) {
        return vec4<f32>(0.0);
    }

    let steps = i32(max(u.params.x, 1.0));
    let span = t1 - t0;
    // Samples cluster near the cloud entry (where the density first rises) and
    // thin out with distance, so a shallow below-deck ray does not stair-step.
    var u01 = ign(frag.xy + f32(u.params.y) * 5.588238) / f32(steps);
    let dt = span / f32(steps);

    let cos_theta = dot(rd, u.sun_dir.xyz);
    let hg_phase = 0.72 * hg(cos_theta, 0.55) + 0.28 * hg(cos_theta, -0.20);
    // Multiple scattering keeps clouds bright even with the sun behind the
    // camera; the HG lobe adds the forward-scatter rim on top.
    let phase = 0.50 + 3.0 * hg_phase;
    let ambient = mix(u.horizon.rgb, u.zenith.rgb, saturate(rd.y * 0.5 + 0.5));

    var transmittance = 1.0;
    var scatter = vec3<f32>(0.0);

    for (var i = 0; i < steps; i += 1) {
        if (transmittance < 0.02) {
            break;
        }
        let p = ro + rd * (t0 + span * (u01 * u01 * 0.65 + u01 * 0.35));
        let d = density(p);
        if (d > 0.001) {
            // Jitter the light ray too, so the fixed light-step spacing shows
            // up as noise instead of contour bands across the cloud tops.
            let sun_t = sun_transmittance(p, ign(frag.xy + f32(i) * 3.17));
            // Cheap multiple-scattering stand-in: low powers of the sun
            // transmittance fake light that has bounced a few times inside the
            // cloud, which keeps the shadowed side from going black.
            let sun_eff = 0.85 * sun_t + 0.10 * pow(sun_t, 0.45) + 0.05;
            let powder = 1.0 - exp(-d * 10.0);
            // Sky light dies off toward the base: no sun gets there, and only
            // a fraction of the sky dome sees the underside.
            let height = clamp((p.y - u.domain_min.y) / (u.domain_max.y - u.domain_min.y), 0.0, 1.0);
            let sky_occ = mix(0.42, 1.0, smoothstep(0.0, 0.65, height));
            var light = u.sun_color.rgb * phase * sun_eff * powder * 1.8;
            light += ambient * u.zenith.w * sky_occ * (0.50 + 0.50 * sun_eff);
            // Extinction: density times the scale in `sun_color.w` (1/m at
            // full density), integrated over this step.
            let trans = exp(-d * u.sun_color.w * dt);
            scatter += light * (1.0 - trans) * transmittance;
            transmittance *= trans;
        }
        u01 += 1.0 / f32(steps);
    }

    // The view camera tonemaps with `Tonemapping::None`, so keep cloud radiance
    // in the same range as the hand-authored sky instead of clipping to white.
    scatter = vec3<f32>(1.0) - exp(-scatter * 1.15);

    let alpha = 1.0 - transmittance;
    return vec4<f32>(scatter, alpha);
}
