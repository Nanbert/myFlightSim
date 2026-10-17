// Jet exhaust heat haze: a screen-space refraction behind the nozzles.
//
// A fullscreen post-process that rides on Bevy's `FullscreenMaterial` (see
// `src/fx.rs`, mirroring the rain-on-lens pass in `src/lens.rs`). For each
// nozzle the CPU passes its screen position (in aspect-corrected UV) and the
// nozzle's downstream direction; the shader displaces the frame sample within a
// small elongated lobe around that point, wobbling the displacement with a
// scrolling noise field. The result is the shimmering "you can see through the
// hot air" refraction trailing a running engine.

import bevy_core_pipeline::fullscreen_vertex_shader::FullscreenVertexOutput;

struct Haze {
    // Nozzle 0: screen position + downstream direction, both in aspect-corrected
    // UV centred on the screen (x already multiplied by the aspect ratio).
    uv0x: f32,
    uv0y: f32,
    dir0x: f32,
    dir0y: f32,
    // Nozzle 0: strength (UV offset scale), lobe radius, lobe length, unused.
    str0: f32,
    rad0: f32,
    len0: f32,
    pad0: f32,
    // Nozzle 1, same layout.
    uv1x: f32,
    uv1y: f32,
    dir1x: f32,
    dir1y: f32,
    str1: f32,
    rad1: f32,
    len1: f32,
    pad1: f32,
    // x = time (s), y = global intensity 0..1, z = exhaust heat 0..1, w = unused.
    time: f32,
    intensity: f32,
    temp: f32,
    pad3: f32,
};

@group(0) @binding(0) var screenTexture: texture_2d<f32>;
@group(0) @binding(1) var samp: sampler;
@group(0) @binding(2) var<uniform> settings: Haze;

fn hash21(p: vec2<f32>) -> f32 {
    var q = fract(p * vec2<f32>(0.1031, 0.1030));
    q = q + dot(q, q.yx + 33.33);
    return fract(q.x * q.y);
}

fn noise2(p: vec2<f32>) -> f32 {
    let i = floor(p);
    let f = fract(p);
    let u = f * f * (3.0 - 2.0 * f);
    let a = hash21(i);
    let b = hash21(i + vec2<f32>(1.0, 0.0));
    let c = hash21(i + vec2<f32>(0.0, 1.0));
    let d = hash21(i + vec2<f32>(1.0, 1.0));
    return mix(mix(a, b, u.x), mix(c, d, u.x), u.y);
}

// Displacement contributed by one nozzle at aspect-corrected screen point `q`.
// The lobe is an ellipse: `rad` wide across the exhaust and `len` long
// downstream. Both come from projecting the actual plume (see `manage_haze`),
// so the haze tracks the flame's on-screen size instead of covering a fixed
// rectangle of the frame.
//
// Nothing upstream of the nozzle may shimmer: that region is the airframe. The
// weight is therefore zero for `along < 0`, ramps in over the first few percent
// of the plume (a smooth seam at the rim) and fades out at the plume's tail.
fn haze_at(q: vec2<f32>, p: vec2<f32>, d: vec2<f32>, rad: f32, len: f32) -> vec2<f32> {
    let n = vec2<f32>(-d.y, d.x);
    let v = q - p;
    let along = dot(v, d);
    let across = dot(v, n);

    let a = across / max(rad, 1e-5);
    let t = along / max(len, 1e-5);
    let head = smoothstep(0.0, 0.08, t);
    let tail = 1.0 - smoothstep(0.55, 1.0, t);
    let w = exp(-a * a) * head * tail;

    // Scrolling shimmer, aligned to the lobe so it flows downstream. It speeds
    // up with the exhaust heat: a hotter plume churns its slipstream faster.
    let s = noise2(vec2<f32>(across * 60.0, along * 45.0 - settings.time * (20.0 + 26.0 * settings.temp)));
    return n * (w * (0.45 + 0.55 * s));
}

@fragment
fn fragment(in: FullscreenVertexOutput) -> @location(0) vec4<f32> {
    let res = vec2<f32>(textureDimensions(screenTexture));
    let aspect = res.x / res.y;

    let src = textureSampleLevel(screenTexture, samp, in.uv, 0.0).rgb;
    if settings.intensity < 0.001 {
        return vec4<f32>(src, 1.0);
    }

    // Aspect-corrected, centred coordinates matching how the CPU passes them.
    var q = in.uv - vec2<f32>(0.5, 0.5);
    q.x = q.x * aspect;

    let p0 = vec2<f32>(settings.uv0x, settings.uv0y);
    let p1 = vec2<f32>(settings.uv1x, settings.uv1y);
    let d0 = vec2<f32>(settings.dir0x, settings.dir0y);
    let d1 = vec2<f32>(settings.dir1x, settings.dir1y);

    var off = vec2<f32>(0.0);
    off = off + haze_at(q, p0, d0, settings.rad0, settings.len0) * settings.str0;
    off = off + haze_at(q, p1, d1, settings.rad1, settings.len1) * settings.str1;
    off = off * settings.intensity;

    // Back to UV space (undo the x aspect stretch) before resampling.
    let sample_uv = in.uv + vec2<f32>(off.x / aspect, off.y);
    let col = textureSampleLevel(screenTexture, samp, sample_uv, 0.0).rgb;

    return vec4<f32>(col, 1.0);
}
