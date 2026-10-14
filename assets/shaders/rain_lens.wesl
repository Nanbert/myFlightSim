// Rain on the lens: a screen-space droplet post-process.
//
// Port of the droplet field from *The Drive Home* by Martijn Steinrucken
// (BigWings), shadertoy.com/view/MdfBRX. The original is a night-drive scene
// that refracts its own procedural lights through rain on the windshield; we
// keep only the droplet field and its refraction, applied to the live frame
// instead of a raymarched scene, so it rides on Bevy's `FullscreenMaterial`
// (see `src/lens.rs`).
//
// The field is three layers of falling drops-with-trails at increasing density.
// Each cell returns a 2D offset to the water sitting in it; displacing the
// screen sample by that offset makes each drop bend the frame like a little
// lens, and the tiles that hold no water return a zero offset so the rest of
// the glass stays perfectly sharp.
//
// License: *The Drive Home* is published under Shadertoy's default
// CC BY-NC-SA 3.0 (non-commercial, share-alike) — resolve before shipping the
// sim commercially. The Bevy plumbing around it (`src/lens.rs`) is original.

import bevy_core_pipeline::fullscreen_vertex_shader::FullscreenVertexOutput;

struct RainLens {
    intensity: f32,
    time: f32,
    pad0: f32,
    pad1: f32,
};

@group(0) @binding(0) var screenTexture: texture_2d<f32>;
@group(0) @binding(1) var samp: sampler;
@group(0) @binding(2) var<uniform> settings: RainLens;

// --- tuning ---
const REFRACT: f32 = 2.0;     // drop lens strength (screen-UV offset scale)
const RIM_GLINT: f32 = 0.15;  // brightness of the lit (lower) rim of a drop
const RIM_DARK: f32 = 0.05;   // how dark the shadowed (upper) rim goes
const RIM_BODY: f32 = 0.03;   // faint sheen filling the body of a drop
const VIGNETTE: f32 = 0.15;   // corner darkening
const TAU: f32 = 6.2831853;

// smoothstep that also accepts `a > b`: the droplet field relies on reversed
// edges, which WGSL's builtin does not guarantee.
fn S(a: f32, b: f32, x: f32) -> f32 {
    let t = clamp((x - a) / (b - a), 0.0, 1.0);
    return t * t * (3.0 - 2.0 * t);
}

// 3-channel hash (Dave Hoskins).
fn N31(p: f32) -> vec3<f32> {
    var p3 = fract(vec3<f32>(p) * vec3<f32>(0.1031, 0.11369, 0.13787));
    p3 = p3 + vec3<f32>(dot(p3, p3.yzx + 19.19));
    return fract(vec3<f32>(
        (p3.x + p3.y) * p3.z,
        (p3.x + p3.z) * p3.y,
        (p3.y + p3.z) * p3.x,
    ));
}

// A drop's vertical slide over its life, plus its velocity (used to bulge the
// drop where the water piles up as it accelerates).
fn SawTooth(t: f32) -> f32 {
    return cos(t + cos(t)) + sin(2.0 * t) * 0.2 + sin(4.0 * t) * 0.02;
}

fn DeltaSawTooth(t: f32) -> f32 {
    return 0.4 * cos(2.0 * t) + 0.08 * cos(4.0 * t) - (1.0 - sin(t)) * sin(t + cos(t));
}

// What one droplet layer contributes at a pixel: `offset` is where the water
// sits (drives refraction), `mask` is how much water covers the pixel, and
// `nrm` is the direction from the drop's centre out to the pixel (drives the
// light/dark rim shading).
struct Drop {
    offset: vec2<f32>,
    mask: f32,
    nrm: vec2<f32>,
};

// One layer of the droplet field. Every cell of the grid holds (at most) one
// drop that slides downward, dragging a beaded trail. Returns the offset of the
// water within the cell — zero where the glass is clear — plus a mask marking
// where water sits at all (the drop, or its trail).
fn GetDrops(uv_in: vec2<f32>, seed: f32, m: f32, time: f32) -> Drop {
    var t = time + m * 30.0;
    var uv = uv_in;
    uv.y = uv.y + t * 0.05;
    uv = uv * vec2<f32>(13.0, 3.5);

    let id = floor(uv);
    let n = N31(id.x + (id.y + seed) * 546.35242);

    // Local cell coords, stretched vertically so drops read as tall teardrops.
    var bd = fract(uv) - vec2<f32>(0.5, 0.5);
    bd.y = bd.y * 4.0;
    bd.x = bd.x + (n.x - 0.5) * 0.6;

    t = t + n.z * TAU;
    let slide = SawTooth(t);

    // The trail: beads left behind, sliding at a different rate to the drop.
    let ts = 1.5;
    let trailPos = vec2<f32>(
        bd.x * ts,
        (fract(bd.y * ts * 2.0 - t * 2.0) - 0.5) * 0.5,
    );

    // Slide the drop, then add its shape: the square term buckles the bottom
    // outward by its own speed, giving the classic teardrop silhouette.
    bd.y = bd.y + slide * 2.0;
    var dropShape = bd.x * bd.x;
    dropShape = dropShape * DeltaSawTooth(t);
    bd.y = bd.y + dropShape;

    let d = length(bd);
    var trailMask = S(-0.2, 0.2, bd.y);
    trailMask = trailMask * bd.y;

    let td = length(trailPos * max(0.5, trailMask));
    let mainDrop = S(0.2, 0.1, d);

    var dropTrail = S(0.1, 0.02, td);
    dropTrail = dropTrail * trailMask;

    let offset = mix(bd * mainDrop, trailPos, dropTrail);
    let mask = max(mainDrop, dropTrail);
    // The drop sits where bd = 0, so bd is the vector from its centre out to
    // this pixel. Un-stretch y (it was scaled by 4) to get a screen-space dir.
    let nrm = normalize(vec2<f32>(bd.x, bd.y * 0.25) + vec2<f32>(1e-5, 1e-5));
    return Drop(offset, mask, nrm);
}

@fragment
fn fragment(in: FullscreenVertexOutput) -> @location(0) vec4<f32> {
    let res = vec2<f32>(textureDimensions(screenTexture));
    let aspect = res.x / res.y;

    let src = textureSampleLevel(screenTexture, samp, in.uv, 0.0).rgb;
    if settings.intensity < 0.001 {
        return vec4<f32>(src, 1.0);
    }

    let t = settings.time;
    let m = 0.0;

    // Seed space: centred and aspect-corrected.
    var uv = in.uv - vec2<f32>(0.5, 0.5);
    uv.x = uv.x * aspect;

    // The droplet pattern drifts and turns slowly, so it never sits static.
    let ang = -((sin(t * 0.1) * 0.5 + 0.5) * 0.5);
    let a = ang * ang;
    let ca = cos(a);
    let sa = sin(a);
    var dropUv = vec2<f32>(uv.x * ca + uv.y * sa, -uv.x * sa + uv.y * ca);
    dropUv.x = dropUv.x + (-sin(t * 0.1)) * 0.5;

    // More rain → more layers come up, from a thin mist to a downpour.
    let l0 = S(0.0, 0.35, settings.intensity);
    let l1 = S(0.0, 0.6, settings.intensity);
    let l2 = S(0.0, 1.0, settings.intensity);

    let d0 = GetDrops(dropUv, 1.0, m, t);
    let d1 = GetDrops(dropUv * 1.4, 10.0, m, t);
    let d2 = GetDrops(dropUv * 2.4, 25.0, m, t);

    var offs = d0.offset * l0 + d1.offset * l1 + d2.offset * l2;
    let lens = max(d0.mask * l0, max(d1.mask * l1, d2.mask * l2));
    let nrm = d0.nrm * (d0.mask * l0)
        + d1.nrm * (d1.mask * l1)
        + d2.nrm * (d2.mask * l2);

    // Refract: displace the screen sample by the drop offset. `offs` is in
    // aspect-corrected pattern space, so undo the x stretch before sampling.
    let sample_uv = in.uv - vec2<f32>(offs.x / aspect, offs.y) * REFRACT;
    var col = textureSampleLevel(screenTexture, samp, sample_uv, 0.0).rgb;

    // Water catches the light. The mask fills each drop, so `4*m*(1-m)` peaks
    // exactly on the drop's rim: a thin outline that reads as the bead of a
    // water drop even against a flat sky that refracts to nothing. Light from
    // above gathers on the drop's lower rim and the upper rim stays dark, which
    // is what stops it reading as a flat soap bubble.
    let wet = clamp(lens, 0.0, 1.0);
    let ring = clamp(4.0 * wet * (1.0 - wet), 0.0, 1.0);
    let nl = normalize(nrm + vec2<f32>(1e-5, 1e-5));
    let lower = clamp(-nl.y, 0.0, 1.0);
    let upper = clamp(nl.y, 0.0, 1.0);
    col = col + vec3<f32>(ring * ring * RIM_GLINT * (0.25 + 1.1 * lower));
    col = col - vec3<f32>(ring * ring * RIM_DARK * upper);
    col = col + vec3<f32>(wet * wet * RIM_BODY);

    // Gentle vignette, as if peering through a wet window.
    let d = in.uv - vec2<f32>(0.5, 0.5);
    col = col * (1.0 - dot(d, d) * VIGNETTE);

    return vec4<f32>(col, 1.0);
}
