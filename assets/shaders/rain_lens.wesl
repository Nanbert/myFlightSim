// Rain on the lens: a screen-space droplet post-process for the chase views.
// The cockpit view removes the pass entirely (`src/lens.rs`).
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
// The uniform carries the air data that makes the film behave like water in a
// slipstream: `phase` is an animation clock advanced faster with dynamic
// pressure, `flow` both stretches each drop into a wake and sets how fast the
// whole field streams across the glass, and `rain_share` decides whether the
// deposit is a downpour or a fine condensation haze. The wetness itself
// (`intensity`) is supplied by `src/lens.rs`, which only feeds the film when
// there is rain or condensing air to deposit.
//
// License: *The Drive Home* is published under Shadertoy's default
// CC BY-NC-SA 3.0 (non-commercial, share-alike) — resolve before shipping the
// sim commercially. The Bevy plumbing around it (`src/lens.rs`) is original.

import bevy_core_pipeline::fullscreen_vertex_shader::FullscreenVertexOutput;

struct RainLens {
    intensity: f32,
    phase: f32,
    flow: f32,
    rain_share: f32,
    // `.x` = weather rain intensity, 0 (clear) .. 1 (downpour): heavier rain
    // falls faster. `.yzw` pad the row; the droplet speed is also scattered
    // per drop below.
    rain_pad: vec4<f32>,
};

@group(0) @binding(0) var screenTexture: texture_2d<f32>;
@group(0) @binding(1) var samp: sampler;
@group(0) @binding(2) var<uniform> settings: RainLens;

// --- tuning ---
const REFRACT: f32 = 0.8;     // drop lens strength (screen-UV offset scale)
const RIM_GLINT: f32 = 0.03;  // brightness of the lit (lower) rim of a drop
const RIM_DARK: f32 = 0.008;  // how dark the shadowed (upper) rim goes
const RIM_BODY: f32 = 0.003;  // faint sheen filling the body of a drop
const VIGNETTE: f32 = 0.15;   // corner darkening
const TAU: f32 = 6.2831853;
// Whole-field drift, in screen heights per unit of `t`. Bevy's UV points down,
// so a negative value sinks the film under gravity while a positive one streams
// it up the glass in the slipstream. The original shader was a moving car's
// windshield and used a positive constant even at rest; parked, that reads as
// rain floating upward.
const REST_SCROLL: f32 = -0.05;
const FLOW_SCROLL: f32 = 0.80;

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

// A drop's vertical slide over its life. A true sawtooth: every drop travels
// only downward (Bevy's UV y points down) and recycles at the top of its cell.
// The original used an oscillating cosine, so half the drops climbed the glass
// and the rain read as floating upward whenever the airframe was parked.
// `rate` is the drop's own fall speed, scattered per cell in `GetDrops`, and
// `phase` gives every cell its own place in the fall (a full cycle, so a
// random rate cannot bunch the drops into a visible wave).
const DROP_SLIDE_RATE: f32 = 2.0 / TAU;
fn SawTooth(t: f32, rate: f32, phase: f32) -> f32 {
    return 0.5 - fract(t * rate + phase);
}

// Weather rain-rate -> mean fall-speed multiplier. A drizzle crawls and a
// storm drives the film down; the square keeps light rain clearly slower
// instead of splitting the range linearly. (0.9 is the heaviest preset.)
const RAIN_FALL_BASE: f32 = 0.35;
const RAIN_FALL_GAIN: f32 = 3.9;
fn rain_fall(rain: f32) -> f32 {
    return RAIN_FALL_BASE + RAIN_FALL_GAIN * rain * rain;
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
// `scroll` is the vertical drift for one unit of `t`, in this layer's pattern
// space; callers scale it by their uv multiplier so every layer streams at the
// same speed on screen.
fn GetDrops(uv_in: vec2<f32>, seed: f32, m: f32, time: f32, scroll: f32) -> Drop {
    var t = time + m * 30.0;
    var uv = uv_in;
    uv.y = uv.y + t * scroll;
    uv = uv * vec2<f32>(20.0, 5.4);

    let id = floor(uv);
    let n = N31(id.x + (id.y + seed) * 546.35242);

    // Local cell coords, stretched vertically so drops read as tall teardrops.
    var bd = fract(uv) - vec2<f32>(0.5, 0.5);
    bd.y = bd.y * 4.0;
    bd.x = bd.x + (n.x - 0.5) * 0.6;

    // Heavier rain falls faster. The hash scatters each cell's drop around
    // that mean, so the field doesn't slide as one uniform sheet. `n.z` is a
    // full-cycle phase offset, so the scattered rates stay evenly mixed.
    let phase = n.z;
    let speed = rain_fall(settings.rain_pad.x) * (0.6 + 0.8 * n.y);
    let rate = DROP_SLIDE_RATE * speed;
    let slide = SawTooth(t, rate, phase);

    // The trail: beads left behind, sliding at the drop's own rate.
    let ts = 1.5;
    let trailPos = vec2<f32>(
        bd.x * ts,
        (fract(bd.y * ts * 2.0 - t * 2.0 * speed + phase) - 0.5) * 0.5,
    );

    // Slide the drop, then add its shape: the square term buckles the bottom
    // outward by its own speed, giving the classic teardrop silhouette.
    bd.y = bd.y + slide * 2.0;
    var dropShape = bd.x * bd.x;
    dropShape = dropShape * -rate;
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

// One layer, smeared by the airflow. A second sample further along the +y side
// the beaded trail already occupies extends the drop into a wake: the head
// still does the refracting, the displaced copy just stretches the silhouette.
// `shear` and `scroll` are in this layer's pattern space, so callers scale both
// by their uv multiplier to keep the streak length and stream speed fixed on
// screen.
fn GetStreaked(uv: vec2<f32>, seed: f32, m: f32, t: f32, shear: f32, scroll: f32) -> Drop {
    let head = GetDrops(uv, seed, m, t, scroll);
    if shear <= 0.0 {
        return head;
    }
    let tail = GetDrops(uv + vec2<f32>(0.0, shear), seed, m, t, scroll);
    // Weight by coverage squared: the head dominates where it is present, and
    // the tail only takes over where the head has already slid away.
    let wa = head.mask * head.mask;
    let wb = tail.mask * tail.mask;
    let norm = wa + wb + 1e-5;
    let offset = (head.offset * wa + tail.offset * wb) / norm;
    let nrm = normalize((head.nrm * wa + tail.nrm * wb) / norm + vec2<f32>(1e-5, 1e-5));
    return Drop(offset, max(head.mask, tail.mask), nrm);
}

@fragment
fn fragment(in: FullscreenVertexOutput) -> @location(0) vec4<f32> {
    let res = vec2<f32>(textureDimensions(screenTexture));
    let aspect = res.x / res.y;

    let src = textureSampleLevel(screenTexture, samp, in.uv, 0.0).rgb;
    if settings.intensity < 0.001 {
        return vec4<f32>(src, 1.0);
    }

    let t = settings.phase;
    let m = 0.0;
    let fall = rain_fall(settings.rain_pad.x);
    // Dynamic pressure stretches each drop into a wake. A fixed screen length,
    // so the per-layer uv scale below is compensated in the shear passed down.
    let shear = 0.10 * settings.flow;
    // How far the whole field streams per unit of `t`, in screen-heights. At
    // rest gravity draws the film down at the rain's own rate; the flow term is
    // the slipstream, which pushes it up the glass. `PHASE_FLOW` in `lens.rs`
    // already quickens `t` with q, so the two together make the film race at
    // fighter speed instead of crawling.
    let scroll = REST_SCROLL * fall + FLOW_SCROLL * settings.flow;

    // Centred and aspect-corrected screen space. The droplet pattern drifts
    // and turns slowly, so it never sits static.
    var uv = in.uv - vec2<f32>(0.5, 0.5);
    uv.x = uv.x * aspect;
    let ang = -((sin(t * 0.1) * 0.5 + 0.5) * 0.5);
    let a = ang * ang;
    let ca = cos(a);
    let sa = sin(a);
    var dropUv = vec2<f32>(uv.x * ca + uv.y * sa, -uv.x * sa + uv.y * ca);
    dropUv.x = dropUv.x + (-sin(t * 0.1)) * 0.5;

    // More water → more layers come up, from a thin mist to a downpour.
    // Condensation (low `rain_share`) only ever reaches the fine layers: it is
    // a haze of beads, not the sheet of drops a rainstorm throws at the glass.
    //
    // Dynamic pressure tears large beads apart long before it can keep them on
    // the glass. Above a couple hundred knots the big-drop layers fade out
    // entirely and what remains is a fine, streaked spray — not a screenful of
    // bubbles. (`lens.rs` thins the film's coverage with flow for the same
    // reason.) These are just weights, not extra geometry: the pass costs the
    // same however many droplets are visible.
    let thin = S(0.0, 1.0, settings.flow);
    let l0 = S(0.0, 0.35, settings.intensity) * max(1.0 - 1.4 * thin, 0.0) * 0.60;
    let l1 = S(0.0, 0.6, settings.intensity) * (0.35 + 0.65 * settings.rain_share)
        * max(1.0 - 0.75 * thin, 0.0);
    let l2 = S(0.0, 1.0, settings.intensity) * (0.15 + 0.85 * settings.rain_share)
        * (1.0 - 0.30 * thin);

    let d0 = GetStreaked(dropUv, 1.0, m, t, shear, scroll);
    let d1 = GetStreaked(dropUv * 1.4, 10.0, m, t, shear * 1.4, scroll * 1.4);
    let d2 = GetStreaked(dropUv * 2.4, 25.0, m, t, shear * 2.4, scroll * 2.4);

    var offs = d0.offset * l0 + d1.offset * l1 + d2.offset * l2;
    var lens = max(d0.mask * l0, max(d1.mask * l1, d2.mask * l2));
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
