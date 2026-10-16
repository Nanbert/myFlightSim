// Afterburner exhaust plume — one soft additive billboard per puff.
//
// `src/fx.rs` lays a fixed ring of these along each nozzle's plume axis and
// recycles each one from the nozzle aft, so the plume is a *stream of
// overlapping soft discs* rather than a shell. That is the whole difference
// from the cone this replaced: a shell has one silhouette no matter how it is
// shaded, whereas a stream of discs can fray, and the plume's edge is then made
// of the individual puffs that happen to still be covering it.
//
// Three things stop it reading as a row of identical blobs:
//
// - The disc is a *volume*, not a decal. The opacity falls off as a smooth
//   radial ramp with no hard rim, so where puffs overlap their contributions
//   simply sum into a solid column, and the silhouette frays out where the
//   outermost ones stop covering.
// - The rim is displaced by a 2D noise field sampled in the quad's own polar
//   coordinates, so no puff ends on a clean circle. `anim.z` scales that
//   raggedness and the CPU drives it from the puff's age: a round hot
//   cross-section at the nozzle, a torn one at the tail.
// - The interior is mottled by a second noise field scrolling with time, so
//   each puff boils on its own rather than the plume pulsing as one sheet.
//
// The puffs composite as a translucent *volume*, not as an additive sum:
// `AlphaMode::Blend` gives plain "over", so a ray through the plume settles
// toward the puff colour instead of accumulating. That matters here — the plume
// is thirty-odd puffs deep, and summing them would run past the top of the
// range within the first metre. As "over" it is the puff *colour* that carries
// the brightness, and the near-nozzle colour rides past 1.0 so the camera's
// bloom catches the core.
//
// The trap is the same one the cone hit: the camera runs `Tonemapping::None`,
// so any channel at or above 1.0 clips flat there. Green and blue therefore stay
// under 1.0 except in the first metre of the plume, which is the only place that
// is meant to read as white-hot.
//
// All three noise samples are taken in the quad's coordinates, never in world
// space. World space would be wrong: at 300 m/s the sample point sweeps tens of
// metres a frame and the pattern aliases into television static.

import bevy_pbr::render::forward_io::VertexOutput;

struct FlamePuffParams {
    // rgb = puff colour (red may exceed 1.0 for bloom), a = peak opacity 0..1.
    color: vec4<f32>,
    // x = time (s), y = per-puff seed 0..1, z = rim raggedness 0..1,
    // w = unused.
    anim: vec4<f32>,
};

@group(constants::MATERIAL_BIND_GROUP) @binding(0) var<uniform> params: FlamePuffParams;

// 2D value hash (Dave Hoskins style), and the value noise built on it.
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

@fragment
fn fragment(in: VertexOutput) -> @location(0) vec4<f32> {
    let time = params.anim.x;
    let seed = params.anim.y;
    let ragged = params.anim.z;

    // Polar coordinates over the quad: `d` is 0 at the centre and 1 at the edge
    // midpoints, running past 1 in the corners so they mask off cleanly.
    let rel = in.uv - vec2<f32>(0.5);
    let d = length(rel) * 2.0;
    let ang = atan2(rel.y, rel.x);
    // Sample the circle as (cos, sin) rather than by angle: the field then
    // wraps seamlessly where `atan2` jumps at ±π, which a raw angle tears open.
    let ring = vec2<f32>(cos(ang), sin(ang));

    // Torn rim. The radius the ramp is evaluated at wobbles with angle, so the
    // silhouette is a cloud edge rather than a circle. Scaling it by `ragged`
    // leaves the nozzle end with a clean round cross-section, which is what
    // makes the column look like it is still coming out of a pipe.
    let rim = noise2(ring * 1.7 + vec2<f32>(seed * 23.0, seed * 7.0));
    let d_r = d * (1.0 + ragged * 0.75 * (rim - 0.5) * 2.0);

    // Body: a soft radial ramp, brightest on the axis. The low floor on the
    // inner smoothstep is deliberate — a hard core would show as a disc wherever
    // puffs fail to overlap, and summing soft ramps already builds a solid
    // column where they do.
    var a = 1.0 - smoothstep(0.08, 1.0, d_r);

    // Interior mottle, also in polar coordinates: breaks each disc into patches
    // so the plume boils instead of brightening and dimming as one sheet. On the
    // opacity, not the colour, because against a dark sky it is *coverage* that
    // the eye reads as density.
    let gust = noise2(ring * 3.6 + vec2<f32>(time * 2.1, seed * 31.0));
    a = a * mix(0.40, 1.30, gust);

    // Tearing, scaled by the same `ragged` the rim uses. The mottle above only
    // varies density; this one is allowed to reach zero, so the outer plume is
    // punched through with holes and its silhouette breaks into wisps. That is
    // what keeps the tail reading as *particles* rather than as one long smudge
    // — the discs alone overlap far too much to separate on their own.
    let tear = noise2(ring * 2.8 + vec2<f32>(seed * 13.0, time * 1.4));
    a = a * mix(1.0, smoothstep(0.28, 0.60, tear), ragged * 0.90);

    a = a * params.color.a;
    if (a < 0.002) {
        discard;
    }
    return vec4<f32>(params.color.rgb, a);
}
