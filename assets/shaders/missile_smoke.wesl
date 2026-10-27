// Missile exhaust smoke — one soft billboard per puff.
//
// `src/missile_smoke.rs` drops a puff every fiftieth of a second of a missile's
// flight while its motor burns, and the puffs linger long after the round has
// gone, so the trail is a *column of overlapping discs* that frays at the edges
// instead of a tube with one silhouette.
//
// What makes it read as smoke rather than a row of grey circles:
//
// - The disc is a volume. Opacity falls off on a smooth radial ramp with no hard
//   rim, and a guard term pins the quad's own edge to zero, so overlapping puffs
//   simply sum into a solid column and the silhouette frays out where the
//   outermost ones stop covering.
// - The rim is displaced by 2D value noise sampled around the disc, so no puff
//   ends on a clean circle. `anim.z` scales that raggedness, and the CPU drives
//   it from the puff's age: round at the nozzle, torn once it has been in the
//   air a while.
// - The interior is mottled by two noise fields, one scrolling slowly with time
//   so each puff boils while it lives. That mottle also *shades* the colour: the
//   thin parts of a billow read as the shadowed side of the cloud, the dense
//   parts as the sunlit side, and a radial dome term darkens the underside of
//   the volume. Without it every puff is a flat white sticker.
//
// Both fields are sampled in the quad's own UV space, never in world space: at
// 1 km/s the sample point sweeps tens of metres per frame and a world-space
// field would alias into television static. Only the rim displacement uses polar
// coordinates, where the `atan2` seam is invisible because it only moves the
// radius.
//
// The puffs blend as "over", not additively: a trail is many puffs deep, and
// summing them would run past white within the first metre. As "over" the puff
// colour carries the brightness, and the near-white base is what makes it read
// as sunlit vapour.

import bevy_pbr::render::forward_io::VertexOutput;

struct SmokeParams {
    // rgb = lit smoke colour; a = peak opacity 0..1.
    color: vec4<f32>,
    // x = time (s), y = per-puff seed 0..1, z = rim raggedness 0..1,
    // w = shading strength 0..1.
    anim: vec4<f32>,
};

@group(constants::MATERIAL_BIND_GROUP) @binding(0) var<uniform> params: SmokeParams;

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
    let shade = params.anim.w;

    // Polar coordinates over the quad: `d` is 0 at the centre and 1 at the edge
    // midpoints, running past 1 in the corners.
    let rel = in.uv - vec2<f32>(0.5);
    let d = length(rel) * 2.0;
    let ang = atan2(rel.y, rel.x);
    let ring = vec2<f32>(cos(ang), sin(ang));

    // Billows, in the quad's own uv so there is no singular point at the centre
    // (a polar field would fan out from it as radial spokes). Two octaves: a
    // broad one that drifts and a fine one that flickers, plus the per-puff
    // seed so no two puffs share a pattern.
    let gust = noise2(in.uv * 3.2 + vec2<f32>(seed * 17.0, seed * 5.0 - time * 0.18));
    let fine = noise2(in.uv * 7.5 + vec2<f32>(seed * 29.0 + time * 0.24, seed * 43.0));
    let mottle = gust * 0.68 + fine * 0.32;

    // Torn rim: the radius the ramp is evaluated at wobbles with angle, so the
    // silhouette is a cloud edge rather than a circle.
    let rim = noise2(ring * 1.7 + vec2<f32>(seed * 23.0, seed * 7.0));
    let d_r = d * (1.0 + ragged * 0.45 * (rim - 0.5) * 2.0);

    // Body: a soft radial ramp, then a hard guard that pins the quad's own edge
    // to zero. Without the guard the rim displacement can leave opacity at the
    // edge midpoint and the billboard shows its square corners.
    var a = 1.0 - smoothstep(0.05, 1.0, d_r);
    a = a * (1.0 - smoothstep(0.72, 0.98, d));
    // Thickness: the broad field moves opacity a little, the fine one more, so
    // the disc is lumpy rather than a smooth gradient.
    a = a * mix(0.58, 1.06, mottle);
    // Tearing: the outer cloud is punched through with holes so the column
    // breaks into wisps instead of ending in a smooth tube. Driven by age.
    a = a * mix(1.0, smoothstep(0.16, 0.68, fine + 0.2), ragged * 0.32);

    a = a * params.color.a;
    if (a < 0.002) {
        discard;
    }

    // Volume shading. The dome term darkens the rim (the part of the cloud seen
    // edge-on, so the most self-shadowed) and the mottle breaks the interior
    // into lit and shadowed lobes. The same gradient the eye reads on a real
    // contrail, and the cheapest way to give a flat quad some body.
    let dome = 1.0 - smoothstep(0.05, 1.0, d);
    let lit = mix(0.86, 1.05, (dome * 0.45 + mottle * 0.55));
    let rgb = params.color.rgb * mix(1.0, lit, shade);

    return vec4<f32>(rgb, a);
}
