// Fixed cloud cards: vertical camera-facing body cards plus horizontal top
// cards. Both are ordinary transparent geometry, so the depth buffer sorts them
// against terrain and aircraft instead of hiding them behind a sky dome.
//
// From below, the vertical cards read as a cloud wall/ceiling. From above they
// would collapse into a flat white sheet, so `CloudTop` cards lie horizontally
// near the deck top and take over that view.

import bevy_pbr::render::forward_io::VertexOutput;

struct CloudCardParams {
    // xyz = camera position, w = time (s)
    camera: vec4<f32>,
    // xyz = direction toward the sun, w = rain (0..1)
    sun: vec4<f32>,
    // rgb = cloud tint, a = opacity
    tint: vec4<f32>,
    // x = seed, y = night, z = time (s), w = horizontal top card
    anim: vec4<f32>,
    // rgb = haze colour, w = density
    fog: vec4<f32>,
};

@group(constants::MATERIAL_BIND_GROUP) @binding(0) var<uniform> u: CloudCardParams;
@group(constants::MATERIAL_BIND_GROUP) @binding(1) var cloud_tex: texture_2d<f32>;
@group(constants::MATERIAL_BIND_GROUP) @binding(2) var cloud_samp: sampler;

@fragment
fn fragment(in: VertexOutput) -> @location(0) vec4<f32> {
    let is_top = clamp(u.anim.w, 0.0, 1.0);
    let seed = u.anim.x;
    let theta = seed * 6.2831853 + u.anim.z * 0.004;
    let c = cos(theta);
    let s = sin(theta);
    let centered = in.uv - vec2<f32>(0.5);
    let uv = vec2<f32>(c * centered.x - s * centered.y, s * centered.x + c * centered.y) + 0.5;
    let tex = textureSampleLevel(cloud_tex, cloud_samp, uv, 0.0);

    let dist = length(in.world_position.xyz - u.camera.xyz);
    // Vertical cards fade out near the eye; the full-screen fog carries the
    // inside-the-cloud look. Top cards sit much closer to a camera that is
    // just above the deck, so they use a tighter near fade.
    let near_fade = mix(
        smoothstep(600.0, 2600.0, dist),
        smoothstep(40.0, 500.0, dist),
        is_top,
    );
    let density_gain = mix(1.25, 0.95, is_top);
    var alpha = tex.a * u.tint.a * density_gain * near_fade;
    if (alpha < 0.003) {
        discard;
    }

    // Vertical cards get a top-lit vertical gradient. Horizontal top cards get
    // a grey, softly shadowed body instead of a flat white fill: thin edges and
    // holes are darker, so overlapping cards read as cloud tops, not snow.
    let vertical_body = 0.52 + 0.52 * smoothstep(0.0, 1.0, uv.y);
    let top_body = 0.68 + 0.32 * tex.a;
    let body = mix(vertical_body, top_body, is_top);
    var col = u.tint.rgb * body;
    col *= 0.93 + 0.07 * tex.rgb;
    col *= mix(1.0, 0.80 + 0.20 * tex.a, is_top);

    let sun = normalize(u.sun.xyz);
    let sun_amount = clamp(dot(normalize(in.world_normal), sun), 0.0, 1.0);
    col *= 0.82 + 0.24 * sun_amount;

    // Aerial perspective: cards fold into the same haze as the sky.
    let haze = 1.0 - exp(-dist * u.fog.w);
    col = mix(col, u.fog.rgb, clamp(haze, 0.0, 1.0));
    alpha *= 1.0 - 0.55 * clamp(haze, 0.0, 1.0);

    // Rain dims the deck, night sweeps it toward navy.
    col *= (1.0 - 0.22 * u.sun.w);
    let night = clamp(u.anim.y, 0.0, 1.0);
    col = (col + vec3<f32>(0.006, 0.010, 0.020) * night) * (1.0 - 0.86 * night);

    return vec4<f32>(col, alpha);
}
