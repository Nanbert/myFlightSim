// Pilot G-strain: a screen-space grey-out and tunnel-vision overlay.
//
// A full-screen `UiMaterial` drawn above the HUD. It never samples the frame:
// it lays a veil over it, so the effect costs one transparent quad. `strain`
// is the smoothed physiological load from `src/g_strain.rs`, 0 (comfortable)
// .. 1 (the field has closed to a bright centre). The rim goes black while the
// centre drains to grey — the way a real grey-out reads, rather than a plain
// fade to black.

import bevy_ui_render::ui_vertex_output::UiVertexOutput;

struct GStrainParams {
    // x = strain 0..1, yzw unused (keeps the uniform 16 bytes).
    a: vec4<f32>,
};

@group(1) @binding(0) var<uniform> u: GStrainParams;

@fragment
fn fragment(in: UiVertexOutput) -> @location(0) vec4<f32> {
    let s = clamp(u.a.x, 0.0, 1.0);
    if (s < 0.002) {
        return vec4<f32>(0.0);
    }

    let size = max(in.size, vec2<f32>(1.0, 1.0));
    let aspect = size.x / size.y;
    var q = in.uv - vec2<f32>(0.5, 0.5);
    q.x = q.x * aspect;
    // 0 at the centre, 1 at the corners (aspect-corrected).
    let r = length(q) / (0.5 * sqrt(aspect * aspect + 1.0));

    // The clear field narrows toward the centre as the pilot greys out.
    let inner = mix(0.72, 0.20, s);
    let edge = smoothstep(inner, 1.02, r);

    // Fade the whole pass in from zero so the onset at 6 G is not a visible
    // step, and add a faint grey veil in the centre for the colour loss.
    let gate = smoothstep(0.0, 0.25, s);
    let a = clamp(gate * (edge * (0.28 + 0.64 * s) + 0.16 * s), 0.0, 0.96);

    // Black rim, grey centre: a grey-out rather than a fade to black.
    let col = mix(vec3<f32>(0.52, 0.53, 0.56), vec3<f32>(0.0), edge);
    return vec4<f32>(col, a);
}
