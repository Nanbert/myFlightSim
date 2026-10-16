// Transonic vapor cone.
//
// A translucent cone shell around the F-22's forward fuselage (see
// `src/vapor.rs`), drawn with the default mesh vertex shader and this fragment
// shader. Its opacity is driven by Mach from the CPU: it blooms as the aircraft
// crosses the sound barrier and is empty (discarded) everywhere else.
//
// `Cone`'s UVs are polar — 0.5,0.5 at the apex, radius 0.5 out at the base rim —
// so the radial coordinate is a longitudinal one: 0 at the nose through 1 at the
// wide end. The shell fades at both ends and brightens at grazing angles (more
// vapor along the line of sight), with a drifting noise field so it never reads
// as a clean surface.

import bevy_pbr::render::forward_io::VertexOutput;
import bevy_pbr::render::mesh_view_bindings::view;

struct ConeParams {
    // x = intensity (0..1), y = time (s), z/w unused.
    anim: vec4<f32>,
    // rgb = vapor colour.
    tint: vec4<f32>,
};

@group(constants::MATERIAL_BIND_GROUP) @binding(0) var<uniform> params: ConeParams;

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

// Cheap 3D field: two axis-aligned planes blended, enough structure to break up
// the shell without a real trilinear lattice.
fn noise3(p: vec3<f32>) -> f32 {
    let xy = noise2(p.xy + p.z * 17.0);
    let yz = noise2(p.yz + p.x * 17.0);
    return mix(xy, yz, 0.5);
}

@fragment
fn fragment(in: VertexOutput) -> @location(0) vec4<f32> {
    let intensity = params.anim.x;
    if (intensity < 0.004) {
        discard;
    }
    let time = params.anim.y;

    // Longitudinal coordinate down the cone: 0 at the apex, 1 at the base rim.
    let t = clamp(length(in.uv - vec2<f32>(0.5)) * 2.0, 0.0, 1.0);
    // Soft at the nose, fading before the wide opening: a shell, not a cap.
    let shape = smoothstep(0.0, 0.35, t) * smoothstep(1.0, 0.45, t);

    // Grazing view brightens the rim — more vapor along the sight line.
    let n = normalize(in.world_normal);
    let v = normalize(view.world_position - in.world_position.xyz);
    let fresnel = pow(1.0 - abs(dot(n, v)), 2.5);

    // Drifting condensation, so the shell breathes instead of sitting still.
    let wp = in.world_position.xyz;
    let nz = noise3(wp * 3.0 + vec3<f32>(0.0, 0.0, time * 2.0));

    let a = intensity * shape * (0.35 + 0.65 * fresnel) * (0.5 + 0.7 * nz);
    let alpha = clamp(a, 0.0, 1.0) * 0.6;
    if (alpha < 0.004) {
        discard;
    }
    return vec4<f32>(params.tint.rgb, alpha);
}
