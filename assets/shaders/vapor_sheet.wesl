// Volumetric wing-surface condensation sheet.
//
// One body-frame patch per wing, sitting just above the upper surface. Instead
// of a flat alpha decal, each fragment marches a short path through the
// boundary layer (from the wing surface toward the camera) and integrates a 3D
// noise density. The result has real thickness, parallax and self-occlusion,
// and the sheet rides the airframe so it gathers in place and dissipates
// instead of leaving a long wake.

import bevy_pbr::render::forward_io::VertexOutput;
import bevy_pbr::render::mesh_view_bindings::view;

struct SheetParams {
    // x = intensity (0..1), y = time (s), z = side, w = layer.
    anim: vec4<f32>,
    // rgb = vapour colour.
    tint: vec4<f32>,
    // xyz = direction toward the sun.
    light: vec4<f32>,
};

@group(constants::MATERIAL_BIND_GROUP) @binding(0) var<uniform> params: SheetParams;

const STEPS: i32 = 18;
// Thickness of the boundary layer above the wing, metres.
const THICKNESS: f32 = 1.40;

fn hash31(p: vec3<f32>) -> f32 {
    var q = fract(p * vec3<f32>(0.1031, 0.1030, 0.0973));
    q += dot(q, q.yxz + 33.33);
    return fract((q.x + q.y) * q.z);
}

fn noise3(p: vec3<f32>) -> f32 {
    let i = floor(p);
    let f = fract(p);
    let u = f * f * (3.0 - 2.0 * f);
    let x00 = mix(hash31(i), hash31(i + vec3<f32>(1.0, 0.0, 0.0)), u.x);
    let x10 = mix(
        hash31(i + vec3<f32>(0.0, 1.0, 0.0)),
        hash31(i + vec3<f32>(1.0, 1.0, 0.0)),
        u.x,
    );
    let x01 = mix(
        hash31(i + vec3<f32>(0.0, 0.0, 1.0)),
        hash31(i + vec3<f32>(1.0, 0.0, 1.0)),
        u.x,
    );
    let x11 = mix(
        hash31(i + vec3<f32>(0.0, 1.0, 1.0)),
        hash31(i + vec3<f32>(1.0, 1.0, 1.0)),
        u.x,
    );
    return mix(mix(x00, x10, u.y), mix(x01, x11, u.y), u.z);
}

fn fbm3(p: vec3<f32>) -> f32 {
    var sum = 0.0;
    var amp = 0.5;
    var q = p;
    for (var i = 0; i < 3; i = i + 1) {
        sum += amp * noise3(q);
        q = q * 2.07 + vec3<f32>(13.1, 7.7, 19.3);
        amp *= 0.5;
    }
    return sum;
}

@fragment
fn fragment(in: VertexOutput) -> @location(0) vec4<f32> {
    let intensity = params.anim.x;
    if (intensity < 0.004) {
        discard;
    }

    let uv = in.uv;
    let t = params.anim.y;
    // A noise-driven edge keeps the trapezoid patch from cutting a clean line.
    let edge_n = fbm3(vec3<f32>(uv * 4.0, t * 0.20));
    // u: root -> tip; v: leading edge -> trailing edge.
    let span = smoothstep(0.02 + 0.05 * edge_n, 0.14 + 0.06 * edge_n, uv.x)
        * (1.0 - smoothstep(0.66 + 0.08 * edge_n, 0.98, uv.x));
    let chord = smoothstep(0.01 + 0.05 * edge_n, 0.12 + 0.06 * edge_n, uv.y)
        * (1.0 - smoothstep(0.50 + 0.08 * edge_n, 0.95, uv.y));
    let edge = span * chord;
    if (edge < 0.002) {
        discard;
    }

    // March from the wing surface toward the camera. `h` is the fraction of the
    // boundary-layer thickness along the view ray, so the density is densest at
    // the surface and feathers out above it.
    let P = in.world_position.xyz;
    let V = normalize(view.world_position - P);
    let side = params.anim.z;
    let dt = THICKNESS / f32(STEPS);
    let sun = normalize(params.light.xyz);

    var acc = vec3<f32>(0.0);
    var trans = 1.0;
    for (var i = 0; i < STEPS; i = i + 1) {
        let h = (f32(i) + 0.5) / f32(STEPS);
        let height = pow(1.0 - h, 1.35);
        let q = vec3<f32>(
            uv.x * 6.0 + side * 7.0,
            uv.y * 9.0,
            h * 3.0 + t * 0.45,
        );
        let n = fbm3(q + vec3<f32>(0.0, t * 0.10, 0.0));
        let d = edge * height * smoothstep(0.30, 0.72, n) * intensity * 9.0;
        let a = 1.0 - exp(-d * dt);
        if (a > 0.0001) {
            let ndl = max(sun.y, 0.0);
            // Silver lining when looking toward the sun through the sheet.
            let back = pow(max(dot(-V, sun), 0.0), 3.0);
            let lit = clamp(0.30 + 0.70 * ndl + 0.45 * back, 0.0, 1.5);
            let col = mix(
                vec3<f32>(0.55, 0.65, 0.78),
                vec3<f32>(1.0, 0.98, 0.94),
                clamp(ndl * 0.7 + back * 0.5, 0.0, 1.0),
            ) * lit;
            acc += col * a * trans;
            trans *= 1.0 - a;
        }
        if (trans < 0.02) {
            break;
        }
    }

    let alpha = 1.0 - trans;
    if (alpha < 0.004) {
        discard;
    }
    return vec4<f32>(acc, alpha);
}
