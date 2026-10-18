// Wing-surface condensation layer.
//
// One wing-shaped volume per side. The box is only the raymarch bounds; the
// density is masked to the F-22 wing planform in box-local coordinates and
// falls off with height, so the layer hugs the upper surface instead of reading
// as a blob. `anim.w` shrinks the thickness toward the wing as the burst dies.

import bevy_pbr::render::forward_io::VertexOutput;
import bevy_pbr::render::mesh_view_bindings::view;

struct BoxParams {
    color: vec4<f32>,
    // x = intensity, y = time (s), z = side, w = thickness scale (0..1).
    anim: vec4<f32>,
    light: vec4<f32>,
    world_to_local: mat4x4<f32>,
};

@group(constants::MATERIAL_BIND_GROUP) @binding(0) var<uniform> params: BoxParams;

const STEPS: i32 = 20;

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

fn intersect_unit_box(ro: vec3<f32>, rd: vec3<f32>) -> vec2<f32> {
    let inv = 1.0 / rd;
    let a = (-0.5 - ro) * inv;
    let b = (0.5 - ro) * inv;
    let tmin = min(a, b);
    let tmax = max(a, b);
    let enter = max(max(tmin.x, tmin.y), tmin.z);
    let exit = min(min(tmax.x, tmax.y), tmax.z);
    return vec2<f32>(enter, exit);
}

@fragment
fn fragment(in: VertexOutput) -> @location(0) vec4<f32> {
    let intensity = params.anim.x;
    if (intensity < 0.004) {
        discard;
    }

    let cam = view.world_position;
    let P = in.world_position.xyz;
    let world_rd = normalize(P - cam);
    let ro = (params.world_to_local * vec4<f32>(cam, 1.0)).xyz;
    let rd = normalize((params.world_to_local * vec4<f32>(world_rd, 0.0)).xyz);
    let hit = intersect_unit_box(ro, rd);
    let t0 = max(hit.x, 0.0);
    let t1 = hit.y;
    if (t1 <= t0) {
        discard;
    }

    let dt = (t1 - t0) / f32(STEPS);
    let sun = normalize(params.light.xyz);
    let side = params.anim.z;
    let s = max(params.anim.w, 0.001);

    var acc = vec3<f32>(0.0);
    var trans = 1.0;
    for (var i = 0; i < STEPS; i = i + 1) {
        let p = ro + rd * (t0 + (f32(i) + 0.5) * dt);

        // Box local x/z are the planform; y is the layer height. Scaling only
        // y makes the burst collapse down onto the wing instead of becoming a
        // small ball.
        let h = clamp((p.y + 0.5) / s, 0.0, 1.0);
        let lx = select(p.x, -p.x, side > 0.5);
        let n = fbm3(
            vec3<f32>(p.x * 4.0, p.y * 4.0, p.z * 4.0)
                + vec3<f32>(side * 9.0, params.anim.y * 0.25, 0.0),
        );
        let n2 = fbm3(
            vec3<f32>(p.x * 8.0, p.y * 6.0, p.z * 8.0)
                + vec3<f32>(5.0, params.anim.y * 0.60, side * 2.0),
        );

        // Speed-driven footprint. At low speed the ellipse is small and sits at
        // the root/leading edge; as speed rises it grows and moves outboard so
        // the same burst covers more of the wing.
        let cov = clamp(params.color.w, 0.25, 1.0);
        let root_bias = 1.0 - cov;
        let cx = 0.26 * root_bias;
        let cz = -0.20 - 0.12 * root_bias;
        let wob = (n - 0.5) * 0.04;
        let r = 0.18 + 0.82 * cov;
        let radial = 1.0 - smoothstep(
            0.30,
            0.52,
            length(vec2<f32>((lx - cx) * 1.15, (p.z - cz) * 1.10)) / r + wob,
        );
        // At full coverage the ellipse reaches the raymarch box faces; fade the
        // density out before them so the layer never gets a rectangular cut.
        let box_fade = smoothstep(0.0, 0.14, 0.5 - abs(p.x))
            * smoothstep(0.0, 0.14, 0.5 - abs(p.z));
        let edge = radial * box_fade;

        let height = pow(1.0 - h, 1.15);
        let field = edge * height * (0.25 + 0.75 * n) * (0.70 + 0.30 * n2);
        let density = field * intensity * 1000.0;
        let a = 1.0 - exp(-density * dt);

        if (a > 0.0001) {
            let ndl = max(sun.y, 0.0);
            let V = normalize(cam - P);
            let back = pow(max(dot(-V, sun), 0.0), 3.0);
            let lit = clamp(0.30 + 0.70 * ndl + 0.45 * back, 0.0, 1.5)
                * (0.80 + 0.20 * n2);
            let shade = smoothstep(0.0, 0.85, h);
            let col = mix(
                vec3<f32>(0.42, 0.52, 0.66),
                vec3<f32>(0.94, 0.97, 1.0),
                shade * 0.55,
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
