// Local volumetric wing-vapor box.
//
// Two boxes (one per wing) ride the airframe. Each fragment ray-marches the
// boundary-layer density inside the box, so the cloud has a real 3D silhouette
// and self-occlusion instead of reading as a flat decal or a stack of
// billboards. Density is densest at the wing surface and feathers upward.

import bevy_pbr::render::forward_io::VertexOutput;
import bevy_pbr::render::mesh_view_bindings::view;

struct BoxParams {
    // rgb = vapour colour, a unused.
    color: vec4<f32>,
    // x = intensity, y = time (s), z = side, w unused.
    anim: vec4<f32>,
    // xyz = direction toward the sun.
    light: vec4<f32>,
    // World -> box-local (unit cube) transform.
    world_to_local: mat4x4<f32>,
};

@group(constants::MATERIAL_BIND_GROUP) @binding(0) var<uniform> params: BoxParams;

const STEPS: i32 = 16;

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
    var acc = vec3<f32>(0.0);
    var trans = 1.0;

    for (var i = 0; i < STEPS; i = i + 1) {
        let p = ro + rd * (t0 + (f32(i) + 0.5) * dt);
        let h = clamp(p.y + 0.5, 0.0, 1.0);
        let height = smoothstep(0.0, 0.22, h) * (1.0 - smoothstep(0.58, 1.0, h));
        let n = fbm3(
            vec3<f32>(p.x * 5.5, p.y * 4.0, p.z * 5.5)
                + vec3<f32>(side * 9.0, params.anim.y * 0.20, 0.0),
        );
        // Rounded, noise-dithered ellipsoid: the box faces never shape the
        // visible cloud, only the density falloff does.
        let radial = 1.0 - smoothstep(
            0.20,
            0.55,
            length(p * 1.20) + (n - 0.5) * 0.08,
        );
        let ex = smoothstep(0.0, 0.08, 0.5 - abs(p.x));
        let ez = smoothstep(0.0, 0.08, 0.5 - abs(p.z));
        let edge = radial * ex * ez;
        let n2 = fbm3(
            vec3<f32>(p.x * 11.0, p.y * 8.0, p.z * 11.0)
                + vec3<f32>(4.0, params.anim.y * 0.35, side * 3.0),
        );
        let d = edge
            * height
            * smoothstep(0.20, 0.62, n)
            * (0.55 + 0.45 * n2)
            * intensity
            * 14.0;
        let a = 1.0 - exp(-d * dt);
        if (a > 0.0001) {
            let ndl = max(sun.y, 0.0);
            let V = normalize(cam - P);
            let back = pow(max(dot(-V, sun), 0.0), 3.0);
            let lit = clamp(0.32 + 0.68 * ndl + 0.45 * back, 0.0, 1.5);
            // Cool shadow at the base, warm white at the top: the wing is
            // already light, so a uniform white cloud would vanish into it.
            let shade = smoothstep(0.0, 0.55, h);
            let col = mix(
                vec3<f32>(0.36, 0.46, 0.60),
                vec3<f32>(1.0, 0.98, 0.94),
                shade,
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
