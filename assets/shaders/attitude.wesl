// The Cessna-172 attitude indicator (artificial horizon), drawn procedurally.
//
// A `UiMaterial` on a square node: for each pixel relative to the dial centre
// we un-rotate by the bank angle and offset by the pitch, giving the pixel's
// position in the *earth* frame. From there the sky/ground split, the pitch
// ladder and the horizon are all trivial; the fixed parts (bezel, bank scale,
// symbolic aircraft) are drawn in the node's own frame on top.
//
// All positions are in node pixels; `R` is the dial radius, so the whole layout
// scales with the on-screen gauge size.

import bevy_ui_render::ui_vertex_output::UiVertexOutput;

struct AttitudeUniform {
    /// x = pitch (deg, + nose up), y = roll (deg, + right wing down).
    params: vec4<f32>,
};

@group(1) @binding(0) var<uniform> u: AttitudeUniform;

const SKY_HORIZON: vec3<f32> = vec3<f32>(0.36, 0.60, 0.86);
const SKY_TOP: vec3<f32> = vec3<f32>(0.10, 0.28, 0.62);
const GND_HORIZON: vec3<f32> = vec3<f32>(0.52, 0.34, 0.16);
const GND_DEEP: vec3<f32> = vec3<f32>(0.26, 0.15, 0.06);
const MARK: vec3<f32> = vec3<f32>(0.95, 0.95, 0.95);
const AIRCRAFT: vec3<f32> = vec3<f32>(0.98, 0.86, 0.18);

const BEZEL: vec3<f32> = vec3<f32>(0.055, 0.055, 0.063);
const BEZEL_HI: vec3<f32> = vec3<f32>(0.180, 0.188, 0.212);
const BEZEL_RIM: vec3<f32> = vec3<f32>(0.408, 0.423, 0.455);

// Pitch range that spans the dial radius: 30 deg of pitch reaches the edge.
const PITCH_SPAN: f32 = 30.0;

fn deg2rad(d: f32) -> f32 { return d * 0.0174532925; }

@fragment
fn fragment(in: UiVertexOutput) -> @location(0) vec4<f32> {
    let size = in.size;
    let px = (in.uv - vec2<f32>(0.5, 0.5)) * size; // offset from centre, y down
    let R = size.x * 0.4845;
    let u01 = length(px) / R;

    if u01 > 1.0 {
        return vec4<f32>(0.0, 0.0, 0.0, 0.0);
    }
    if u01 >= 0.90 {
        if u01 > 0.985 { return vec4<f32>(BEZEL_RIM, 1.0); }
        if u01 > 0.950 { return vec4<f32>(BEZEL_HI, 1.0); }
        return vec4<f32>(BEZEL, 1.0);
    }

    let pxd = R / PITCH_SPAN; // node px per degree of pitch
    let pitch = u.params.x;
    let roll = u.params.y;

    // The earth frame is the node frame un-rotated by the bank angle.
    let c = cos(deg2rad(roll));
    let s = sin(deg2rad(roll));
    let ey = -px.x * s + px.y * c - pitch * pxd; // 0 = horizon, + = below

    // --- sky / ground ---
    let skyt = clamp(-ey / (R * 0.62), 0.0, 1.0);
    let gnd = clamp(ey / (R * 0.62), 0.0, 1.0);
    var col = select(mix(GND_HORIZON, GND_DEEP, gnd), mix(SKY_HORIZON, SKY_TOP, skyt), ey < 0.0);

    // --- pitch ladder: a line every 5 deg, longer and heavier every 10 ---
    let deg = ey / pxd;                     // earth-frame pitch, + below horizon
    let nearest = round(deg / 5.0) * 5.0;
    let major = (nearest / 10.0) == round(nearest / 10.0);
    let halfw = select(0.22, 0.38, major);  // line half-width, in degrees
    let span = select(0.42, 0.62, major);   // line half-length, fraction of R
    let absnearest = abs(nearest);
    let on_line = abs(deg - nearest) < halfw && absnearest >= 5.0 && absnearest <= 30.0;
    let ex = px.x * c + px.y * s;           // earth-frame horizontal
    if on_line && abs(ex) < span * R {
        col = MARK;
    }
    // Horizon line, full width.
    if abs(ey) < 0.9 {
        col = MARK;
    }

    // --- fixed bank scale, near the top of the case ---
    let ang = atan2(px.x, -px.y);           // 0 = straight up, + = roll right
    let degang = abs(ang) * 57.29578;
    let near_mark = abs(degang - 10.0) < 0.6 || abs(degang - 20.0) < 0.6
        || abs(degang - 30.0) < 0.6 || abs(degang - 60.0) < 0.6 || abs(degang - 90.0) < 0.6;
    if near_mark && u01 >= 0.78 && u01 < 0.90 {
        col = MARK;
    }
    // Reference triangle at the very top.
    if abs(px.x) < (u01 - 0.70) * R * 0.9 && u01 >= 0.72 && u01 < 0.80 {
        col = MARK;
    }

    // --- symbolic aircraft: a wing bar with tips turned down ---
    let ax = px.x / R;
    let ay = px.y / R;
    var sym = abs(ay) < 0.014 && abs(ax) < 0.30;
    sym = sym || (abs(abs(ax) - 0.30) < 0.014 && ay > -0.014 && ay < 0.075);
    sym = sym || length(vec2<f32>(ax, ay)) < 0.028;
    if sym {
        col = AIRCRAFT;
    }

    return vec4<f32>(col, 1.0);
}
