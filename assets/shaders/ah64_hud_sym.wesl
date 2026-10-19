// AH-64D IHADSS-style head-up display.
//
// Drawn procedurally from the uniform, so it stays sharp at any resolution.
// The symbology follows the DCS AH-64D guide: heading tape, airspeed and
// barometric/radar altitude, torque and rotor RPM, collective, vertical speed,
// flight-path marker, waypoint and the arcade auto-hover/auto-balance flags.
//
// The Apache has no fixed HUD; this is the IHADSS monocle symbology rendered as
// a screen overlay, visible in both the chase and cockpit views.

struct Ah64Params {
    // cas_kts, alt_ft, vs_fpm, radar_alt_ft
    a: vec4<f32>,
    // heading_deg, pitch_deg, roll_deg, collective_pct
    b: vec4<f32>,
    // torque_pct, rotor_rpm_pct, fpm_x_deg, fpm_y_deg
    c: vec4<f32>,
    // gs_kts, g, waypoint_km, waypoint_bearing_deg
    d: vec4<f32>,
    // arcade, hover_state (0 off,1 takeoff,2 hold,3 land), balance, on_ground
    e: vec4<f32>,
    // layout (unused by the screen overlay)
    f: vec4<f32>,
};

const HUD: vec3<f32> = vec3<f32>(0.36, 1.0, 0.50);
/// Alpha multiplier for the dim layer (scale ticks, dots, ladder labels).
const DIM_ALPHA: f32 = 0.52;
const TAU: f32 = 6.28318530718;

// 5x7 font. Seven rows of five bits is 35 bits — one too many for a u32 — so
// rows 0-5 live in `FONT_LO` (as in the original packing: row `r` at bit `5·r`,
// MSB leftmost) and the bottom row in `FONT_HI`. Glyph ids: 0-9 digits, 10-35
// A-Z, 36 '.', 37 '-', 38 '+', 39 ':', 40 '/', 41 degree sign, 42 alpha.
const FONT_LO: array<u32, 43> = array<u32, 43>(
    0x239ACE2Eu, //  0 '0'
    0x08421184u, //  1 '1'
    0x1041062Eu, //  2 '2'
    0x2211105Fu, //  3 '3'
    0x05F928C2u, //  4 '4'
    0x2210FA1Fu, //  5 '5'
    0x231F4106u, //  6 '6'
    0x1082083Fu, //  7 '7'
    0x2317462Eu, //  8 '8'
    0x0417C62Eu, //  9 '9'
    0x231FC62Eu, // 10 'A'
    0x231F463Eu, // 11 'B'
    0x2308422Eu, // 12 'C'
    0x2318C63Eu, // 13 'D'
    0x210F421Fu, // 14 'E'
    0x210F421Fu, // 15 'F'
    0x231BC22Eu, // 16 'G'
    0x231FC631u, // 17 'H'
    0x0842108Eu, // 18 'I'
    0x24210847u, // 19 'J'
    0x254C5251u, // 20 'K'
    0x21084210u, // 21 'L'
    0x231AD771u, // 22 'M'
    0x2319D731u, // 23 'N'
    0x2318C62Eu, // 24 'O'
    0x210F463Eu, // 25 'P'
    0x2558C62Eu, // 26 'Q'
    0x254F463Eu, // 27 'R'
    0x0217420Fu, // 28 'S'
    0x0842109Fu, // 29 'T'
    0x2318C631u, // 30 'U'
    0x1518C631u, // 31 'V'
    0x375AC631u, // 32 'W'
    0x22A22A31u, // 33 'X'
    0x08422A31u, // 34 'Y'
    0x2082083Fu, // 35 'Z'
    0x18000000u, // 36 '.'
    0x000F8000u, // 37 '-'
    0x084F9080u, // 38 '+'
    0x18C03180u, // 39 ':'
    0x10820841u, // 40 '/'
    0x00064A4Cu, // 41 degree sign
    0x2718BC00u, // 42 alpha
);

/// Bottom row (row 6) of each glyph, five bits.
const FONT_HI: array<u32, 43> = array<u32, 43>(
    0x0000000Eu, //  0 '0'
    0x0000000Eu, //  1 '1'
    0x0000001Fu, //  2 '2'
    0x0000000Eu, //  3 '3'
    0x00000002u, //  4 '4'
    0x0000000Eu, //  5 '5'
    0x0000000Eu, //  6 '6'
    0x00000008u, //  7 '7'
    0x0000000Eu, //  8 '8'
    0x0000000Cu, //  9 '9'
    0x00000011u, // 10 'A'
    0x0000001Eu, // 11 'B'
    0x0000000Eu, // 12 'C'
    0x0000001Eu, // 13 'D'
    0x0000001Fu, // 14 'E'
    0x00000010u, // 15 'F'
    0x0000000Eu, // 16 'G'
    0x00000011u, // 17 'H'
    0x0000000Eu, // 18 'I'
    0x0000000Cu, // 19 'J'
    0x00000011u, // 20 'K'
    0x0000001Fu, // 21 'L'
    0x00000011u, // 22 'M'
    0x00000011u, // 23 'N'
    0x0000000Eu, // 24 'O'
    0x00000010u, // 25 'P'
    0x0000000Du, // 26 'Q'
    0x00000011u, // 27 'R'
    0x0000001Eu, // 28 'S'
    0x00000004u, // 29 'T'
    0x0000000Eu, // 30 'U'
    0x00000004u, // 31 'V'
    0x00000011u, // 32 'W'
    0x00000011u, // 33 'X'
    0x00000004u, // 34 'Y'
    0x0000001Fu, // 35 'Z'
    0x0000000Cu, // 36 '.'
    0x00000000u, // 37 '-'
    0x00000000u, // 38 '+'
    0x00000000u, // 39 ':'
    0x00000010u, // 40 '/'
    0x00000000u, // 41 degree sign
    0x0000000Du, // 42 alpha
);

/// Bank scale ticks, degrees of bank from level, left to right.
const BANK_TICKS: array<f32, 11> = array<f32, 11>(
    -60.0, -45.0, -30.0, -20.0, -10.0, 0.0, 10.0, 20.0, 30.0, 45.0, 60.0,
);

// Letter glyph ids used by the fixed labels.
const CH_A: u32 = 10u;
const CH_B: u32 = 11u;
const CH_C: u32 = 12u;
const CH_D: u32 = 13u;
const CH_E: u32 = 14u;
const CH_F: u32 = 15u;
const CH_G: u32 = 16u;
const CH_H: u32 = 17u;
const CH_K: u32 = 20u;
const CH_L: u32 = 21u;
const CH_M: u32 = 22u;
const CH_Q: u32 = 26u;
const CH_N: u32 = 23u;
const CH_R: u32 = 27u;
const CH_S: u32 = 28u;
const CH_T: u32 = 29u;
const CH_V: u32 = 31u;
const CH_W: u32 = 32u;
const CH_DOT: u32 = 36u;
const CH_ALPHA: u32 = 42u;
const CH_DEG: u32 = 41u;

// --- drawing primitives ----------------------------------------------------

fn line(p: vec2<f32>, a: vec2<f32>, b: vec2<f32>, w: f32) -> f32 {
    let pa = p - a;
    let ba = b - a;
    let h = clamp(dot(pa, ba) / max(dot(ba, ba), 1e-6), 0.0, 1.0);
    return 1.0 - smoothstep(w, w + 1.0, length(pa - ba * h));
}

fn ring(p: vec2<f32>, c: vec2<f32>, r: f32, w: f32) -> f32 {
    return 1.0 - smoothstep(w, w + 1.0, abs(length(p - c) - r));
}

fn disc(p: vec2<f32>, c: vec2<f32>, r: f32) -> f32 {
    return 1.0 - smoothstep(r - 0.8, r + 0.8, length(p - c));
}

fn sd_box(p: vec2<f32>, c: vec2<f32>, half: vec2<f32>) -> f32 {
    let d = abs(p - c) - half;
    return length(max(d, vec2<f32>(0.0))) + min(max(d.x, d.y), 0.0);
}

fn box_outline(p: vec2<f32>, c: vec2<f32>, half: vec2<f32>, w: f32) -> f32 {
    return 1.0 - smoothstep(w, w + 1.0, abs(sd_box(p, c, half)));
}

fn tri(p: vec2<f32>, a: vec2<f32>, b: vec2<f32>, c: vec2<f32>, w: f32) -> f32 {
    return max(line(p, a, b, w), max(line(p, b, c, w), line(p, c, a, w)));
}

// --- 5x7 text --------------------------------------------------------------

fn glyph_at(p: vec2<f32>, org: vec2<f32>, s: f32, id: u32) -> f32 {
    if (id >= 43u) {
        return 0.0;
    }
    let q = (p - org) / s;
    if (q.x < 0.0 || q.x >= 5.0 || q.y < 0.0 || q.y >= 7.0) {
        return 0.0;
    }
    let col = u32(q.x);
    let row = u32(q.y);
    let lo = (FONT_LO[id] >> (row * 5u)) & 31u;
    let bits = select(lo, FONT_HI[id], row == 6u);
    return f32((bits >> (4u - col)) & 1u);
}

/// Unsigned integer, `nd` zero-padded digits.
fn draw_u(p: vec2<f32>, org: vec2<f32>, s: f32, value: f32, nd: u32) -> f32 {
    var acc = 0.0;
    let v = max(value, 0.0);
    for (var i = 0u; i < nd; i = i + 1u) {
        let pw = pow(10.0, f32(nd - 1u - i));
        let d = u32(floor(v / pw + 0.001)) % 10u;
        let o = org + vec2<f32>(f32(i) * 6.0 * s, 0.0);
        acc = max(acc, glyph_at(p, o, s, d));
    }
    return acc;
}

/// Integer with a leading '-' when negative (or '+' when `plus`).
fn draw_i(p: vec2<f32>, org: vec2<f32>, s: f32, value: f32, nd: u32, plus: bool) -> f32 {
    var acc = 0.0;
    var dx = 0.0;
    let neg = value < -0.5;
    if (neg) {
        acc = max(acc, glyph_at(p, org, s, 37u));
        dx = 6.0 * s;
    } else if (plus) {
        acc = max(acc, glyph_at(p, org, s, 38u));
        dx = 6.0 * s;
    }
    acc = max(acc, draw_u(p, org + vec2<f32>(dx, 0.0), s, abs(value), nd));
    return acc;
}

/// Fixed-point decimal: `nd` integer digits then `.` and `ndec` fraction digits.
fn draw_dec(p: vec2<f32>, org: vec2<f32>, s: f32, value: f32, nd: u32, ndec: u32, signed: bool) -> f32 {
    var acc = 0.0;
    var x = org.x;
    let v = abs(value);
    if (signed) {
        let neg = value < -0.05;
        acc = max(acc, glyph_at(p, org, s, select(38u, 37u, neg)));
        x = x + 6.0 * s;
    }
    let whole = floor(v);
    acc = max(acc, draw_u(p, vec2<f32>(x, org.y), s, whole, nd));
    x = x + f32(nd) * 6.0 * s;
    acc = max(acc, glyph_at(p, vec2<f32>(x, org.y), s, CH_DOT));
    x = x + 6.0 * s;
    let frac = v - whole;
    for (var i = 0u; i < ndec; i = i + 1u) {
        let scaled = floor(frac * pow(10.0, f32(i + 1u)) + 0.001);
        let d = u32(scaled) % 10u;
        acc = max(acc, glyph_at(p, vec2<f32>(x + f32(i) * 6.0 * s, org.y), s, d));
    }
    return acc;
}

/// Digits drawn in the roll-rotated ladder frame (`rc`/`rs` = cos/sin of roll),
/// so pitch-ladder labels stay parallel to their rungs.
fn digits_ladder(
    p: vec2<f32>,
    c: vec2<f32>,
    rc: f32,
    rs: f32,
    org: vec2<f32>,
    s: f32,
    value: f32,
    nd: u32,
) -> f32 {
    var acc = 0.0;
    let d = p - c;
    let e = vec2<f32>(d.x * rc + d.y * rs, -d.x * rs + d.y * rc);
    for (var i = 0u; i < nd; i = i + 1u) {
        let pw = pow(10.0, f32(nd - 1u - i));
        let dig = u32(floor(max(value, 0.0) / pw + 0.001)) % 10u;
        let o = org + vec2<f32>(f32(i) * 6.0 * s, 0.0);
        acc = max(acc, glyph_at(e, o, s, dig));
    }
    return acc;
}

// --- instrument dials ------------------------------------------------------

/// Ten-dot fine scale (10 kt / 100 ft per dot) with a needle at the value's
/// remainder inside one full turn.
fn dial(p: vec2<f32>, cc: vec2<f32>, S: f32, frac: f32) -> f32 {
    let r = 0.078 * S;
    var acc = 0.0;
    for (var i = 0; i < 10; i = i + 1) {
        let a = f32(i) * 0.62831853;
        let d = vec2<f32>(sin(a), -cos(a));
        acc = max(acc, disc(p, cc + d * r, 0.0036 * S));
    }
    let na = frac * TAU;
    let nd = vec2<f32>(sin(na), -cos(na));
    return max(acc, line(p, cc + nd * (r * 0.30), cc + nd * (r * 0.96), 1.2));
}

// --- layout ----------------------------------------------------------------

fn wrap180(deg: f32) -> f32 {
    return (deg + 180.0) - floor((deg + 180.0) / 360.0) * 360.0 - 180.0;
}

/// Draw the AH-64 IHADSS symbology for `u` at pixel position `p`.
fn sym(u: Ah64Params, p: vec2<f32>, size: vec2<f32>) -> vec4<f32> {
    let S = size.y;
    let c = size * 0.5;

    let cas = u.a.x;
    let alt = u.a.y;
    let vs = u.a.z;
    let ralt = u.a.w;
    let hdg = u.b.x;
    let pitch = u.b.y;
    let roll = u.b.z;
    let coll = u.b.w;
    let tq = u.c.x;
    let nr = u.c.y;
    let fpm_x = u.c.z;
    let fpm_y = u.c.w;
    let gs = u.d.x;
    let g = u.d.y;
    let wp_km = u.d.z;
    let wp_brg = u.d.w;
    let arcade = u.e.x;
    let hover = u.e.y;
    let balance = u.e.z;
    let on_ground = u.e.w;

    // Screen pixels per degree of pitch/offset.
    let ppd = S * 0.0150;
    let ts = 0.0024 * S;
    let ds = 0.0032 * S;
    let lh = 9.0 * ts;

    var bright = 0.0;
    var dim = 0.0;

    // --- IHADSS monocle ring ------------------------------------------------
    // A faint circular field, so the overlay reads as the helmet display
    // rather than a fixed HUD glass.
    dim = max(dim, ring(p, c, 0.305 * S, 0.9) * 0.34);
    for (var i = 0; i < 12; i = i + 1) {
        let a = f32(i) * 0.5235988;
        let d = vec2<f32>(sin(a), -cos(a));
        if (i % 3 == 0) {
            dim = max(dim, line(p, c + d * 0.305 * S, c + d * 0.322 * S, 1.0));
        }
    }

    // --- heading tape -------------------------------------------------------
    let tape_y = c.y - 0.405 * S;
    let tape_hw = 0.210 * S;
    let ptd = 0.0070 * S;
    let hdg_i = floor(hdg / 5.0 + 0.5) * 5.0;
    for (var k: i32 = -9; k <= 9; k = k + 1) {
        let deg = hdg_i + f32(k) * 5.0;
        let dx = (deg - hdg) * ptd;
        if (abs(dx) < tape_hw) {
            let major = abs(deg - floor(deg / 10.0) * 10.0) < 0.01;
            let x = c.x + dx;
            let y1 = tape_y + select(0.007, 0.012, major) * S;
            dim = max(dim, line(p, vec2<f32>(x, tape_y + 0.003 * S), vec2<f32>(x, y1), 1.0));
            if (major) {
                let shown = (deg + 360.0) - floor((deg + 360.0) / 360.0) * 360.0;
                let lw = 3.0 * 6.0 * ts;
                dim = max(dim, draw_u(p, vec2<f32>(x - lw * 0.5, tape_y - 0.028 * S), ts, shown, 3));
            }
        }
    }
    let hdg_box = vec2<f32>(c.x, tape_y - 0.011 * S);
    let hdg_w = 3.0 * 6.0 * ts;
    bright = max(bright, draw_u(p, vec2<f32>(hdg_box.x - hdg_w * 0.5, hdg_box.y - 3.5 * ts), ts, hdg - floor(hdg / 360.0) * 360.0, 3));
    bright = max(bright, box_outline(p, hdg_box, vec2<f32>(hdg_w * 0.5 + 0.004 * S, 0.011 * S), 1.1));
    bright = max(bright, line(p, vec2<f32>(c.x - 0.007 * S, tape_y + 0.018 * S), vec2<f32>(c.x, tape_y + 0.010 * S), 1.1));
    bright = max(bright, line(p, vec2<f32>(c.x, tape_y + 0.010 * S), vec2<f32>(c.x + 0.007 * S, tape_y + 0.018 * S), 1.1));

    // --- earth-referenced horizon and pitch ladder --------------------------
    let rc = cos(radians(roll));
    let rs = sin(radians(roll));
    let dpx = p - c;
    let ex = dpx.x * rc + dpx.y * rs;
    let ey = -dpx.x * rs + dpx.y * rc - pitch * ppd;

    if (abs(ex) < 0.190 * S) {
        bright = max(bright, 1.0 - smoothstep(0.9, 1.9, abs(ey)));
    }
    for (var i: i32 = -8; i <= 8; i = i + 1) {
        if (i == 0) { continue; }
        let pd = f32(i) * 5.0;
        let ry = -pd * ppd;
        let on = 1.0 - smoothstep(0.5, 1.7, abs(ey - ry));
        if (on > 0.0) {
            let half = select(0.060, 0.082, i % 2 == 0) * S;
            let gap = 0.012 * S;
            let ax = abs(ex);
            let seg = select(0.0, 1.0, ax > gap && ax < half) * on;
            bright = max(bright, seg);
            if (i % 2 == 0) {
                let lx = -half - 0.020 * S;
                let ly = ry - 3.5 * ts;
                dim = max(dim, digits_ladder(p, c, rc, rs, vec2<f32>(lx, ly), ts, abs(pd), 2));
            }
        }
    }

    // --- waterline / flight-path marker -------------------------------------
    let wl_a = vec2<f32>(c.x, c.y - 0.016 * S);
    let wl_l = vec2<f32>(c.x - 0.013 * S, c.y + 0.004 * S);
    let wl_r = vec2<f32>(c.x + 0.013 * S, c.y + 0.004 * S);
    bright = max(bright, tri(p, wl_a, wl_l, wl_r, 1.1));
    bright = max(bright, line(p, vec2<f32>(c.x, c.y + 0.004 * S), vec2<f32>(c.x, c.y + 0.020 * S), 1.1));

    let fpm = c + vec2<f32>(fpm_x * ppd, fpm_y * ppd);
    bright = max(bright, ring(p, fpm, 0.0105 * S, 1.2));
    bright = max(bright, line(p, fpm + vec2<f32>(-0.023 * S, 0.0), fpm + vec2<f32>(-0.011 * S, 0.0), 1.2));
    bright = max(bright, line(p, fpm + vec2<f32>(0.011 * S, 0.0), fpm + vec2<f32>(0.023 * S, 0.0), 1.2));
    bright = max(bright, line(p, fpm + vec2<f32>(0.0, -0.0105 * S), fpm + vec2<f32>(0.0, -0.018 * S), 1.2));

    // --- waypoint -----------------------------------------------------------
    // The waypoint is drawn as a diamond on the heading-relative bearing; the
    // distance is fixed in the demo (no nav database yet).
    let rel = clamp(wrap180(wp_brg - hdg), -60.0, 60.0);
    let wp = vec2<f32>(c.x + rel * ppd * 0.8, c.y - 0.205 * S);
    let wd = 0.010 * S;
    bright = max(bright, line(p, wp + vec2<f32>(0.0, -wd), wp + vec2<f32>(wd, 0.0), 1.1));
    bright = max(bright, line(p, wp + vec2<f32>(wd, 0.0), wp + vec2<f32>(0.0, wd), 1.1));
    bright = max(bright, line(p, wp + vec2<f32>(0.0, wd), wp + vec2<f32>(-wd, 0.0), 1.1));
    bright = max(bright, line(p, wp + vec2<f32>(-wd, 0.0), wp + vec2<f32>(0.0, -wd), 1.1));
    bright = max(bright, glyph_at(p, wp + vec2<f32>(-16.0 * ts, 12.0 * ts), ts, CH_W));
    bright = max(bright, glyph_at(p, wp + vec2<f32>(-10.0 * ts, 12.0 * ts), ts, 0u));
    bright = max(bright, glyph_at(p, wp + vec2<f32>(-4.0 * ts, 12.0 * ts), ts, 1u));
    bright = max(bright, draw_dec(p, wp + vec2<f32>(6.0 * ts, 12.0 * ts), ts, wp_km, 2, 1, false));

    // --- speed box (left) ---------------------------------------------------
    let spd_c = vec2<f32>(c.x - 0.305 * S, c.y - 0.130 * S);
    let spd_h = vec2<f32>(0.070 * S, 0.043 * S);
    bright = max(bright, box_outline(p, spd_c, spd_h, 1.2));
    bright = max(bright, draw_u(p, spd_c + vec2<f32>(-0.038 * S, -0.014 * S), ds, cas, 3));
    dim = max(dim, glyph_at(p, spd_c + vec2<f32>(-0.020 * S, 0.016 * S), ts, CH_K));
    dim = max(dim, glyph_at(p, spd_c + vec2<f32>(-0.014 * S, 0.016 * S), ts, CH_T));
    dim = max(dim, glyph_at(p, spd_c + vec2<f32>(-0.008 * S, 0.016 * S), ts, CH_S));

    // --- altitude box (right) ----------------------------------------------
    let alt_c = vec2<f32>(c.x + 0.305 * S, c.y - 0.130 * S);
    let alt_h = vec2<f32>(0.078 * S, 0.043 * S);
    bright = max(bright, box_outline(p, alt_c, alt_h, 1.2));
    bright = max(bright, draw_u(p, alt_c + vec2<f32>(-0.052 * S, -0.014 * S), ds, alt, 5));
    dim = max(dim, glyph_at(p, alt_c + vec2<f32>(-0.016 * S, 0.016 * S), ts, CH_F));
    dim = max(dim, glyph_at(p, alt_c + vec2<f32>(-0.010 * S, 0.016 * S), ts, CH_T));
    if (ralt >= 0.0) {
        bright = max(bright, glyph_at(p, alt_c + vec2<f32>(-0.050 * S, 0.016 * S), ts, CH_R));
        bright = max(bright, draw_u(p, alt_c + vec2<f32>(-0.020 * S, 0.016 * S), ts, ralt, 4));
    }

    // --- vertical speed (right, outside the altitude box) -------------------
    let vs_x = c.x + 0.415 * S;
    let vs_y = c.y - 0.130 * S;
    dim = max(dim, line(p, vec2<f32>(vs_x, vs_y - 0.070 * S), vec2<f32>(vs_x, vs_y + 0.070 * S), 0.8));
    let vs_n = clamp(vs / 4000.0, -1.0, 1.0);
    let vs_tip = vec2<f32>(vs_x, vs_y - vs_n * 0.070 * S);
    bright = max(bright, line(p, vec2<f32>(vs_x, vs_y), vs_tip, 1.6));
    bright = max(bright, draw_i(p, vec2<f32>(vs_x + 8.0 * ts, vs_y - 0.085 * S), ts, vs, 4, false));

    // --- torque / rotor / collective (bottom) ------------------------------
    let tq_y = c.y + 0.315 * S;
    let tq_x = c.x - 0.250 * S;
    let tq_w = 0.160 * S;
    dim = max(dim, box_outline(p, vec2<f32>(tq_x + tq_w * 0.5, tq_y), vec2<f32>(tq_w * 0.5, 0.012 * S), 1.0));
    let tq_fill = clamp(tq / 120.0, 0.0, 1.0);
    bright = max(bright, line(p, vec2<f32>(tq_x + 1.0, tq_y), vec2<f32>(tq_x + tq_w * tq_fill, tq_y), 2.0));
    bright = max(bright, glyph_at(p, vec2<f32>(tq_x - 18.0 * ts, tq_y - 3.5 * ts), ts, CH_T));
    bright = max(bright, glyph_at(p, vec2<f32>(tq_x - 12.0 * ts, tq_y - 3.5 * ts), ts, CH_Q));
    bright = max(bright, draw_u(p, vec2<f32>(tq_x + tq_w + 8.0 * ts, tq_y - 3.5 * ts), ts, tq, 3));
    dim = max(dim, glyph_at(p, vec2<f32>(tq_x + tq_w + 30.0 * ts, tq_y - 3.5 * ts), ts, CH_DEG));

    let nr_x = c.x - 0.250 * S;
    let nr_y = tq_y + 0.045 * S;
    bright = max(bright, glyph_at(p, vec2<f32>(nr_x - 12.0 * ts, nr_y - 3.5 * ts), ts, CH_N));
    bright = max(bright, glyph_at(p, vec2<f32>(nr_x - 6.0 * ts, nr_y - 3.5 * ts), ts, CH_R));
    bright = max(bright, draw_u(p, vec2<f32>(nr_x, nr_y - 3.5 * ts), ts, nr, 3));
    dim = max(dim, glyph_at(p, vec2<f32>(nr_x + 22.0 * ts, nr_y - 3.5 * ts), ts, CH_DEG));

    let coll_x = c.x + 0.075 * S;
    bright = max(bright, glyph_at(p, vec2<f32>(coll_x, nr_y - 3.5 * ts), ts, CH_C)); // C
    bright = max(bright, glyph_at(p, vec2<f32>(coll_x + 6.0 * ts, nr_y - 3.5 * ts), ts, 24u)); // O
    bright = max(bright, draw_u(p, vec2<f32>(coll_x + 18.0 * ts, nr_y - 3.5 * ts), ts, coll, 3));
    dim = max(dim, glyph_at(p, vec2<f32>(coll_x + 40.0 * ts, nr_y - 3.5 * ts), ts, CH_DEG));

    // --- annunciators -------------------------------------------------------
    let ax = c.x - 0.335 * S;
    var ay = c.y - 0.290 * S;
    if (arcade > 0.5) {
        bright = max(bright, glyph_at(p, vec2<f32>(ax, ay), ts, CH_A));
        bright = max(bright, glyph_at(p, vec2<f32>(ax + 6.0 * ts, ay), ts, CH_R));
        bright = max(bright, glyph_at(p, vec2<f32>(ax + 12.0 * ts, ay), ts, CH_C));
        ay = ay + lh;
    }
    if (balance > 0.5) {
        bright = max(bright, glyph_at(p, vec2<f32>(ax, ay), ts, CH_B));
        bright = max(bright, glyph_at(p, vec2<f32>(ax + 6.0 * ts, ay), ts, CH_A));
        bright = max(bright, glyph_at(p, vec2<f32>(ax + 12.0 * ts, ay), ts, CH_L));
        ay = ay + lh;
    }
    if (hover > 1.5) {
        bright = max(bright, glyph_at(p, vec2<f32>(ax, ay), ts, CH_H));
        bright = max(bright, glyph_at(p, vec2<f32>(ax + 6.0 * ts, ay), ts, 24u));
        bright = max(bright, glyph_at(p, vec2<f32>(ax + 12.0 * ts, ay), ts, CH_V));
        bright = max(bright, glyph_at(p, vec2<f32>(ax + 18.0 * ts, ay), ts, CH_E));
        bright = max(bright, glyph_at(p, vec2<f32>(ax + 24.0 * ts, ay), ts, CH_R));
    } else if (hover > 0.5) {
        bright = max(bright, glyph_at(p, vec2<f32>(ax, ay), ts, CH_V));
        bright = max(bright, glyph_at(p, vec2<f32>(ax + 6.0 * ts, ay), ts, CH_T));
        bright = max(bright, glyph_at(p, vec2<f32>(ax + 12.0 * ts, ay), ts, 24u));
        bright = max(bright, glyph_at(p, vec2<f32>(ax + 18.0 * ts, ay), ts, CH_L));
    }
    if (on_ground > 0.5) {
        bright = max(bright, glyph_at(p, vec2<f32>(ax, c.y - 0.360 * S), ts, CH_G));
        bright = max(bright, glyph_at(p, vec2<f32>(ax + 6.0 * ts, c.y - 0.360 * S), ts, CH_N));
        bright = max(bright, glyph_at(p, vec2<f32>(ax + 12.0 * ts, c.y - 0.360 * S), ts, CH_D));
    }

    // Small "IHADSS" tag at the bottom right.
    dim = max(dim, glyph_at(p, vec2<f32>(c.x + 0.205 * S, c.y + 0.250 * S), ts, 18u));
    dim = max(dim, glyph_at(p, vec2<f32>(c.x + 0.211 * S, c.y + 0.250 * S), ts, CH_H));
    dim = max(dim, glyph_at(p, vec2<f32>(c.x + 0.217 * S, c.y + 0.250 * S), ts, CH_A));
    dim = max(dim, glyph_at(p, vec2<f32>(c.x + 0.223 * S, c.y + 0.250 * S), ts, CH_D));
    dim = max(dim, glyph_at(p, vec2<f32>(c.x + 0.229 * S, c.y + 0.250 * S), ts, CH_S));
    dim = max(dim, glyph_at(p, vec2<f32>(c.x + 0.235 * S, c.y + 0.250 * S), ts, CH_S));

    let a = max(bright, dim * DIM_ALPHA);
    if (a < 0.004) {
        return vec4<f32>(0.0);
    }
    return vec4<f32>(HUD, clamp(a, 0.0, 1.0));
}
