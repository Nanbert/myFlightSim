// F-22A head-up display, nav mode — symbology layout.
//
// Shared by the full-screen `UiMaterial` overlay (`f22_hud.wesl`) and the
// combiner-glass mesh material (`f22_hud_glass.wesl`). Everything is drawn
// procedurally from the uniform alone, so every element stays sharp at any
// resolution. The layout follows the HUD chapter of the TMS F-22A manual:
//
//   - heading tape with the current heading boxed, and a bank scale/pointer
//   - earth-stabilised pitch ladder and horizon, waterline marker at the
//     boresight, flight path marker at the ground-referenced flight path
//   - KCAS and barometric-altitude dials with fine-scale needles (10 kt / 100 ft
//     per dot), vertical-speed reference arc, accel/decel caret
//   - left stack: groundspeed, true airspeed, Mach, G, AoA, peak G
//   - right stack: vertical speed, radar altitude
//   - wind aloft, and the gear-down approach AoA bar
//
// Everything is laid out in fractions of the screen height `S`, centred on the
// screen, so the HUD keeps its proportions at any window shape.


struct HudParams {
    // cas_kts, alt_ft, vs_fpm, pitch_deg
    a: vec4<f32>,
    // roll_deg, heading_deg, alpha_deg, beta_deg
    b: vec4<f32>,
    // mach, g, max_g, gs_kts
    c: vec4<f32>,
    // tas_kts, radar_alt_ft, fpm_x_deg, fpm_y_deg
    d: vec4<f32>,
    // speed_trend_kt_s, wind_from_deg, wind_kt, gear_pos
    e: vec4<f32>,
    // Combiner layout only: xy = virtual screen size (px-like units), z = a
    // downward shift applied to the layout so it centres on the glass. Unused
    // by the screen overlay, which lays out against the real viewport.
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
const CH_G: u32 = 16u;
const CH_K: u32 = 20u;
const CH_M: u32 = 22u;
const CH_R: u32 = 27u;
const CH_S: u32 = 28u;
const CH_T: u32 = 29u;
const CH_V: u32 = 31u;
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

/// Draw the symbology for `u` at pixel position `p` on a `size`-sized surface.
fn sym(u: HudParams, p: vec2<f32>, size: vec2<f32>) -> vec4<f32> {
    let S = size.y;
    let c = size * 0.5;

    let cas = u.a.x;
    let alt = u.a.y;
    let vs = u.a.z;
    let pitch = u.a.w;
    let roll = u.b.x;
    let hdg = u.b.y;
    let alpha = u.b.z;
    let beta = u.b.w;
    let mach = u.c.x;
    let g = u.c.y;
    let gmax = u.c.z;
    let gs = u.c.w;
    let tas = u.d.x;
    let ralt = u.d.y;
    let fpm_x = u.d.z;
    let fpm_y = u.d.w;
    let trend = u.e.x;
    let wind_from = u.e.y;
    let wind_kt = u.e.z;
    let gear = u.e.w;

    // Screen pixels per degree of pitch/offset.
    let ppd = S * 0.0155;
    // Small text (labels and stack values), and the larger dial numerals.
    let ts = 0.0023 * S;
    let ds = 0.0031 * S;
    let lh = 9.0 * ts; // one text line plus a row of leading

    var bright = 0.0;
    var dim = 0.0;

    // --- bank scale and pointer (top arc) ----------------------------------
    let rb = 0.315 * S;
    for (var i = 0; i < 11; i = i + 1) {
        let bd = BANK_TICKS[i];
        let dir = vec2<f32>(sin(radians(bd)), -cos(radians(bd)));
        let major = bd == 0.0 || abs(bd) >= 30.0;
        let len = select(0.006, 0.011, major) * S;
        dim = max(dim, line(p, c + dir * rb, c + dir * (rb - len), 1.1));
    }
    let pdir = vec2<f32>(sin(radians(roll)), -cos(radians(roll)));
    let ptang = vec2<f32>(-pdir.y, pdir.x);
    let pr = 0.315 * S;
    let ptip = c + pdir * (pr - 0.024 * S);
    let pb1 = c + pdir * (pr - 0.006 * S) + ptang * (0.006 * S);
    let pb2 = c + pdir * (pr - 0.006 * S) - ptang * (0.006 * S);
    bright = max(bright, tri(p, ptip, pb1, pb2, 1.1));
    // Slip ball: slides along the pointer with lateral acceleration.
    let slip = clamp(beta * 0.08, -1.0, 1.0);
    bright = max(bright, disc(p, c + pdir * (pr - 0.040 * S) + ptang * (slip * 0.020 * S), 0.0026 * S));

    // --- heading tape -------------------------------------------------------
    let tape_y = c.y - 0.400 * S;
    let tape_hw = 0.200 * S;
    // Screen px per degree of heading. Wide enough that 10° labels (3 digits at
    // `ts`) don't crowd each other.
    let ptd = 0.0072 * S;
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
    // Current heading, boxed, and the centre caret.
    let hdg_box = vec2<f32>(c.x, tape_y - 0.011 * S);
    let hdg_w = 3.0 * 6.0 * ts;
    bright = max(bright, draw_u(p, vec2<f32>(hdg_box.x - hdg_w * 0.5, hdg_box.y - 3.5 * ts), ts, hdg - floor(hdg / 360.0) * 360.0, 3));
    bright = max(bright, box_outline(p, hdg_box, vec2<f32>(hdg_w * 0.5 + 0.004 * S, 0.011 * S), 1.1));
    bright = max(bright, line(p, vec2<f32>(c.x - 0.007 * S, tape_y + 0.018 * S), vec2<f32>(c.x, tape_y + 0.010 * S), 1.1));
    bright = max(bright, line(p, vec2<f32>(c.x, tape_y + 0.010 * S), vec2<f32>(c.x + 0.007 * S, tape_y + 0.018 * S), 1.1));

    // --- wind aloft ---------------------------------------------------------
    if (wind_kt > 1.0) {
        let wy = tape_y + 0.034 * S;
        let toward = wind_from + 180.0;
        let wdir = vec2<f32>(sin(radians(toward - hdg)), -cos(radians(toward - hdg)));
        let ar0 = vec2<f32>(c.x + 0.140 * S, wy);
        let ar1 = ar0 + wdir * 0.026 * S;
        let wperp = vec2<f32>(-wdir.y, wdir.x);
        bright = max(bright, line(p, ar0, ar1, 1.3));
        bright = max(bright, line(p, ar1, ar1 - wdir * 0.008 * S + wperp * 0.005 * S, 1.3));
        bright = max(bright, line(p, ar1, ar1 - wdir * 0.008 * S - wperp * 0.005 * S, 1.3));
        let wx = c.x + 0.172 * S;
        bright = max(bright, draw_u(p, vec2<f32>(wx, wy - 1.5 * lh), ts, wind_from, 3));
        bright = max(bright, glyph_at(p, vec2<f32>(wx + 3.0 * 6.0 * ts, wy - 1.5 * lh), ts, CH_DEG));
        bright = max(bright, draw_u(p, vec2<f32>(wx, wy + 0.2 * lh), ts, wind_kt, 2));
        bright = max(bright, glyph_at(p, vec2<f32>(wx + 2.0 * 6.0 * ts, wy + 0.2 * lh), ts, CH_K));
        bright = max(bright, glyph_at(p, vec2<f32>(wx + 3.0 * 6.0 * ts, wy + 0.2 * lh), ts, CH_T));
    }

    // --- pitch ladder, horizon ---------------------------------------------
    let rc = cos(radians(roll));
    let rs = sin(radians(roll));
    let dpx = p - c;
    let ex = dpx.x * rc + dpx.y * rs;
    let ey = -dpx.x * rs + dpx.y * rc - pitch * ppd;

    // Horizon. Kept short of the speed/altitude dials on either side.
    if (abs(ex) < 0.202 * S) {
        bright = max(bright, 1.0 - smoothstep(0.9, 1.9, abs(ey)));
    }
    for (var i: i32 = -12; i <= 12; i = i + 1) {
        if (i == 0) {
            continue;
        }
        let pd = f32(i) * 5.0;
        let ry = -pd * ppd;
        let on = 1.0 - smoothstep(0.5, 1.7, abs(ey - ry));
        if (on > 0.0) {
            let half = select(0.082, 0.104, i % 2 == 0) * S;
            let gap = 0.014 * S;
            let ax = abs(ex);
            var seg = select(0.0, 1.0, ax > gap && ax < half) * on;
            if (pd < 0.0) {
                let dash = fract(ex / (0.013 * S));
                seg = seg * select(0.0, 1.0, dash < 0.60);
            }
            bright = max(bright, seg);
            if (i % 2 == 0) {
                let lx = -half - 0.022 * S;
                let ly = ry - 3.5 * ts;
                dim = max(dim, digits_ladder(p, c, rc, rs, vec2<f32>(lx, ly), ts, abs(pd), 2));
            }
        }
    }

    // --- waterline marker (nose reference, at the boresight) ----------------
    let wl_a = vec2<f32>(c.x, c.y - 0.016 * S);
    let wl_l = vec2<f32>(c.x - 0.013 * S, c.y + 0.004 * S);
    let wl_r = vec2<f32>(c.x + 0.013 * S, c.y + 0.004 * S);
    bright = max(bright, tri(p, wl_a, wl_l, wl_r, 1.1));
    bright = max(bright, line(p, vec2<f32>(c.x, c.y + 0.004 * S), vec2<f32>(c.x, c.y + 0.022 * S), 1.1));
    bright = max(bright, line(p, vec2<f32>(c.x - 0.007 * S, c.y + 0.017 * S), vec2<f32>(c.x + 0.007 * S, c.y + 0.017 * S), 1.1));

    // --- flight path marker (ground track + flight path angle) --------------
    let fpm = c + vec2<f32>(fpm_x * ppd, fpm_y * ppd);
    bright = max(bright, ring(p, fpm, 0.0105 * S, 1.2));
    bright = max(bright, line(p, fpm + vec2<f32>(-0.023 * S, 0.0), fpm + vec2<f32>(-0.011 * S, 0.0), 1.2));
    bright = max(bright, line(p, fpm + vec2<f32>(0.011 * S, 0.0), fpm + vec2<f32>(0.023 * S, 0.0), 1.2));
    bright = max(bright, line(p, fpm + vec2<f32>(0.0, -0.0105 * S), fpm + vec2<f32>(0.0, -0.018 * S), 1.2));

    // --- approach angle (gear down, target 12 deg AoA) ----------------------
    // Drawn just left of the FPM out to the side that says which way the AoA is
    // off target, so it never sits behind the waterline symbol.
    if (gear > 0.5) {
        let err = alpha - 12.0;
        if (abs(err) > 0.8) {
            let bx = fpm.x - 0.034 * S;
            let bar = min(abs(err) * ppd * 0.6, 0.032 * S) + 0.016 * S;
            let sgn = select(-1.0, 1.0, err < 0.0);
            let y0 = fpm.y + sgn * 0.010 * S;
            let y1 = fpm.y + sgn * bar;
            bright = max(bright, line(p, vec2<f32>(bx, y0), vec2<f32>(bx, y1), 1.3));
            bright = max(bright, line(p, vec2<f32>(bx, y0), vec2<f32>(bx + 0.010 * S, y0), 1.3));
        }
    }

    // --- speed and altitude dials ------------------------------------------
    let dial_y = c.y - 0.005 * S;
    let cas_c = vec2<f32>(c.x - 0.302 * S, dial_y);
    let alt_c = vec2<f32>(c.x + 0.302 * S, dial_y);
    let cas_mod = cas - floor(cas / 100.0) * 100.0;
    let alt_mod = alt - floor(alt / 1000.0) * 1000.0;
    dim = max(dim, dial(p, cas_c, S, cas_mod / 100.0));
    dim = max(dim, dial(p, alt_c, S, alt_mod / 1000.0));
    let cas_w = 3.0 * 6.0 * ds;
    let alt_w = 5.0 * 6.0 * ds;
    bright = max(bright, draw_u(p, vec2<f32>(cas_c.x - cas_w * 0.5, cas_c.y - 3.5 * ds), ds, cas, 3));
    bright = max(bright, draw_u(p, vec2<f32>(alt_c.x - alt_w * 0.5, alt_c.y - 3.5 * ds), ds, alt, 5));

    // Vertical-speed reference: arc around the altitude dial, full circle at
    // +/- 38,000 fpm (manual, item 5).
    let r_vs = 0.078 * S + 0.014 * S;
    dim = max(dim, ring(p, alt_c, r_vs, 0.7) * 0.55);
    let aoff = p - alt_c;
    if (abs(length(aoff) - r_vs) < 1.5) {
        let sweep = clamp(vs / 38000.0, -1.0, 1.0);
        // Clockwise from the top, folded into 0..1 so a full ±1 sweep works.
        var ang01 = atan2(aoff.x, -aoff.y) / TAU;
        if (ang01 < 0.0) {
            ang01 = ang01 + 1.0;
        }
        if (sweep >= 0.0) {
            if (ang01 <= sweep) {
                bright = max(bright, 1.0);
            }
        } else {
            if (ang01 >= 1.0 + sweep) {
                bright = max(bright, 1.0);
            }
        }
    }

    // Accel/decel caret beside the airspeed dial.
    if (abs(trend) > 0.4) {
        let ax = cas_c.x - 0.078 * S - 0.018 * S;
        let ay = cas_c.y;
        let s2 = 0.0065 * S;
        if (trend > 0.0) {
            bright = max(bright, tri(p, vec2<f32>(ax, ay - s2), vec2<f32>(ax - s2 * 0.8, ay + s2 * 0.5), vec2<f32>(ax + s2 * 0.8, ay + s2 * 0.5), 1.2));
        } else {
            bright = max(bright, tri(p, vec2<f32>(ax, ay + s2), vec2<f32>(ax - s2 * 0.8, ay - s2 * 0.5), vec2<f32>(ax + s2 * 0.8, ay - s2 * 0.5), 1.2));
        }
    }

    // --- left data stack: GS / T / M / G / alpha / peak G --------------------
    // Above the airspeed dial, so the two never overlap.
    let lx = cas_c.x - 0.075 * S;
    let vx = lx + 0.042 * S;
    var ly = dial_y - 0.111 * S - 5.0 * lh;
    bright = max(bright, glyph_at(p, vec2<f32>(lx, ly), ts, CH_G));
    bright = max(bright, glyph_at(p, vec2<f32>(lx + 6.0 * ts, ly), ts, CH_S));
    bright = max(bright, draw_u(p, vec2<f32>(vx + 6.0 * ts, ly), ts, gs, 3));
    ly = ly + lh;
    bright = max(bright, glyph_at(p, vec2<f32>(lx, ly), ts, CH_T));
    bright = max(bright, draw_u(p, vec2<f32>(vx + 6.0 * ts, ly), ts, tas, 3));
    ly = ly + lh;
    bright = max(bright, glyph_at(p, vec2<f32>(lx, ly), ts, CH_M));
    bright = max(bright, draw_dec(p, vec2<f32>(vx + 6.0 * ts, ly), ts, mach, 1, 2, false));
    ly = ly + lh;
    bright = max(bright, glyph_at(p, vec2<f32>(lx, ly), ts, CH_G));
    bright = max(bright, draw_dec(p, vec2<f32>(vx + 6.0 * ts, ly), ts, g, 1, 1, true));
    ly = ly + lh;
    bright = max(bright, glyph_at(p, vec2<f32>(lx, ly), ts, CH_ALPHA));
    bright = max(bright, draw_dec(p, vec2<f32>(vx + 6.0 * ts, ly), ts, alpha, 2, 1, true));
    ly = ly + lh;
    bright = max(bright, draw_dec(p, vec2<f32>(vx + 6.0 * ts, ly), ts, gmax, 1, 1, true));

    // --- right data stack: VV / R ------------------------------------------
    // Above the altitude dial, mirroring the left stack.
    let rx = alt_c.x - 0.075 * S;
    let rvx = rx + 0.042 * S;
    var ry = dial_y - 0.111 * S - lh;
    bright = max(bright, glyph_at(p, vec2<f32>(rx, ry), ts, CH_V));
    bright = max(bright, glyph_at(p, vec2<f32>(rx + 6.0 * ts, ry), ts, CH_V));
    bright = max(bright, draw_i(p, vec2<f32>(rvx + 6.0 * ts, ry), ts, vs, 4, false));
    ry = ry + lh;
    // Radar altitude is a four-digit readout and only useful near the ground:
    // blank it above 10,000 ft rather than wrap a fifth digit away.
    if (ralt >= 0.0) {
        bright = max(bright, glyph_at(p, vec2<f32>(rx, ry), ts, CH_R));
        bright = max(bright, draw_u(p, vec2<f32>(rvx + 6.0 * ts, ry), ts, ralt, 4));
    }

    let a = max(bright, dim * DIM_ALPHA);
    if (a < 0.004) {
        return vec4<f32>(0.0);
    }
    return vec4<f32>(HUD, clamp(a, 0.0, 1.0));
}
