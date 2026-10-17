// Boeing 737 primary flight display, drawn procedurally.
//
// One `UiMaterial` node per pilot, both fed the same uniform so the captain's
// and first officer's displays cannot disagree. The layout follows the 737 NG
// PFD: FMA row, attitude with flight directors, speed tape with the selected
// speed and Vr bugs, altitude tape with the selected-altitude bug, V/S scale,
// heading tape, and a bottom row with groundspeed (or flaps on the ground) and
// the barometric setting.
//
// Everything is laid out in fractions of the node height, so the two displays
// stay proportionate when the window narrows. The 5x7 font is the same packing
// the F-22 HUD uses.

import bevy_ui_render::ui_vertex_output::UiVertexOutput;

struct PfdParams {
    // cas_kts, alt_ft, vs_fpm, pitch_deg
    a: vec4<f32>,
    // roll_deg, heading_deg, selected_speed_kts, selected_alt_ft
    b: vec4<f32>,
    // selected_heading_deg, flap_position_norm, gs_kts, vr_kts
    c: vec4<f32>,
    // fd_pitch_deg, fd_bank_deg, baro_hpa, radio_alt_ft
    d: vec4<f32>,
    // engaged, ap_disc, on_ground, speed_is_mach
    e: vec4<f32>,
    // at_mode, lat_mode, vert_mode, mach
    f: vec4<f32>,
    // time_s, slip, tas_kts, unused
    g: vec4<f32>,
};

@group(1) @binding(0) var<uniform> u: PfdParams;

const SCREEN: vec3<f32> = vec3<f32>(0.010, 0.016, 0.013);
const BEZEL_DARK: vec3<f32> = vec3<f32>(0.020, 0.021, 0.023);
const BEZEL_LIGHT: vec3<f32> = vec3<f32>(0.105, 0.110, 0.120);
const TAPE_BG: vec3<f32> = vec3<f32>(0.085, 0.088, 0.092);
const WHITE: vec3<f32> = vec3<f32>(0.93, 0.96, 0.95);
const DIM: vec3<f32> = vec3<f32>(0.58, 0.64, 0.62);
const MAGENTA: vec3<f32> = vec3<f32>(0.96, 0.30, 0.86);
const GREEN: vec3<f32> = vec3<f32>(0.32, 0.95, 0.48);
const AMBER: vec3<f32> = vec3<f32>(1.00, 0.64, 0.12);
const BLACK: vec3<f32> = vec3<f32>(0.0, 0.0, 0.0);
const SKY_HORIZON: vec3<f32> = vec3<f32>(0.31, 0.55, 0.83);
const SKY_TOP: vec3<f32> = vec3<f32>(0.08, 0.22, 0.55);
const GND_HORIZON: vec3<f32> = vec3<f32>(0.55, 0.35, 0.16);
const GND_DEEP: vec3<f32> = vec3<f32>(0.27, 0.16, 0.06);

// 5x7 font. Seven rows of five bits is 35 bits — one too many for a u32 — so
// rows 0-5 live in `FONT_LO` (row `r` at bit `5·r`, MSB leftmost) and the
// bottom row in `FONT_HI`. Glyph ids: 0-9 digits, 10-35 A-Z, 36 '.', 37 '-',
// 38 '+', 39 ':', 40 '/', 41 degree sign, 42 alpha.
const FONT_LO: array<u32, 43> = array<u32, 43>(
    0x239ACE2Eu, 0x08421184u, 0x1041062Eu, 0x2211105Fu, 0x05F928C2u,
    0x2210FA1Fu, 0x231F4106u, 0x1082083Fu, 0x2317462Eu, 0x0417C62Eu,
    0x231FC62Eu, 0x231F463Eu, 0x2308422Eu, 0x2318C63Eu, 0x210F421Fu,
    0x210F421Fu, 0x231BC22Eu, 0x231FC631u, 0x0842108Eu, 0x24210847u,
    0x254C5251u, 0x21084210u, 0x231AD771u, 0x2319D731u, 0x2318C62Eu,
    0x210F463Eu, 0x2558C62Eu, 0x254F463Eu, 0x0217420Fu, 0x0842109Fu,
    0x2318C631u, 0x1518C631u, 0x375AC631u, 0x22A22A31u, 0x08422A31u,
    0x2082083Fu, 0x18000000u, 0x000F8000u, 0x084F9080u, 0x18C03180u,
    0x10820841u, 0x00064A4Cu, 0x2718BC00u,
);
const FONT_HI: array<u32, 43> = array<u32, 43>(
    0x0000000Eu, 0x0000000Eu, 0x0000001Fu, 0x0000000Eu, 0x00000002u,
    0x0000000Eu, 0x0000000Eu, 0x00000008u, 0x0000000Eu, 0x0000000Cu,
    0x00000011u, 0x0000001Eu, 0x0000000Eu, 0x0000001Eu, 0x0000001Fu,
    0x00000010u, 0x0000000Eu, 0x00000011u, 0x0000000Eu, 0x0000000Cu,
    0x00000011u, 0x0000001Fu, 0x00000011u, 0x00000011u, 0x0000000Eu,
    0x00000010u, 0x0000000Du, 0x00000011u, 0x0000001Eu, 0x00000004u,
    0x0000000Eu, 0x00000004u, 0x00000011u, 0x00000011u, 0x00000004u,
    0x0000001Fu, 0x0000000Cu, 0x00000000u, 0x00000000u, 0x00000000u,
    0x00000010u, 0x00000000u, 0x0000000Du,
);

// Letter glyph ids used by the fixed labels.
const CH_A: u32 = 10u;
const CH_C: u32 = 12u;
const CH_D: u32 = 13u;
const CH_E: u32 = 14u;
const CH_F: u32 = 15u;
const CH_G: u32 = 16u;
const CH_H: u32 = 17u;
const CH_I: u32 = 18u;
const CH_L: u32 = 21u;
const CH_M: u32 = 22u;
const CH_N: u32 = 23u;
const CH_O: u32 = 24u;
const CH_P: u32 = 25u;
const CH_R: u32 = 27u;
const CH_S: u32 = 28u;
const CH_T: u32 = 29u;
const CH_V: u32 = 31u;
const CH_W: u32 = 32u;
const CH_DOT: u32 = 36u;
const CH_SLASH: u32 = 40u;
const CH_SP: u32 = 99u;

const BANK_TICKS: array<f32, 11> = array<f32, 11>(
    -60.0, -45.0, -30.0, -20.0, -10.0, 0.0, 10.0, 20.0, 30.0, 45.0, 60.0,
);

// --- drawing primitives ----------------------------------------------------

fn line(p: vec2<f32>, a: vec2<f32>, b: vec2<f32>, w: f32) -> f32 {
    let pa = p - a;
    let ba = b - a;
    let h = clamp(dot(pa, ba) / max(dot(ba, ba), 1e-6), 0.0, 1.0);
    return 1.0 - smoothstep(w, w + 1.0, length(pa - ba * h));
}

fn box_outline(p: vec2<f32>, c: vec2<f32>, half: vec2<f32>, w: f32) -> f32 {
    let d = abs(p - c) - half;
    let sd = length(max(d, vec2<f32>(0.0))) + min(max(d.x, d.y), 0.0);
    return 1.0 - smoothstep(w, w + 1.0, abs(sd));
}

fn box_fill(p: vec2<f32>, c: vec2<f32>, half: vec2<f32>) -> f32 {
    let d = abs(p - c) - half;
    let sd = length(max(d, vec2<f32>(0.0))) + min(max(d.x, d.y), 0.0);
    return 1.0 - smoothstep(-1.0, 1.0, sd);
}

fn disc(p: vec2<f32>, c: vec2<f32>, r: f32) -> f32 {
    return 1.0 - smoothstep(r - 0.8, r + 0.8, length(p - c));
}

fn tri(p: vec2<f32>, a: vec2<f32>, b: vec2<f32>, c: vec2<f32>, w: f32) -> f32 {
    return max(line(p, a, b, w), max(line(p, b, c, w), line(p, c, a, w)));
}

fn rect_clip(p: vec2<f32>, c: vec2<f32>, half: vec2<f32>) -> f32 {
    let d = abs(p - c) - half;
    let sd = length(max(d, vec2<f32>(0.0))) + min(max(d.x, d.y), 0.0);
    return 1.0 - smoothstep(-1.0, 1.0, sd);
}

fn rounded_clip(p: vec2<f32>, c: vec2<f32>, half: vec2<f32>, r: f32) -> f32 {
    let d = abs(p - c) - half + vec2<f32>(r);
    let sd = length(max(d, vec2<f32>(0.0))) + min(max(d.x, d.y), 0.0) - r;
    return 1.0 - smoothstep(-1.0, 1.0, sd);
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
        acc = max(acc, glyph_at(p, org + vec2<f32>(f32(i) * 6.0 * s, 0.0), s, d));
    }
    return acc;
}

/// Integer with a leading '-' when negative (or '+' when `plus`).
fn draw_i(p: vec2<f32>, org: vec2<f32>, s: f32, value: f32, nd: u32, plus: bool) -> f32 {
    var acc = 0.0;
    var dx = 0.0;
    if (value < -0.5) {
        acc = max(acc, glyph_at(p, org, s, 37u));
        dx = 6.0 * s;
    } else if (plus) {
        acc = max(acc, glyph_at(p, org, s, 38u));
        dx = 6.0 * s;
    }
    return max(acc, draw_u(p, org + vec2<f32>(dx, 0.0), s, abs(value), nd));
}

/// A fixed-size word built from glyph ids. Unused slots are ignored.
fn draw_word(p: vec2<f32>, org: vec2<f32>, s: f32, ids: array<u32, 8>, n: u32) -> f32 {
    var acc = 0.0;
    for (var i = 0u; i < n; i = i + 1u) {
        if (ids[i] < 43u) {
            acc = max(acc, glyph_at(p, org + vec2<f32>(f32(i) * 6.0 * s, 0.0), s, ids[i]));
        }
    }
    return acc;
}

fn text_w(n: u32, s: f32) -> f32 {
    return (f32(n) * 6.0 - 1.0) * s;
}

fn center_x(cx: f32, n: u32, s: f32) -> f32 {
    return cx - text_w(n, s) * 0.5;
}

fn ndigits(value: f32) -> u32 {
    let v = u32(clamp(round(value), 0.0, 99999.0));
    if (v >= 10000u) { return 5u; }
    if (v >= 1000u) { return 4u; }
    if (v >= 100u) { return 3u; }
    if (v >= 10u) { return 2u; }
    return 1u;
}

fn draw_u_trim(p: vec2<f32>, org: vec2<f32>, s: f32, value: f32) -> f32 {
    return draw_u(p, org, s, value, ndigits(value));
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

fn wrap180(deg: f32) -> f32 {
    var d = (deg + 180.0) - floor((deg + 180.0) / 360.0) * 360.0 - 180.0;
    if (d <= -180.0) {
        d = d + 360.0;
    }
    return d;
}

// --- display ---------------------------------------------------------------

fn pfd(u: PfdParams, p: vec2<f32>, size: vec2<f32>) -> vec4<f32> {
    let W = size.x;
    let H = size.y;

    let cas = u.a.x;
    let alt = u.a.y;
    let vs = u.a.z;
    let pitch = u.a.w;
    let roll = u.b.x;
    let heading = u.b.y;
    let sel_speed = u.b.z;
    let sel_alt = u.b.w;
    let sel_hdg = u.c.x;
    let flap_norm = u.c.y;
    let gs = u.c.z;
    let vr = u.c.w;
    let engaged = u.e.x > 0.5;
    let disc_annun = u.e.y > 0.5;
    let on_ground = u.e.z > 0.5;
    let speed_is_mach = u.e.w > 0.5;

    let ts = 0.0040 * H;       // tape and label text
    let bs = 0.0070 * H;       // speed box
    let abs_ = 0.0056 * H;     // altitude box
    let hbs = 0.0062 * H;      // heading box
    let fma_s = 0.0038 * H;
    let bot_s = 0.0034 * H;

    // --- case -------------------------------------------------------------
    var col = SCREEN;
    let edge = min(min(p.x, W - p.x), min(p.y, H - p.y));
    if (edge < 0.008 * H) {
        col = BEZEL_DARK;
    } else if (edge < 0.013 * H) {
        col = BEZEL_LIGHT;
    } else if (edge < 0.018 * H) {
        col = BEZEL_DARK;
    }

    // --- attitude ---------------------------------------------------------
    let ac = vec2<f32>(0.50 * W, 0.425 * H);
    let ah = vec2<f32>(0.205 * W, 0.265 * H);
    let att_clip = rounded_clip(p, ac, ah, 0.035 * W);
    let ppd = ah.y / 20.0; // 20 deg of pitch reaches the top edge
    let q = p - ac;
    let rc = cos(radians(roll));
    let rs = sin(radians(roll));
    let ex = q.x * rc + q.y * rs;
    let ey = -q.x * rs + q.y * rc - pitch * ppd;

    if (att_clip > 0.0) {
        let skyt = clamp(-ey / (ah.y * 1.8), 0.0, 1.0);
        let gndt = clamp(ey / (ah.y * 1.8), 0.0, 1.0);
        let sky = mix(SKY_HORIZON, SKY_TOP, skyt);
        let gnd = mix(GND_HORIZON, GND_DEEP, gndt);
        col = mix(col, select(gnd, sky, ey < 0.0), att_clip);
    }

    var dim = 0.0;
    var bright = 0.0;
    var green = 0.0;
    var mag = 0.0;
    var amber = 0.0;
    var fill = 0.0; // black backgrounds, under the symbology
    var sym = 0.0;  // black aircraft reference, over the ladder

    // Horizon.
    bright = max(bright, (1.0 - smoothstep(0.7, 1.7, abs(ey))) * att_clip);

    // Pitch ladder: a line every 5°, heavier every 10°, dashed below. The
    // numbers sit just outside both ends of the 10° rungs.
    for (var i: i32 = -4; i <= 4; i = i + 1) {
        if (i == 0) {
            continue;
        }
        let pd = f32(i) * 5.0;
        let ry = -pd * ppd;
        let major = i % 2 == 0;
        let half = select(0.30, 0.55, major) * ah.x;
        let on = 1.0 - smoothstep(0.5, 1.7, abs(ey - ry));
        if (on > 0.0) {
            let gap = 0.11 * ah.x;
            let ax = abs(ex);
            var seg = select(0.0, 1.0, ax > gap && ax < half) * on;
            if (pd < 0.0) {
                let dash = fract(ex / (0.055 * ah.x));
                seg = seg * select(0.0, 1.0, dash < 0.6);
            }
            bright = max(bright, seg * att_clip);
        }
        if (major) {
            // The rung lives at e.y = pitch*ppd + ry in the rotated frame.
            let ly = ry + pitch * ppd - 3.5 * ts;
            let lx = half + 0.05 * ah.x;
            let text = text_w(2u, ts);
            bright = max(bright, digits_ladder(p, ac, rc, rs, vec2<f32>(-lx - text, ly), ts, abs(pd), 2u) * att_clip);
            bright = max(bright, digits_ladder(p, ac, rc, rs, vec2<f32>(lx, ly), ts, abs(pd), 2u) * att_clip);
        }
    }

    // Bank scale and pointer along the top of the attitude area.
    let rb = min(0.93 * ah.y, ah.x / 0.866);
    for (var i = 0; i < 11; i = i + 1) {
        let bd = BANK_TICKS[i];
        let dir = vec2<f32>(sin(radians(bd)), -cos(radians(bd)));
        let major = bd == 0.0 || abs(bd) >= 30.0;
        let len = select(0.035, 0.070, major) * ah.y;
        dim = max(dim, line(p, ac + dir * rb, ac + dir * (rb - len), 1.0) * att_clip);
    }
    let pdir = vec2<f32>(sin(radians(roll)), -cos(radians(roll)));
    let ptang = vec2<f32>(-pdir.y, pdir.x);
    let ptip = ac + pdir * (rb - 0.075 * ah.y);
    let pb1 = ac + pdir * (rb - 0.018 * ah.y) + ptang * (0.022 * ah.x);
    let pb2 = ac + pdir * (rb - 0.018 * ah.y) - ptang * (0.022 * ah.x);
    bright = max(bright, tri(p, ptip, pb1, pb2, 1.0) * att_clip);

    // Flight director bars, at the commanded pitch and bank.
    if (engaged && !disc_annun) {
        let fdy = ac.y + (pitch - u.d.x) * ppd;
        let hw = 0.62 * ah.x;
        let gap = 0.14 * ah.x;
        mag = max(mag, line(p, vec2<f32>(ac.x - hw, fdy), vec2<f32>(ac.x - gap, fdy), 2.0) * att_clip);
        mag = max(mag, line(p, vec2<f32>(ac.x + gap, fdy), vec2<f32>(ac.x + hw, fdy), 2.0) * att_clip);
        let fdx = ac.x + (u.d.y - roll) * ppd * 1.3;
        let vh = 0.55 * ah.y;
        mag = max(mag, line(p, vec2<f32>(fdx, ac.y - vh), vec2<f32>(fdx, ac.y - 0.11 * ah.y), 2.0) * att_clip);
        mag = max(mag, line(p, vec2<f32>(fdx, ac.y + 0.11 * ah.y), vec2<f32>(fdx, ac.y + vh), 2.0) * att_clip);
    }

    // Fixed aircraft reference: black wing bar with down-turned tips.
    let tip_dx = 0.40 * ah.x;
    let tip_dy = 0.075 * ah.y;
    sym = max(sym, line(p, vec2<f32>(ac.x - tip_dx, ac.y), vec2<f32>(ac.x + tip_dx, ac.y), 2.4) * att_clip);
    sym = max(sym, line(p, vec2<f32>(ac.x - tip_dx, ac.y), vec2<f32>(ac.x - tip_dx - 0.045 * ah.x, ac.y + tip_dy), 2.4) * att_clip);
    sym = max(sym, line(p, vec2<f32>(ac.x + tip_dx, ac.y), vec2<f32>(ac.x + tip_dx + 0.045 * ah.x, ac.y + tip_dy), 2.4) * att_clip);
    sym = max(sym, disc(p, ac, 0.018 * ah.x) * att_clip);

    // Slip/skid: the index sits just below the aircraft symbol.
    let slip = clamp(u.g.y * 0.5, -1.0, 1.0);
    let skid_c = vec2<f32>(ac.x, ac.y + 0.105 * ah.y);
    dim = max(dim, box_outline(p, skid_c, vec2<f32>(0.060 * ah.x, 0.022 * ah.y), 1.0) * att_clip);
    bright = max(bright, box_fill(p, skid_c + vec2<f32>(slip * 0.045 * ah.x, 0.0), vec2<f32>(0.010 * ah.x, 0.014 * ah.y)) * att_clip);

    // --- speed tape -------------------------------------------------------
    let tape_top = 0.110 * H;
    let tape_bot = 0.820 * H;
    let tape_cy = (tape_top + tape_bot) * 0.5;
    let sp_x0 = 0.030 * W;
    let sp_x1 = 0.240 * W;
    let sp_ppk = (tape_bot - tape_top) / 100.0; // ±50 kt
    let sp_box = vec2<f32>(0.072 * W, 0.040 * H);
    let sp_box_c = vec2<f32>(0.095 * W, tape_cy);

    // Grey tape strip behind the scale, as on the real display.
    col = mix(col, TAPE_BG, rect_clip(
        p,
        vec2<f32>((sp_x0 + sp_x1) * 0.5, 0.485 * H),
        vec2<f32>((sp_x1 - sp_x0) * 0.5, 0.390 * H),
    ));

    fill = max(fill, box_fill(p, sp_box_c, sp_box));

    let sp_base = floor((cas - 50.0) / 10.0) * 10.0;
    for (var i: i32 = 0; i <= 20; i = i + 1) {
        let v = sp_base + f32(i) * 10.0;
        if (v < 0.0) {
            continue;
        }
        let y = tape_cy - (v - cas) * sp_ppk;
        if (y < tape_top || y > tape_bot) {
            continue;
        }
        if (abs(y - tape_cy) < sp_box.y + 3.5 * ts + 2.0) {
            continue;
        }
        let major = abs(v - round(v / 20.0) * 20.0) < 0.1;
        let len = select(0.026, 0.042, major) * W;
        dim = max(dim, line(p, vec2<f32>(sp_x1 - len, y), vec2<f32>(sp_x1, y), select(1.0, 1.6, major)));
        if (major) {
            let nd = ndigits(v);
            let xr = sp_x1 - 0.050 * W;
            bright = max(bright, draw_u(p, vec2<f32>(xr - text_w(nd, ts), y - 3.5 * ts), ts, v, nd));
        }
    }

    // Current speed, boxed, with a pointer at the tape edge.
    bright = max(bright, box_outline(p, sp_box_c, sp_box, 1.4));
    bright = max(bright, tri(p, vec2<f32>(sp_box_c.x + sp_box.x + 0.012 * W, sp_box_c.y), vec2<f32>(sp_box_c.x + sp_box.x, sp_box_c.y - 0.010 * H), vec2<f32>(sp_box_c.x + sp_box.x, sp_box_c.y + 0.010 * H), 1.0));
    let sp_nd = ndigits(cas);
    bright = max(bright, draw_u(p, vec2<f32>(sp_box_c.x - text_w(sp_nd, bs) * 0.5, sp_box_c.y - 3.5 * bs), bs, cas, sp_nd));

    // Selected speed, readout above the tape and bug on the inner edge.
    if (!speed_is_mach) {
        let sp_sel_nd = ndigits(sel_speed);
        mag = max(mag, draw_u(p, vec2<f32>(sp_x1 - text_w(sp_sel_nd, ts), 0.070 * H), ts, sel_speed, sp_sel_nd));
        let y = tape_cy - (sel_speed - cas) * sp_ppk;
        if (y > tape_top && y < tape_bot) {
            mag = max(mag, line(p, vec2<f32>(sp_x1 - 0.045 * W, y), vec2<f32>(sp_x1, y), 2.0));
            mag = max(mag, tri(p, vec2<f32>(sp_x1 - 0.045 * W, y), vec2<f32>(sp_x1 - 0.025 * W, y - 0.010 * H), vec2<f32>(sp_x1 - 0.025 * W, y + 0.010 * H), 1.0));
        }
    }

    // Rotation-speed bug and ROTATE callout on the ground.
    if (on_ground) {
        let y = tape_cy - (vr - cas) * sp_ppk;
        if (y > tape_top && y < tape_bot) {
            green = max(green, line(p, vec2<f32>(sp_x1 - 0.045 * W, y), vec2<f32>(sp_x1, y), 2.4));
            if (abs(y - tape_cy) > sp_box.y + 3.5 * ts + 2.0) {
                let vr_word = array<u32, 8>(CH_V, CH_R, CH_SP, CH_SP, CH_SP, CH_SP, CH_SP, CH_SP);
                green = max(green, draw_word(p, vec2<f32>(sp_x1 - 0.045 * W - text_w(2u, ts), y - 3.5 * ts), ts, vr_word, 2u));
            }
        }
        if (cas >= vr - 2.0) {
            let glow = 0.60 + 0.40 * sin(u.g.x * 9.0);
            let rot = array<u32, 8>(CH_R, CH_O, CH_T, CH_A, CH_T, CH_E, CH_SP, CH_SP);
            green = max(green, draw_word(p, vec2<f32>(center_x(ac.x, 6u, ts * 1.35), 0.700 * H), ts * 1.35, rot, 6u) * glow);
        }
    }

    // --- altitude tape ----------------------------------------------------
    let al_x0 = 0.745 * W;
    let al_x1 = 0.925 * W;
    let al_ppf = (tape_bot - tape_top) / 700.0; // ±350 ft
    let al_box = vec2<f32>(0.090 * W, 0.040 * H);
    let al_box_c = vec2<f32>((al_x0 + al_x1) * 0.5, tape_cy);

    col = mix(col, TAPE_BG, rect_clip(
        p,
        vec2<f32>((al_x0 + al_x1) * 0.5, 0.485 * H),
        vec2<f32>(0.098 * W, 0.390 * H),
    ));

    fill = max(fill, box_fill(p, al_box_c, al_box));

    let al_base = floor((alt - 350.0) / 100.0) * 100.0;
    for (var i: i32 = 0; i <= 14; i = i + 1) {
        let v = al_base + f32(i) * 100.0;
        if (v < 0.0) {
            continue;
        }
        let y = tape_cy - (v - alt) * al_ppf;
        if (y < tape_top || y > tape_bot) {
            continue;
        }
        if (abs(y - tape_cy) < al_box.y + 3.5 * ts + 2.0) {
            continue;
        }
        let major = abs(v - round(v / 200.0) * 200.0) < 0.1;
        let len = select(0.026, 0.042, major) * W;
        dim = max(dim, line(p, vec2<f32>(al_x0, y), vec2<f32>(al_x0 + len, y), select(1.0, 1.6, major)));
        if (major) {
            let nd = ndigits(v);
            bright = max(bright, draw_u(p, vec2<f32>(al_x0 + 0.050 * W, y - 3.5 * ts), ts, v, nd));
        }
    }

    // Current altitude, boxed, with a pointer at the tape edge.
    bright = max(bright, box_outline(p, al_box_c, al_box, 1.4));
    bright = max(bright, tri(p, vec2<f32>(al_box_c.x - al_box.x - 0.012 * W, al_box_c.y), vec2<f32>(al_box_c.x - al_box.x, al_box_c.y - 0.010 * H), vec2<f32>(al_box_c.x - al_box.x, al_box_c.y + 0.010 * H), 1.0));
    let al_nd = ndigits(alt);
    bright = max(bright, draw_u(p, vec2<f32>(al_box_c.x - text_w(al_nd, abs_) * 0.5, al_box_c.y - 3.5 * abs_), abs_, alt, al_nd));

    // Selected altitude, readout above the tape and a marker on its inner edge.
    let sa_nd = ndigits(sel_alt);
    mag = max(mag, draw_u(p, vec2<f32>(al_x1 - text_w(sa_nd, ts), 0.070 * H), ts, sel_alt, sa_nd));
    let sa_y = tape_cy - (sel_alt - alt) * al_ppf;
    if (sa_y > tape_top && sa_y < tape_bot) {
        let bx = al_x0 - 0.018 * W;
        mag = max(mag, box_outline(p, vec2<f32>(bx, sa_y), vec2<f32>(0.014 * W, 0.011 * H), 1.4));
        mag = max(mag, line(p, vec2<f32>(bx + 0.014 * W, sa_y), vec2<f32>(al_x0, sa_y), 1.4));
    }

    // --- vertical speed scale --------------------------------------------
    let vs_x = 0.968 * W;
    let vs_half = 0.190 * H; // ±2000 fpm
    let vs_ppf = vs_half / 2000.0;
    dim = max(dim, line(p, vec2<f32>(vs_x, tape_cy - vs_half), vec2<f32>(vs_x, tape_cy + vs_half), 1.0));
    for (var i: i32 = -4; i <= 4; i = i + 1) {
        if (i == 0) {
            continue;
        }
        let y = tape_cy - f32(i) * 500.0 * vs_ppf;
        let len = 0.016 * W;
        dim = max(dim, line(p, vec2<f32>(vs_x - len, y), vec2<f32>(vs_x, y), 1.0));
    }
    let vs_py = tape_cy - clamp(vs, -2000.0, 2000.0) * vs_ppf;
    bright = max(bright, tri(p, vec2<f32>(vs_x - 0.004 * W, vs_py), vec2<f32>(vs_x - 0.038 * W, vs_py - 0.012 * H), vec2<f32>(vs_x - 0.038 * W, vs_py + 0.012 * H), 1.2));

    // --- heading tape -----------------------------------------------------
    let hdg_cx = 0.50 * W;
    let hy = 0.805 * H;
    let hdg_ppd = (0.60 * W) / 60.0; // ±30°
    let h_box = vec2<f32>(0.060 * W, 0.030 * H);
    fill = max(fill, box_fill(p, vec2<f32>(hdg_cx, hy), h_box));

    let hdg_base = floor((heading - 30.0) / 5.0) * 5.0;
    for (var i: i32 = 0; i <= 12; i = i + 1) {
        let deg = hdg_base + f32(i) * 5.0;
        let dx = (deg - heading) * hdg_ppd;
        if (abs(dx) > 0.30 * W) {
            continue;
        }
        let major = abs(deg - round(deg / 10.0) * 10.0) < 0.1;
        let x = hdg_cx + dx;
        let len = select(0.012, 0.020, major) * H;
        dim = max(dim, line(p, vec2<f32>(x, hy + 0.034 * H), vec2<f32>(x, hy + 0.034 * H + len), 1.0));
        if (major) {
            var shown = deg - floor(deg / 360.0) * 360.0;
            let ly = hy + 0.060 * H;
            let cardinal = abs(shown - round(shown / 90.0) * 90.0) < 0.1;
            if (cardinal) {
                var ch = CH_N;
                if (abs(shown - 90.0) < 0.1) { ch = CH_E; }
                if (abs(shown - 180.0) < 0.1) { ch = CH_S; }
                if (abs(shown - 270.0) < 0.1) { ch = CH_W; }
                dim = max(dim, glyph_at(p, vec2<f32>(x - 2.5 * ts, ly), ts, ch));
            } else {
                let tens = shown / 10.0;
                dim = max(dim, draw_u(p, vec2<f32>(x - text_w(2u, ts) * 0.5, ly), ts, tens, 2u));
            }
        }
    }

    // Current heading, boxed (north reads 360).
    var hdg_shown = heading - floor(heading / 360.0) * 360.0;
    if (hdg_shown < 0.5) {
        hdg_shown = 360.0;
    }
    bright = max(bright, box_outline(p, vec2<f32>(hdg_cx, hy), h_box, 1.4));
    bright = max(bright, draw_u(p, vec2<f32>(hdg_cx - text_w(3u, hbs) * 0.5, hy - 3.5 * hbs), hbs, hdg_shown, 3u));

    // Selected heading bug, clamped to the tape ends.
    let hdg_off = wrap180(sel_hdg - heading);
    if (abs(hdg_off) <= 30.0) {
        let bx = hdg_cx + hdg_off * hdg_ppd;
        mag = max(mag, line(p, vec2<f32>(bx, hy - 0.030 * H), vec2<f32>(bx, hy), 2.2));
        mag = max(mag, tri(p, vec2<f32>(bx, hy - 0.030 * H), vec2<f32>(bx - 0.012 * W, hy - 0.046 * H), vec2<f32>(bx + 0.012 * W, hy - 0.046 * H), 1.0));
    }

    // --- FMA --------------------------------------------------------------
    if (disc_annun) {
        let w = array<u32, 8>(CH_A, CH_P, CH_SP, CH_D, CH_I, CH_S, CH_C, CH_SP);
        amber = max(amber, draw_word(p, vec2<f32>(center_x(0.5 * W, 7u, fma_s), 0.022 * H), fma_s, w, 7u));
    } else {
        let at = u.f.x;
        var at_word = array<u32, 8>(CH_SP, CH_SP, CH_SP, CH_SP, CH_SP, CH_SP, CH_SP, CH_SP);
        var at_n = 0u;
        if (at > 1.5) {
            at_word = array<u32, 8>(CH_N, CH_SP, CH_SP, CH_SP, CH_SP, CH_SP, CH_SP, CH_SP);
            at_n = 1u;
        } else if (at > 0.5) {
            at_word = array<u32, 8>(CH_S, CH_P, CH_D, CH_SP, CH_SP, CH_SP, CH_SP, CH_SP);
            at_n = 3u;
        }
        green = max(green, draw_word(p, vec2<f32>(center_x(0.20 * W, at_n, fma_s), 0.022 * H), fma_s, at_word, at_n));

        if (engaged) {
            if (u.f.y > 0.5) {
                let roll_word = array<u32, 8>(CH_H, CH_D, CH_G, CH_SP, CH_S, CH_E, CH_L, CH_SP);
                green = max(green, draw_word(p, vec2<f32>(center_x(0.5 * W, 7u, fma_s), 0.022 * H), fma_s, roll_word, 7u));
            }
            var vert_word = array<u32, 8>(CH_SP, CH_SP, CH_SP, CH_SP, CH_SP, CH_SP, CH_SP, CH_SP);
            var vert_n = 0u;
            if (u.f.z > 2.5) {
                vert_word = array<u32, 8>(CH_L, CH_V, CH_L, CH_SP, CH_C, CH_H, CH_G, CH_SP);
                vert_n = 7u;
            } else if (u.f.z > 1.5) {
                vert_word = array<u32, 8>(CH_V, CH_SLASH, CH_S, CH_SP, CH_SP, CH_SP, CH_SP, CH_SP);
                vert_n = 3u;
            } else if (u.f.z > 0.5) {
                vert_word = array<u32, 8>(CH_A, CH_L, CH_T, CH_SP, CH_H, CH_O, CH_L, CH_D);
                vert_n = 8u;
            }
            green = max(green, draw_word(p, vec2<f32>(center_x(0.80 * W, vert_n, fma_s), 0.022 * H), fma_s, vert_word, vert_n));

            let cmd = array<u32, 8>(CH_C, CH_M, CH_D, CH_SP, CH_SP, CH_SP, CH_SP, CH_SP);
            green = max(green, draw_word(p, vec2<f32>(center_x(0.5 * W, 3u, fma_s), 0.052 * H), fma_s, cmd, 3u));

            let fd = array<u32, 8>(CH_F, CH_D, CH_SP, CH_SP, CH_SP, CH_SP, CH_SP, CH_SP);
            green = max(green, draw_word(p, vec2<f32>(center_x(0.5 * W, 2u, fma_s), 0.125 * H), fma_s, fd, 2u));
        }
    }

    // --- bottom data row --------------------------------------------------
    let by = 0.930 * H;
    if (on_ground) {
        let flap_word = array<u32, 8>(CH_F, CH_L, CH_A, CH_P, CH_SP, CH_SP, CH_SP, CH_SP);
        let fx = 0.040 * W;
        green = max(green, draw_word(p, vec2<f32>(fx, by), bot_s, flap_word, 4u));
        let flap_deg = round(flap_norm * 40.0 / 5.0) * 5.0;
        green = max(green, draw_u(p, vec2<f32>(fx + 5.0 * 6.0 * bot_s, by), bot_s, flap_deg, 2u));
    } else {
        let gs_word = array<u32, 8>(CH_G, CH_S, CH_SP, CH_SP, CH_SP, CH_SP, CH_SP, CH_SP);
        let gx = 0.040 * W;
        bright = max(bright, draw_word(p, vec2<f32>(gx, by), bot_s, gs_word, 2u));
        bright = max(bright, draw_u(p, vec2<f32>(gx + 3.0 * 6.0 * bot_s, by), bot_s, gs, 3u));
    }

    // Barometric setting, in green on the real display.
    let hpa_word = array<u32, 8>(CH_H, CH_P, CH_A, CH_SP, CH_SP, CH_SP, CH_SP, CH_SP);
    let hx = 0.960 * W - text_w(8u, bot_s);
    green = max(green, draw_u(p, vec2<f32>(hx, by), bot_s, u.d.z, 4u));
    green = max(green, draw_word(p, vec2<f32>(hx + 5.0 * 6.0 * bot_s, by), bot_s, hpa_word, 3u));

    // --- composite --------------------------------------------------------
    col = mix(col, BLACK, clamp(fill, 0.0, 1.0));
    col = mix(col, DIM, clamp(dim, 0.0, 1.0));
    col = mix(col, WHITE, clamp(bright, 0.0, 1.0));
    col = mix(col, BLACK, clamp(sym, 0.0, 1.0));
    col = mix(col, GREEN, clamp(green, 0.0, 1.0));
    col = mix(col, MAGENTA, clamp(mag, 0.0, 1.0));
    col = mix(col, AMBER, clamp(amber, 0.0, 1.0));

    return vec4<f32>(col, 1.0);
}

@fragment
fn fragment(in: UiVertexOutput) -> @location(0) vec4<f32> {
    return pfd(u, in.uv * in.size, in.size);
}
