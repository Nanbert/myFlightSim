//! The Cessna-172 turn coordinator.
//!
//! Two instruments sharing one case. Above is the *rate of turn* indicator: a
//! symbolic aircraft seen from behind that banks away from level in proportion
//! to how fast the nose is swinging — wings level at the case marks is a
//! standard-rate turn, 3°/s, a full circle in two minutes. Below is the
//! *slip/skid* indicator, an inclinometer ball in a tube: it sits centred when
//! the turn is coordinated and rolls to the side you should push rudder
//! ("step on the ball"). Both read the *turn*, not the bank — hence the
//! placard, "no pitch information".

use bevy::prelude::*;

use super::*;
use crate::sim::AircraftState;

/// Turn rate that banks the symbol to the case marks (standard rate), deg/s.
const STD_RATE_DPS: f32 = 3.0;
/// Symbol bank at standard rate.
const FULL_BANK_DEG: f32 = 20.0;
/// Lateral load factor (g) that pins the ball against its stop.
const FULL_SLIP_G: f32 = 0.15;

// The slip tube, in dial-radius units.

/// Height of the tube's middle below the dial centre.
const TUBE_CY: f32 = 0.40;
/// Half the tube's horizontal reach.
const TUBE_HX: f32 = 0.34;
/// How far the tube's ends rise above its middle (its curvature).
const TUBE_RISE: f32 = 0.045;
/// Tube half-thickness.
const TUBE_H: f32 = 0.070;
/// Ball travel each way at full deflection.
const BALL_TRAVEL: f32 = 0.27;

#[derive(Component)]
pub(super) struct TurnSymbol;

#[derive(Component)]
pub(super) struct SlipBall;

// --- Face rasterisation ---------------------------------------------------

const FACE: [u8; 4] = [16, 16, 18, 255];
const BEZEL: [u8; 4] = [14, 14, 16, 255];
const BEZEL_HI: [u8; 4] = [46, 48, 54, 255];
const BEZEL_RIM: [u8; 4] = [104, 108, 116, 255];
const TICK: [u8; 4] = [228, 228, 228, 255];
/// The light glass tube the ball rolls in, and the shadow along its lower rim.
const TUBE: [u8; 4] = [176, 178, 184, 255];
const TUBE_EDGE: [u8; 4] = [86, 88, 94, 255];

/// The fixed case: bezel, the two reference marks the wingtips swing to, and
/// the slip tube.
fn face_sample(x: f32, y: f32) -> [u8; 4] {
    let u = (x * x + y * y).sqrt() / DIAL_R;
    if u > 1.0 {
        return [0, 0, 0, 0];
    }
    if u >= 0.90 {
        return if u > 0.985 {
            BEZEL_RIM
        } else if u > 0.95 {
            BEZEL_HI
        } else {
            BEZEL
        };
    }
    let (nx, ny) = (x / DIAL_R, y / DIAL_R);

    // Reference marks either side of the symbol, at the wing line.
    if ny.abs() < 0.026 && (0.31..0.385).contains(&nx.abs()) {
        return TICK;
    }

    // The slip tube: a shallow concave-up groove with rounded ends.
    let t = (nx / TUBE_HX).clamp(-1.0, 1.0);
    let ny_t = TUBE_CY - TUBE_RISE * t * t;
    let dx = (nx.abs() - (TUBE_HX - TUBE_H)).max(0.0);
    let d = ((ny - ny_t).powi(2) + dx * dx).sqrt();
    if d < TUBE_H {
        return TUBE;
    }
    if d < TUBE_H + 0.010 {
        return TUBE_EDGE;
    }
    FACE
}

/// The banked aircraft symbol: wings and tail, white, centred on the dial so
/// the whole planform turns about its own middle.
fn symbol_sample(x: f32, y: f32) -> [u8; 4] {
    let (nx, ny) = (x / DIAL_R, y / DIAL_R);
    let wings = ny.abs() < 0.022 && nx.abs() < 0.30;
    let body = nx.abs() < 0.020 && (-0.03..0.13).contains(&ny);
    let tail = (ny - 0.125).abs() < 0.018 && nx.abs() < 0.095;
    if wings || body || tail {
        [238, 238, 242, 255]
    } else {
        [0, 0, 0, 0]
    }
}

/// The inclinometer ball, drawn at the centre and slid along the tube.
fn ball_sample(x: f32, y: f32) -> [u8; 4] {
    let (nx, ny) = (x / DIAL_R, y / DIAL_R);
    let r = (nx * nx + ny * ny).sqrt();
    if r > 0.062 {
        return [0, 0, 0, 0];
    }
    // Steel: darker than the glass tube, with a highlight to the upper left.
    let t = ((nx + ny) / (2.0 * 0.062) + 0.5).clamp(0.0, 1.0);
    let v = 58.0 + 96.0 * (1.0 - t);
    [v as u8, v as u8, (v + 6.0) as u8, 255]
}

// --- Scene ------------------------------------------------------------------

pub(super) fn setup(mut commands: Commands, mut images: ResMut<Assets<Image>>) {
    let face = images.add(make_image(raster_face(face_sample)));
    let symbol = images.add(make_image(raster_face(symbol_sample)));
    let ball = images.add(make_image(raster_face(ball_sample)));

    let c = NODE * 0.5;
    let root = spawn_root(&mut commands, PanelKey::Turn, Side::Right, 1, 0);
    commands.entity(root).with_children(|p| {
        spawn_image(p, face, ());

        let dim = Color::srgb(0.85, 0.85, 0.85);
        spawn_label(p, "TURN", c, c - DIAL_PX * 0.66, 12.0, Color::WHITE);
        spawn_label(p, "COORDINATOR", c, c - DIAL_PX * 0.55, 12.0, Color::WHITE);

        // Tube ends.
        spawn_label(p, "L", c - DIAL_PX * 0.43, c + DIAL_PX * 0.36, 14.0, Color::WHITE);
        spawn_label(p, "R", c + DIAL_PX * 0.43, c + DIAL_PX * 0.36, 14.0, Color::WHITE);

        spawn_label(p, "2 MIN", c, c + DIAL_PX * 0.70, 12.0, Color::WHITE);
        spawn_label(p, "NO PITCH", c, c + DIAL_PX * 0.80, 9.0, dim);
        spawn_label(p, "INFORMATION", c, c + DIAL_PX * 0.89, 9.0, dim);

        spawn_image(p, symbol, TurnSymbol);
        spawn_image(p, ball, SlipBall);
    });
}

/// Bank the symbol with the rate of turn and slide the ball with the lateral
/// load. The symbol texture points level, so a right turn just rolls it
/// clockwise (right wing down); the ball's texture is centred, so its offset
/// moves it along the tube — slightly uphill toward the ends, as the groove is
/// curved.
pub(super) fn update(
    state: Res<AircraftState>,
    mut sym: Query<&mut UiTransform, (With<TurnSymbol>, Without<SlipBall>)>,
    mut ball: Query<&mut UiTransform, (With<SlipBall>, Without<TurnSymbol>)>,
) {
    if let Ok(mut t) = sym.single_mut() {
        let bank = (state.turn_rate_dps as f32 / STD_RATE_DPS).clamp(-1.0, 1.0);
        t.rotation = Rot2::degrees(bank * FULL_BANK_DEG);
    }
    if let Ok(mut t) = ball.single_mut() {
        // The ball lags the aircraft's lateral acceleration, and that aero side
        // force is exactly `n-pilot-y-norm`; so it rolls opposite the sign.
        let d = (-state.slip as f32 / FULL_SLIP_G).clamp(-1.0, 1.0);
        let bx = d * BALL_TRAVEL;
        let by = TUBE_CY - TUBE_RISE * (bx / TUBE_HX).powi(2);
        t.translation = Val2::px(bx * DIAL_PX, by * DIAL_PX);
    }
}
