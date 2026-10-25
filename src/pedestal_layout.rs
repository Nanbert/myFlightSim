//! The 737 control stand's asset-pixel coordinates.
//!
//! One table serves the bake (`pedestal_bake`) and the live strip
//! (`pedestal_ui`), so a lever cannot drift from its slot or its label. The
//! stand is drawn as a front elevation of the pedestal: every gate runs
//! vertically, forward/full at the top and aft/idle at the bottom.
//!
//! The flap gate is deliberately not linear in degrees. Its nine book stops are
//! printed at even spacing — like the real gate, where UP/1/2/5 are crowded
//! together — and the lever and the actual-flap pointer share that scale. The
//! numeric truth stays in the labels and in the live degree readout.

use bevy::prelude::Vec2;

use crate::panel_geom::PRect;

/// The baked layout's pixel space. The PNG is exactly this crop, at 2x.
pub const CROP_W: f32 = 2470.0;
pub const CROP_H: f32 = 540.0;

/// The chase-view strip's on-screen size. It sits above the MCP strip.
pub const PANEL_W: f32 = 1100.0;
pub const PANEL_H: f32 = 240.0;

/// The whole plate.
pub const PANEL: PRect = PRect::new(0.0, 0.0, CROP_W, CROP_H);

/// The gate travel, shared by every lever: forward/full at the top, aft at the
/// bottom.
pub const TRAVEL_TOP: f32 = 96.0;
pub const TRAVEL_BOTTOM: f32 = 462.0;

/// How far the strip sits above the bottom edge of the screen, and the gap it
/// leaves over the MCP strip (which starts at 12 + 200).
pub const BOTTOM: f32 = 220.0;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LeverId {
    Flap,
    Speedbrake,
    ThrottleLeft,
    ThrottleRight,
    Gear,
}

/// One vertical lever gate.
#[derive(Clone, Copy, Debug)]
pub struct Lever {
    pub id: LeverId,
    /// The slot itself: `x0..x1` is the channel, `y0` full/up, `y1` idle/down.
    pub channel: PRect,
    /// The visible handle, in asset px. The stem is a thin bar up from the
    /// bottom of the channel to the handle's centre.
    pub knob: Vec2,
    /// The click target. Wider than the slot so the printed labels are live
    /// detent targets too.
    pub hit: PRect,
    /// Handle half-width for the stem, asset px.
    pub stem_w: f32,
}

/// The five gates, left to right.
pub const LEVERS: &[Lever] = &[
    Lever {
        id: LeverId::Flap,
        channel: PRect::new(346.0, TRAVEL_TOP, 398.0, TRAVEL_BOTTOM),
        knob: Vec2::new(88.0, 46.0),
        hit: PRect::new(256.0, 66.0, 440.0, 492.0),
        stem_w: 10.0,
    },
    Lever {
        id: LeverId::Speedbrake,
        channel: PRect::new(614.0, TRAVEL_TOP, 666.0, TRAVEL_BOTTOM),
        knob: Vec2::new(104.0, 46.0),
        hit: PRect::new(586.0, 66.0, 830.0, 492.0),
        stem_w: 12.0,
    },
    Lever {
        id: LeverId::ThrottleLeft,
        channel: PRect::new(964.0, TRAVEL_TOP, 1016.0, TRAVEL_BOTTOM),
        knob: Vec2::new(96.0, 52.0),
        hit: PRect::new(924.0, 66.0, 1050.0, 480.0),
        stem_w: 12.0,
    },
    Lever {
        id: LeverId::ThrottleRight,
        channel: PRect::new(1134.0, TRAVEL_TOP, 1186.0, TRAVEL_BOTTOM),
        knob: Vec2::new(96.0, 52.0),
        hit: PRect::new(1094.0, 66.0, 1220.0, 480.0),
        stem_w: 12.0,
    },
    Lever {
        // The gear gate is a short throw: a couple of inches, not the flap's
        // full quadrant. A full-height slot would read as an empty hole.
        id: LeverId::Gear,
        channel: PRect::new(1704.0, 262.0, 1756.0, 430.0),
        knob: Vec2::new(76.0, 76.0),
        hit: PRect::new(1668.0, 200.0, 1800.0, 480.0),
        stem_w: 10.0,
    },
];

/// The flap lever's nine printed stops, bottom to top: `(caption, norm)`.
pub const FLAP_STOPS: &[(&str, f64)] = &[
    ("UP", 0.0),
    ("1", 1.0 / 40.0),
    ("2", 2.0 / 40.0),
    ("5", 5.0 / 40.0),
    ("10", 10.0 / 40.0),
    ("15", 15.0 / 40.0),
    ("25", 25.0 / 40.0),
    ("30", 30.0 / 40.0),
    ("40", 1.0),
];

/// The actual-flap indicator: a recessed well left of the flap gate, with a
/// fill that rises with the trailing edge.
pub const FLAP_INDICATOR: PRect = PRect::new(172.0, TRAVEL_TOP, 232.0, TRAVEL_BOTTOM);
/// The lit part of the indicator, inside the bezel.
pub const FLAP_TRACK: PRect = PRect::new(178.0, 106.0, 226.0, 452.0);
/// The stop captions' centre line.
pub const FLAP_LABEL_X: f32 = 296.0;

impl Lever {
    pub fn cx(&self) -> f32 {
        self.channel.cx()
    }

    /// 0..1 up the gate (0 = bottom/aft, 1 = top/forward). The flap gate uses
    /// even visual spacing between its stops; the gear handle is the one gate
    /// whose up position means "up" rather than "more".
    pub fn unit(&self, value: f64) -> f32 {
        let v = value.clamp(0.0, 1.0);
        match self.id {
            LeverId::Flap => flap_unit(v) as f32,
            LeverId::Gear => (1.0 - v) as f32,
            _ => v as f32,
        }
    }

    /// The handle's centre y.
    pub fn y(&self, value: f64) -> f32 {
        self.channel.y1 - self.unit(value) * (self.channel.y1 - self.channel.y0)
    }

    /// The value a click at `y` asks for, snapped to the gate's stops.
    pub fn value_at(&self, y: f32) -> f64 {
        let unit = ((self.channel.y1 - y) / (self.channel.y1 - self.channel.y0)).clamp(0.0, 1.0);
        let raw = match self.id {
            LeverId::Flap => flap_value(unit as f64),
            LeverId::Gear => 1.0 - unit as f64,
            _ => unit as f64,
        };
        self.snap(raw)
    }

    /// Snap a raw value to the nearest mechanical stop. Throttles are the only
    /// continuous gates.
    pub fn snap(&self, value: f64) -> f64 {
        let v = value.clamp(0.0, 1.0);
        match self.id {
            LeverId::Flap => nearest_flap_stop(v),
            LeverId::Gear | LeverId::Speedbrake => {
                if v >= 0.5 {
                    1.0
                } else {
                    0.0
                }
            }
            LeverId::ThrottleLeft | LeverId::ThrottleRight => v,
        }
    }

    /// The flap gate's printed caption for a stop, if `value` is on one.
    pub fn flap_caption(&self, value: f64) -> Option<&'static str> {
        FLAP_STOPS
            .iter()
            .find(|(_, norm)| (norm - value).abs() < 1e-9)
            .map(|(label, _)| *label)
    }
}

/// Which engine indicator bar.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum EngineSide {
    Left,
    Right,
}

/// The two N2 bars, right of the throttle pair. N2 is read on a 0..100 scale,
/// with the CFM56's idle at 60.
pub const N2_TRACKS: &[(EngineSide, PRect)] = &[
    (
        EngineSide::Left,
        PRect::new(1279.0, 130.0, 1303.0, 430.0),
    ),
    (
        EngineSide::Right,
        PRect::new(1361.0, 130.0, 1385.0, 430.0),
    ),
];

/// A lamp lens. The bake cuts a dark lens; the live layer lights it.
#[derive(Clone, Copy, Debug)]
pub struct LightSpec {
    pub id: LightId,
    pub lens: PRect,
    /// Round lenses for the three gear lights; rectangles for annunciators.
    pub round: bool,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LightId {
    GearNose,
    GearLeft,
    GearRight,
    GearUnsafe,
    GearTransit,
    At,
    Rev,
    SpdBrk,
}

pub const LIGHTS: &[LightSpec] = &[
    LightSpec {
        id: LightId::GearNose,
        lens: PRect::new(1826.0, 146.0, 1874.0, 194.0),
        round: true,
    },
    LightSpec {
        id: LightId::GearLeft,
        lens: PRect::new(1926.0, 146.0, 1974.0, 194.0),
        round: true,
    },
    LightSpec {
        id: LightId::GearRight,
        lens: PRect::new(2026.0, 146.0, 2074.0, 194.0),
        round: true,
    },
    LightSpec {
        id: LightId::GearUnsafe,
        lens: PRect::new(1865.0, 330.0, 1935.0, 374.0),
        round: false,
    },
    LightSpec {
        id: LightId::GearTransit,
        lens: PRect::new(1975.0, 330.0, 2045.0, 374.0),
        round: false,
    },
    LightSpec {
        id: LightId::At,
        lens: PRect::new(1440.0, 150.0, 1510.0, 196.0),
        round: false,
    },
    LightSpec {
        id: LightId::Rev,
        lens: PRect::new(1440.0, 250.0, 1510.0, 296.0),
        round: false,
    },
    LightSpec {
        id: LightId::SpdBrk,
        lens: PRect::new(745.0, 250.0, 815.0, 296.0),
        round: false,
    },
];

/// The reverse latch: a bar under the throttle pair. Reverse is a latch in the
/// throttle gate, so it is a push target rather than a third lever.
pub const REVERSE_BAR: PRect = PRect::new(930.0, 486.0, 1230.0, 524.0);

/// Static printed labels that are not stop captions: `(text, x, y, font px)`.
pub const LABELS: &[(&str, f32, f32, f32)] = &[
    ("FLAP", 372.0, 44.0, 30.0),
    ("SPEED BRAKE", 640.0, 44.0, 26.0),
    ("THROTTLE", 1100.0, 44.0, 28.0),
    ("LANDING GEAR", 1880.0, 44.0, 26.0),
    ("CONTROL STAND", 2290.0, 44.0, 22.0),
    ("ACTUAL", 202.0, 72.0, 15.0),
    ("UP", 740.0, 122.0, 20.0),
    ("DOWN", 740.0, 440.0, 20.0),
    ("SPD BRK", 780.0, 320.0, 18.0),
    ("NOSE", 1850.0, 214.0, 18.0),
    ("LEFT", 1950.0, 214.0, 18.0),
    ("RIGHT", 2050.0, 214.0, 18.0),
    ("UNSAFE", 1900.0, 398.0, 17.0),
    ("TRANSIT", 2010.0, 398.0, 17.0),
    ("N2", 1291.0, 114.0, 20.0),
    ("N2", 1373.0, 114.0, 20.0),
    ("A/T", 1475.0, 218.0, 20.0),
    ("REV", 1475.0, 318.0, 20.0),
    ("CLICK A DETENT", 2290.0, 126.0, 17.0),
    ("DRAG A LEVER", 2290.0, 154.0, 17.0),
    ("KEYS STILL WORK", 2290.0, 182.0, 17.0),
    ("737-800", 2290.0, 502.0, 18.0),
];

/// The flap gate's even visual fractions for its nine stops: `i / 8`.
pub fn flap_unit(cmd: f64) -> f64 {
    let v = cmd.clamp(0.0, 1.0);
    for i in 1..FLAP_STOPS.len() {
        let (_, hi) = FLAP_STOPS[i];
        if v <= hi {
            let (_, lo) = FLAP_STOPS[i - 1];
            let along = if hi > lo { (v - lo) / (hi - lo) } else { 0.0 };
            return (i as f64 - 1.0 + along) / (FLAP_STOPS.len() - 1) as f64;
        }
    }
    1.0
}

/// The inverse of [`flap_unit`]: a visual fraction back to `flap-cmd-norm`.
pub fn flap_value(unit: f64) -> f64 {
    let u = unit.clamp(0.0, 1.0);
    let n = FLAP_STOPS.len() - 1;
    let scaled = u * n as f64;
    let i = (scaled.floor() as usize).min(n - 1);
    let along = scaled - i as f64;
    let (_, lo) = FLAP_STOPS[i];
    let (_, hi) = FLAP_STOPS[i + 1];
    lo + along * (hi - lo)
}

/// The nearest printed stop to `cmd`.
pub fn nearest_flap_stop(cmd: f64) -> f64 {
    FLAP_STOPS
        .iter()
        .map(|(_, norm)| *norm)
        .min_by(|a, b| {
            (a - cmd)
                .abs()
                .partial_cmp(&(b - cmd).abs())
                .unwrap_or(std::cmp::Ordering::Equal)
        })
        .unwrap_or(0.0)
}

/// A lever by id.
pub fn lever(id: LeverId) -> &'static Lever {
    LEVERS.iter().find(|l| l.id == id).expect("lever table")
}

/// A lamp by id.
pub fn light(id: LightId) -> &'static LightSpec {
    LIGHTS.iter().find(|l| l.id == id).expect("light table")
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_layout_covers_the_baked_crop() {
        assert_eq!(PANEL.x1, CROP_W);
        assert_eq!(PANEL.y1, CROP_H);
        // The strip keeps the crop's aspect to within a pixel.
        let crop = CROP_W / CROP_H;
        let strip = PANEL_W / PANEL_H;
        assert!((crop - strip).abs() < 0.01, "{crop} vs {strip}");
    }

    #[test]
    fn every_lever_and_lamp_sits_inside_the_plate() {
        for l in LEVERS {
            assert!(PANEL.contains(Vec2::new(l.channel.x0, l.channel.y0)));
            assert!(PANEL.contains(Vec2::new(l.channel.x1, l.channel.y1)));
            assert!(PANEL.contains(Vec2::new(l.hit.x0, l.hit.y0)));
            assert!(PANEL.contains(Vec2::new(l.hit.x1, l.hit.y1)));
            // The handle must not run off the top or bottom of the plate.
            assert!(l.y(1.0) - l.knob.y * 0.5 >= 0.0);
            assert!(l.y(0.0) + l.knob.y * 0.5 <= CROP_H);
        }
        for spec in LIGHTS {
            assert!(PANEL.contains(Vec2::new(spec.lens.x0, spec.lens.y0)));
            assert!(PANEL.contains(Vec2::new(spec.lens.x1, spec.lens.y1)));
        }
        assert!(PANEL.contains(Vec2::new(REVERSE_BAR.x1, REVERSE_BAR.y1)));
    }

    #[test]
    fn labels_sit_inside_the_plate() {
        for &(_, x, y, size) in LABELS {
            assert!((0.0..=CROP_W).contains(&x), "{x}");
            assert!((size..=CROP_H).contains(&y), "{y}");
        }
    }

    #[test]
    fn the_flap_gate_prints_its_nine_book_stops_evenly() {
        let lever = lever(LeverId::Flap);
        assert_eq!(FLAP_STOPS.len(), 9);
        // The lever's travel is even in stop index, not in degrees.
        for (i, (_, norm)) in FLAP_STOPS.iter().enumerate() {
            let want = i as f32 / 8.0;
            assert!((lever.unit(*norm) - want).abs() < 1e-6, "stop {i}");
        }
        // UP is at the bottom, 40 at the top.
        assert!((lever.y(0.0) - TRAVEL_BOTTOM).abs() < 1e-4);
        assert!((lever.y(1.0) - TRAVEL_TOP).abs() < 1e-4);
    }

    #[test]
    fn every_flap_detent_round_trips_through_the_pixel_scale() {
        let lever = lever(LeverId::Flap);
        for (_, norm) in FLAP_STOPS {
            let y = lever.y(*norm);
            let back = lever.value_at(y);
            assert!((back - norm).abs() < 1e-9, "{norm} -> {y} -> {back}");
        }
        // Mid-travel between 5 and 10 snaps to whichever stop is nearer.
        let mid = (lever.y(5.0 / 40.0) + lever.y(10.0 / 40.0)) * 0.5;
        let snapped = lever.value_at(mid);
        assert!(snapped == 5.0 / 40.0 || snapped == 10.0 / 40.0);
    }

    #[test]
    fn the_gear_handle_points_down_when_the_gear_is_down() {
        let gear = lever(LeverId::Gear);
        assert!(gear.y(1.0) > gear.y(0.0), "down must sit lower than up");
        assert!((gear.y(1.0) - gear.channel.y1).abs() < 1e-4);
        assert!((gear.y(0.0) - gear.channel.y0).abs() < 1e-4);
        // Clicking the lower half commands down, the upper half up.
        assert_eq!(gear.value_at(gear.channel.y1), 1.0);
        assert_eq!(gear.value_at(gear.channel.y0), 0.0);
        // The gate is a short throw, clear of the lamp row above it.
        assert!(gear.channel.h() < TRAVEL_BOTTOM - TRAVEL_TOP);
        assert!(gear.channel.y0 > 200.0);
    }

    #[test]
    fn the_throttles_are_the_only_continuous_gates() {
        for l in LEVERS {
            let continuous = matches!(l.id, LeverId::ThrottleLeft | LeverId::ThrottleRight);
            let raw = 0.31;
            let snapped = l.snap(raw);
            if continuous {
                assert_eq!(snapped, raw, "{:?}", l.id);
            } else {
                assert_ne!(snapped, raw, "{:?}", l.id);
            }
        }
    }

    #[test]
    fn the_clicks_reach_their_gate() {
        for l in LEVERS {
            assert!(l.hit.contains(Vec2::new(l.cx(), l.channel.cy())), "{:?}", l.id);
        }
        // The reverse latch is its own target, clear of the throttle slots.
        assert!(REVERSE_BAR.contains(Vec2::new(REVERSE_BAR.cx(), REVERSE_BAR.cy())));
        for l in LEVERS {
            if matches!(l.id, LeverId::ThrottleLeft | LeverId::ThrottleRight) {
                assert!(l.hit.y1 <= REVERSE_BAR.y0, "{:?}", l.id);
            }
        }
    }
}
