//! The 737 Mode Control Panel's layout, in baked-image pixels.
//!
//! `assets/ui/737_mcp_3d.png` is a 4940x898 orthographic render of the panel
//! (2x the 2470x449 layout space below). Every consumer works in those 2470x449
//! asset pixels and scales by its own `panel_width / CROP_W`:
//!
//! - the chase-view strip (`mcp_ui`) draws the baked image at 1100x200;
//! - the cockpit panel (`mcp_panel`) draws it at a viewport-fitted width;
//! - the bake tool (`mcp_bake`) builds the 3D meshes that produced the PNG.
//!
//! Keeping one table means the strip and the cockpit panel hit exactly what
//! they draw, and re-baking after a layout change cannot drift from the
//! hotspots.

use bevy::prelude::*;

pub use crate::panel_geom::PRect;

use crate::autopilot::McpWindow;

/// The baked layout's pixel space. The PNG is exactly this crop, at 2x.
pub const CROP_W: f32 = 2470.0;
pub const CROP_H: f32 = 449.0;

/// The chase-view strip's on-screen size. `pfd.rs` lays the two bottom PFDs
/// out around this width, so changing it moves the PFDs too.
pub const PANEL_W: f32 = 1100.0;
pub const PANEL_H: f32 = 200.0;

/// Engraved label colour, matching the panel's warm amber print.
pub const LABEL: Color = Color::srgb(1.0, 0.82, 0.50);

/// The whole plate.
pub const PANEL: PRect = PRect::new(0.0, 0.0, CROP_W, CROP_H);

/// A clickable control that is not a setpoint knob.
#[derive(Component, Clone, Copy, PartialEq, Eq)]
pub enum McpPress {
    AtArm,
    Cmd,
    HdgSel,
    AltHold,
    Vs,
    LvlChg,
    /// APP: arm the ILS beams (LOC + GS). It sits one row below the LNAV/VNAV
    /// column, left of ALT HOLD.
    Approach,
    /// The speed window itself switches between kt and Mach, like pushing the
    /// speed knob on the real MCP.
    SpeedUnit,
}

/// Functional push buttons. The rect is both the cap and the click target.
pub const BUTTONS: &[(PRect, McpPress, &str)] = &[
    (
        PRect::new(1934.0, 106.0, 2039.0, 215.0),
        McpPress::Cmd,
        "CMD",
    ),
    (
        PRect::new(779.0, 321.0, 883.0, 423.0),
        McpPress::LvlChg,
        "LVL CHG",
    ),
    (
        PRect::new(949.0, 321.0, 1053.0, 423.0),
        McpPress::HdgSel,
        "HDG SEL",
    ),
    (
        PRect::new(1124.0, 318.0, 1210.0, 395.0),
        McpPress::Approach,
        "APP",
    ),
    (
        PRect::new(1345.0, 315.0, 1460.0, 430.0),
        McpPress::AltHold,
        "ALT HLD",
    ),
    (
        PRect::new(1463.0, 315.0, 1578.0, 430.0),
        McpPress::Vs,
        "V/S",
    ),
];

/// The A/T ARM toggle, clickable as a press.
pub const AT_SWITCH: PRect = PRect::new(325.0, 150.0, 430.0, 258.0);

/// Live setpoint windows. The rect is the glass; the label sits above it.
pub const WINDOWS: &[(PRect, McpWindow, &str)] = &[
    (
        PRect::new(466.0, 112.0, 734.0, 180.0),
        McpWindow::Speed,
        "IAS/MACH",
    ),
    (
        PRect::new(928.0, 110.0, 1071.0, 180.0),
        McpWindow::Heading,
        "HEADING",
    ),
    (
        PRect::new(1266.0, 107.0, 1532.0, 178.0),
        McpWindow::Altitude,
        "ALTITUDE",
    ),
    (
        PRect::new(1595.0, 104.0, 1866.0, 176.0),
        McpWindow::VerticalSpeed,
        "VERT SPEED",
    ),
];

/// Live setpoint knobs. The click target is the whole rect, split in halves.
pub const KNOBS: &[(PRect, McpWindow)] = &[
    (PRect::new(595.0, 232.0, 705.0, 350.0), McpWindow::Speed),
    (PRect::new(935.0, 205.0, 1055.0, 315.0), McpWindow::Heading),
    (
        PRect::new(1360.0, 205.0, 1485.0, 315.0),
        McpWindow::Altitude,
    ),
];

/// The vertical-speed thumbwheel: upper half climbs, lower half descends.
pub const VS_WHEEL: PRect = PRect::new(1770.0, 215.0, 1825.0, 405.0);

/// Decorative push buttons whose label is printed on the cap.
pub const DECOR_CAPS: &[(PRect, &str)] = &[
    (PRect::new(775.0, 81.0, 878.0, 184.0), "VNAV"),
    (PRect::new(1117.0, 80.0, 1219.0, 185.0), "LNAV"),
    (PRect::new(1124.0, 216.0, 1204.0, 290.0), "VOR\nLOC"),
    (PRect::new(333.0, 315.0, 414.0, 389.0), "N1"),
    (PRect::new(438.0, 315.0, 519.0, 389.0), "SPEED"),
    (PRect::new(2068.0, 105.0, 2172.0, 212.0), "CMD"),
    (PRect::new(1932.0, 228.0, 2037.0, 346.0), "CWS"),
    (PRect::new(2066.0, 227.0, 2171.0, 345.0), "CWS"),
];

/// Decorative small round buttons; the label sits above them on the plate.
pub const DECOR_ROUND: &[(PRect, &str)] = &[
    (PRect::new(507.0, 220.0, 573.0, 290.0), "CO"),
    (PRect::new(692.0, 241.0, 766.0, 308.0), "SPD INTV"),
    (PRect::new(1463.0, 241.0, 1538.0, 308.0), "ALT INTV"),
];

/// The A/P disengage bar.
pub const DISENGAGE: PRect = PRect::new(1920.0, 302.0, 2186.0, 370.0);

/// Decorative COURSE windows and knobs (the 737's second, inactive MCP side).
pub const COURSE_L_WINDOW: PRect = PRect::new(109.0, 105.0, 237.0, 161.0);
pub const COURSE_L_KNOB: PRect = PRect::new(100.0, 200.0, 190.0, 290.0);
pub const COURSE_R_WINDOW: PRect = PRect::new(2295.0, 105.0, 2400.0, 161.0);
pub const COURSE_R_KNOB: PRect = PRect::new(2291.0, 203.0, 2377.0, 290.0);

/// Decorative F/D switches.
pub const FD_L: PRect = PRect::new(241.0, 325.0, 278.0, 425.0);
pub const FD_R: PRect = PRect::new(2223.0, 315.0, 2260.0, 415.0);

/// Engraved labels that are not attached to a cap or window table.
/// `(text, x, y, font px)`.
pub const EXTRA_LABELS: &[(&str, f32, f32, f32)] = &[
    ("A/T\nARM", 377.0, 98.0, 24.0),
    ("OFF", 377.0, 256.0, 20.0),
    ("A/P ENGAGE", 2053.0, 80.0, 24.0),
    ("F/D\nON", 260.0, 316.0, 20.0),
    ("OFF", 260.0, 427.0, 18.0),
    ("F/D\nON", 2242.0, 305.0, 20.0),
    ("OFF", 2242.0, 417.0, 18.0),
    ("UP", 1857.0, 225.0, 20.0),
    ("DN", 1857.0, 400.0, 20.0),
];

/// The lit glass of a window, excluding its bezel.
pub fn glass(window: McpWindow) -> PRect {
    match window {
        McpWindow::Speed => WINDOWS[0].0,
        McpWindow::Heading => WINDOWS[1].0,
        McpWindow::Altitude => WINDOWS[2].0,
        McpWindow::VerticalSpeed => WINDOWS[3].0,
    }
}

pub fn slots(window: McpWindow) -> usize {
    match window {
        McpWindow::Speed => 4,
        McpWindow::Heading => 3,
        McpWindow::Altitude => 5,
        McpWindow::VerticalSpeed => 5,
    }
}

/// Cap label size: the longer labels ("LVL CHG", "ALT HLD") need a smaller
/// face to stay inside the cap.
pub fn cap_font(label: &str) -> f32 {
    if label.chars().count() > 6 {
        20.0
    } else {
        22.0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_layout_covers_the_baked_crop() {
        assert_eq!(PANEL.x0, 0.0);
        assert_eq!(PANEL.y0, 0.0);
        assert_eq!(PANEL.x1, CROP_W);
        assert_eq!(PANEL.y1, CROP_H);
    }

    #[test]
    fn functional_controls_sit_inside_the_plate() {
        for &(rect, _, _) in BUTTONS {
            assert!(rect.x0 >= PANEL.x0 && rect.x1 <= PANEL.x1);
            assert!(rect.y0 >= PANEL.y0 && rect.y1 <= PANEL.y1);
        }
        for &(rect, _) in KNOBS {
            assert!(rect.x0 >= PANEL.x0 && rect.x1 <= PANEL.x1);
        }
        for &(rect, _, _) in WINDOWS {
            assert!(rect.x0 >= PANEL.x0 && rect.x1 <= PANEL.x1);
        }
        assert!(AT_SWITCH.x0 >= PANEL.x0 && AT_SWITCH.x1 <= PANEL.x1);
        assert!(VS_WHEEL.x0 >= PANEL.x0 && VS_WHEEL.x1 <= PANEL.x1);
    }

    #[test]
    fn the_strip_keeps_the_baked_aspect() {
        // The strip rounds 1100/200 to a whole height; the 0.02% difference is
        // invisible and keeps `pfd.rs`'s round numbers.
        assert!((PANEL_W / PANEL_H - CROP_W / CROP_H).abs() < 0.01);
    }

    #[test]
    fn a_click_on_a_cap_hits_its_own_target() {
        for &(rect, _, _) in BUTTONS {
            assert!(rect.contains(Vec2::new(rect.cx(), rect.cy())));
        }
    }

    #[test]
    fn labels_sit_inside_the_plate() {
        for &(_, x, y, _) in EXTRA_LABELS {
            assert!((0.0..=CROP_W).contains(&x));
            assert!((0.0..=CROP_H).contains(&y));
        }
        for &(rect, _) in DECOR_ROUND {
            assert!((0.0..=CROP_H).contains(&(rect.y0 - 12.0)));
        }
        for &(rect, _, _) in WINDOWS {
            assert!((0.0..=CROP_H).contains(&(rect.y0 - 38.0)));
        }
    }

    #[test]
    fn the_wheel_labels_match_its_halves() {
        // The wheel's upper half climbs and its lower half descends, so the
        // UP label must sit above the DN label beside the wheel.
        let wheel = VS_WHEEL;
        let labels: Vec<(&str, f32)> = EXTRA_LABELS
            .iter()
            .filter(|(t, _, _, _)| matches!(*t, "UP" | "DN"))
            .copied()
            .map(|(t, x, y, _)| {
                assert!(
                    x > wheel.x0 && x - wheel.x1 < 50.0,
                    "the {t} label sits beside the wheel"
                );
                (t, y)
            })
            .collect();
        assert_eq!(labels.len(), 2, "one UP and one DN beside the wheel");
        assert_eq!(labels[0].0, "UP", "the climbing half is labeled UP");
        assert!(
            labels[0].1 < labels[1].1,
            "UP sits above DN the way the wheel's halves run"
        );
        assert_eq!(labels[1].0, "DN", "the descending half is labeled DN");
    }
}
