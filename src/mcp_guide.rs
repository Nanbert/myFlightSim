//! The MCP GUIDE menu page: the baked panel with every switch clickable,
//! each carrying its sim meaning, its real 737 function and the links
//! between them.
//!
//! The page rides the menu's row pool for its prose and brings its own
//! overlay for the diagram: `assets/ui/737_mcp_3d.png` (the orthographic
//! bake `mcp_bake` renders from the layout tables) with a hotspot per
//! control, rects read straight from `mcp_layout` so the diagram can never
//! drift from the live panel. Click a switch (or walk the topics with the
//! arrow keys) and the rows below explain it: what it does in the sim, what
//! it does on the real jet, and which buttons move which.

use bevy::picking::hover::Hovered;
use bevy::prelude::*;
use bevy::ui_widgets::{Activate, Button};

use crate::app_state::AppState;
use crate::autopilot::McpWindow;
use crate::mcp_layout::{self, PRect};
use crate::menu_page::PageLine;
use crate::ui_theme;

/// The diagram's on-screen width; the height follows the bake's aspect.
const GUIDE_W: f32 = 1080.0;
/// Blank rows the diagram occupies above the prose (the card's row pool).
const IMAGE_ROWS: usize = 9;

// --- the topics ---------------------------------------------------------------

/// Which panel region (or regions) a topic speaks for. The rects come off
/// the `mcp_layout` tables - the same source the live panel clicks use.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum GuideTopic {
    Cmd,
    SpeedWindow,
    SpeedKnob,
    AtArm,
    HdgWindow,
    LvlChg,
    HdgSel,
    Approach,
    AltHold,
    Vs,
    AltWindow,
    VsWindow,
    Disengage,
    Decor,
    Links,
}

/// One guide topic: the control group, its title, and its prose.
pub struct Topic {
    pub id: GuideTopic,
    pub title: &'static str,
    pub lines: &'static [&'static str],
}

/// The guide's reading order: the master first, the targets next, the mode
/// buttons down the row, then the grey plate and the summary.
pub const TOPICS: &[Topic] = &[
    Topic {
        id: GuideTopic::Cmd,
        title: "CMD A/P",
        lines: &[
            "THE AUTOPILOT MASTER.",
            "SIM: key G, or this switch. Airborne only - it refuses on the",
            "ground. On engagement the jet comes up in HDG SEL + ALT HOLD:",
            "it holds the present heading and captures the present altitude.",
            "Pressing it again drops everything, the A/T with it.",
            "",
            "REAL 737: the FMA's CMD row lights when a mode is actually",
            "steering. Airline departures and arrivals are flown on the box;",
            "the pilot's craft is setting targets and then VERIFYING the FMA",
            "- the annunciation is the only contract with the automation.",
            "",
            "LINKS: every mode key (1/2/3/4/6) does nothing until CMD is on.",
            "The DISENGAGE bar and any stick tap disconnect it (M6's red line).",
        ],
    },
    Topic {
        id: GuideTopic::SpeedWindow,
        title: "SPEED WINDOW",
        lines: &[
            "THE SPEED TARGET, in kt or Mach.",
            "SIM: click the window to swap IAS/MACH (or key M). Scroll = 5 kt",
            "coarse.",
            "",
            "REAL 737: this is SELECTED speed - what you want, when you want",
            "it. The A/T in SPD steers thrust to this number and the LVL CHG",
            "pitch loop steers pitch to it. Above the crossover the real crew",
            "thinks in Mach; the window swaps with them.",
            "",
            "LINKS: LVL CHG flies pitch to hold this speed at climb thrust;",
            "A/T SPD holds it with thrust at any pitch. Feed the window before",
            "you feed the mode - a stale 250 here is a stale climb (M6's",
            "WINDOW row grades exactly this).",
        ],
    },
    Topic {
        id: GuideTopic::SpeedKnob,
        title: "SPEED KNOB",
        lines: &[
            "FINE SPEED TRIM - one knob, two halves.",
            "SIM: left half steps down, right half steps up, 1 kt per click;",
            "press and hold to repeat. The scroll wheel does the coarse work.",
            "",
            "REAL 737: the knob is how the target creeps onto the window -",
            "walking 250 down to 180 on the arrival without hammering a key",
            "forty times.",
            "",
            "LINKS: the number lands in the same window the A/T SPD and the",
            "LVL CHG pitch loop read. There is no second truth: window, knob,",
            "FMA - one target.",
        ],
    },
    Topic {
        id: GuideTopic::AtArm,
        title: "A/T ARM",
        lines: &[
            "THE AUTOTHROTTLE SWITCH.",
            "SIM: key 5 toggles A/T SPD on and off. The modes drive it too:",
            "4 LVL CHG arms it in N1 (climb thrust), 2 ALT HOLD hands it back",
            "to SPD. Your own throttle keys kick it off - you touched it, it",
            "is yours. Reverse drops it, as a retard would on the real jet.",
            "",
            "REAL 737: the A/T flies speed with thrust so the pitch loop can",
            "fly something else. It is shared work: thrust for speed, pitch",
            "for the path.",
            "",
            "LINKS: SPD holds the speed window; N1 holds climb thrust toward",
            "the altitude window. Off, and every speed task is back in your",
            "hands - the M6 lesson fails a disconnect for exactly that reason.",
        ],
    },
    Topic {
        id: GuideTopic::HdgWindow,
        title: "HEADING WINDOW + KNOB",
        lines: &[
            "THE HEADING BUG - WHAT HDG SEL STEERS TO.",
            "SIM: scroll = 5 deg coarse, knob halves = 1 deg fine. The bug",
            "shows on the PFD's heading tape.",
            "",
            "REAL 737: radar vectors live here. 'Fly heading 330, intercept",
            "the localizer' is this window plus a bug set BEFORE the turn.",
            "",
            "LINKS: since HDG SEL is already the live lateral mode after",
            "engagement, setting the bug IS the command - no press needed",
            "(read the FMA, not the buttons). LOC capture with APP armed",
            "takes the steering away from the bug: the jet flies the beam.",
            "The A/P banks at most 25 deg on the way.",
        ],
    },
    Topic {
        id: GuideTopic::LvlChg,
        title: "LVL CHG",
        lines: &[
            "THE CLIMB/DESCENT WORKHORSE.",
            "SIM: key 4. Speed on pitch at climb (or idle) thrust - the A/T",
            "comes up in N1 by itself. It captures the altitude window and",
            "hands over to ALT HOLD, with the A/T back at SPD.",
            "",
            "REAL 737: level change is the mode for 'when ATC moves you':",
            "climb or descend at a managed speed to the window. It protects",
            "speed with pitch where V/S protects only the rate.",
            "",
            "LINKS: the altitude window is the target - set it FIRST. A stale",
            "window flies the jet the wrong way (M6's LEG fail). Selecting it",
            "cancels an APP arm, as any vertical mode does.",
        ],
    },
    Topic {
        id: GuideTopic::HdgSel,
        title: "HDG SEL",
        lines: &[
            "THE LATERAL MODE AFTER ENGAGEMENT.",
            "SIM: key 1 - but note the FMA: HDG SEL is what engagement gives",
            "you, so the key is usually a confirmation, not a command. The",
            "heading window's bug steers the jet the moment it changes.",
            "",
            "REAL 737: the mode for radar vectors. The teaching point is the",
            "FMA habit: KNOW what mode is live before you touch anything.",
            "Redundant presses and unnoticed modes are where automation",
            "surprises start.",
            "",
            "LINKS: APP takes the steering when the localizer captures. The",
            "bug still shows on the tape, but the jet is flying the beam.",
        ],
    },
    Topic {
        id: GuideTopic::Approach,
        title: "APP",
        lines: &[
            "THE ILS MASTER ARM.",
            "SIM: key 6 arms LOC + G/S. The localizer captures first (inside",
            "1.6 deg of the course), then the glideslope - and only while the",
            "jet is still descending. A second press disarms.",
            "",
            "REAL 737: APP is how an ILS approach is flown: arm it outside",
            "the gate, verify LOC green on the FMA, then G/S green, then fly",
            "the beam down. 'Established' means BOTH annunciated - that is",
            "the check the M6 card freezes on.",
            "",
            "LINKS: the arm is fragile by design - 1/2/3/4 (any lateral or",
            "vertical selection) cancels it, and pressing 6 twice disarms.",
            "A level-off steals the G/S: the capture wants a live descent.",
        ],
    },
    Topic {
        id: GuideTopic::AltHold,
        title: "ALT HOLD",
        lines: &[
            "CAPTURE THE ALTITUDE - NOW.",
            "SIM: key 2. The current altitude becomes the held altitude the",
            "same frame. An A/T in N1 comes back to SPD.",
            "",
            "REAL 737: this is the 'stop the climb NOW' tool - the mid-climb",
            "ATC level-off, or the traffic response. Where LVL CHG flies to",
            "the window, ALT HOLD plants the jet where it is.",
            "",
            "LINKS: it cancels an APP arm like any vertical mode, and it",
            "hands the A/T from N1 to SPD - level flight wants speed, not",
            "thrust. M6's ATC stop-climb is this button's whole drill.",
        ],
    },
    Topic {
        id: GuideTopic::Vs,
        title: "V/S",
        lines: &[
            "PITCH HOLDS A RATE.",
            "SIM: key 3, with the V/S window set first: the jet flies that",
            "rate until the altitude window, where it captures to ALT HOLD.",
            "",
            "REAL 737: V/S is the open-loop mode - it protects the RATE, not",
            "the speed (the A/T does that) and not the bottom (the window",
            "does). Perfect for meeting a glideslope from above; watch it",
            "everywhere else.",
            "",
            "LINKS: the altitude window is the floor. Level flight cannot",
            "capture a glideslope - the G/S wants a live descent when the",
            "beam arrives (the sim draws the line at -50 ft/min).",
        ],
    },
    Topic {
        id: GuideTopic::AltWindow,
        title: "ALTITUDE WINDOW + KNOB",
        lines: &[
            "EVERY VERTICAL MODE'S TARGET.",
            "SIM: scroll = 1000 ft coarse, knob halves = 100 ft fine. LVL CHG",
            "and V/S both capture it and hand over to ALT HOLD.",
            "",
            "REAL 737: the one number ATC's altitude clearance becomes. It",
            "is set BEFORE the mode: the window is the intent, the button is",
            "the execution, the FMA is the confirmation.",
            "",
            "LINKS: ALT HOLD ignores it (it captures now). LVL CHG climbing",
            "with a stale window is the classic altitude bust - hence the",
            "set-then-press habit the whole guide preaches.",
        ],
    },
    Topic {
        id: GuideTopic::VsWindow,
        title: "V/S WINDOW + WHEEL",
        lines: &[
            "THE DESCENT RATE DIAL.",
            "SIM: scroll or the wheel, 100 ft/min per step; the wheel's upper",
            "half climbs, the lower half descends. V/S flies whatever sits in",
            "here.",
            "",
            "REAL 737: 1,500-2,500 fpm down is a normal arrival pace; the",
            "wheel walks the rate onto the window without spinning past it.",
            "",
            "LINKS: the rate only matters while V/S is the live vertical",
            "mode. The G/S, once captured, ignores it and flies the beam.",
        ],
    },
    Topic {
        id: GuideTopic::Disengage,
        title: "DISENGAGE BAR",
        lines: &[
            "THE TRUST BAR.",
            "SIM: the bar drops the A/P - and the A/T with it. In the M6",
            "automation lesson this is the red line: the box flies the",
            "arrival, and the bar ends the run.",
            "",
            "REAL 737: the disconnect bar is grabbed for hand-flown gates",
            "and for taking over when the automation surprises you. The AP",
            "DISC annunciation and the horn make sure the whole crew knows",
            "who is flying now.",
            "",
            "LINKS: after a disconnect every mode is dark. Re-engaging",
            "starts from HDG SEL + ALT HOLD at the present altitude - the",
            "window contents survive, the modes do not.",
        ],
    },
    Topic {
        id: GuideTopic::Decor,
        title: "GREY CONTROLS - NOT MODELLED",
        lines: &[
            "THE PLATE IS FULL; THE SIM FLYS WHAT IT HAS.",
            "VNAV/LNAV: the flight-management modes - lateral and vertical",
            "navigation off a route. No FMC in this sim, so no VNAV/LNAV;",
            "LVL CHG and HDG SEL do their jobs by hand.",
            "",
            "VOR/LOC: raw-VOR and localizer-only capture (an ILS's lateral",
            "without the glide). The COURSE knobs and windows feed those,",
            "and the F/D switches show the director bars with the A/P off.",
            "",
            "The sim's own TODO list carries them all. The lessons fly the",
            "modes it has - and fly them well.",
        ],
    },
    Topic {
        id: GuideTopic::Links,
        title: "LINKS & INTERLOCKS",
        lines: &[
            "THE WEB BEHIND THE PANEL - read this one twice.",
            "G  ENGAGE: comes up HDG SEL + ALT HOLD, A/T off.",
            "4  LVL CHG: A/T -> N1. Captures the ALT window -> ALT HOLD,",
            "             A/T -> SPD. Cancels an APP arm.",
            "2  ALT HOLD: captures NOW. A/T N1 -> SPD. Cancels an APP arm.",
            "3  V/S: flies the rate. Captures the ALT window -> ALT HOLD,",
            "             A/T -> SPD. Cancels an APP arm.",
            "1  HDG SEL: steering by bug - already live after the engage.",
            "6  APP: arms LOC then G/S. Cancelled by 1/2/3/4; re-press",
            "             disarms. LOC before G/S, always.",
            "G/S: needs a live descent (vs < -50). Level flight = miss.",
            "5  A/T: SPD on/off by hand; N1/SPD managed by the modes;",
            "             your throttle keys or the reverse drop it.",
            "",
            "THE HABIT: window first, button second, FMA always.",
        ],
    },
];

/// The topic count the menu's cursor wraps against.
pub const TOPIC_COUNT: usize = TOPICS.len();

/// The guide's state: the selected topic, and whether the guide page is on
/// top of the menu's stack (menu.rs writes `shown` from its page match; the
/// sync system only reads it).
#[derive(Resource, Default)]
pub struct McpGuideState {
    pub topic: usize,
    pub shown: bool,
}

/// A hotspot on the diagram: clicking it selects its topic.
#[derive(Component)]
pub(crate) struct GuideSpot(usize);

/// The diagram layer's root (positioned over the card's top rows).
#[derive(Component)]
struct GuideLayer;

/// The rects a topic covers, read off the `mcp_layout` tables - one source
/// of truth for the live panel and this diagram alike.
fn topic_rects(topic: GuideTopic) -> Vec<PRect> {
    fn press_rect(press: mcp_layout::McpPress) -> PRect {
        mcp_layout::BUTTONS
            .iter()
            .find(|(_, p, _)| *p == press)
            .map(|(r, _, _)| *r)
            .expect("the layout table carries every mode button")
    }
    fn window_rect(window: McpWindow) -> PRect {
        mcp_layout::WINDOWS
            .iter()
            .find(|(_, w, _)| *w == window)
            .map(|(r, _, _)| *r)
            .expect("the layout table carries every window")
    }
    fn knob_rect(window: McpWindow) -> PRect {
        mcp_layout::KNOBS
            .iter()
            .find(|(_, w)| *w == window)
            .map(|(r, _)| *r)
            .expect("the layout table carries every knob")
    }
    match topic {
        GuideTopic::Cmd => vec![press_rect(mcp_layout::McpPress::Cmd)],
        GuideTopic::SpeedWindow => vec![window_rect(McpWindow::Speed)],
        GuideTopic::SpeedKnob => vec![knob_rect(McpWindow::Speed)],
        GuideTopic::AtArm => vec![mcp_layout::AT_SWITCH],
        GuideTopic::HdgWindow => vec![
            window_rect(McpWindow::Heading),
            knob_rect(McpWindow::Heading),
        ],
        GuideTopic::LvlChg => vec![press_rect(mcp_layout::McpPress::LvlChg)],
        GuideTopic::HdgSel => vec![press_rect(mcp_layout::McpPress::HdgSel)],
        GuideTopic::Approach => vec![press_rect(mcp_layout::McpPress::Approach)],
        GuideTopic::AltHold => vec![press_rect(mcp_layout::McpPress::AltHold)],
        GuideTopic::Vs => vec![press_rect(mcp_layout::McpPress::Vs)],
        GuideTopic::AltWindow => vec![
            window_rect(McpWindow::Altitude),
            knob_rect(McpWindow::Altitude),
        ],
        GuideTopic::VsWindow => vec![
            window_rect(McpWindow::VerticalSpeed),
            mcp_layout::VS_WHEEL,
        ],
        GuideTopic::Disengage => vec![mcp_layout::DISENGAGE],
        GuideTopic::Decor => {
            let mut v: Vec<PRect> = mcp_layout::DECOR_CAPS.iter().map(|(r, _)| *r).collect();
            v.extend(mcp_layout::DECOR_ROUND.iter().map(|(r, _)| *r));
            v.push(mcp_layout::COURSE_L_WINDOW);
            v.push(mcp_layout::COURSE_L_KNOB);
            v.push(mcp_layout::COURSE_R_WINDOW);
            v.push(mcp_layout::COURSE_R_KNOB);
            v.push(mcp_layout::FD_L);
            v.push(mcp_layout::FD_R);
            v
        }
        GuideTopic::Links => Vec::new(),
    }
}

/// The prose rows for one topic: the diagram's blank rows on top, then the
/// title and its lines. The topic index rides the bottom hint row.
pub fn lines(topic: usize) -> Vec<PageLine> {
    let t = &TOPICS[topic.min(TOPIC_COUNT - 1)];
    let mut out = vec![PageLine::blank(); IMAGE_ROWS];
    out.push(PageLine::heading(t.title));
    for line in t.lines {
        if line.is_empty() {
            out.push(PageLine::blank());
        } else {
            out.push(PageLine::body(*line));
        }
    }
    out.push(PageLine::blank());
    out.push(PageLine::hint(format!(
        "[{}] of [{}] - click the panel or walk with the arrows",
        topic.min(TOPIC_COUNT - 1) + 1,
        TOPIC_COUNT
    )));
    out
}

/// A hotspot on the diagram, positioned off the layout tables.
fn spot_node(rect: PRect, scale: f32) -> Node {
    let (x, y, w, h) = rect.scaled(scale);
    Node {
        position_type: PositionType::Absolute,
        left: Val::Px(x),
        top: Val::Px(y),
        width: Val::Px(w),
        height: Val::Px(h),
        border: UiRect::all(Val::Px(1.5)),
        border_radius: BorderRadius::all(Val::Px(4.0 * scale)),
        ..default()
    }
}

/// The diagram layer over the card's top rows: the bake's image plus a
/// hotspot per topic rect. Spawned once per menu visit; `sync_guide` shows
/// it only while the guide page is on top.
pub(crate) fn spawn_guide_layer(
    mut commands: Commands,
    assets: Res<AssetServer>,
    card: Query<Entity, With<crate::menu::CardRoot>>,
) {
    let Ok(card) = card.single() else {
        return;
    };
    let image = assets.load("ui/737_mcp_3d.png");
    let scale = GUIDE_W / mcp_layout::CROP_W;
    commands
        .spawn((
            GuideLayer,
            Pickable::IGNORE,
            ImageNode::new(image),
            Node {
                position_type: PositionType::Absolute,
                left: Val::Percent(50.0),
                top: Val::Px(10.0),
                width: Val::Px(GUIDE_W),
                height: Val::Px(GUIDE_W / mcp_layout::CROP_W * mcp_layout::CROP_H),
                display: Display::None,
                ..default()
            },
            UiTransform {
                translation: Val2::percent(-50.0, 0.0),
                ..default()
            },
            DespawnOnExit(AppState::Menu),
            ChildOf(card),
        ))
        .with_children(|panel| {
            // The bake carries hardware only - the engraved labels and the
            // seven-segment glyphs are the same overlays the live panel
            // wears, and the digits read the real MCP, so the diagram shows
            // the windows exactly as the last flight left them.
            crate::mcp_ui::spawn_labels(panel, scale);
            for window in [
                McpWindow::Speed,
                McpWindow::Heading,
                McpWindow::Altitude,
                McpWindow::VerticalSpeed,
            ] {
                crate::mcp_ui::spawn_window(panel, window, scale);
            }
            for (ti, topic) in TOPICS.iter().enumerate() {
                for rect in topic_rects(topic.id) {
                    panel.spawn((
                        Button,
                        Hovered(false),
                        GuideSpot(ti),
                        spot_node(rect, scale),
                        BackgroundColor(Color::NONE),
                        BorderColor::all(Color::NONE),
                    ));
                }
            }
        });
}

/// A click on a hotspot selects its topic.
pub(crate) fn on_guide_activate(
    activate: On<Activate>,
    spots: Query<&GuideSpot>,
    mut state: ResMut<McpGuideState>,
) {
    if let Ok(spot) = spots.get(activate.entity) {
        state.topic = spot.0;
    }
}

/// Show the layer only on the guide page; keep the selected (and hovered)
/// hotspot lit so the panel always points at the prose.
fn sync_guide(
    shown: Res<McpGuideState>,
    state: Res<McpGuideState>,
    layer: Query<&mut Node, With<GuideLayer>>,
    spots: Query<(&GuideSpot, &Hovered, &mut BorderColor), With<Button>>,
) {
    ui_theme::set_display::<GuideLayer>(
        layer,
        if shown.shown {
            Display::Flex
        } else {
            Display::None
        },
    );
    for (spot, hovered, mut border) in spots {
        let want = if state.topic == spot.0 {
            ui_theme::accent()
        } else if hovered.0 {
            ui_theme::accent_soft()
        } else {
            Color::NONE
        };
        if border.top != want {
            *border = BorderColor::all(want);
        }
    }
}

pub struct McpGuidePlugin;

impl Plugin for McpGuidePlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<McpGuideState>()
            .add_observer(on_guide_activate)
            .add_systems(Update, sync_guide.run_if(in_state(AppState::Menu)));
    }
}

#[cfg(test)]
mod tests {
    use super::{topic_rects, GuideTopic, TOPIC_COUNT, TOPICS};

    /// Every interactive region of the live layout is covered by some
    /// topic's rects: the six mode buttons, the A/T, all four windows,
    /// all three knobs, the V/S wheel and the disengage bar.
    #[test]
    fn every_control_is_clickable_somewhere() {
        use crate::mcp_layout::{self, PRect};

        let all: Vec<PRect> = TOPICS
            .iter()
            .flat_map(|t| topic_rects(t.id))
            .collect();
        for (rect, _, label) in mcp_layout::BUTTONS {
            assert!(
                all.contains(rect),
                "{label} has no guide topic"
            );
        }
        assert!(
            all.contains(&mcp_layout::AT_SWITCH),
            "the A/T switch has no topic"
        );
        for (rect, _, label) in mcp_layout::WINDOWS {
            assert!(all.contains(rect), "{label} has no topic");
        }
        for (i, (rect, _)) in mcp_layout::KNOBS.iter().enumerate() {
            assert!(all.contains(rect), "knob {i} has no topic");
        }
        assert!(
            all.contains(&mcp_layout::VS_WHEEL),
            "the V/S wheel has no topic"
        );
        assert!(
            all.contains(&mcp_layout::DISENGAGE),
            "the disengage bar has no topic"
        );
    }

    /// The prose is menu-grade: ASCII (the font is ASCII-only), inside the
    /// line budget, and never empty.
    #[test]
    fn the_prose_is_ascii_and_filled() {
        for topic in TOPICS {
            assert!(!topic.lines.is_empty(), "{} has no prose", topic.title);
            for line in topic.lines {
                assert!(line.is_ascii(), "non-ASCII in {}: {line}", topic.title);
                assert!(line.chars().count() <= 88, "line too long: {line}");
            }
        }
    }

    /// The topic index wraps through the menu's cursor; `lines` clamps.
    #[test]
    fn the_topic_index_clamps() {
        let out = super::lines(TOPIC_COUNT + 3);
        assert!(out.len() > super::IMAGE_ROWS + 2, "the clamped topic renders");
        let links = TOPICS.last().expect("the summary topic");
        assert_eq!(links.id, GuideTopic::Links, "the summary rides last");
    }
}
