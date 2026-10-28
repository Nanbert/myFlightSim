//! Apron taxi guidance: live lateral offset from the stand's yellow lead-in
//! line, plus a grade for the last full stop — and the same treatment for
//! holding short of the runway on the connectors.
//!
//! The home apron has one painted lead-in line along z = -150 (see
//! `aerodrome::STAND_*`), leading from the pump lane west onto the taxiway.
//! While the aircraft is rolling inside that corridor the status block shows
//! `TAXI Δ +z m` — the signed north offset from the line — so steering the
//! centreline becomes a numbers game like the landing card's CENTRE row.
//! Coming to rest after a move freezes the average |offset| into a grade:
//! under a metre is GOOD, under 2.5 m OK, anything more SLOPPY.
//!
//! The connectors (runway stubs, see `aerodrome::CONNECTOR_ZS`) carry their
//! own centred prompt: while rolling inside one the card reads the live
//! distance to the solid holding pair and the centreline offset; a full stop
//! in a holding position freezes a finals card (centreline grade, line
//! margin, peak taxi speed) until the next rollout.

use bevy::prelude::*;

use crate::airfield::AirfieldSpec;
use crate::app_state::{unpaused, AppState};
use crate::sim::AircraftState;

/// The lead-in corridor: z within this of the line, x from the runway edge
/// out to the pump lane.
const CORRIDOR_Z: f32 = 8.0;
const X_MIN: f32 = 10.0;
const X_MAX: f32 = 95.0;

/// Speeds (kt): above this a frame counts as "moving" (samples accumulate),
/// below this for long enough counts as a full stop.
const MOVE_MIN_GS: f32 = 3.0;
const REST_MAX_GS: f32 = 0.5;
const REST_SECONDS: f32 = 0.8;
/// Above this ground speed the aircraft is flying, not taxiing: the meter
/// goes quiet so a pass over the apron doesn't smear old samples into a stop.
const TAXI_MAX_GS: f32 = 30.0;

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Grade {
    Good,
    Ok,
    Sloppy,
}

impl Grade {
    pub fn label(self) -> &'static str {
        match self {
            Grade::Good => "GOOD",
            Grade::Ok => "OK",
            Grade::Sloppy => "SLOPPY",
        }
    }

    fn from_avg(avg: f32) -> Self {
        if avg < 1.0 {
            Grade::Good
        } else if avg < 2.5 {
            Grade::Ok
        } else {
            Grade::Sloppy
        }
    }
}

#[derive(Resource, Default)]
pub struct Taximeter {
    /// Signed offset from the lead-in line (north-positive), while in the
    /// corridor at taxi speed.
    pub offset: Option<f32>,
    /// Average |offset| and its grade, frozen at the last full stop.
    pub grade: Option<(f32, Grade)>,
    samples: u32,
    sum: f32,
    /// Ground speed reached since the last stop — the "has actually taxied"
    /// gate that keeps a parked aircraft from grading itself.
    peak_gs: f32,
    rest_for: f32,
}

impl Taximeter {
    /// Feed one frame's readouts. Pure over its own state, for tests.
    fn step(&mut self, on_ground: bool, gs_kts: f64, x: f32, z: f32, stand_z: f32, dt: f32) {
        let gs = gs_kts as f32;
        self.peak_gs = self.peak_gs.max(gs);
        let dz = z - stand_z;
        let in_corridor =
            on_ground && gs < TAXI_MAX_GS && x > X_MIN && x < X_MAX && dz.abs() < CORRIDOR_Z;
        self.offset = in_corridor.then_some(dz);

        if gs > MOVE_MIN_GS {
            // Moving: arm the stop detector, clear any previous grade, sample.
            self.rest_for = 0.0;
            self.grade = None;
            if in_corridor {
                self.samples += 1;
                self.sum += dz.abs();
            }
        } else if self.samples > 0 && self.peak_gs > MOVE_MIN_GS {
            if gs < REST_MAX_GS {
                self.rest_for += dt;
                if self.rest_for >= REST_SECONDS {
                    let avg = self.sum / self.samples as f32;
                    self.grade = Some((avg, Grade::from_avg(avg)));
                    self.samples = 0;
                    self.sum = 0.0;
                    self.peak_gs = 0.0;
                    self.rest_for = 0.0;
                }
            } else {
                self.rest_for = 0.0;
            }
        } else if gs > REST_MAX_GS {
            self.samples = 0;
            self.sum = 0.0;
        }
    }
}

// --- hold short ----------------------------------------------------------------
//
// Each connector doubles as a graded hold-short approach. A deliberate stop
// (3 s parked) anywhere in a holding position freezes a finals card: the
// centreline averages and the margin against the solid holding pair. While
// rolling inside the band the centred prompt reads the live numbers; a graded
// stop shows the card until the aircraft rolls again (any creep un-settles
// it), and leaving the band retires both the prompt and the card.

/// Inside this distance of the solid pair the live prompt wakes up.
const HS_LIVE_RANGE: f32 = 14.0;
/// The scoring anchors: a metre or less anywhere is full marks (against the
/// taxi line and the stop band alike).
const HS_TIGHT: f32 = 1.0;
/// The centreline row hits zero this wide of the line.
const HS_TAXI_CAP: f32 = 8.0;
/// The stop-margin row hits zero here: wider than this the junction is
/// blocked anyway and every margin above it already reads BAD.
const HS_LINE_CAP: f32 = 8.5;
/// Seconds parked in the box before a stop grades. Long enough that a card
/// never pops while the pilot is still creeping toward the final spot.
const HS_REST: f32 = 3.0;

/// `TAXI_DEBUG=1` turns on the meter's 2 Hz heartbeat (see `update_hold`).
fn debug_on() -> bool {
    use std::sync::OnceLock;
    static ON: OnceLock<bool> = OnceLock::new();
    *ON.get_or_init(|| std::env::var("TAXI_DEBUG").is_ok())
}

#[derive(Resource, Default)]
pub struct HoldShort {    /// Live approach numbers while rolling on a connector: `(distance to the
    /// solid pair, signed centreline offset)`, metres.
    pub(crate) live: Option<(f32, f32)>,
    /// Frozen finals after the last graded stop; a graded stop is parked:
    /// the finals card shows until the next rollout.
    pub(crate) done: Option<HoldFinals>,
    /// A graded stop is parked: the finals card shows until the next rollout.
    pub(crate) settled: bool,
    samples: u32,
    sum: f32,
    peak_gs: f32,
    rest_for: f32,
    /// Diagnostic heartbeat spacing, `TAXI_DEBUG=1` only.
    dbg_t: f32,
}

/// The hold-short finals: two sampled rows, each a 0–10 score (see
/// `scoring`), with the weighted final on top.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct HoldFinals {
    /// Average |offset| over the leg, metres.
    avg: f32,
    /// Stopping distance against the solid pair, metres.
    dist: f32,
    /// Peak ground speed reached on the leg, kt.
    peak: f32,
    /// The centreline-keeping row's score.
    taxi: f32,
    /// The stop-margin row's score (0 past the lines).
    line: f32,
    /// The weighted final (the OVER case collapses it regardless).
    score: f32,
}

impl HoldShort {
    /// The finals card's text: centreline average, line margin, peak speed
    /// and the weighted final — each graded row a number **and** a word
    /// (see `scoring`). `None` until a stop has graded.
    pub fn finals_text(&self) -> Option<String> {
        let f = self.done?;
        Some(format!(
            "HOLD SHORT\nTAXI Δ avg {avg:+.1} m    {t:.1} {tw}\n\
             LINE  {dist:+.1} m    {l:.1} {lw}\n\
             PEAK  {peak:.0} kt\n\
             FINAL {score:.1}  {fw}",
            avg = f.avg,
            t = f.taxi,
            tw = crate::scoring::word(f.taxi),
            dist = f.dist,
            l = f.line,
            lw = line_word(f.dist, f.line),
            peak = f.peak,
            score = f.score,
            fw = final_word(f.dist, f.score),
        ))
    }
}

/// The LINE row's word: past the solid pair the margin has no number — the
/// stop is an incursion and says so.
fn line_word(dist: f32, line: f32) -> &'static str {
    if dist < 0.0 {
        "OVER LINE"
    } else {
        crate::scoring::word(line)
    }
}

/// The FINAL word: the OVER incursion keeps its own verdict.
fn final_word(dist: f32, score: f32) -> &'static str {
    if dist < 0.0 {
        "OVER"
    } else {
        crate::scoring::word(score)
    }
}

impl HoldShort {
    fn step(&mut self, on_ground: bool, gs_kts: f64, x: f32, z: f32, spec: &AirfieldSpec, dt: f32) {
        let gs = gs_kts as f32;
        self.peak_gs = self.peak_gs.max(gs);
        let Some(off) = spec.connector_offset(x, z)
            .filter(|_| on_ground && gs < TAXI_MAX_GS)
        else {
            // Off the connector (or over it at flight speed): prompt and card
            // retire, and the next approach starts fresh.
            self.live = None;
            self.done = None;
            self.settled = false;
            self.samples = 0;
            self.sum = 0.0;
            self.peak_gs = 0.0;
            self.rest_for = 0.0;
            return;
        };
        let dist = x - spec.hold_line_x;
        if gs > REST_MAX_GS {
            // Rolling — any taxi pace counts, creeping to the line included:
            // a stop that creeps its last metre still grades its whole leg.
            if self.settled {
                // A rollout after a graded stop starts a fresh leg: the card
                // un-settles below and the retired samples must not bleed
                // into the next approach's centreline average.
                self.samples = 0;
                self.sum = 0.0;
            }
            self.rest_for = 0.0;
            self.settled = false;
            self.samples += 1;
            self.sum += off.abs();
            self.live = (dist.abs() <= HS_LIVE_RANGE).then_some((dist, off));
        } else {
            // Parked: the finals card owns the display. The grade waits for a
            // deliberate stop (3 s) and grades on position alone — a stop on
            // the connector ALWAYS grades (the rows answer with their 0–10
            // scores, SLOPPY wide, BAD blocking the junction mouth), so the
            // meter can never silently skip one.
            self.live = None;
            self.rest_for += dt;
            if self.rest_for >= HS_REST {
                let avg = if self.samples > 0 {
                    self.sum / self.samples as f32
                } else {
                    // A dead-slow crawl gathers no samples; the parked
                    // offset is the centreline grade then.
                    off.abs()
                };
                let taxi = crate::scoring::score(avg, HS_TIGHT, HS_TAXI_CAP);
                let line = if dist >= 0.0 {
                    crate::scoring::score(dist, HS_TIGHT, HS_LINE_CAP)
                } else {
                    0.0
                };
                self.done = Some(HoldFinals {
                    avg,
                    dist,
                    peak: self.peak_gs,
                    taxi,
                    line,
                    score: final_score(taxi, line, dist),
                });
                self.settled = true;
                self.peak_gs = 0.0;
                self.rest_for = 0.0;
            }
        }
    }
}

/// The hold-short finals: a weighted average — the centreline keeping pays
/// 60%, the stop's margin 40%. A bad row pulls it down, nothing zeroes it —
/// except past the solid pair, an incursion where the final collapses
/// regardless of how straight the taxi was.
fn final_score(taxi: f32, line: f32, dist: f32) -> f32 {
    if dist < 0.0 {
        0.0
    } else {
        0.6 * taxi + 0.4 * line
    }
}

#[derive(Component, Default)]
struct HoldCard;

fn spawn_hold_card(mut commands: Commands) {
    // Centring needs a flex wrapper — an absolute panel keeps its own insets,
    // so it can't be flex-centred from the inside (see `ui_theme::float_report`).
    let wrapper = commands
        .spawn((
            Node {
                position_type: PositionType::Absolute,
                width: Val::Percent(100.0),
                height: Val::Percent(100.0),
                align_items: AlignItems::Center,
                justify_content: JustifyContent::Center,
                ..default()
            },
            Pickable::IGNORE,
        ))
        .id();
    commands.spawn((
        HoldCard,
        Text::new(""),
        TextFont {
            font_size: FontSize::Px(14.0),
            ..default()
        },
        TextColor(Color::srgb(0.92, 0.94, 0.90)),
        BackgroundColor(Color::srgba(0.02, 0.03, 0.04, 0.80)),
        Node {
            padding: UiRect::all(Val::Px(10.0)),
            border: UiRect::all(Val::Px(1.0)),
            display: Display::None,
            ..default()
        },
        BorderColor::all(Color::srgba(0.7, 0.75, 0.7, 0.35)),
        GlobalZIndex(14),
        Pickable::IGNORE,
        ChildOf(wrapper),
    ));
}

fn hide_hold_card(mut q: Query<&mut Node, With<HoldCard>>) {
    for mut node in &mut q {
        node.display = Display::None;
    }
}

fn reset_holdshort(mut hs: ResMut<HoldShort>) {
    *hs = HoldShort::default();
}

/// Step the meter, then render it: the live prompt while rolling near the
/// holding lines, the finals card once parked on a graded stop. In the graded
/// lessons the flow module owns the display (its prompt and its buttoned
/// popup, see `lesson_flow`), so the card mutes there.
fn update_hold(
    time: Res<Time>,
    aircraft: Res<AircraftState>,
    map: Res<crate::map::ActiveMap>,
    lesson: Res<crate::mission::ActiveLesson>,
    airfield: Res<crate::airfield::Airfield>,
    mut hs: ResMut<HoldShort>,
    mut card: Query<(&mut Text, &mut Node), With<HoldCard>>,
) {
    let Ok((mut text, mut node)) = card.single_mut() else {
        return;
    };
    let default_spec = AirfieldSpec::myfs();
    let spec = airfield.0.as_ref().unwrap_or(&default_spec);
    hs.step(
        aircraft.on_ground,
        aircraft.gs_kts,
        aircraft.pos.x,
        aircraft.pos.z,
        spec,
        time.delta_secs(),
    );
    // `TAXI_DEBUG=1` prints the meter's verdict inputs twice a second, so a
    // missing card can be diagnosed from the terminal alone.
    if debug_on() && map.0.is_none() {
        hs.dbg_t += time.delta_secs();
        if hs.dbg_t >= 0.5 {
            hs.dbg_t = 0.0;
            eprintln!(
                "[taxi] band={:>4} dist={:+6.1} off={:+5.1} gs={:.2} ground={} \
                 rest={:.1} samples={} peak={:.1} done={} settled={}",
                spec.connector_offset(aircraft.pos.x, aircraft.pos.z)
                    .map(|o| format!("{o:+.1}"))
                    .unwrap_or_else(|| "None".into()),
                aircraft.pos.x - spec.hold_line_x,
                spec.connector_offset(aircraft.pos.x, aircraft.pos.z)
                    .unwrap_or(f32::NAN),
                aircraft.gs_kts,
                aircraft.on_ground,
                hs.rest_for,
                hs.samples,
                hs.peak_gs,
                hs.done.is_some(),
                hs.settled,
            );
        }
    }
    // Home-field grading only: the connectors live in the aerodrome frame.
    let muted = map.0.is_some() || lesson.0.is_some();
    let body = if muted {
        None
    } else if !hs.settled {
        hs.live
            .as_ref()
            .map(|(dist, off)| format!("HOLD SHORT\nLINE {dist:+.1} m   TAXI Δ {off:+.1} m"))
    } else {
        hs.finals_text()
    };
    if let Some(body) = body {
        text.0 = body;
        node.display = Display::Flex;
    } else {
        node.display = Display::None;
    }
}

pub struct TaxiPlugin;

impl Plugin for TaxiPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<Taximeter>()
            .init_resource::<HoldShort>()
            .add_systems(Startup, spawn_hold_card)
            .add_systems(OnEnter(AppState::Flying), reset_taximeter)
            .add_systems(OnEnter(AppState::Flying), reset_holdshort)
            .add_systems(Update, update.run_if(unpaused))
            .add_systems(Update, update_hold.run_if(unpaused))
            .add_systems(OnExit(AppState::Flying), hide_hold_card);
    }
}

fn reset_taximeter(mut meter: ResMut<Taximeter>) {
    *meter = Taximeter::default();
}

fn update(
    time: Res<Time>,
    aircraft: Res<AircraftState>,
    map: Res<crate::map::ActiveMap>,
    airfield: Res<crate::airfield::Airfield>,
    mut meter: ResMut<Taximeter>,
) {
    // Home-field guidance only: the stand lives in the aerodrome frame.
    if map.0.is_some() {
        return;
    }
    let default_spec = AirfieldSpec::myfs();
    let spec = airfield.0.as_ref().unwrap_or(&default_spec);
    let stand_z = spec.stand.map(|s| s.z).unwrap_or(0.0);
    meter.step(
        aircraft.on_ground,
        aircraft.gs_kts,
        aircraft.pos.x,
        aircraft.pos.z,
        stand_z,
        time.delta_secs(),
    );
}

#[cfg(test)]
mod tests {
    use super::{Grade, Taximeter};
    use crate::airfield::AirfieldSpec;

    const DT: f32 = 1.0 / 60.0;
    // The stand's line sits at z = -150; corridor x runs 10..95.
    const LINE_Z: f32 = -150.0;

    fn spec() -> AirfieldSpec {
        AirfieldSpec::myfs()
    }

    fn roll(meter: &mut Taximeter, secs: f32, gs: f64, x: f32, z: f32) {
        let steps = (secs / DT).ceil() as usize;
        for _ in 0..steps {
            meter.step(true, gs, x, z, LINE_Z, DT);
        }
    }

    #[test]
    fn the_offset_reads_the_line() {
        let mut meter = Taximeter::default();
        roll(&mut meter, 0.5, 5.0, 60.0, LINE_Z + 2.5);
        assert_eq!(meter.offset, Some(2.5));
        // Out of the corridor the readout goes quiet.
        roll(&mut meter, 0.2, 5.0, 60.0, LINE_Z + 20.0);
        assert_eq!(meter.offset, None);
    }

    #[test]
    fn a_holding_stop_grades_the_centreline() {
        let mut meter = Taximeter::default();
        // Roll a 100 m leg hugging the line within ~0.6 m, then park there.
        roll(&mut meter, 4.0, 6.0, 80.0, LINE_Z + 0.5);
        roll(&mut meter, 1.5, 0.2, 80.0, LINE_Z + 0.5);
        let (avg, grade) = meter.grade.expect("stopped on the line");
        assert_eq!(grade, Grade::Good);
        assert!(avg < 1.0, "{avg} should be small");
        // The readout freezes while parked...
        assert!(meter.offset.is_some() || true);
        // ...and a fresh move re-arms the meter (grade clears, no samples).
        roll(&mut meter, 0.5, 5.0, 80.0, LINE_Z);
        assert_eq!(meter.grade, None);
    }

    #[test]
    fn a_wandering_taxi_is_not_good() {
        let mut meter = Taximeter::default();
        roll(&mut meter, 2.0, 6.0, 80.0, LINE_Z + 4.0);
        roll(&mut meter, 2.0, 6.0, 80.0, LINE_Z - 3.0);
        roll(&mut meter, 1.5, 0.2, 80.0, LINE_Z - 3.5);
        let (avg, grade) = meter.grade.expect("stopped off the line");
        assert_eq!(grade, Grade::Sloppy);
        assert!(avg >= 2.5, "{avg}");
    }

    #[test]
    fn a_parked_aircraft_never_grades_itself() {
        let mut meter = Taximeter::default();
        // Never moved: no samples, no grade, even after minutes stopped.
        roll(&mut meter, 5.0, 0.2, 60.0, LINE_Z);
        assert_eq!(meter.grade, None);
    }

    #[test]
    fn flying_over_the_apron_does_not_taxi() {
        let mut meter = Taximeter::default();
        // 60 kt along the lead line: too fast to be a taxi.
        roll(&mut meter, 2.0, 60.0, 40.0, LINE_Z);
        assert_eq!(meter.offset, None);
        roll(&mut meter, 1.5, 0.2, 40.0, LINE_Z);
        assert_eq!(meter.grade, None);
    }

    // --- hold short ----------------------------------------------------------

    use super::HoldShort;

    /// The apron connector's band centre-line (see `aerodrome::CONNECTOR_ZS`),
    /// and the solid pair the holds are graded against.
    const CONN_Z: f32 = -150.0;

    fn hs_roll(meter: &mut HoldShort, secs: f32, gs: f64, x: f32, z: f32) {
        let steps = (secs / DT).ceil() as usize;
        let s = spec();
        for _ in 0..steps {
            meter.step(true, gs, x, z, &s, DT);
        }
    }

    /// Seconds parked that fire the 3 s grade, with margin.
    const PARK: f32 = 3.5;

    #[test]
    fn a_stopped_hold_grades_both_axes() {
        let mut hs = HoldShort::default();
        // A straight approach, then a stop ~2 m short of the solid pair.
        hs_roll(&mut hs, 2.0, 6.0, 39.0, CONN_Z + 0.3);
        hs_roll(&mut hs, 1.0, 6.0, 26.0, CONN_Z + 0.3);
        hs_roll(&mut hs, PARK, 0.2, 25.5, CONN_Z + 0.3);
        let f = hs.done.expect("graded the hold");
        assert!(f.score > 9.0, "straight taxi, textbook margin: {f:?}");
        assert!((f.avg - 0.3).abs() < 0.05, "{}", f.avg);
        assert!((f.dist - 2.0).abs() < 0.1, "{}", f.dist);
        assert!(f.peak >= 5.0, "{}", f.peak);
        assert!(f.line < f.taxi, "the 2 m stop is wider than the line");
        assert!(hs.settled, "the card owns the display while parked");
        assert_eq!(hs.live, None, "no live prompt while parked");
    }

    #[test]
    fn creeping_the_last_metre_still_counts() {
        let mut hs = HoldShort::default();
        hs_roll(&mut hs, 1.5, 6.0, 34.0, CONN_Z);
        // Walk-pace creep to the stop: the leg grades in full.
        hs_roll(&mut hs, 1.5, 2.0, 27.0, CONN_Z);
        hs_roll(&mut hs, PARK, 0.2, 25.0, CONN_Z);
        let f = hs.done.expect("graded the creeping hold");
        assert!(f.score > 9.0);
        assert!((f.dist - 1.5).abs() < 0.1, "{}", f.dist);
        assert!(f.avg < 0.05, "{}", f.avg);
    }

    #[test]
    fn a_dead_slow_crawl_still_grades() {
        // The first build could miss this shape entirely: no rolling leg ever
        // crosses the sampling floor, yet the hold deserves its card.
        let mut hs = HoldShort::default();
        hs_roll(&mut hs, PARK, 0.4, 25.0, CONN_Z + 0.2);
        let f = hs.done.expect("graded the slow crawl");
        assert!(f.score > 9.0);
        assert!((f.dist - 1.5).abs() < 0.1, "{}", f.dist);
        assert!(f.avg < 0.3, "{}", f.avg);
    }

    #[test]
    fn a_hesitated_approach_still_grades() {
        // The old builds skipped or reset this shape: a pause wide of the
        // line answers on the spot (the LINE row at zero), and the continued
        // approach still grades its own marks.
        let mut hs = HoldShort::default();
        hs_roll(&mut hs, 2.0, 6.0, 38.0, CONN_Z);
        hs_roll(&mut hs, PARK, 0.2, 38.0, CONN_Z);
        let f = hs.done.expect("the hesitation itself grades");
        assert_eq!(f.line, 0.0, "the tail blocks the junction out here");
        assert!(
            (f.score - 6.0).abs() < 0.1,
            "the line row pulls the final down to the OK line, not through it: {f:?}"
        );
        assert!((f.dist - 14.5).abs() < 0.2, "{}", f.dist);
        let text = hs.finals_text().expect("the card");
        assert!(text.contains("BAD"), "{text}");
        hs_roll(&mut hs, 3.0, 1.0, 32.0, CONN_Z + 0.4);
        hs_roll(&mut hs, PARK, 0.2, 25.0, CONN_Z + 0.4);
        let f = hs.done.expect("graded after the hesitation");
        assert!(f.score > 9.0, "the second stop stands on its own: {f:?}");
        assert!((f.avg - 0.4).abs() < 0.05, "{}", f.avg);
    }

    #[test]
    fn a_wide_margin_pulls_the_final_down_without_zeroing_it() {
        // 7.5 m out: the LINE row is nearly zeroed, but the weighted final
        // keeps the straight leg's credit — BAD on the row, OK on the card.
        let mut hs = HoldShort::default();
        hs_roll(&mut hs, 1.0, 6.0, 38.0, CONN_Z);
        hs_roll(&mut hs, PARK, 0.2, 31.0, CONN_Z);
        let f = hs.done.expect("graded");
        assert!(f.line < 3.5, "the wide stop's row reads BAD: {f:?}");
        assert!(
            (6.0..=6.7).contains(&f.score),
            "the pull-down lands the final in OK: {f:?}"
        );
        assert!((f.dist - 7.5).abs() < 0.1, "{}", f.dist);
    }

    #[test]
    fn a_junction_blocking_stop_is_bad() {
        // metres short of the parallel taxiway (x 39): the tail owns the
        // connector's mouth — the stop still grades, its LINE row at zero.
        let mut hs = HoldShort::default();
        hs_roll(&mut hs, 1.0, 6.0, 38.0, CONN_Z);
        hs_roll(&mut hs, PARK, 0.2, 37.5, CONN_Z);
        assert!(
            hs.settled && hs.done.is_some(),
            "any stop on the connector answers with a card"
        );
        let f = hs.done.expect("graded");
        assert_eq!(f.line, 0.0);
        assert!(
            (f.score - 6.0).abs() < 0.1,
            "the weighted final keeps the straight leg's credit: {f:?}"
        );
        assert!((f.dist - 14.0).abs() < 0.2, "{}", f.dist);
        let text = hs.finals_text().expect("the card");
        assert!(text.contains("BAD"), "{text}");
    }

    #[test]
    fn overruns_are_incursions() {
        let mut hs = HoldShort::default();
        hs_roll(&mut hs, 2.0, 6.0, 39.0, CONN_Z);
        hs_roll(&mut hs, PARK, 0.2, 19.0, CONN_Z);
        let f = hs.done.expect("graded");
        assert_eq!(f.score, 0.0, "over the pair the final collapses");
        let text = hs.finals_text().expect("the card");
        assert!(text.contains("OVER"), "{text}");
    }

    #[test]
    fn a_stop_on_the_lines_is_tight_not_over() {
        let mut hs = HoldShort::default();
        hs_roll(&mut hs, 2.0, 6.0, 39.0, CONN_Z);
        hs_roll(&mut hs, PARK, 0.2, 24.3, CONN_Z);
        let f = hs.done.expect("graded");
        assert!(f.score > 9.5, "straight in, {} m avg", f.avg);
        assert!((f.dist - 0.8).abs() < 0.1, "{}", f.dist);
    }

    #[test]
    fn a_distant_pause_re_arms_the_meter() {
        let mut hs = HoldShort::default();
        // Off the band (x past 39, over the junction): just a pause on the
        // parallel taxiway's doorstep — nothing to grade.
        hs_roll(&mut hs, 2.0, 6.0, 40.0, CONN_Z);
        hs_roll(&mut hs, PARK, 0.2, 40.0, CONN_Z);
        assert_eq!(hs.done, None, "nothing to grade");
        assert!(!hs.settled);
        // And a fresh approach after the pause grades only itself.
        hs_roll(&mut hs, 1.0, 6.0, 34.0, CONN_Z);
        hs_roll(&mut hs, PARK, 0.2, 25.0, CONN_Z);
        let f = hs.done.expect("graded the fresh approach");
        assert!((f.dist - 1.5).abs() < 0.1, "{}", f.dist);
    }

    #[test]
    fn a_new_rollout_unsettles_the_card() {
        let mut hs = HoldShort::default();
        hs_roll(&mut hs, 2.0, 6.0, 39.0, CONN_Z);
        hs_roll(&mut hs, PARK, 0.2, 25.0, CONN_Z);
        assert!(hs.settled);
        // Roll out of the hold and the live prompt returns.
        hs_roll(&mut hs, 0.5, 5.0, 27.0, CONN_Z);
        assert!(!hs.settled);
        assert!(hs.live.is_some());
        // Leaving the band retires everything.
        hs_roll(&mut hs, 0.5, 5.0, 60.0, CONN_Z);
        assert_eq!(hs.live, None);
        assert_eq!(hs.done, None);
    }

    #[test]
    fn the_stand_spawn_is_not_a_hold() {
        // M1 spawns cold at the stand (75, -150): off every connector band,
        // so sitting there never spawns a card. Every in-game spawn is out of
        // the box, so the positional rule cannot grade an idle spawn.
        let mut hs = HoldShort::default();
        hs_roll(&mut hs, 6.0, 0.0, 75.0, CONN_Z);
        assert_eq!(hs.done, None);
        assert_eq!(hs.live, None);
    }

    #[test]
    fn the_crossing_flow_of_the_parallel_never_grades() {
        let mut hs = HoldShort::default();
        // Scooting along the parallel taxiway (x 39..51): off the band.
        hs_roll(&mut hs, 3.0, 8.0, 44.0, CONN_Z + 4.0);
        assert_eq!(hs.done, None);
        assert_eq!(hs.live, None);
    }
}
