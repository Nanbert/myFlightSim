//! Per-aircraft tutorial pages for the start menu.
//!
//! Each page walks the same two tracks: what the sim does today, and what
//! the real aircraft's procedure looks like. The game-key rows are generated
//! from the aircraft's fixed profile in [`crate::aircraft_profile`].
//!
//! The built-in font is ASCII-only, so every string in this module has to stay
//! ASCII. The long text is shown through a fixed-height window; `Up/Down`
//! scroll one line and `PageUp/PageDown` one screen.

use crate::aircraft_profile::{self, Action, ToggleAction};
use crate::menu_page::PageLine;
use crate::input::Bindings;

/// Body lines visible at once. Chosen so the page still fits a 720p window at
/// the menu's 20 px font, with the header and padding.
pub const ROWS: usize = 20;

/// Highest valid scroll offset for `id`.
pub fn max_scroll(id: &str) -> usize {
    lines(id).len().saturating_sub(ROWS)
}

/// Render the visible window of the aircraft's tutorial, with a page counter.
pub fn render(id: &str, scroll: usize) -> String {
    let lines = lines(id);
    let max = lines.len().saturating_sub(ROWS);
    let scroll = scroll.min(max);
    let first = if max == 0 { 1 } else { scroll / ROWS + 1 };
    let pages = if max == 0 { 1 } else { max / ROWS + 1 };

    let mut out = format!("{} - TUTORIAL\n", title(id));
    out.push_str(&format!(
        "page {first}/{pages}   [Up/Down] line   [PgUp/PgDn] page   [Esc] close\n\n"
    ));
    for i in 0..ROWS {
        if let Some(line) = lines.get(scroll + i) {
            out.push_str(line);
        }
        out.push('\n');
    }
    out.push_str(
        "\nGAME = what this sim does now.  REAL = the actual aircraft.",
    );
    out
}

fn title(id: &str) -> &'static str {
    match aircraft_profile::normalized(id) {
        "f22" => "F-22 RAPTOR",
        "ah1s" => "AH-64D APACHE",
        "737" => "BOEING 737",
        _ => "CESSNA 172",
    }
}

/// A small line builder: headings and aligned key rows are the whole page.
struct Page(Vec<String>);

impl Page {
    fn new() -> Self {
        Page(Vec::new())
    }

    fn line(&mut self, text: impl Into<String>) {
        self.0.push(text.into());
    }

    fn blank(&mut self) {
        self.0.push(String::new());
    }

    /// A section heading followed by its rows (no blank line in between).
    fn heading(&mut self, text: &str) {
        self.blank();
        self.0.push(text.to_owned());
    }

    /// `  label         value`, aligned to column 17.
    fn row(&mut self, label: &str, value: impl AsRef<str>) {
        self.0.push(format!("  {label:<14} {}", value.as_ref()));
    }
}

// --- key lookup --------------------------------------------------------------

/// The `pos` / `neg` key pair for one axis, labelled in command order: the
/// positive key first, e.g. pitch is `S (down) / W (up)`.
fn axis(b: &Bindings, action: Action, pos: &str, neg: &str) -> String {
    match b.list.iter().find(|x| x.action == action) {
        Some(x) => match x.neg {
            Some(n) => format!(
                "{} ({pos}) / {} ({neg})",
                aircraft_profile::key_name(x.pos),
                aircraft_profile::key_name(n)
            ),
            None => aircraft_profile::key_name(x.pos),
        },
        None => "n/a".to_owned(),
    }
}

fn toggle(b: &Bindings, action: ToggleAction) -> String {
    b.toggles
        .iter()
        .find(|t| t.action == action)
        .map(|t| aircraft_profile::key_name(t.key))
        .unwrap_or_else(|| "n/a".to_owned())
}

/// The four controls every fixed-wing profile shares, in reading order.
fn fixed_wing_rows(p: &mut Page, b: &Bindings) {
    p.row("Pitch", axis(b, Action::Elevator, "down", "up"));
    p.row("Roll", axis(b, Action::Aileron, "right", "left"));
    p.row("Yaw", axis(b, Action::Rudder, "left", "right"));
    p.row("Throttle", axis(b, Action::Throttle, "more", "less"));
    p.row("Pitch trim", axis(b, Action::PitchTrim, "down", "up"));
    p.row("Flaps", axis(b, Action::Flap, "down", "up"));
    p.row("Brakes", axis(b, Action::BrakeLeft, "", ""));
}

/// The rows every aircraft has after its type-specific controls. Spawns live
/// in the menu now (PLAY under the aircraft, the F-22's MISSIONS row) — the
/// fly-time relocate keys R/P/Y are gone.
fn common_tail_rows(p: &mut Page, b: &Bindings) {
    p.row("Re-trim", toggle(b, ToggleAction::Trim));
    p.row("Spawn states", "MENU (fixed)");
    p.row("View / HUD", "C / H  (fixed keys)");
    p.row("Back to menu", "Esc  (fixed key)");
}

// --- Cessna 172 --------------------------------------------------------------

fn c172() -> Vec<String> {
    let b = Bindings::for_aircraft("c172p");
    let mut p = Page::new();

    p.heading("GAME vs REAL");
    p.line("  This page covers both. GAME rows are what this sim does now.");
    p.line("  REAL rows are the real aircraft's procedure.");
    p.line("  The Cessna has fixed gear and no autopilot, so there is no");
    p.line("  gear key and G does nothing.");

    p.heading("GAME CONTROLS (fixed keys)");
    fixed_wing_rows(&mut p, &b);
    p.row("Starter crank", axis(&b, Action::Starter, "crank", ""));
    p.row("Master switch", toggle(&b, ToggleAction::Master));
    p.row("Exterior lights", toggle(&b, ToggleAction::Lights));
    p.row("Fine control", "Space (hold)");
    common_tail_rows(&mut p, &b);

    p.heading("CHASE VIEW CONTROL DISCS");
    p.line("  The two discs at the bottom of the chase view show the yoke");
    p.line("  (pitch/roll, with the amber mark showing trim) and the rudder");
    p.line("  pedals (yaw, with left/right toe brakes). They are a keyboard");
    p.line("  aid, not real C172 instruments, and are hidden in the cockpit.");
    p.line("  Tap a pitch/roll key for a 4% nudge; hold 120 ms to ramp.");
    p.line("  Release lingers 100 ms, then springs back. Space = fine (0.25x).");

    p.heading("KEYBOARD ASSISTS (SETTINGS)");
    p.line("  SETTINGS has BANK HOLD and TURN COORD, both off by default.");
    p.line("  BANK HOLD captures the bank when you release A/D and holds it.");
    p.line("  TURN COORD adds rudder from the slip ball; your Q/E always wins.");
    p.line("  A cyan BANK or COORD label appears on the matching disc when");
    p.line("  the assist is working. Real C172 has neither; these are aids.");

    p.heading("LESSONS (GAME, MENU > PLAY)");
    p.line("  PLAY no longer jumps straight into the cockpit. It opens the");
    p.line("  SELECT START page - free flight on top, then the graded");
    p.line("  lessons: M1 TAXI starts cold on the apron stand (start, taxi");
    p.line("  out, hold short; the meter grades the stop), M2 TAKEOFF starts");
    p.line("  warm on the runway (the takeoff card grades rotation speed,");
    p.line("  pitch and bank), and M3 LANDING sits you on a 2 NM final with");
    p.line("  flaps 20 (the landing card does the rest). The HUD line shows");
    p.line("  your lesson.");

    p.heading("HOME FIELD (GAME)");
    p.line("  MYFS has runway 36/18, 1200 m. The menu's lesson row under");
    p.line("  the Cessna chooses the spawn: M2 TAKEOFF puts you on the");
    p.line("  runway, heading 360, about 140 m past the 36 threshold.");
    p.line("  PAPI is left of the runway: 2 red / 2 white on a 3-degree");
    p.line("  path, all white high, all red low. The windsock is on the");
    p.line("  right. Taxiway, apron, hangar and tower are east of the runway.");
    p.line("  Both 36 and 18 have PAPI. The orange fuel pump stands on the");
    p.line("  apron: stop the aircraft within a few metres of it and the");
    p.line("  tanks fill to full (370 lb total). After touchdown a report");
    p.line("  card shows rate, centreline, speed, point, bank, bounces, a");
    p.line("  0-100 score and the crosswind. In wind the card also grades");
    p.line("  your crab: land crabbed into the wind (the drift wants about");
    p.line("  asin(xwind/65) degrees of nose offset) and a perfect card");
    p.line("  stays perfect; a straight or crosswise landing loses points.");
    p.line("  The status block always shows the wind: x.kt levels blow at");
    p.line("  300 deg, so runway 36 takes 0.87 x kt from the left.");

    p.heading("TAXI (GAME, MENU > PLAY > M1 TAXI)");
    p.line("  M1 TAXI spawns you on the apron stand, nose west on the yellow");
    p.line("  lead-in line, cold and dark - the full taxi drill from the ramp.");
    p.line("  Follow the line west, join the taxiway, hold short of 36 at");
    p.line("  the yellow bars, then line up. At night the blue edge lights");
    p.line("  tell the same story the paint does. Rolling near the line,");
    p.line("  the status block shows TAXI +/- m; come to rest on the paint");
    p.line("  within a metre and the meter grades the stop GOOD. The pump");
    p.line("  lane branches off the lead-in line to the refueller.");

    p.heading("COLD START (GAME, MENU > PLAY > M1)");
    p.line("  M1 TAXI hands you a truly shut-down Cessna and the start flow");
    p.line("  becomes real (PLAY and the other lessons spawn warm and");
    p.line("  running instead):");
    p.line("  1. M turns the master on. The COLD START CHECKLIST panel in");
    p.line("     the bottom-left corner ticks the steps off as you do them.");
    p.line("  2. Hold I: the starter cranks the prop. Two seconds in, the");
    p.line("     engine catches, spooling on its own inertia.");
    p.line("  3. Release I once she runs. She keeps running without a");
    p.line("     starter, like a real magneto engine. The panel then hands");
    p.line("     you the BEFORE TAKEOFF list: flaps 10, throttle full, and");
    p.line("     it closes when the wheels leave.");
    p.line("  4. Running out of fuel in flight flames out (RPM winds down,");
    p.line("     thrust zero). The start checklist re-opens by itself.");
    p.line("     Restart: taxi to the pump (it tops the tanks), then M +");
    p.line("     hold I. A normal flight never sees this - the tanks last");
    p.line("     hours of full-power flying even from the 200-lb spawn");
    p.line("     load - so the pump is for discipline, not emergencies.");

    p.heading("NIGHT FLIGHT (GAME, SETTINGS TIME OF DAY)");
    p.line("  GLOBAL settings has a TIME OF DAY row: DAY, DUSK or NIGHT.");
    p.line("  Dusk parks the sun on the horizon; night trades it for");
    p.line("  moonlight and a starfield, and the ground goes dark around");
    p.line("  the field lights: white runway edges, green thresholds, red");
    p.line("  end bars, blue taxiway lines, apron floods and the tower");
    p.line("  beacon. The setting is live - flip it mid-flight.");
    p.line("  L is the exterior-lights switch: nav tips, tail light,");
    p.line("  flashing beacon and strobes, plus a landing spotlight. At");
    p.line("  dusk/night they switch on by themselves; L overrides. The");
    p.line("  landing beam is real light - outside its cone you only have");
    p.line("  the runway lights, so keep them centred.");

    p.heading("ILS APPROACH (GAME)");
    p.line("  Both runways have a full ILS: localizer plus 3-degree");
    p.line("  glideslope, radiated from the far end, usable from about");
    p.line("  5 NM out. The CDI (bottom-right of the panel) shows it:");
    p.line("  the course card sits under the top index at the current");
    p.line("  beam course (card 0 = runway 36 from the south, 18 = runway");
    p.line("  18 from the north); the white LOC needle sways about the");
    p.line("  centre - tilt it back level with Q/E to hold course - and");
    p.line("  the amber glideslope pointer slides along the right dot");
    p.line("  column, up when you are below the beam. Fly toward the");
    p.line("  pointers: needle left means the course lies to the left.");
    p.line("  A white TO flag shows while the beam holds you; outside the");
    p.line("  volume the needles centre and amber OFF takes its place.");
    p.line("  Deviation dots: half scale is two dots = 1.25 deg localizer,");
    p.line("  0.35 deg glideslope. The same numbers drive the autopilot's");
    p.line("  beams on the 737, so both aircraft agree on the approach.");

    p.heading("FIRST FLIGHT (GAME)");
    p.line("  1. You start parked on the runway with 60% power. Hold Ctrl");
    p.line("     to pull the throttle back before taxiing.");
    p.line("  2. Taxi with a little power (tap Shift). Steer with Q/E and");
    p.line("     hold B to brake. The brake is spring-loaded.");
    p.line("  3. Line up, press F once for 10 degrees of flap (the flap");
    p.line("     lever has four detents: F/V step UP/10/20/30 deg), then");
    p.line("     hold Shift for full power.");
    p.line("  4. At 55-60 kt, tap W to raise the nose. Keep the runway");
    p.line("     centreline with small Q/E inputs.");
    p.line("  5. Climb at 70-80 kt. At about 1000 ft AGL, lower the nose");
    p.line("     and let the speed build to 100-110 kt.");
    p.line("  6. Press T when the attitude is stable. It re-solves trim");
    p.line("     while keeping the current airspeed and position.");
    p.line("  7. To land, reduce power early, set flaps with F, and cross");
    p.line("     the threshold at 60-65 kt. Flare with W, then brake.");

    p.heading("REAL C172 PROCEDURE (PUBLIC INFO)");
    p.line("  Preflight: fuel quantity and sumps, oil, tires, control");
    p.line("  freedom, flaps, static ports, tiedowns. Set takeoff trim.");
    p.line("  Engine start: fuel selector BOTH, mixture RICH, master on,");
    p.line("  beacon on, prime 2-3 strokes cold, throttle 1/4 inch,");
    p.line("  starter. Check oil pressure within 30 s; lean for taxi.");
    p.line("  Taxi: 800-1000 RPM, steer with the nosewheel and rudder,");
    p.line("  use differential brakes for tight turns.");
    p.line("  Run-up: 1800 RPM; check magnetos (max 150 RPM drop each),");
    p.line("  carb heat, vacuum, oil temperature/pressure, idle.");
    p.line("  Takeoff: flaps 0-10 deg, full throttle, rotate 55-60 kt,");
    p.line("  climb 70-80 kt (Vy 74 kt).");
    p.line("  Cruise: 2200-2400 RPM, lean the mixture, 110-125 kt.");
    p.line("  Landing: carb heat, 1500 RPM, flaps 10/20/30 deg, 65-70 kt");
    p.line("  on final, 60-65 kt over the threshold, flare, touchdown,");
    p.line("  brakes. Carb heat cold, mixture idle cutoff, master off.");

    p.0
}

// --- F-22 --------------------------------------------------------------------

fn f22() -> Vec<String> {
    let b = Bindings::for_aircraft("f22");
    let mut p = Page::new();

    p.heading("GAME vs REAL");
    p.line("  The real F-22 is fly-by-wire; here you drive pitch, roll and");
    p.line("  yaw directly and the FDM enforces its own limits.");
    p.line("  FREE FLY uses the normal F-22 aero and TVC model. MISSIONS");
    p.line("  fly the arcade jet: faster burner spool, automatic energy");
    p.line("  management and the combat systems. There is no mode switch.");

    p.heading("GAME CONTROLS (fixed keys)");
    fixed_wing_rows(&mut p, &b);
    p.row("Landing gear", toggle(&b, ToggleAction::Gear));
    p.row("Thrust vector", toggle(&b, ToggleAction::Tvc));
    p.row("Lock / cycle target", toggle(&b, ToggleAction::CycleTarget));
    p.row("Weapon select", toggle(&b, ToggleAction::CycleWeapon));
    p.row("TSD range", toggle(&b, ToggleAction::RadarRange));
    p.row("Fire missile", toggle(&b, ToggleAction::FireMissile));
    p.row("Gun (hold)", toggle(&b, ToggleAction::FireGun));
    p.row("Flares", toggle(&b, ToggleAction::Flares));
    common_tail_rows(&mut p, &b);

    p.heading("FIRST FLIGHT (GAME)");
    p.line("  1. You start parked, gear down, throttle at 60%. Hold Ctrl");
    p.line("     to idle, then taxi with short taps of Space.");
    p.line("  2. Takeoff: full Space, then hold W at about 150-160 kt to");
    p.line("     rotate. Press U to raise the gear once climbing.");
    p.line("  3. FREE FLY gives you a plain throttle lever: it stays where");
    p.line("     you set it, so watch the speed. In MISSIONS the arcade jet");
    p.line("     manages the burner itself: straight flight spools it to");
    p.line("     full, a hard pull drains it toward cruise, Space tops it up.");
    p.line("  4. J enables the 2D thrust-vectoring nozzles. TVC adds pitch");
    p.line("     authority at low speed and high angle of attack; it is");
    p.line("     not a roll or yaw control.");
    p.line("  5. T re-trims for the current speed and altitude. Spawns live");
    p.line("     in the menu: F-22 > FREE FLY or F-22 > MISSIONS.");

    p.heading("AIR COMBAT (ARCADE, MISSIONS)");
    p.line("  A mission map (F-22 > MISSIONS > ARCHIPELAGO / ALPINE) puts two");
    p.line("  circling drones and one orange escort ahead of you. Tab locks");
    p.line("  the nearest target in the nose cone and cycles to the next;");
    p.line("  the locked box turns gold and shows the range. Enter launches");
    p.line("  one of your two AIM-9s - the side bays open by themselves and");
    p.line("  the missile leads the turn. Hold G for the gun, L pops flares.");
    p.line("  K switches AIM-9 / AIM-120: the AIM-120 locks out to 18 km and");
    p.line("  flies farther, but turns less. The box marks the selected store.");
    p.line("  The corner TSD is heading-up (nose at the top); R cycles 10/20/40");
    p.line("  km. Red dots are drones, orange the escort, gold your lock; the");
    p.line("  locked contact also shows range, closure, heading and altitude.");
    p.line("  The escort chases and fires IR missiles. Edge arrows point at");
    p.line("  each inbound round and show its range: break hard or pop flares");
    p.line("  to seduce it. With a locked target inside gun range, a ring");
    p.line("  marks the bullets' lead point: put the nose cross on it. A hit");
    p.line("  respawns you at the mission start with a fresh loadout; kills");
    p.line("  and the wave survive. FREE FLY carries no weapons; enter");
    p.line("  MISSIONS when you want the fight.");

    p.heading("REAL F-22 PROCEDURE (PUBLIC INFO)");
    p.line("  Preflight: walk-around, canopy, ejection seat, fuel and");
    p.line("  oxygen, flight controls, stores and weapon-bay doors.");
    p.line("  Start: APU/JFS, then each F119 engine; avionics and INS");
    p.line("  alignment; flight-control BIT; check nozzles and brakes.");
    p.line("  Taxi: nosewheel steering, brake checks, then line up.");
    p.line("  Takeoff: full afterburner, release brakes, rotate around");
    p.line("  160 kt, gear up, accelerate in MIL power or afterburner.");
    p.line("  Climb: 350-400 kt or Mach 0.9 to the operational altitude.");
    p.line("  The F-22 can supercruise above Mach 1.5 without afterburner;");
    p.line("  ceiling is about 65,000 ft.");
    p.line("  Combat: sensor fusion, BVR missiles, AIM-9X and gun. The");
    p.line("  vectored nozzles help pointing at high AoA, and the FBW");
    p.line("  keeps the jet inside its G and AoA envelope.");
    p.line("  Landing: visual pattern, gear down, about 150-170 kt on");
    p.line("  final, flare and aerobrake, then wheel brakes.");

    p.0
}

// --- AH-64D ------------------------------------------------------------------

fn ah64() -> Vec<String> {
    let b = Bindings::for_aircraft("ah1s");
    let mut p = Page::new();

    p.heading("GAME vs REAL");
    p.line("  The AH-64D has no ailerons or elevator. You fly it with");
    p.line("  cyclic (pitch/roll), collective (thrust) and pedals (yaw).");
    p.line("  Realistic uses the rotor model; Arcade adds hover and");
    p.line("  auto-balance helpers. Select the mode in SETTINGS.");

    p.heading("GAME CONTROLS (fixed keys)");
    p.row("Cyclic pitch", axis(&b, Action::Elevator, "up", "down"));
    p.row("Cyclic roll", axis(&b, Action::Aileron, "left", "right"));
    p.row("Pedals", axis(&b, Action::Rudder, "left", "right"));
    p.row("Collective", axis(&b, Action::Collective, "up", "down"));
    p.row("Pitch trim", axis(&b, Action::PitchTrim, "down", "up"));
    p.row("Rotor trim", toggle(&b, ToggleAction::RotorTrim));
    p.row("Re-trim", toggle(&b, ToggleAction::Trim));
    p.row("Hover / VTOL", toggle(&b, ToggleAction::HeliHover));
    p.row("Auto balance", toggle(&b, ToggleAction::HeliBalance));
    p.row("Spawn states", "MENU (fixed)");
    p.row("View / HUD", "C / H  (fixed keys)");
    p.row("Back to menu", "Esc  (fixed key)");

    p.heading("FIRST FLIGHT (GAME)");
    p.line("  1. You start on the ground with the engine running and the");
    p.line("     collective at zero. Hold Shift to raise collective and");
    p.line("     lift off; the rotor needs a moment to build thrust.");
    p.line("  2. Press Space to solve a hands-free hover. The trim keeps the");
    p.line("     current airspeed, so at a stop it is a hover trim.");
    p.line("  3. Move with the cyclic: tap S to lower the nose and");
    p.line("     accelerate, W to raise it and slow down. Use small inputs.");
    p.line("  4. Steer with Q/E. Add collective (Shift) to climb and reduce");
    p.line("     it (Ctrl) to descend. Press Space again after a stable");
    p.line("     change to re-trim.");
    p.line("  5. Arcade only: J runs a vertical take-off / hover / landing");
    p.line("     state machine; B auto-levels when you release the cyclic.");
    p.line("  6. The realistic helicopter is pitch-unstable. Trim often; a");
    p.line("     centred stick then holds the trimmed flight path.");

    p.heading("REAL AH-64D PROCEDURE (PUBLIC INFO)");
    p.line("  Crew: the pilot flies; the copilot/gunner runs sensors and");
    p.line("  weapons.");
    p.line("  Preflight: rotor, blades, hydraulics, engines, fuel, TADS,");
    p.line("  PNVS, IHADSS, weapons and chaff/flare.");
    p.line("  Start: APU, then both T700 engines; engage the rotor and");
    p.line("  check rotor RPM, hydraulic and electrical systems.");
    p.line("  Before takeoff: hover check, force-trim check, engine and");
    p.line("  systems checks, then hover-taxi or climb out.");
    p.line("  Hover: face into wind, pick reference points, hold altitude");
    p.line("  and heading. Effective translational lift starts near 15-25 kt.");
    p.line("  Flight: cyclic sets attitude and speed; collective sets");
    p.line("  power/altitude; pedals control yaw and turn coordination.");
    p.line("  Landing: approach into wind, reduce speed and descend, settle");
    p.line("  into a hover, then cushion the touchdown.");
    p.line("  Emergency: autorotation with rotor RPM and glide-speed");
    p.line("  management; the governor and force trim help the pilot.");

    p.0
}

// --- 737 ---------------------------------------------------------------------

fn b737() -> Vec<String> {
    let b = Bindings::for_aircraft("737");
    let mut p = Page::new();

    p.heading("GAME vs REAL");
    p.line("  The 737 is the only aircraft here with an autopilot and");
    p.line("  autothrottle. You start on the runway; the A/P can only");
    p.line("  engage airborne.");
    p.line("  TUTORIAL > MCP GUIDE walks the whole panel: click any switch");
    p.line("  and it explains itself - sim meaning, real 737 function, and");
    p.line("  how the modes hand off to each other.");

    p.heading("M1 TAXI - THE GRADED TAXI (GAME)");
    p.line("  PLAY > M1 TAXI seats you at the ZSNM stand, engines idling,");
    p.line("  parking brake holding. The drill: follow the lead-in line");
    p.line("  out, turn left onto the parallel taxiway, taxi 1.6 km south,");
    p.line("  then right onto the runway-36 stub and one clean stop before");
    p.line("  the holding pair.");
    p.line("  The meter grades the whole leg: centreline keeping, the speed");
    p.line("  band (20 kt on the straight parallel, 10 kt on the apron, the");
    p.line("  stub and the turns), thrust discipline (bump the levers to");
    p.line("  break away, then idle), and the stop's nose-to-line margin.");
    p.line("  Nosewheel authority fades with speed - 35 deg at a walk,");
    p.line("  pedal-only 7 deg by 40 kt - so slow down before the turns.");
    p.line("  Wheels on grass or a nose past the solid pair fails the run.");
    p.blank();
    p.line("  SCORE: LINE 0.40 - CENTRELINE 0.30 - SPEED 0.20 - THRUST 0.10.");

    p.heading("M2 TAKEOFF - FLY THE CALLOUTS (GAME)");
    p.line("  PLAY > M2 TAKEOFF lines you up on ZSNM's numbers, engines");
    p.line("  idling. The centre prompt plays the monitoring pilot: set");
    p.line("  flaps 5 and full throttle, then 80 KT - THRUST SET, V1,");
    p.line("  ROTATE ~179, POSITIVE RATE - GEAR UP (U). At the 1,500 ft");
    p.line("  acceleration altitude bring the flaps up (hold V) and let the");
    p.line("  speed build to the 250 kt class.");
    p.line("  The takeoff ends at the handoff: A/P engaged (G) with the");
    p.line("  aeroplane clean, held two quiet seconds - a stick tap drops");
    p.line("  the A/P and reopens the window. Engage between 1,000 and");
    p.line("  3,000 ft, LVL CHG (4) with the MCP altitude set above, the");
    p.line("  heading bug on 360, the A/T on (5): a level-off handoff pays,");
    p.line("  not collapses. Clean up and hand off by 5,000 ft or the run");
    p.line("  fails. The card grades rotation, centreline, the clean-up,");
    p.line("  the climb speed band and the automation handoff itself.");
    p.blank();
    p.line("  SCORE: ROTATION 0.30 - CLEAN-UP 0.20 - AUTOMATION 0.20 - RLW 0.15 - CLIMB 0.15.");

    p.heading("M3 LANDING - THE GRADED ARRIVAL (GAME)");
    p.line("  PLAY > M3 LANDING seats you 4.9 NM out on ZSNM's ILS 36,");
    p.line("  160 kt, flaps 5, gear up. The centre prompt plays the PM:");
    p.line("  flaps 15, gear, flaps 30, slow to Vref+5 - the MCP speed");
    p.line("  window shows how many [ presses take you there. By 1,000 ft");
    p.line("  you must be stable: gear down, flaps 15 or more, inside");
    p.line("  Vref+25, sinking no harder than 1,000 fpm. Anything red there");
    p.line("  is a GO AROUND FAIL naming what broke. Then fly the needles,");
    p.line("  grease it onto the 300 m mark, and stop: reverse out by 80 kt,");
    p.line("  throttles idle, brakes to a full stop. Past 600 fpm at the");
    p.line("  wheels is a hard-landing FAIL; a bounce costs the card.");
    p.blank();
    p.line("  SCORE: TOUCH 0.45 - ROLLOUT 0.20 - GS 0.15 - LOC 0.10 - SPEED 0.10.");

    p.heading("M4 GO-AROUND - FLY THE CALL (GAME)");
    p.line("  PLAY > M4 GO-AROUND seats you 8 km out at landing config,");
    p.line("  140 kt, flaps 30, gear down. The approach is not graded -");
    p.line("  fly it stable, and crossing 1,000 ft the sim makes the call");
    p.line("  for you: GO AROUND. From that frame the drill is graded:");
    p.line("  thrust (SHIFT, or A/P: 4 LVL CHG with the MCP altitude set to");
    p.line("  3,000 first), pitch up 8-20 degrees, POSITIVE RATE - GEAR UP");
    p.line("  (U), FLAPS 15 (F) low, and climb in the band (140-190 kt");
    p.line("  dirty, the 250-class clean) to a level-off at 3,000 ft.");
    p.line("  Keep flying the approach and the run fails; so does sinking");
    p.line("  200 ft past the call, banking past 30 degrees, or bleeding");
    p.line("  below 130 kt dirty. The card freezes at the capture; Enter");
    p.line("  re-arms and the next descent through the gate goes again.");
    p.blank();
    p.line("  SCORE: REACT <=2 s - GEAR <=3 s - FLAPS 15 by 2,000 ft - LEVEL +-150 ft.");

    p.heading("M5 RTO - THE REJECTED TAKEOFF (GAME)");
    p.line("  PLAY > M5 RTO lines you up on ZSNM's numbers, engines idling.");
    p.line("  Set flaps 5 (F) and takeoff thrust, roll through the 80 KT");
    p.line("  crosscheck - and at 100 kt the sim makes the call: STOP.");
    p.line("  From that frame: throttles to idle at once (the REACT clock),");
    p.line("  brakes (B), reverse (K) out early, and hold the centreline to");
    p.line("  a full stop. The card grades the setup, the reaction, how far");
    p.line("  past the call the roll ran, the call-to-stop clock, the");
    p.line("  centreline and the buckets. Roll through V1 or fly the call to");
    p.line("  a liftoff and the run fails. Enter re-arms on the runway.");
    p.blank();
    p.line("  SCORE: REACT <=2 s - PEAK <=110 kt - STOP <=10 s - BUCKETS <=2 s - LINE.");

    p.heading("M6 AUTOMATION - THE COUPLED ARRIVAL (GAME)");
    p.line("  PLAY > M6 AUTOMATION seats you 20 km out and 4 km right of");
    p.line("  the course at 3,000 ft, clean, with the A/P off. From there");
    p.line("  the box flies the whole arrival - window first, button");
    p.line("  second, every time:");
    p.line("  1. G (CMD), MCP ALT 4000 (arrows), then 4 LVL CHG. The climb");
    p.line("     starts; the A/T N1 comes with the mode.");
    p.line("  2. Crossing 3,500 ft ATC calls STOP THE CLIMB - answer with");
    p.line("     2 ALT HOLD at once, or the run fails past 3,900.");
    p.line("  3. HDG SEL is already live (the FMA says so) - set the bug to");
    p.line("     330 (arrows) and the jet turns to intercept the course.");
    p.line("  4. At about 13 km: MCP ALT 1000, V/S 2800 DOWN (PgDn), then");
    p.line("     3. The descent meets the beam out front.");
    p.line("  5. 6 APP arms both beams; set IAS 180 ([ keys) and the A/T");
    p.line("     SPD (5, usually already on) holds it. LOC takes, then G/S -");
    p.line("     established, and the card freezes.");
    p.line("  The A/P flies all of it: a disconnect is a FAIL, a window set");
    p.line("  after the button sends the box the wrong way, a leg that never");
    p.line("  reaches its target is a FAIL naming the leg. The card grades");
    p.line("  the windows, the stop, the intercept, the descent and the");
    p.line("  beams. No Continue - the lesson ends established; land it as");
    p.line("  free flight, or run M3 for the graded touchdown.");
    p.blank();
    p.line("  SCORE: window before press - stop <=3 s - intercept dev <=0.10 - A/T 180.");

    p.heading("GAME CONTROLS (fixed keys)");
    fixed_wing_rows(&mut p, &b);
    p.row("Landing gear", toggle(&b, ToggleAction::Gear));
    p.row("Autopilot", toggle(&b, ToggleAction::Autopilot));
    p.row("HDG SEL", toggle(&b, ToggleAction::HdgSel));
    p.row("ALT HOLD", toggle(&b, ToggleAction::AltHold));
    p.row("V/S", toggle(&b, ToggleAction::VsMode));
    p.row("LVL CHG", toggle(&b, ToggleAction::LvlChg));
    p.row("A/T", toggle(&b, ToggleAction::AutoThrottle));
    p.row("IAS / MACH", toggle(&b, ToggleAction::SpeedMach));
    p.row("Speedbrake", toggle(&b, ToggleAction::SpeedBrake));
    p.row("Reverse", toggle(&b, ToggleAction::Reverse));
    common_tail_rows(&mut p, &b);

    p.heading("USING THE AUTOPILOT (GAME)");
    p.line("  1. Get airborne and trimmed, then press G. The A/P engages in");
    p.line("     HDG SEL + ALT HOLD and captures the current altitude. It");
    p.line("     refuses to engage on the ground. Pilot stick input drops the");
    p.line("     A/P; throttle input drops only the autothrottle.");
    p.line("  2. Set the MCP windows with the mouse. The panel sits at the");
    p.line("     bottom of the screen: scroll a window for coarse steps and");
    p.line("     click the knob halves for fine steps. In the cockpit, click");
    p.line("     the modelled MCP board to raise the MCP panel.");
    p.line("  3. 1 = HDG SEL, 2 = ALT HOLD (captures now), 3 = V/S,");
    p.line("     4 = LVL CHG, 5 = A/T, M = IAS/MACH display, 6 = APP.");
    p.line("  4. V/S and LVL CHG capture the MCP altitude when reached.");
    p.line("     LVL CHG flies speed on pitch with A/T N1, then hands back to");
    p.line("     SPD when the altitude is captured.");

    p.heading("ILS APPROACH WITH APP (GAME)");
    p.line("  MYFS has a full ILS on both runway ends (course 360 / 180).");
    p.line("  1. Fly south of the field, gear down (a quick way in: press Y");
    p.line("     or edit the flight, or just fly out and come back around).");
    p.line("  2. Inside 5 NM of the final the beams are live; the PFD shows");
    p.line("     the deviation scales and the MCP APP light arms both beams.");
    p.line("  3. Intercept: cross the localizer inside 1.6 deg while");
    p.line("     descending (V/S with the bar) and APP captures LOC, then");
    p.line("     G/S once it is on the localizer at the beam.");
    p.line("  4. The A/P flies beams; the A/T holds the MCP speed, then");
    p.line("     RETARDs to idle at 27 ft. Below 50 ft it flares itself and");
    p.line("     drops off at touchdown. Land: spoilers L, reverse K, brake.");
    p.line("     Amber GLIDESLOPE / SINK RATE annunciations watch the sink.");

    p.heading("SPEEDBRAKE AND REVERSE (GAME)");
    p.line("  Speedbrake toggles the flight spoilers: an extra stall of drag");
    p.line("  and a 15% lift cut, both real model coefficients. Use them for");
    p.line("  level fast descents and on the landing roll.");
    p.line("  Reverse arms the thrust buckets; the FDM only bites while the");
    p.line("  wheels carry weight, and it stows itself the moment you leave");
    p.line("  the ground. Keep the throttles near idle and hold reverse to");
    p.line("  about 80 kt, then brakes (B) do the rest. Reverse drops the");
    p.line("  autothrottle, as the retard would on the real jet.");

    p.heading("FIRST FLIGHT (GAME)");
    p.line("  1. You spawn on the runway at 60% power. Set flaps with F.");
    p.line("  2. Full throttle (Shift); at about 140-150 kt tap W to rotate.");
    p.line("     Press U to raise the gear.");
    p.line("  3. Climb at 180-200 kt, or engage the A/P (G) above 1000 ft.");
    p.line("  4. Y gives an airborne trimmed start at 3000 ft / 110 kt if you");
    p.line("     want to practise the A/P without a takeoff.");

    p.heading("CHECKLIST PANEL (GAME)");
    p.line("  The bottom-left panel ticks itself off as you fly: flaps 5 and");
    p.line("  full throttle on the ground, then positive rate, gear up and");
    p.line("  flaps up in the climb. There is nothing to click - it reads the");
    p.line("  airframe directly, and a touch-and-go rolls the ground card");
    p.line("  back open by itself.");
    p.line("  Leaving the ground with the roll established another card");
    p.line("  grades the takeoff: liftoff speed against the PFD's VR line,");
    p.line("  the rotation attitude, bank, and how fast the gear came up.");
    p.line("  Descending through 1000 ft AAL a stability gate counts the");
    p.line("  approach setup: gear down, flaps 15, inside Vref + 25 and a");
    p.line("  gentle sink. UNSTABLE names what to fix before the fence.");

    p.heading("REAL 737 PROCEDURE (PUBLIC INFO)");
    p.line("  Preflight: walk-around, fuel, oil, hydraulics, tires and");
    p.line("  brakes; FMC route, performance and takeoff data; trim.");
    p.line("  Start: pushback, APU/ground air, start both engines, check");
    p.line("  N1/N2, EGT, oil pressure and generator online.");
    p.line("  Taxi: tiller or rudder pedals, autobrake, flight controls.");
    p.line("  Takeoff: flaps 5 usually, TO/GA, V1 call, VR rotate, positive");
    p.line("  rate and gear up, V2+15 climb, then A/P on.");
    p.line("  Climb/cruise: LVL CHG or V/S, FMC LNAV/VNAV, Mach .78, step");
    p.line("  climb as fuel burns; manage pressurisation and fuel.");
    p.line("  Descent/approach: VNAV or LVL CHG, 250 kt below 10,000 ft,");
    p.line("  flaps and gear on schedule, ILS capture.");
    p.line("  Landing: Vref + 5, flare, autobrake, spoilers, reverse, then");
    p.line("  taxi in and shut down.");

    p.0
}

/// One aircraft's INFO page: a short history and the real specifications.
/// Pure facts about the real aeroplane - the lessons and the MCP GUIDE
/// carry the how-to-fly.
struct InfoPage {
    /// The `aircraft_profile` id this page answers to.
    id: &'static str,
    name: &'static str,
    history: &'static [&'static str],
    /// `(label, value)` spec rows, rendered two-column.
    specs: &'static [(&'static str, &'static str)],
}

const INFO_PAGES: &[InfoPage] = &[
    InfoPage {
        id: "c172p",
        name: "CESSNA 172 SKYHAWK",
        history: &[
            "First flown in 1955 and built ever since, the Skyhawk is the",
            "most produced aircraft in history: more than 45,000 airframes,",
            "and the default flight-school machine on every continent. A",
            "tricycle-gear development of the Cessna 170, it trades glamour",
            "for an honest, forgiving wing - which is exactly why it teaches",
            "so well.",
        ],
        specs: &[
            ("FIRST FLIGHT", "1955"),
            ("BUILT", "45,000+"),
            ("WINGSPAN", "36 ft 1 in"),
            ("LENGTH", "27 ft 2 in"),
            ("ENGINE", "Lycoming IO-360, 160 hp"),
            ("CRUISE", "122 kt"),
            ("SERVICE CEILING", "13,500 ft"),
            ("RANGE", "approx 640 NM"),
            ("MTOW", "2,550 lb"),
        ],
    },
    InfoPage {
        id: "737",
        name: "BOEING 737-800 (NG)",
        history: &[
            "First flown in 1967, the 737 grew into the most-built jet",
            "airliner in history - over 11,000 delivered. The NG generation",
            "(1997) brought bigger wings, the CFM56-7B and the winglets this",
            "model wears; the -800 is the type's workhorse on every short",
            "and medium route in the world.",
        ],
        specs: &[
            ("FIRST FLIGHT", "1967 (NG: 1997)"),
            ("DELIVERED", "11,000+ all variants"),
            ("WINGSPAN", "117 ft 5 in (winglets)"),
            ("LENGTH", "129 ft 6 in"),
            ("ENGINES", "2 x CFM56-7B"),
            ("CRUISE", "Mach 0.785"),
            ("SERVICE CEILING", "41,000 ft"),
            ("RANGE", "2,935 NM"),
            ("MTOW", "155,500 lb"),
            ("SEATS", "162-189"),
        ],
    },
    InfoPage {
        id: "f22",
        name: "LOCKHEED MARTIN F-22 RAPTOR",
        history: &[
            "The YF-22 won the Advanced Tactical Fighter fly-off in 1991;",
            "the production F-22 first flew in 1997 and entered service in",
            "2005 as its era's only operational fifth-generation fighter.",
            "Production stopped at 187 aircraft - an air-supremacy fighter,",
            "priced like one. Figures below are the public estimates.",
        ],
        specs: &[
            ("FIRST FLIGHT", "1997 (YF-22: 1990)"),
            ("BUILT", "187"),
            ("WINGSPAN", "44 ft 6 in"),
            ("LENGTH", "62 ft"),
            ("ENGINES", "2 x Pratt & Whitney F119"),
            ("MAX SPEED", "Mach 2.25 (supercruise 1.8)"),
            ("SERVICE CEILING", "approx 65,000 ft"),
            ("MTOW", "83,500 lb"),
        ],
    },
    InfoPage {
        id: "ah1s",
        name: "BOEING AH-64D APACHE LONGBOW",
        history: &[
            "The YAH-64 first flew in 1975; the Apache entered Army service",
            "in 1986, and the Longbow D-model (1997) added the mast-mounted",
            "radar and a new spine of sensors. More than 1,200 Apaches of",
            "all variants have been built, and the type is still in",
            "production.",
        ],
        specs: &[
            ("FIRST FLIGHT", "1975 (YAH-64)"),
            ("BUILT", "1,200+ all variants"),
            ("ROTOR DIAMETER", "48 ft"),
            ("FUSELAGE LENGTH", "49 ft"),
            ("ENGINES", "2 x GE T700-701C"),
            ("CRUISE", "150 kt"),
            ("NEVER-EXCEED", "229 kt"),
            ("SERVICE CEILING", "21,000 ft"),
            ("MTOW", "23,000 lb"),
            ("CREW", "2"),
        ],
    },
];

/// The INFO page: the aircraft's story and its real numbers.
pub fn info_lines(id: &str) -> Vec<PageLine> {
    let normalized = aircraft_profile::normalized(id);
    let Some(page) = INFO_PAGES.iter().find(|p| p.id == normalized) else {
        return vec![
            PageLine::heading("INFO"),
            PageLine::body("No info for this aircraft."),
        ];
    };
    let mut out = vec![PageLine::heading(page.name)];
    for line in page.history {
        if line.is_empty() {
            out.push(PageLine::blank());
        } else {
            out.push(PageLine::body(*line));
        }
    }
    out.push(PageLine::blank());
    out.push(PageLine::heading("SPECS"));
    for (label, value) in page.specs {
        out.push(PageLine::option(*label, *value, false));
    }
    out.push(PageLine::blank());
    out.push(PageLine::hint("public figures - the real aeroplane, not this sim"));
    out
}

fn lines(id: &str) -> Vec<String> {
    match aircraft_profile::normalized(id) {
        "f22" => f22(),
        "ah1s" => ah64(),
        "737" => b737(),
        _ => c172(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn ids() -> [&'static str; 4] {
        ["c172p", "f22", "ah1s", "737"]
    }

    #[test]
    fn every_aircraft_page_covers_game_and_real() {
        for id in ids() {
            // The markers can land on any scroll page (the 737's M1 section
            // pushes GAME CONTROLS past the first window).
            let whole = (0..=max_scroll(id))
                .map(|s| render(id, s))
                .collect::<String>();
            assert!(whole.contains("GAME CONTROLS"), "{id} has no game keys");
            assert!(whole.contains("REAL"), "{id} has no real procedure");
        }
    }

    #[test]
    fn pages_are_ascii_and_fit_the_panel_width() {
        for id in ids() {
            for line in lines(id) {
                assert!(line.is_ascii(), "non-ASCII in {id}: {line}");
                assert!(
                    line.chars().count() <= 88,
                    "{} chars in {id}: {line}",
                    line.chars().count()
                );
            }
        }
    }

    #[test]
    fn every_info_page_tells_the_story_and_the_numbers() {
        for id in ids() {
            let lines = info_lines(id);
            assert!(lines.len() > 8, "{id} info is thin");
            assert!(
                lines
                    .iter()
                    .any(|line| line.left == "SPECS" || line.right == "SPECS"),
                "{id} info has no specs"
            );
            for line in &lines {
                assert!(line.left.is_ascii(), "non-ASCII in {id}: {}", line.left);
                assert!(line.right.is_ascii(), "non-ASCII in {id}: {}", line.right);
                assert!(
                    line.left.chars().count() <= 88 && line.right.chars().count() <= 88,
                    "info line too long in {id}"
                );
            }
        }
    }

    #[test]
    fn every_page_scrolls() {
        for id in ids() {
            assert!(max_scroll(id) > 0, "{id} tutorial fits on one screen");
        }
    }

    #[test]
    fn rendering_clamps_the_scroll_and_keeps_a_full_window() {
        for id in ids() {
            let max = max_scroll(id);
            let text = render(id, usize::MAX);
            // Header + page line + blank + ROWS body lines + blank + footer.
            assert_eq!(text.lines().count(), ROWS + 5);
            assert!(
                text.contains("PROCEDURE"),
                "{id}'s last page is not the procedure section"
            );
            let pages = max / ROWS + 1;
            assert!(text.contains(&format!("page {pages}/{pages}")), "{text}");
        }
    }

    #[test]
    fn game_key_rows_show_the_profile_defaults() {
        let text = render("c172p", 0);
        assert!(text.contains("S (down) / W (up)"), "{text}");
        // The F-22's throttle really is the space bar.
        assert!(render("f22", 0).contains("Space (more)"));
    }

    #[test]
    fn helicopter_page_has_helicopter_controls() {
        let text = render("ah1s", 0);
        assert!(text.contains("Collective"));
        assert!(text.contains("Hover / VTOL"));
        assert!(text.contains("Rotor trim"));
    }
}
