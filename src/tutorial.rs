//! Per-aircraft tutorial pages for the start menu.
//!
//! Each page walks through the same three things: what the sim does today,
//! what the real aircraft's procedure looks like, and which systems are not
//! modelled yet (marked `TODO`). The game-key rows are generated from the
//! aircraft's current [`Keymap`], so a rebind in SETTINGS shows up here too.
//!
//! The built-in font is ASCII-only, so every string in this module has to stay
//! ASCII. The long text is shown through a fixed-height window; `Up/Down`
//! scroll one line and `PageUp/PageDown` one screen.

use crate::aircraft_profile::{self, Action, ToggleAction};
use crate::config::Keymap;
use crate::input::Bindings;

/// Body lines visible at once. Chosen so the page still fits a 720p window at
/// the menu's 20 px font, with the header and padding.
pub const ROWS: usize = 20;

/// Highest valid scroll offset for `id` under the current `keymap`.
pub fn max_scroll(id: &str, keymap: &Keymap) -> usize {
    lines(id, keymap).len().saturating_sub(ROWS)
}

/// Render the visible window of the aircraft's tutorial, with a page counter.
pub fn render(id: &str, keymap: &Keymap, scroll: usize) -> String {
    let lines = lines(id, keymap);
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
        "\nGAME = what this sim does now.  REAL = the actual aircraft.  TODO = not modelled.",
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

/// The `pos` / `neg` key pair for one axis, ordered so the label reads
/// naturally: e.g. pitch is `W (up) / S (down)`.
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
    p.row("Pitch", axis(b, Action::Elevator, "up", "down"));
    p.row("Roll", axis(b, Action::Aileron, "left", "right"));
    p.row("Yaw", axis(b, Action::Rudder, "left", "right"));
    p.row("Throttle", axis(b, Action::Throttle, "more", "less"));
    p.row("Pitch trim", axis(b, Action::PitchTrim, "down", "up"));
    p.row("Flaps", axis(b, Action::Flap, "down", "up"));
    p.row("Brakes", axis(b, Action::BrakeLeft, "", ""));
}

/// The rows every aircraft has after its type-specific controls.
fn common_tail_rows(p: &mut Page, b: &Bindings) {
    p.row("Re-trim", toggle(b, ToggleAction::Trim));
    p.row("Cruise reset", toggle(b, ToggleAction::Cruise));
    p.row("Reset runway", "R  (fixed key)");
    p.row("View / HUD", "C / H  (fixed keys)");
    p.row("Back to menu", "Esc  (fixed key)");
}

// --- Cessna 172 --------------------------------------------------------------

fn c172(keymap: &Keymap) -> Vec<String> {
    let b = Bindings::for_aircraft("c172p", keymap);
    let mut p = Page::new();

    p.heading("GAME vs REAL");
    p.line("  This page covers both. GAME rows are what this sim does now.");
    p.line("  REAL rows are the real aircraft's procedure. TODO rows list");
    p.line("  systems that are not modelled yet.");
    p.line("  The Cessna has fixed gear and no autopilot, so there is no");
    p.line("  gear key and G does nothing.");

    p.heading("GAME CONTROLS (your current keys)");
    fixed_wing_rows(&mut p, &b);
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

    p.heading("HOME FIELD (GAME)");
    p.line("  MYFS has runway 36/18, 1200 m. R puts you on the runway,");
    p.line("  heading 360, about 140 m past the 36 threshold.");
    p.line("  PAPI is left of the runway: 2 red / 2 white on a 3-degree");
    p.line("  path, all white high, all red low. The windsock is on the");
    p.line("  right. Taxiway, apron, hangar and tower are east of the runway.");
    p.line("  Landing scoring is not modelled yet; it is the next step.");

    p.heading("FIRST FLIGHT (GAME)");
    p.line("  1. You start parked on the runway with 60% power. Hold Ctrl");
    p.line("     to pull the throttle back before taxiing.");
    p.line("  2. Taxi with a little power (tap Shift). Steer with Q/E and");
    p.line("     hold B to brake. The brake is spring-loaded.");
    p.line("  3. Line up, press F for about 10 degrees of flap, then hold");
    p.line("     Shift for full power.");
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

    p.heading("NOT MODELLED YET (TODO)");
    p.line("  TODO fuel selector, mixture, primer, magnetos, carb heat.");
    p.line("  TODO engine start/shutdown and oil-temperature limits.");
    p.line("  TODO nosewheel steering and differential braking.");
    p.line("  TODO flap detents: the sim's flap lever is continuous.");
    p.line("  TODO stall warning, spins, carb icing and weight balance.");
    p.line("  TODO ATC, traffic and a real navigation stack.");

    p.0
}

// --- F-22 --------------------------------------------------------------------

fn f22(keymap: &Keymap) -> Vec<String> {
    let b = Bindings::for_aircraft("f22", keymap);
    let mut p = Page::new();

    p.heading("GAME vs REAL");
    p.line("  The real F-22 is fly-by-wire; here you drive pitch, roll and");
    p.line("  yaw directly and the FDM enforces its own limits.");
    p.line("  Realistic uses the normal F-22 aero and TVC model. Arcade");
    p.line("  spools the burner faster, manages energy automatically and");
    p.line("  is easier to handle. Choose the mode in SETTINGS.");

    p.heading("GAME CONTROLS (your current keys)");
    fixed_wing_rows(&mut p, &b);
    p.row("Landing gear", toggle(&b, ToggleAction::Gear));
    p.row("Thrust vector", toggle(&b, ToggleAction::Tvc));
    common_tail_rows(&mut p, &b);

    p.heading("FIRST FLIGHT (GAME)");
    p.line("  1. You start parked, gear down, throttle at 60%. Hold Ctrl");
    p.line("     to idle, then taxi with short taps of Space.");
    p.line("  2. Takeoff: full Space, then hold W at about 150-160 kt to");
    p.line("     rotate. Press U to raise the gear once climbing.");
    p.line("  3. Arcade mode manages the burner: straight flight spools it");
    p.line("     to full, a hard pull drains it toward cruise. Tap Space");
    p.line("     to top it up. In Realistic the lever stays where you set");
    p.line("     it, so watch the speed.");
    p.line("  4. J enables the 2D thrust-vectoring nozzles. TVC adds pitch");
    p.line("     authority at low speed and high angle of attack; it is");
    p.line("     not a roll or yaw control.");
    p.line("  5. T re-trims for the current speed and altitude. Y respawns");
    p.line("     airborne at the cruise condition if you get lost.");

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

    p.heading("NOT MODELLED YET (TODO)");
    p.line("  TODO engine start, APU/JFS, fuel and oxygen systems.");
    p.line("  TODO FBW envelope protection, G/AoA limiter and stalls.");
    p.line("  TODO weapons, bay doors, radar, RWR, HMD and sensors.");
    p.line("  TODO afterburner visuals/audio and nozzle animation.");
    p.line("  TODO landing drag chute, aerobraking and brake heat.");
    p.line("  TODO damage, ejection and realistic combat systems.");

    p.0
}

// --- AH-64D ------------------------------------------------------------------

fn ah64(keymap: &Keymap) -> Vec<String> {
    let b = Bindings::for_aircraft("ah1s", keymap);
    let mut p = Page::new();

    p.heading("GAME vs REAL");
    p.line("  The AH-64D has no ailerons or elevator. You fly it with");
    p.line("  cyclic (pitch/roll), collective (thrust) and pedals (yaw).");
    p.line("  Realistic uses the rotor model; Arcade adds hover and");
    p.line("  auto-balance helpers. Select the mode in SETTINGS.");

    p.heading("GAME CONTROLS (your current keys)");
    p.row("Cyclic pitch", axis(&b, Action::Elevator, "up", "down"));
    p.row("Cyclic roll", axis(&b, Action::Aileron, "left", "right"));
    p.row("Pedals", axis(&b, Action::Rudder, "left", "right"));
    p.row("Collective", axis(&b, Action::Collective, "up", "down"));
    p.row("Pitch trim", axis(&b, Action::PitchTrim, "down", "up"));
    p.row("Rotor trim", toggle(&b, ToggleAction::RotorTrim));
    p.row("Re-trim", toggle(&b, ToggleAction::Trim));
    p.row("Hover / VTOL", toggle(&b, ToggleAction::HeliHover));
    p.row("Auto balance", toggle(&b, ToggleAction::HeliBalance));
    p.row("Cruise reset", toggle(&b, ToggleAction::Cruise));
    p.row("Reset runway", "R  (fixed key)");
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

    p.heading("NOT MODELLED YET (TODO)");
    p.line("  TODO engine/APU start, rotor engagement and RPM limits.");
    p.line("  TODO autorotation, governor, hydraulics and damage.");
    p.line("  TODO weapons, TADS/PNVS, IHADSS, sensors, countermeasures.");
    p.line("  TODO dual-crew station and copilot/gunner operation.");
    p.line("  TODO force-trim release, SCAS/autopilot hold modes and fuel.");
    p.line("  TODO weight and balance, dust and ground effect.");

    p.0
}

// --- 737 ---------------------------------------------------------------------

fn b737(keymap: &Keymap) -> Vec<String> {
    let b = Bindings::for_aircraft("737", keymap);
    let mut p = Page::new();

    p.heading("GAME vs REAL");
    p.line("  The 737 is the only aircraft here with an autopilot and");
    p.line("  autothrottle. You start on the runway; the A/P can only");
    p.line("  engage airborne.");

    p.heading("GAME CONTROLS (your current keys)");
    fixed_wing_rows(&mut p, &b);
    p.row("Landing gear", toggle(&b, ToggleAction::Gear));
    p.row("Autopilot", toggle(&b, ToggleAction::Autopilot));
    p.row("HDG SEL", toggle(&b, ToggleAction::HdgSel));
    p.row("ALT HOLD", toggle(&b, ToggleAction::AltHold));
    p.row("V/S", toggle(&b, ToggleAction::VsMode));
    p.row("LVL CHG", toggle(&b, ToggleAction::LvlChg));
    p.row("A/T", toggle(&b, ToggleAction::AutoThrottle));
    p.row("IAS / MACH", toggle(&b, ToggleAction::SpeedMach));
    common_tail_rows(&mut p, &b);

    p.heading("USING THE AUTOPILOT (GAME)");
    p.line("  1. Get airborne and trimmed, then press G. The A/P engages in");
    p.line("     HDG SEL + ALT HOLD and captures the current altitude. It");
    p.line("     refuses to engage on the ground. Pilot stick input drops the");
    p.line("     A/P; throttle input drops only the autothrottle.");
    p.line("  2. Set the MCP windows with the mouse. The panel sits at the");
    p.line("     bottom of the screen: scroll a window for coarse steps and");
    p.line("     click the knob halves for fine steps. In the cockpit, click");
    p.line("     the modelled MCP board to focus it.");
    p.line("  3. 1 = HDG SEL, 2 = ALT HOLD (captures now), 3 = V/S,");
    p.line("     4 = LVL CHG, 5 = A/T, M = IAS/MACH display.");
    p.line("  4. V/S and LVL CHG capture the MCP altitude when reached.");
    p.line("     LVL CHG flies speed on pitch with A/T N1, then hands back to");
    p.line("     SPD when the altitude is captured.");

    p.heading("FIRST FLIGHT (GAME)");
    p.line("  1. You spawn on the runway at 60% power. Set flaps with F.");
    p.line("  2. Full throttle (Shift); at about 140-150 kt tap W to rotate.");
    p.line("     Press U to raise the gear.");
    p.line("  3. Climb at 180-200 kt, or engage the A/P (G) above 1000 ft.");
    p.line("  4. Y gives an airborne trimmed start at 3000 ft / 110 kt if you");
    p.line("     want to practise the A/P without a takeoff.");

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

    p.heading("NOT MODELLED YET (TODO)");
    p.line("  TODO FMC/CDU, LNAV/VNAV, route, performance and fuel.");
    p.line("  TODO ILS/glideslope, autoland and flight-director modes.");
    p.line("  TODO autobrake, spoilers/speedbrake and reverse thrust.");
    p.line("  TODO engine start, pressurisation, hydraulics and failures.");
    p.line("  TODO full 737 systems, checklists and crew coordination.");

    p.0
}

fn lines(id: &str, keymap: &Keymap) -> Vec<String> {
    match aircraft_profile::normalized(id) {
        "f22" => f22(keymap),
        "ah1s" => ah64(keymap),
        "737" => b737(keymap),
        _ => c172(keymap),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn ids() -> [&'static str; 4] {
        ["c172p", "f22", "ah1s", "737"]
    }

    #[test]
    fn every_aircraft_page_covers_game_real_and_todo() {
        for id in ids() {
            let text = render(id, &Keymap::default(), 0);
            assert!(text.contains("GAME CONTROLS"), "{id} has no game keys");
            assert!(text.contains("REAL"), "{id} has no real procedure");
            assert!(text.contains("TODO"), "{id} has no TODO section");
        }
    }

    #[test]
    fn pages_are_ascii_and_fit_the_panel_width() {
        for id in ids() {
            for line in lines(id, &Keymap::default()) {
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
    fn every_page_scrolls() {
        for id in ids() {
            assert!(
                max_scroll(id, &Keymap::default()) > 0,
                "{id} tutorial fits on one screen"
            );
        }
    }

    #[test]
    fn rendering_clamps_the_scroll_and_keeps_a_full_window() {
        for id in ids() {
            let max = max_scroll(id, &Keymap::default());
            let text = render(id, &Keymap::default(), usize::MAX);
            // Header + page line + blank + ROWS body lines + blank + footer.
            assert_eq!(text.lines().count(), ROWS + 5);
            assert!(
                text.contains("TODO"),
                "{id}'s last page is not the TODO page"
            );
            let pages = max / ROWS + 1;
            assert!(text.contains(&format!("page {pages}/{pages}")), "{text}");
        }
    }

    #[test]
    fn game_key_rows_follow_the_current_bindings() {
        let mut keymap = Keymap::default();
        keymap.set_axis(
            Action::Elevator,
            bevy::prelude::KeyCode::KeyK,
            Some(bevy::prelude::KeyCode::KeyI),
        );
        let text = render("c172p", &keymap, 0);
        assert!(text.contains("K (up) / I (down)"), "{text}");
        // The F-22's throttle really is the space bar.
        assert!(render("f22", &Keymap::default(), 0).contains("Space (more)"));
    }

    #[test]
    fn helicopter_page_has_helicopter_controls() {
        let text = render("ah1s", &Keymap::default(), 0);
        assert!(text.contains("Collective"));
        assert!(text.contains("Hover / VTOL"));
        assert!(text.contains("Rotor trim"));
    }
}
