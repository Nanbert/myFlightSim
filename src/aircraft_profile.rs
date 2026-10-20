//! Per-aircraft control profiles: what controls an aircraft has, which keys
//! drive them, and whether it has an autopilot.
//!
//! The profile is the single place that knows an aircraft's control set. Each
//! aircraft gets its own fully independent list — there is no shared runtime
//! state between aircraft — but the *defaults* for the controls they have in
//! common are written the same, so muscle memory carries across aircraft.
//!
//! Capabilities live here (app-side) rather than in the FDM trait, so they work
//! on every backend and, later, on `wasm`. The FDM's own predicates
//! (`has_tvc`/`has_rotor`/`has_retractable_gear`) still describe the physics.

use bevy::prelude::KeyCode;

/// How a binding responds to its keys.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Axis {
    /// Ramp toward the key direction; return to neutral on release.
    Spring,
    /// Accumulate while a key is held; hold position on release.
    Hold,
}

/// Semantic id for an axis control. Used by the HUD hint line and (later) the
/// rebind UI.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Action {
    Elevator,
    Aileron,
    Rudder,
    Throttle,
    Collective,
    PitchTrim,
    Flap,
    BrakeLeft,
    BrakeRight,
    /// Crank the engine over with the starter motor (c172p). Held like the
    /// throttle: the ramp IS the crank, and the engine lights when the prop
    /// reaches firing speed with spark and fuel present.
    Starter,
}

impl Action {
    pub fn label(self) -> &'static str {
        match self {
            Action::Elevator => "pitch",
            Action::Aileron => "roll",
            Action::Rudder => "yaw",
            Action::Throttle => "throttle",
            Action::Collective => "collective",
            Action::PitchTrim => "trim",
            Action::Flap => "flaps",
            Action::BrakeLeft | Action::BrakeRight => "brakes",
            Action::Starter => "starter",
        }
    }
}

/// A press-type (edge) control: a key that fires an action once when tapped.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum ToggleAction {
    /// Engage/disengage the autopilot (only on aircraft that have one).
    Autopilot,
    /// Toggle thrust vectoring (`fcs/tvc-enable`).
    Tvc,
    /// Raise/lower the gear (`gear/gear-cmd-norm`).
    Gear,
    /// Re-trim for the current condition, keeping airspeed and position.
    Trim,
    /// A rotorcraft's hands-free level-flight trim.
    RotorTrim,
    /// Arcade helicopter: vertical take-off / hover / land state machine.
    HeliHover,
    /// Arcade helicopter: auto-level and rate damping.
    HeliBalance,
    /// Select heading-select lateral mode (737 A/P).
    HdgSel,
    /// Capture and hold the current altitude (737 A/P).
    AltHold,
    /// Select vertical-speed mode (737 A/P).
    VsMode,
    /// Select speed-on-pitch level-change mode (737 A/P).
    LvlChg,
    /// Engage/disengage the autothrottle in SPD (737).
    AutoThrottle,
    /// Switch the MCP speed window between IAS and Mach (737).
    SpeedMach,
    /// Deploy/stow the speedbrake (`fcs/speedbrake-cmd-norm`, 737).
    SpeedBrake,
    /// Arm/disarm thrust reverse (`fcs/reverse-cmd-norm`, 737). The FDM only
    /// extends it while the wheels carry weight.
    Reverse,
    /// Arm/disarm the APP mode (LOC + GS beams, 737 MCP).
    Approach,
    /// Master switch: battery + alternator as one (`propulsion/
    /// ignition-enable`, c172p OFF until the pilot turns it on for the cold
    /// start; with the engine already running the mags keep it alive anyway).
    Master,
    /// Exterior lights pack (nav + beacon + strobe + landing light, c172p).
    /// Defaults on at night; the key overrides for day ops or a night run
    /// with the lights off.
    Lights,
    /// Combat: cycle/lock the bandit in the HUD cone (arcade F-22).
    CycleTarget,
    /// Combat: switch between the IR and BVR missile.
    CycleWeapon,
    /// Combat: cycle the TSD range (10 / 20 / 40 km).
    RadarRange,
    /// Combat: launch the selected missile at the locked bandit.
    FireMissile,
    /// Combat: fire the gun while held.
    FireGun,
    /// Combat: release flares (visual only until bandits shoot back).
    Flares,
}

impl ToggleAction {
    pub fn label(self) -> &'static str {
        match self {
            ToggleAction::Autopilot => "autopilot",
            ToggleAction::Tvc => "TVC",
            ToggleAction::Gear => "gear",
            ToggleAction::Trim => "retrim",
            ToggleAction::RotorTrim => "trim",
            ToggleAction::HeliHover => "hover / VTOL",
            ToggleAction::HeliBalance => "auto balance",
            ToggleAction::HdgSel => "HDG SEL",
            ToggleAction::AltHold => "ALT HOLD",
            ToggleAction::VsMode => "V/S",
            ToggleAction::LvlChg => "LVL CHG",
            ToggleAction::AutoThrottle => "A/T",
            ToggleAction::SpeedMach => "IAS/MACH",
            ToggleAction::SpeedBrake => "speedbrake",
            ToggleAction::Reverse => "reverse",
            ToggleAction::Approach => "APP",
            ToggleAction::Master => "master",
            ToggleAction::Lights => "lights",
            ToggleAction::CycleTarget => "lock target",
            ToggleAction::CycleWeapon => "weapon",
            ToggleAction::RadarRange => "radar range",
            ToggleAction::FireMissile => "fire missile",
            ToggleAction::FireGun => "gun",
            ToggleAction::Flares => "flares",
        }
    }
}

/// One axis (held-key) control.
#[derive(Clone, Copy)]
pub struct BindingSpec {
    pub action: Action,
    /// JSBSim property written to the FDM each step.
    pub property: &'static str,
    pub axis: Axis,
    pub pos: KeyCode,
    pub neg: Option<KeyCode>,
    pub rate: f64,
    pub min: f64,
    pub max: f64,
    /// Starting value (throttle/collective may begin raised).
    pub initial: f64,
    /// Discrete stop positions (normalized, ascending). Empty = a continuous
    /// lever. The C172's flap lever has four detents; the jet levers stay
    /// continuous.
    pub detents: &'static [f64],
}

/// One press-type control.
#[derive(Clone, Copy)]
pub struct ToggleSpec {
    pub action: ToggleAction,
    pub key: KeyCode,
}

/// An aircraft's complete control set.
pub struct Profile {
    pub has_autopilot: bool,
    /// Whether the aircraft has an arcade dynamics path at all. The F-22 uses
    /// it for missions and the AH-64D for its simplified rotor flight.
    pub has_arcade: bool,
    /// Whether the pilot can switch arcade mode on from the settings page. The
    /// F-22's arcade path is mission-only, so it has no switch.
    pub arcade_selectable: bool,
    pub bindings: Vec<BindingSpec>,
    pub toggles: Vec<ToggleSpec>,
}

// --- shared control constructors (the identical-across-aircraft defaults) ---

fn elevator() -> BindingSpec {
    BindingSpec {
        action: Action::Elevator,
        property: "fcs/elevator-cmd-norm",
        axis: Axis::Spring,
        pos: KeyCode::KeyS,
        neg: Some(KeyCode::KeyW),
        rate: 3.0,
        min: -1.0,
        max: 1.0,
        initial: 0.0,
        detents: &[],
    }
}
fn aileron() -> BindingSpec {
    BindingSpec {
        action: Action::Aileron,
        property: "fcs/aileron-cmd-norm",
        axis: Axis::Spring,
        pos: KeyCode::KeyD,
        neg: Some(KeyCode::KeyA),
        rate: 3.0,
        min: -1.0,
        max: 1.0,
        initial: 0.0,
        detents: &[],
    }
}
fn rudder() -> BindingSpec {
    BindingSpec {
        action: Action::Rudder,
        property: "fcs/rudder-cmd-norm",
        axis: Axis::Spring,
        pos: KeyCode::KeyQ,
        neg: Some(KeyCode::KeyE),
        rate: 3.0,
        min: -1.0,
        max: 1.0,
        initial: 0.0,
        detents: &[],
    }
}
fn throttle(pos: KeyCode) -> BindingSpec {
    BindingSpec {
        action: Action::Throttle,
        property: "fcs/throttle-cmd-norm",
        axis: Axis::Hold,
        pos,
        neg: Some(KeyCode::ControlLeft),
        rate: 0.6,
        min: 0.0,
        max: 1.0,
        initial: 0.6,
        detents: &[],
    }
}
fn collective() -> BindingSpec {
    BindingSpec {
        action: Action::Collective,
        property: "fcs/collective-cmd-norm",
        axis: Axis::Hold,
        pos: KeyCode::ShiftLeft,
        neg: Some(KeyCode::ControlLeft),
        rate: 0.6,
        min: 0.0,
        max: 1.0,
        initial: 0.0,
        detents: &[],
    }
}
fn pitch_trim() -> BindingSpec {
    BindingSpec {
        action: Action::PitchTrim,
        property: "fcs/pitch-trim-cmd-norm",
        axis: Axis::Hold,
        pos: KeyCode::KeyZ,
        neg: Some(KeyCode::KeyX),
        rate: 0.3,
        min: -1.0,
        max: 1.0,
        initial: 0.0,
        detents: &[],
    }
}
fn flap() -> BindingSpec {
    BindingSpec {
        action: Action::Flap,
        property: "fcs/flap-cmd-norm",
        axis: Axis::Hold,
        pos: KeyCode::KeyF,
        neg: Some(KeyCode::KeyV),
        rate: 0.5,
        min: 0.0,
        max: 1.0,
        initial: 0.0,
        detents: &[],
    }
}

/// The C172 flap lever's stops as normalized `flap-cmd-norm` values. The wing's
/// full-flap deflection is 30°, so the mid detents land on 10° and 20° exactly.
pub const C172_FLAP_DETENTS: &[f64] = &[0.0, 1.0 / 3.0, 2.0 / 3.0, 1.0];
fn brake(action: Action, property: &'static str) -> BindingSpec {
    BindingSpec {
        action,
        property,
        axis: Axis::Spring,
        pos: KeyCode::KeyB,
        neg: None,
        rate: 4.0,
        min: 0.0,
        max: 1.0,
        initial: 0.0,
        detents: &[],
    }
}

/// The starter crank. Unpaired (crank is one-way); the fast ramp reaches the
/// firing speed after roughly a second of holding the key with a live spark.
fn starter() -> BindingSpec {
    BindingSpec {
        action: Action::Starter,
        property: "propulsion/engine[0]/starter-cmd-norm",
        axis: Axis::Hold,
        pos: KeyCode::KeyI,
        neg: None,
        rate: 3.0,
        min: 0.0,
        max: 1.0,
        initial: 0.0,
        detents: &[],
    }
}

fn core_bindings(throttle_pos: KeyCode) -> Vec<BindingSpec> {
    vec![
        elevator(),
        aileron(),
        rudder(),
        throttle(throttle_pos),
        pitch_trim(),
        flap(),
        brake(Action::BrakeLeft, "fcs/left-brake-cmd-norm"),
        brake(Action::BrakeRight, "fcs/right-brake-cmd-norm"),
    ]
}

fn core_toggles() -> Vec<ToggleSpec> {
    vec![ToggleSpec {
        action: ToggleAction::Trim,
        key: KeyCode::KeyT,
    }]
}

// --- per-aircraft profiles ---------------------------------------------------

/// Canonical aircraft id: the menu uses `"737"`, the FDM accepts `"b737"` too.
/// Config sections are keyed by this, so `b737` and `737` share one keymap.
pub fn normalized(id: &str) -> &str {
    match id {
        "b737" => "737",
        "c172" => "c172p",
        "ah-1s" | "ah-64d" | "ah64d" => "ah1s",
        other => other,
    }
}

/// The profile for an aircraft id. Unknown ids fall back to the Cessna, which
/// mirrors `Fdm::new`'s own default.
pub fn profile(id: &str) -> Profile {
    match normalized(id) {
        "f22" => Profile {
            has_autopilot: false,
            has_arcade: true,
            // Arcade is mission-only: FREE FLY always uses the realistic model.
            arcade_selectable: false,
            // The arcade boost lives on the space bar.
            bindings: core_bindings(KeyCode::Space),
            toggles: {
                let mut t = core_toggles();
                t.push(ToggleSpec {
                    action: ToggleAction::Gear,
                    key: KeyCode::KeyU,
                });
                t.push(ToggleSpec {
                    action: ToggleAction::Tvc,
                    key: KeyCode::KeyJ,
                });
                // Arcade combat. Tab walks the targets in the HUD cone, Enter
                // launches the locked IR missile, G holds the gun, L pops flares.
                t.push(ToggleSpec {
                    action: ToggleAction::CycleTarget,
                    key: KeyCode::Tab,
                });
                t.push(ToggleSpec {
                    action: ToggleAction::CycleWeapon,
                    key: KeyCode::KeyK,
                });
                t.push(ToggleSpec {
                    action: ToggleAction::RadarRange,
                    key: KeyCode::KeyR,
                });
                t.push(ToggleSpec {
                    action: ToggleAction::FireMissile,
                    key: KeyCode::Enter,
                });
                t.push(ToggleSpec {
                    action: ToggleAction::FireGun,
                    key: KeyCode::KeyG,
                });
                t.push(ToggleSpec {
                    action: ToggleAction::Flares,
                    key: KeyCode::KeyL,
                });
                t
            },
        },
        "ah1s" => Profile {
            has_autopilot: false,
            // AH-64D: realistic rotor physics or the simplified helicopter
            // arcade rules (auto hover + auto balance) selected in Settings.
            has_arcade: true,
            arcade_selectable: true,
            // A helicopter: collective instead of throttle, no flaps, no wheels.
            bindings: vec![elevator(), aileron(), rudder(), collective(), pitch_trim()],
            toggles: vec![
                ToggleSpec {
                    action: ToggleAction::RotorTrim,
                    key: KeyCode::Space,
                },
                ToggleSpec {
                    action: ToggleAction::Trim,
                    key: KeyCode::KeyT,
                },
                ToggleSpec {
                    action: ToggleAction::HeliHover,
                    key: KeyCode::KeyJ,
                },
                ToggleSpec {
                    action: ToggleAction::HeliBalance,
                    key: KeyCode::KeyB,
                },
            ],
        },
        "737" => Profile {
            has_autopilot: true,
            has_arcade: false,
            arcade_selectable: false,
            bindings: core_bindings(KeyCode::ShiftLeft),
            toggles: {
                let mut t = core_toggles();
                t.push(ToggleSpec {
                    action: ToggleAction::Autopilot,
                    key: KeyCode::KeyG,
                });
                t.push(ToggleSpec {
                    action: ToggleAction::Gear,
                    key: KeyCode::KeyU,
                });
                // MCP mode buttons, laid out like the 737's mode control panel.
                t.push(ToggleSpec {
                    action: ToggleAction::HdgSel,
                    key: KeyCode::Digit1,
                });
                t.push(ToggleSpec {
                    action: ToggleAction::AltHold,
                    key: KeyCode::Digit2,
                });
                t.push(ToggleSpec {
                    action: ToggleAction::VsMode,
                    key: KeyCode::Digit3,
                });
                t.push(ToggleSpec {
                    action: ToggleAction::LvlChg,
                    key: KeyCode::Digit4,
                });
                t.push(ToggleSpec {
                    action: ToggleAction::AutoThrottle,
                    key: KeyCode::Digit5,
                });
                t.push(ToggleSpec {
                    action: ToggleAction::SpeedMach,
                    key: KeyCode::KeyM,
                });
                // Ground deceleration: the speedbrake handle and the reverse
                // thrust. Reverse only bites with weight on the wheels.
                t.push(ToggleSpec {
                    action: ToggleAction::SpeedBrake,
                    key: KeyCode::KeyL,
                });
                t.push(ToggleSpec {
                    action: ToggleAction::Reverse,
                    key: KeyCode::KeyK,
                });
                // APP (ILS approach): arms LOC + GS on the MCP.
                t.push(ToggleSpec {
                    action: ToggleAction::Approach,
                    key: KeyCode::Digit6,
                });
                t
            },
        },
        // The C172 plus every unknown id: keyboard feel, no autopilot. The
        // cold-start set rides along; on a running spawn the pilot simply
        // never touches M/I.
        _ => Profile {
            has_autopilot: false,
            has_arcade: false,
            arcade_selectable: false,
            bindings: {
                let mut b = core_bindings(KeyCode::ShiftLeft);
                // The flap lever is a real four-detent lever: UP / 10° / 20° /
                // 30°. `F` and `V` step between stops instead of slewing a
                // continuous command; the surface still travels at its own
                // (slow, electric) rate.
                for spec in &mut b {
                    if spec.action == Action::Flap {
                        spec.detents = C172_FLAP_DETENTS;
                    }
                }
                b.push(starter());
                b
            },
            toggles: {
                let mut t = core_toggles();
                t.push(ToggleSpec {
                    action: ToggleAction::Master,
                    key: KeyCode::KeyM,
                });
                // Exterior lights: nav + beacon + strobe + landing spotlight in
                // one switch. `L` is free here (the 737 spends it on the
                // speedbrake).
                t.push(ToggleSpec {
                    action: ToggleAction::Lights,
                    key: KeyCode::KeyL,
                });
                t
            },
        },
    }
}

/// Compact key name for HUD hints: `KeyW` -> `W`, `ShiftLeft` -> `Shift`.
pub fn key_name(k: KeyCode) -> String {
    let raw = format!("{k:?}");
    let trimmed = raw
        .strip_prefix("Key")
        .or_else(|| raw.strip_prefix("Digit"))
        .unwrap_or(&raw);
    match trimmed {
        "ShiftLeft" | "ShiftRight" => "Shift".to_owned(),
        "ControlLeft" | "ControlRight" => "Ctrl".to_owned(),
        "ArrowUp" => "Up".to_owned(),
        "ArrowDown" => "Down".to_owned(),
        "ArrowLeft" => "Left".to_owned(),
        "ArrowRight" => "Right".to_owned(),
        other => other.to_owned(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn b737_alias_resolves_to_the_737_profile() {
        assert!(profile("b737").has_autopilot);
        assert_eq!(
            profile("b737").bindings.len(),
            profile("737").bindings.len()
        );
    }

    #[test]
    fn only_the_737_has_an_autopilot() {
        assert!(profile("737").has_autopilot);
        assert!(!profile("c172p").has_autopilot);
        assert!(!profile("f22").has_autopilot);
        assert!(!profile("ah1s").has_autopilot);
    }

    #[test]
    fn the_c172_flap_lever_has_four_detents_and_the_jets_do_not() {
        let flap = |id| {
            profile(id)
                .bindings
                .iter()
                .find(|b| b.action == Action::Flap)
                .map(|b| b.detents)
                .unwrap_or(&[])
        };
        let detents = flap("c172p");
        assert_eq!(detents.len(), 4, "UP / 10 / 20 / 30");
        assert_eq!(detents[0], 0.0);
        assert!((detents[1] - 10.0 / 30.0).abs() < 1e-9);
        assert!((detents[2] - 20.0 / 30.0).abs() < 1e-9);
        assert_eq!(detents[3], 1.0);
        assert!(flap("737").is_empty());
        assert!(flap("f22").is_empty());
    }

    #[test]
    fn only_the_apache_offers_an_arcade_switch() {
        // Both carry an arcade dynamics path, but the F-22's is mission-only:
        // its settings page has no flight-model row.
        assert!(profile("f22").has_arcade);
        assert!(!profile("f22").arcade_selectable);
        assert!(profile("ah1s").has_arcade);
        assert!(profile("ah1s").arcade_selectable);
        assert!(!profile("c172p").has_arcade);
        assert!(!profile("c172p").arcade_selectable);
        assert!(!profile("737").has_arcade);
        assert!(!profile("737").arcade_selectable);
    }

    #[test]
    fn aircraft_have_decoupled_control_sets() {
        // The helicopter has a collective and no flaps/brakes; the Cessna the
        // reverse. This is the point of per-aircraft decoupling.
        let heli = profile("ah1s");
        assert!(heli.bindings.iter().any(|b| b.action == Action::Collective));
        assert!(!heli.bindings.iter().any(|b| b.action == Action::Flap));

        let c172 = profile("c172p");
        assert!(c172.bindings.iter().any(|b| b.action == Action::Throttle));
        assert!(!c172.bindings.iter().any(|b| b.action == Action::Collective));

        // Gear and TVC toggles are F-22-only; the Cessna has neither.
        let has = |id, a| profile(id).toggles.iter().any(|t| t.action == a);
        assert!(has("f22", ToggleAction::Gear) && has("f22", ToggleAction::Tvc));
        assert!(!has("c172p", ToggleAction::Gear));
        assert!(!has("c172p", ToggleAction::Tvc));
        // And the autopilot is the 737's alone.
        assert!(has("737", ToggleAction::Autopilot));
        assert!(!has("f22", ToggleAction::Autopilot));
    }

    #[test]
    fn common_controls_share_defaults() {
        // Muscle memory: pitch binds to S/W on every aircraft that has it.
        for id in ["c172p", "f22", "737", "ah1s"] {
            let p = profile(id);
            let elev = p
                .bindings
                .iter()
                .find(|b| b.action == Action::Elevator)
                .unwrap();
            assert_eq!(elev.pos, KeyCode::KeyS);
            assert_eq!(elev.neg, Some(KeyCode::KeyW));
        }
    }

    #[test]
    fn the_f22_throttle_moves_to_the_space_bar() {
        // The arcade F-22 spools with Space; every other aircraft keeps the
        // shared Shift/Ctrl throttle so muscle memory carries over.
        let pos = |id: &str| {
            profile(id)
                .bindings
                .into_iter()
                .find(|b| b.action == Action::Throttle)
                .unwrap()
                .pos
        };
        assert_eq!(pos("f22"), KeyCode::Space);
        assert_eq!(pos("c172p"), KeyCode::ShiftLeft);
        assert_eq!(pos("737"), KeyCode::ShiftLeft);
    }

    #[test]
    fn key_names_are_compact() {
        assert_eq!(key_name(KeyCode::KeyW), "W");
        assert_eq!(key_name(KeyCode::ShiftLeft), "Shift");
        assert_eq!(key_name(KeyCode::ControlLeft), "Ctrl");
        assert_eq!(key_name(KeyCode::Space), "Space");
    }
}
