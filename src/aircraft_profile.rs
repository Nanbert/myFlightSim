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

    /// Stable id for the config file and the rebind UI.
    pub fn id(self) -> &'static str {
        match self {
            Action::Elevator => "elevator",
            Action::Aileron => "aileron",
            Action::Rudder => "rudder",
            Action::Throttle => "throttle",
            Action::Collective => "collective",
            Action::PitchTrim => "pitch-trim",
            Action::Flap => "flap",
            Action::BrakeLeft => "brake-left",
            Action::BrakeRight => "brake-right",
            Action::Starter => "starter",
        }
    }

    pub fn from_id(s: &str) -> Option<Action> {
        Some(match s {
            "elevator" => Action::Elevator,
            "aileron" => Action::Aileron,
            "rudder" => Action::Rudder,
            "throttle" => Action::Throttle,
            "collective" => Action::Collective,
            "pitch-trim" => Action::PitchTrim,
            "flap" => Action::Flap,
            "brake-left" => Action::BrakeLeft,
            "brake-right" => Action::BrakeRight,
            "starter" => Action::Starter,
            _ => return None,
        })
    }

    /// Whether the control has a second (negative-direction) key slot.
    pub fn is_paired(self) -> bool {
        !matches!(
            self,
            Action::BrakeLeft | Action::BrakeRight | Action::Starter
        )
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
        }
    }

    /// Stable id for the config file and the rebind UI.
    pub fn id(self) -> &'static str {
        match self {
            ToggleAction::Autopilot => "autopilot",
            ToggleAction::Tvc => "tvc",
            ToggleAction::Gear => "gear",
            ToggleAction::Trim => "trim",
            ToggleAction::RotorTrim => "rotor-trim",
            ToggleAction::HeliHover => "heli-hover",
            ToggleAction::HeliBalance => "heli-balance",
            ToggleAction::HdgSel => "hdg-sel",
            ToggleAction::AltHold => "alt-hold",
            ToggleAction::VsMode => "vs",
            ToggleAction::LvlChg => "lvl-chg",
            ToggleAction::AutoThrottle => "autothrottle",
            ToggleAction::SpeedMach => "speed-mach",
            ToggleAction::SpeedBrake => "speedbrake",
            ToggleAction::Reverse => "reverse",
            ToggleAction::Approach => "approach",
            ToggleAction::Master => "master",
            ToggleAction::Lights => "lights",
        }
    }

    pub fn from_id(s: &str) -> Option<ToggleAction> {
        Some(match s {
            "autopilot" => ToggleAction::Autopilot,
            "tvc" => ToggleAction::Tvc,
            "gear" => ToggleAction::Gear,
            "trim" => ToggleAction::Trim,
            "rotor-trim" => ToggleAction::RotorTrim,
            "heli-hover" => ToggleAction::HeliHover,
            "heli-balance" => ToggleAction::HeliBalance,
            "hdg-sel" => ToggleAction::HdgSel,
            "alt-hold" => ToggleAction::AltHold,
            "vs" => ToggleAction::VsMode,
            "lvl-chg" => ToggleAction::LvlChg,
            "autothrottle" => ToggleAction::AutoThrottle,
            "speed-mach" => ToggleAction::SpeedMach,
            "speedbrake" => ToggleAction::SpeedBrake,
            "reverse" => ToggleAction::Reverse,
            "approach" => ToggleAction::Approach,
            "master" => ToggleAction::Master,
            "lights" => ToggleAction::Lights,
            _ => return None,
        })
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
    /// Whether the aircraft offers the Realistic/Arcade flight-model switch on
    /// its settings page. The F-22 and the AH-64D carry arcade dynamics paths.
    pub has_arcade: bool,
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
                t
            },
        },
        "ah1s" => Profile {
            has_autopilot: false,
            // AH-64D: realistic rotor physics or the simplified helicopter
            // arcade rules (auto hover + auto balance) selected in Settings.
            has_arcade: true,
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

/// Whether a key can survive a round-trip through [`key_from_name`]. The rebind
/// UI rejects anything else, so a captured key never silently reverts to its
/// default on reload.
pub fn is_bindable(k: KeyCode) -> bool {
    key_from_name(&format!("{k:?}")) == Some(k)
}

/// Parse a key from the *serialized* form, which is the raw `KeyCode` debug
/// name (`"KeyS"`, `"ShiftLeft"`, `"ArrowUp"`). Bevy's `KeyCode` has no
/// `FromStr`, and the game only ever binds the keys listed here, so a small
/// explicit table beats dragging in serde (which is also a wasm-hostile dep
/// story). Returns `None` for anything unknown, letting callers fall back to
/// the profile default.
pub fn key_from_name(s: &str) -> Option<KeyCode> {
    Some(match s {
        "KeyA" => KeyCode::KeyA,
        "KeyB" => KeyCode::KeyB,
        "KeyC" => KeyCode::KeyC,
        "KeyD" => KeyCode::KeyD,
        "KeyE" => KeyCode::KeyE,
        "KeyF" => KeyCode::KeyF,
        "KeyG" => KeyCode::KeyG,
        "KeyH" => KeyCode::KeyH,
        "KeyI" => KeyCode::KeyI,
        "KeyJ" => KeyCode::KeyJ,
        "KeyK" => KeyCode::KeyK,
        "KeyL" => KeyCode::KeyL,
        "KeyM" => KeyCode::KeyM,
        "KeyN" => KeyCode::KeyN,
        "KeyO" => KeyCode::KeyO,
        "KeyP" => KeyCode::KeyP,
        "KeyQ" => KeyCode::KeyQ,
        "KeyR" => KeyCode::KeyR,
        "KeyS" => KeyCode::KeyS,
        "KeyT" => KeyCode::KeyT,
        "KeyU" => KeyCode::KeyU,
        "KeyV" => KeyCode::KeyV,
        "KeyW" => KeyCode::KeyW,
        "KeyX" => KeyCode::KeyX,
        "KeyY" => KeyCode::KeyY,
        "KeyZ" => KeyCode::KeyZ,
        "Digit0" => KeyCode::Digit0,
        "Digit1" => KeyCode::Digit1,
        "Digit2" => KeyCode::Digit2,
        "Digit3" => KeyCode::Digit3,
        "Digit4" => KeyCode::Digit4,
        "Digit5" => KeyCode::Digit5,
        "Digit6" => KeyCode::Digit6,
        "Digit7" => KeyCode::Digit7,
        "Digit8" => KeyCode::Digit8,
        "Digit9" => KeyCode::Digit9,
        "ShiftLeft" => KeyCode::ShiftLeft,
        "ShiftRight" => KeyCode::ShiftRight,
        "ControlLeft" => KeyCode::ControlLeft,
        "ControlRight" => KeyCode::ControlRight,
        "AltLeft" => KeyCode::AltLeft,
        "AltRight" => KeyCode::AltRight,
        "Space" => KeyCode::Space,
        "Tab" => KeyCode::Tab,
        "Enter" => KeyCode::Enter,
        "Escape" => KeyCode::Escape,
        "Backspace" => KeyCode::Backspace,
        "ArrowUp" => KeyCode::ArrowUp,
        "ArrowDown" => KeyCode::ArrowDown,
        "ArrowLeft" => KeyCode::ArrowLeft,
        "ArrowRight" => KeyCode::ArrowRight,
        "Comma" => KeyCode::Comma,
        "Period" => KeyCode::Period,
        "Slash" => KeyCode::Slash,
        "Semicolon" => KeyCode::Semicolon,
        "Quote" => KeyCode::Quote,
        "BracketLeft" => KeyCode::BracketLeft,
        "BracketRight" => KeyCode::BracketRight,
        "Backslash" => KeyCode::Backslash,
        "Minus" => KeyCode::Minus,
        "Equal" => KeyCode::Equal,
        _ => return None,
    })
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
    fn the_f22_and_apache_offer_arcade_mode() {
        assert!(profile("f22").has_arcade);
        assert!(profile("ah1s").has_arcade);
        assert!(!profile("c172p").has_arcade);
        assert!(!profile("737").has_arcade);
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

    #[test]
    fn key_serialize_round_trips() {
        // Every key the profiles use must survive name -> parse -> name.
        for k in [
            KeyCode::KeyS,
            KeyCode::KeyW,
            KeyCode::ShiftLeft,
            KeyCode::ControlLeft,
            KeyCode::Space,
            KeyCode::KeyG,
            KeyCode::ArrowUp,
            KeyCode::Digit3,
        ] {
            let name = format!("{k:?}");
            assert_eq!(key_from_name(&name), Some(k), "failed on {name}");
        }
        assert_eq!(key_from_name("NotAKey"), None);
    }

    #[test]
    fn action_ids_round_trip() {
        for a in [
            Action::Elevator,
            Action::Aileron,
            Action::Rudder,
            Action::Throttle,
            Action::Collective,
            Action::PitchTrim,
            Action::Flap,
            Action::BrakeLeft,
            Action::BrakeRight,
        ] {
            assert_eq!(Action::from_id(a.id()), Some(a));
        }
    }

    #[test]
    fn toggle_ids_round_trip() {
        for a in [
            ToggleAction::Autopilot,
            ToggleAction::Tvc,
            ToggleAction::Gear,
            ToggleAction::Trim,
            ToggleAction::RotorTrim,
            ToggleAction::HdgSel,
            ToggleAction::AltHold,
            ToggleAction::VsMode,
            ToggleAction::LvlChg,
            ToggleAction::AutoThrottle,
            ToggleAction::SpeedMach,
            ToggleAction::SpeedBrake,
            ToggleAction::Reverse,
            ToggleAction::Approach,
        ] {
            assert_eq!(ToggleAction::from_id(a.id()), Some(a));
        }
    }
}
