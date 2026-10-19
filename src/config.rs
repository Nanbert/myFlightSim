//! Persisted user settings: the global weather level and per-aircraft key
//! overrides.
//!
//! Storage sits behind [`ConfigStore`] so the native build can write a file
//! while a future browser (wasm) build supplies a `localStorage` sink; the rest
//! of the app only ever talks to the trait. The format is hand-rolled text (no
//! serde dependency), which keeps that wasm story simple.
//!
//! Only the keys a pilot has actually changed are stored, so a later change to
//! a default in [`crate::aircraft_profile`] still reaches everyone who left that
//! key alone.

use std::collections::{HashMap, HashSet};
use std::path::PathBuf;

use bevy::prelude::*;

use crate::aircraft_profile::{self, Action, ToggleAction};
use crate::weather::Weather;

/// The sink the config text comes from and goes to. Native uses [`FileStore`];
/// a wasm build would implement this over `localStorage`.
pub trait ConfigStore: Send + Sync + 'static {
    fn load(&self) -> Option<String>;
    fn save(&self, text: &str);
}

/// One pilot's key overrides for a single aircraft. Empty entries mean "use the
/// profile default".
#[derive(Clone, Default, Debug, PartialEq)]
pub struct Keymap {
    axes: HashMap<Action, (KeyCode, Option<KeyCode>)>,
    toggles: HashMap<ToggleAction, KeyCode>,
}

impl Keymap {
    /// The overridden `(pos, neg)` pair for an axis, or `None` to use the
    /// profile default.
    pub fn axis(&self, action: Action) -> Option<(KeyCode, Option<KeyCode>)> {
        self.axes.get(&action).copied()
    }

    pub fn set_axis(&mut self, action: Action, pos: KeyCode, neg: Option<KeyCode>) {
        self.axes.insert(action, (pos, neg));
    }

    /// The overridden key for a toggle, or `None` to use the profile default.
    pub fn toggle(&self, action: ToggleAction) -> Option<KeyCode> {
        self.toggles.get(&action).copied()
    }

    pub fn set_toggle(&mut self, action: ToggleAction, key: KeyCode) {
        self.toggles.insert(action, key);
    }

    pub fn is_empty(&self) -> bool {
        self.axes.is_empty() && self.toggles.is_empty()
    }
}

/// Keyboard assists for one aircraft. Both default to off; the C172 exposes
/// them on its settings page.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct AssistConfig {
    pub bank_hold: bool,
    pub turn_coord: bool,
}

/// All persisted settings. Lives as a resource so the menu can read and write
/// it; changes are flushed through [`ConfigSink`].
#[derive(Resource, Default, Debug, PartialEq)]
pub struct GameConfig {
    /// Index into [`crate::weather::RAIN_LEVELS`].
    pub weather_level: usize,
    /// Index into [`crate::weather::WIND_LEVELS`].
    pub wind_level: usize,
    /// ENGINE = COLD: spawn the C172 shut down and let the pilot run the start
    /// flow (master M, crank I). Ignored by every other aircraft.
    pub engine_cold: bool,
    keymaps: HashMap<String, Keymap>,
    assists: HashMap<String, AssistConfig>,
    /// Aircraft set to fly in arcade mode, by canonical id. An id is present iff
    /// arcade is on, so the realistic default is simply absence.
    arcade: HashSet<String>,
}

impl GameConfig {
    /// The keymap for an aircraft id (canonicalized), defaulting to "no
    /// overrides".
    pub fn keymap(&self, id: &str) -> Keymap {
        self.keymaps
            .get(aircraft_profile::normalized(id))
            .cloned()
            .unwrap_or_default()
    }

    /// Mutable access to an aircraft's keymap, creating it if absent.
    pub fn keymap_mut(&mut self, id: &str) -> &mut Keymap {
        let id = aircraft_profile::normalized(id).to_owned();
        self.keymaps.entry(id).or_default()
    }

    /// Whether an aircraft flies arcade. Realistic (the default) unless the
    /// pilot has switched it on.
    pub fn arcade(&self, id: &str) -> bool {
        self.arcade.contains(aircraft_profile::normalized(id))
    }

    /// Turn an aircraft's arcade mode on or off, dropping the entry when off so
    /// the default (realistic) is just absence.
    pub fn set_arcade(&mut self, id: &str, on: bool) {
        let id = aircraft_profile::normalized(id).to_owned();
        if on {
            self.arcade.insert(id);
        } else {
            self.arcade.remove(&id);
        }
    }

    /// The keyboard assists for an aircraft (both off by default).
    pub fn assists(&self, id: &str) -> AssistConfig {
        self.assists
            .get(aircraft_profile::normalized(id))
            .copied()
            .unwrap_or_default()
    }

    fn set_assist(&mut self, id: &str, change: impl FnOnce(&mut AssistConfig)) {
        let id = aircraft_profile::normalized(id).to_owned();
        let mut a = self.assists.get(&id).copied().unwrap_or_default();
        change(&mut a);
        if a == AssistConfig::default() {
            self.assists.remove(&id);
        } else {
            self.assists.insert(id, a);
        }
    }

    pub fn set_bank_hold(&mut self, id: &str, on: bool) {
        self.set_assist(id, |a| a.bank_hold = on);
    }

    pub fn set_turn_coord(&mut self, id: &str, on: bool) {
        self.set_assist(id, |a| a.turn_coord = on);
    }
}

/// The boxed store, kept so the menu can save without knowing the backend.
#[derive(Resource)]
pub struct ConfigSink(pub Box<dyn ConfigStore>);

/// Persist the current config. Called after any setting changes.
pub fn save(config: &GameConfig, sink: &ConfigSink) {
    sink.0.save(&serialize(config));
}

pub struct ConfigPlugin;

impl Plugin for ConfigPlugin {
    fn build(&self, app: &mut App) {
        let store = FileStore::new();
        let config = store.load().map(|t| parse(&t)).unwrap_or_default();
        app.insert_resource(config)
            .insert_resource(ConfigSink(Box::new(store)))
            // Startup, so `Weather` (from `WeatherPlugin`) already exists.
            .add_systems(Startup, apply_weather);
    }
}

fn apply_weather(config: Res<GameConfig>, mut weather: ResMut<Weather>) {
    weather.set_level(config.weather_level as isize);
    weather.set_wind_level(config.wind_level as isize);
}

// --- text format -------------------------------------------------------------
//
//   weather = 2
//
//   [c172p]
//   elevator = KeyS KeyW
//   gear = KeyU
//
//   [arcade]
//   f22 = 1
//
// Action and toggle ids live in the same `[aircraft]` section because the two
// id namespaces are disjoint. The one reserved section name is `arcade`, whose
// keys are aircraft ids rather than controls.

/// The section header whose keys are aircraft ids, not control ids.
const ARCADE_SECTION: &str = "arcade";
/// Per-aircraft assist sections are written as `[c172p.assist]`.
const ASSIST_SUFFIX: &str = ".assist";

fn truthy(value: &str) -> bool {
    matches!(value, "1" | "true" | "on")
}

fn serialize(config: &GameConfig) -> String {
    let mut out = format!("weather = {}\n", config.weather_level);
    out.push_str(&format!("wind = {}\n", config.wind_level));
    if config.engine_cold {
        out.push_str("engine_cold = 1\n");
    }

    let mut ids: Vec<&String> = config.keymaps.keys().collect();
    ids.sort();
    for id in ids {
        let km = &config.keymaps[id];
        if km.is_empty() {
            continue;
        }
        out.push_str(&format!("\n[{id}]\n"));

        // Sort so the file is stable (HashMap order is not).
        let mut axes: Vec<(Action, (KeyCode, Option<KeyCode>))> =
            km.axes.iter().map(|(a, v)| (*a, *v)).collect();
        axes.sort_by_key(|(a, _)| a.id());
        for (action, (pos, neg)) in axes {
            match neg {
                Some(neg) => out.push_str(&format!("{} = {pos:?} {neg:?}\n", action.id())),
                None => out.push_str(&format!("{} = {pos:?}\n", action.id())),
            }
        }

        let mut toggles: Vec<(ToggleAction, KeyCode)> =
            km.toggles.iter().map(|(a, k)| (*a, *k)).collect();
        toggles.sort_by_key(|(a, _)| a.id());
        for (action, key) in toggles {
            out.push_str(&format!("{} = {key:?}\n", action.id()));
        }
    }

    if !config.arcade.is_empty() {
        out.push_str(&format!("\n[{ARCADE_SECTION}]\n"));
        let mut ids: Vec<&String> = config.arcade.iter().collect();
        ids.sort();
        for id in ids {
            out.push_str(&format!("{id} = 1\n"));
        }
    }

    let mut assist_ids: Vec<&String> = config.assists.keys().collect();
    assist_ids.sort();
    for id in assist_ids {
        let a = config.assists[id];
        out.push_str(&format!("\n[{id}.assist]\n"));
        if a.bank_hold {
            out.push_str("bank-hold = 1\n");
        }
        if a.turn_coord {
            out.push_str("turn-coord = 1\n");
        }
    }
    out
}

fn parse(text: &str) -> GameConfig {
    let mut config = GameConfig::default();
    let mut section: Option<&str> = None;

    for raw in text.lines() {
        let line = raw.trim();
        if line.is_empty() || line.starts_with('#') {
            continue;
        }
        if let Some(id) = line.strip_prefix('[').and_then(|s| s.strip_suffix(']')) {
            section = Some(id.trim());
            continue;
        }
        let Some((key, value)) = line.split_once('=') else {
            continue;
        };
        let (key, value) = (key.trim(), value.trim());

        match section {
            None => {
                if key == "weather" {
                    if let Ok(n) = value.parse::<usize>() {
                        config.weather_level = n;
                    }
                }
                if key == "wind" {
                    if let Ok(n) = value.parse::<usize>() {
                        config.wind_level = n;
                    }
                }
                if key == "engine_cold" {
                    config.engine_cold = truthy(value);
                }
            }
            Some(ARCADE_SECTION) => {
                // `key` is an aircraft id here, not a control. Only a truthy
                // value is stored; anything else means "leave realistic".
                if truthy(value) {
                    config.set_arcade(key, true);
                }
            }
            Some(s) if s.ends_with(ASSIST_SUFFIX) => {
                let id = &s[..s.len() - ASSIST_SUFFIX.len()];
                match key {
                    "bank-hold" => config.set_bank_hold(id, truthy(value)),
                    "turn-coord" => config.set_turn_coord(id, truthy(value)),
                    _ => {}
                }
            }
            Some(id) => {
                let mut toks = value.split_whitespace();
                let Some(k0) = toks.next().and_then(aircraft_profile::key_from_name) else {
                    continue;
                };
                let neg = toks.next().and_then(aircraft_profile::key_from_name);
                let km = config.keymap_mut(id);
                if let Some(action) = Action::from_id(key) {
                    km.set_axis(action, k0, neg);
                } else if let Some(action) = ToggleAction::from_id(key) {
                    km.set_toggle(action, k0);
                }
            }
        }
    }
    config
}

// --- native store ------------------------------------------------------------

/// Writes `myflightsim/config.cfg` under the XDG config dir (or `~/.config`),
/// using only `std::env` — no `dirs` dependency.
pub struct FileStore {
    path: PathBuf,
}

impl FileStore {
    pub fn new() -> Self {
        FileStore {
            path: default_path(),
        }
    }
}

impl Default for FileStore {
    fn default() -> Self {
        Self::new()
    }
}

fn default_path() -> PathBuf {
    let base = std::env::var_os("XDG_CONFIG_HOME")
        .map(PathBuf::from)
        .filter(|p| p.is_absolute())
        .or_else(|| std::env::var_os("HOME").map(|h| PathBuf::from(h).join(".config")))
        .unwrap_or_else(|| PathBuf::from("."));
    base.join("myflightsim").join("config.cfg")
}

impl ConfigStore for FileStore {
    fn load(&self) -> Option<String> {
        std::fs::read_to_string(&self.path).ok()
    }

    fn save(&self, text: &str) {
        if let Some(parent) = self.path.parent() {
            let _ = std::fs::create_dir_all(parent);
        }
        let _ = std::fs::write(&self.path, text);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::aircraft_profile::{Action, ToggleAction};

    #[test]
    fn round_trips_through_the_text_format() {
        let mut config = GameConfig {
            weather_level: 2,
            ..default()
        };
        {
            let km = config.keymap_mut("c172p");
            km.set_axis(Action::Elevator, KeyCode::ArrowUp, Some(KeyCode::ArrowDown));
            km.set_axis(Action::BrakeLeft, KeyCode::Space, None);
            km.set_toggle(ToggleAction::Gear, KeyCode::KeyG);
        }
        {
            let km = config.keymap_mut("f22");
            km.set_toggle(ToggleAction::Tvc, KeyCode::KeyJ);
        }

        let text = serialize(&config);
        assert_eq!(parse(&text), config);

        // And again, so serialize is idempotent on a parsed file.
        assert_eq!(serialize(&parse(&text)), text);
    }

    #[test]
    fn arcade_mode_round_trips_per_aircraft() {
        let mut config = GameConfig::default();
        config.set_arcade("f22", true);
        // Realistic is the default, so it is simply not stored.
        assert!(!config.arcade("c172p"));
        assert!(config.arcade("f22"));

        let text = serialize(&config);
        assert_eq!(parse(&text), config);
        assert_eq!(serialize(&parse(&text)), text);
        assert!(parse(&text).arcade("f22"));

        // Toggling off drops the entry, so the file has no stale section.
        config.set_arcade("f22", false);
        assert!(!config.arcade("f22"));
        assert!(!serialize(&config).contains("[arcade]"));
    }

    #[test]
    fn arcade_section_normalizes_the_aircraft_id() {
        let cfg = parse("[arcade]\nb737 = 1\n");
        assert!(cfg.arcade("737") && cfg.arcade("b737"));
    }

    #[test]
    fn assists_round_trip_per_aircraft() {
        let mut config = GameConfig::default();
        assert_eq!(config.assists("c172p"), AssistConfig::default());
        config.set_bank_hold("c172p", true);
        config.set_turn_coord("c172p", true);
        assert_eq!(
            config.assists("c172p"),
            AssistConfig {
                bank_hold: true,
                turn_coord: true
            }
        );

        let text = serialize(&config);
        assert!(text.contains("[c172p.assist]"));
        assert_eq!(parse(&text), config);
        assert_eq!(serialize(&parse(&text)), text);

        // Turning both off removes the section, so defaults stay implicit.
        config.set_bank_hold("c172p", false);
        config.set_turn_coord("c172p", false);
        assert_eq!(config.assists("c172p"), AssistConfig::default());
        assert!(!serialize(&config).contains(".assist"));
    }

    #[test]
    fn weather_is_read_from_the_header() {
        let cfg = parse("weather = 3\n");
        assert_eq!(cfg.weather_level, 3);
    }

    #[test]
    fn unknown_keys_and_junk_lines_are_ignored() {
        let cfg = parse("weather = 1\n\n[737]\nnot-a-control = KeyA\nelevator = Nope KeyW\n");
        assert_eq!(cfg.weather_level, 1);
        // The bogus axis line is dropped (unknown key name), so nothing stored
        // beyond the section itself.
        assert!(cfg.keymap("737").axis(Action::Elevator).is_none());
    }

    #[test]
    fn missing_neg_key_parses_as_a_single_slot() {
        let cfg = parse("[737]\ngear = KeyU\n");
        assert_eq!(
            cfg.keymap("737").toggle(ToggleAction::Gear),
            Some(KeyCode::KeyU)
        );
    }

    #[test]
    fn keymap_lookup_normalizes_the_id() {
        let mut config = GameConfig::default();
        config
            .keymap_mut("b737")
            .set_toggle(ToggleAction::Gear, KeyCode::KeyU);
        assert_eq!(
            config.keymap("737").toggle(ToggleAction::Gear),
            Some(KeyCode::KeyU)
        );
    }
}
