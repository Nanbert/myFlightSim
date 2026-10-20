//! Persisted user settings: weather/wind/time, per-aircraft assists and the
//! AH-64D's arcade choice.
//!
//! Storage sits behind [`ConfigStore`] so the native build can write a file
//! while a future browser (wasm) build supplies a `localStorage` sink; the rest
//! of the app only ever talks to the trait. The format is hand-rolled text (no
//! serde dependency), which keeps that wasm story simple.
//!
//! Key bindings are fixed in [`crate::aircraft_profile`], so the file never
//! stores controls; sections left behind by older builds are ignored.

use std::collections::{HashMap, HashSet};
use std::path::PathBuf;

use bevy::prelude::*;

use crate::aircraft_profile;
use crate::weather::Weather;

/// The sink the config text comes from and goes to. Native uses [`FileStore`];
/// a wasm build would implement this over `localStorage`.
pub trait ConfigStore: Send + Sync + 'static {
    fn load(&self) -> Option<String>;
    fn save(&self, text: &str);
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
    /// Index into [`crate::weather::TIME_LABELS`].
    pub time_level: usize,
    assists: HashMap<String, AssistConfig>,
    /// Aircraft set to fly in arcade mode, by canonical id. An id is present iff
    /// arcade is on, so the realistic default is simply absence.
    arcade: HashSet<String>,
}

impl GameConfig {
    /// Whether an aircraft flies arcade. Realistic (the default) unless the
    /// pilot has switched it on. Aircraft without a switch (the F-22, whose
    /// arcade path is mission-only) can never be arcade through this setting.
    pub fn arcade(&self, id: &str) -> bool {
        let id = aircraft_profile::normalized(id);
        aircraft_profile::profile(id).arcade_selectable && self.arcade.contains(id)
    }

    /// Turn an aircraft's arcade mode on or off, dropping the entry when off so
    /// the default (realistic) is just absence. A stale entry for an aircraft
    /// that no longer offers the switch is ignored and dropped on the next save.
    pub fn set_arcade(&mut self, id: &str, on: bool) {
        let id = aircraft_profile::normalized(id).to_owned();
        if on && aircraft_profile::profile(&id).arcade_selectable {
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
    weather.set_time_level(config.time_level as isize);
}

// --- text format -------------------------------------------------------------
//
//   weather = 2
//
//   [arcade]
//   ah1s = 1
//
//   [c172p.assist]
//   bank-hold = 1
//
// The reserved section names are `arcade` (aircraft ids) and `<id>.assist`
// (the C172's keyboard assists). Any other section is a leftover keymap from
// an older build and is ignored.

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
    out.push_str(&format!("time = {}\n", config.time_level));

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
                if key == "time" {
                    if let Ok(n) = value.parse::<usize>() {
                        // Clamp to the known presets, not just accept any int.
                        config.time_level = n.min(crate::weather::TIME_LABELS.len() - 1);
                    }
                }
                // A leftover `engine_cold = 1` line from an older file is
                // silently ignored: the start plan belongs to the lesson now.
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
            // Any other section is a keymap from an older build; bindings are
            // fixed now, so ignore it (the next save drops it).
            Some(_) => {}
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
    app_dir().join("config.cfg")
}

/// The directory persisted app files live in (`<XDG config>/myflightsim`, or
/// `~/.config/myflightsim`). Shared by anything else that wants a save file —
/// the hop ledger keeps its `ledger.cfg` here too.
pub fn app_dir() -> PathBuf {
    let base = std::env::var_os("XDG_CONFIG_HOME")
        .map(PathBuf::from)
        .filter(|p| p.is_absolute())
        .or_else(|| std::env::var_os("HOME").map(|h| PathBuf::from(h).join(".config")))
        .unwrap_or_else(|| PathBuf::from("."));
    base.join("myflightsim")
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

    #[test]
    fn round_trips_through_the_text_format() {
        let mut config = GameConfig {
            weather_level: 2,
            wind_level: 1,
            time_level: 2,
            ..default()
        };
        config.set_bank_hold("c172p", true);
        config.set_arcade("ah1s", true);

        let text = serialize(&config);
        assert_eq!(parse(&text), config);

        // And again, so serialize is idempotent on a parsed file.
        assert_eq!(serialize(&parse(&text)), text);
    }

    #[test]
    fn arcade_mode_round_trips_per_aircraft() {
        let mut config = GameConfig::default();
        config.set_arcade("ah1s", true);
        // Realistic is the default, so it is simply not stored.
        assert!(!config.arcade("c172p"));
        assert!(config.arcade("ah1s"));

        let text = serialize(&config);
        assert_eq!(parse(&text), config);
        assert_eq!(serialize(&parse(&text)), text);
        assert!(parse(&text).arcade("ah1s"));

        // Toggling off drops the entry, so the file has no stale section.
        config.set_arcade("ah1s", false);
        assert!(!config.arcade("ah1s"));
        assert!(!serialize(&config).contains("[arcade]"));
    }

    #[test]
    fn the_f22_cannot_be_saved_into_arcade_mode() {
        // Its arcade path is mission-only, so the setting is refused and a
        // stale entry migrates away on the next save.
        let mut config = GameConfig::default();
        config.set_arcade("f22", true);
        assert!(!config.arcade("f22"));
        assert!(!serialize(&config).contains("f22"));

        let stale = parse("[arcade]\nf22 = 1\n");
        assert!(!stale.arcade("f22"));
        assert!(!serialize(&stale).contains("f22"));
    }

    #[test]
    fn arcade_section_normalizes_the_aircraft_id() {
        let cfg = parse("[arcade]\nah64d = 1\n");
        assert!(cfg.arcade("ah1s") && cfg.arcade("ah-64d"));
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
    fn time_of_day_parses_and_clamps() {
        assert_eq!(parse("time = 2\n").time_level, 2);
        // Out of range: clamp to the last preset instead of panicking later.
        assert_eq!(parse("time = 7\n").time_level, 2);
    }

    #[test]
    fn legacy_keymap_sections_are_ignored() {
        // Bindings are fixed now; an old `[aircraft]` key section must not
        // leak into the parsed settings, and the next save drops it.
        let cfg = parse("weather = 1\n\n[737]\ngear = KeyU\nelevator = KeyS KeyW\n");
        assert_eq!(cfg.weather_level, 1);
        assert!(!serialize(&cfg).contains("[737]"));
    }
}
