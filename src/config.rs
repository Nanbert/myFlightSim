//! Persisted user settings: weather/wind/time, per-aircraft assists, the
//! AH-64D's arcade choice and the F-22's arcade combat loadout.
//!
//! Storage sits behind [`ConfigStore`] so the native build can write a file
//! while the browser (wasm) build supplies a `localStorage` sink; the rest
//! of the app only ever talks to the trait. The format is hand-rolled text (no
//! serde dependency), which keeps that wasm story simple.
//!
//! Key bindings are fixed in [`crate::aircraft_profile`], so the file never
//! stores controls; sections left behind by older builds are ignored.

use std::collections::{HashMap, HashSet};
#[cfg(not(target_arch = "wasm32"))]
use std::path::PathBuf;

use bevy::prelude::*;

use crate::aircraft_profile;
use crate::weather::Weather;

/// The sink the config text comes from and goes to. Native builds use
/// [`FileStore`]; the wasm build implements this over `localStorage`.
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

/// Per-aircraft keyboard-feel multipliers, all `1.0` at the code's defaults.
/// The settings page shows absolute values (seconds, percent); the file keeps
/// the ratio to default, so retuning a baseline never warps a dialed feel.
/// Absent entry = all-defaults (the getters answer `Default`).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FeelSetting {
    /// Full-travel **time** multipliers: 1.2 means the ramp rate divides by
    /// 1.2, i.e. the axis takes 20 % longer to sweep. (Time is the thing the
    /// player feels and the page shows; the rate divides at use.)
    pub travel_elevator: f64,
    pub travel_aileron: f64,
    pub travel_rudder: f64,
    pub travel_throttle: f64,
    /// Tap nudge size multiplier.
    pub tap: f64,
    /// Release-linger time multiplier: how long the axis rests before its
    /// spring-back starts.
    pub linger: f64,
    /// Spring-back ramp speed multiplier on top of the profile's
    /// fraction-of-press-rate.
    pub release: f64,
    /// Fine-mode step scale multiplier.
    pub fine: f64,
}

impl Default for FeelSetting {
    fn default() -> Self {
        Self {
            travel_elevator: 1.0,
            travel_aileron: 1.0,
            travel_rudder: 1.0,
            travel_throttle: 1.0,
            tap: 1.0,
            linger: 1.0,
            release: 1.0,
            fine: 1.0,
        }
    }
}

/// Every feel multiplier lives in the same band; the page steps inside it.
const FEEL_MULT_MIN: f64 = 0.25;
const FEEL_MULT_MAX: f64 = 4.0;

/// A finite or infinite store choice on the F-22's combat settings page.
/// `Stock` keeps the mission's built-in loadout, `Count` is a literal 1-10
/// rounds/missiles/flares choice, and `Infinite` never depletes.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum AmmoSetting {
    #[default]
    Stock,
    Count(u8),
    Infinite,
}

impl AmmoSetting {
    /// The configured finite count, or `None` for stock/infinite. Combat treats
    /// stock as its own default constant and infinite as "never decrement".
    pub fn count(self) -> Option<u8> {
        match self {
            AmmoSetting::Count(n) => Some(n),
            _ => None,
        }
    }

    /// The next choice in the menu cycle: STOCK, 1 ... 10, INF (and back).
    pub fn cycled(self, dir: isize) -> Self {
        const CHOICES: usize = 12; // stock + 1..=10 + infinite
        let index = match self {
            AmmoSetting::Stock => 0,
            AmmoSetting::Count(n) => n.clamp(1, 10) as usize,
            AmmoSetting::Infinite => 11,
        };
        let next = if dir >= 0 {
            (index + 1) % CHOICES
        } else {
            (index + CHOICES - 1) % CHOICES
        };
        match next {
            0 => AmmoSetting::Stock,
            1..=10 => AmmoSetting::Count(next as u8),
            _ => AmmoSetting::Infinite,
        }
    }
}

/// The F-22's arcade combat loadout, set on its SETTINGS page. The stock values
/// mirror the constants in `combat.rs` (3 bandits per wave, 480 gun rounds,
/// 2 IR + 2 BVR missiles and 8 flares).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct CombatConfig {
    /// Bandits per wave, 1..=10.
    pub enemies: u8,
    pub rounds: AmmoSetting,
    pub missiles: AmmoSetting,
    pub flares: AmmoSetting,
}

impl Default for CombatConfig {
    fn default() -> Self {
        Self {
            enemies: 3,
            rounds: AmmoSetting::Stock,
            missiles: AmmoSetting::Stock,
            flares: AmmoSetting::Stock,
        }
    }
}

/// `1 ..= 10`, wrapping. Shared by the menu and its tests.
pub fn cycle_enemies(n: u8, dir: isize) -> u8 {
    let n = n.clamp(1, 10) as isize;
    (((n - 1 + dir).rem_euclid(10)) + 1) as u8
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
    /// Per-aircraft keyboard-feel multipliers. An id is present iff its feel
    /// differs from the defaults anywhere.
    feel: HashMap<String, FeelSetting>,
    /// Aircraft set to fly in arcade mode, by canonical id. An id is present iff
    /// arcade is on, so the realistic default is simply absence.
    arcade: HashSet<String>,
    /// Per-aircraft arcade combat loadouts. Only the F-22 writes one; an id is
    /// present iff its loadout differs from the stock mission defaults.
    combat: HashMap<String, CombatConfig>,
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

    /// The keyboard-feel multipliers for an aircraft (all defaults when absent).
    pub fn feel(&self, id: &str) -> FeelSetting {
        self.feel
            .get(aircraft_profile::normalized(id))
            .copied()
            .unwrap_or_default()
    }

    /// Adjust one feel knob, clamped into its band. The entry is dropped once
    /// every multiplier is back at default, keeping the file minimal.
    pub fn set_feel(&mut self, id: &str, change: impl FnOnce(&mut FeelSetting)) {
        let id = aircraft_profile::normalized(id).to_owned();
        let mut f = self.feel.get(&id).copied().unwrap_or_default();
        change(&mut f);
        for m in [
            &mut f.travel_elevator,
            &mut f.travel_aileron,
            &mut f.travel_rudder,
            &mut f.travel_throttle,
            &mut f.tap,
            &mut f.linger,
            &mut f.release,
            &mut f.fine,
        ] {
            *m = m.clamp(FEEL_MULT_MIN, FEEL_MULT_MAX);
        }
        if f == FeelSetting::default() {
            self.feel.remove(&id);
        } else {
            self.feel.insert(id, f);
        }
    }

    /// Wipe every feel knob back to the defaults (the page's RESTORE ALL).
    pub fn reset_feel(&mut self, id: &str) {
        self.feel.remove(aircraft_profile::normalized(id));
    }

    /// The arcade combat loadout for an aircraft (stock when absent).
    pub fn combat(&self, id: &str) -> CombatConfig {
        self.combat
            .get(aircraft_profile::normalized(id))
            .copied()
            .unwrap_or_default()
    }

    /// Adjust an aircraft's combat loadout, keeping enemies in 1..=10 and
    /// dropping the entry once every field is back at stock.
    pub fn set_combat(&mut self, id: &str, change: impl FnOnce(&mut CombatConfig)) {
        let id = aircraft_profile::normalized(id).to_owned();
        let mut c = self.combat.get(&id).copied().unwrap_or_default();
        change(&mut c);
        c.enemies = c.enemies.clamp(1, 10);
        if c == CombatConfig::default() {
            self.combat.remove(&id);
        } else {
            self.combat.insert(id, c);
        }
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
        let store = default_store();
        let config = store.load().map(|t| parse(&t)).unwrap_or_default();
        app.insert_resource(config)
            .insert_resource(ConfigSink(store))
            // Startup, so `Weather` (from `WeatherPlugin`) already exists.
            .add_systems(Startup, apply_weather);
    }
}

/// The store this build persists through: a config file on the desktop,
/// `localStorage` on the web.
fn default_store() -> Box<dyn ConfigStore> {
    #[cfg(not(target_arch = "wasm32"))]
    {
        Box::new(FileStore::new())
    }
    #[cfg(target_arch = "wasm32")]
    {
        Box::new(LocalStorageStore::new(CONFIG_KEY))
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
//   [f22.combat]
//   enemies = 5
//   rounds = inf
//   missiles = 4
//   flares = stock
//
// The reserved section names are `arcade` (aircraft ids), `<id>.assist`
// (the C172's keyboard assists), `<id>.feel` (feel multipliers) and
// `<id>.combat` (the F-22's arcade loadout). Any other section is a leftover
// keymap from an older build and is ignored.

/// The section header whose keys are aircraft ids, not control ids.
const ARCADE_SECTION: &str = "arcade";
/// Per-aircraft assist sections are written as `[c172p.assist]`.
const ASSIST_SUFFIX: &str = ".assist";
/// Per-aircraft keyboard-feel sections are written as `[c172p.feel]`.
const FEEL_SUFFIX: &str = ".feel";
/// Per-aircraft arcade combat loadouts are written as `[f22.combat]`.
const COMBAT_SUFFIX: &str = ".combat";

fn truthy(value: &str) -> bool {
    matches!(value, "1" | "true" | "on")
}

/// Parse a combat store cell: `inf`, `stock`, or a literal 1-10. Anything else
/// falls back to stock, so a hand-edited file can never inject a bad count.
fn parse_ammo(value: &str) -> AmmoSetting {
    match value.to_ascii_lowercase().as_str() {
        "inf" | "infinite" | "-1" => AmmoSetting::Infinite,
        "stock" | "default" | "0" => AmmoSetting::Stock,
        other => other
            .parse::<u8>()
            .ok()
            .filter(|n| (1..=10).contains(n))
            .map(AmmoSetting::Count)
            .unwrap_or(AmmoSetting::Stock),
    }
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

    let mut feel_ids: Vec<&String> = config.feel.keys().collect();
    feel_ids.sort();
    for id in feel_ids {
        let f = config.feel[id];
        out.push_str(&format!("\n[{id}.feel]\n"));
        for (key, m) in [
            ("elev-travel", f.travel_elevator),
            ("aile-travel", f.travel_aileron),
            ("rudd-travel", f.travel_rudder),
            ("thro-travel", f.travel_throttle),
            ("tap", f.tap),
            ("linger", f.linger),
            ("return", f.release),
            ("fine", f.fine),
        ] {
            if m != 1.0 {
                out.push_str(&format!("{key} = {m:.3}\n"));
            }
        }
    }

    let mut combat_ids: Vec<&String> = config.combat.keys().collect();
    combat_ids.sort();
    for id in combat_ids {
        let c = config.combat[id];
        out.push_str(&format!("\n[{id}.combat]\n"));
        if c.enemies != CombatConfig::default().enemies {
            out.push_str(&format!("enemies = {}\n", c.enemies));
        }
        for (key, choice) in [
            ("rounds", c.rounds),
            ("missiles", c.missiles),
            ("flares", c.flares),
        ] {
            match choice {
                AmmoSetting::Stock => {}
                AmmoSetting::Count(n) => out.push_str(&format!("{key} = {n}\n")),
                AmmoSetting::Infinite => out.push_str(&format!("{key} = inf\n")),
            }
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
            Some(s) if s.ends_with(FEEL_SUFFIX) => {
                let id = &s[..s.len() - FEEL_SUFFIX.len()];
                if let Ok(v) = value.parse::<f64>() {
                    // Clamp at ingest too: a hand-edited file can hold junk.
                    let v = v.clamp(FEEL_MULT_MIN, FEEL_MULT_MAX);
                    config.set_feel(id, |f| match key {
                        "elev-travel" => f.travel_elevator = v,
                        "aile-travel" => f.travel_aileron = v,
                        "rudd-travel" => f.travel_rudder = v,
                        "thro-travel" => f.travel_throttle = v,
                        "tap" => f.tap = v,
                        "linger" => f.linger = v,
                        "return" => f.release = v,
                        "fine" => f.fine = v,
                        _ => {}
                    });
                }
            }
            Some(s) if s.ends_with(COMBAT_SUFFIX) => {
                let id = &s[..s.len() - COMBAT_SUFFIX.len()];
                match key {
                    "enemies" => {
                        if let Ok(n) = value.parse::<u8>() {
                            config.set_combat(id, |c| c.enemies = n);
                        }
                    }
                    "rounds" => {
                        let choice = parse_ammo(value);
                        config.set_combat(id, |c| c.rounds = choice);
                    }
                    "missiles" => {
                        let choice = parse_ammo(value);
                        config.set_combat(id, |c| c.missiles = choice);
                    }
                    "flares" => {
                        let choice = parse_ammo(value);
                        config.set_combat(id, |c| c.flares = choice);
                    }
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
#[cfg(not(target_arch = "wasm32"))]
pub struct FileStore {
    path: PathBuf,
}

#[cfg(not(target_arch = "wasm32"))]
impl FileStore {
    pub fn new() -> Self {
        FileStore {
            path: default_path(),
        }
    }

    /// A store for a sibling file in the same directory (the hop ledger).
    pub fn at(path: PathBuf) -> Self {
        FileStore { path }
    }
}

#[cfg(not(target_arch = "wasm32"))]
impl Default for FileStore {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(not(target_arch = "wasm32"))]
fn default_path() -> PathBuf {
    app_dir().join("config.cfg")
}

/// The directory persisted app files live in (`<XDG config>/myflightsim`, or
/// `~/.config/myflightsim`). Shared by anything else that wants a save file —
/// the hop ledger keeps its `ledger.cfg` here too.
#[cfg(not(target_arch = "wasm32"))]
pub fn app_dir() -> PathBuf {
    let base = std::env::var_os("XDG_CONFIG_HOME")
        .map(PathBuf::from)
        .filter(|p| p.is_absolute())
        .or_else(|| std::env::var_os("HOME").map(|h| PathBuf::from(h).join(".config")))
        .unwrap_or_else(|| PathBuf::from("."));
    base.join("myflightsim")
}

#[cfg(not(target_arch = "wasm32"))]
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

// --- web store ---------------------------------------------------------------

/// The `localStorage` keys this game owns, so a shared origin (a dev server
/// hosting several projects) can't collide with us.
#[cfg(target_arch = "wasm32")]
const CONFIG_KEY: &str = "myflightsim.config";
#[cfg(target_arch = "wasm32")]
pub const LEDGER_KEY: &str = "myflightsim.ledger";

/// Reads and writes the same hand-rolled text through `localStorage` under a
/// key. No web handle is held between calls — the storage is re-fetched per
/// call — which keeps the store `Send + Sync` for free.
#[cfg(target_arch = "wasm32")]
pub struct LocalStorageStore {
    key: &'static str,
}

#[cfg(target_arch = "wasm32")]
impl LocalStorageStore {
    pub fn new(key: &'static str) -> Self {
        LocalStorageStore { key }
    }
}

#[cfg(target_arch = "wasm32")]
impl ConfigStore for LocalStorageStore {
    fn load(&self) -> Option<String> {
        let storage = web_sys::window()?.local_storage().ok()??;
        storage.get_item(self.key).ok().flatten()
    }

    fn save(&self, text: &str) {
        if let Some(Ok(Some(storage))) = web_sys::window().map(|w| w.local_storage()) {
            let _ = storage.set_item(self.key, text);
        }
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
    fn feel_knobs_round_trip_per_aircraft() {
        let mut config = GameConfig::default();
        config.set_feel("c172p", |f| f.travel_elevator = 1.2);
        config.set_feel("737", |f| {
            f.linger = 1.5;
            f.release = 0.8;
        });
        // An aircraft left untouched stores nothing.
        assert_eq!(config.feel("ah1s"), FeelSetting::default());

        let text = serialize(&config);
        assert_eq!(parse(&text), config);
        assert_eq!(serialize(&parse(&text)), text);
        let again = parse(&text);
        assert_eq!(again.feel("c172p").travel_elevator, 1.2);
        assert_eq!(again.feel("737").linger, 1.5);

        // Individual knobs read off a rotted id default cleanly.
        assert_eq!(again.feel("c172p").tap, 1.0);
    }

    #[test]
    fn feel_clamps_junk_at_ingest_and_erase_is_idempotent() {
        let cfg = parse("[c172p.feel]\nelev-travel = 99\ntap = -5\njunk = 7\n");
        let f = cfg.feel("c172p");
        assert_eq!(f.travel_elevator, 4.0);
        assert_eq!(f.tap, 0.25);

        // Restoring every knob drops the section, and restoring an aircraft
        // with no section is a quiet no-op.
        let mut config = GameConfig::default();
        config.set_feel("c172p", |f| f.travel_aileron = 2.0);
        assert!(serialize(&config).contains("[c172p.feel]"));
        config.set_feel("c172p", |f| f.travel_aileron = 1.0);
        assert_eq!(config.feel("c172p"), FeelSetting::default());
        assert!(!serialize(&config).contains("[c172p.feel]"));
        config.reset_feel("c172p"); // was already default: fine
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
    fn combat_loadout_round_trips_and_clamps() {
        let mut config = GameConfig::default();
        assert_eq!(config.combat("f22"), CombatConfig::default());
        config.set_combat("f22", |c| {
            c.enemies = 7;
            c.rounds = AmmoSetting::Count(4);
            c.missiles = AmmoSetting::Infinite;
            c.flares = AmmoSetting::Count(10);
        });

        let text = serialize(&config);
        assert!(text.contains("[f22.combat]"));
        assert_eq!(parse(&text), config);
        assert_eq!(serialize(&parse(&text)), text);

        // Junk enemies clamp into 1..=10; junk stores fall back to stock.
        let cfg = parse("[f22.combat]\nenemies = 0\nrounds = 99\nmissiles = inf\n");
        assert_eq!(cfg.combat("f22").enemies, 1);
        assert_eq!(cfg.combat("f22").rounds, AmmoSetting::Stock);
        assert_eq!(cfg.combat("f22").missiles, AmmoSetting::Infinite);

        // Back to stock drops the section entirely.
        config.set_combat("f22", |c| *c = CombatConfig::default());
        assert!(!serialize(&config).contains("[f22.combat]"));
    }

    #[test]
    fn ammo_choice_cycles_stock_numbers_and_infinite() {
        assert_eq!(AmmoSetting::Stock.cycled(1), AmmoSetting::Count(1));
        assert_eq!(AmmoSetting::Count(10).cycled(1), AmmoSetting::Infinite);
        assert_eq!(AmmoSetting::Infinite.cycled(1), AmmoSetting::Stock);
        assert_eq!(AmmoSetting::Stock.cycled(-1), AmmoSetting::Infinite);
        assert_eq!(cycle_enemies(10, 1), 1);
        assert_eq!(cycle_enemies(1, -1), 10);
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
