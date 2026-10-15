//! Drives the start menu by injecting `KeyboardInput` messages (this box has no
//! `wtype`/`ydotool`), screenshotting each screen: default, ABOUT, INFO,
//! SETTINGS, and PLAY → flight.
//!
//! Run: `cargo run --example menu_shot`

#[path = "../src/aircraft_profile.rs"]
mod aircraft_profile;
#[path = "../src/config.rs"]
mod config;
#[path = "../src/input.rs"]
mod input;
#[path = "../src/app_state.rs"]
mod app_state;
#[path = "../src/sim.rs"]
mod sim;
#[path = "../src/view.rs"]
mod view;
#[path = "../src/weather.rs"]
mod weather;
#[path = "../src/menu.rs"]
mod menu;

use bevy::input::keyboard::{Key, KeyboardInput, NativeKey};
use bevy::input::ButtonState;
use bevy::prelude::*;
use bevy::render::view::window::screenshot::{save_to_disk, Screenshot};
use bevy::window::PrimaryWindow;
use bevy::winit::WinitSettings;

#[derive(Resource, Default)]
struct Frame(u32);

/// Screenshots must not touch the real config file, so the sink is a no-op and
/// the config starts empty.
struct NullStore;
impl config::ConfigStore for NullStore {
    fn load(&self) -> Option<String> {
        None
    }
    fn save(&self, _: &str) {}
}

/// (frame, key code, logical key). Pressed on the frame, released the next.
const SCRIPT: &[(u32, KeyCode, Key)] = &[
    // → ABOUT (top 0 → 5), then open it.
    (80, KeyCode::ArrowRight, Key::ArrowRight),
    (90, KeyCode::ArrowRight, Key::ArrowRight),
    (100, KeyCode::ArrowRight, Key::ArrowRight),
    (110, KeyCode::ArrowRight, Key::ArrowRight),
    (120, KeyCode::ArrowRight, Key::ArrowRight),
    (140, KeyCode::Enter, Key::Enter),
    (200, KeyCode::Escape, Key::Escape), // close ABOUT
    // Wrap to CESSNA (5 → 0), drop to sub-row, → INFO, open it.
    (220, KeyCode::ArrowRight, Key::ArrowRight),
    (240, KeyCode::ArrowDown, Key::ArrowDown),
    (260, KeyCode::ArrowRight, Key::ArrowRight),
    (280, KeyCode::Enter, Key::Enter),
    (340, KeyCode::Escape, Key::Escape), // close INFO
    // → aircraft SETTINGS (the control editor), open it, and rebind row 0:
    // Enter arms the capture, then K is assigned.
    (360, KeyCode::ArrowRight, Key::ArrowRight),
    (380, KeyCode::Enter, Key::Enter),
    (400, KeyCode::Enter, Key::Enter),
    // Only `key_code` drives `ButtonInput<KeyCode>`; the logical key is unused
    // here (a letter has no const-constructible `Key` variant).
    (410, KeyCode::KeyK, Key::Unidentified(NativeKey::Unidentified)),
    (440, KeyCode::Escape, Key::Escape), // close aircraft SETTINGS
    // Up to the top row, over to the global SETTINGS entry, open it, and bump
    // the weather two steps (CLEAR -> LIGHT RAIN -> RAIN).
    (460, KeyCode::ArrowUp, Key::ArrowUp),
    (470, KeyCode::ArrowRight, Key::ArrowRight),
    (480, KeyCode::ArrowRight, Key::ArrowRight),
    (490, KeyCode::ArrowRight, Key::ArrowRight),
    (500, KeyCode::ArrowRight, Key::ArrowRight),
    (520, KeyCode::Enter, Key::Enter),
    (540, KeyCode::ArrowRight, Key::ArrowRight),
    (560, KeyCode::ArrowRight, Key::ArrowRight),
    (600, KeyCode::Escape, Key::Escape), // close global SETTINGS
    // ← back to CESSNA, down to the sub-row (its cursor is already on PLAY),
    // launch.
    (620, KeyCode::ArrowLeft, Key::ArrowLeft),
    (630, KeyCode::ArrowLeft, Key::ArrowLeft),
    (640, KeyCode::ArrowLeft, Key::ArrowLeft),
    (650, KeyCode::ArrowLeft, Key::ArrowLeft),
    (670, KeyCode::ArrowDown, Key::ArrowDown),
    (690, KeyCode::Enter, Key::Enter),
];

fn main() {
    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "menu_shot".into(),
                resolution: (1200, 800).into(),
                ..default()
            }),
            ..default()
        }))
        .insert_resource(Time::<Fixed>::from_hz(1.0 / sim::DT))
        .add_plugins((
            sim::SimPlugin,
            config::ConfigPlugin,
            input::InputPlugin,
            view::ViewPlugin,
            weather::WeatherPlugin,
            menu::MenuPlugin,
        ))
        // Override the file sink the plugin installed: no reads or writes here.
        .insert_resource(config::GameConfig::default())
        .insert_resource(config::ConfigSink(Box::new(NullStore)))
        // Keep the frame loop running even while the window is unfocused — the
        // default game mode starves the frame-indexed script below to ~1 Hz when
        // the compositor steals focus.
        .insert_resource(WinitSettings::continuous())
        .init_resource::<Frame>()
        // `First` runs before `bevy_input`'s PreUpdate pass, so our synthetic
        // messages are what `keyboard_input_system` translates this frame.
        .add_systems(First, inject)
        .add_systems(Update, shot)
        .run();
}

fn inject(
    frame: Res<Frame>,
    mut writer: MessageWriter<KeyboardInput>,
    q: Query<Entity, With<PrimaryWindow>>,
) {
    let Ok(window) = q.single() else {
        return;
    };
    let f = frame.0;
    for (n, key_code, logical_key) in SCRIPT {
        let state = if f == *n {
            ButtonState::Pressed
        } else if f == *n + 1 {
            ButtonState::Released
        } else {
            continue;
        };
        writer.write(KeyboardInput {
            key_code: *key_code,
            logical_key: logical_key.clone(),
            state,
            text: None,
            repeat: false,
            window,
        });
    }
}

fn shot(mut commands: Commands, mut f: ResMut<Frame>, mut exit: MessageWriter<AppExit>) {
    f.0 += 1;
    for (n, path) in [
        (70u32, "/tmp/menu_1_default.png"),
        (180, "/tmp/menu_2_about.png"),
        (320, "/tmp/menu_3_info.png"),
        (420, "/tmp/menu_4_settings.png"),
        (580, "/tmp/menu_5_global.png"),
        (800, "/tmp/menu_6_play.png"),
    ] {
        if f.0 == n {
            commands
                .spawn(Screenshot::primary_window())
                .observe(save_to_disk(path));
            info!("screenshot -> {path}");
        }
    }
    if f.0 == 860 {
        exit.write(AppExit::Success);
    }
}
