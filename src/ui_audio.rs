//! Menu UI sound effects: one synthesized "blip family" in memory.
//!
//! Same self-contained approach as the combat launch tone: the sounds are
//! synthesized as WAV bytes at startup and kept as `AudioSource` handles — no
//! asset files to ship. Every variant shares one recipe (the approved menu
//! click: a ~970 Hz sine blip with a bright transient sparkle, a soft second
//! harmonic and a fast exponential decay), so the family reads as one voice:
//! nav moves are a quieter, shorter blip, Enter is the plain blip, opening a
//! card is an ascending pair and closing one a descending pair.
//!
//! Callers never spawn audio directly: they emit [`UiSfxEvent`]s and the
//! reader system plays them, so hot code paths stay free of audio plumbing.

use bevy::audio::{AudioPlayer, AudioSource, PlaybackSettings, Volume};
use bevy::prelude::*;
use std::sync::Arc;

use crate::combat::wav_bytes;

const RATE: u32 = 44_100;
/// Family anchor pitch (Hz): every variant sits on this base tone.
const BASE_HZ: f32 = 970.0;
/// Exponential-decay time constant of the full select blip (s).
const SELECT_TAU_S: f32 = 0.055;
/// Noise-sparkle length on the attack transient (s).
const SPARKLE_S: f32 = 0.012;
/// Silence between the two blips of a confirm/back pair (s).
const PAIR_GAP_S: f32 = 0.05;
/// Playback loudness shared by all variants; the per-variant balance lives in
/// the synthesis (waveform amplitudes), so one playback constant fits all.
const PLAYBACK_VOLUME: f32 = 0.6;

/// Which menu sound to play.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum UiSfxKind {
    /// Cursor/row moved: quieter, shorter blip.
    Hover,
    /// Enter: activate the pointed-at item — the plain approved blip.
    Select,
    /// A card opened: ascending pair.
    Confirm,
    /// A card closed: descending pair.
    Back,
}

/// Fired by menu navigation; consumed by [`play_sfx_events`].
#[derive(Message)]
pub struct UiSfxEvent(pub UiSfxKind);

/// Pre-built handles for the four menu blips.
#[derive(Resource)]
pub struct UiSfx {
    select: Handle<AudioSource>,
    hover: Handle<AudioSource>,
    confirm: Handle<AudioSource>,
    back: Handle<AudioSource>,
}

pub struct UiAudioPlugin;

impl Plugin for UiAudioPlugin {
    fn build(&self, app: &mut App) {
        app.add_message::<UiSfxEvent>()
            .add_systems(Startup, setup_sfx)
            .add_systems(Update, play_sfx_events);
    }
}

/// Synthesize the family once, in memory (no asset files, like the launch
/// tone in `combat.rs`).
fn setup_sfx(mut commands: Commands, mut sources: ResMut<Assets<AudioSource>>) {
    let mut add = |samples: Vec<i16>| {
        sources.add(AudioSource {
            bytes: Arc::from(wav_bytes(&samples)),
        })
    };
    let select = add(blip(BASE_HZ, SELECT_TAU_S, 0.32, 0.25, 0.75, 0.25));
    // Nav moves: same pitch, tighter decay, dimmer sparkle, quieter peak.
    let hover = add(blip(BASE_HZ, 0.03, 0.16, 0.18, 0.38, 0.12));
    // Opening a card: an ascending pair (base -> a fourth up).
    let confirm = add(pair(BASE_HZ, 1_293.0, 0.30, 0.25, 0.70));
    // Closing a card: a descending pair (base -> an octave down).
    let back = add(pair(BASE_HZ, 647.0, 0.30, 0.25, 0.70));
    commands.insert_resource(UiSfx {
        select,
        hover,
        confirm,
        back,
    });
}

/// Spawn a one-shot player; [`PlaybackSettings::DESPAWN`] reaps it when done.
/// The entity is unscoped, so a blip fired on a state transition survives it.
pub fn play(commands: &mut Commands, sfx: &UiSfx, kind: UiSfxKind) {
    let handle = match kind {
        UiSfxKind::Hover => sfx.hover.clone(),
        UiSfxKind::Select => sfx.select.clone(),
        UiSfxKind::Confirm => sfx.confirm.clone(),
        UiSfxKind::Back => sfx.back.clone(),
    };
    commands.spawn((
        AudioPlayer::new(handle),
        PlaybackSettings::DESPAWN.with_volume(Volume::Linear(PLAYBACK_VOLUME)),
    ));
}

/// Drain the event queue into one-shot players.
fn play_sfx_events(
    mut events: MessageReader<UiSfxEvent>,
    sfx: Option<Res<UiSfx>>,
    mut commands: Commands,
) {
    let Some(sfx) = sfx else {
        return;
    };
    for event in events.read() {
        play(&mut commands, &sfx, event.0);
    }
}

/// One sine blip: instant attack, exponential decay, a bright noise sparkle on
/// the transient and a soft second harmonic — the approved menu-click recipe.
/// The sparkle uses a deterministic LCG so the sound is stable across runs.
fn blip(hz: f32, tau_s: f32, sparkle: f32, harm2: f32, peak: f32, dur_s: f32) -> Vec<i16> {
    let rate = RATE as f32;
    let n = (rate * dur_s).round() as usize;
    let sparkle_n = (rate * SPARKLE_S).round() as usize;
    let mut out = Vec::with_capacity(n);
    let mut seed = 0x2545_4917u32;
    for i in 0..n {
        let t = i as f32 / rate;
        let env = (-t / tau_s).exp();
        let mut s = (std::f32::consts::TAU * hz * t).sin()
            + harm2 * (std::f32::consts::TAU * 2.0 * hz * t).sin();
        s *= env;
        if i < sparkle_n {
            seed = seed.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
            let r = (seed >> 8) as f32 / 8_388_608.0 - 1.0;
            s += sparkle * (1.0 - i as f32 / sparkle_n as f32) * r;
        }
        out.push((s.clamp(-1.0, 1.0) * peak * i16::MAX as f32) as i16);
    }
    out
}

/// Two blips back to back with a short gap — the confirm/back rhythm.
fn pair(hz_a: f32, hz_b: f32, sparkle: f32, harm2: f32, peak: f32) -> Vec<i16> {
    let mut out = blip(hz_a, SELECT_TAU_S, sparkle, harm2, peak, 0.18);
    out.extend(std::iter::repeat_n(
        0i16,
        (RATE as f32 * PAIR_GAP_S).round() as usize,
    ));
    out.extend(blip(hz_b, SELECT_TAU_S, sparkle, harm2, peak, 0.18));
    out
}
