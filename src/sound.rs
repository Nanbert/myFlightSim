//! Procedural propeller sound. Rather than ship a recording, we *synthesize*
//! one loop of the engine tone and drive its playback speed from the FDM's
//! engine rpm — so the pitch tracks rpm the way a real prop's blade-pass tone
//! does. Synthesis (vs. pitch-shifting a recording) keeps the timbre honest:
//! the whole harmonic stack is generated at the right frequency, not warped.
//!
//! The tone is additive: a fundamental at the blade-pass frequency plus its
//! harmonics (that stack *is* the "brrrr"), a band-limited noise bed for rasp,
//! and a gentle amplitude modulation at the blade rate. Every partial is an
//! integer multiple of the fundamental and the loop spans an integer number of
//! fundamental periods, so the loop wraps with no seam/click.

use bevy::audio::{AudioPlayer, AudioSink, AudioSinkPlayback, AudioSource, PlaybackSettings, Volume};
use bevy::prelude::*;
use std::f64::consts::TAU;

use crate::sim::AircraftState;

const SAMPLE_RATE: u32 = 48_000;

/// The loop is synthesized as if the engine ran at this rpm. Playback speed is
/// then `rpm / REF_RPM`, which scales the entire spectrum — and so the
/// blade-pass tone — linearly with rpm.
const REF_RPM: f64 = 2400.0;
/// Blade-pass frequency = rpm/60 * blades. c172p drives a 2-blade prop.
const BLADES: f64 = 2.0;
/// Fundamental periods per loop. An integer count makes every partial complete
/// a whole number of cycles, which is what makes the loop seamless.
const PERIODS: u32 = 40;

/// Below this rpm the engine is treated as off and faded to silence.
const RPM_IDLE: f64 = 400.0;
const RPM_MAX: f64 = 2700.0;

/// Master loudness at full rpm.
const VOLUME: f32 = 0.5;

pub struct SoundPlugin;

impl Plugin for SoundPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Startup, start_prop_sound)
            .add_systems(Update, drive_prop_sound);
    }
}

/// Marker for the looping prop-sound entity.
#[derive(Component)]
struct PropSound;

fn start_prop_sound(mut commands: Commands, mut sources: ResMut<Assets<AudioSource>>) {
    let wav = synth_loop_wav();
    let handle = sources.add(AudioSource { bytes: wav.into() });
    // Start silent; `drive_prop_sound` fades it in once the engine is running.
    commands.spawn((
        AudioPlayer::new(handle),
        PlaybackSettings::LOOP.with_volume(Volume::Linear(0.0)),
        PropSound,
    ));
}

fn drive_prop_sound(state: Res<AircraftState>, mut q: Query<&mut AudioSink, With<PropSound>>) {
    let Ok(mut sink) = q.single_mut() else {
        return;
    };

    if state.rpm < RPM_IDLE {
        sink.set_volume(Volume::Linear(0.0));
        return;
    }

    // The whole point: blade-pass frequency follows rpm. Clamp so sub-idle or
    // over-speed readings don't drive the sink to a silly (or zero) rate.
    let speed = (state.rpm / REF_RPM).clamp(0.15, 1.6) as f32;
    sink.set_speed(speed);

    // Louder with rpm, and a touch more with throttle (load).
    let load = ((state.rpm - RPM_IDLE) / (RPM_MAX - RPM_IDLE)).clamp(0.0, 1.0) as f32;
    let vol = VOLUME * (0.35 + 0.65 * load) * (0.75 + 0.25 * state.throttle as f32);
    sink.set_volume(Volume::Linear(vol.clamp(0.0, 1.0)));
}

/// One additive voice: the partial table plus the normalization applied on top.
struct Voice {
    partials: Vec<(f64, f64, f64)>, // (frequency Hz, amplitude, phase rad)
    f0: f64,
    /// Divisor that brings the summed peak down to 1.0.
    inv_peak: f64,
}

impl Voice {
    fn build() -> Self {
        let f0 = REF_RPM / 60.0 * BLADES; // 80 Hz
        let mut rng = Lcg::new(0x5eed_1234);
        let mut partials = Vec::new();

        // Tonal body: blade-pass partials, amplitude falling as 1/h so the low
        // harmonics dominate — the "chug" of the prop.
        for h in 1..=6u32 {
            partials.push((f0 * h as f64, 1.0 / h as f64, rng.unit() * TAU));
        }
        // Rasp: dense higher harmonics at small random amplitudes/phases. Random
        // phases spread the energy into something noise-like, yet every term is
        // an integer multiple of f0 so the loop still closes seamlessly. (True
        // white noise would not be periodic and would click at the wrap.)
        for h in 7..=80u32 {
            let amp = 0.15 / (h as f64).sqrt();
            partials.push((f0 * h as f64, amp, rng.unit() * TAU));
        }

        let mut voice = Voice {
            partials,
            f0,
            inv_peak: 1.0,
        };
        // Normalize by scanning one loop.
        let n = voice.loop_len();
        let peak = (0..n)
            .map(|i| voice.raw(i as f64 / SAMPLE_RATE as f64).abs())
            .fold(0.0f64, f64::max);
        voice.inv_peak = if peak > 0.0 { 1.0 / peak } else { 1.0 };
        voice
    }

    /// Samples per loop: an integer number of fundamental periods.
    fn loop_len(&self) -> usize {
        (SAMPLE_RATE as f64 / self.f0 * PERIODS as f64).round() as usize
    }

    /// Un-normalized waveform at time `t`, including the blade-rate modulation.
    fn raw(&self, t: f64) -> f64 {
        let mut v = 0.0;
        for &(freq, amp, phase) in &self.partials {
            v += amp * (TAU * freq * t + phase).sin();
        }
        // Gentle amplitude modulation at the blade-pass rate adds the pulse.
        v * (1.0 + 0.12 * (TAU * self.f0 * t).sin())
    }

    fn sample(&self, t: f64) -> f32 {
        (self.raw(t) * self.inv_peak) as f32
    }
}

/// Synthesize the loop and encode it as 16-bit mono PCM WAV bytes.
fn synth_loop_wav() -> Vec<u8> {
    let voice = Voice::build();
    let n = voice.loop_len();
    let buf: Vec<f32> = (0..n)
        .map(|i| voice.sample(i as f64 / SAMPLE_RATE as f64))
        .collect();
    wav_i16_mono(&buf, SAMPLE_RATE)
}

/// Minimal 16-bit mono PCM WAV encoder. Bevy hands the bytes to rodio, which
/// sniffs the RIFF header and decodes them (needs bevy's `wav` feature).
fn wav_i16_mono(samples: &[f32], sample_rate: u32) -> Vec<u8> {
    const CHANNELS: u16 = 1;
    const BITS: u16 = 16;
    let block_align = CHANNELS * (BITS / 8);
    let byte_rate = sample_rate * block_align as u32;
    let data_len = (samples.len() * 2) as u32;

    let mut out = Vec::with_capacity(44 + samples.len() * 2);
    out.extend_from_slice(b"RIFF");
    out.extend_from_slice(&(36 + data_len).to_le_bytes());
    out.extend_from_slice(b"WAVE");
    out.extend_from_slice(b"fmt ");
    out.extend_from_slice(&16u32.to_le_bytes()); // fmt chunk size
    out.extend_from_slice(&1u16.to_le_bytes()); // PCM
    out.extend_from_slice(&CHANNELS.to_le_bytes());
    out.extend_from_slice(&sample_rate.to_le_bytes());
    out.extend_from_slice(&byte_rate.to_le_bytes());
    out.extend_from_slice(&block_align.to_le_bytes());
    out.extend_from_slice(&BITS.to_le_bytes());
    out.extend_from_slice(b"data");
    out.extend_from_slice(&data_len.to_le_bytes());
    for &s in samples {
        let v = (s.clamp(-1.0, 1.0) * i16::MAX as f32) as i16;
        out.extend_from_slice(&v.to_le_bytes());
    }
    out
}

/// Tiny deterministic PRNG (LCG) so the synthesized sound is reproducible.
struct Lcg(u64);

impl Lcg {
    fn new(seed: u64) -> Self {
        Lcg(seed)
    }

    fn unit(&mut self) -> f64 {
        self.0 = self
            .0
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        (self.0 >> 40) as f64 / (1u64 << 24) as f64
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Single-frequency Goertzel magnitude of `samples` at `freq`.
    fn tone_mag(samples: &[f32], freq: f64) -> f64 {
        let (mut re, mut im) = (0.0f64, 0.0f64);
        let w = TAU * freq / SAMPLE_RATE as f64;
        for (n, &x) in samples.iter().enumerate() {
            let a = w * n as f64;
            re += x as f64 * a.cos();
            im -= x as f64 * a.sin();
        }
        (re * re + im * im).sqrt() / samples.len() as f64
    }

    fn loop_samples() -> Vec<f32> {
        let voice = Voice::build();
        (0..voice.loop_len())
            .map(|i| voice.sample(i as f64 / SAMPLE_RATE as f64))
            .collect()
    }

    #[test]
    fn loop_spans_whole_periods() {
        let voice = Voice::build();
        // f0 = 2400/60*2 = 80 Hz; 48000/80 = 600 samples/period * 40 periods.
        assert!((voice.f0 - 80.0).abs() < 1e-9, "f0 = {}", voice.f0);
        assert_eq!(voice.loop_len(), 24_000);
    }

    #[test]
    fn fundamental_is_the_blade_pass_frequency() {
        let s = loop_samples();
        // The strongest low tone must sit at the blade-pass frequency (80 Hz),
        // beating its neighbours and the octave probe is a harmonic, not a peak.
        let f0 = tone_mag(&s, 80.0);
        assert!(f0 > tone_mag(&s, 70.0) * 3.0, "80 Hz not dominant over 70 Hz");
        assert!(f0 > tone_mag(&s, 90.0) * 3.0, "80 Hz not dominant over 90 Hz");
        println!("tone peak: 80 Hz -> {f0:.4}");
    }

    #[test]
    fn loop_wraps_without_a_seam() {
        let voice = Voice::build();
        let n = voice.loop_len();
        // One loop past the start is exactly one more period: periodic by
        // construction, so the wrap value matches the first sample.
        let first = voice.sample(0.0);
        let wrapped = voice.sample(n as f64 / SAMPLE_RATE as f64);
        assert!(
            (first - wrapped).abs() < 1e-4,
            "seam discontinuity: {first} vs {wrapped}"
        );
    }

    #[test]
    fn waveform_is_normalized() {
        let s = loop_samples();
        let peak = s.iter().fold(0.0f32, |m, &x| m.max(x.abs()));
        assert!((peak - 1.0).abs() < 1e-3, "peak = {peak}");
    }

    #[test]
    fn wav_header_is_well_formed() {
        let wav = synth_loop_wav();
        assert_eq!(&wav[0..4], b"RIFF");
        assert_eq!(&wav[8..12], b"WAVE");
        assert_eq!(&wav[36..40], b"data");
        // 44-byte header + 24000 samples * 2 bytes.
        assert_eq!(wav.len(), 44 + 24_000 * 2);
    }
}
