//! Piston-engine start sounds for the C172's cold-start flow, synthesized in
//! memory like the combat launch tone and the menu blips (`ui_audio.rs`) —
//! no asset files to ship.
//!
//! The flow has three audible moments, each triggered by a state edge rather
//! than a key press (the key handling already routes through the FDM):
//!
//! - **Master relay** (`M`): `ignition-enable` toggles → a soft relay clunk,
//!   the same sound on and off like a real switch.
//! - **Starter crank** (`I` held): `starter-cmd-norm > 0.5` while the engine
//!   is not yet running → a looping motor whine, despawned on release.
//! - **The catch** (`running` latches true): the starter whine gives way to a
//!   few combustion pops and the engine settles into an idle rumble.
//!
//! On top of that sits the continuous **prop loop**: five pre-rendered
//! seamless 3 s loops at fixed rpms (700–2700) all play at once, each with a
//! partition-of-unity weight from the live `engine-rpm`, so the engine's
//! pitch and character morph continuously as the throttle moves — the classic
//! multi-loop crossfade, since `PlaybackSettings` has no playback rate.
//!
//! Everything is gated to the C172 (`sim.name`); warm spawns, the jets and
//! the heli stay silent.

use bevy::audio::{
    AudioPlayer, AudioSink, AudioSinkPlayback, AudioSource, PlaybackSettings, Volume,
};
use bevy::prelude::*;
use std::sync::Arc;

use crate::app_state::{unpaused, AppState};
use crate::combat::wav_bytes;
use crate::sim::{AircraftState, Sim};

const RATE: u32 = 44_100;
/// Playback loudness per moment (the synthesis carries the internal balance).
const CLUNK_VOLUME: f32 = 0.5;
const CRANK_VOLUME: f32 = 0.45;
const CATCH_VOLUME: f32 = 0.6;
/// Master loudness of the continuous prop loop (per-band weights crossfade it).
const PROP_VOLUME: f32 = 0.55;
/// The partition-of-unity weights keep the *signal* level flat from idle to
/// redline, but the ear disagrees: the 23 Hz idle drone reads as much louder
/// than the 73 Hz cruise whine at the same dBFS (equal-loudness contours), and
/// an idle engine really is quieter than a loaded one. This curve compensates
/// by rolling the whole loop down as rpm falls, anchored at 1.0 at redline —
/// the point where the loop levels were judged correct.
/// Exponent 0 = flat (energy-preserving), 1.0 = linear with rpm.
const PROP_LOUDNESS_EXP: f64 = 0.5;
/// Fixed-rpm loop centers, idle through redline. Even 500 rpm spacing = the
/// crossfade width, so the partition-of-unity weights below sum to exactly 1
/// at every rpm in range.
const PROP_BANDS: [f64; 5] = [700.0, 1200.0, 1700.0, 2200.0, 2700.0];
/// Weight reach: a band is full at its center, zero half a spacing away —
/// overlapping neighbors then sum to exactly 1 at every rpm in range.
const PROP_BAND_WIDTH: f64 = 500.0;
/// The five band loops, generated from two MCP recordings (idle 700 rpm,
/// cruise 2400 rpm): each band is one of the two tape-resampled by its rpm
/// ratio, so pitch/rhythm/texture scale like the real engine. Seamless loops
/// resampled uniformly stay seamless.
const PROP_BAND_PATHS: [&str; 5] = [
    "audio/prop/band_700.wav",
    "audio/prop/band_1200.wav",
    "audio/prop/band_1700.wav",
    "audio/prop/band_2200.wav",
    "audio/prop/band_2700.wav",
];

/// The starter-crank loop plus the two one-shots, and the loop player handle.
#[derive(Resource)]
pub struct EngineSfx {
    clunk: Handle<AudioSource>,
    crank: Handle<AudioSource>,
    catch: Handle<AudioSource>,
    /// Live crank-loop player, spawned while the starter is engaged.
    loop_player: Option<Entity>,
    /// The last-seen `master_on` / `engine_running` pair; `None` until the
    /// first update so a warm spawn never fires a sound.
    prev: Option<(bool, bool)>,
}

/// The five pre-rendered prop bands and their live players.
#[derive(Resource)]
pub struct PropLoop {
    handles: [Handle<AudioSource>; 5],
    /// The five band players, alive for one whole flight and crossfaded by
    /// volume — volume 0 means audible silence, so the sinks are never churned.
    players: [Option<Entity>; 5],
    /// Smoothed rpm so throttle steps glide instead of stepping.
    smooth_rpm: f64,
    /// One rpm print per flight: keeps the whole chain checkable from the log.
    rpm_logged: bool,
}

/// Marks the looping prop band players.
#[derive(Component)]
struct PropBand;

pub struct EngineAudioPlugin;

impl Plugin for EngineAudioPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Startup, setup_engine_sfx)
            .add_systems(
                Update,
                (engine_audio_edges, prop_loop_follow)
                    .chain()
                    .run_if(in_state(AppState::Flying))
                    .run_if(unpaused),
            )
            // A fresh flight resets the edge memory (a warm spawn must not
            // replay the catch) and brings the band players up; leaving drops
            // them so the menu pays no mixing cost.
            .add_systems(
                OnEnter(AppState::Flying),
                (engine_sfx_rearm, prop_loop_start),
            )
            .add_systems(
                OnExit(AppState::Flying),
                (engine_sfx_rearm, prop_loop_stop),
            );
    }
}

/// Synthesize the cold-start sounds once, in memory; the prop band loops come
/// from recorded assets (MCP-generated, see `PROP_BAND_PATHS`).
fn setup_engine_sfx(
    mut commands: Commands,
    mut sources: ResMut<Assets<AudioSource>>,
    assets: Res<AssetServer>,
) {
    let mut add = |samples: Vec<i16>| {
        sources.add(AudioSource {
            bytes: Arc::from(wav_bytes(&samples)),
        })
    };
    let clunk = add(relay_clunk());
    let crank = add(starter_crank());
    let catch = add(ignition_catch());
    commands.insert_resource(EngineSfx {
        clunk,
        crank,
        catch,
        loop_player: None,
        prev: None,
    });
    let handles: Vec<Handle<AudioSource>> = PROP_BAND_PATHS
        .iter()
        .map(|path| assets.load(*path))
        .collect();
    commands.insert_resource(PropLoop {
        handles: handles.try_into().expect("PROP_BANDS length"),
        players: Default::default(),
        smooth_rpm: 0.0,
        rpm_logged: false,
    });
}

/// Play a one-shot; despawned by [`PlaybackSettings::DESPAWN`] when done.
fn play(commands: &mut Commands, handle: &Handle<AudioSource>, volume: f32) {
    commands.spawn((
        AudioPlayer::new(handle.clone()),
        PlaybackSettings::DESPAWN.with_volume(Volume::Linear(volume)),
    ));
}

/// Watch the master/running edges and the starter command each frame.
fn engine_audio_edges(
    mut commands: Commands,
    sim: Option<Res<Sim>>,
    state: Res<AircraftState>,
    sfx: Option<ResMut<EngineSfx>>,
) {
    let Some(mut sfx) = sfx else {
        return;
    };
    // The cold-start flow is a c172p-only concept.
    let Some(sim) = sim else {
        return;
    };
    if !sim.engine_cold {
        // Not in the cold flow: just keep the loop bookkeeping consistent.
        stop_crank_loop(&mut commands, &mut sfx);
        sfx.prev = Some((state.master_on, state.engine_running));
        return;
    }

    if let Some((master, running)) = sfx.prev {
        if master != state.master_on {
            // The relay clunks on and off alike.
            play(&mut commands, &sfx.clunk, CLUNK_VOLUME);
        }
        if running && !state.engine_running {
            // Died back to cold: the catch is gone until it latches again.
            stop_crank_loop(&mut commands, &mut sfx);
        }
        if !running && state.engine_running {
            // The catch: pops and idle. The crank loop's job is done.
            play(&mut commands, &sfx.catch, CATCH_VOLUME);
            stop_crank_loop(&mut commands, &mut sfx);
        }
    }
    sfx.prev = Some((state.master_on, state.engine_running));

    // Starter cranking (hold I): a loop until the catch or the release.
    let cranking = sim.model.get("propulsion/engine[0]/starter-cmd-norm") > 0.5
        && !state.engine_running;
    match (&sfx.loop_player, cranking) {
        (None, true) => {
            let id = commands
                .spawn((
                    AudioPlayer::new(sfx.crank.clone()),
                    PlaybackSettings::LOOP.with_volume(Volume::Linear(CRANK_VOLUME)),
                ))
                .id();
            sfx.loop_player = Some(id);
        }
        (Some(_), false) => stop_crank_loop(&mut commands, &mut sfx),
        _ => {}
    }
}

/// Despawn the crank-loop player, if one is up.
fn stop_crank_loop(commands: &mut Commands, sfx: &mut EngineSfx) {
    if let Some(id) = sfx.loop_player.take() {
        commands.entity(id).despawn();
    }
}

/// Ride the live engine rpm across the five fixed-rpm bands. Bevy only reads
/// [`PlaybackSettings`] when it creates the sink, so the crossfade is driven
/// through [`AudioSinkPlayback::set_volume`] every frame; the players stay up
/// for the whole flight ([`prop_loop_start`] / [`prop_loop_stop`]) rather than
/// being recycled at each band boundary. C172 only.
fn prop_loop_follow(
    sim: Option<Res<Sim>>,
    state: Res<AircraftState>,
    time: Res<Time>,
    prop: Option<ResMut<PropLoop>>,
    mut players: Query<Option<&mut AudioSink>, With<PropBand>>,
) {
    let (Some(sim), Some(mut prop)) = (sim, prop) else {
        return;
    };
    // A piston prop is a C172-only sound in this roster.
    let piston = sim.name == "c172p";

    // Glide the rpm reading (the FDM can step it around on throttle chops);
    // an ~8 Hz time constant turns steps into a continuous morph.
    let glide = (time.delta_secs_f64() * 8.0).clamp(0.0, 1.0);
    let target = if piston { state.engine_rpm } else { 0.0 };
    prop.smooth_rpm += (target - prop.smooth_rpm) * glide;

    let master = prop.smooth_rpm.max(0.0);
    let loudness = prop_loudness_curve(master);

    if !prop.rpm_logged && prop.smooth_rpm.abs() > 1.0 {
        prop.rpm_logged = true;
        info!(
            "prop audio: aircraft={} engine_rpm={:.0} gain={:.2}",
            sim.name, state.engine_rpm, loudness
        );
    }

    for (i, band) in PROP_BANDS.iter().enumerate() {
        let weight = if piston {
            (1.0 - ((master - band).abs() / PROP_BAND_WIDTH)).clamp(0.0, 1.0)
        } else {
            0.0
        };
        // The sink only exists once bevy has decoded the loop; until then the
        // player sits at its initial (silent) volume and this simply waits.
        if let Some(id) = prop.players[i] {
            if let Ok(Some(mut sink)) = players.get_mut(id) {
                sink.set_volume(Volume::Linear(
                    weight as f32 * PROP_VOLUME * loudness,
                ));
            }
        }
    }
}

/// Ear-compensated master gain for the whole loop set: anchored at 1.0 on the
/// redline band and rolling off as rpm falls, so the flat-energy crossfade
/// also reads as a flat *loudness* ramp. See `PROP_LOUDNESS_EXP`.
fn prop_loudness_curve(rpm: f64) -> f32 {
    let redline = PROP_BANDS[PROP_BANDS.len() - 1];
    ((rpm / redline).clamp(0.0, 1.0)).powf(PROP_LOUDNESS_EXP) as f32
}

/// Bring the five band players up for a fresh flight, all muted: the rpm feed
/// wakes them through the crossfade.
fn prop_loop_start(mut commands: Commands, prop: Option<ResMut<PropLoop>>) {
    let Some(mut prop) = prop else {
        return;
    };
    for i in 0..PROP_BANDS.len() {
        if prop.players[i].is_none() {
            let handle = prop.handles[i].clone();
            prop.players[i] = Some(
                commands
                    .spawn((
                        AudioPlayer::new(handle),
                        PlaybackSettings::LOOP.with_volume(Volume::Linear(0.0)),
                        PropBand,
                    ))
                    .id(),
            );
        }
    }
    prop.smooth_rpm = 0.0;
    prop.rpm_logged = false;
}

/// Drop the band players when the flight ends.
fn prop_loop_stop(mut commands: Commands, prop: Option<ResMut<PropLoop>>) {
    let Some(mut prop) = prop else {
        return;
    };
    for slot in prop.players.iter_mut() {
        if let Some(id) = slot.take() {
            commands.entity(id).despawn();
        }
    }
}

/// Clear the edge memory and reap the crank loop, on entering *and* leaving
/// `Flying`: a fresh flight starts from silence, and a mid-crank
/// pause-to-menu must not leave the starter whining forever.
fn engine_sfx_rearm(mut commands: Commands, sfx: Option<ResMut<EngineSfx>>) {
    let Some(mut sfx) = sfx else {
        return;
    };
    stop_crank_loop(&mut commands, &mut *sfx);
    sfx.prev = None;
}

// --- synthesis (cold-start one-shots) ---------------------------------------

/// Master-switch relay: a 2 ms contact click over a ~65 Hz thump, gone in 60 ms.
fn relay_clunk() -> Vec<i16> {
    let rate = RATE as f32;
    let n = (rate * 0.06).round() as usize;
    let mut seed = 0x68e3_1a2bu32;
    let mut out = Vec::with_capacity(n);
    for i in 0..n {
        let t = i as f32 / rate;
        let thump = (std::f32::consts::TAU * 65.0 * t).sin() * (-t / 0.012).exp();
        let click = if t < 0.004 {
            seed = seed.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
            let r = (seed >> 8) as f32 / 8_388_608.0 - 1.0;
            r * (1.0 - t / 0.004)
        } else {
            0.0
        };
        out.push((((thump + 0.35 * click).clamp(-1.0, 1.0)) * 0.7 * i16::MAX as f32) as i16);
    }
    out
}

/// One crank-loop sample stream: a 1.000 s seamless loop. Every component's
/// frequency divides 44100 exactly (220, 1100, 6 Hz), so phase and wobble land
/// on their starting values at the loop point — no click, no fade hack.
fn starter_crank() -> Vec<i16> {
    let rate = RATE as f32;
    let n = RATE as usize;
    let mut out = Vec::with_capacity(n);
    for i in 0..n {
        let t = i as f32 / rate;
        // Starter motor: a mid whine with a strong second harmonic, plus the
        // ring-gear whine an octave-and-a-half up and a slow wobble.
        let wobble = 1.0 + 0.15 * (std::f32::consts::TAU * 6.0 * t).sin();
        let motor = (std::f32::consts::TAU * 220.0 * t).sin()
            + 0.5 * (std::f32::consts::TAU * 440.0 * t).sin()
            + 0.25 * (std::f32::consts::TAU * 660.0 * t).sin();
        let gear = 0.12 * (std::f32::consts::TAU * 1100.0 * t).sin();
        let s = (motor + gear) * wobble;
        out.push((s * 0.3 * i16::MAX as f32) as i16);
    }
    out
}

/// The catch, ~2.0 s: the starter whine gives up (0–0.7 s), three combustion
/// pops arrive while rpm is still half-hearted (0.35–0.65 s), then the engine
/// pulls into a steady four-cylinder idle rumble (~23 Hz firing pulses).
fn ignition_catch() -> Vec<i16> {
    let rate = RATE as f32;
    let dur = 2.0;
    let n = (rate * dur).round() as usize;
    let mut out = Vec::with_capacity(n);
    let mut seed = 0x1f2e_3d4cu32;
    // Continuous phase for the sweeping whine: the frequency ramps but the
    // waveform must not jump.
    let mut whine_phase = 0.0f32;
    // One-pole state for the rumble's broadband layer (a rumble, not hiss).
    let mut lp = 0.0f32;
    for i in 0..n {
        let t = i as f32 / rate;
        let mut s = 0.0;

        // Starter whine, sweeping up as it strains, handing over by 0.7 s.
        let whine = (t / 0.5).min(1.0);
        let f = 130.0 + 110.0 * whine * whine;
        whine_phase += std::f32::consts::TAU * f / rate;
        let whine_amp = 0.30
            * (1.0 - ((t - 0.45) / 0.25).clamp(0.0, 1.0))
            * (0.9 + 0.1 * (std::f32::consts::TAU * 8.0 * t).sin());
        s += whine_amp
            * (whine_phase.sin()
                + 0.5 * (2.0 * whine_phase).sin()
                + 0.15 * (4.0 * whine_phase).sin());

        // Combustion pops: low thumps with a noise edge, uneven like reality.
        for (t0, f0, amp) in [(0.38, 85.0, 0.85), (0.52, 110.0, 0.7), (0.63, 130.0, 0.6)] {
            let dt = t - t0;
            if dt >= 0.0 && dt < 0.12 {
                s += amp * (std::f32::consts::TAU * f0 * dt).sin() * (-dt / 0.035).exp();
                seed = seed.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
                let r = (seed >> 8) as f32 / 8_388_608.0 - 1.0;
                s += amp * 0.4 * r * (-dt / 0.02).exp();
            }
        }

        // Idle rumble: firing pulses rising in from 0.75 s, then steady.
        let idle_in = ((t - 0.75) / 0.35).clamp(0.0, 1.0);
        let idle_in = idle_in * idle_in * (3.0 - 2.0 * idle_in); // smoothstep
        if idle_in > 0.0 {
            // O-360 at ~700 rpm: ~23 Hz firing, strong harmonics + mechanical
            // broadband underneath.
            let p = std::f32::consts::TAU * 23.3 * t;
            s += idle_in
                * 0.55
                * (p.sin() + 0.6 * (2.0 * p).sin() + 0.35 * (3.0 * p).sin() + 0.2 * (5.0 * p).sin());
            seed = seed.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
            let r = (seed >> 8) as f32 / 8_388_608.0 - 1.0;
            lp += 0.15 * (r - lp); // cheap one-pole low-pass
            s += idle_in * 0.18 * lp;
        }

        out.push((s.clamp(-1.0, 1.0) * 0.75 * i16::MAX as f32) as i16);
    }
    out
}
