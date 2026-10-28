//! Lesson ATC voice: the C172's graded lessons (M1 TAXI, M2 TAKEOFF,
//! M3 LANDING) each open with a tower-controller callout, and the 737's
//! M1 TAXI, M2 TAKEOFF, and M3 LANDING add a pilot readback — mechanical,
//! deadpan, no inflection (Qwen3-TTS, Uncle_Fu for the ATC voice at 1.3x,
//! Aiden for the 737 PNF at 1.0x, radio-processed: 200 Hz–2.6 kHz
//! band-pass plus a low airport-ambience bed so it reads as a distant radio,
//! not a mic next to the ear).
//!
//! Two playback paths:
//! - **Opening callouts** (M1/M2/M3 for both airframes) fire once on
//!   entering `Flying`, gated by `ActiveLesson` — the same state-gated idiom
//!   as the engine audio.
//! - **Mid-flight callouts** (737 M4 GO-AROUND at 1000 ft, M5 RTO at 100 kt,
//!   M6 AUTOMATION stop-climb at 3500 ft) fire when the lesson's own `tick`
//!   decides the scripted call is due — the lesson sends an
//!   [`AtcVoiceEvent`] and this module plays the matching clip.
//!
//! Warm spawns, free flight and every other airframe/lesson stay silent.
//! The clips ship as pre-mixed assets (voice + ambience already balanced,
//! peak-normalised) — there is no per-frame mixing to speak of here.

use bevy::audio::{AudioPlayer, PlaybackSettings, Volume};
use bevy::prelude::*;

use crate::app_state::AppState;
use crate::mission::{ActiveLesson, Lesson};
use crate::sim::StartFlight;

/// Which mid-flight callout to play. The opening callouts (M1/M2/M3) are
/// routed by [`ActiveLesson`] in [`play_for_active_lesson`]; these kinds are
/// routed by [`AtcVoiceEvent`] in [`play_atc_voice_event`].
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum AtcVoiceKind {
    /// 737 M4 GO-AROUND: the scripted call at 1000 ft.
    GoAround,
    /// 737 M5 RTO: the scripted STOP call at 100 kt.
    Stop,
    /// 737 M6 AUTOMATION: the ATC stop-climb at 3500 ft.
    AltHold,
}

/// Fired by the jet lessons when their scripted call is due. The reader
/// system [`play_atc_voice_event` `spawns` a player with the matching clip.
#[derive(Message)]
pub struct AtcVoiceEvent(pub AtcVoiceKind);

/// The C172 M1 TAXI opening callout (7 s, voice + radio ambience):
/// "Cessna Seventeen two, taxi two four left, Alpha Tango."
const M1_TAXI_CLIP: &str = "audio/atc/c172_m1_taxi_Uncle_Fu_radio.wav";
/// The C172 M2 TAKEOFF opening callout (8 s, voice + radio ambience):
/// "Cessna Seventeen two, cleared for takeoff runway two four left."
const M2_TAKEOFF_CLIP: &str = "audio/atc/c172_m2_takeoff_Uncle_Fu_radio.wav";
/// The C172 M3 LANDING opening callout (8 s, voice + radio ambience):
/// "Cessna Seventeen two, cleared to land runway three six."
const M3_LANDING_CLIP: &str = "audio/atc/c172_m3_landing_Uncle_Fu_radio.wav";
/// The 737 M1 TAXI opening callout (17 s, ATC 1.3x + 1 s silence + pilot
/// readback + radio ambience): "Flight 172, taxi to runway three six via
/// Alpha Bravo Charlie, report ready." — read back by the PNF. Commercial
/// ops read back every clearance; the C172's shorter GA-style calls do not.
const M1_JET_TAXI_CLIP: &str =
    "audio/atc/b737_m1_jet_taxi_Uncle_Fu_and_Aiden.wav";
/// The 737 M2 TAKEOFF opening callout (18 s, ATC 1.3x + 1 s silence + pilot
/// readback + radio ambience): "Flight 172, cleared for takeoff runway
/// three six." — read back by the PNF.
const M2_JET_TAKEOFF_CLIP: &str =
    "audio/atc/b737_m2_jet_takeoff_Uncle_Fu_and_Aiden.wav";
/// The 737 M3 LANDING opening callout (12 s, ATC 1.3x + 1 s silence + pilot
/// readback + radio ambience): "Flight 172, cleared to land runway
/// three six." — read back by the PNF.
const M3_JET_LANDING_CLIP: &str =
    "audio/atc/b737_m3_jet_landing_Uncle_Fu_and_Aiden.wav";
/// The 737 M4 GO-AROUND mid-flight callout (8.5 s; ATC 1.3x, 0.7 s silence,
/// pilot readback, radio ambience): "Flight 172, go around, go around."
/// Urgent crew call — the shorter silence reflects the split-second reaction
/// budget. Fires at 1000 ft in [`crate::jet_go_around`].
const M4_GO_AROUND_CLIP: &str =
    "audio/atc/b737_m4_go_around_Uncle_Fu_and_Aiden.wav";
/// The 737 M5 RTO mid-flight callout (16.5 s, ATC 1.3x + 0.7 s silence +
/// pilot readback + radio ambience): "Flight 172, stop, stop." Urgent
/// crew call — fires at 100 kt in [`crate::jet_rto`].
const M5_RTO_CLIP: &str = "audio/atc/b737_m5_rto_Uncle_Fu_and_Aiden.wav";
/// The 737 M6 AUTOMATION mid-flight callout (14.5 s; ATC 1.3x, 1 s silence,
/// pilot readback, radio ambience): "Flight 172, ALT HOLD." The calm
/// ATC stop-climb — fires at 3500 ft in [`crate::jet_automation`].
const M6_AUTOMATION_CLIP: &str =
    "audio/atc/b737_m6_automation_Uncle_Fu_and_Aiden.wav";
/// Tower-radio loudness: enough to read clearly over the idle prop loop,
/// quiet enough to sit under cockpit talk. Adjust here if the mix feels off.
const VOICE_VOLUME: f32 = 0.65;

/// The pre-loaded lesson-voice clips. `None` for rows with no voice yet — the
/// player simply never spawns, so an empty slot costs nothing.
#[derive(Resource, Default)]
pub struct LessonVoices {
    m1_taxi: Option<Handle<AudioSource>>,
    m2_takeoff: Option<Handle<AudioSource>>,
    m3_landing: Option<Handle<AudioSource>>,
    m1_jet_taxi: Option<Handle<AudioSource>>,
    m2_jet_takeoff: Option<Handle<AudioSource>>,
    m3_jet_landing: Option<Handle<AudioSource>>,
    m4_go_around: Option<Handle<AudioSource>>,
    m5_rto: Option<Handle<AudioSource>>,
    m6_automation: Option<Handle<AudioSource>>,
}

/// The clip for a mid-flight callout kind. Returns `None` if the clip hasn't
/// been loaded yet — the event is silently dropped (the lesson can keep
/// working without the audio).
fn clip_for_kind(kind: AtcVoiceKind, voices: &LessonVoices) -> Option<&Handle<AudioSource>> {
    Some(match kind {
        AtcVoiceKind::GoAround => voices.m4_go_around.as_ref()?,
        AtcVoiceKind::Stop => voices.m5_rto.as_ref()?,
        AtcVoiceKind::AltHold => voices.m6_automation.as_ref()?,
    })
}

pub struct AtcVoicePlugin;

impl Plugin for AtcVoicePlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<LessonVoices>()
            .add_message::<AtcVoiceEvent>()
            .add_systems(Startup, setup_voices)
            // `.after(StartFlight)` — the same idiom `map::enter_mission` and
            // `aerodrome::spawn_aerodrome` use to order themselves behind
            // `start_flight`, which is the system that copies `PendingLesson`
            // into `ActiveLesson`. Without the ordering, this system can
            // observe the default `None` lesson and never spawn the player.
            .add_systems(OnEnter(AppState::Flying), play_for_active_lesson.after(StartFlight))
            // Mid-flight callouts: the lesson's `tick` writes an event when
            // its scripted call is due; this system drains the queue every
            // frame and spawns a player. Runs alongside the lesson's own
            // systems (no ordering dependency — the event is already
            // committed by the time it's read).
            .add_systems(Update, play_atc_voice_event);
    }
}

/// Load every lesson clip once; the handles stay valid for the whole app run.
fn setup_voices(
    mut voices: ResMut<LessonVoices>,
    assets: Res<AssetServer>,
) {
    voices.m1_taxi = Some(assets.load(M1_TAXI_CLIP));
    voices.m2_takeoff = Some(assets.load(M2_TAKEOFF_CLIP));
    voices.m3_landing = Some(assets.load(M3_LANDING_CLIP));
    voices.m1_jet_taxi = Some(assets.load(M1_JET_TAXI_CLIP));
    voices.m2_jet_takeoff = Some(assets.load(M2_JET_TAKEOFF_CLIP));
    voices.m3_jet_landing = Some(assets.load(M3_JET_LANDING_CLIP));
    voices.m4_go_around = Some(assets.load(M4_GO_AROUND_CLIP));
    voices.m5_rto = Some(assets.load(M5_RTO_CLIP));
    voices.m6_automation = Some(assets.load(M6_AUTOMATION_CLIP));
}

/// On entering a flight, play the clip for the lesson just started. Runs
/// after the `StartFlight` set (which contains `start_flight`), so `ActiveLesson`
/// is already finalised — a map mission or a free flight leaves it `None` and
/// nothing plays. The audio player carries no despawn trigger beyond
/// [`PlaybackSettings::DESPAWN`]: the callout is meant to be heard over
/// silence and over the idle prop loop, and it stops on its own when done.
fn play_for_active_lesson(
    mut commands: Commands,
    active: Res<ActiveLesson>,
    voices: Res<LessonVoices>,
) {
    let handle = match active.0 {
        Some(Lesson::Taxi) => &voices.m1_taxi,
        Some(Lesson::Takeoff) => &voices.m2_takeoff,
        Some(Lesson::Landing) => &voices.m3_landing,
        Some(Lesson::JetTaxi) => &voices.m1_jet_taxi,
        Some(Lesson::JetTakeoff) => &voices.m2_jet_takeoff,
        Some(Lesson::JetLanding) => &voices.m3_jet_landing,
        // Free flight and the 737's mid-flight lessons (go-around, RTO,
        // automation) don't play on enter — they fire on their own scripted
        // call, routed through [`AtcVoiceEvent`].
        _ => return,
    };
    if let Some(handle) = handle {
        commands.spawn((
            AudioPlayer::new(handle.clone()),
            PlaybackSettings::DESPAWN.with_volume(Volume::Linear(VOICE_VOLUME)),
            // A callout is part of the flight, not the menu: leaving mid-call
            // (Esc → Enter is one keystroke away, and the M1 clearance runs 17 s)
            // must not leave the controller talking over the hangar.
            DespawnOnExit(AppState::Flying),
        ));
    }
}

/// Drain the mid-flight callout queue. Each event spawns a one-shot player
/// with the matching clip; the player despawns itself when done.
fn play_atc_voice_event(
    mut events: MessageReader<AtcVoiceEvent>,
    mut commands: Commands,
    voices: Res<LessonVoices>,
) {
    for event in events.read() {
        if let Some(handle) = clip_for_kind(event.0, &voices) {
            commands.spawn((
                AudioPlayer::new(handle.clone()),
                PlaybackSettings::DESPAWN.with_volume(Volume::Linear(VOICE_VOLUME)),
                DespawnOnExit(AppState::Flying),
            ));
        }
    }
}
