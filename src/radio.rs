//! F-22 combat radio chatter: the AWACS controller, the wingman and the
//! player's own brevity calls.
//!
//! Combat is already loud (gun, missiles, RWR tone); radio is the one channel
//! that carries *information* over that noise, so it is deliberately sparse:
//! nine calls plus one condition watch (a bandit sliding onto the six),
//! one voice on the air at a time, and a hard cooldown per call. The clips are baked one-shots (`assets/audio/radio/`,
//! 48 kHz, −6 dBFS peak, band-passed 200–2600 Hz over a carrier bed) — the
//! same radio chain as the ATC lesson voices, so the tower and Darkstar sound
//! like two stations on one net.
//!
//! The combat systems only say *when*; this module owns *what* plays, the
//! queue and the bookkeeping — the same split as [`crate::atc_voice`]. Cues
//! arrive as [`RadioEvent`] messages:
//!
//! - **Emergency** (inbound missile call) steps on whatever is on the air;
//! - **Scripted** (check-in, picture clean) always goes through;
//! - **Event** (Fox / Splash / Winchester) is rate-limited;
//! - **Flavor** (Tally, "good kill") is the first to be dropped when busy.
//!
//! Flare releases are deliberately off the net: the player pops them on a
//! reflex, often several times a fight, and a brevity call every time buried
//! the calls that matter (inbound, splash). The dispenser and the cartridge
//! whoosh carry that moment; the counter sits on the HUD.
//!
//! A call is "on the air" while its player entity lives — `DESPAWN` removes it
//! when the clip ends — plus an 8 s fallback so a missing asset can never
//! wedge the queue.

use bevy::audio::{AudioPlayer, AudioSource, PlaybackSettings, Volume};
use bevy::prelude::*;

use crate::app_state::{flying, unpaused, AppState};

/// Radio sits under the airflow and the missile motor (ATC uses 0.65).
const RADIO_VOLUME: f32 = 0.60;
/// Silence between two calls, so back-to-back brevity does not clip.
const RADIO_GAP_S: f32 = 0.35;
/// Two calls can wait for the air; a third is noise, not radio.
const MAX_PENDING: usize = 2;
/// A clip that outlives this must not hold the slot (e.g. a missing asset).
const BUSY_TIMEOUT_S: f32 = 8.0;

/// One scripted call. The order matches [`LINES`]; [`RadioCue::index`] is the
/// bridge between the enum and the per-cue bookkeeping arrays.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum RadioCue {
    /// First wave spawns: AWACS calls the picture, the flight lead answers.
    Kickoff,
    /// First lock of the flight: the wingman has visual.
    Tally,
    /// IR launch.
    FoxTwo,
    /// AMRAAM launch.
    FoxThree,
    /// Bandit destroyed.
    Splash,
    /// Every second kill: the wingman acknowledges.
    GoodKill,
    /// An enemy round is in the air — the one call that may cut in.
    Inbound,
    /// Missiles gone.
    Winchester,
    /// Last bandit down: AWACS sends the flight home.
    PictureClean,
    /// A drone has slid onto the six — a condition call, not an event.
    BanditSix,
}

impl RadioCue {
    /// Every cue, in table order. `index()` must agree with this.
    pub const ALL: [RadioCue; 10] = [
        RadioCue::Kickoff,
        RadioCue::Tally,
        RadioCue::FoxTwo,
        RadioCue::FoxThree,
        RadioCue::Splash,
        RadioCue::GoodKill,
        RadioCue::Inbound,
        RadioCue::Winchester,
        RadioCue::PictureClean,
        RadioCue::BanditSix,
    ];

    const fn index(self) -> usize {
        match self {
            RadioCue::Kickoff => 0,
            RadioCue::Tally => 1,
            RadioCue::FoxTwo => 2,
            RadioCue::FoxThree => 3,
            RadioCue::Splash => 4,
            RadioCue::GoodKill => 5,
            RadioCue::Inbound => 6,
            RadioCue::Winchester => 7,
            RadioCue::PictureClean => 8,
            RadioCue::BanditSix => 9,
        }
    }
}

/// Fired by the combat systems when a call is due. The reader in
/// [`RadioPlugin`] queues it and plays the matching clip.
#[derive(Message)]
pub struct RadioEvent(pub RadioCue);

/// Who gets to talk over whom. Lower is more urgent.
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Debug)]
enum Priority {
    Emergency,
    Scripted,
    Event,
    Flavor,
}

/// How often a cue may play in one flight.
#[derive(Clone, Copy, Debug)]
enum Limit {
    /// Once per flight (radio bookkeeping, not audio timing).
    Once,
    /// At most once per this many seconds.
    Every(f32),
}

struct Line {
    cue: RadioCue,
    clip: &'static str,
    priority: Priority,
    limit: Limit,
}

/// The nine calls, in [`RadioCue`] order. Cooldowns are longer than the clips
/// so a burst of hits cannot machine-gun the net.
const LINES: [Line; 10] = [
    Line {
        cue: RadioCue::Kickoff,
        clip: "audio/radio/r01_kickoff.wav",
        priority: Priority::Scripted,
        limit: Limit::Once,
    },
    Line {
        cue: RadioCue::Tally,
        clip: "audio/radio/r02_tally.wav",
        priority: Priority::Flavor,
        limit: Limit::Once,
    },
    Line {
        cue: RadioCue::FoxTwo,
        clip: "audio/radio/r03_fox2.wav",
        priority: Priority::Event,
        limit: Limit::Every(1.5),
    },
    Line {
        cue: RadioCue::FoxThree,
        clip: "audio/radio/r04_fox3.wav",
        priority: Priority::Event,
        limit: Limit::Every(1.5),
    },
    Line {
        cue: RadioCue::Splash,
        clip: "audio/radio/r05_splash.wav",
        priority: Priority::Event,
        limit: Limit::Every(1.2),
    },
    Line {
        cue: RadioCue::GoodKill,
        clip: "audio/radio/r06_goodkill.wav",
        priority: Priority::Flavor,
        limit: Limit::Every(15.0),
    },
    Line {
        cue: RadioCue::Inbound,
        clip: "audio/radio/r07_inbound.wav",
        priority: Priority::Emergency,
        limit: Limit::Every(9.0),
    },
    Line {
        cue: RadioCue::Winchester,
        clip: "audio/radio/r08_winchester.wav",
        priority: Priority::Event,
        limit: Limit::Once,
    },
    Line {
        cue: RadioCue::PictureClean,
        clip: "audio/radio/r09_pictureclean.wav",
        priority: Priority::Scripted,
        limit: Limit::Once,
    },
    Line {
        cue: RadioCue::BanditSix,
        clip: "audio/radio/r10_bandit_six.wav",
        priority: Priority::Event,
        limit: Limit::Every(12.0),
    },
];

fn line(cue: RadioCue) -> &'static Line {
    debug_assert_eq!(LINES[cue.index()].cue, cue, "line table out of order");
    &LINES[cue.index()]
}

/// The loaded clips. `None` slots simply never play, so the module keeps
/// working if a bake is missing.
#[derive(Resource)]
struct RadioClips([Option<Handle<AudioSource>>; 10]);

impl Default for RadioClips {
    fn default() -> Self {
        Self(std::array::from_fn(|_| None))
    }
}

/// Playback bookkeeping. One slot for "on the air", the rest is policy.
#[derive(Resource)]
struct RadioQueue {
    /// Waiting calls, most urgent first when drained.
    pending: Vec<RadioCue>,
    /// The player entity currently speaking.
    active: Option<Entity>,
    active_cue: Option<RadioCue>,
    /// When the current call started, for the fallback timeout.
    active_since: f32,
    /// Per-cue cooldown, seconds. `0` means ready.
    cooldowns: [f32; 10],
    /// `Once` cues already spent this flight.
    played: [bool; 10],
    /// Mandatory silence after the last call.
    gap: f32,
    /// One diagnostic line per flight, like `g_strain`'s first breath.
    logged: bool,
}

impl Default for RadioQueue {
    fn default() -> Self {
        Self {
            pending: Vec::with_capacity(MAX_PENDING + 1),
            active: None,
            active_cue: None,
            active_since: 0.0,
            cooldowns: [0.0; 10],
            played: [false; 10],
            gap: 0.0,
            logged: false,
        }
    }
}

pub struct RadioPlugin;

impl Plugin for RadioPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<RadioClips>()
            .init_resource::<RadioQueue>()
            .add_message::<RadioEvent>()
            .add_systems(Startup, setup_radio)
            .add_systems(OnEnter(AppState::Flying), reset_radio)
            .add_systems(
                Update,
                (enqueue, pump)
                    .chain()
                    .run_if(flying)
                    .run_if(unpaused),
            );
    }
}

/// Load every call once; the handles stay valid for the whole app run.
fn setup_radio(mut clips: ResMut<RadioClips>, assets: Res<AssetServer>) {
    for (slot, line) in clips.0.iter_mut().zip(LINES.iter()) {
        *slot = Some(assets.load(line.clip));
    }
}

/// A new flight (or a retry) hears the net from scratch: the check-in plays
/// again, cooldowns are clear, nothing is left on the air.
fn reset_radio(mut queue: ResMut<RadioQueue>) {
    *queue = RadioQueue::default();
}

/// Queue incoming calls, dropping repeats, spent one-shots and the least
/// important call when the queue is full.
fn enqueue(
    mut events: MessageReader<RadioEvent>,
    mut queue: ResMut<RadioQueue>,
    mut commands: Commands,
    time: Res<Time>,
) {
    for RadioEvent(cue) in events.read() {
        let cue = *cue;
        if !accepts(&queue, cue) {
            continue;
        }
        let l = line(cue);

        // An emergency call steps on a lower-priority transmission instead of
        // queueing behind it — a missile warning is not "please hold".
        if l.priority == Priority::Emergency {
            if let (Some(entity), Some(active)) = (queue.active, queue.active_cue) {
                if line(active).priority > Priority::Emergency {
                    commands.entity(entity).despawn();
                    queue.active = None;
                    queue.active_cue = None;
                    queue.active_since = time.elapsed_secs();
                    queue.gap = 0.0;
                }
            }
        }

        push_pending(&mut queue, cue);
    }
}

/// Whether a cue may enter the queue at all: not cooling down, not a spent
/// one-shot, and not already waiting.
fn accepts(queue: &RadioQueue, cue: RadioCue) -> bool {
    let l = line(cue);
    let i = cue.index();
    if queue.cooldowns[i] > 0.0 {
        return false;
    }
    if matches!(l.limit, Limit::Once) && queue.played[i] {
        return false;
    }
    !queue.pending.contains(&cue)
}

/// Insert a cue, keeping at most [`MAX_PENDING`]; the least important one
/// (ties: the newest) is the one dropped.
fn push_pending(queue: &mut RadioQueue, cue: RadioCue) {
    queue.pending.push(cue);
    while queue.pending.len() > MAX_PENDING {
        let mut worst = 0;
        for i in 1..queue.pending.len() {
            if line(queue.pending[i]).priority >= line(queue.pending[worst]).priority {
                worst = i;
            }
        }
        queue.pending.remove(worst);
    }
}

/// Highest-priority pending call, oldest first within a priority.
fn take_next(queue: &mut RadioQueue) -> Option<RadioCue> {
    if queue.pending.is_empty() {
        return None;
    }
    let mut best = 0;
    for i in 1..queue.pending.len() {
        if line(queue.pending[i]).priority < line(queue.pending[best]).priority {
            best = i;
        }
    }
    Some(queue.pending.remove(best))
}

/// Advance the cooldowns and put the next call on the air when the slot frees
/// up. A call is on the air while its entity lives (`DESPAWN` removes it at
/// the end of the clip), with a timeout so a lost asset cannot wedge the net.
fn pump(
    mut commands: Commands,
    mut queue: ResMut<RadioQueue>,
    clips: Res<RadioClips>,
    entities: Query<()>,
    time: Res<Time>,
) {
    let dt = time.delta_secs();
    for cd in &mut queue.cooldowns {
        *cd = (*cd - dt).max(0.0);
    }
    queue.gap = (queue.gap - dt).max(0.0);

    if let Some(entity) = queue.active {
        let alive = entities.get(entity).is_ok();
        let timed_out = time.elapsed_secs() - queue.active_since > BUSY_TIMEOUT_S;
        if !alive || timed_out {
            if timed_out && alive {
                commands.entity(entity).despawn();
            }
            queue.active = None;
            queue.active_cue = None;
            queue.gap = RADIO_GAP_S;
        }
    }

    if queue.active.is_some() || queue.gap > 0.0 {
        return;
    }
    let Some(cue) = take_next(&mut queue) else {
        return;
    };
    let Some(handle) = clips.0[cue.index()].clone() else {
        return;
    };

    let entity = commands
        .spawn((
            AudioPlayer::new(handle),
            PlaybackSettings::DESPAWN.with_volume(Volume::Linear(RADIO_VOLUME)),
            DespawnOnExit(AppState::Flying),
        ))
        .id();
    queue.active = Some(entity);
    queue.active_cue = Some(cue);
    queue.active_since = time.elapsed_secs();
    match line(cue).limit {
        Limit::Every(seconds) => queue.cooldowns[cue.index()] = seconds,
        Limit::Once => queue.played[cue.index()] = true,
    }
    if !queue.logged {
        queue.logged = true;
        info!("radio: first call on the air ({cue:?})");
    }
    debug!("radio: on the air: {cue:?}");
}

#[cfg(test)]
mod tests {
    use super::*;

    fn queue() -> RadioQueue {
        RadioQueue::default()
    }

    #[test]
    fn table_order_matches_the_enum() {
        for (i, cue) in RadioCue::ALL.iter().enumerate() {
            assert_eq!(cue.index(), i, "{cue:?} sits at the wrong index");
            assert_eq!(LINES[i].cue, *cue);
        }
    }

    #[test]
    fn once_cues_stop_after_they_play() {
        let mut q = queue();
        assert!(accepts(&q, RadioCue::Kickoff));
        q.played[RadioCue::Kickoff.index()] = true;
        assert!(!accepts(&q, RadioCue::Kickoff));
    }

    #[test]
    fn repeat_while_cooling_down_is_dropped() {
        let mut q = queue();
        assert!(accepts(&q, RadioCue::FoxTwo));
        q.cooldowns[RadioCue::FoxTwo.index()] = 1.5;
        assert!(!accepts(&q, RadioCue::FoxTwo));
        q.cooldowns[RadioCue::FoxTwo.index()] = 0.0;
        assert!(accepts(&q, RadioCue::FoxTwo));
    }

    #[test]
    fn a_queued_cue_is_not_queued_twice() {
        let mut q = queue();
        push_pending(&mut q, RadioCue::Splash);
        assert!(!accepts(&q, RadioCue::Splash));
    }

    #[test]
    fn cooldown_makes_a_cue_ready_again() {
        let mut q = queue();
        let i = RadioCue::FoxTwo.index();
        q.cooldowns[i] = 1.5;
        // 2 s of 100 ms ticks: more than the 1.5 s cooldown.
        for _ in 0..20 {
            for cd in &mut q.cooldowns {
                *cd = (*cd - 0.1).max(0.0);
            }
        }
        assert!(q.cooldowns[i] < 1e-6, "1.5 s of cooldown must clear");
    }

    #[test]
    fn pending_never_exceeds_the_cap() {
        let mut q = queue();
        push_pending(&mut q, RadioCue::Tally);
        push_pending(&mut q, RadioCue::GoodKill);
        push_pending(&mut q, RadioCue::Splash);
        assert_eq!(q.pending.len(), MAX_PENDING);
        // Splash is the only Event in the mix, so the flavor calls are the ones
        // that make way for it.
        assert!(q.pending.contains(&RadioCue::Splash));
        assert!(!q.pending.contains(&RadioCue::GoodKill));
    }

    #[test]
    fn most_urgent_call_leaves_first() {
        let mut q = queue();
        push_pending(&mut q, RadioCue::Tally);
        push_pending(&mut q, RadioCue::Inbound);
        assert_eq!(take_next(&mut q), Some(RadioCue::Inbound));
        assert_eq!(take_next(&mut q), Some(RadioCue::Tally));
        assert_eq!(take_next(&mut q), None);
    }

    #[test]
    fn emergency_outranks_everything_else() {
        for cue in RadioCue::ALL {
            if cue == RadioCue::Inbound {
                assert_eq!(line(cue).priority, Priority::Emergency);
            } else {
                assert!(line(cue).priority > Priority::Emergency);
            }
        }
    }
}
