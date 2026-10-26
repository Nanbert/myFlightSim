//! Pilot G-strain: grey-out, tunnel vision and the sustained over-G caution.
//!
//! The F-22's HUD already prints the load factor; this module supplies the
//! physiological half that a HUD number cannot: above ~6 G the visual field
//! shrinks and drains to grey (`assets/shaders/g_strain.wesl`), and a sustained
//! pull past 9 G raises a brief OVER-G caution. Real fighters do not sound a
//! horn for this — the pilot's vision and the flight-control limits are the
//! warning — so the cue is visual, not a buzzer.
//!
//! The strain is smoothed with a fast attack and a slower recovery, so a
//! momentary spike does not flicker the overlay and the grey takes a moment to
//! clear after the pull is released.
//!
//! Over-G also has a voice. A real pilot does not pant — panting lowers G
//! tolerance — they fly the AGSM: a tense, held body and a forced "hook"
//! exhale every few seconds. This module plays exactly that, gated on the same
//! smoothed strain as the grey-out: a strained exhale above ~7 G (interval
//! shrinking as the pull deepens) and one deep recovery inhale when the pull is
//! released. The clips are pre-rendered one-shots (no loop), so repetitions
//! never read as a tape.

use bevy::audio::{AudioPlayer, AudioSource, PlaybackSettings, Volume};
use bevy::prelude::*;
use bevy::render::render_resource::{AsBindGroup, ShaderType};
use bevy::shader::ShaderRef;
use bevy::ui_render::prelude::{MaterialNode, UiMaterial, UiMaterialPlugin};

use crate::app_state::AppState;
use crate::sim::AircraftState;

/// Load factor where the visual field starts to close in.
const ONSET_G: f32 = 6.0;
/// Load factor where the tunnel is fully closed.
const FULL_G: f32 = 9.5;
/// Load factor held before the airframe caution appears.
const OVER_G: f32 = 9.0;
/// How long above `OVER_G` before the caution shows (s).
const OVER_G_DELAY: f32 = 0.8;
/// Grey-out onset / recovery time constants (s).
const ATTACK_TAU: f32 = 0.18;
const RECOVER_TAU: f32 = 0.9;

/// Breathing: first exhale once the displayed strain passes ~7 G, and the
/// recovery inhale once it falls back below `INHALE_STRAIN`.
const EXHALE_STRAIN: f32 = 0.25;
const INHALE_STRAIN: f32 = 0.05;
/// Wait this long after the pull starts before the first strained exhale.
const EXHALE_FIRST_S: f32 = 0.2;
/// Exhale cadence at the onset of strain and at full tunnel, seconds. The
/// interval tightens with strain, and a small deterministic jitter keeps the
/// rhythm from sounding metronomic.
const EXHALE_BASE_S: f32 = 3.4;
const EXHALE_TIGHTEN_S: f32 = 0.8;
/// Mix gains (sources are mastered to -20 LUFS; these sit under the airflow).
const EXHALE_VOLUME: f32 = 0.40;
const INHALE_VOLUME: f32 = 0.40;
/// A recovery inhale is a once-per-unload event, not a machine gun.
const INHALE_COOLDOWN_S: f32 = 1.0;

#[derive(Clone, Copy, ShaderType, Default)]
struct GStrainParams {
    /// x = strain 0..1; yzw unused.
    a: Vec4,
}

#[derive(Asset, TypePath, AsBindGroup, Clone, Default)]
struct GStrainMaterial {
    #[uniform(0)]
    params: GStrainParams,
}

impl UiMaterial for GStrainMaterial {
    fn fragment_shader() -> ShaderRef {
        "shaders/g_strain.wesl".into()
    }
}

#[derive(Component)]
struct GStrainOverlay;

#[derive(Component)]
struct OverGCaution;

/// The AGSM breath clips and their cadence state.
#[derive(Resource)]
struct BreathSfx {
    exhale: Handle<AudioSource>,
    inhale: Handle<AudioSource>,
    /// Seconds until the next exhale while the pull is held; 0 means idle.
    exhale_in: f32,
    /// The pull went deep enough that releasing it owes a recovery breath.
    strained: bool,
    inhale_cd: f32,
    /// LCG state for the exhale-interval jitter (no `rand` dependency).
    seed: u32,
    /// One diagnostic print per flight: proves the breath chain armed.
    logged: bool,
}

#[derive(Resource, Default)]
struct Strain {
    /// Smoothed display value, 0..1.
    shown: f32,
    /// Time spent above `OVER_G` (s), with a faster decay below it.
    over_g_s: f32,
    /// Screenshot hook (`G_STRAIN=1`): hold the overlay fully closed.
    forced: bool,
}

pub struct GStrainPlugin;

impl Plugin for GStrainPlugin {
    fn build(&self, app: &mut App) {
        app.insert_resource(Strain {
            forced: std::env::var("G_STRAIN").is_ok_and(|v| v == "1"),
            ..default()
        })
        .add_plugins(UiMaterialPlugin::<GStrainMaterial>::default())
            .add_systems(Startup, (spawn_overlay, setup_breath_sfx))
            .add_systems(Update, update_strain);
    }
}

/// Load the three AGSM breath one-shots once.
fn setup_breath_sfx(mut commands: Commands, assets: Res<AssetServer>) {
    commands.insert_resource(BreathSfx {
        exhale: assets.load("audio/f22/breath_exhale.wav"),
        inhale: assets.load("audio/f22/breath_inhale.wav"),
        exhale_in: 0.0,
        strained: false,
        inhale_cd: 0.0,
        seed: 0x5eed_1e55,
        logged: false,
    });
}

fn spawn_overlay(mut commands: Commands, mut materials: ResMut<Assets<GStrainMaterial>>) {
    let mat = materials.add(GStrainMaterial::default());
    commands
        .spawn((
            GStrainOverlay,
            MaterialNode(mat),
            Node {
                position_type: PositionType::Absolute,
                left: Val::Px(0.0),
                top: Val::Px(0.0),
                width: Val::Percent(100.0),
                height: Val::Percent(100.0),
                display: Display::None,
                ..default()
            },
            GlobalZIndex(40),
        ))
        .with_children(|root| {
            root.spawn((
                OverGCaution,
                Text::new("OVER-G"),
                TextFont {
                    font_size: FontSize::Px(24.0),
                    ..default()
                },
                TextColor(Color::srgb(1.0, 0.22, 0.16)),
                TextLayout::new(Justify::Center, LineBreak::NoWrap),
                Node {
                    position_type: PositionType::Absolute,
                    top: Val::Percent(16.0),
                    left: Val::Px(0.0),
                    width: Val::Percent(100.0),
                    ..default()
                },
                Visibility::Hidden,
            ));
        });
}

fn update_strain(
    mut commands: Commands,
    time: Res<Time>,
    app_state: Res<State<AppState>>,
    state: Res<AircraftState>,
    mut strain: ResMut<Strain>,
    breath: Option<ResMut<BreathSfx>>,
    mut materials: ResMut<Assets<GStrainMaterial>>,
    overlays: Query<(&MaterialNode<GStrainMaterial>, &mut Node), With<GStrainOverlay>>,
    mut caution: Query<&mut Visibility, With<OverGCaution>>,
) {
    let dt = time.delta_secs().clamp(0.0, 0.1);
    let flying = *app_state.get() == AppState::Flying;
    let g = if flying { state.g_norm as f32 } else { 1.0 };
    let target = if strain.forced {
        1.0
    } else if flying {
        strain_target(g)
    } else {
        0.0
    };

    // Fast onset, slow recovery: a hard pull greys the view at once, while the
    // field takes a beat to return after the aircraft unloads.
    let tau = if target > strain.shown {
        ATTACK_TAU
    } else {
        RECOVER_TAU
    };
    strain.shown += (target - strain.shown) * (1.0 - (-dt / tau).exp());
    if strain.shown < 0.004 {
        strain.shown = 0.0;
    }

    if flying && g > OVER_G {
        strain.over_g_s += dt;
    } else {
        strain.over_g_s = (strain.over_g_s - dt * 2.0).max(0.0);
    }
    let warn = flying && strain.over_g_s >= OVER_G_DELAY;

    for (node_mat, mut node) in overlays {
        if let Some(mut mat) = materials.get_mut(&node_mat.0) {
            mat.params = GStrainParams {
                a: Vec4::new(strain.shown, 0.0, 0.0, 0.0),
            };
        }
        let want = if strain.shown > 0.0 {
            Display::Flex
        } else {
            Display::None
        };
        if node.display != want {
            node.display = want;
        }
    }
    for mut vis in &mut caution {
        let want = if warn {
            Visibility::Inherited
        } else {
            Visibility::Hidden
        };
        if *vis != want {
            *vis = want;
        }
    }

    if let Some(mut breath) = breath {
        breathe(
            &mut commands,
            &mut breath,
            dt,
            flying,
            strain.shown,
            target,
        );
    }
}

/// Drive the AGSM breath cadence from the same smoothed strain the grey-out
/// uses. Both sounds are one-shots: a loop would read as a tape within
/// seconds, and a breath is naturally intermittent anyway.
fn breathe(
    commands: &mut Commands,
    sfx: &mut BreathSfx,
    dt: f32,
    flying: bool,
    strain: f32,
    // The unsmoothed strain target: the inhale must land on the *release*,
    // not two seconds later after the grey has slowly cleared.
    target: f32,
) {
    if !flying {
        sfx.exhale_in = 0.0;
        sfx.strained = false;
        sfx.inhale_cd = 0.0;
        sfx.logged = false;
        return;
    }
    sfx.inhale_cd = (sfx.inhale_cd - dt).max(0.0);

    // Hook breaths: one shortly after the pull starts, then every few seconds,
    // tighter as the tunnel closes in.
    if strain > EXHALE_STRAIN {
        if sfx.exhale_in <= 0.0 {
            sfx.exhale_in = EXHALE_FIRST_S;
        }
        sfx.exhale_in -= dt;
        if sfx.exhale_in <= 0.0 {
            play_breath(commands, &sfx.exhale, EXHALE_VOLUME);
            if !sfx.logged {
                sfx.logged = true;
                info!("g audio: first hook breath at strain={strain:.2}");
            }
            let base = EXHALE_BASE_S - EXHALE_TIGHTEN_S * strain;
            sfx.exhale_in = base + 0.6 * jitter(&mut sfx.seed);
        }
    } else {
        sfx.exhale_in = 0.0;
    }

    // Recovery inhale: only after a real pull, and only once the grey has
    // mostly cleared, so it lands on the release rather than on the pull.
    if target > 0.3 {
        sfx.strained = true;
    }
    if sfx.strained && target < INHALE_STRAIN && sfx.inhale_cd <= 0.0 {
        play_breath(commands, &sfx.inhale, INHALE_VOLUME);
        sfx.strained = false;
        sfx.inhale_cd = INHALE_COOLDOWN_S;
    }
}

/// Spawn one breath one-shot; it despawns itself, and leaving the flight takes
/// any tail with it.
fn play_breath(commands: &mut Commands, handle: &Handle<AudioSource>, volume: f32) {
    commands.spawn((
        AudioPlayer::new(handle.clone()),
        PlaybackSettings::DESPAWN.with_volume(Volume::Linear(volume)),
        DespawnOnExit(AppState::Flying),
    ));
}

/// Deterministic -1..1 jitter for the exhale interval.
fn jitter(seed: &mut u32) -> f32 {
    *seed = seed.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
    (*seed >> 8) as f32 / 16_777_216.0 - 1.0
}

/// Smooth onset: nothing below `ONSET_G`, full tunnel at `FULL_G`.
fn strain_target(g: f32) -> f32 {
    let x = ((g - ONSET_G) / (FULL_G - ONSET_G)).clamp(0.0, 1.0);
    x * x * (3.0 - 2.0 * x)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn exhale_tightens_with_strain() {
        let at_onset = EXHALE_BASE_S - EXHALE_TIGHTEN_S * EXHALE_STRAIN;
        let at_tunnel = EXHALE_BASE_S - EXHALE_TIGHTEN_S;
        assert!(at_onset > 2.5, "onset cadence {at_onset} too fast");
        assert!(at_tunnel > 2.0, "full-tunnel cadence {at_tunnel} too fast");
        assert!(at_tunnel < at_onset, "cadence must tighten with strain");
    }

    #[test]
    fn exhale_jitter_stays_within_its_band() {
        let mut seed = 1u32;
        for _ in 0..200 {
            let j = jitter(&mut seed);
            assert!((-1.0..=1.0).contains(&j), "jitter {j} escaped");
        }
    }

    #[test]
    fn strain_starts_at_six_g_and_saturates() {
        assert_eq!(strain_target(0.0), 0.0);
        assert_eq!(strain_target(ONSET_G), 0.0);
        assert_eq!(strain_target(FULL_G), 1.0);
        assert_eq!(strain_target(15.0), 1.0);
        assert!(strain_target(7.0) > 0.0 && strain_target(7.0) < 0.5);
        assert!(strain_target(9.0) > strain_target(8.0));
    }
}
