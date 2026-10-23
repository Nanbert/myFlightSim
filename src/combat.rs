//! Arcade air combat for the F-22: bandit drones, a simple IR lock, a guided
//! missile and the gun.
//!
//! The flight model stays authoritative — combat only reads the airframe pose
//! and speed, and owns its own entities (drones, missiles, tracers, effects)
//! and HUD. It runs in an arcade mission (or with the `COMBAT=1` dev hook) and
//! is deliberately small: the point of this slice is the lock → shoot → kill
//! loop, not a radar simulation. The F-22's real side bays and two Sidewinder
//! meshes are driven from `f22.glb` (see [`drive_bays`]); the model has no main
//! bay, so the AIM-120-class rounds leave from belly centreline stations.

use std::sync::Arc;

use bevy::audio::{AudioPlayer, AudioSource, PlaybackSettings, Volume};
use bevy::ecs::query::QueryFilter;
use bevy::prelude::*;
use bevy::text::FontSize;

use crate::aircraft_profile::ToggleAction;
use crate::app_state::{unpaused, AppState};
use crate::input::Bindings;
use crate::map::ActiveMap;
use crate::sim::{AircraftState, Sim};
use crate::view::{model_to_body, AircraftRoot};

/// The IR lock envelope: the target must sit inside this cone and range.
const LOCK_RANGE_M: f32 = 6500.0;
const LOCK_CONE_DEG: f32 = 25.0;
/// A lock is kept a little further/lazier than it can be acquired, so a small
/// manoeuvre does not drop it the instant the crosshair slides off.
const KEEP_RANGE_M: f32 = 8000.0;
const KEEP_CONE_DEG: f32 = 35.0;
/// The AIM-120's much wider BVR envelope.
const BVR_LOCK_RANGE_M: f32 = 18_000.0;
const BVR_LOCK_CONE_DEG: f32 = 35.0;
const BVR_KEEP_RANGE_M: f32 = 22_000.0;
const BVR_KEEP_CONE_DEG: f32 = 45.0;

/// Missile kinematics, tuned for a dogfight rather than a spec sheet: fast
/// enough to catch a turning drone, slow enough that a hard break can defeat it.
const MISSILE_ACCEL: f32 = 210.0;
const MISSILE_SPEED_MAX: f32 = 900.0;
const MISSILE_MOTOR_S: f32 = 4.0;
const MISSILE_TURN_RAD_S: f32 = 1.5;
const MISSILE_LIFE_S: f32 = 14.0;
const MISSILE_HIT_M: f32 = 12.0;
/// AIM-120: faster and longer-legged than the Sidewinder, with a wider lock
/// envelope; a little less agile, so a hard turn can still beat it up close.
const AMRAAM_ACCEL: f32 = 300.0;
const AMRAAM_SPEED_MAX: f32 = 1150.0;
const AMRAAM_MOTOR_S: f32 = 6.0;
const AMRAAM_TURN_RAD_S: f32 = 0.9;
const AMRAAM_LIFE_S: f32 = 30.0;
const AMRAAM_ROUNDS: i32 = 2;

const GUN_INTERVAL_S: f32 = 0.075;
const GUN_SPEED: f32 = 900.0;
const GUN_LIFE_S: f32 = 1.4;
const GUN_HIT_M: f32 = 8.0;
const GUN_DAMAGE: i32 = 1;
const GUN_ROUNDS: i32 = 480;
/// The M61A2 sits in the right wing root: the muzzle is right of the
/// centreline, above the inlet, a little ahead of the CG (body frame, metres).
const GUN_PORT_RIGHT: f32 = 1.55;
const GUN_PORT_UP: f32 = 0.55;
const GUN_PORT_AHEAD: f32 = 3.6;
/// Every Nth round also puffs a little gun gas at the muzzle.
const GUN_SMOKE_EVERY: i32 = 6;

/// Smoke puffs per launch and how long they linger, in seconds.
const SMOKE_PUFFS: usize = 7;
const SMOKE_LIFE_S: f32 = 1.6;
/// Missile exhaust length at the mesh's local Z scale, metres.
const FLAME_LEN: f32 = 2.6;

const BANDIT_HP: i32 = 8;
/// Target drones: a lazy orbit (~1.7 G) instead of the 5 G fighter turn v1 had,
/// so the lock cone is flyable and a gun pass is possible.
const DRONE_SPEED: f32 = 180.0;
const DRONE_RADIUS: f32 = 1800.0;
/// Escorts chase the player at ~3 G. They are faster than the jet's cruise so
/// they can close, but a boosted F-22 can still run away.
const ESCORT_SPEED: f32 = 260.0;
const ESCORT_TURN_RAD_S: f32 = 0.14;
const ESCORT_FIRE_RANGE: f32 = 2600.0;
const ESCORT_FIRE_CONE_DEG: f32 = 18.0;
const ESCORT_FIRE_CD_S: f32 = 6.0;
const BANDITS_PER_WAVE: usize = 3;
const WAVE_DELAY_S: f32 = 4.0;

/// Incoming IR missile: slower and less agile than the player's AIM-9, so a
/// hard break or a flare can defeat it.
const THREAT_SPEED_MAX: f32 = 480.0;
const THREAT_ACCEL: f32 = 160.0;
const THREAT_MOTOR_S: f32 = 3.0;
const THREAT_TURN_RAD_S: f32 = 0.75;
const THREAT_LIFE_S: f32 = 9.0;
const THREAT_HIT_M: f32 = 12.0;
const MAX_INBOUND: usize = 2;
/// A flare this close for this long seduces an incoming missile.
const DECOY_RANGE_M: f32 = 400.0;
const DECOY_LOCK_S: f32 = 0.3;
/// Edge markers for inbound missiles: pooled slots and their size / inset.
const THREAT_MARKS: usize = 4;
const MARK_W: f32 = 150.0;
const MARK_H: f32 = 30.0;
/// Edge margins. The bottom is deeper so a behind-the-jet marker sits above
/// the lock/weapons text instead of on top of it.
const EDGE_X: f32 = 58.0;
const EDGE_TOP: f32 = 48.0;
const EDGE_BOTTOM: f32 = 108.0;
/// Corner tactical display: 360° heading-up. These are the selectable ranges;
/// the outer ring is always the selected range, the inner ring half of it.
const RADAR_RANGES_M: [f32; 3] = [10_000.0, 20_000.0, 40_000.0];
const RADAR_RADIUS_PX: f32 = 88.0;
const RADAR_CENTRE_PX: f32 = 95.0;
const RADAR_DOTS: usize = 8;

/// Door travel rate (fraction of full travel per second): 0.3 s to open.
const BAY_RATE: f32 = 3.4;
/// The four side-bay door nodes and the sign of their hinge rotation about the
/// model's fore-aft axis. Opposite sides, and the inner/outer panels on each
/// side, hinge opposite ways (verified from the GLB's own animation).
const DOORS: [(&str, f32); 4] = [
    ("Cube.058_88", 1.0),
    ("Cube.062_90", -1.0),
    ("Cube.064_83", -1.0),
    ("Cube.061_89", 1.0),
];
/// The two missile launch rails: their local extension at full bay-open, taken
/// from the GLB animation (translation in the model frame, rotation about Z).
const RAILS: [(&str, Vec3, f32); 2] = [
    ("Cube.068_76", Vec3::new(0.0304, -0.0108, 0.0), -0.1607),
    ("Cube.069_78", Vec3::new(-0.0304, 0.0108, 0.0), 0.1607),
];
/// The two in-bay Sidewinder meshes, hidden once their missile is fired.
const MISSILE_MESHES: [[&str; 3]; 2] = [
    ["Object_141", "Object_142", "Object_143"],
    ["Object_147", "Object_148", "Object_149"],
];

/// The two air-to-air weapons. `Ir` is the current AIM-9-class round fired from
/// the side bays; `Bvr` is the AIM-120-class round.
#[derive(Clone, Copy, PartialEq, Eq)]
enum Weapon {
    Ir,
    Bvr,
}

#[derive(Resource)]
pub struct Combat {
    /// Set once the first wave and the HUD have been spawned in this flight.
    pub spawned: bool,
    lock: Option<Entity>,
    weapon: Weapon,
    /// Sidewinder-class rounds left (side bays).
    pub missiles: i32,
    /// AIM-120-class rounds left (belly stations; the model has no main bay).
    pub bvr: i32,
    pub rounds: i32,
    pub flares: i32,
    pub kills: u32,
    pub wave: u32,
    /// Selected TSD range, one of [`RADAR_RANGES_M`].
    radar_range_m: f32,
    gun_cd: f32,
    /// Eased bay-door travel, 0 closed … 1 open.
    bay: f32,
    /// Countdown to the next wave, or 0 when a wave is alive.
    respawn: f32,
    /// "Shot down / respawned" banner timer, seconds.
    down: f32,
}

impl Default for Combat {
    fn default() -> Self {
        Combat {
            spawned: false,
            lock: None,
            weapon: Weapon::Ir,
            missiles: 2,
            bvr: AMRAAM_ROUNDS,
            rounds: GUN_ROUNDS,
            flares: 8,
            kills: 0,
            wave: 0,
            radar_range_m: RADAR_RANGES_M[1],
            gun_cd: 0.0,
            bay: 0.0,
            respawn: 0.0,
            down: 0.0,
        }
    }
}

/// The procedural meshes and materials, built with the first wave and reused
/// for every missile, tracer, explosion and flare afterwards.
#[derive(Resource)]
struct CombatAssets {
    bandit_body: Handle<Mesh>,
    bandit_wing: Handle<Mesh>,
    bandit_mat: Handle<StandardMaterial>,
    escort_mat: Handle<StandardMaterial>,
    missile_mesh: Handle<Mesh>,
    missile_mat: Handle<StandardMaterial>,
    bvr_mesh: Handle<Mesh>,
    bvr_mat: Handle<StandardMaterial>,
    tracer_mesh: Handle<Mesh>,
    tracer_mat: Handle<StandardMaterial>,
    boom_mesh: Handle<Mesh>,
    boom_mat: Handle<StandardMaterial>,
    flare_mesh: Handle<Mesh>,
    flare_mat: Handle<StandardMaterial>,
    flame_mesh: Handle<Mesh>,
    flame_mat: Handle<StandardMaterial>,
    smoke_mesh: Handle<Mesh>,
    flash_mesh: Handle<Mesh>,
    flash_mat: Handle<StandardMaterial>,
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum BanditRole {
    /// Circles a fixed orbit; the easy targets.
    Drone,
    /// Chases the player and launches IR missiles when it has a shot.
    Escort,
}

#[derive(Component)]
struct Bandit {
    role: BanditRole,
    /// Orbit centre/radius/phase — only meaningful for a drone.
    center: Vec3,
    radius: f32,
    angle: f32,
    speed: f32,
    vel: Vec3,
    hp: i32,
    fire_cd: f32,
}

#[derive(Component)]
struct Missile {
    target: Entity,
    vel: Vec3,
    life: f32,
    motor: f32,
    /// Per-round kinematics (Sidewinder vs AMRAAM).
    accel: f32,
    max_speed: f32,
    turn: f32,
    /// The exhaust plume entity, dropped when the motor burns out.
    flame: Option<Entity>,
}

/// An IR missile fired by an escort at the player.
#[derive(Component)]
struct EnemyMissile {
    vel: Vec3,
    life: f32,
    motor: f32,
    flame: Option<Entity>,
    /// A flare that has seduced this round; it stops chasing the jet.
    decoy: Option<Entity>,
    /// Seconds a flare has been close enough to seduce it.
    flare_time: f32,
}

/// The bright exhaust bead riding behind a missile.
#[derive(Component)]
struct MissileFlame;

/// A smoke puff (missile launch cloud or gun gas) that grows and fades.
#[derive(Component)]
struct Smoke {
    vel: Vec3,
    age: f32,
    life: f32,
    size0: f32,
    size1: f32,
    alpha0: f32,
    mat: Handle<StandardMaterial>,
}

/// A one-frame-ish bright bloom at the gun muzzle.
#[derive(Component)]
struct MuzzleFlash {
    life: f32,
}

#[derive(Component)]
struct Tracer {
    vel: Vec3,
    prev: Vec3,
    life: f32,
}

#[derive(Component)]
struct Explosion {
    age: f32,
    life: f32,
}

#[derive(Component)]
struct Flare {
    vel: Vec3,
    life: f32,
}

#[derive(Component)]
struct TargetBox(usize);

#[derive(Component)]
struct StatusText;

#[derive(Component)]
struct LockText;

#[derive(Component)]
struct ThreatText;

/// One edge marker for an incoming missile: a rotating arrow plus its range.
#[derive(Component)]
struct ThreatMark(usize);

#[derive(Component)]
struct ThreatGlyph(usize);

#[derive(Component)]
struct ThreatDist(usize);

/// The computed gun lead point: put the nose cross on this ring and fire.
#[derive(Component)]
struct GunPipper;

/// One contact dot inside the corner tactical display.
#[derive(Component)]
struct RadarDot(usize);

#[derive(Component)]
struct RadarLabel;

#[derive(Component)]
struct RadarInfo;

/// MAWS-style launch warning: a short synthesized two-pulse tone. An IR seeker
/// is passive, so there is no lock tone — this plays only once a hostile
/// missile has actually left the rail.
#[derive(Resource)]
struct LaunchTone(Handle<AudioSource>);

pub struct CombatPlugin;

impl Plugin for CombatPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<Combat>()
            .add_systems(Startup, setup_launch_tone)
            .add_systems(
                Update,
                (
                    spawn_combat,
                    update_lock,
                    update_bandits,
                    fire_weapons,
                    guide_missiles,
                    guide_threats,
                    update_tracers,
                    update_effects,
                    respawn_wave,
                    drive_bays,
                    update_hud,
                    update_threat_marks,
                    update_gun_pipper,
                    update_radar,
                )
                    .chain()
                    .run_if(unpaused),
            )
            .add_systems(OnEnter(AppState::Flying), reset_combat);
    }
}

/// Build the launch tone once, in memory: a synthesized WAV keeps the warning
/// self-contained (no asset file to ship) and short enough to cut through the
/// fight without becoming a siren.
fn setup_launch_tone(mut commands: Commands, mut sources: ResMut<Assets<AudioSource>>) {
    let handle = sources.add(AudioSource {
        bytes: Arc::from(launch_tone_wav()),
    });
    commands.insert_resource(LaunchTone(handle));
}

fn play_launch_tone(commands: &mut Commands, tone: &LaunchTone) {
    commands.spawn((
        AudioPlayer::new(tone.0.clone()),
        PlaybackSettings::DESPAWN.with_volume(Volume::Linear(0.55)),
        DespawnOnExit(AppState::Flying),
    ));
}

const TONE_RATE: u32 = 44_100;
const TONE_HZ: f32 = 1_180.0;
const TONE_PULSE_S: f32 = 0.085;
const TONE_GAP_S: f32 = 0.045;

/// Two crisp pulses — the classic "missile away" rhythm. The envelope is a
/// short click-free attack and release; the 1.18 kHz tone sits above the
/// engine and wind noise.
fn launch_tone_wav() -> Vec<u8> {
    let mut samples: Vec<i16> = Vec::new();
    for _ in 0..2 {
        let n = (TONE_RATE as f32 * TONE_PULSE_S).round() as usize;
        for i in 0..n {
            let t = i as f32 / TONE_RATE as f32;
            let attack = (t / 0.004).min(1.0);
            let release = ((TONE_PULSE_S - t) / 0.02).clamp(0.0, 1.0);
            let s = (std::f32::consts::TAU * TONE_HZ * t).sin() * attack * release * 0.55;
            samples.push((s * i16::MAX as f32) as i16);
        }
        let silence = (TONE_RATE as f32 * TONE_GAP_S).round() as usize;
        samples.extend(std::iter::repeat_n(0i16, silence));
    }
    wav_bytes(&samples)
}

/// Minimal 16-bit mono PCM WAV container, which Bevy's `wav` feature decodes.
fn wav_bytes(samples: &[i16]) -> Vec<u8> {
    let data_len = (samples.len() * 2) as u32;
    let mut out = Vec::with_capacity(44 + data_len as usize);
    out.extend_from_slice(b"RIFF");
    out.extend_from_slice(&(36 + data_len).to_le_bytes());
    out.extend_from_slice(b"WAVE");
    out.extend_from_slice(b"fmt ");
    out.extend_from_slice(&16u32.to_le_bytes());
    out.extend_from_slice(&1u16.to_le_bytes()); // PCM
    out.extend_from_slice(&1u16.to_le_bytes()); // mono
    out.extend_from_slice(&TONE_RATE.to_le_bytes());
    out.extend_from_slice(&(TONE_RATE * 2).to_le_bytes());
    out.extend_from_slice(&2u16.to_le_bytes());
    out.extend_from_slice(&16u16.to_le_bytes());
    out.extend_from_slice(b"data");
    out.extend_from_slice(&data_len.to_le_bytes());
    for s in samples {
        out.extend_from_slice(&s.to_le_bytes());
    }
    out
}

fn reset_combat(mut combat: ResMut<Combat>) {
    *combat = Combat::default();
}

/// Combat exists only for the F-22 in arcade mode, and only in a mission arena
/// (or with the `COMBAT=1` screenshot hook, which can run over the test grid).
fn combat_active(sim: &Sim, state: &AircraftState, map: &ActiveMap) -> bool {
    sim.name == "f22"
        && state.arcade
        && (map.0.is_some() || std::env::var("COMBAT").is_ok_and(|v| v == "1"))
}

fn combat_assets(meshes: &mut Assets<Mesh>, mats: &mut Assets<StandardMaterial>) -> CombatAssets {
    CombatAssets {
        bandit_body: meshes.add(Cuboid::new(1.4, 1.2, 7.0)),
        bandit_wing: meshes.add(Cuboid::new(9.0, 0.22, 2.2)),
        bandit_mat: mats.add(StandardMaterial {
            base_color: Color::srgb(0.42, 0.10, 0.09),
            metallic: 0.3,
            perceptual_roughness: 0.6,
            emissive: LinearRgba::new(0.05, 0.01, 0.0, 1.0),
            ..default()
        }),
        // Escorts read hotter than the drones: they are the ones that shoot.
        escort_mat: mats.add(StandardMaterial {
            base_color: Color::srgb(0.62, 0.22, 0.06),
            metallic: 0.35,
            perceptual_roughness: 0.5,
            emissive: LinearRgba::new(0.12, 0.02, 0.0, 1.0),
            ..default()
        }),
        missile_mesh: meshes.add(Cuboid::new(0.16, 0.16, 2.0)),
        missile_mat: mats.add(StandardMaterial {
            base_color: Color::srgb(0.78, 0.78, 0.74),
            metallic: 0.6,
            perceptual_roughness: 0.35,
            ..default()
        }),
        bvr_mesh: meshes.add(Cuboid::new(0.18, 0.18, 3.8)),
        bvr_mat: mats.add(StandardMaterial {
            base_color: Color::srgb(0.86, 0.87, 0.84),
            metallic: 0.5,
            perceptual_roughness: 0.4,
            ..default()
        }),
        tracer_mesh: meshes.add(Cuboid::new(0.18, 0.18, 5.0)),
        tracer_mat: mats.add(StandardMaterial {
            base_color: Color::srgb(1.0, 0.72, 0.25),
            emissive: LinearRgba::new(6.0, 3.0, 0.4, 1.0),
            ..default()
        }),
        boom_mesh: meshes.add(Sphere::new(1.0)),
        boom_mat: mats.add(StandardMaterial {
            base_color: Color::srgb(1.0, 0.55, 0.12),
            emissive: LinearRgba::new(8.0, 3.2, 0.5, 1.0),
            ..default()
        }),
        flare_mesh: meshes.add(Sphere::new(0.35)),
        flare_mat: mats.add(StandardMaterial {
            base_color: Color::srgb(1.0, 0.9, 0.6),
            emissive: LinearRgba::new(10.0, 6.0, 1.5, 1.0),
            ..default()
        }),
        // Elongated sphere: reads as a hot exhaust bead at any range, with the
        // local Z scale giving it its length.
        flame_mesh: meshes.add(Sphere::new(0.5)),
        flame_mat: mats.add(StandardMaterial {
            base_color: Color::srgb(1.0, 0.82, 0.45),
            emissive: LinearRgba::new(14.0, 7.0, 1.6, 1.0),
            ..default()
        }),
        smoke_mesh: meshes.add(Sphere::new(1.0)),
        flash_mesh: meshes.add(Sphere::new(0.22)),
        flash_mat: mats.add(StandardMaterial {
            base_color: Color::srgb(1.0, 0.86, 0.55),
            emissive: LinearRgba::new(24.0, 13.0, 3.5, 1.0),
            ..default()
        }),
    }
}

fn spawn_combat(
    mut commands: Commands,
    mut combat: ResMut<Combat>,
    sim: Res<Sim>,
    state: Res<AircraftState>,
    map: Res<ActiveMap>,
    root: Query<&Transform, With<AircraftRoot>>,
    existing: Query<(), With<Bandit>>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut mats: ResMut<Assets<StandardMaterial>>,
) {
    if combat.spawned || !existing.is_empty() || !combat_active(&sim, &state, &map) {
        return;
    }
    let Ok(root) = root.single() else {
        return;
    };
    let assets = combat_assets(&mut meshes, &mut mats);
    let spawned = spawn_wave(&mut commands, &assets, root);
    commands.insert_resource(assets);
    spawn_hud(&mut commands);
    combat.spawned = true;
    combat.wave = 1;
    // Screenshot hooks: hold the first bandit locked, and optionally start on
    // the BVR store.
    if std::env::var("COMBAT_LOCK").is_ok() {
        combat.lock = spawned.first().copied();
    }
    if std::env::var("COMBAT_WEAPON").is_ok_and(|v| v == "bvr") {
        combat.weapon = Weapon::Bvr;
    }
}

/// Spawn one wave ahead of the jet: two orbiting drones plus one escort that
/// turns into the player and shoots. Escorts are the orange airframes.
fn spawn_wave(commands: &mut Commands, assets: &CombatAssets, root: &Transform) -> Vec<Entity> {
    let fwd = root.rotation * Vec3::NEG_Z;
    let side = root.rotation * Vec3::X;
    let offsets = [
        (2500.0, -900.0, 120.0, BanditRole::Drone),
        (3200.0, 800.0, -80.0, BanditRole::Drone),
        // The escort starts behind the jet: the missile warning is the first
        // thing the player hears about it.
        (-2500.0, 0.0, 200.0, BanditRole::Escort),
    ];
    let mut spawned = Vec::with_capacity(offsets.len());
    for (i, (ahead, lateral, up, role)) in offsets.into_iter().enumerate() {
        let center = root.translation + fwd * ahead + side * lateral + Vec3::Y * up;
        let (speed, radius, angle) = match role {
            BanditRole::Drone => (DRONE_SPEED, DRONE_RADIUS, i as f32 * 2.1),
            BanditRole::Escort => (ESCORT_SPEED, 0.0, 0.0),
        };
        let vel = match role {
            BanditRole::Drone => Vec3::new(-angle.sin(), 0.0, angle.cos()) * speed,
            // The escort starts pointed at the jet.
            BanditRole::Escort => (root.translation - center).normalize_or_zero() * speed,
        };
        let mat = match role {
            BanditRole::Drone => assets.bandit_mat.clone(),
            BanditRole::Escort => assets.escort_mat.clone(),
        };
        let mut entity = commands.spawn((
            Bandit {
                role,
                center,
                radius,
                angle,
                speed,
                vel,
                hp: BANDIT_HP,
                fire_cd: ESCORT_FIRE_CD_S,
            },
            Mesh3d(assets.bandit_body.clone()),
            MeshMaterial3d(mat.clone()),
            Transform::from_translation(center),
            DespawnOnExit(AppState::Flying),
        ));
        entity.with_children(|p| {
            p.spawn((
                Mesh3d(assets.bandit_wing.clone()),
                MeshMaterial3d(mat.clone()),
                Transform::from_xyz(0.0, 0.0, 0.6),
            ));
            p.spawn((
                Mesh3d(assets.bandit_wing.clone()),
                MeshMaterial3d(mat.clone()),
                Transform::from_xyz(0.0, 0.0, 2.6).with_scale(Vec3::new(0.45, 1.0, 0.5)),
            ));
        });
        spawned.push(entity.id());
    }
    spawned
}

/// The combat overlay: one pooled box per visible bandit plus the two text
/// lines at the bottom.
fn spawn_hud(commands: &mut Commands) {
    commands
        .spawn((
            Node {
                position_type: PositionType::Absolute,
                width: Val::Percent(100.0),
                height: Val::Percent(100.0),
                ..default()
            },
            GlobalZIndex(21),
            DespawnOnExit(AppState::Flying),
        ))
        .with_children(|root| {
            // Incoming-missile / respawn banner.
            root.spawn(Node {
                position_type: PositionType::Absolute,
                left: Val::Percent(50.0),
                top: Val::Px(64.0),
                width: Val::Px(560.0),
                margin: UiRect {
                    left: Val::Px(-280.0),
                    ..default()
                },
                justify_content: JustifyContent::Center,
                ..default()
            })
            .with_children(|label| {
                label.spawn((
                    ThreatText,
                    Text::new(""),
                    TextFont {
                        font_size: FontSize::Px(18.0),
                        ..default()
                    },
                    TextColor(Color::srgb(1.0, 0.2, 0.15)),
                    TextLayout::new(Justify::Center, LineBreak::NoWrap),
                ));
            });
            // Inbound-missile edge markers: a rotating ">" and a range label.
            for i in 0..THREAT_MARKS {
                root.spawn((
                    ThreatMark(i),
                    Node {
                        position_type: PositionType::Absolute,
                        display: Display::None,
                        width: Val::Px(MARK_W),
                        height: Val::Px(MARK_H),
                        align_items: AlignItems::Center,
                        ..default()
                    },
                    GlobalZIndex(30),
                ))
                .with_children(|mark| {
                    mark.spawn((
                        ThreatGlyph(i),
                        Text::new(">"),
                        TextFont {
                            font_size: FontSize::Px(28.0),
                            ..default()
                        },
                        TextColor(Color::srgb(1.0, 0.25, 0.15)),
                        UiTransform::IDENTITY,
                    ));
                    mark.spawn((
                        ThreatDist(i),
                        Node {
                            position_type: PositionType::Absolute,
                            left: Val::Px(32.0),
                            top: Val::Px(5.0),
                            ..default()
                        },
                        Text::new(""),
                        TextFont {
                            font_size: FontSize::Px(17.0),
                            ..default()
                        },
                        TextColor(Color::srgb(1.0, 0.4, 0.2)),
                        TextLayout::new(Justify::Left, LineBreak::NoWrap),
                    ));
                });
            }
            // Gun lead point: a ring at the target's future position; the nose
            // cross goes on it when firing.
            root.spawn((
                GunPipper,
                Node {
                    position_type: PositionType::Absolute,
                    display: Display::None,
                    width: Val::Px(20.0),
                    height: Val::Px(20.0),
                    border: UiRect::all(Val::Px(2.0)),
                    border_radius: BorderRadius::all(Val::Percent(50.0)),
                    ..default()
                },
                BorderColor::all(Color::srgb(1.0, 0.85, 0.3)),
                BackgroundColor(Color::NONE),
                GlobalZIndex(23),
            ));
            // Corner tactical display: heading-up, 360°, 20 km outer ring and
            // a 10 km inner ring. Contacts are coloured by role and the locked
            // one turns gold; the nose tick points up.
            root.spawn((
                Node {
                    position_type: PositionType::Absolute,
                    right: Val::Px(26.0),
                    top: Val::Px(26.0),
                    width: Val::Px(190.0),
                    height: Val::Px(190.0),
                    border: UiRect::all(Val::Px(2.0)),
                    border_radius: BorderRadius::all(Val::Percent(50.0)),
                    ..default()
                },
                BorderColor::all(Color::srgba(0.35, 0.9, 0.55, 0.55)),
                BackgroundColor(Color::srgba(0.02, 0.10, 0.06, 0.55)),
                GlobalZIndex(22),
            ))
            .with_children(|panel| {
                panel.spawn((
                    Node {
                        position_type: PositionType::Absolute,
                        left: Val::Px(47.0),
                        top: Val::Px(47.0),
                        width: Val::Px(96.0),
                        height: Val::Px(96.0),
                        border: UiRect::all(Val::Px(1.0)),
                        border_radius: BorderRadius::all(Val::Percent(50.0)),
                        ..default()
                    },
                    BorderColor::all(Color::srgba(0.35, 0.9, 0.55, 0.25)),
                    BackgroundColor(Color::NONE),
                ));
                // Own-ship centre and the nose tick at the top.
                panel.spawn((
                    Node {
                        position_type: PositionType::Absolute,
                        left: Val::Px(93.0),
                        top: Val::Px(93.0),
                        width: Val::Px(4.0),
                        height: Val::Px(4.0),
                        border_radius: BorderRadius::all(Val::Percent(50.0)),
                        ..default()
                    },
                    BackgroundColor(Color::srgba(0.5, 1.0, 0.6, 0.9)),
                ));
                panel.spawn((
                    Node {
                        position_type: PositionType::Absolute,
                        left: Val::Px(92.0),
                        top: Val::Px(6.0),
                        width: Val::Px(2.0),
                        height: Val::Px(10.0),
                        ..default()
                    },
                    BackgroundColor(Color::srgba(0.5, 1.0, 0.6, 0.6)),
                ));
                panel
                    .spawn(Node {
                        position_type: PositionType::Absolute,
                        left: Val::Px(0.0),
                        bottom: Val::Px(8.0),
                        width: Val::Px(186.0),
                        justify_content: JustifyContent::Center,
                        ..default()
                    })
                    .with_children(|label| {
                        label.spawn((
                            RadarLabel,
                            Text::new("TSD 20KM"),
                            TextFont {
                                font_size: FontSize::Px(10.0),
                                ..default()
                            },
                            TextColor(Color::srgba(0.5, 1.0, 0.6, 0.8)),
                        ));
                    });
                for i in 0..RADAR_DOTS {
                    panel.spawn((
                        RadarDot(i),
                        Node {
                            position_type: PositionType::Absolute,
                            display: Display::None,
                            width: Val::Px(8.0),
                            height: Val::Px(8.0),
                            ..default()
                        },
                        BackgroundColor(Color::srgb(1.0, 0.3, 0.2)),
                        GlobalZIndex(23),
                    ));
                }
            });
            // Locked-contact telemetry under the scope.
            root.spawn(Node {
                position_type: PositionType::Absolute,
                right: Val::Px(26.0),
                top: Val::Px(224.0),
                width: Val::Px(240.0),
                justify_content: JustifyContent::FlexEnd,
                ..default()
            })
            .with_children(|label| {
                label.spawn((
                    RadarInfo,
                    Text::new(""),
                    TextFont {
                        font_size: FontSize::Px(12.0),
                        ..default()
                    },
                    TextColor(Color::srgba(0.5, 1.0, 0.6, 0.85)),
                    TextLayout::new(Justify::Right, LineBreak::NoWrap),
                ));
            });
            for i in 0..BANDITS_PER_WAVE {
                root.spawn((
                    TargetBox(i),
                    Node {
                        position_type: PositionType::Absolute,
                        display: Display::None,
                        width: Val::Px(34.0),
                        height: Val::Px(34.0),
                        border: UiRect::all(Val::Px(2.0)),
                        ..default()
                    },
                    BorderColor::all(Color::srgb(1.0, 0.25, 0.15)),
                    BackgroundColor(Color::NONE),
                    GlobalZIndex(21),
                ));
            }
            root.spawn(Node {
                position_type: PositionType::Absolute,
                left: Val::Percent(50.0),
                bottom: Val::Px(58.0),
                width: Val::Px(560.0),
                margin: UiRect {
                    left: Val::Px(-280.0),
                    ..default()
                },
                justify_content: JustifyContent::Center,
                ..default()
            })
            .with_children(|label| {
                label.spawn((
                    LockText,
                    Text::new(""),
                    TextFont {
                        font_size: FontSize::Px(16.0),
                        ..default()
                    },
                    TextColor(Color::srgb(1.0, 0.3, 0.2)),
                    TextLayout::new(Justify::Center, LineBreak::NoWrap),
                ));
            });
            root.spawn(Node {
                position_type: PositionType::Absolute,
                left: Val::Percent(50.0),
                bottom: Val::Px(36.0),
                width: Val::Px(560.0),
                margin: UiRect {
                    left: Val::Px(-280.0),
                    ..default()
                },
                justify_content: JustifyContent::Center,
                ..default()
            })
            .with_children(|label| {
                label.spawn((
                    StatusText,
                    Text::new(""),
                    TextFont {
                        font_size: FontSize::Px(14.0),
                        ..default()
                    },
                    TextColor(Color::srgb(0.55, 1.0, 0.65)),
                    TextLayout::new(Justify::Center, LineBreak::NoWrap),
                ));
            });
        });
}

fn key_of(bindings: &Bindings, action: ToggleAction) -> Option<KeyCode> {
    bindings
        .toggles
        .iter()
        .find(|t| t.action == action)
        .map(|t| t.key)
}

fn angle_off(fwd: Vec3, to: Vec3) -> f32 {
    fwd.normalize_or_zero()
        .dot(to.normalize_or_zero())
        .clamp(-1.0, 1.0)
        .acos()
        .to_degrees()
}

#[allow(clippy::too_many_arguments)]
fn update_bandits(
    mut commands: Commands,
    time: Res<Time>,
    state: Res<AircraftState>,
    assets: Option<Res<CombatAssets>>,
    launch_tone: Option<Res<LaunchTone>>,
    player: Query<&Transform, (With<AircraftRoot>, Without<Bandit>)>,
    mut q: Query<(&mut Bandit, &mut Transform)>,
    threats: Query<&EnemyMissile>,
) {
    let dt = time.delta_secs();
    let (Ok(player_tf), Some(assets)) = (player.single(), assets) else {
        return;
    };
    let player_pos = player_tf.translation;
    let player_fwd = player_tf.rotation * Vec3::NEG_Z;
    let player_vel = player_fwd * (state.cas_kts * 0.514_444) as f32;
    let inbound = threats.iter().filter(|m| m.decoy.is_none()).count();

    for (mut bandit, mut tf) in &mut q {
        match bandit.role {
            BanditRole::Drone => {
                bandit.angle += bandit.speed / bandit.radius * dt;
                let (s, c) = bandit.angle.sin_cos();
                let pos = bandit.center + Vec3::new(c * bandit.radius, 0.0, s * bandit.radius);
                let tangent = Vec3::new(-s, 0.0, c);
                bandit.vel = tangent * bandit.speed;
                tf.translation = pos;
                // Nose along the tangent (model forward is -Z), banked in.
                let base = Transform::from_translation(pos).looking_to(tangent, Vec3::Y);
                tf.rotation = base.rotation * Quat::from_rotation_z(0.55);
            }
            BanditRole::Escort => {
                bandit.fire_cd = (bandit.fire_cd - dt).max(0.0);
                let to = player_pos - tf.translation;
                let dist = to.length();
                let speed = bandit.speed.max(1.0);
                // Lead the chase a little so it does not sit behind a fast jet.
                let desired =
                    (player_pos + player_vel * (dist / speed) - tf.translation).normalize_or_zero();
                let current = bandit.vel.normalize_or_zero();
                let angle = current.dot(desired).clamp(-1.0, 1.0).acos();
                let dir = if angle > 1e-4 {
                    let axis = current.cross(desired).normalize_or_zero();
                    let turn = (ESCORT_TURN_RAD_S * dt).min(angle);
                    Quat::from_axis_angle(axis, turn) * current
                } else {
                    desired
                };
                bandit.vel = dir * speed;
                tf.translation += bandit.vel * dt;
                let base = Transform::from_translation(tf.translation).looking_to(dir, Vec3::Y);
                let bank = current.cross(desired).dot(Vec3::Y).clamp(-1.0, 1.0) * 0.9;
                tf.rotation = base.rotation * Quat::from_rotation_z(bank);

                // A tail shot: the escort must actually be behind the jet, not
                // firing head-on through it. Cap the rounds in the air so the
                // fight stays readable.
                let behind = behind_the_tail(player_pos, player_fwd, tf.translation);
                if bandit.fire_cd <= 0.0
                    && behind
                    && can_escort_fire(dir, to, dist)
                    && inbound < MAX_INBOUND
                {
                    bandit.fire_cd = ESCORT_FIRE_CD_S;
                    spawn_threat(
                        &mut commands,
                        &assets,
                        launch_tone.as_deref(),
                        tf.translation + dir * 5.0,
                        bandit.vel,
                        dir,
                    );
                }
            }
        }
    }
}

/// True when `other` sits aft of the player's beam, i.e. the player would have
/// to look back to see it. Escorts only shoot from there.
fn behind_the_tail(player_pos: Vec3, player_fwd: Vec3, other: Vec3) -> bool {
    (other - player_pos).dot(player_fwd) < 0.0
}

/// The escort's shot gate: the jet has to sit in the nose cone and inside the
/// missile's useful range.
fn can_escort_fire(forward: Vec3, to_player: Vec3, dist: f32) -> bool {
    dist < ESCORT_FIRE_RANGE && angle_off(forward, to_player) < ESCORT_FIRE_CONE_DEG
}

/// Fire one IR missile from an escort: the same body as the player's round,
/// but slower, less agile and with a shorter motor.
fn spawn_threat(
    commands: &mut Commands,
    assets: &CombatAssets,
    launch_tone: Option<&LaunchTone>,
    origin: Vec3,
    carrier_vel: Vec3,
    dir: Vec3,
) {
    let rot = Quat::from_rotation_arc(Vec3::NEG_Z, dir);
    let flame = spawn_flame(commands, assets, origin, rot);
    if let Some(tone) = launch_tone {
        play_launch_tone(commands, tone);
    }
    commands.spawn((
        EnemyMissile {
            vel: carrier_vel + dir * 40.0,
            life: THREAT_LIFE_S,
            motor: THREAT_MOTOR_S,
            flame: Some(flame),
            decoy: None,
            flare_time: 0.0,
        },
        Mesh3d(assets.missile_mesh.clone()),
        MeshMaterial3d(assets.missile_mat.clone()),
        Transform::from_translation(origin)
            .with_rotation(rot)
            .with_scale(Vec3::splat(1.2)),
        DespawnOnExit(AppState::Flying),
    ));
}

/// The bright exhaust bead behind any missile (player or incoming).
fn spawn_flame(commands: &mut Commands, assets: &CombatAssets, at: Vec3, rot: Quat) -> Entity {
    commands
        .spawn((
            MissileFlame,
            Mesh3d(assets.flame_mesh.clone()),
            MeshMaterial3d(assets.flame_mat.clone()),
            Transform::from_translation(at)
                .with_rotation(rot)
                .with_scale(Vec3::new(0.5, 0.5, FLAME_LEN)),
            DespawnOnExit(AppState::Flying),
        ))
        .id()
}

fn update_lock(
    keys: Res<ButtonInput<KeyCode>>,
    bindings: Res<Bindings>,
    root: Query<&Transform, With<AircraftRoot>>,
    bandits: Query<(Entity, &Transform), With<Bandit>>,
    mut combat: ResMut<Combat>,
) {
    let Ok(root) = root.single() else {
        return;
    };
    let fwd = root.rotation * Vec3::NEG_Z;
    let (acq_range, acq_cone, keep_range, keep_cone) = match combat.weapon {
        Weapon::Ir => (LOCK_RANGE_M, LOCK_CONE_DEG, KEEP_RANGE_M, KEEP_CONE_DEG),
        Weapon::Bvr => (
            BVR_LOCK_RANGE_M,
            BVR_LOCK_CONE_DEG,
            BVR_KEEP_RANGE_M,
            BVR_KEEP_CONE_DEG,
        ),
    };

    if let Some(key) = key_of(&bindings, ToggleAction::RadarRange) {
        if keys.just_pressed(key) {
            let i = RADAR_RANGES_M
                .iter()
                .position(|r| *r == combat.radar_range_m)
                .unwrap_or(1);
            combat.radar_range_m = RADAR_RANGES_M[(i + 1) % RADAR_RANGES_M.len()];
        }
    }

    // Drop a lock when its target dies or slides out of the keep envelope.
    if let Some(e) = combat.lock {
        match bandits.get(e) {
            Ok((_, tf)) => {
                let to = tf.translation - root.translation;
                if to.length() > keep_range || angle_off(fwd, to) > keep_cone {
                    combat.lock = None;
                }
            }
            Err(_) => combat.lock = None,
        }
    }

    // Screenshot hook: hold the first bandit locked regardless of the envelope
    // so a headless pass can photograph the lock box and open bays.
    if std::env::var("COMBAT_LOCK").is_ok() {
        combat.lock = bandits.iter().next().map(|(e, _)| e);
    }

    let Some(key) = key_of(&bindings, ToggleAction::CycleTarget) else {
        return;
    };
    if !keys.just_pressed(key) {
        return;
    }

    // Distance order; each press takes the entry after the current lock so the
    // player can walk the sky instead of sticking to the nearest bandit.
    let mut candidates: Vec<(f32, Entity)> = bandits
        .iter()
        .filter_map(|(entity, tf)| {
            let to = tf.translation - root.translation;
            let dist = to.length();
            (dist <= acq_range && angle_off(fwd, to) <= acq_cone).then_some((dist, entity))
        })
        .collect();
    candidates.sort_by(|a, b| a.0.total_cmp(&b.0));
    if candidates.is_empty() {
        combat.lock = None;
        return;
    }
    combat.lock = Some(
        match combat
            .lock
            .and_then(|cur| candidates.iter().position(|c| c.1 == cur))
        {
            Some(i) => candidates[(i + 1) % candidates.len()].1,
            None => candidates[0].1,
        },
    );
}

#[allow(clippy::too_many_arguments)]
fn fire_weapons(
    mut commands: Commands,
    time: Res<Time>,
    keys: Res<ButtonInput<KeyCode>>,
    bindings: Res<Bindings>,
    state: Res<AircraftState>,
    root: Query<&Transform, With<AircraftRoot>>,
    bandits: Query<&Transform, With<Bandit>>,
    mut in_bay: Query<(&Name, &mut Visibility), With<Visibility>>,
    assets: Option<Res<CombatAssets>>,
    mut mats: ResMut<Assets<StandardMaterial>>,
    mut combat: ResMut<Combat>,
) {
    let (Ok(root), Some(assets)) = (root.single(), assets) else {
        return;
    };
    let dt = time.delta_secs();
    let fwd = root.rotation * Vec3::NEG_Z;
    let air_vel = fwd * (state.cas_kts * 0.514_444) as f32;

    if let Some(key) = key_of(&bindings, ToggleAction::CycleWeapon) {
        if keys.just_pressed(key) {
            combat.weapon = match combat.weapon {
                Weapon::Ir => Weapon::Bvr,
                Weapon::Bvr => Weapon::Ir,
            };
        }
    }

    if let Some(key) = key_of(&bindings, ToggleAction::FireMissile) {
        if keys.just_pressed(key) {
            if let Some(target) = combat.lock {
                let Ok(target_tf) = bandits.get(target) else {
                    combat.lock = None;
                    return;
                };
                match combat.weapon {
                    Weapon::Ir if combat.missiles > 0 => {
                        let left = combat.missiles % 2 == 0;
                        let bay = Vec3::new(if left { 0.176 } else { -0.176 }, 0.014, -0.01);
                        let origin = root.translation + root.rotation * model_to_body("f22", bay);
                        let dir = (target_tf.translation - origin).normalize_or_zero();
                        // The exhaust bead is its own entity (it has to sit behind
                        // the missile and flicker); the missile owns its id so it
                        // can take the flame down with it.
                        let flame = spawn_flame(&mut commands, &assets, origin, root.rotation);
                        commands.spawn((
                            Missile {
                                target,
                                vel: air_vel + dir * 40.0,
                                life: MISSILE_LIFE_S,
                                motor: MISSILE_MOTOR_S,
                                accel: MISSILE_ACCEL,
                                max_speed: MISSILE_SPEED_MAX,
                                turn: MISSILE_TURN_RAD_S,
                                flame: Some(flame),
                            },
                            Mesh3d(assets.missile_mesh.clone()),
                            MeshMaterial3d(assets.missile_mat.clone()),
                            Transform::from_translation(origin)
                                .with_rotation(root.rotation)
                                .with_scale(Vec3::splat(1.6)),
                            DespawnOnExit(AppState::Flying),
                        ));
                        // A quick cloud at the bay mouth: the round leaves with
                        // the missile's eject charge and hangs for a moment.
                        for i in 0..SMOKE_PUFFS {
                            let a = i as f32 / SMOKE_PUFFS as f32 * std::f32::consts::TAU;
                            let puff = origin
                                + root.rotation * Vec3::new(a.cos() * 0.6, a.sin() * 0.3, -0.4);
                            let push =
                                root.rotation * Vec3::new(a.cos() * 3.0, a.sin() * 1.5 - 1.0, 4.0);
                            let mat = mats.add(StandardMaterial {
                                base_color: Color::srgba(0.62, 0.62, 0.65, 0.55),
                                alpha_mode: AlphaMode::Blend,
                                unlit: true,
                                ..default()
                            });
                            commands.spawn((
                                Smoke {
                                    vel: air_vel * 0.6 + push,
                                    age: 0.0,
                                    life: SMOKE_LIFE_S,
                                    size0: 1.2,
                                    size1: 6.2,
                                    alpha0: 0.55,
                                    mat: mat.clone(),
                                },
                                Mesh3d(assets.smoke_mesh.clone()),
                                MeshMaterial3d(mat),
                                Transform::from_translation(puff).with_scale(Vec3::splat(1.2)),
                                DespawnOnExit(AppState::Flying),
                            ));
                        }
                        combat.missiles -= 1;
                        for mesh in MISSILE_MESHES[if left { 0 } else { 1 }] {
                            for (name, mut vis) in &mut in_bay {
                                if name.as_str() == mesh {
                                    *vis = Visibility::Hidden;
                                }
                            }
                        }
                    }
                    Weapon::Bvr if combat.bvr > 0 => {
                        // The model has no main bay: the AIM-120 leaves from a
                        // belly centreline station with a puff of gas.
                        let bay = Vec3::new(0.0, -0.02, 0.05);
                        let origin = root.translation + root.rotation * model_to_body("f22", bay);
                        let dir = (target_tf.translation - origin).normalize_or_zero();
                        let rot = Quat::from_rotation_arc(Vec3::NEG_Z, dir);
                        let flame = spawn_flame(&mut commands, &assets, origin, rot);
                        commands.spawn((
                            Missile {
                                target,
                                vel: air_vel + dir * 60.0,
                                life: AMRAAM_LIFE_S,
                                motor: AMRAAM_MOTOR_S,
                                accel: AMRAAM_ACCEL,
                                max_speed: AMRAAM_SPEED_MAX,
                                turn: AMRAAM_TURN_RAD_S,
                                flame: Some(flame),
                            },
                            Mesh3d(assets.bvr_mesh.clone()),
                            MeshMaterial3d(assets.bvr_mat.clone()),
                            Transform::from_translation(origin)
                                .with_rotation(rot)
                                .with_scale(Vec3::splat(1.2)),
                            DespawnOnExit(AppState::Flying),
                        ));
                        for i in 0..4 {
                            let a = i as f32 / 4.0 * std::f32::consts::TAU;
                            let puff = origin
                                + root.rotation * Vec3::new(a.cos() * 0.5, a.sin() * 0.3, -0.3);
                            let push =
                                root.rotation * Vec3::new(a.cos() * 2.5, a.sin() * 1.0 - 1.0, 3.0);
                            let mat = mats.add(StandardMaterial {
                                base_color: Color::srgba(0.62, 0.62, 0.65, 0.40),
                                alpha_mode: AlphaMode::Blend,
                                unlit: true,
                                ..default()
                            });
                            commands.spawn((
                                Smoke {
                                    vel: air_vel * 0.6 + push,
                                    age: 0.0,
                                    life: 0.9,
                                    size0: 0.7,
                                    size1: 3.2,
                                    alpha0: 0.40,
                                    mat: mat.clone(),
                                },
                                Mesh3d(assets.smoke_mesh.clone()),
                                MeshMaterial3d(mat),
                                Transform::from_translation(puff).with_scale(Vec3::splat(0.7)),
                                DespawnOnExit(AppState::Flying),
                            ));
                        }
                        combat.bvr -= 1;
                    }
                    _ => {}
                }
            }
        }
    }

    combat.gun_cd = (combat.gun_cd - dt).max(0.0);
    if let Some(key) = key_of(&bindings, ToggleAction::FireGun) {
        if keys.pressed(key) && combat.rounds > 0 && combat.gun_cd <= 0.0 {
            combat.gun_cd = GUN_INTERVAL_S;
            combat.rounds -= 1;
            // The M61A2's muzzle in the right wing root.
            let muzzle = root.translation
                + fwd * GUN_PORT_AHEAD
                + root.rotation * Vec3::new(GUN_PORT_RIGHT, GUN_PORT_UP, 0.0);
            commands.spawn((
                Tracer {
                    vel: air_vel + fwd * GUN_SPEED,
                    prev: muzzle,
                    life: GUN_LIFE_S,
                },
                Mesh3d(assets.tracer_mesh.clone()),
                MeshMaterial3d(assets.tracer_mat.clone()),
                Transform::from_translation(muzzle).with_rotation(root.rotation),
                DespawnOnExit(AppState::Flying),
            ));
            // A hot bloom at the port on every round; the rapid fire makes it
            // strobe rather than sit as a steady light.
            commands.spawn((
                MuzzleFlash { life: 0.06 },
                Mesh3d(assets.flash_mesh.clone()),
                MeshMaterial3d(assets.flash_mat.clone()),
                Transform::from_translation(muzzle)
                    .with_rotation(root.rotation)
                    .with_scale(Vec3::splat(1.3)),
                DespawnOnExit(AppState::Flying),
            ));
            // Gun gas every few rounds: a thin puff that falls behind the jet.
            if combat.rounds % GUN_SMOKE_EVERY == 0 {
                let mat = mats.add(StandardMaterial {
                    base_color: Color::srgba(0.66, 0.66, 0.68, 0.30),
                    alpha_mode: AlphaMode::Blend,
                    unlit: true,
                    ..default()
                });
                commands.spawn((
                    Smoke {
                        vel: air_vel * 0.6 + fwd * 15.0 + root.rotation * Vec3::new(2.0, 1.0, 0.0),
                        age: 0.0,
                        life: 0.55,
                        size0: 0.25,
                        size1: 1.3,
                        alpha0: 0.30,
                        mat: mat.clone(),
                    },
                    Mesh3d(assets.smoke_mesh.clone()),
                    MeshMaterial3d(mat),
                    Transform::from_translation(muzzle).with_scale(Vec3::splat(0.25)),
                    DespawnOnExit(AppState::Flying),
                ));
            }
        }
    }

    if let Some(key) = key_of(&bindings, ToggleAction::Flares) {
        if keys.just_pressed(key) && combat.flares > 0 {
            combat.flares -= 1;
            for i in 0..6 {
                let a = i as f32 / 6.0 * std::f32::consts::TAU;
                let side = Vec3::new(a.cos(), -0.6, a.sin()) * 12.0;
                let pos = root.translation + root.rotation * Vec3::new(0.0, -2.0, 1.0);
                commands.spawn((
                    Flare {
                        vel: air_vel + root.rotation * side - fwd * 20.0,
                        life: 2.5,
                    },
                    Mesh3d(assets.flare_mesh.clone()),
                    MeshMaterial3d(assets.flare_mat.clone()),
                    Transform::from_translation(pos),
                    DespawnOnExit(AppState::Flying),
                ));
            }
        }
    }
}

fn boom(commands: &mut Commands, assets: &CombatAssets, at: Vec3) {
    commands.spawn((
        Explosion {
            age: 0.0,
            life: 0.55,
        },
        Mesh3d(assets.boom_mesh.clone()),
        MeshMaterial3d(assets.boom_mat.clone()),
        Transform::from_translation(at).with_scale(Vec3::splat(2.0)),
        DespawnOnExit(AppState::Flying),
    ));
}

/// The plume rides a metre or so behind the tail and flickers while the motor
/// burns; the grain-out frame takes it away. Shared by both sides' rounds
/// (`width` is the x/y bead scale, `len_scale` stretches the z plume). Generic
/// over the query filter so each guiding system keeps its own `Without` set.
#[allow(clippy::too_many_arguments)]
fn service_flame<F: QueryFilter>(
    commands: &mut Commands,
    flames: &mut Query<&mut Transform, F>,
    slot: &mut Option<Entity>,
    burning: bool,
    tf: &Transform,
    dir: Vec3,
    now: f32,
    behind: f32,
    width: f32,
    len_scale: f32,
) {
    let Some(f) = *slot else { return };
    if burning {
        if let Ok(mut ft) = flames.get_mut(f) {
            let pulse = 1.0 + 0.16 * (now * 90.0).sin();
            ft.translation = tf.translation - dir * behind;
            ft.rotation = Quat::from_rotation_arc(Vec3::Z, -dir);
            ft.scale = Vec3::new(width, width, FLAME_LEN * len_scale * pulse);
        }
    } else {
        commands.entity(f).despawn();
        *slot = None;
    }
}

/// Guide the escorts' incoming rounds, seduce them with flares, and respawn the
/// pilot when one connects.
#[allow(clippy::too_many_arguments)]
fn guide_threats(
    mut commands: Commands,
    time: Res<Time>,
    state: Res<AircraftState>,
    assets: Option<Res<CombatAssets>>,
    mut sim: ResMut<Sim>,
    map: Res<ActiveMap>,
    mut combat: ResMut<Combat>,
    player: Query<
        &Transform,
        (
            With<AircraftRoot>,
            Without<MissileFlame>,
            Without<EnemyMissile>,
        ),
    >,
    mut threats: Query<
        (Entity, &mut EnemyMissile, &mut Transform),
        (Without<MissileFlame>, Without<Flare>),
    >,
    mut flames: Query<
        &mut Transform,
        (
            With<MissileFlame>,
            Without<EnemyMissile>,
            Without<Missile>,
            Without<Bandit>,
            Without<Flare>,
        ),
    >,
    flares: Query<
        (Entity, &Transform),
        (With<Flare>, Without<EnemyMissile>, Without<MissileFlame>),
    >,
) {
    let dt = time.delta_secs();
    combat.down = (combat.down - dt).max(0.0);
    let (Ok(player_tf), Some(assets)) = (player.single(), assets) else {
        return;
    };
    let player_pos = player_tf.translation;
    let player_vel = player_tf.rotation * Vec3::NEG_Z * (state.cas_kts * 0.514_444) as f32;
    // A hit clears every round in the air, including flames; remember them all
    // before the loop mutably borrows the query.
    let all: Vec<(Entity, Option<Entity>)> = threats.iter().map(|(e, m, _)| (e, m.flame)).collect();

    for (entity, mut m, mut tf) in &mut threats {
        m.life -= dt;
        if m.life <= 0.0 {
            despawn_flying(&mut commands, entity, &mut m.flame);
            continue;
        }

        // A flare that stays close for a moment steals the seeker.
        if m.decoy.is_none() {
            let mut nearest = None;
            let mut best = DECOY_RANGE_M;
            for (e, ftf) in &flares {
                let d = (ftf.translation - tf.translation).length();
                if d < best {
                    best = d;
                    nearest = Some(e);
                }
            }
            match nearest {
                Some(e) => {
                    m.flare_time += dt;
                    if m.flare_time >= DECOY_LOCK_S {
                        m.decoy = Some(e);
                    }
                }
                None => m.flare_time = (m.flare_time - dt * 2.0).max(0.0),
            }
        }

        let (target_pos, target_vel) = match m.decoy {
            Some(e) => match flares.get(e) {
                Ok((_, f)) => (f.translation, Vec3::ZERO),
                // It followed the decoy in; with the flare gone the round is
                // spent rather than circling back as an unkillable ghost.
                Err(_) => {
                    despawn_flying(&mut commands, entity, &mut m.flame);
                    continue;
                }
            },
            None => (player_pos, player_vel),
        };
        let pos = tf.translation;
        let speed = m.vel.length().max(1.0);
        let dir = lead_chase_dir(pos, m.vel, target_pos, target_vel, THREAT_TURN_RAD_S, dt);
        let accel = if m.motor > 0.0 {
            m.motor -= dt;
            THREAT_ACCEL
        } else {
            -12.0
        };
        let speed = (speed + accel * dt).clamp(90.0, THREAT_SPEED_MAX);
        m.vel = dir * speed;
        let next = pos + m.vel * dt;
        tf.translation = next;
        tf.rotation = Quat::from_rotation_arc(Vec3::NEG_Z, dir);
        let burning = m.motor > 0.0;
        service_flame(
            &mut commands,
            &mut flames,
            &mut m.flame,
            burning,
            &tf,
            dir,
            time.elapsed_secs(),
            1.6,
            0.45,
            0.8,
        );

        // A seduced round only kills flares; the jet is safe from it now.
        if m.decoy.is_none() && segment_point_distance(pos, next, player_pos) < THREAT_HIT_M {
            let at = player_pos;
            for (id, flame) in &all {
                if *id == entity {
                    continue;
                }
                commands.entity(*id).despawn();
                if let Some(f) = flame {
                    commands.entity(*f).despawn();
                }
            }
            despawn_flying(&mut commands, entity, &mut m.flame);
            boom(&mut commands, &assets, at);
            // Arcade respawn: a fresh jet at the mission spawn, score and wave
            // intact. The camera snaps over the next fixed step.
            let (alt, kts) = map
                .0
                .map(|m| (m.spawn_alt_ft(), m.spawn_kts()))
                .unwrap_or((9_000.0, 350.0));
            sim.reset_air(alt, kts);
            combat.lock = None;
            combat.weapon = Weapon::Ir;
            combat.missiles = 2;
            combat.bvr = AMRAAM_ROUNDS;
            combat.rounds = GUN_ROUNDS;
            combat.flares = 8;
            combat.bay = 0.0;
            combat.down = 3.0;
        }
    }
}

fn guide_missiles(
    mut commands: Commands,
    time: Res<Time>,
    assets: Option<Res<CombatAssets>>,
    mut combat: ResMut<Combat>,
    mut missiles: Query<(Entity, &mut Missile, &mut Transform), Without<Bandit>>,
    mut flames: Query<&mut Transform, (With<MissileFlame>, Without<Missile>, Without<Bandit>)>,
    bandits: Query<(&Bandit, &Transform), Without<Missile>>,
) {
    let Some(assets) = assets else {
        return;
    };
    let dt = time.delta_secs();
    for (entity, mut m, mut tf) in &mut missiles {
        m.life -= dt;
        if m.life <= 0.0 {
            despawn_flying(&mut commands, entity, &mut m.flame);
            continue;
        }
        let Ok((bandit, target_tf)) = bandits.get(m.target) else {
            despawn_flying(&mut commands, entity, &mut m.flame);
            continue;
        };
        let pos = tf.translation;
        let speed = m.vel.length().max(1.0);
        // Lead the target by the time the missile needs to close the gap.
        let dir = lead_chase_dir(pos, m.vel, target_tf.translation, bandit.vel, m.turn, dt);
        // Motor burn, then a little drag once the grain is gone.
        let accel = if m.motor > 0.0 {
            m.motor -= dt;
            m.accel
        } else {
            -8.0
        };
        let speed = (speed + accel * dt).clamp(80.0, m.max_speed);
        m.vel = dir * speed;
        let next = pos + m.vel * dt;
        tf.translation = next;
        tf.rotation = Quat::from_rotation_arc(Vec3::NEG_Z, dir);

        let burning = m.motor > 0.0;
        service_flame(
            &mut commands,
            &mut flames,
            &mut m.flame,
            burning,
            &tf,
            dir,
            time.elapsed_secs(),
            1.8,
            0.55,
            1.0,
        );

        if segment_point_distance(pos, next, target_tf.translation) < MISSILE_HIT_M {
            let at = target_tf.translation;
            despawn_flying(&mut commands, entity, &mut m.flame);
            commands.entity(m.target).despawn();
            boom(&mut commands, &assets, at);
            combat.kills += 1;
        }
    }
}

/// Despawn a round and its plume; both sides' rounds fly the same shape.
fn despawn_flying(commands: &mut Commands, entity: Entity, flame: &mut Option<Entity>) {
    commands.entity(entity).despawn();
    if let Some(f) = flame.take() {
        commands.entity(f).despawn();
    }
}

/// Lead-pursuit guidance: steer toward the closing time's predicted target
/// point, capped to `turn_cap` rad/s (same solve the player's missiles and the
/// threats both run).
fn lead_chase_dir(
    pos: Vec3,
    vel: Vec3,
    target: Vec3,
    target_vel: Vec3,
    turn_cap: f32,
    dt: f32,
) -> Vec3 {
    let speed = vel.length().max(1.0);
    let t_go = (target - pos).length() / speed;
    let predicted = target + target_vel * t_go;
    let desired = (predicted - pos).normalize_or_zero();
    let current = vel / speed;
    let angle = current.dot(desired).clamp(-1.0, 1.0).acos();
    if angle > 1e-4 {
        let axis = current.cross(desired).normalize_or_zero();
        let turn = (turn_cap * dt).min(angle);
        Quat::from_axis_angle(axis, turn) * current
    } else {
        current
    }
}

fn update_tracers(
    mut commands: Commands,
    time: Res<Time>,
    assets: Option<Res<CombatAssets>>,
    mut combat: ResMut<Combat>,
    mut tracers: Query<(Entity, &mut Tracer, &mut Transform), Without<Bandit>>,
    mut bandits: Query<(Entity, &mut Bandit, &Transform), Without<Tracer>>,
) {
    let Some(assets) = assets else {
        return;
    };
    let dt = time.delta_secs();
    for (entity, mut tracer, mut tf) in &mut tracers {
        tracer.life -= dt;
        tracer.prev = tf.translation;
        tf.translation += tracer.vel * dt;
        let mut hit = None;
        for (bandit_entity, mut bandit, bandit_tf) in &mut bandits {
            if bandit.hp <= 0 {
                continue;
            }
            if segment_point_distance(tracer.prev, tf.translation, bandit_tf.translation)
                < GUN_HIT_M
            {
                bandit.hp -= GUN_DAMAGE;
                hit = Some((bandit_entity, bandit.hp <= 0, bandit_tf.translation));
                break;
            }
        }
        if let Some((bandit_entity, dead, at)) = hit {
            commands.entity(entity).despawn();
            if dead {
                commands.entity(bandit_entity).despawn();
                boom(&mut commands, &assets, at);
                combat.kills += 1;
            }
        } else if tracer.life <= 0.0 {
            commands.entity(entity).despawn();
        }
    }
}

/// Distance from a point to a segment, for swept tracer hits.
fn segment_point_distance(a: Vec3, b: Vec3, p: Vec3) -> f32 {
    let ab = b - a;
    let t = if ab.length_squared() > 1e-6 {
        ((p - a).dot(ab) / ab.length_squared()).clamp(0.0, 1.0)
    } else {
        0.0
    };
    (a + ab * t - p).length()
}

fn update_effects(
    mut commands: Commands,
    time: Res<Time>,
    mut mats: ResMut<Assets<StandardMaterial>>,
    mut booms: Query<(Entity, &mut Explosion, &mut Transform), (Without<Flare>, Without<Smoke>)>,
    mut flares: Query<(Entity, &mut Flare, &mut Transform), (Without<Explosion>, Without<Smoke>)>,
    mut smokes: Query<(Entity, &mut Smoke, &mut Transform), (Without<Explosion>, Without<Flare>)>,
    mut flashes: Query<(Entity, &mut MuzzleFlash)>,
) {
    let dt = time.delta_secs();
    for (entity, mut flash) in &mut flashes {
        flash.life -= dt;
        if flash.life <= 0.0 {
            commands.entity(entity).despawn();
        }
    }
    for (entity, mut boom, mut tf) in &mut booms {
        boom.age += dt;
        if boom.age >= boom.life {
            commands.entity(entity).despawn();
            continue;
        }
        let k = boom.age / boom.life;
        tf.scale = Vec3::splat(2.0 + 26.0 * k);
    }
    for (entity, mut flare, mut tf) in &mut flares {
        flare.life -= dt;
        if flare.life <= 0.0 {
            commands.entity(entity).despawn();
            continue;
        }
        tf.translation += flare.vel * dt;
        flare.vel *= 0.98;
    }
    for (entity, mut smoke, mut tf) in &mut smokes {
        smoke.age += dt;
        if smoke.age >= smoke.life {
            commands.entity(entity).despawn();
            continue;
        }
        let k = (smoke.age / smoke.life).clamp(0.0, 1.0);
        tf.translation += smoke.vel * dt;
        smoke.vel *= (1.0 - 1.8 * dt).max(0.0);
        smoke.vel.y += 1.2 * dt;
        tf.scale = Vec3::splat(smoke.size0 + (smoke.size1 - smoke.size0) * k);
        if let Some(mut mat) = mats.get_mut(&smoke.mat) {
            let a = smoke.alpha0 * (1.0 - k) * (1.0 - k);
            mat.base_color = Color::srgba(0.62, 0.62, 0.65, a);
        }
    }
}

fn respawn_wave(
    mut commands: Commands,
    time: Res<Time>,
    sim: Res<Sim>,
    state: Res<AircraftState>,
    map: Res<ActiveMap>,
    root: Query<&Transform, With<AircraftRoot>>,
    bandits: Query<(), With<Bandit>>,
    assets: Option<Res<CombatAssets>>,
    mut combat: ResMut<Combat>,
) {
    if !combat.spawned || bandits.iter().next().is_some() {
        return;
    }
    if combat.respawn <= 0.0 {
        combat.respawn = WAVE_DELAY_S;
        return;
    }
    combat.respawn -= time.delta_secs();
    if combat.respawn > 0.0 {
        return;
    }
    combat.respawn = 0.0;
    let (Some(assets), Ok(root)) = (assets, root.single()) else {
        return;
    };
    if !combat_active(&sim, &state, &map) {
        return;
    }
    combat.wave += 1;
    spawn_wave(&mut commands, &assets, root);
}

/// Drive the four side-bay doors and the two launch rails from the eased bay
/// state. The door nodes' origins are the hinges, so a local rotation is enough;
/// no pivot math like the nozzles need.
fn drive_bays(
    time: Res<Time>,
    mut combat: ResMut<Combat>,
    mut nodes: Query<(&Name, &mut Transform)>,
) {
    let target = if combat.lock.is_some() { 1.0 } else { 0.0 };
    let step = BAY_RATE * time.delta_secs();
    combat.bay = if combat.bay < target {
        (combat.bay + step).min(target)
    } else {
        (combat.bay - step).max(target)
    };
    let travel = combat.bay * std::f32::consts::FRAC_PI_2;
    for (name, mut tf) in &mut nodes {
        if let Some((_, sign)) = DOORS.iter().find(|(n, _)| *n == name.as_str()) {
            tf.rotation = Quat::from_axis_angle(Vec3::Z, sign * travel);
        }
        if let Some((_, offset, angle)) = RAILS.iter().find(|(n, _, _)| *n == name.as_str()) {
            tf.translation = *offset * combat.bay;
            tf.rotation = Quat::from_axis_angle(Vec3::Z, angle * combat.bay);
        }
    }
}

fn update_hud(
    root: Query<&Transform, With<AircraftRoot>>,
    cams: Query<
        (&Camera, &GlobalTransform),
        (With<Camera3d>, Without<crate::cloud_camera::CloudCamera>),
    >,
    bandits: Query<(Entity, &Transform), With<Bandit>>,
    combat: Res<Combat>,
    mut weapons: ResMut<crate::hud::HudWeapons>,
    mut boxes: Query<(&TargetBox, &mut Node, &mut BorderColor)>,
    mut status: Query<&mut Text, (With<StatusText>, Without<LockText>, Without<ThreatText>)>,
    mut lock_text: Query<&mut Text, (With<LockText>, Without<StatusText>, Without<ThreatText>)>,
    mut threat_text: Query<&mut Text, (With<ThreatText>, Without<StatusText>, Without<LockText>)>,
) {
    // The F-22 HUD draws the stores block; a negative count hides it.
    if combat.spawned {
        weapons.ir = combat.missiles;
        weapons.bvr = combat.bvr;
        weapons.selected = match combat.weapon {
            Weapon::Ir => 0,
            Weapon::Bvr => 1,
        };
        weapons.rounds = combat.rounds;
        weapons.flares = combat.flares;
        weapons.kills = combat.kills;
    } else {
        weapons.ir = -1;
    }
    let Ok(root) = root.single() else {
        return;
    };
    let Ok((camera, cam_tf)) = cams.single() else {
        return;
    };
    let size = camera.logical_viewport_size().unwrap_or(Vec2::ZERO);
    let mut projected: Vec<(Entity, Vec2)> = bandits
        .iter()
        .filter_map(|(entity, tf)| {
            let p = camera.world_to_viewport(cam_tf, tf.translation).ok()?;
            (p.x > 0.0 && p.y > 0.0 && p.x < size.x && p.y < size.y).then_some((entity, p))
        })
        .collect();
    projected.sort_by_key(|(e, _)| e.index());
    let mut boxed: Vec<_> = boxes.iter_mut().collect();
    boxed.sort_by_key(|(b, _, _)| b.0);
    for (slot, (_, mut node, mut border)) in boxed.into_iter().enumerate() {
        match projected.get(slot) {
            Some((entity, p)) => {
                node.display = Display::Flex;
                node.left = Val::Px(p.x - 17.0);
                node.top = Val::Px(p.y - 17.0);
                let locked = combat.lock == Some(*entity);
                *border = BorderColor::all(if locked {
                    Color::srgb(1.0, 0.85, 0.25)
                } else {
                    Color::srgb(1.0, 0.25, 0.15)
                });
                node.border = UiRect::all(Val::Px(if locked { 3.0 } else { 2.0 }));
            }
            None => node.display = Display::None,
        }
    }

    if let Ok(mut text) = status.single_mut() {
        // Stores live on the HUD now; the bottom line is just the wave.
        text.0 = format!("WAVE {}", combat.wave);
    }
    if let Ok(mut text) = lock_text.single_mut() {
        text.0 = combat
            .lock
            .and_then(|e| bandits.get(e).ok())
            .map(|(_, tf)| {
                let d = (tf.translation - root.translation).length();
                format!("LOCK  {:.1} KM", d / 1000.0)
            })
            .unwrap_or_else(|| "NO LOCK".to_owned());
    }
    if let Ok(mut text) = threat_text.single_mut() {
        // The center banner is reserved for the respawn notice; inbound
        // missile warnings live on the screen edge (see `update_threat_marks`).
        let msg = if combat.down > 0.0 {
            "SHOT DOWN - RESPAWNED"
        } else {
            ""
        };
        if text.0 != msg {
            text.0 = msg.to_owned();
        }
    }
}

/// One edge marker per inbound missile: the arrow points toward its bearing and
/// the label carries the range. Off-screen and behind-the-jet threats collapse
/// onto the nearest edge so the player knows which way to break.
#[allow(clippy::too_many_arguments)]
fn update_threat_marks(
    cams: Query<
        (&Camera, &GlobalTransform),
        (With<Camera3d>, Without<crate::cloud_camera::CloudCamera>),
    >,
    player: Query<&Transform, (With<AircraftRoot>, Without<EnemyMissile>)>,
    threats: Query<(&Transform, &EnemyMissile)>,
    mut marks: Query<(&ThreatMark, &mut Node)>,
    mut glyphs: Query<(&ThreatGlyph, &mut UiTransform)>,
    mut dists: Query<(&ThreatDist, &mut Text)>,
) {
    let (Ok((camera, cam_tf)), Ok(player_tf)) = (cams.single(), player.single()) else {
        for (_, mut node) in &mut marks {
            node.display = Display::None;
        }
        return;
    };
    let Some(size) = camera.logical_viewport_size() else {
        for (_, mut node) in &mut marks {
            node.display = Display::None;
        }
        return;
    };
    if size.x < 2.0 || size.y < 2.0 {
        return;
    }
    let center = size * 0.5;
    let cam_rot = cam_tf.rotation();

    let mut inbound: Vec<(f32, Vec3)> = threats
        .iter()
        .filter(|(_, m)| m.decoy.is_none())
        .map(|(tf, _)| {
            (
                (tf.translation - player_tf.translation).length(),
                tf.translation,
            )
        })
        .collect();
    inbound.sort_by(|a, b| a.0.total_cmp(&b.0));

    let mut slots: Vec<_> = marks.iter_mut().collect();
    slots.sort_by_key(|(m, _)| m.0);
    let mut glyph_slots: Vec<_> = glyphs.iter_mut().collect();
    glyph_slots.sort_by_key(|(g, _)| g.0);
    let mut dist_slots: Vec<_> = dists.iter_mut().collect();
    dist_slots.sort_by_key(|(d, _)| d.0);

    for (slot, (_, mut node)) in slots.into_iter().enumerate() {
        let Some((dist, pos)) = inbound.get(slot) else {
            node.display = Display::None;
            continue;
        };
        let local = cam_rot.inverse() * (*pos - cam_tf.translation());
        let mut d = if local.z < 0.0 {
            // In front: the projection itself gives the screen bearing, even
            // when the round is outside the viewport.
            camera
                .world_to_viewport(cam_tf, *pos)
                .ok()
                .map(|p| p - center)
                .filter(|v| v.length_squared() > 1.0)
                .unwrap_or(Vec2::new(0.0, -1.0))
        } else {
            // Behind: fold it onto the bottom edge, left/right by bearing.
            Vec2::new(local.x, local.z.abs() + local.y * 0.3)
        };
        if d.length_squared() < 1e-4 {
            d = Vec2::new(0.0, 1.0);
        }
        let dir = d.normalize();
        let sx = (center.x - EDGE_X) / dir.x.abs().max(1e-4);
        let margin_y = if dir.y > 0.0 { EDGE_BOTTOM } else { EDGE_TOP };
        let sy = (center.y - margin_y) / dir.y.abs().max(1e-4);
        let edge = center + dir * sx.min(sy);
        node.display = Display::Flex;
        node.left = Val::Px(edge.x - MARK_W * 0.5);
        node.top = Val::Px(edge.y - MARK_H * 0.5);
        if let Some((_, tf)) = glyph_slots.get_mut(slot) {
            tf.rotation = Rot2::radians(dir.y.atan2(dir.x));
        }
        if let Some((_, text)) = dist_slots.get_mut(slot) {
            text.0 = range_label(*dist);
        }
    }
}

fn range_label(m: f32) -> String {
    if m >= 1000.0 {
        format!("{:.1}KM", m / 1000.0)
    } else {
        format!("{:.0}M", m)
    }
}

/// Where the bullets must meet the target: iterate the time of flight until the
/// lead point and the range agree. No gravity or dispersion in this gun, so
/// this is the whole fire-control solution.
fn lead_point(muzzle: Vec3, target: Vec3, target_vel: Vec3, bullet_speed: f32) -> Vec3 {
    let speed = bullet_speed.max(1.0);
    let mut t = (target - muzzle).length() / speed;
    for _ in 0..3 {
        let p = target + target_vel * t;
        t = (p - muzzle).length() / speed;
    }
    target + target_vel * t
}

fn hide_gun_pipper(pipper: &mut Query<(&GunPipper, &mut Node)>) {
    for (_, mut node) in pipper.iter_mut() {
        node.display = Display::None;
    }
}

/// Put the computed gun lead ring on the HUD. The ring is placed in the HUD's
/// own angular layout (the same px-per-degree the pitch ladder uses), so the
/// nose cross goes on it in the chase HUD; hide it outside gun range or the
/// HUD when there is no locked target.
fn update_gun_pipper(
    state: Res<AircraftState>,
    root: Query<&Transform, (With<AircraftRoot>, Without<Bandit>)>,
    cams: Query<
        (&Camera, &GlobalTransform),
        (With<Camera3d>, Without<crate::cloud_camera::CloudCamera>),
    >,
    bandits: Query<(&Transform, &Bandit)>,
    combat: Res<Combat>,
    mut pipper: Query<(&GunPipper, &mut Node)>,
) {
    let (Ok(root), Ok((camera, _))) = (root.single(), cams.single()) else {
        hide_gun_pipper(&mut pipper);
        return;
    };
    let Some(size) = camera.logical_viewport_size() else {
        hide_gun_pipper(&mut pipper);
        return;
    };
    let Some(target) = combat.lock else {
        hide_gun_pipper(&mut pipper);
        return;
    };
    let Ok((target_tf, bandit)) = bandits.get(target) else {
        hide_gun_pipper(&mut pipper);
        return;
    };
    let fwd = root.rotation * Vec3::NEG_Z;
    let air_vel = fwd * (state.cas_kts * 0.514_444) as f32;
    let muzzle = root.translation
        + fwd * GUN_PORT_AHEAD
        + root.rotation * Vec3::new(GUN_PORT_RIGHT, GUN_PORT_UP, 0.0);
    let bullet_speed = (air_vel + fwd * GUN_SPEED).length();
    if (target_tf.translation - muzzle).length() > bullet_speed * GUN_LIFE_S {
        hide_gun_pipper(&mut pipper);
        return;
    }
    let lead = lead_point(muzzle, target_tf.translation, bandit.vel, bullet_speed);
    // The HUD lays out angles with 0.0155 * height px per degree; place the
    // ring in that same frame so it sits with the symbology.
    let local = root.rotation.inverse() * (lead - root.translation);
    let az = local.x.atan2(-local.z).to_degrees();
    let el = local.y.atan2(local.x.hypot(local.z)).to_degrees();
    let ppd = size.y * 0.0155;
    let x = size.x * 0.5 + az * ppd;
    let y = size.y * 0.5 - el * ppd;
    if x < 24.0 || y < 24.0 || x > size.x - 24.0 || y > size.y - 24.0 {
        hide_gun_pipper(&mut pipper);
        return;
    }
    for (_, mut node) in &mut pipper {
        node.display = Display::Flex;
        node.left = Val::Px(x - 10.0);
        node.top = Val::Px(y - 10.0);
    }
}

/// Keep the corner tactical display in sync: heading-up, nearest contacts first
/// (so the pool only drops the farthest), locked contact gold.
fn update_radar(
    state: Res<AircraftState>,
    root: Query<&Transform, With<AircraftRoot>>,
    bandits: Query<(Entity, &Transform, &Bandit)>,
    combat: Res<Combat>,
    mut dots: Query<(&RadarDot, &mut Node, &mut BackgroundColor)>,
    mut label: Query<&mut Text, (With<RadarLabel>, Without<RadarInfo>)>,
    mut info: Query<&mut Text, (With<RadarInfo>, Without<RadarLabel>)>,
) {
    let Ok(root) = root.single() else {
        for (_, mut node, _) in &mut dots {
            node.display = Display::None;
        }
        for mut text in &mut label {
            text.0.clear();
        }
        for mut text in &mut info {
            text.0.clear();
        }
        return;
    };
    let range = combat.radar_range_m;
    let mut contacts: Vec<(Entity, Vec3, BanditRole)> = bandits
        .iter()
        .filter_map(|(entity, tf, bandit)| {
            let rel = tf.translation - root.translation;
            (rel.length() <= range).then_some((entity, rel, bandit.role))
        })
        .collect();
    contacts.sort_by(|a, b| a.1.length_squared().total_cmp(&b.1.length_squared()));

    let mut slots: Vec<_> = dots.iter_mut().collect();
    slots.sort_by_key(|(dot, _, _)| dot.0);
    let scale = RADAR_RADIUS_PX / range;
    for (slot, (_, mut node, mut bg)) in slots.into_iter().enumerate() {
        let Some((entity, rel, role)) = contacts.get(slot) else {
            node.display = Display::None;
            continue;
        };
        // Heading-up: +X is right, +Z is aft (down on the scope).
        let local = root.rotation.inverse() * *rel;
        let x = RADAR_CENTRE_PX + local.x * scale;
        let y = RADAR_CENTRE_PX + local.z * scale;
        node.display = Display::Flex;
        node.left = Val::Px(x - 4.0);
        node.top = Val::Px(y - 4.0);
        bg.0 = if combat.lock == Some(*entity) {
            Color::srgb(1.0, 0.85, 0.25)
        } else {
            match role {
                BanditRole::Escort => Color::srgb(1.0, 0.45, 0.10),
                BanditRole::Drone => Color::srgb(1.0, 0.25, 0.15),
            }
        };
    }

    if let Ok(mut text) = label.single_mut() {
        text.0 = format!("TSD {:.0}KM", range / 1000.0);
    }
    let player_vel = root.rotation * Vec3::NEG_Z * (state.cas_kts * 0.514_444) as f32;
    let telemetry = match combat.lock {
        Some(entity) => match bandits.get(entity) {
            Ok((_, tf, bandit)) => {
                let rel = tf.translation - root.translation;
                let dist = rel.length();
                let rel_vel = bandit.vel - player_vel;
                let closure = if dist > 1.0 {
                    -rel.dot(rel_vel) / dist
                } else {
                    0.0
                };
                let hdg = bandit
                    .vel
                    .x
                    .atan2(-bandit.vel.z)
                    .to_degrees()
                    .rem_euclid(360.0);
                let dalt = (tf.translation.y - root.translation.y) / 0.3048;
                format!(
                    "R {:.1}KM  VC {:+.0}KT\nHDG {:03.0}  ALT {:+.0}FT",
                    dist / 1000.0,
                    closure * 1.943_84,
                    hdg,
                    dalt
                )
            }
            Err(_) => "NO LOCK".to_owned(),
        },
        None => "NO LOCK".to_owned(),
    };
    if let Ok(mut text) = info.single_mut() {
        if text.0 != telemetry {
            text.0 = telemetry;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn lock_cone_and_range_gate() {
        let fwd = Vec3::NEG_Z;
        assert_eq!(angle_off(fwd, Vec3::NEG_Z).round(), 0.0);
        assert!(angle_off(fwd, Vec3::NEG_Z * 2.0 + Vec3::X).round() > 20.0);
        assert!(angle_off(fwd, Vec3::NEG_Z + Vec3::X * 0.9).round() <= 45.0);
    }

    #[test]
    fn segment_hit_catches_a_tracer_flying_past() {
        let a = Vec3::ZERO;
        let b = Vec3::new(0.0, 0.0, -100.0);
        assert!(segment_point_distance(a, b, Vec3::new(1.0, 1.0, -50.0)) < 2.0);
        assert!(segment_point_distance(a, b, Vec3::new(20.0, 0.0, -50.0)) > 10.0);
    }

    #[test]
    fn missile_lead_is_ahead_of_a_crossing_target() {
        let target = Vec3::new(0.0, 0.0, 1000.0);
        let target_vel = Vec3::new(200.0, 0.0, 0.0);
        let pos = Vec3::ZERO;
        let speed = 600.0;
        let t_go = (target - pos).length() / speed;
        let predicted = target + target_vel * t_go;
        assert!(predicted.x > target.x, "lead should be ahead in x");
    }

    #[test]
    fn escorts_only_shoot_inside_their_cone() {
        let fwd = Vec3::NEG_Z;
        assert!(can_escort_fire(fwd, Vec3::NEG_Z * 2000.0, 2000.0));
        // Outside the missile's useful range.
        assert!(!can_escort_fire(fwd, Vec3::NEG_Z * 3000.0, 3000.0));
        // Well off boresight.
        assert!(!can_escort_fire(
            fwd,
            Vec3::new(1000.0, 0.0, -100.0),
            1005.0
        ));
    }

    #[test]
    fn tail_shot_needs_the_escort_behind_the_player() {
        let player = Vec3::ZERO;
        let fwd = Vec3::NEG_Z;
        // +Z is behind the player's nose.
        assert!(behind_the_tail(player, fwd, Vec3::new(0.0, 0.0, 1500.0)));
        assert!(!behind_the_tail(player, fwd, Vec3::new(0.0, 0.0, -1500.0)));
    }

    #[test]
    fn gun_lead_leads_a_crossing_target() {
        let muzzle = Vec3::ZERO;
        let target = Vec3::new(0.0, 0.0, -600.0);
        let vel = Vec3::new(100.0, 0.0, 0.0);
        let lead = lead_point(muzzle, target, vel, 900.0);
        assert!(lead.x > target.x, "lead should be ahead in x");
        assert!(lead.z < 0.0, "lead stays down-range");
    }

    #[test]
    fn range_label_switches_at_a_kilometre() {
        assert_eq!(range_label(850.0), "850M");
        assert_eq!(range_label(1250.0), "1.2KM");
    }

    #[test]
    fn drones_turn_below_two_g() {
        let turn_rate = DRONE_SPEED / DRONE_RADIUS;
        let g = DRONE_SPEED * turn_rate / 9.81;
        assert!(g < 2.0, "drone turn is {g:.2} G");
    }

    #[test]
    fn launch_tone_is_a_valid_short_wav() {
        let wav = launch_tone_wav();
        assert_eq!(&wav[0..4], b"RIFF");
        assert_eq!(&wav[8..12], b"WAVE");
        assert_eq!(&wav[36..40], b"data");
        let data_len = u32::from_le_bytes(wav[40..44].try_into().unwrap()) as usize;
        assert_eq!(wav.len(), 44 + data_len);
        // Two ~85 ms pulses plus one gap: short enough to be a cue, not a siren.
        let seconds = data_len as f32 / (TONE_RATE as f32 * 2.0);
        assert!((0.2..0.35).contains(&seconds), "tone is {seconds:.3} s");
    }
}
