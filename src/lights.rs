//! Exterior aircraft lighting for night ops (the C172, the one aircraft with
//! a real night-VFR checklist): red/green nav tips, a white tail light, a
//! flashing red beacon, double-flash wingtip strobes, and a landing spotlight.
//!
//! One switch, `L`, drives the whole pack. Without a manual toggle the pack
//! follows the time of day — on automatically whenever the sky is not DAY —
//! so a night spawn arrives with the airframe lit like a real rental that was
//! left ready. The mesh lamps are pure emissive (they read against the
//! aerodrome's own emissive runway lights); only the landing light carries a
//! real [`SpotLight`].
//!
//! Positions are in the `AircraftRoot` body frame (x forward, y right, z
//! down, metres from the CG), so each lamp sits in the airframe's painted
//! spot rather than being re-fitted per model.

use bevy::prelude::*;

use crate::app_state::{unpaused, AppState};
use crate::sim::Sim;
use crate::view::{spawn_flight_aircraft, AircraftRoot};
use crate::weather::Weather;

/// Flash timing, seconds. Beacon = one slow pulse; strobe = a double pop per
/// cycle.
const BEACON_PERIOD: f32 = 1.2;
const BEACON_ON: f32 = 0.16;
const STROBE_PERIOD: f32 = 1.4;
const STROBE_A: f32 = 0.05;
const STROBE_GAP: f32 = 0.11;
const STROBE_B: f32 = 0.05;

/// Lamp offsets. The `AircraftRoot`'s local frame is the body axes expressed
/// in Bevy terms (`sync_aircraft` sets `rot = S·R·Sᵀ`, so a local vector `v`
/// lands at body position `Sᵀ·v`; with the axis map in `sim.rs` that means
/// local x = right, local y = up, local z = aft). Constants below are written
/// body-first (fwd, right, down) then mapped.
const NAV_LEFT_BODY: (f32, f32, f32) = (0.1, -5.53, -0.5);
const NAV_RIGHT_BODY: (f32, f32, f32) = (0.1, 5.53, -0.5);
const NAV_TAIL_BODY: (f32, f32, f32) = (-2.55, 0.0, 0.43);
/// Beacon rides the fin top; strobes sit just inboard of the nav tips.
const BEACON_BODY: (f32, f32, f32) = (-2.2, 0.0, -1.3);
const STROBE_OFFSET_Y: f32 = 0.25;
/// The landing lamp hangs at the left wing root, aimed forward and a touch
/// down: beam direction in body terms, mapped the same way.
const LANDING_POS_BODY: (f32, f32, f32) = (0.35, -0.95, -0.55);
const LANDING_DIR_BODY: (f32, f32, f32) = (1.0, 0.0, 0.10);

/// Body (fwd, right, down) -> the root's local frame: local = (right, up,
/// aft). See the comment on the constants above for why.
fn body_to_local(b: (f32, f32, f32)) -> Vec3 {
    Vec3::new(b.1, -b.2, -b.0)
}

/// Exterior-lights switch state. `forced = None` follows the time of day; a
/// value from the `L` key overrides until the next flight resets it.
#[derive(Resource, Default)]
pub struct AircraftLights {
    forced: Option<bool>,
}

impl AircraftLights {
    /// Whether the light pack is lit right now: automatically on whenever
    /// the sky is not DAY (dusk and night both), the manual key overrides.
    pub fn effective(&self, weather: &Weather) -> bool {
        self.forced.unwrap_or_else(|| weather.night_factor() > 0.0)
    }

    /// Flip the pack, store the manual override, and return the new state.
    pub fn toggle(&mut self, weather: &Weather) -> bool {
        let v = !self.effective(weather);
        self.forced = Some(v);
        v
    }

    /// Drop the manual override: a fresh flight starts at its preset's default
    /// (off by day, on at dusk/night) instead of inheriting the last trip's
    /// key state.
    fn reset(&mut self) {
        self.forced = None;
    }
}

// --- markers ----------------------------------------------------------------

/// A steady lamp that just follows the switch: nav tips and the tail light.
#[derive(Component)]
struct Nav;

/// A self-timed lamp.
#[derive(Component)]
enum Flasher {
    Beacon,
    Strobe,
}

/// The landing-light spotlight (the pack's only real light source).
#[derive(Component)]
struct LandingSpot;

// --- spawn ------------------------------------------------------------------

/// Parent the whole pack under the flight's `AircraftRoot` so it rides the
/// body frame. C172 only: the other airframes keep their unlit night
/// silhouettes for now.
fn spawn_aircraft_lights(
    mut commands: Commands,
    sim: Res<Sim>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    roots: Query<Entity, With<AircraftRoot>>,
) {
    if sim.name != "c172p" {
        return;
    }
    let Ok(root) = roots.single() else {
        return;
    };
    let bulb = meshes.add(Sphere::new(0.08));
    let red = materials.add(StandardMaterial {
        base_color: Color::srgb(0.6, 0.04, 0.02),
        emissive: LinearRgba::rgb(5.0, 0.2, 0.06),
        ..default()
    });
    let green = materials.add(StandardMaterial {
        base_color: Color::srgb(0.03, 0.5, 0.1),
        emissive: LinearRgba::rgb(0.2, 5.0, 0.9),
        ..default()
    });
    let white = materials.add(StandardMaterial {
        base_color: Color::srgb(0.9, 0.9, 0.85),
        emissive: LinearRgba::rgb(4.5, 4.4, 4.0),
        ..default()
    });
    let beacon_mat = materials.add(StandardMaterial {
        base_color: Color::srgb(0.8, 0.05, 0.02),
        emissive: LinearRgba::rgb(6.0, 0.15, 0.05),
        ..default()
    });
    commands.entity(root).with_children(|parent| {
        // Nav tips and tail — always at the switch's mercy, no flash phase.
        parent.spawn((
            Mesh3d(bulb.clone()),
            MeshMaterial3d(red.clone()),
            Transform::from_translation(body_to_local(NAV_LEFT_BODY)),
            Nav,
            DespawnOnExit(AppState::Flying),
        ));
        parent.spawn((
            Mesh3d(bulb.clone()),
            MeshMaterial3d(green.clone()),
            Transform::from_translation(body_to_local(NAV_RIGHT_BODY)),
            Nav,
            DespawnOnExit(AppState::Flying),
        ));
        parent.spawn((
            Mesh3d(bulb.clone()),
            MeshMaterial3d(white.clone()),
            Transform::from_translation(body_to_local(NAV_TAIL_BODY)),
            Nav,
            DespawnOnExit(AppState::Flying),
        ));

        // Flashing pack: beacon on the fin top, strobes just inboard of the
        // nav tips.
        parent.spawn((
            Mesh3d(bulb.clone()),
            MeshMaterial3d(beacon_mat.clone()),
            Transform::from_translation(body_to_local(BEACON_BODY)),
            Flasher::Beacon,
            DespawnOnExit(AppState::Flying),
        ));
        for y in [
            NAV_LEFT_BODY.1 + STROBE_OFFSET_Y,
            NAV_RIGHT_BODY.1 - STROBE_OFFSET_Y,
        ] {
            let at = body_to_local((NAV_LEFT_BODY.0, y, NAV_LEFT_BODY.2));
            parent.spawn((
                Mesh3d(bulb.clone()),
                MeshMaterial3d(white.clone()),
                Transform::from_translation(at),
                Flasher::Strobe,
                DespawnOnExit(AppState::Flying),
            ));
        }

        // Landing light: a real spotlight beaming down the left wing line.
        parent.commands().spawn((
            SpotLight {
                // ~25k lumens, warm halogen. Shadows stay off: one light, no
                // need for the cost.
                color: Color::srgb(1.0, 0.95, 0.85),
                intensity: 25_000.0,
                inner_angle: (10.0f32).to_radians(),
                outer_angle: (17.0f32).to_radians(),
                range: 220.0,
                ..default()
            },
            Transform::from_translation(body_to_local(LANDING_POS_BODY)).with_rotation(
                Quat::from_rotation_arc(
                    Vec3::NEG_Z,
                    body_to_local(LANDING_DIR_BODY).normalize_or_zero(),
                ),
            ),
            LandingSpot,
            DespawnOnExit(AppState::Flying),
        ));
    });
}

// --- run --------------------------------------------------------------------

/// Steady lamps follow the switch every frame.
fn sync_steady_lights(
    lights: Res<AircraftLights>,
    weather: Res<Weather>,
    mut nav: Query<&mut Visibility, (With<Nav>, Without<LandingSpot>)>,
    mut spot: Query<&mut Visibility, With<LandingSpot>>,
) {
    let on = lights.effective(&weather);
    let visible = if on {
        Visibility::Visible
    } else {
        Visibility::Hidden
    };
    for mut v in &mut nav {
        *v = visible;
    }
    for mut v in &mut spot {
        *v = visible;
    }
}

/// Flashing lamps pulse on their own clocks while the pack is on.
fn sync_flashing_lights(
    lights: Res<AircraftLights>,
    weather: Res<Weather>,
    time: Res<Time>,
    mut flashers: Query<(&mut Visibility, &Flasher)>,
) {
    let on = lights.effective(&weather);
    let t = time.elapsed_secs();
    for (mut v, flasher) in &mut flashers {
        // Wrap negatives so a `t` before zero (shouldn't happen) stays sane.
        let phase = t.rem_euclid(match flasher {
            Flasher::Beacon => BEACON_PERIOD,
            Flasher::Strobe => STROBE_PERIOD,
        });
        *v = if !on {
            Visibility::Hidden
        } else {
            match flasher {
                Flasher::Beacon => {
                    if phase < BEACON_ON {
                        Visibility::Visible
                    } else {
                        Visibility::Hidden
                    }
                }
                // Two quick pops: [0, A] and [GAP, GAP + B].
                Flasher::Strobe => {
                    let second = phase - STROBE_GAP;
                    if phase < STROBE_A || (0.0..STROBE_B).contains(&second) {
                        Visibility::Visible
                    } else {
                        Visibility::Hidden
                    }
                }
            }
        };
    }
}

/// Clear the manual override so a new flight starts at the preset default.
fn reset_toggle(mut lights: ResMut<AircraftLights>) {
    lights.reset();
}

pub struct LightsPlugin;

impl Plugin for LightsPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<AircraftLights>()
            // `spawn_flight_aircraft` must have created the `AircraftRoot`
            // first; the pack rides under it.
            .add_systems(
                OnEnter(AppState::Flying),
                (
                    reset_toggle,
                    spawn_aircraft_lights.after(spawn_flight_aircraft),
                ),
            )
            .add_systems(
                Update,
                (sync_steady_lights, sync_flashing_lights).run_if(unpaused),
            );
    }
}
