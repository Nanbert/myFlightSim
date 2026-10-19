//! The C172's home aerodrome: a flat, procedural airfield at the world origin.
//!
//! The runway runs north/south (36/18) along `-Z`, matching the FDM's heading
//! convention (`psi = 0` is north). `R` already parks the aircraft at the
//! origin at `rest_h_ft`, so the runway threshold is placed just ahead of it
//! and the takeoff roll runs north.
//!
//! Everything here is visual: the FDM still sees its flat ground plane at
//! `y = 0`. Pavement sits at `y = 0`, paint a few millimetres above it, and the
//! grass a few centimetres below, so the wheels do not sink into the scenery.

use bevy::prelude::*;

use crate::aircraft_profile;
use crate::app_state::AppState;
use crate::map::ActiveMap;
use crate::sim::{AircraftState, Sim, StartFlight};

/// Runway 36/18, metres.
const RUNWAY_LENGTH: f32 = 1200.0;
const RUNWAY_WIDTH: f32 = 30.0;
/// Threshold 36 sits well ahead of the aircraft's reset point (z = 0), so the
/// starting view is not dominated by the threshold bars.
const THRESHOLD_Z: f32 = 140.0;
const RUNWAY_CENTER_Z: f32 = THRESHOLD_Z - RUNWAY_LENGTH * 0.5;
const RUNWAY_END_Z: f32 = THRESHOLD_Z - RUNWAY_LENGTH;

/// Taxiway and apron, east of the runway.
const TAXI_X: f32 = 45.0;
const TAXI_Z0: f32 = THRESHOLD_Z - 6.0;
const TAXI_Z1: f32 = RUNWAY_END_Z + 36.0;
const APRON_X: f32 = 111.0;
const APRON_Z: f32 = -150.0;

/// PAPI units sit left of runway 36, 300 m down the runway from the threshold.
const PAPI_Z: f32 = THRESHOLD_Z - 300.0;
const PAPI_ANGLES: [f32; 4] = [3.5, 3.17, 2.83, 2.5];

const GRASS_Y: f32 = -0.04;
const PAVE_Y: f32 = 0.0;
const PAINT_Y: f32 = 0.015;

#[derive(Resource)]
struct PapiMats {
    red: Handle<StandardMaterial>,
    white: Handle<StandardMaterial>,
}

#[derive(Component)]
struct PapiLight {
    angle_deg: f32,
}

#[derive(Component)]
struct Windsock;

pub struct AerodromePlugin;

impl Plugin for AerodromePlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(
            OnEnter(AppState::Flying),
            spawn_aerodrome.after(StartFlight),
        )
        .add_systems(
            Update,
            (update_papi, update_windsock).run_if(in_state(AppState::Flying)),
        );
    }
}

fn spawn_aerodrome(
    mut commands: Commands,
    sim: Res<Sim>,
    map: Res<ActiveMap>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    // The airfield belongs to the default C172 scene. Missions force the F-22
    // and build their own terrain.
    if aircraft_profile::normalized(&sim.name) != "c172p" || map.0.is_some() {
        return;
    }

    let grass = materials.add(StandardMaterial {
        base_color: Color::srgb(0.13, 0.21, 0.11),
        perceptual_roughness: 1.0,
        ..default()
    });
    let asphalt = materials.add(StandardMaterial {
        base_color: Color::srgb(0.075, 0.082, 0.088),
        perceptual_roughness: 0.95,
        ..default()
    });
    let concrete = materials.add(StandardMaterial {
        base_color: Color::srgb(0.38, 0.38, 0.36),
        perceptual_roughness: 0.95,
        ..default()
    });
    let paint = materials.add(StandardMaterial {
        base_color: Color::srgb(0.92, 0.92, 0.86),
        perceptual_roughness: 0.8,
        ..default()
    });
    let yellow = materials.add(StandardMaterial {
        base_color: Color::srgb(0.86, 0.68, 0.12),
        perceptual_roughness: 0.8,
        ..default()
    });
    let metal = materials.add(StandardMaterial {
        base_color: Color::srgb(0.34, 0.37, 0.38),
        perceptual_roughness: 0.65,
        metallic: 0.35,
        ..default()
    });
    let roof = materials.add(StandardMaterial {
        base_color: Color::srgb(0.20, 0.22, 0.23),
        perceptual_roughness: 0.75,
        ..default()
    });
    let dark = materials.add(StandardMaterial {
        base_color: Color::srgb(0.06, 0.07, 0.08),
        perceptual_roughness: 0.9,
        ..default()
    });
    let orange = materials.add(StandardMaterial {
        base_color: Color::srgb(0.92, 0.38, 0.08),
        perceptual_roughness: 0.8,
        ..default()
    });
    let red = materials.add(StandardMaterial {
        base_color: Color::srgb(1.0, 0.08, 0.03),
        emissive: LinearRgba::rgb(4.0, 0.10, 0.02),
        ..default()
    });
    let white = materials.add(StandardMaterial {
        base_color: Color::srgb(1.0, 0.98, 0.90),
        emissive: LinearRgba::rgb(4.0, 3.6, 2.8),
        ..default()
    });
    let lamp = materials.add(StandardMaterial {
        base_color: Color::srgb(1.0, 0.85, 0.55),
        emissive: LinearRgba::rgb(3.0, 2.0, 0.8),
        ..default()
    });
    let green = materials.add(StandardMaterial {
        base_color: Color::srgb(0.2, 1.0, 0.35),
        emissive: LinearRgba::rgb(0.2, 3.0, 0.5),
        ..default()
    });

    // One unit plane, scaled per marking/pavement slab.
    let quad = meshes.add(Plane3d::default().mesh().size(1.0, 1.0));
    let slab = |commands: &mut Commands,
                mat: &Handle<StandardMaterial>,
                w: f32,
                l: f32,
                x: f32,
                z: f32,
                y: f32| {
        commands.spawn((
            Mesh3d(quad.clone()),
            MeshMaterial3d(mat.clone()),
            Transform::from_xyz(x, y, z).with_scale(Vec3::new(w, 1.0, l)),
            DespawnOnExit(AppState::Flying),
        ));
    };

    // Ground, runway, taxiway, apron. The grass is huge and hazed out; the
    // pavement sits exactly at the FDM ground plane so the wheels do not sink.
    slab(&mut commands, &grass, 50_000.0, 50_000.0, 0.0, 0.0, GRASS_Y);
    slab(
        &mut commands,
        &asphalt,
        RUNWAY_WIDTH,
        RUNWAY_LENGTH,
        0.0,
        RUNWAY_CENTER_Z,
        PAVE_Y,
    );
    slab(
        &mut commands,
        &asphalt,
        12.0,
        TAXI_Z0 - TAXI_Z1,
        TAXI_X,
        (TAXI_Z0 + TAXI_Z1) * 0.5,
        PAVE_Y + 0.004,
    );
    for z in [TAXI_Z0, TAXI_Z1, APRON_Z] {
        slab(
            &mut commands,
            &asphalt,
            TAXI_X - RUNWAY_WIDTH * 0.5,
            12.0,
            (TAXI_X + RUNWAY_WIDTH * 0.5) * 0.5,
            z,
            PAVE_Y + 0.003,
        );
    }
    slab(
        &mut commands,
        &concrete,
        120.0,
        160.0,
        APRON_X,
        APRON_Z,
        PAVE_Y + 0.005,
    );

    // --- markings -------------------------------------------------------
    // Runway edge lines.
    for x in [-(RUNWAY_WIDTH * 0.5 - 0.5), RUNWAY_WIDTH * 0.5 - 0.5] {
        slab(
            &mut commands,
            &paint,
            0.35,
            RUNWAY_LENGTH - 2.0,
            x,
            RUNWAY_CENTER_Z,
            PAINT_Y,
        );
    }
    // Centreline dashes. Start one dash-length clear of the reset point so the
    // spawn view does not open on a giant white bar under the nose.
    let mut z = THRESHOLD_Z - 30.0;
    while z > RUNWAY_END_Z + 10.0 {
        slab(&mut commands, &paint, 0.9, 9.0, 0.0, z, PAINT_Y);
        z -= 30.0;
    }
    // Threshold bars at both ends.
    for i in 0..8 {
        let x = -13.3 + i as f32 * 3.8;
        slab(
            &mut commands,
            &paint,
            1.4,
            20.0,
            x,
            THRESHOLD_Z - 10.0,
            PAINT_Y,
        );
        slab(
            &mut commands,
            &paint,
            1.4,
            20.0,
            x,
            RUNWAY_END_Z + 10.0,
            PAINT_Y,
        );
    }
    // Aiming points, 300 m in from each threshold.
    for (end, into) in [(THRESHOLD_Z, -1.0_f32), (RUNWAY_END_Z, 1.0_f32)] {
        for x in [-7.0, 7.0] {
            slab(
                &mut commands,
                &paint,
                6.0,
                45.0,
                x,
                end + into * 300.0,
                PAINT_Y,
            );
        }
    }
    // Touchdown-zone bars. Skip the 150 m row: at the reset point it sits right
    // under the wings and dominates the opening view.
    for (end, into) in [(THRESHOLD_Z, -1.0_f32), (RUNWAY_END_Z, 1.0_f32)] {
        for d in [450.0] {
            for x in [-4.5, 4.5] {
                slab(&mut commands, &paint, 3.0, 12.0, x, end + into * d, PAINT_Y);
            }
        }
    }
    // Taxiway centreline and connector lines.
    slab(
        &mut commands,
        &yellow,
        0.4,
        TAXI_Z0 - TAXI_Z1,
        TAXI_X,
        (TAXI_Z0 + TAXI_Z1) * 0.5,
        PAINT_Y + 0.002,
    );
    for z in [TAXI_Z0, TAXI_Z1, APRON_Z] {
        slab(
            &mut commands,
            &yellow,
            TAXI_X - RUNWAY_WIDTH * 0.5,
            0.4,
            (TAXI_X + RUNWAY_WIDTH * 0.5) * 0.5,
            z,
            PAINT_Y + 0.002,
        );
    }

    // --- hangar and buildings ------------------------------------------
    let box_mesh =
        |meshes: &mut Assets<Mesh>, w: f32, h: f32, d: f32| meshes.add(Cuboid::new(w, h, d));
    let hangar = box_mesh(&mut meshes, 40.0, 12.0, 30.0);
    let hangar_roof = box_mesh(&mut meshes, 44.0, 1.2, 34.0);
    let door = box_mesh(&mut meshes, 30.0, 8.0, 0.3);
    let small = box_mesh(&mut meshes, 18.0, 7.0, 16.0);
    let tower = box_mesh(&mut meshes, 5.0, 12.0, 5.0);
    let tower_cab = box_mesh(&mut meshes, 8.0, 3.0, 8.0);
    commands.spawn((
        Mesh3d(hangar),
        MeshMaterial3d(metal.clone()),
        Transform::from_xyz(120.0, 6.0, APRON_Z),
        DespawnOnExit(AppState::Flying),
    ));
    commands.spawn((
        Mesh3d(hangar_roof),
        MeshMaterial3d(roof.clone()),
        Transform::from_xyz(120.0, 12.6, APRON_Z),
        DespawnOnExit(AppState::Flying),
    ));
    commands.spawn((
        Mesh3d(door),
        MeshMaterial3d(dark.clone()),
        Transform::from_xyz(120.0, 4.0, APRON_Z + 15.2),
        DespawnOnExit(AppState::Flying),
    ));
    commands.spawn((
        Mesh3d(small),
        MeshMaterial3d(metal.clone()),
        Transform::from_xyz(80.0, 3.5, APRON_Z - 70.0),
        DespawnOnExit(AppState::Flying),
    ));
    commands.spawn((
        Mesh3d(tower),
        MeshMaterial3d(concrete.clone()),
        Transform::from_xyz(70.0, 6.0, APRON_Z - 110.0),
        DespawnOnExit(AppState::Flying),
    ));
    commands.spawn((
        Mesh3d(tower_cab),
        MeshMaterial3d(dark.clone()),
        Transform::from_xyz(70.0, 12.5, APRON_Z - 110.0),
        DespawnOnExit(AppState::Flying),
    ));

    // --- runway edge / threshold lights ---------------------------------
    let light_mesh = meshes.add(Cuboid::new(0.28, 0.28, 0.28));
    for x in [-16.2, 16.2] {
        let mut z = THRESHOLD_Z - 20.0;
        while z > RUNWAY_END_Z {
            commands.spawn((
                Mesh3d(light_mesh.clone()),
                MeshMaterial3d(lamp.clone()),
                Transform::from_xyz(x, 0.22, z),
                DespawnOnExit(AppState::Flying),
            ));
            z -= 120.0;
        }
    }
    for z in [THRESHOLD_Z, RUNWAY_END_Z] {
        for i in 0..5 {
            commands.spawn((
                Mesh3d(light_mesh.clone()),
                MeshMaterial3d(green.clone()),
                Transform::from_xyz(-12.0 + i as f32 * 6.0, 0.22, z),
                DespawnOnExit(AppState::Flying),
            ));
        }
    }

    // --- PAPI, left of runway 36 ---------------------------------------
    let papi_housing = meshes.add(Cuboid::new(0.9, 0.5, 0.5));
    let papi_lamp = meshes.add(Cuboid::new(0.7, 0.35, 0.12));
    for (i, angle) in PAPI_ANGLES.iter().enumerate() {
        let x = -30.0 - i as f32 * 9.0;
        commands.spawn((
            Mesh3d(papi_housing.clone()),
            MeshMaterial3d(dark.clone()),
            Transform::from_xyz(x, 0.25, PAPI_Z),
            DespawnOnExit(AppState::Flying),
        ));
        commands.spawn((
            PapiLight { angle_deg: *angle },
            Mesh3d(papi_lamp.clone()),
            MeshMaterial3d(red.clone()),
            Transform::from_xyz(x, 0.28, PAPI_Z + 0.30),
            DespawnOnExit(AppState::Flying),
        ));
    }
    commands.insert_resource(PapiMats { red, white });

    // --- windsock -------------------------------------------------------
    let pole = meshes.add(Cylinder::new(0.12, 6.0));
    let sock = meshes.add(Cone::new(0.50, 2.6));
    commands.spawn((
        Mesh3d(pole),
        MeshMaterial3d(metal.clone()),
        Transform::from_xyz(28.0, 3.0, -60.0),
        DespawnOnExit(AppState::Flying),
    ));
    commands.spawn((
        Windsock,
        Mesh3d(sock),
        MeshMaterial3d(orange),
        Transform::from_xyz(28.0, 6.2, -58.7)
            .with_rotation(Quat::from_rotation_x(std::f32::consts::FRAC_PI_2)),
        DespawnOnExit(AppState::Flying),
    ));
}

/// PAPI: white above the unit's angle, red below. On a 3-degree approach the
/// four units read 2 red / 2 white.
fn update_papi(
    state: Res<AircraftState>,
    mats: Res<PapiMats>,
    mut q: Query<(&PapiLight, &mut MeshMaterial3d<StandardMaterial>)>,
) {
    for (light, mut material) in &mut q {
        let distance = state.pos.z - PAPI_Z;
        let angle = if distance > 1.0 {
            (state.pos.y / distance).atan().to_degrees()
        } else {
            0.0
        };
        material.0 = if angle >= light.angle_deg {
            mats.white.clone()
        } else {
            mats.red.clone()
        };
    }
}

/// The windsock points downwind. The sim has no wind in the FDM yet, so it
/// hangs in an arbitrary light northerly and sways a little.
fn update_windsock(time: Res<Time>, mut q: Query<&mut Transform, With<Windsock>>) {
    let sway = (time.elapsed_secs() * 1.3).sin() * 0.06;
    let base = Quat::from_rotation_x(std::f32::consts::FRAC_PI_2);
    for mut tf in &mut q {
        tf.rotation = Quat::from_rotation_y(sway) * base;
    }
}

#[cfg(test)]
mod tests {
    use super::PAPI_ANGLES;

    fn white(angle: f32, threshold: f32) -> bool {
        angle >= threshold
    }

    #[test]
    fn papi_reads_two_white_two_red_on_a_three_degree_path() {
        let lights: Vec<bool> = PAPI_ANGLES.iter().map(|a| white(3.0, *a)).collect();
        assert_eq!(lights, [false, false, true, true]);
    }

    #[test]
    fn papi_goes_all_white_high_and_all_red_low() {
        assert!(PAPI_ANGLES.iter().all(|a| white(5.0, *a)));
        assert!(PAPI_ANGLES.iter().all(|a| !white(1.5, *a)));
    }
}
