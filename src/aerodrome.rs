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
use crate::app_state::{unpaused, AppState};
use crate::map::ActiveMap;
use crate::sim::{AircraftState, Sim, StartFlight};

/// Runway 36/18, metres.
pub const RUNWAY_LENGTH: f32 = 1200.0;
pub const RUNWAY_WIDTH: f32 = 30.0;
/// Threshold 36 sits well ahead of the aircraft's reset point (z = 0), so the
/// starting view is not dominated by the threshold bars.
pub const THRESHOLD_Z: f32 = 140.0;
const RUNWAY_CENTER_Z: f32 = THRESHOLD_Z - RUNWAY_LENGTH * 0.5;
pub const RUNWAY_END_Z: f32 = THRESHOLD_Z - RUNWAY_LENGTH;

/// Taxiway and apron, east of the runway.
const TAXI_X: f32 = 45.0;
const TAXI_Z0: f32 = THRESHOLD_Z - 6.0;
const TAXI_Z1: f32 = RUNWAY_END_Z + 36.0;
const APRON_X: f32 = 111.0;
const APRON_Z: f32 = -150.0;
/// The C172 ramp stand on the lead-in line: the `P` spawn parks at (75, -150)
/// facing the taxiway. Far enough from the pump that spawning cold doesn't
/// auto-fill the tanks, aligned with the connector at the apron's own level.
pub const STAND_X: f32 = 75.0;
/// The lead-in line runs along the apron's connector (the z = -150 stub).
pub const STAND_Z: f32 = APRON_Z;
/// Where the refuelling lane leaves the lead-in line for the pump.
const PUMP_LANE_X: f32 = PUMP_POS.x;

/// PAPI units sit left of runway 36, 300 m down the runway from the threshold.
const PAPI_Z: f32 = THRESHOLD_Z - 300.0;
/// And the reciprocal set, 300 m in from the 18 threshold.
const PAPI_18_Z: f32 = RUNWAY_END_Z + 300.0;
const PAPI_ANGLES: [f32; 4] = [3.5, 3.17, 2.83, 2.5];

#[derive(Clone, Copy, PartialEq, Eq)]
enum PapiSide {
    /// Runway 36: approach from +Z.
    South,
    /// Runway 18: approach from -Z.
    North,
}

#[derive(Component)]
struct PapiLight {
    angle_deg: f32,
    side: PapiSide,
    z: f32,
}

const GRASS_Y: f32 = -0.04;
const PAVE_Y: f32 = 0.0;
const PAINT_Y: f32 = 0.015;

#[derive(Resource)]
struct PapiMats {
    red: Handle<StandardMaterial>,
    white: Handle<StandardMaterial>,
}

#[derive(Component)]
struct Windsock;

/// Segment layout for the painted threshold numbers: `(dx, dy, horizontal)`,
/// in *half-cell* units, in a paper frame where +Y runs to the top of the
/// glyph and +X to the reader's right.
const SEGS: [(f32, f32, bool); 7] = [
    (0.0, 1.0, true),    // A: top bar
    (1.0, 0.5, false),   // B: upper right
    (1.0, -0.5, false),  // C: lower right
    (0.0, -1.0, true),   // D: bottom bar
    (-1.0, -0.5, false), // E: lower left
    (-1.0, 0.5, false),  // F: upper left
    (0.0, 0.0, true),    // G: middle bar
];

/// Which segments make up each digit the thresholds need.
fn digit_segs(d: char) -> &'static [usize] {
    match d {
        '0' => &[0, 1, 2, 3, 4, 5],
        '1' => &[1, 2],
        '3' => &[0, 1, 2, 3, 6],
        '6' => &[0, 2, 3, 4, 5, 6],
        '8' => &[0, 1, 2, 3, 4, 5, 6],
        _ => &[],
    }
}

/// Paint a two-digit runway designator as painted strokes. `mx`/`mz` map the
/// paper frame into the world: 36 reads for a northbound pilot (paper right is
/// world east, top of the glyph points north), 18 for a southbound one.
#[allow(clippy::too_many_arguments)]
fn paint_designator(
    commands: &mut Commands,
    quad: &Handle<Mesh>,
    paint: &Handle<StandardMaterial>,
    text: &str,
    cx: f32,
    cz: f32,
    mx: f32,
    mz: f32,
) {
    let cell_w = 5.6_f32;
    let cell_h = 12.0_f32;
    let seg = 1.1_f32;
    let (half_w, half_h) = (cell_w * 0.5, cell_h * 0.5);
    let step = cell_w + 1.2;
    let y = PAINT_Y + 0.002;
    let chars: Vec<char> = text.chars().collect();
    let n = chars.len() as f32;
    for (i, d) in chars.iter().enumerate() {
        // The '1' is a single right-side stroke; recentre that digit.
        let mut px = (i as f32 - (n - 1.0) * 0.5) * step;
        if *d == '1' {
            px -= half_w - seg * 0.5;
        }
        for &si in digit_segs(*d) {
            let (dx, dy, horiz) = SEGS[si];
            let sx = cx + px + dx * half_w * mx;
            let sz = cz + dy * half_h * mz;
            let (w, l) = if horiz { (cell_w, seg) } else { (seg, half_h) };
            commands.spawn((
                Mesh3d(quad.clone()),
                MeshMaterial3d(paint.clone()),
                Transform::from_xyz(sx, y, sz).with_scale(Vec3::new(w, 1.0, l)),
                DespawnOnExit(AppState::Flying),
            ));
        }
    }
}

pub struct AerodromePlugin;

impl Plugin for AerodromePlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(
            OnEnter(AppState::Flying),
            spawn_aerodrome.after(StartFlight),
        )
        .add_systems(
            Update,
            (update_papi, update_windsock).run_if(unpaused),
        )
        .add_systems(FixedUpdate, update_pump.run_if(unpaused));
    }
}

/// Marker on the apron's fuel pump entity.
#[derive(Component)]
struct FuelPump;

/// Where the pump stands (world m), and the parked-capture radius around it.
const PUMP_POS: Vec3 = Vec3::new(90.0, 1.0, APRON_Z - 40.0);
const PUMP_RADIUS_M: f64 = 12.0;

/// Stopping on the apron beside the pump tops the C172's tanks back up. Pure
/// convenience with honest framing: the pump only moves fuel, never an engine
/// state — a refuelled-but-stopped engine still needs the M/I start flow.
fn update_pump(mut sim: ResMut<Sim>) {
    // Only the Rust-backend C172 has a fuel model to fill.
    if sim.model.get("propulsion/tank-capacity-lbs") <= 0.0 {
        return;
    }
    let (pos, _, _) = sim.pose();
    let dx = (pos.x - PUMP_POS.x) as f64;
    let dz = (pos.z - PUMP_POS.z) as f64;
    // Actually parked: weight on the wheels and essentially stopped.
    let parked = sim.model.get("gear/wow") > 0.5 && sim.model.get("velocities/vg-fps") < 3.4;
    if dx * dx + dz * dz <= PUMP_RADIUS_M * PUMP_RADIUS_M && parked {
        sim.model.set("propulsion/refuel-cmd", 1.0);
    }
}

fn spawn_aerodrome(
    mut commands: Commands,
    sim: Res<Sim>,
    map: Res<ActiveMap>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    // The airfield belongs to the default scene: the C172's home club and the
    // 737's (short but usable) practice runway. Missions force the F-22
    // and build their own terrain.
    let n = aircraft_profile::normalized(&sim.name);
    if !matches!(n, "c172p" | "737") || map.0.is_some() {
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
    // Taxiway edge lights: quiet blue, a step dimmer than the runway stuff.
    let blue = materials.add(StandardMaterial {
        base_color: Color::srgb(0.2, 0.5, 1.0),
        emissive: LinearRgba::rgb(0.25, 0.9, 3.4),
        ..default()
    });
    // Threshold bars run a step dimmer than the beacon green: from five miles
    // out the five-bulb bar still blooms into one blob, just a smaller one.
    let green_dim = materials.add(StandardMaterial {
        base_color: Color::srgb(0.2, 1.0, 0.35),
        emissive: LinearRgba::rgb(0.15, 2.0, 0.4),
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
    // Threshold designators: "36" just past the north bars, the reciprocal
    // "18" just past the south ones, both reading toward their approach.
    paint_designator(
        &mut commands,
        &quad,
        &paint,
        "36",
        0.0,
        THRESHOLD_Z - 16.0,
        1.0,
        -1.0,
    );
    paint_designator(
        &mut commands,
        &quad,
        &paint,
        "18",
        0.0,
        RUNWAY_END_Z + 16.0,
        -1.0,
        1.0,
    );
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
    // Apron taxi guidance: a lead-in line along z = -150 from the pump lane
    // west to the taxiway centreline, a stub lane from the pump north onto
    // it, and a stop bar where the C172's ramp stand sits (the `P` spawn).
    // The blue edge lights and this one yellow line make a dark-apron taxi a
    // matter of following the paint.
    slab(
        &mut commands,
        &yellow,
        PUMP_LANE_X - TAXI_X,
        0.4,
        (PUMP_LANE_X + TAXI_X) * 0.5,
        STAND_Z,
        PAINT_Y + 0.002,
    );
    slab(
        &mut commands,
        &yellow,
        0.4,
        PUMP_POS.z - STAND_Z,
        PUMP_LANE_X,
        (PUMP_POS.z + STAND_Z) * 0.5,
        PAINT_Y + 0.002,
    );
    // The stand's stop bar: taxi until the tail crosses it, then stop.
    slab(
        &mut commands,
        &yellow,
        0.4,
        3.0,
        STAND_X,
        STAND_Z,
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
        // Runway holding position: two solid lines, then two dashed lines.
        for x in [18.0, 19.5] {
            slab(&mut commands, &yellow, 0.3, 12.0, x, z, PAINT_Y + 0.004);
        }
        for x in [22.0, 23.5] {
            for dz in [-4.0, 0.0, 4.0] {
                slab(&mut commands, &yellow, 0.3, 3.0, x, z + dz, PAINT_Y + 0.004);
            }
        }
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

    // A fuel pump on the apron. Stopping on the apron next to it refuels the
    // C172 automatically (see `update_pump`).
    let pump = box_mesh(&mut meshes, 1.2, 2.0, 0.9);
    commands.spawn((
        Mesh3d(pump),
        MeshMaterial3d(orange.clone()),
        Transform::from_xyz(PUMP_POS.x, PUMP_POS.y, PUMP_POS.z),
        FuelPump,
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
                MeshMaterial3d(green_dim.clone()),
                Transform::from_xyz(-12.0 + i as f32 * 6.0, 0.22, z),
                DespawnOnExit(AppState::Flying),
            ));
        }
    }

    // Red runway end bars, just outside each end: whichever direction is in
    // use, the landing pilot sees green at the threshold and red where the
    // pavement gives out.
    for z in [THRESHOLD_Z + 4.0, RUNWAY_END_Z - 4.0] {
        for i in 0..5 {
            commands.spawn((
                Mesh3d(light_mesh.clone()),
                MeshMaterial3d(red.clone()),
                Transform::from_xyz(-12.0 + i as f32 * 6.0, 0.22, z),
                DespawnOnExit(AppState::Flying),
            ));
        }
    }
    // Blue taxiway edge lights: along the parallel strip and the two stubs
    // the C172 actually taxies (threshold connector and apron connector).
    for x in [TAXI_X - 7.0, TAXI_X + 7.0] {
        let mut z = TAXI_Z1 + 4.0;
        while z < TAXI_Z0 {
            commands.spawn((
                Mesh3d(light_mesh.clone()),
                MeshMaterial3d(blue.clone()),
                Transform::from_xyz(x, 0.20, z),
                DespawnOnExit(AppState::Flying),
            ));
            z += 65.0;
        }
    }
    for z in [TAXI_Z0, APRON_Z] {
        for x in [18.0, 27.0, 36.0, TAXI_X + 7.0] {
            for dz in [-6.6, 6.6] {
                commands.spawn((
                    Mesh3d(light_mesh.clone()),
                    MeshMaterial3d(blue.clone()),
                    Transform::from_xyz(x, 0.20, z + dz),
                    DespawnOnExit(AppState::Flying),
                ));
            }
        }
    }

    // Apron floodlights: two warm point lights over the parking area so a
    // night C172 can find its parking spot (and the fuel pump) by itself.
    let pole_mesh = meshes.add(Cuboid::new(0.15, 5.5, 0.15));
    let head_mesh = meshes.add(Cuboid::new(0.8, 0.35, 0.8));
    let flood_mat = materials.add(StandardMaterial {
        base_color: Color::srgb(1.0, 0.9, 0.7),
        emissive: LinearRgba::rgb(6.0, 5.0, 3.2),
        ..default()
    });
    for z in [APRON_Z + 30.0, APRON_Z - 60.0] {
        commands.spawn((
            Mesh3d(pole_mesh.clone()),
            MeshMaterial3d(concrete.clone()),
            Transform::from_xyz(100.0, 2.75, z),
            DespawnOnExit(AppState::Flying),
        ));
        commands.spawn((
            Mesh3d(head_mesh.clone()),
            MeshMaterial3d(flood_mat.clone()),
            Transform::from_xyz(100.0, 5.6, z),
            DespawnOnExit(AppState::Flying),
        ));
        commands.spawn((
            PointLight {
                color: Color::srgb(1.0, 0.92, 0.78),
                intensity: 14_000.0,
                range: 80.0,
                ..default()
            },
            Transform::from_xyz(100.0, 5.3, z),
        ));
    }

    // Tower beacon: a steady green-white pair on the mast, so the field has a
    // landmark that survives the dark.
    let beacon_mesh = meshes.add(Sphere::new(0.2));
    for (i, mat) in [green.clone(), white.clone()].into_iter().enumerate() {
        commands.spawn((
            Mesh3d(beacon_mesh.clone()),
            MeshMaterial3d(mat),
            Transform::from_xyz(70.0, 14.0 + i as f32 * 0.5, APRON_Z - 110.0),
            DespawnOnExit(AppState::Flying),
        ));
    }

    // --- PAPI, both directions ------------------------------------------
    let papi_housing = meshes.add(Cuboid::new(0.9, 0.5, 0.5));
    let papi_lamp = meshes.add(Cuboid::new(0.7, 0.35, 0.12));
    for (i, angle) in PAPI_ANGLES.iter().enumerate() {
        // Runway 36: left of the approach is -X; the lamps face +Z.
        let x = -30.0 - i as f32 * 9.0;
        commands.spawn((
            Mesh3d(papi_housing.clone()),
            MeshMaterial3d(dark.clone()),
            Transform::from_xyz(x, 0.25, PAPI_Z),
            DespawnOnExit(AppState::Flying),
        ));
        commands.spawn((
            PapiLight {
                angle_deg: *angle,
                side: PapiSide::South,
                z: PAPI_Z,
            },
            Mesh3d(papi_lamp.clone()),
            MeshMaterial3d(red.clone()),
            Transform::from_xyz(x, 0.28, PAPI_Z + 0.30),
            DespawnOnExit(AppState::Flying),
        ));
    }
    for (i, angle) in PAPI_ANGLES.iter().enumerate() {
        // Runway 18: left of the approach is +X; the lamps face -Z.
        let x = 30.0 + (3 - i) as f32 * 9.0;
        commands.spawn((
            Mesh3d(papi_housing.clone()),
            MeshMaterial3d(dark.clone()),
            Transform::from_xyz(x, 0.25, PAPI_18_Z),
            DespawnOnExit(AppState::Flying),
        ));
        commands.spawn((
            PapiLight {
                angle_deg: *angle,
                side: PapiSide::North,
                z: PAPI_18_Z,
            },
            Mesh3d(papi_lamp.clone()),
            MeshMaterial3d(red.clone()),
            Transform::from_xyz(x, 0.28, PAPI_18_Z - 0.30),
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

/// Distance from a PAPI unit to the aircraft, measured along the approach.
fn approach_distance(side: PapiSide, light_z: f32, aircraft_z: f32) -> f32 {
    match side {
        PapiSide::South => aircraft_z - light_z,
        PapiSide::North => light_z - aircraft_z,
    }
}

/// PAPI: white above the unit's angle, red below. On a 3-degree approach the
/// four units read 2 red / 2 white.
///
/// Only the home airfield builds the field (`spawn_aerodrome` early-returns
/// for the F-22 and the missions, which lay down their own terrain), and with
/// it the PAPI paints they recolour — so the mats arrive behind an `Option`
/// and a flight without an airfield simply skips the pass instead of
/// panicking its first frame in it.
fn update_papi(
    state: Res<AircraftState>,
    mats: Option<Res<PapiMats>>,
    mut q: Query<(&PapiLight, &mut MeshMaterial3d<StandardMaterial>)>,
) {
    let Some(mats) = mats else {
        return;
    };
    for (light, mut material) in &mut q {
        let distance = approach_distance(light.side, light.z, state.pos.z);
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

/// The windsock reads the weather's wind: it swings round to lie downwind and
/// its droop shrinks as the wind strengthens (calm hangs limp, 15 kt lies it
/// flat). The FDM gets the same field through the property bus, so what the
/// pilot sees on the ground is what the aircraft feels.
fn update_windsock(
    time: Res<Time>,
    weather: Res<crate::weather::Weather>,
    mut q: Query<&mut Transform, With<Windsock>>,
) {
    let sway = (time.elapsed_secs() * 1.3).sin() * 0.05;
    //_mount: align + droop + sway, ordering matters for the droop axis
    let toward = weather.wind_toward_deg().to_radians();
    let droop = (std::f32::consts::FRAC_PI_2 - weather.wind_kts * 0.11)
        .clamp(0.0, std::f32::consts::FRAC_PI_2);
    let base = Quat::from_rotation_x(std::f32::consts::FRAC_PI_2);
    let align = Quat::from_rotation_y(std::f32::consts::PI - toward);
    let hang = Quat::from_rotation_x(droop);
    let sway_q = Quat::from_rotation_y(sway);
    let rotation = align * hang * sway_q * base;
    // Keep the wide mouth anchored at the pole top while the tail swings.
    let tip = rotation * Vec3::Y;
    let anchor = Vec3::new(28.0, 6.2, -60.0);
    for mut tf in &mut q {
        tf.rotation = rotation;
        tf.translation = anchor + tip * 1.3;
    }
}

/// Distance from the selected threshold, along the runway, in metres. `heading`
/// is the aircraft's true heading: northbound uses the 36 threshold, southbound
/// uses the 18 threshold.
pub fn runway_along(heading_deg: f32, z: f32) -> f32 {
    if heading_deg.to_radians().cos() >= 0.0 {
        THRESHOLD_Z - z
    } else {
        z - RUNWAY_END_Z
    }
}

/// Lateral distance from the centreline, metres.
pub fn runway_lateral(x: f32) -> f32 {
    x.abs()
}

/// Whether a world point is on the paved surface (with a small tolerance).
pub fn on_runway(x: f32, z: f32) -> bool {
    runway_lateral(x) <= RUNWAY_WIDTH * 0.5 + 1.0
        && (RUNWAY_END_Z - 1.0..=THRESHOLD_Z + 1.0).contains(&z)
}

#[cfg(test)]
mod tests {
    use super::{
        approach_distance, on_runway, runway_along, PapiSide, PAPI_ANGLES, RUNWAY_END_Z,
        THRESHOLD_Z,
    };

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

    #[test]
    fn runway_along_uses_the_active_threshold() {
        // Runway 36: threshold at +THRESHOLD_Z, aiming point 300 m in.
        assert!((runway_along(0.0, THRESHOLD_Z) - 0.0).abs() < 1e-3);
        assert!((runway_along(0.0, THRESHOLD_Z - 300.0) - 300.0).abs() < 1e-3);
        // Runway 18: threshold at RUNWAY_END_Z, aiming point 300 m in.
        assert!((runway_along(180.0, RUNWAY_END_Z) - 0.0).abs() < 1e-3);
        assert!((runway_along(180.0, RUNWAY_END_Z + 300.0) - 300.0).abs() < 1e-3);
    }

    #[test]
    fn runway_contains_the_pavement_and_rejects_the_grass() {
        assert!(on_runway(0.0, 0.0));
        assert!(on_runway(14.0, -300.0));
        assert!(!on_runway(20.0, -300.0));
        assert!(!on_runway(0.0, THRESHOLD_Z + 10.0));
    }

    #[test]
    fn papi_distance_is_positive_on_the_approach_side() {
        // Runway 36: aircraft south of the 36 PAPI.
        assert!(approach_distance(PapiSide::South, -160.0, 1000.0) > 0.0);
        // Runway 18: aircraft north of the 18 PAPI.
        assert!(approach_distance(PapiSide::North, -760.0, -2000.0) > 0.0);
    }
}
