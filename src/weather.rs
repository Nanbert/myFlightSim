//! Weather: rain, plus the overcast level that goes with it.
//!
//! Rain is a pool of thin streak quads that live in a box centred on the camera.
//! Each frame they fall (with wind drift) and are wrapped back to the top of the
//! box, so a fixed pool reads as an endless field of rain with no spawning — the
//! same recycling trick the sky dome uses for clouds. The streaks billboard
//! about the vertical axis so they always face the viewer.
//!
//! Only the *state* lives here. The things that react to it — the sky dome
//! (`sky.rs`) and the scene lights (`view.rs`) — own those assets themselves and
//! just read [`Weather`], so there is no back-dependency.

use bevy::prelude::*;

/// Rain intensities cycled by `T`. Index 0 is clear; the values double as the
/// overcast levels `sky.rs` pre-bakes its texture at.
pub const RAIN_LEVELS: [f32; 4] = [0.0, 0.3, 0.6, 0.9];

/// Current weather. `rain` is 0 (clear) … 1 (downpour).
#[derive(Resource)]
pub struct Weather {
    pub rain: f32,
}

impl Default for Weather {
    fn default() -> Self {
        Weather { rain: 0.0 }
    }
}

impl Weather {
    /// Short HUD label for the current intensity.
    pub fn label(&self) -> &'static str {
        match self.rain {
            r if r < 0.05 => "CLEAR",
            r if r < 0.45 => "LIGHT RAIN",
            r if r < 0.75 => "RAIN",
            _ => "STORM",
        }
    }
}

/// Number of streaks in the pool (all shown at full intensity).
const DROPS: usize = 4500;
/// Half-extents of the rain box around the camera: `BOX_XZ` horizontally,
/// `BOX_Y` vertically. Drops wrap within it.
const BOX_XZ: f32 = 70.0;
const BOX_Y: f32 = 45.0;
/// Fall speed (m/s) at `speed = 1.0`, and wind drift (m/s).
const FALL: f32 = 34.0;
const WIND: f32 = 6.0;
/// Streak quad size (m): a thin, tall sliver.
const STREAK_W: f32 = 0.035;
const STREAK_H: f32 = 0.8;

/// Ground splashes: ripples where the rain strikes the ground plane.
const SPLASHES: usize = 260;
/// Ring radius (m) at birth and at death; it expands over one lifetime.
const SPLASH_R0: f32 = 0.08;
const SPLASH_R1: f32 = 0.55;
/// Ripple lifetime (s) and how far around the camera they spawn (m).
const SPLASH_LIFE: f32 = 0.45;
const SPLASH_SPREAD: f32 = 30.0;
/// Height above the ground plane to draw them, to avoid z-fighting the grid.
const SPLASH_Y: f32 = 0.02;
/// Hide splashes once the camera climbs past this (m): you don't see ripples on
/// the ground from cruise altitude, and drawing them there just looks like noise.
const SPLASH_MAX_CAM_H: f32 = 34.0;

#[derive(Component)]
struct RainDrop {
    /// Per-drop speed multiplier, for a little parallax.
    speed: f32,
}

#[derive(Resource)]
struct RainMaterial(Handle<StandardMaterial>);

/// One ripple. `age`/`life` drive an expand-and-fade; when `age` passes `life`
/// the ripple recycles to a fresh spot. Each carries its own material so it can
/// fade independently (one shared material would fade the whole field together).
#[derive(Component)]
struct Splash {
    age: f32,
    life: f32,
    material: Handle<StandardMaterial>,
}

pub struct WeatherPlugin;

impl Plugin for WeatherPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<Weather>()
            .add_systems(Startup, (setup_rain, setup_splashes))
            .add_systems(Update, (cycle_weather, update_rain, update_splashes));
    }
}

fn setup_rain(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    // A single quad in the XY plane: billboarded about Y each frame, its local
    // +Y stays world-up, so it reads as a vertical streak at any yaw.
    let mesh = meshes.add(Rectangle::new(STREAK_W, STREAK_H));
    let material = materials.add(StandardMaterial {
        base_color: Color::srgba(0.72, 0.80, 0.95, 0.0),
        unlit: true,
        alpha_mode: AlphaMode::Blend,
        cull_mode: None,
        ..default()
    });

    let mut rng = Lcg::new(0x9E11_0D5E);
    for _ in 0..DROPS {
        let p = Vec3::new(
            rng.range(-BOX_XZ, BOX_XZ),
            rng.range(-BOX_Y, BOX_Y),
            rng.range(-BOX_XZ, BOX_XZ),
        );
        commands.spawn((
            RainDrop {
                speed: rng.range(0.7, 1.3),
            },
            Mesh3d(mesh.clone()),
            MeshMaterial3d(material.clone()),
            Transform::from_translation(p),
            // Hidden until the level is raised (all drops start invisible).
            Visibility::Hidden,
        ));
    }
    commands.insert_resource(RainMaterial(material));
}

fn setup_splashes(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    // A thin unit ring, scaled per ripple. No texture: the ring geometry *is*
    // the ripple.
    let mesh = meshes.add(Annulus::new(0.7, 1.0));
    // Lie flat on the ground (the annulus is authored in the XY plane).
    let flat = Quat::from_rotation_x(-std::f32::consts::FRAC_PI_2);

    let mut rng = Lcg::new(0x5A1A_5B00);
    for _ in 0..SPLASHES {
        let material = materials.add(StandardMaterial {
            base_color: Color::srgba(0.86, 0.91, 1.0, 0.0),
            unlit: true,
            alpha_mode: AlphaMode::Blend,
            cull_mode: None,
            ..default()
        });
        let life = SPLASH_LIFE * rng.range(0.8, 1.2);
        commands.spawn((
            Splash {
                // Stagger ages so the field doesn't pulse in lockstep.
                age: rng.next() * life,
                life,
                material: material.clone(),
            },
            Mesh3d(mesh.clone()),
            MeshMaterial3d(material),
            Transform::from_translation(random_disk(&mut rng, Vec3::ZERO, SPLASH_SPREAD))
                .with_rotation(flat),
            Visibility::Hidden,
        ));
    }
}

/// `T` steps through the intensity levels (wrapping back to clear).
fn cycle_weather(keys: Res<ButtonInput<KeyCode>>, mut weather: ResMut<Weather>) {
    if !keys.just_pressed(KeyCode::KeyT) {
        return;
    }
    let idx = RAIN_LEVELS
        .iter()
        .position(|&l| (l - weather.rain).abs() < 1e-3)
        .unwrap_or(0);
    weather.rain = RAIN_LEVELS[(idx + 1) % RAIN_LEVELS.len()];
}

fn update_rain(
    time: Res<Time>,
    weather: Res<Weather>,
    mat: Res<RainMaterial>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    camera: Query<&Transform, (With<Camera3d>, Without<RainDrop>)>,
    mut drops: Query<(&mut Transform, &mut Visibility, &RainDrop)>,
) {
    let Ok(cam) = camera.single() else {
        return;
    };
    let cam = cam.translation;
    let dt = time.delta_secs();
    let rain = weather.rain;

    if let Some(mut m) = materials.get_mut(&mat.0) {
        m.base_color.set_alpha(0.14 + 0.30 * rain);
    }

    // Show a fraction of the pool proportional to intensity: light rain = thin.
    let shown = (rain * DROPS as f32) as usize;
    for (i, (mut tf, mut vis, drop)) in drops.iter_mut().enumerate() {
        if i >= shown {
            *vis = Visibility::Hidden;
            continue;
        }
        *vis = Visibility::Inherited;

        // Fall and drift, then wrap into the box that follows the camera.
        tf.translation.y -= FALL * drop.speed * dt;
        tf.translation.x += WIND * drop.speed * dt;
        let rel = tf.translation - cam;
        let mut wy = rel.y;
        if wy < -BOX_Y {
            wy += 2.0 * BOX_Y;
        }
        tf.translation = cam + Vec3::new(wrap(rel.x, BOX_XZ), wy, wrap(rel.z, BOX_XZ));

        // Billboard about Y so the streak faces the camera while staying upright.
        let dir = cam - tf.translation;
        tf.rotation = Quat::from_rotation_y(dir.x.atan2(dir.z));
    }
}

/// Expand-and-fade the ripples, recycling each when it dies. Only shown in the
/// rain and only while the camera is near the ground.
fn update_splashes(
    time: Res<Time>,
    weather: Res<Weather>,
    mut rng: Local<Lcg>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    camera: Query<&Transform, (With<Camera3d>, Without<Splash>)>,
    mut splashes: Query<(&mut Transform, &mut Visibility, &mut Splash)>,
) {
    let Ok(cam) = camera.single() else {
        return;
    };
    let cam = cam.translation;
    let dt = time.delta_secs();
    let rain = weather.rain;
    let visible = rain > 0.02 && cam.y < SPLASH_MAX_CAM_H;
    // More rain → more of the pool active.
    let shown = (rain * SPLASHES as f32) as usize;

    for (i, (mut tf, mut vis, mut splash)) in splashes.iter_mut().enumerate() {
        if !visible || i >= shown {
            *vis = Visibility::Hidden;
            continue;
        }
        *vis = Visibility::Inherited;

        splash.age += dt;
        if splash.age >= splash.life {
            splash.age -= splash.life;
            tf.translation = random_disk(&mut rng, cam, SPLASH_SPREAD);
        }

        let t = (splash.age / splash.life).clamp(0.0, 1.0);
        // Quick pop then slow-out, the way a real ripple spreads.
        let e = 1.0 - (1.0 - t) * (1.0 - t);
        tf.scale = Vec3::splat(SPLASH_R0 + (SPLASH_R1 - SPLASH_R0) * e);
        if let Some(mut m) = materials.get_mut(&splash.material) {
            m.base_color.set_alpha((1.0 - t) * 0.5 * rain);
        }
    }
}

/// A uniformly random point on the ground (`y = SPLASH_Y`) within `radius` of
/// `centre`, by the usual `sqrt` trick for a uniform disk.
fn random_disk(rng: &mut Lcg, centre: Vec3, radius: f32) -> Vec3 {
    let ang = rng.range(0.0, std::f32::consts::TAU);
    let r = radius * rng.next().sqrt();
    Vec3::new(
        centre.x + r * ang.cos(),
        SPLASH_Y,
        centre.z + r * ang.sin(),
    )
}

/// Wrap `v` into `[-h, h)`.
fn wrap(v: f32, h: f32) -> f32 {
    (v + h).rem_euclid(2.0 * h) - h
}

/// Tiny deterministic PRNG, so the drop field is reproducible.
#[derive(Default)]
struct Lcg(u64);

impl Lcg {
    fn new(seed: u64) -> Self {
        Lcg(seed)
    }

    fn next(&mut self) -> f32 {
        self.0 = self
            .0
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        ((self.0 >> 40) as f32) / (1u64 << 24) as f32
    }

    fn range(&mut self, a: f32, b: f32) -> f32 {
        a + (b - a) * self.next()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn wrap_maps_into_the_half_open_interval() {
        assert_eq!(wrap(0.0, 10.0), 0.0);
        assert!((wrap(15.0, 10.0) - -5.0).abs() < 1e-5);
        assert!((wrap(-15.0, 10.0) - 5.0).abs() < 1e-5);
        // Anything already inside is unchanged.
        assert!((wrap(3.0, 10.0) - 3.0).abs() < 1e-5);
    }

    #[test]
    fn cycle_walks_the_levels_and_wraps() {
        // The index walk the key handler performs.
        let step = |r: f32| {
            let i = RAIN_LEVELS.iter().position(|&l| (l - r).abs() < 1e-3).unwrap();
            RAIN_LEVELS[(i + 1) % RAIN_LEVELS.len()]
        };
        assert_eq!(step(0.0), 0.3);
        assert_eq!(step(0.9), 0.0, "wraps back to clear");
    }

    #[test]
    fn labels_track_intensity() {
        assert_eq!(Weather { rain: 0.0 }.label(), "CLEAR");
        assert_eq!(Weather { rain: 0.3 }.label(), "LIGHT RAIN");
        assert_eq!(Weather { rain: 0.6 }.label(), "RAIN");
        assert_eq!(Weather { rain: 0.9 }.label(), "STORM");
    }

    #[test]
    fn splash_points_stay_on_the_ground_within_spread() {
        let mut rng = Lcg::new(42);
        let centre = Vec3::new(100.0, 5.0, -30.0);
        for _ in 0..500 {
            let p = random_disk(&mut rng, centre, SPLASH_SPREAD);
            assert_eq!(p.y, SPLASH_Y, "splashes sit on the ground plane");
            let d = ((p.x - centre.x).powi(2) + (p.z - centre.z).powi(2)).sqrt();
            assert!(d <= SPLASH_SPREAD + 1e-3, "outside the spawn disk: {d}");
        }
    }
}
