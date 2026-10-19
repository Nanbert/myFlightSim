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

/// Rain intensities selectable from the menu's global settings page. Index 0 is
/// clear; the values double as the overcast levels `sky.rs` pre-bakes its
/// texture at.
pub const RAIN_LEVELS: [f32; 4] = [0.0, 0.3, 0.6, 0.9];

/// Human-readable names for [`RAIN_LEVELS`], shown in the settings page.
pub const RAIN_LABELS: [&str; 4] = ["CLEAR", "LIGHT RAIN", "RAIN", "STORM"];

/// Wind speeds (knots) selectable from the menu's global settings page.
pub const WIND_LEVELS: [f32; 4] = [0.0, 5.0, 10.0, 15.0];

/// Human-readable names for [`WIND_LEVELS`].
pub const WIND_LABELS: [&str; 4] = ["CALM", "5 KT", "10 KT", "15 KT"];

/// The direction the wind levels blow *from*, degrees true. West-northwest:
/// a headwind with a right crosswind component for runway 36, so the level
/// matters for both the takeoff roll and the crab down final.
pub const WIND_FROM_DEG: f32 = 300.0;

/// Time-of-day presets selectable from the menu's global settings page.
pub const TIME_LABELS: [&str; 3] = ["DAY", "DUSK", "NIGHT"];

/// How "night" each preset is, 0 (full day) .. 1 (deep night). Every
/// light-eating piece of the scene — the sun, the sky/terrain/water shaders,
/// the aircraft lights' default state — reads this one scalar. Dusk keeps just
/// enough ambient to fly pattern work by; night is for the lights.
pub const NIGHT_FACTORS: [f32; 3] = [0.0, 0.5, 1.0];

/// Current weather. `rain` is 0 (clear) … 1 (downpour); `wind_kts` is the
/// ambient wind speed at surface level; `time_level` indexes the time-of-day
/// presets.
#[derive(Resource)]
pub struct Weather {
    pub rain: f32,
    pub wind_kts: f32,
    pub time_level: usize,
}

impl Default for Weather {
    fn default() -> Self {
        Weather {
            rain: 0.0,
            wind_kts: 0.0,
            time_level: 0,
        }
    }
}

impl Weather {
    /// Short HUD label for the current intensity.
    pub fn label(&self) -> &'static str {
        RAIN_LABELS[self.level()]
    }

    /// Index into [`RAIN_LEVELS`] nearest the current intensity.
    pub fn level(&self) -> usize {
        RAIN_LEVELS
            .iter()
            .position(|&l| (l - self.rain).abs() < 1e-3)
            .unwrap_or(0)
    }

    /// Set the intensity to level `i` (clamped, wrapping) and return the new
    /// index. Used by the menu's settings page.
    pub fn set_level(&mut self, i: isize) {
        let n = RAIN_LEVELS.len() as isize;
        let level = (i % n + n) % n;
        self.rain = RAIN_LEVELS[level as usize];
    }

    /// Short label for the current wind level.
    pub fn wind_label(&self) -> &'static str {
        WIND_LABELS[self.wind_level()]
    }

    /// Index into [`WIND_LEVELS`] nearest the current speed.
    pub fn wind_level(&self) -> usize {
        WIND_LEVELS
            .iter()
            .position(|&l| (l - self.wind_kts).abs() < 1e-3)
            .unwrap_or(0)
    }
    /// Set the wind speed to level `i` (clamped, wrapping) and return the new
    /// index. Used by the menu's settings page.
    pub fn set_wind_level(&mut self, i: isize) -> usize {
        let n = WIND_LEVELS.len() as isize;
        let level = (i % n + n) % n;
        self.wind_kts = WIND_LEVELS[level as usize];
        level as usize
    }

    /// Where the wind blows *toward*, degrees true (the sock's deflection).
    pub fn wind_toward_deg(&self) -> f32 {
        WIND_FROM_DEG + 180.0
    }

    /// Signed crosswind component (+ = blowing *from the pilot's right* when
    /// facing the runway heading) and headwind component (- = tailwind) in
    /// knots. Runway 36 with the WNW levels on: the wind comes 60° off the
    /// nose from the left, so the crosswind goes into the report card as
    /// "FROM LEFT" and pushes the drift to the right.
    ///
    /// Free function because it is judged against a chosen runway heading,
    /// not the weather's own state.
    pub fn components(wind_kts: f32, wind_from_deg: f32, rwy_hdg_deg: f32) -> (f32, f32) {
        let delta = (wind_from_deg - rwy_hdg_deg).to_radians();
        (wind_kts * delta.sin(), wind_kts * delta.cos())
    }

    /// Wind velocity as NED components in *feet per second*, the FDM/JSBSim
    /// bus unit, so `sim.rs` can push it straight through the property bus.
    pub fn wind_ned_fps(&self) -> (f64, f64) {
        const KTS_TO_FPS: f64 = 1.68781;
        let to = (self.wind_toward_deg() as f64).to_radians();
        let v = self.wind_kts as f64 * KTS_TO_FPS;
        (v * to.cos(), v * to.sin())
    }

    /// Short HUD label for the current time-of-day preset.
    pub fn time_label(&self) -> &'static str {
        TIME_LABELS[self.time_level.min(TIME_LABELS.len() - 1)]
    }

    /// 0 (full day) .. 1 (deep night) for the current preset — the single
    /// scalar every light-eating consumer dims by.
    pub fn night_factor(&self) -> f32 {
        NIGHT_FACTORS[self.time_level.min(NIGHT_FACTORS.len() - 1)]
    }

    /// Set the time-of-day to level `i` (wrapping) and return the new index.
    /// Used by the menu's settings page.
    pub fn set_time_level(&mut self, i: isize) -> usize {
        let n = TIME_LABELS.len() as isize;
        let level = (i % n + n) % n;
        self.time_level = level as usize;
        self.time_level
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
            .add_systems(Update, (update_rain, update_splashes));
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

fn update_rain(
    time: Res<Time>,
    weather: Res<Weather>,
    mat: Res<RainMaterial>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    camera: Query<
        &Transform,
        (
            With<Camera3d>,
            Without<crate::cloud_camera::CloudCamera>,
            Without<RainDrop>,
        ),
    >,
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
    camera: Query<
        &Transform,
        (
            With<Camera3d>,
            Without<crate::cloud_camera::CloudCamera>,
            Without<Splash>,
        ),
    >,
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
    Vec3::new(centre.x + r * ang.cos(), SPLASH_Y, centre.z + r * ang.sin())
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
    fn night_factor_follows_the_time_level() {
        let mut w = Weather::default();
        assert_eq!(w.night_factor(), 0.0);
        assert_eq!(w.set_time_level(1), 1);
        assert_eq!(w.time_label(), "DUSK");
        assert_eq!(w.night_factor(), 0.5);
        assert_eq!(w.set_time_level(3), 0); // wraps into range, DAY again
        assert_eq!(w.set_time_level(4), 1);
        assert_eq!(w.set_time_level(-1), 2); // negatives wrap too
        assert_eq!(w.time_label(), "NIGHT");
    }

    #[test]
    fn wrap_maps_into_the_half_open_interval() {
        assert_eq!(wrap(0.0, 10.0), 0.0);
        assert!((wrap(15.0, 10.0) - -5.0).abs() < 1e-5);
        assert!((wrap(-15.0, 10.0) - 5.0).abs() < 1e-5);
        // Anything already inside is unchanged.
        assert!((wrap(3.0, 10.0) - 3.0).abs() < 1e-5);
    }

    #[test]
    fn set_level_walks_the_levels_and_wraps() {
        let mut w = Weather::default();
        w.set_level(1);
        assert_eq!(w.rain, RAIN_LEVELS[1]);
        assert_eq!(w.level(), 1);
        w.set_level(3);
        assert_eq!(w.rain, RAIN_LEVELS[3]);
        // Wraps around both ends.
        w.set_level(4);
        assert_eq!(w.level(), 0);
        w.set_level(-1);
        assert_eq!(w.level(), 3);
    }

    #[test]
    fn labels_track_intensity() {
        assert_eq!(
            Weather {
                rain: 0.0,
                wind_kts: 0.0,
                time_level: 0
            }
            .label(),
            "CLEAR"
        );
        assert_eq!(
            Weather {
                rain: 0.3,
                wind_kts: 0.0,
                time_level: 0
            }
            .label(),
            "LIGHT RAIN"
        );
        assert_eq!(
            Weather {
                rain: 0.6,
                wind_kts: 0.0,
                time_level: 0
            }
            .label(),
            "RAIN"
        );
        assert_eq!(
            Weather {
                rain: 0.9,
                wind_kts: 0.0,
                time_level: 0
            }
            .label(),
            "STORM"
        );
    }

    #[test]
    fn wind_levels_cycle_like_the_rain_ones() {
        let mut w = Weather::default();
        assert_eq!(w.wind_label(), "CALM");
        w.set_wind_level(1);
        assert_eq!(w.wind_kts, 5.0);
        assert_eq!(w.wind_label(), "5 KT");
        // Wraps around both ends.
        w.set_wind_level(4);
        assert_eq!(w.wind_level(), 0);
        w.set_wind_level(-1);
        assert_eq!(w.wind_level(), 3);
        assert_eq!(w.wind_kts, 15.0);
    }

    /// The bus direction: from 300 deg means blowing toward 120 deg, so the
    /// NED components are east-strong and north-negative (blowing away from
    /// the north).
    #[test]
    fn wind_ned_components_match_the_from_direction() {
        let mut w = Weather::default();
        w.wind_kts = 10.0;
        let (north, east) = w.wind_ned_fps();
        // toward 120 deg: cos(120) = -0.5, sin(120) = +0.866.
        assert!((north / east - (-0.5 / 0.866)).abs() < 1e-3);
        // Magnitude: 10 kt in ft/s.
        let expected = 10.0 * 1.68781;
        assert!((north.hypot(east) - expected).abs() < 1e-6);
    }

    #[test]
    fn crosswind_components_split_head_and_cross() {
        // Runway 36, wind from 300 at 15 kt: 60 deg off the nose -> 13 kt
        // from the left (negative) and 7.5 kt of headwind.
        let (xw, hw) = Weather::components(15.0, 300.0, 0.0);
        assert!((xw - -15.0 * (60_f32).to_radians().sin()).abs() < 1e-4);
        assert!((xw + 12.99).abs() < 0.01, "from the left: {xw}");
        assert!((hw - 7.5).abs() < 1e-4);
        // Facing runway 18 the same breeze arrives from the right and from
        // behind: sign flips, headwind becomes tailwind.
        let (xw18, hw18) = Weather::components(15.0, 300.0, 180.0);
        assert!((xw18 - 12.99).abs() < 0.01, "from the right: {xw18}");
        assert!((hw18 + 7.5).abs() < 1e-4);
        // Dead downwind / dead headwind carry no crosswind (within float
        // noise off the exact 60-degree angle).
        let (head_xw, _) = Weather::components(10.0, 0.0, 0.0);
        assert!(head_xw.abs() < 1e-4);
        let (tail_xw, _) = Weather::components(10.0, 180.0, 0.0);
        assert!(tail_xw.abs() < 1e-4);
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
