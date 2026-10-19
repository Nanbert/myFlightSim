//! Moisture on the glass: the lens in the chase views, the canopy in the cockpit.
//!
//! Water films the glass only when the air can deposit it: precipitation, or
//! aerodynamic condensation — the same local pressure/temperature drop that
//! lights the F-22's vapor in `vapor.rs`. The film is a state, not a switch: it
//! builds while the source persists, then dries. Both the drying rate and the
//! drop animation scale with dynamic pressure, so the beads are sheared off in a
//! fraction of a second at fighter speed while a film at rest lingers for
//! seconds. Clear, dry air supplies nothing, so the glass stays clear.
//!
//! The droplet field itself is a port of *Heartfelt* (Martijn Steinrucken /
//! BigWings, shadertoy `ltffzl`), reworked to sample the live frame instead of a
//! raymarched scene. The pass reads [`Weather`] and [`AircraftState`] and rides
//! on Bevy's `FullscreenMaterial`; the component is attached to the camera only
//! while there is film, so a dry sky costs nothing.
//!
//! In the cockpit view the film belongs to the canopy, not the camera: the
//! shader indexes the droplet field by direction in the airframe frame (using
//! [`Ui`]'s head look), so the drops stay on the glass as the pilot looks
//! around, and it masks them off the instrument panel.

use bevy::core_pipeline::fullscreen_material::{FullscreenMaterial, FullscreenMaterialPlugin};
use bevy::prelude::*;
use bevy::render::extract_component::ExtractComponent;
use bevy::render::render_resource::ShaderType;
use bevy::render::RenderApp;
use bevy::shader::ShaderRef;

use crate::app_state::AppState;
use crate::sim::{AircraftState, Sim};
use crate::view::Ui;
use crate::weather::Weather;

/// Window polygons carried to the shader: eight per aircraft is one slot per
/// window the C172 and 737 show from the pilot seat.
const GLASS_POLYS: usize = 8;
/// Vertices per polygon slot (the measured outlines are simplified to twelve).
const GLASS_VERTS: usize = 12;
/// `vec4`s per polygon slot: two vertices each.
const GLASS_VECS: usize = GLASS_VERTS / 2;

/// Per-frame parameters handed to `assets/shaders/rain_lens.wesl`. The scalar
/// group is 32 bytes (two 16-byte rows), which aligns the `vec4` array that
/// follows it for WGSL's uniform layout rules.
#[derive(Component, Clone, Copy, ShaderType, ExtractComponent)]
#[extract_app(RenderApp)]
struct RainLens {
    /// Film coverage, 0 (dry) … 1 (downpour).
    intensity: f32,
    /// Animation clock (s), advanced faster as dynamic pressure rises — the
    /// same beads slide quicker and recycle sooner on a fast airframe.
    phase: f32,
    /// Normalised dynamic pressure, 0 (parked) … 1 (fighter speed). Drives the
    /// drying rate in Rust and the streak length / shear in the shader.
    flow: f32,
    /// Share of the film that came from precipitation rather than condensation.
    /// Rain covers all three droplet layers; a condensation film stays fine and
    /// sparse.
    rain_share: f32,
    /// 1 in the cockpit view: index the droplet field by the canopy direction
    /// instead of the screen, so the water belongs to the glass around the
    /// pilot instead of the camera lens.
    canopy: f32,
    /// Cockpit head-look (radians) relative to the airframe, used to turn the
    /// screen ray into a canopy direction.
    look_yaw: f32,
    look_pitch: f32,
    /// 1 when the per-aircraft window outlines gate the film, 0 to keep the
    /// F-22's plain screen-space fade.
    glass: f32,
    /// Vertex count of each window slot, four counts per `vec4`.
    glass_counts: [Vec4; 2],
    /// Window outlines in airframe azimuth/elevation (radians): twelve vertices
    /// per slot, two per `vec4`.
    glass_verts: [Vec4; GLASS_POLYS * GLASS_VECS],
}

impl Default for RainLens {
    fn default() -> Self {
        Self {
            intensity: 0.0,
            phase: 0.0,
            flow: 0.0,
            rain_share: 0.0,
            canopy: 0.0,
            look_yaw: 0.0,
            look_pitch: 0.0,
            glass: 0.0,
            glass_counts: [Vec4::ZERO; 2],
            glass_verts: [Vec4::ZERO; GLASS_POLYS * GLASS_VECS],
        }
    }
}

/// A window outline in airframe azimuth/elevation (degrees): positive azimuth
/// to the right, positive elevation up.
type Window = &'static [(f32, f32)];

/// Cessna 172 windows, measured from the pilot eye with Blender raycasts that
/// count hits on the model's exterior glass meshes. Seven panes: left door,
/// windshield, right door, and the rear quarter windows.
const C172_WINDOWS: &[Window] = &[
    &[
        (-122.5, -34.25),
        (-104.5, -34.25),
        (-86.5, -34.25),
        (-68.5, -28.25),
        (-50.5, -23.75),
        (-32.5, -20.75),
        (-32.5, 10.75),
        (-50.5, 12.25),
        (-68.5, 12.25),
        (-86.5, 12.25),
        (-104.5, 10.75),
        (-122.5, 9.25),
    ],
    &[
        (-30.5, -11.75),
        (-14.1, -8.75),
        (2.3, 3.25),
        (18.7, 3.25),
        (35.1, -8.75),
        (51.5, -8.75),
        (51.5, 21.25),
        (35.1, 27.25),
        (18.7, 30.25),
        (2.3, 30.25),
        (-14.1, 25.75),
        (-30.5, 27.25),
    ],
    &[
        (54.5, -13.25),
        (64.9, -16.25),
        (75.3, -16.25),
        (85.7, -10.25),
        (96.1, -19.25),
        (106.5, -19.25),
        (106.5, 4.75),
        (96.1, 4.75),
        (85.7, 6.25),
        (75.3, 6.25),
        (64.9, 6.25),
        (54.5, 3.25),
    ],
    &[
        (-162.5, -14.75),
        (-156.7, -17.75),
        (-150.9, -22.25),
        (-145.1, -25.25),
        (-139.3, -26.75),
        (-133.5, -28.25),
        (-133.5, 3.25),
        (-139.3, 3.25),
        (-145.1, 1.75),
        (-150.9, 1.75),
        (-156.7, 0.25),
        (-162.5, -1.25),
    ],
    &[
        (113.5, -17.75),
        (119.9, -17.75),
        (126.3, -17.75),
        (132.7, -16.25),
        (139.1, -14.75),
        (145.5, -11.75),
        (145.5, -2.75),
        (139.1, -1.25),
        (132.7, 0.25),
        (126.3, 1.75),
        (119.9, 1.75),
        (113.5, 1.75),
    ],
    &[
        (150.5, -8.75),
        (154.5, -7.25),
        (158.5, -5.75),
        (162.5, -5.75),
        (166.5, -4.25),
        (170.5, -4.25),
        (170.5, 1.75),
        (166.5, 3.25),
        (162.5, 3.25),
        (158.5, 3.25),
        (154.5, 1.75),
        (150.5, 0.25),
    ],
    &[
        (172.5, -4.25),
        (176.5, -4.25),
        (180.5, -5.75),
        (184.5, -5.75),
        (188.5, -7.25),
        (192.5, -8.75),
        (192.5, -1.25),
        (188.5, 1.75),
        (184.5, 3.25),
        (180.5, 3.25),
        (176.5, 3.25),
        (172.5, 3.25),
    ],
];

/// 737 flight-deck windows, measured the same way but by counting rays that
/// escape the interior (the model's windows are open geometry, not glass).
/// Six panes: a sliding window on each side and two forward panes on each
/// pilot's side.
const B737_WINDOWS: &[Window] = &[
    &[
        (-101.5, 10.75),
        (-91.3, 6.25),
        (-81.1, 3.25),
        (-70.9, 1.75),
        (-60.7, -1.25),
        (-50.5, -2.75),
        (-50.5, 31.75),
        (-60.7, 31.75),
        (-70.9, 31.75),
        (-81.1, 31.75),
        (-91.3, 30.25),
        (-101.5, 28.75),
    ],
    &[
        (-41.5, -2.75),
        (-35.1, -2.75),
        (-28.7, -2.75),
        (-22.3, -1.25),
        (-15.9, 0.25),
        (-9.5, 1.75),
        (-9.5, 22.75),
        (-15.9, 24.25),
        (-22.3, 25.75),
        (-28.7, 27.25),
        (-35.1, 28.75),
        (-41.5, 28.75),
    ],
    &[
        (-3.5, 3.25),
        (2.3, 1.75),
        (8.1, 1.75),
        (13.9, 3.25),
        (19.7, 3.25),
        (25.5, 9.25),
        (25.5, 16.75),
        (19.7, 16.75),
        (13.9, 18.25),
        (8.1, 19.75),
        (2.3, 19.75),
        (-3.5, 21.25),
    ],
    &[
        (23.5, 3.25),
        (29.7, 3.25),
        (35.9, 3.25),
        (42.1, 1.75),
        (48.3, 1.75),
        (54.5, 9.25),
        (54.5, 12.25),
        (48.3, 13.75),
        (42.1, 13.75),
        (35.9, 15.25),
        (29.7, 15.25),
        (23.5, 7.75),
    ],
    &[
        (51.5, 1.75),
        (55.9, 0.25),
        (60.3, -1.25),
        (64.7, -1.25),
        (69.1, -1.25),
        (73.5, 4.75),
        (73.5, 10.75),
        (69.1, 10.75),
        (64.7, 12.25),
        (60.3, 12.25),
        (55.9, 9.25),
        (51.5, 4.75),
    ],
    &[
        (74.5, -1.25),
        (77.5, 0.25),
        (80.5, 0.25),
        (83.5, 1.75),
        (86.5, 1.75),
        (89.5, 3.25),
        (89.5, 9.25),
        (86.5, 10.75),
        (83.5, 10.75),
        (80.5, 10.75),
        (77.5, 10.75),
        (74.5, 3.25),
    ],
];

/// Airframe-frame direction for an azimuth/elevation pair, matching the
/// shader's `atan2(d.x, -d.z)` / `asin(d.y)` convention.
#[cfg(test)]
fn body_dir(az: f32, el: f32) -> Vec3 {
    Vec3::new(az.sin() * el.cos(), el.sin(), -az.cos() * el.cos())
}

/// The window mask for `name`, packed for the shader. The F-22 keeps the
/// screen-space fade: it has no measured windows in this pass.
fn glass_mask(name: &str) -> RainLens {
    let windows = match name {
        "737" | "b737" => B737_WINDOWS,
        "f22" => return RainLens::default(),
        _ => C172_WINDOWS,
    };
    let mut lens = RainLens::default();
    for (slot, window) in windows.iter().take(GLASS_POLYS).enumerate() {
        let n = window.len().min(GLASS_VERTS);
        if n < 3 {
            continue;
        }
        let first = slot * GLASS_VECS;
        for i in 0..GLASS_VERTS {
            // Short outlines pad by repeating their last vertex; the shader
            // only walks the first `n`.
            let (az, el) = window[i.min(n - 1)];
            let point = Vec2::new(az.to_radians(), el.to_radians());
            let packed = &mut lens.glass_verts[first + i / 2];
            if i % 2 == 0 {
                packed.x = point.x;
                packed.y = point.y;
            } else {
                packed.z = point.x;
                packed.w = point.y;
            }
        }
        let row = slot / 4;
        match slot % 4 {
            0 => lens.glass_counts[row].x = n as f32,
            1 => lens.glass_counts[row].y = n as f32,
            2 => lens.glass_counts[row].z = n as f32,
            _ => lens.glass_counts[row].w = n as f32,
        }
        lens.glass = 1.0;
    }
    lens
}

impl FullscreenMaterial for RainLens {
    fn fragment_shader() -> ShaderRef {
        "shaders/rain_lens.wesl".into()
    }
}

/// Film state that outlives any single camera attachment.
#[derive(Resource, Default)]
struct Film {
    /// Current coverage, 0..1.
    wet: f32,
    /// Integrated drop-animation clock.
    phase: f32,
}

/// Buildup rate while water is being deposited (1/s).
const BUILD: f32 = 2.5;
/// Drying rate at rest (1/s): a film left on the glass lasts a few seconds.
const DRY_REST: f32 = 0.35;
/// Extra drying per unit of normalised dynamic pressure (1/s). At fighter speed
/// the film is gone in a few tenths of a second — `q` shears it off.
const DRY_FLOW: f32 = 8.0;
/// Share of rain coverage the slipstream takes away at full flow. Drops are
/// shed about as fast as they land, so a fast airframe carries a thin, streaked
/// film rather than a sheet of beads.
const RAIN_SHEAR: f32 = 0.65;
/// Airspeed (kt) where the wash starts and where it saturates, matching the
/// camera-buffet range in `view.rs`.
const FLOW0_KT: f32 = 60.0;
const FLOW1_KT: f32 = 420.0;
/// How fast the droplet field's lifecycle (slide, buckle, recycle) runs at rest
/// and per unit of flow. Drops still fall under gravity when parked; in flight
/// the airflow drives them. The shader additionally scales how fast the whole
/// field streams across the glass by `flow`, so rain — which keeps renewing the
/// coverage — still reads as speed rather than a static film.
const PHASE_REST: f32 = 1.0;
const PHASE_FLOW: f32 = 3.0;

/// Normalised dynamic pressure proxy, 0..1. Calibrated airspeed is the right
/// signal here: it already folds altitude into the impact pressure the glass
/// feels.
fn flow_of(cas_kts: f32) -> f32 {
    let x = ((cas_kts - FLOW0_KT) / (FLOW1_KT - FLOW0_KT)).clamp(0.0, 1.0);
    x * x * (3.0 - 2.0 * x)
}

/// Equilibrium rain coverage: deposition scales with the rain rate, shedding
/// with dynamic pressure, so a downpour lays a full film at rest but only a
/// thin one at fighter speed.
fn rain_coverage(rain: f32, flow: f32) -> f32 {
    (rain * 1.5).min(1.0) * (1.0 - RAIN_SHEAR * flow)
}

/// Airframe condensation that can deposit on the glass: the same high-alpha /
/// transonic pressure drop that feeds the vapor, gated by humidity and by there
/// being enough flow to put the moisture onto the glass. `flying` is false in
/// the menu, where the aircraft state is stale.
fn condensation_source(alpha_deg: f32, mach: f32, cas_kts: f32, rain: f32, flying: bool) -> f32 {
    if !flying {
        return 0.0;
    }
    let humidity = 0.55 + 0.45 * rain;
    let speed = ((cas_kts - 80.0) / (140.0 - 80.0)).clamp(0.0, 1.0);
    let alpha = ((alpha_deg - 14.0) / 10.0).clamp(0.0, 1.0);
    let mach_band = {
        let d = (mach - 1.0) / 0.18;
        (1.0 - d * d).clamp(0.0, 1.0)
    };
    alpha.max(mach_band) * humidity * speed * 0.6
}

/// Advance the film one frame toward `source`. Builds at [`BUILD`]; once the
/// source drops below the current film it dries at a rate that grows with
/// dynamic pressure, so the same film disappears far faster at 400 kt than at
/// taxi speed.
fn film_step(wet: f32, source: f32, flow: f32, dt: f32) -> f32 {
    let source = source.clamp(0.0, 1.0);
    let rate = if source > wet {
        BUILD
    } else {
        DRY_REST + DRY_FLOW * flow
    };
    let next = wet + (source - wet) * (1.0 - (-rate * dt).exp());
    if next < 0.003 {
        0.0
    } else {
        next
    }
}

pub struct LensPlugin;

impl Plugin for LensPlugin {
    fn build(&self, app: &mut App) {
        app.add_plugins(FullscreenMaterialPlugin::<RainLens>::default())
            .init_resource::<Film>()
            .add_systems(Update, manage_lens);
    }
}

/// Advance the film and refresh the camera's uniform; drop the component once
/// the glass is dry, so the pass (and its frame copy) is skipped entirely.
fn manage_lens(
    mut commands: Commands,
    weather: Res<Weather>,
    time: Res<Time>,
    state: Res<AircraftState>,
    app_state: Res<State<AppState>>,
    sim: Res<Sim>,
    ui: Res<Ui>,
    mut film: ResMut<Film>,
    cameras: Query<
        (Entity, Option<&RainLens>),
        (With<Camera3d>, Without<crate::cloud_camera::CloudCamera>),
    >,
) {
    let dt = time.delta_secs().min(0.1);
    let flying = *app_state.get() == AppState::Flying;

    let flow = if flying {
        flow_of(state.cas_kts as f32)
    } else {
        // The menu's aircraft state is stale, so don't let a last flight's
        // airspeed drive the showcase camera.
        0.0
    };

    // Rain supplies the whole glass; condensation is a lighter, finer film.
    let rain_supply = rain_coverage(weather.rain, flow);
    let cond = condensation_source(
        state.alpha_deg as f32,
        state.mach as f32,
        state.cas_kts as f32,
        weather.rain,
        flying,
    );
    let source = rain_supply.max(cond);
    let rain_share = if source > 1e-4 {
        rain_supply / (rain_supply + cond)
    } else {
        0.0
    };

    film.wet = film_step(film.wet, source, flow, dt);
    film.phase += dt * (PHASE_REST + PHASE_FLOW * flow);

    let glass = glass_mask(&sim.name);
    let lens = (film.wet > 0.001).then_some(RainLens {
        intensity: film.wet,
        phase: film.phase,
        flow,
        rain_share,
        canopy: if ui.cockpit { 1.0 } else { 0.0 },
        look_yaw: ui.look_yaw,
        look_pitch: ui.look_pitch,
        glass: glass.glass,
        glass_counts: glass.glass_counts,
        glass_verts: glass.glass_verts,
    });

    for (e, attached) in &cameras {
        match (lens, attached.is_some()) {
            (Some(lens), _) => {
                commands.entity(e).insert(lens);
            }
            (None, true) => {
                commands.entity(e).remove::<RainLens>();
            }
            (None, false) => {}
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn vertex(lens: &RainLens, slot: usize, i: usize) -> Vec2 {
        let v = lens.glass_verts[slot * GLASS_VECS + i / 2];
        if i % 2 == 0 {
            Vec2::new(v.x, v.y)
        } else {
            Vec2::new(v.z, v.w)
        }
    }

    fn slot_len(lens: &RainLens, slot: usize) -> usize {
        let row = lens.glass_counts[slot / 4];
        let n = match slot % 4 {
            0 => row.x,
            1 => row.y,
            2 => row.z,
            _ => row.w,
        };
        n as usize
    }

    /// Signed distance to one window outline in az/el space, positive inside.
    fn poly_margin(lens: &RainLens, p: Vec2, slot: usize) -> f32 {
        let n = slot_len(lens, slot);
        if n < 3 {
            return -1.0;
        }
        let mut inside = false;
        let mut dist = f32::MAX;
        for i in 0..n {
            let a = vertex(lens, slot, i);
            let b = vertex(lens, slot, (i + 1) % n);
            if (a.y > p.y) != (b.y > p.y) {
                let t = (p.y - a.y) / (b.y - a.y);
                if p.x < a.x + t * (b.x - a.x) {
                    inside = !inside;
                }
            }
            let ab = b - a;
            let t = ((p - a).dot(ab) / ab.length_squared().max(1e-9)).clamp(0.0, 1.0);
            dist = dist.min((p - a - ab * t).length());
        }
        if inside {
            dist
        } else {
            -dist
        }
    }

    /// Margin to the nearest edge of the union of window outlines: positive
    /// when `d` looks through glass.
    fn window_margin(lens: &RainLens, d: Vec3) -> f32 {
        let p = Vec2::new(d.x.atan2(-d.z), d.y.clamp(-1.0, 1.0).asin());
        (0..GLASS_POLYS)
            .map(|slot| poly_margin(lens, p, slot))
            .fold(-1.0_f32, f32::max)
    }

    #[test]
    fn c172_windows_cover_the_windshield_but_not_the_panel() {
        let lens = glass_mask("c172p");
        assert_eq!(lens.glass, 1.0, "the C172 must use the window mask");
        // Ahead and slightly up is windshield.
        let ahead = body_dir(0.0, 10.0_f32.to_radians());
        assert!(
            window_margin(&lens, ahead) > 0.05,
            "the windshield should cover the forward view"
        );
        // The glareshield's crown looks out below the windshield's dipped
        // centre edge; the panel and floor are not glass.
        assert!(window_margin(&lens, Vec3::NEG_Z) < 0.0);
        assert!(window_margin(&lens, Vec3::NEG_Y) < 0.0);
        assert!(window_margin(&lens, body_dir(0.0, -1.2)) < 0.0);
        // Looking over the left shoulder finds the left door window.
        assert!(window_margin(&lens, body_dir(-1.4, 0.0)) > 0.0);
    }

    #[test]
    fn b737_windows_cover_the_windshield_but_not_the_panel() {
        let lens = glass_mask("737");
        assert_eq!(lens.glass, 1.0, "the 737 must use the window mask");
        // Ahead and a little up is windshield.
        let ahead = body_dir(0.0, 10.0_f32.to_radians());
        assert!(
            window_margin(&lens, ahead) > 0.0,
            "the forward pane is missing"
        );
        // The sliding windows are off each shoulder.
        let left = body_dir(-70.0_f32.to_radians(), 10.0_f32.to_radians());
        assert!(
            window_margin(&lens, left) > 0.0,
            "the left window is missing"
        );
        let right = body_dir(80.0_f32.to_radians(), 5.0_f32.to_radians());
        assert!(
            window_margin(&lens, right) > 0.0,
            "the right window is missing"
        );
        // The MCP and the rest of the panel are below the windows.
        let panel = body_dir(0.0, -17.0_f32.to_radians());
        assert!(window_margin(&lens, panel) < 0.0);
    }

    #[test]
    fn only_the_measured_cockpits_use_the_window_mask() {
        assert_eq!(glass_mask("f22").glass, 0.0);
    }

    #[test]
    fn flow_ramps_with_airspeed() {
        assert_eq!(flow_of(0.0), 0.0);
        assert_eq!(flow_of(FLOW0_KT), 0.0);
        assert_eq!(flow_of(FLOW1_KT), 1.0);
        assert_eq!(flow_of(600.0), 1.0);
        assert!(flow_of(250.0) > 0.0 && flow_of(250.0) < 1.0);
        // Monotonic.
        assert!(flow_of(300.0) > flow_of(200.0));
    }

    #[test]
    fn clear_level_flight_condenses_nothing() {
        assert_eq!(condensation_source(2.0, 0.5, 300.0, 0.0, true), 0.0);
        // The menu's stale state must not wet the showcase camera.
        assert_eq!(condensation_source(25.0, 1.0, 400.0, 1.0, false), 0.0);
    }

    #[test]
    fn hard_pull_and_transonic_flight_condense() {
        let pull = condensation_source(24.0, 0.5, 300.0, 0.0, true);
        assert!(pull > 0.2, "a hard pull should condense, got {pull}");
        let sonic = condensation_source(2.0, 1.0, 390.0, 0.0, true);
        assert!(sonic > 0.2, "Mach 1 should condense, got {sonic}");
        // Below the flow gate nothing is deposited.
        assert_eq!(condensation_source(24.0, 1.0, 40.0, 0.0, true), 0.0);
        // Humid (raining) air condenses more readily.
        assert!(
            condensation_source(20.0, 0.5, 300.0, 1.0, true)
                > condensation_source(20.0, 0.5, 300.0, 0.0, true)
        );
    }

    #[test]
    fn rain_coverage_thins_as_dynamic_pressure_rises() {
        assert_eq!(rain_coverage(0.9, 0.0), 1.0);
        let fast = rain_coverage(0.9, 1.0);
        assert!(
            fast > 0.0 && fast < 0.5,
            "fast rain should be thin, got {fast}"
        );
        assert_eq!(rain_coverage(0.0, 0.5), 0.0);
    }

    #[test]
    fn rain_wets_the_glass_and_dries_once_it_stops() {
        let mut wet = 0.0;
        for _ in 0..120 {
            wet = film_step(wet, 1.0, 0.5, 1.0 / 60.0);
        }
        assert!(wet > 0.9, "rain should wet the glass, got {wet}");
        for _ in 0..120 {
            wet = film_step(wet, 0.0, 0.5, 1.0 / 60.0);
        }
        assert_eq!(wet, 0.0, "the film should be gone after the rain stops");
    }

    #[test]
    fn a_fast_film_dries_far_sooner_than_a_parked_one() {
        let decay = |flow: f32| {
            let mut wet = 1.0;
            for _ in 0..30 {
                wet = film_step(wet, 0.0, flow, 1.0 / 60.0);
            }
            wet
        };
        let slow = decay(0.0);
        let fast = decay(1.0);
        assert!(
            fast < slow * 0.25,
            "dynamic pressure must shear the film off: fast {fast}, slow {slow}"
        );
    }

    #[test]
    fn wetness_stays_in_range() {
        let mut wet = 0.0;
        for i in 0..600 {
            let source = if i < 300 { 1.0 } else { 0.0 };
            wet = film_step(wet, source, 1.0, 1.0 / 30.0);
            assert!((0.0..=1.0).contains(&wet), "wetness escaped: {wet}");
        }
    }
}
