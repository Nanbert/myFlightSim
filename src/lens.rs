//! Screen-space moisture on the lens.
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

use bevy::core_pipeline::fullscreen_material::{FullscreenMaterial, FullscreenMaterialPlugin};
use bevy::prelude::*;
use bevy::render::extract_component::ExtractComponent;
use bevy::render::render_resource::ShaderType;
use bevy::render::RenderApp;
use bevy::shader::ShaderRef;

use crate::app_state::AppState;
use crate::sim::AircraftState;
use crate::weather::Weather;

/// Per-frame parameters handed to `assets/shaders/rain_lens.wesl`. Four `f32`s
/// fill one 16-byte uniform; the pads keep it aligned for WGSL's uniform layout
/// rules.
#[derive(Component, Clone, Copy, Default, ShaderType, ExtractComponent)]
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
    mut film: ResMut<Film>,
    cameras: Query<(Entity, Option<&RainLens>), With<Camera3d>>,
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

    let lens = (film.wet > 0.001).then_some(RainLens {
        intensity: film.wet,
        phase: film.phase,
        flow,
        rain_share,
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
        assert!(fast > 0.0 && fast < 0.5, "fast rain should be thin, got {fast}");
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
