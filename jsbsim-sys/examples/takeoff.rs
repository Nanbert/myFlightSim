//! Headless autopilot demo: ground takeoff -> climb -> level cruise.
//!
//! Run: `cargo run -p jsbsim-sys --example takeoff`
//!
//! Purpose: prove the FDM flies a full, sane mission under closed-loop control,
//! so a shaky manual flight in the game can be attributed to piloting rather
//! than the simulation. Runs the same `jsbsim_sys::autopilot` law the game uses.
//! Prints a telemetry table; no window, no Bevy.

use jsbsim_sys::{autopilot, jsbsim_src, Fdm};

const DT: f64 = 1.0 / 120.0;

/// Height of the CG above the runway when parked (from c172p/reset00.xml).
/// Seeding the IC at 0 buries the gear and the ground springs fling the aircraft.
const REST_H_FT: f64 = 4.7;

fn main() {
    let root = jsbsim_src();
    let mut fdm = Fdm::new();
    fdm.set_root(&root);
    fdm.set_aircraft_path("aircraft");
    fdm.set_engine_path("engine");
    fdm.set_systems_path("systems");
    fdm.set_dt(DT);
    assert!(fdm.load_model("c172p"), "load_model(c172p) failed");

    // Ground start at the runway threshold, at rest, pointing north.
    fdm.ic_speed_kts(0.0);
    fdm.ic_altitude_ft(REST_H_FT);
    fdm.ic_heading_deg(0.0);
    assert!(fdm.run_ic(), "RunIC failed");

    // Light the engine; without this the throttle commands produce no thrust.
    fdm.set_property("propulsion/set-running", -1.0);

    println!(
        "{:>6}  {:>8}  {:>7}  {:>8}  {:>6}  {:>5}  {:>5}  {:>5}  {:>3}  phase",
        "t[s]", "ALT[ft]", "IAS[kt]", "VS[fpm]", "PITCH", "ROLL", "HDG", "THR", "WOW"
    );

    let total_s = 180.0;
    let steps = (total_s / DT) as usize;

    for i in 0..steps {
        let t = i as f64 * DT;

        let (cmds, phase) = autopilot::step(&fdm);
        autopilot::apply(&mut fdm, &cmds);
        fdm.run();

        let alt = fdm.get_property("position/h-sl-ft");
        let vc = fdm.get_property("velocities/vc-kts");
        let vs = fdm.get_property("velocities/h-dot-fps") * 60.0; // fpm
        let theta = fdm.get_property("attitude/theta-deg");
        let phi = fdm.get_property("attitude/phi-deg");
        let psi = fdm.get_property("attitude/psi-deg");
        let wow = fdm.get_property("gear/wow");

        // Dense rows for the first 3 s (watch it settle on the gear), then 1 Hz.
        let print_now = if t < 3.0 { i % 30 == 0 } else { i % 120 == 0 };
        if print_now {
            println!(
                "{:>6.1}  {:>8.0}  {:>7.0}  {:>8.0}  {:>6.1}  {:>5.1}  {:>5.0}  {:>5.2}  {:>3.0}  {}",
                t, alt, vc, vs, theta, phi, psi, cmds.throttle, wow, phase
            );
        }
    }
}
