//! Scratch probe: headless ground takeoff under the game's autopilot law
//! (pure-Rust FDM), to see whether the c172p can actually hold a climb.
//!
//! Run: `cargo run --example climb_probe`

use flight_model::FlightModel;

fn main() {
    let mut m: Box<dyn FlightModel> = Box::new(fdm::Fdm::new("c172p"));
    // Mirror Sim::reset_ground.
    m.set("ic/vc-kts", 0.0);
    m.set("ic/h-sl-ft", m.rest_h_ft());
    m.set("ic/psi-true-deg", 0.0);
    m.apply_ic();
    m.set("gear/gear-cmd-norm", 1.0);
    m.set("gear/gear-pos-norm", 1.0);
    m.set("propulsion/set-running", -1.0);

    println!(
        "{:>6} {:>8} {:>7} {:>8} {:>6} {:>6} {:>5} {}",
        "t[s]", "ALT", "IAS", "VS[fpm]", "PITCH", "THR", "WOW", "phase"
    );
    for i in 0..=120 * 300 {
        let (cmds, phase) = jsbsim_sys::autopilot::step(&*m);
        jsbsim_sys::autopilot::apply(&mut *m, &cmds);
        if i > 0 {
            m.step();
        }
        if i % 120 == 0 {
            println!(
                "{:>6.0} {:>8.0} {:>7.0} {:>8.0} {:>6.1} {:>6.2} {:>5.0} {}",
                i as f64 / 120.0,
                m.get("position/h-sl-ft"),
                m.get("velocities/vc-kts"),
                m.get("velocities/h-dot-fps") * 60.0,
                m.get("attitude/theta-deg"),
                cmds.throttle,
                m.get("gear/wow"),
                phase,
            );
        }
    }
}
