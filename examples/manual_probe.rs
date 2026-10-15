//! Scratch probe: manual (autopilot OFF) c172p takeoff, driven the way the app
//! drives the model (`input.rs` spring semantics, 3.0 full-deflection/s).
//!
//! Run: `cargo run --example manual_probe`

use flight_model::FlightModel;

const DT: f64 = 1.0 / 120.0;

fn main() {
    let mut m: Box<dyn FlightModel> = if std::env::var("JSB").is_ok() {
        Box::new(jsbsim_sys::Fdm::load("c172p", DT))
    } else {
        Box::new(fdm::Fdm::new("c172p"))
    };

    if std::env::var("TRIMTEST").is_ok() {
        for (vc, alt) in [(80.0, 1000.0), (100.0, 2000.0), (120.0, 3000.0), (75.0, 500.0)] {
            m.set("ic/vc-kts", vc);
            m.set("ic/h-sl-ft", alt);
            m.set("ic/psi-true-deg", 0.0);
            m.apply_ic();
            m.set("propulsion/set-running", -1.0);
            m.trim(vc, alt);
            println!(
                "trim {vc:>5} kt @ {alt:>6} ft -> roll-trim {:+.4} yaw-trim {:+.4} pitch-trim {:+.3} throttle {:.3}",
                m.get("fcs/roll-trim-cmd-norm"),
                m.get("fcs/yaw-trim-cmd-norm"),
                m.get("fcs/pitch-trim-cmd-norm"),
                m.get("fcs/throttle-cmd-norm"),
            );
        }
        return;
    }

    if std::env::var("GLIDE").is_ok() {
        // Airborne at cruise, engine never started: any roll now is aero, not torque.
        m.set("ic/vc-kts", 120.0);
        m.set("ic/h-sl-ft", 3000.0);
        m.set("ic/psi-true-deg", 0.0);
        m.apply_ic();
        println!("{:>6} {:>7} {:>6} {:>6} {:>6} {:>6}", "t[s]", "ALT", "IAS", "PITCH", "ROLL", "PSI");
        for i in 0..=120 * 60 {
            m.set("fcs/throttle-cmd-norm", 0.0);
            m.set("fcs/elevator-cmd-norm", 0.0);
            m.set("fcs/aileron-cmd-norm", 0.0);
            m.set("fcs/rudder-cmd-norm", 0.0);
            m.step();
            if i % 240 == 0 {
                println!(
                    "{:>6.0} {:>7.0} {:>6.0} {:>6.1} {:>6.1} {:>6.1}",
                    i as f64 * DT,
                    m.get("position/h-sl-ft"),
                    m.get("velocities/vc-kts"),
                    m.get("attitude/theta-deg"),
                    m.get("attitude/phi-deg"),
                    m.get("attitude/psi-deg"),
                );
            }
        }
        return;
    }

    m.set("ic/vc-kts", 0.0);
    m.set("ic/h-sl-ft", 4.7);
    m.set("ic/psi-true-deg", 0.0);
    m.apply_ic();
    m.set("gear/gear-cmd-norm", 1.0);
    m.set("gear/gear-pos-norm", 1.0);
    m.set("propulsion/set-running", -1.0);

    // Bindings the app writes every step. Spring elevator (W = nose up = neg).
    let mut throttle = 0.0_f64;

    let lateral = std::env::var("LATERAL").is_ok();
    println!(
        "{:>6} {:>7} {:>6} {:>7} {:>6} {:>6} {:>7} {:>6} {:>6} {}",
        "t[s]", "ALT", "IAS", "VS", "PITCH", "ROLL", "PSI", "ELEV", "AIL", "WOW"
    );
    for i in 0..=120 * 180 {
        throttle = (throttle + 0.6 * DT).min(1.0);
        if let Ok(h) = std::env::var("HOLD") {
            if i > 120 * 18 {
                throttle = h.parse().unwrap_or(1.0);
            }
        }
        if std::env::var("CUT").is_ok() && i > 120 * 22 {
            throttle = 0.0;
        }
        let (cmds, _phase) = jsbsim_sys::autopilot::step(&*m);
        // Optional: press "T" (trim) once at t=25s, then fly hands-off.
        if std::env::var("TRIMFLY").is_ok() && i == 120 * 25 {
            m.trim(m.get("velocities/vc-kts"), m.get("position/h-sl-ft"));
        }
        let (aileron, rudder) = if lateral {
            (cmds.aileron, cmds.rudder)
        } else {
            (0.0, 0.0)
        };
        m.set("fcs/throttle-cmd-norm", throttle);
        m.set("fcs/elevator-cmd-norm", cmds.elevator);
        m.set("fcs/pitch-trim-cmd-norm", 0.0);
        m.set("fcs/aileron-cmd-norm", aileron);
        m.set("fcs/rudder-cmd-norm", rudder);
        m.step();

        if i % 120 == 0 {
            println!(
                "{:>6.0} {:>7.0} {:>6.0} {:>7.0} {:>6.1} {:>6.1} {:>6.1} {:>6.2} {:>6.2} {}",
                i as f64 * DT,
                m.get("position/h-sl-ft"),
                m.get("velocities/vc-kts"),
                m.get("velocities/h-dot-fps") * 60.0,
                m.get("attitude/theta-deg"),
                m.get("attitude/phi-deg"),
                m.get("attitude/psi-deg"),
                cmds.elevator,
                aileron,
                m.get("gear/wow"),
            );
        }
    }
}
