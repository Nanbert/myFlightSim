//! Scratch probe: does a hard pull feel different with TVC on/off at the *app's*
//! start conditions? The app seeds every aircraft at its shared cruise point
//! (3000 ft / 110 kt). Compare that against a fighter-appropriate cruise.

use flight_model::FlightModel;

fn seed(name: &str, alt: f64, vc: f64) -> Box<dyn FlightModel> {
    let mut m: Box<dyn FlightModel> = Box::new(fdm::Fdm::new(name));
    m.set("ic/vc-kts", vc);
    m.set("ic/h-sl-ft", alt);
    m.set("ic/psi-true-deg", 0.0);
    m.apply_ic();
    m.set("propulsion/set-running", -1.0);
    m.trim(vc, alt);
    m
}

/// Hold full nose-up for 1.5 s, then release, and report the peak alpha over 8 s.
fn hard_pull(alt: f64, vc: f64, tvc: bool) -> (f64, f64, f64) {
    let mut m = seed("f22", alt, vc);
    m.set("fcs/tvc-enable", if tvc { 1.0 } else { 0.0 });
    let a0 = m.get("aero/alpha-deg");
    let (mut peak, mut maxq) = (0.0_f64, 0.0_f64);
    for i in 0..=120 * 8 {
        if i > 0 {
            m.step();
        }
        m.set("fcs/elevator-cmd-norm", if i <= 180 { -1.0 } else { 0.0 });
        peak = peak.max(m.get("aero/alpha-deg"));
        maxq = maxq.max(m.get("velocities/q-rad_sec").abs());
    }
    (a0, peak, maxq)
}

fn report(alt: f64, vc: f64) {
    println!("\n=== seed {vc:.0} kt @ {alt:.0} ft ===");
    let (a0, pon, qon) = hard_pull(alt, vc, true);
    let (_, poff, qoff) = hard_pull(alt, vc, false);
    println!("trim alpha {a0:.1}°");
    println!("  TVC ON  peak alpha {pon:6.1}°  max|q| {qon:.2}");
    println!("  TVC OFF peak alpha {poff:6.1}°  max|q| {qoff:.2}");
}

/// The app runs the autopilot by default, and `apply` writes the surface
/// channels *after* the stick bindings. Show that holding full nose-up changes
/// nothing while auto is engaged.
fn ap_eats_stick() {
    let mut m = seed("f22", 3000.0, 110.0);
    println!("\n=== autopilot engaged (app default), holding full W ===");
    for i in 0..=120 * 3 {
        if i > 0 {
            m.step();
        }
        m.set("fcs/elevator-cmd-norm", -1.0); // the pilot's full nose-up
        let (cmds, phase) = jsbsim_sys::autopilot::step(&*m);
        jsbsim_sys::autopilot::apply(&mut *m, &cmds);
        if i % 60 == 0 {
            println!(
                "t={:.1}s  phase={}  theta={:5.1}°  elev-cmd(after AP)={:+.2}",
                i as f64 / 120.0,
                phase,
                m.get("attitude/theta-deg"),
                m.get("fcs/elevator-cmd-norm"),
            );
        }
    }
}

fn main() {
    report(3000.0, 110.0);   // the app's shared cruise point (c172 values)
    report(15000.0, 350.0);  // a fighter-appropriate cruise
    ap_eats_stick();
}
