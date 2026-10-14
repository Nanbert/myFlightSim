//! Scratch probe: pull full nose-up from cruise with TVC on vs off, to tune the
//! F-22's thrust-vectoring authority and see the difference.

use flight_model::FlightModel;

const DT: f64 = 1.0 / 120.0;

fn seed(m: &mut dyn FlightModel, tvc: bool) {
    m.set("ic/vc-kts", 350.0);
    m.set("ic/h-sl-ft", 15000.0);
    m.set("ic/psi-true-deg", 0.0);
    m.apply_ic();
    m.set("propulsion/set-running", -1.0);
    m.set("fcs/throttle-cmd-norm", 0.7);
    m.set("fcs/tvc-enable", if tvc { 1.0 } else { 0.0 });
    m.trim(350.0, 15000.0);
}

fn run(label: &str, tvc: bool) {
    let mut m: Box<dyn FlightModel> = Box::new(fdm::Fdm::new("f22"));
    seed(&mut *m, tvc);
    println!("\n=== {label} ===");
    println!(
        "{:>5} {:>7} {:>7} {:>7} {:>7} {:>9} {:>7}",
        "t", "theta", "alpha", "q", "tvc", "h", "vc"
    );
    let mut peak = 0.0f64;
    let mut peak_theta = 0.0f64;
    for i in 0..=2400 {
        if i > 0 {
            m.step();
        }
        // Pull to high alpha for 1.5 s, then release the stick.
        let cmd = if i <= 180 { -1.0 } else { 0.0 };
        m.set("fcs/elevator-cmd-norm", cmd);
        let a = m.get("aero/alpha-deg");
        peak = peak.max(a);
        peak_theta = peak_theta.max(m.get("attitude/theta-deg"));
        if i % 12 == 0 && (60..=600).contains(&i) {
            println!(
                "{:5.1} {:7.1} {:7.1} {:7.3} {:7.3} {:9.1} {:7.1}",
                i as f64 * DT,
                m.get("attitude/theta-deg"),
                a,
                m.get("velocities/q-rad_sec"),
                m.get("fcs/tvc-pos-rad"),
                m.get("position/h-sl-ft"),
                m.get("velocities/vc-kts"),
            );
        }
    }
    println!(
        "peak alpha = {:.1}  final theta = {:.1}  final vc = {:.0}  final q = {:.3}",
        peak,
        m.get("attitude/theta-deg"),
        m.get("velocities/vc-kts"),
        m.get("velocities/q-rad_sec"),
    );
}

fn main() {
    run("TVC ON", true);
    run("TVC OFF", false);
}
