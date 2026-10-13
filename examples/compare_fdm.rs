//! Validation harness: fly the pure-Rust FDM and JSBSim from identical initial
//! conditions and print their trajectories side by side, so the Rust model can
//! be checked against the C++ reference frame by frame.
//!
//! Run: `cargo run --release --example compare_fdm`

use flight_model::FlightModel;

const DT: f64 = 1.0 / 120.0;

/// One model, driven through the shared property bus.
struct Row {
    t: f64,
    h: f64,
    vc: f64,
    theta: f64,
    phi: f64,
    psi: f64,
    rpm: f64,
}

fn sample(m: &dyn FlightModel, t: f64) -> Row {
    Row {
        t,
        h: m.get("position/h-sl-ft"),
        vc: m.get("velocities/vc-kts"),
        theta: m.get("attitude/theta-deg"),
        phi: m.get("attitude/phi-deg"),
        psi: m.get("attitude/psi-deg"),
        rpm: m.get("propulsion/engine[0]/engine-rpm"),
    }
}

/// The pitch-moment split and its inputs, so a divergence can be traced to a
/// specific term rather than the net attitude.
struct MomentRow {
    alpha_deg: f64,
    qbar: f64,
    fbz: f64,
    thrust: f64,
    m_aero: f64,
    m_prop: f64,
}

fn moment_sample(m: &dyn FlightModel) -> MomentRow {
    MomentRow {
        alpha_deg: m.get("aero/alpha-rad") * 180.0 / std::f64::consts::PI,
        qbar: m.get("aero/qbar-psf"),
        fbz: m.get("forces/fbz-aero-lbs"),
        thrust: m.get("propulsion/engine[0]/thrust-lbs"),
        m_aero: m.get("moments/m-aero-lbsft"),
        m_prop: m.get("moments/m-prop-lbsft"),
    }
}

fn main() {
    let mut jsb: Box<dyn FlightModel> = Box::new(jsbsim_sys::Fdm::load("c172p", DT));
    let mut rust: Box<dyn FlightModel> = Box::new(fdm::Fdm::new("c172p"));

    for m in [&mut jsb, &mut rust] {
        m.set("ic/vc-kts", 120.0);
        m.set("ic/h-sl-ft", 3000.0);
        m.set("ic/psi-true-deg", 0.0);
        m.apply_ic();
        m.set("propulsion/set-running", -1.0);
        m.set("fcs/throttle-cmd-norm", 1.0);
    }

    println!(
        "{:>5} | {:>9} {:>7} {:>7} {:>7} | {:>9} {:>7} {:>7} {:>7} | {:>8} {:>8}",
        "t", "h_jsb", "vc_jsb", "th_jsb", "rpm_jsb", "h_rust", "vc_rust", "th_rust", "rpm_rust",
        "dh", "dvc"
    );

    let mut peak = (0.0f64, 0.0f64);
    for i in 0..=600 {
        if i > 0 {
            jsb.step();
            rust.step();
        }
        if i % 30 != 0 {
            continue;
        }
        let t = i as f64 * DT;
        let j = sample(&*jsb, t);
        let r = sample(&*rust, t);
        let (dh, dvc) = (r.h - j.h, r.vc - j.vc);
        peak.0 = peak.0.max(dh.abs());
        peak.1 = peak.1.max(dvc.abs());
        println!(
            "{:5.1} | {:9.1} {:7.1} {:7.2} {:7.0} | {:9.1} {:7.1} {:7.2} {:7.0} | {:8.1} {:8.1}",
            t, j.h, j.vc, j.theta, j.rpm, r.h, r.vc, r.theta, r.rpm, dh, dvc
        );
    }
    println!(
        "\npeak |Δh| = {:.1} ft, peak |Δvc| = {:.1} kt over 5 s",
        peak.0, peak.1
    );

    println!(
        "\n{:>5} | {:>6} {:>6} | {:>8} {:>7} {:>7} {:>8} {:>8} | {:>8} {:>7} {:>7} {:>8} {:>8}",
        "t", "al_jsb", "al_rs", "qbar_jsb", "fbz_jsb", "thr_j", "maero_j", "mprop_j", "qbar_rs",
        "fbz_rs", "thr_r", "maero_r", "mprop_r"
    );
    let mut jsb2: Box<dyn FlightModel> = Box::new(jsbsim_sys::Fdm::load("c172p", DT));
    let mut rust2: Box<dyn FlightModel> = Box::new(fdm::Fdm::new("c172p"));
    for m in [&mut jsb2, &mut rust2] {
        m.set("ic/vc-kts", 120.0);
        m.set("ic/h-sl-ft", 3000.0);
        m.set("ic/psi-true-deg", 0.0);
        m.apply_ic();
        m.set("propulsion/set-running", -1.0);
        m.set("fcs/throttle-cmd-norm", 1.0);
    }
    for i in 0..=600 {
        if i > 0 {
            jsb2.step();
            rust2.step();
        }
        if i % 60 != 0 {
            continue;
        }
        let t = i as f64 * DT;
        let j = moment_sample(&*jsb2);
        let r = moment_sample(&*rust2);
        println!(
            "{:5.1} | {:6.2} {:6.2} | {:8.1} {:7.0} {:7.0} {:8.0} {:8.0} | {:8.1} {:7.0} {:7.0} {:8.0} {:8.0}",
            t, j.alpha_deg, r.alpha_deg, j.qbar, j.fbz, j.thrust, j.m_aero, j.m_prop, r.qbar,
            r.fbz, r.thrust, r.m_aero, r.m_prop
        );
    }
}
