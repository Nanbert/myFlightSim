//! Rust bindings to JSBSim (LGPL-2.1-or-later), built from a local checkout and
//! linked as a shared library. The C ABI shim lives in `shim/shim.cpp`.

use std::ffi::CString;
use std::os::raw::c_void;
use std::path::Path;

pub mod autopilot;

mod ffi {
    use std::os::raw::{c_char, c_void};

    extern "C" {
        pub fn jsbsim_new() -> *mut c_void;
        pub fn jsbsim_free(fdm: *mut c_void);
        pub fn jsbsim_set_root(fdm: *mut c_void, root: *const c_char);
        pub fn jsbsim_set_aircraft_path(fdm: *mut c_void, path: *const c_char);
        pub fn jsbsim_set_engine_path(fdm: *mut c_void, path: *const c_char);
        pub fn jsbsim_set_systems_path(fdm: *mut c_void, path: *const c_char);
        pub fn jsbsim_set_dt(fdm: *mut c_void, dt: f64);
        pub fn jsbsim_load_model(fdm: *mut c_void, name: *const c_char) -> i32;
        pub fn jsbsim_run_ic(fdm: *mut c_void) -> i32;
        pub fn jsbsim_run(fdm: *mut c_void) -> i32;
        pub fn jsbsim_set_property(fdm: *mut c_void, name: *const c_char, value: f64);
        pub fn jsbsim_get_property(fdm: *mut c_void, name: *const c_char) -> f64;
        pub fn jsbsim_ic_speed_kts(fdm: *mut c_void, kts: f64);
        pub fn jsbsim_ic_altitude_ft(fdm: *mut c_void, ft: f64);
        pub fn jsbsim_ic_heading_deg(fdm: *mut c_void, deg: f64);
    }
}

/// A JSBSim flight dynamics model instance (`FGFDMExec`).
pub struct Fdm {
    ptr: *mut c_void,
}

// `FGFDMExec` is not internally synchronized. Bevy guarantees exclusive access
// through `ResMut`, and we only ever drive it from a single system at a time,
// so handing it across threads between frames is sound.
unsafe impl Send for Fdm {}
unsafe impl Sync for Fdm {}

impl Fdm {
    pub fn new() -> Self {
        let ptr = unsafe { ffi::jsbsim_new() };
        assert!(!ptr.is_null(), "jsbsim_new returned null");
        Fdm { ptr }
    }

    /// Root directory containing `aircraft/`, `engine/`, `systems/`.
    pub fn set_root(&mut self, root: impl AsRef<Path>) {
        let c = cpath(root);
        unsafe { ffi::jsbsim_set_root(self.ptr, c.as_ptr()) };
    }

    pub fn set_aircraft_path(&mut self, path: &str) {
        let c = cstr(path);
        unsafe { ffi::jsbsim_set_aircraft_path(self.ptr, c.as_ptr()) };
    }

    pub fn set_engine_path(&mut self, path: &str) {
        let c = cstr(path);
        unsafe { ffi::jsbsim_set_engine_path(self.ptr, c.as_ptr()) };
    }

    pub fn set_systems_path(&mut self, path: &str) {
        let c = cstr(path);
        unsafe { ffi::jsbsim_set_systems_path(self.ptr, c.as_ptr()) };
    }

    pub fn set_dt(&mut self, dt: f64) {
        unsafe { ffi::jsbsim_set_dt(self.ptr, dt) };
    }

    /// Parse and load the aircraft definition. `name` is the model directory,
    /// e.g. `"c172p"`.
    pub fn load_model(&mut self, name: &str) -> bool {
        let c = cstr(name);
        unsafe { ffi::jsbsim_load_model(self.ptr, c.as_ptr()) != 0 }
    }

    /// Applies the configured initial conditions and runs one `dt = 0` frame.
    pub fn run_ic(&mut self) -> bool {
        unsafe { ffi::jsbsim_run_ic(self.ptr) != 0 }
    }

    /// Advances the simulation by one `dt`.
    pub fn run(&mut self) -> bool {
        unsafe { ffi::jsbsim_run(self.ptr) != 0 }
    }

    pub fn set_property(&mut self, name: &str, value: f64) {
        let c = cstr(name);
        unsafe { ffi::jsbsim_set_property(self.ptr, c.as_ptr(), value) };
    }

    pub fn get_property(&self, name: &str) -> f64 {
        let c = cstr(name);
        unsafe { ffi::jsbsim_get_property(self.ptr, c.as_ptr()) }
    }

    pub fn ic_speed_kts(&mut self, kts: f64) {
        unsafe { ffi::jsbsim_ic_speed_kts(self.ptr, kts) };
    }

    pub fn ic_altitude_ft(&mut self, ft: f64) {
        unsafe { ffi::jsbsim_ic_altitude_ft(self.ptr, ft) };
    }

    pub fn ic_heading_deg(&mut self, deg: f64) {
        unsafe { ffi::jsbsim_ic_heading_deg(self.ptr, deg) };
    }
}

impl Default for Fdm {
    fn default() -> Self {
        Self::new()
    }
}

impl Drop for Fdm {
    fn drop(&mut self) {
        unsafe { ffi::jsbsim_free(self.ptr) };
    }
}

fn cstr(s: &str) -> CString {
    CString::new(s).expect("path contained an interior NUL byte")
}

fn cpath(p: impl AsRef<Path>) -> CString {
    cstr(&p.as_ref().to_string_lossy())
}

/// Default JSBSim checkout used by the smoke test unless `JSBSIM_SRC` is set.
pub fn default_jsbsim_src() -> String {
    std::env::var("JSBSIM_SRC").unwrap_or_else(|_| "/home/nanbert/gitrep/jsbsim".into())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn c172p_climbs_after_throttle_up() {
        let mut fdm = Fdm::new();
        fdm.set_root(default_jsbsim_src());
        fdm.set_aircraft_path("aircraft");
        fdm.set_engine_path("engine");
        fdm.set_systems_path("systems");
        fdm.set_dt(1.0 / 120.0);

        assert!(fdm.load_model("c172p"), "failed to load c172p");
        fdm.ic_speed_kts(120.0);
        fdm.ic_altitude_ft(3000.0);
        assert!(fdm.run_ic(), "RunIC failed");

        for _ in 0..240 {
            fdm.set_property("fcs/throttle-cmd-norm", 1.0);
            assert!(fdm.run(), "Run returned false");
        }

        let h = fdm.get_property("position/h-sl-ft");
        let vc = fdm.get_property("velocities/vc-kts");
        assert!(h > 3000.0, "expected climb above 3000 ft, got {h}");
        assert!(vc > 90.0, "expected airspeed above 90 kt, got {vc}");
    }
}
