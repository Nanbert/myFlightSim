use std::env;
use std::path::PathBuf;

fn main() {
    let src = resolve_jsbsim_src();

    // Build into a stable, profile-independent directory so the runner can
    // locate libJSBSim.so without depending on this crate's hash-based OUT_DIR.
    let build_dir = shared_build_dir();

    let dst = cmake::Config::new(&src)
        .out_dir(&build_dir)
        .define("BUILD_SHARED_LIBS", "ON")
        .define("BUILD_PYTHON_MODULE", "OFF")
        .define("BUILD_DOCS", "OFF")
        .define("BUILD_JULIA_PACKAGE", "OFF")
        .define("BUILD_MATLAB_SFUNCTION", "OFF")
        .define("CMAKE_BUILD_TYPE", "Release")
        .build();

    let lib_dir = dst.join("lib");
    println!("cargo:rustc-link-search=native={}", lib_dir.display());
    println!("cargo:rustc-link-lib=dylib=JSBSim");
    // Native rpath so this crate's own test binary can find the .so at runtime.
    println!("cargo:rustc-link-arg=-Wl,-rpath,{}", lib_dir.display());

    cc::Build::new()
        .cpp(true)
        .std("c++17")
        .warnings(false)
        .file("shim/shim.cpp")
        .include(src.join("src"))
        .include(src.join("src/simgear/misc"))
        .include(src.join("src/initialization"))
        .compile("jsbsim_shim");

    println!("cargo:rerun-if-changed=shim/shim.cpp");
    println!("cargo:rerun-if-env-changed=JSBSIM_SRC");
}

/// Locate the JSBSim source checkout to build from. `$JSBSIM_SRC` wins if set
/// (and must be valid); otherwise a few conventional locations are probed. No
/// per-machine path is baked in, so a fresh clone builds on any machine that
/// either sets `JSBSIM_SRC` or keeps its checkout somewhere standard.
///
/// Mirrored at runtime by `jsbsim_sys::jsbsim_src` (a build script can't share
/// code with the crate it builds).
fn resolve_jsbsim_src() -> PathBuf {
    let marker = "src/FGFDMExec.h";
    if let Ok(v) = env::var("JSBSIM_SRC") {
        if !v.is_empty() {
            let p = PathBuf::from(v);
            assert!(
                p.join(marker).exists(),
                "JSBSIM_SRC={} is not a JSBSim checkout (missing {marker})",
                p.display()
            );
            return p;
        }
    }

    let mut candidates = Vec::new();
    if let Some(home) = env::var_os("HOME") {
        let home = PathBuf::from(home);
        candidates.push(home.join("gitrep/jsbsim"));
        candidates.push(home.join("jsbsim"));
        candidates.push(home.join("src/jsbsim"));
    }
    candidates.push(PathBuf::from("/usr/local/src/jsbsim"));
    candidates.push(PathBuf::from("/opt/jsbsim"));

    candidates
        .iter()
        .find(|p| p.join(marker).exists())
        .cloned()
        .unwrap_or_else(|| {
            let tried: Vec<_> = candidates.iter().map(|p| p.display().to_string()).collect();
            panic!(
                "no JSBSim checkout found. Set JSBSIM_SRC to a JSBSim source tree \
                 (e.g. `export JSBSIM_SRC=$HOME/gitrep/jsbsim`). Looked in: {}",
                tried.join(", ")
            )
        })
}

/// Directory shared between this crate and the top-level app for the built
/// JSBSim library. The top-level build.rs repeats this logic so the app's
/// rpath points at the same place.
fn shared_build_dir() -> PathBuf {
    let manifest = PathBuf::from(env::var("CARGO_MANIFEST_DIR").unwrap());
    let workspace = manifest
        .parent()
        .expect("jsbsim-sys must live in a workspace");
    let target = env::var("CARGO_TARGET_DIR")
        .map(PathBuf::from)
        .unwrap_or_else(|_| workspace.join("target"));
    target.join("jsbsim-build")
}
