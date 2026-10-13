use std::env;
use std::path::PathBuf;

const DEFAULT_JSBSIM_SRC: &str = "/home/nanbert/gitrep/jsbsim";

fn main() {
    let src = PathBuf::from(env::var("JSBSIM_SRC").unwrap_or_else(|_| DEFAULT_JSBSIM_SRC.into()));
    assert!(
        src.join("src/FGFDMExec.h").exists(),
        "JSBSIM_SRC={src:?} is not a JSBSim checkout (missing src/FGFDMExec.h)"
    );

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

/// Directory shared between this crate and the top-level app for the built
/// JSBSim library. The top-level build.rs repeats this logic so the app's
/// rpath points at the same place.
fn shared_build_dir() -> PathBuf {
    let manifest = PathBuf::from(env::var("CARGO_MANIFEST_DIR").unwrap());
    let workspace = manifest.parent().expect("jsbsim-sys must live in a workspace");
    let target = env::var("CARGO_TARGET_DIR")
        .map(PathBuf::from)
        .unwrap_or_else(|_| workspace.join("target"));
    target.join("jsbsim-build")
}
