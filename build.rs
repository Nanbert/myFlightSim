use std::env;
use std::path::PathBuf;

// jsbsim-sys builds libJSBSim.so into <target>/jsbsim-build/lib. Cargo does not
// propagate `rustc-link-arg` from dependencies, so the app binary needs its own
// rpath entry pointing at the same directory.
fn main() {
    // Web builds drop the `jsbsim` feature (libJSBSim can't link to wasm), and
    // wasm-ld rejects `-rpath` outright — so skip the arg on that target.
    if env::var("TARGET").is_ok_and(|t| t.starts_with("wasm32")) {
        return;
    }
    let manifest = PathBuf::from(env::var("CARGO_MANIFEST_DIR").unwrap());
    let target = env::var("CARGO_TARGET_DIR")
        .map(PathBuf::from)
        .unwrap_or_else(|_| manifest.join("target"));
    let lib_dir = target.join("jsbsim-build/lib");
    println!("cargo:rustc-link-arg=-Wl,-rpath,{}", lib_dir.display());
}
