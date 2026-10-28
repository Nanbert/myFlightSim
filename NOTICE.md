# NOTICE

myFlightSim

## Project license

The code in this repository is licensed under the **GNU Lesser General Public
License, version 2.1** — see [`LICENSE`](LICENSE).

`assets/` is **not** covered by that license: the 3D models, textures, audio and
shaders there are third-party works distributed under their own terms, listed in
[`CREDITS.md`](CREDITS.md). Two of them are **CC BY-NC-SA**, so this project —
and any build of it — must not be used commercially.

## JSBSim (LGPL-2.1)

The pure-Rust flight model in `crates/fdm` is a **transcription** of aircraft
aerodynamic data and model structure from [JSBSim](https://github.com/JSBSim-Team/jsbsim)
(LGPL-2.1), notably:

- `crates/fdm/src/aircraft/{c172,b737,f22,ah1s}.rs` — coefficient tables and
  `<function>`/`<table>` structure transcribed from JSBSim's `aircraft/*.xml`
  and `engine/*.xml`.
- `crates/fdm/src/{table,aero}.rs` — evaluators that follow JSBSim's semantics.

JSBSim's C++ source is **not vendored here**. The optional desktop-only backend
(`jsbsim-sys`) builds it from a local checkout (`$JSBSIM_SRC`) as a **shared
library** and links it dynamically. The web build (`--no-default-features`, the
only build published to GitHub Pages) does not contain any JSBSim code.

The full LGPL-2.1 text is in [`LICENSE`](LICENSE). The corresponding source for
the LGPL-covered parts is this repository itself; every published web bundle can
be rebuilt from it with:

```sh
rustup target add wasm32-unknown-unknown
cargo install trunk --locked
trunk build            # Trunk.toml already pins --no-default-features
```

`crates/fdm` is an independent crate, so it can be replaced or modified and the
whole bundle relinked by re-running the command above.

## Third-party Rust crates

Bevy and the rest of the dependency tree are MIT / Apache-2.0 licensed; run
`cargo license` or inspect `Cargo.lock` for the per-crate texts.
