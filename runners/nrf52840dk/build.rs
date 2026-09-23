use std::env;
use std::fs::File;
use std::io::Write;
use std::path::PathBuf;

/// `memory.x` is gitignored upstream (the layout depends on the board and its
/// bootloader), so a suitable file must exist locally. The SuperMini port uses
/// its own `memory.x.supermini`; every other board keeps using `memory.x`.
fn memory_source() -> &'static str {
    if env::var_os("CARGO_FEATURE_BOARD_SUPERMINI").is_some() {
        "memory.x.supermini"
    } else {
        "memory.x"
    }
}

fn main() {
    let out = PathBuf::from(env::var_os("OUT_DIR").unwrap());
    let source = memory_source();
    let bytes = std::fs::read(source).unwrap_or_else(|_| {
        panic!("{source} not found — it is gitignored upstream; create it for your board layout")
    });
    File::create(out.join("memory.x"))
        .unwrap()
        .write_all(&bytes)
        .unwrap();
    println!("cargo:rustc-link-search={}", out.display());
    println!("cargo:rerun-if-changed={source}");
    println!("cargo:rerun-if-changed=build.rs");
}
