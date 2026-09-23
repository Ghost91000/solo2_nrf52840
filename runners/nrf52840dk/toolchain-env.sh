#!/usr/bin/env bash
# Machine-specific toolchain paths for building this runner on Windows
# (Rust with the GNU host + MinGW linker + xpack ARM GCC + pip libclang).
#
# Why each piece is needed:
#   * arm-none-eabi-gcc  — `littlefs2-sys` compiles the littlefs C sources.
#   * libclang.dll       — `littlefs2-sys` also *generates* its bindings with
#                          bindgen, which needs libclang on the host.
#   * MinGW-w64 gcc      — host linker for build scripts / proc-macros (the
#                          rustup host toolchain is x86_64-pc-windows-gnu).
#   * BINDGEN_EXTRA_CLANG_ARGS — points clang at the ARM toolchain headers, so
#                          the bindgen pass can find <stdint.h> for the target.
#
# Source it (`source toolchain-env.sh`) or just run ./build-uf2.sh, which does.

export MINGW_BIN="/c/Users/dude/AppData/Local/Microsoft/WinGet/Packages/BrechtSanders.WinLibs.POSIX.UCRT_Microsoft.Winget.Source_8wekyb3d8bbwe/mingw64/bin"
export ARM_GCC="/c/Users/dude/tools/xpack-arm-none-eabi-gcc-15.2.1-1.1"
export RUSTLIB="$HOME/.rustup/toolchains/1.94.1-x86_64-pc-windows-gnu/lib/rustlib"

export PATH="$ARM_GCC/bin:$MINGW_BIN:$HOME/.cargo/bin:$RUSTLIB/x86_64-pc-windows-gnu/bin:$RUSTLIB/thumbv7em-none-eabihf/bin:$PATH"

# The `cc` crate looks these up under the target triple with underscores.
export CC_thumbv7em_none_eabihf=arm-none-eabi-gcc
export AR_thumbv7em_none_eabihf=arm-none-eabi-ar

# bindgen (littlefs2-sys): native path, and the target's C headers.
export LIBCLANG_PATH="C:/Users/dude/tools/libclang/clang/native"
export BINDGEN_EXTRA_CLANG_ARGS="--target=thumbv7em-none-eabihf -isystem C:/Users/dude/tools/xpack-arm-none-eabi-gcc-15.2.1-1.1/lib/gcc/arm-none-eabi/15.2.1/include -isystem C:/Users/dude/tools/xpack-arm-none-eabi-gcc-15.2.1-1.1/arm-none-eabi/include"
