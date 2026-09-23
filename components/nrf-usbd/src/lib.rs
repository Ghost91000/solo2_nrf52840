//! USB peripheral driver for nRF microcontrollers.

#![no_std]
// Vendored third-party crate. The workspace compiles *everything* with
// `-D warnings` (root .cargo/config.toml), which is aimed at our own code; this
// unmodified 2021 upstream release trips newer rustc lints (e.g.
// `mismatched_lifetime_syntaxes`). Silence linting here instead of rewriting
// upstream code — our only change is the EP0 fix in `usbd.rs`.
#![allow(warnings)]

mod errata;
pub mod trace;
mod pac;
mod usbd;

pub use usbd::Usbd;

/// A trait for device-specific USB peripherals. Implement this to add support for a new hardware
/// platform. Peripherals that have this trait must have the same register block as NRF52 USBD
/// peripherals.
pub unsafe trait UsbPeripheral: Send {
    /// Pointer to the register block
    const REGISTERS: *const ();
}
