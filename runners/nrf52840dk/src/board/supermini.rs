//! SuperMini nRF52840 pin definitions + board-specific HAL.
//!
//! Hardware: nice!nano-compatible ProMicro-footprint clone (nologo.tech
//! "SuperMini" / nRF52840 module E73-2G4M08S1C), stock nice!nano (Adafruit)
//! UF2 bootloader with SoftDevice S140 at 0x1000..0x26000.
//!
//! Pinout (nice!nano mapping, ProMicro header numbering per joric/nrfmicro):
//!   LED        = P0.15   (PIN?/on-board blue, ACTIVE-HIGH — same pin and
//!                         polarity as the bootloader's LED_PRIMARY_PIN)
//!   Button 1   = P0.11   (PIN10, "E6"; active-low, internal pull-up)
//!   Button 3   = P0.24   (PIN8,  "C6"; active-low, internal pull-up)
//!   NFC1/NFC2  = P0.09 / P0.10 (PIN13/PIN14, dedicated NFCT pins — owned by
//!                         the NFCT peripheral, never claimed as GPIO here)
//!   P0.13      = EXT_VCC control on the nice!nano layout — left untouched
//!
//! No capacitive-touch pads: user presence is a real button. With no button
//! soldered, short PIN10 (P0.11) to GND — a single press-and-release counts
//! as Approve, a press of PIN8 (P0.24) as explicit Deny.
//!
//! Deliberately NOT touched: P0.13 (EXT_VCC), P0.31 (battery divider on the
//! nice!nano layout), P0.18 (reset), P0.09/P0.10 (NFC).

use embedded_hal::digital::v2::{InputPin, OutputPin};
use nrf52840_hal::gpio::{p0, Input, Level, Output, Pin, PullUp, PushPull};
use nrf52840_pac::P0;

/// The single on-board LED, active-high: pin HIGH = on.
pub struct Leds {
    led: Pin<Output<PushPull>>,
}

impl Leds {
    pub fn set_brightness(&mut self, b: u8) {
        let _ = if b >= 128 {
            self.led.set_high() // active-high: HIGH = on
        } else {
            self.led.set_low() // LOW = off
        };
    }
}

pub struct Buttons {
    pub btn1: Pin<Input<PullUp>>,
    pub btn3: Pin<Input<PullUp>>,
}

impl Buttons {
    /// "Left" approve source: Btn1 (P0.11). Release commits the gesture, so a
    /// single press-and-release approves.
    pub fn left(&self) -> bool {
        self.btn1.is_low().unwrap_or(false)
    }
    /// No second approve button on this board — the both-held deny gesture is
    /// therefore unreachable, which is fine: Btn3 is the explicit deny.
    pub fn right(&self) -> bool {
        false
    }
    /// Explicit deny shortcut: Btn3 (P0.24).
    pub fn explicit_deny(&self) -> bool {
        self.btn3.is_low().unwrap_or(false)
    }
}

/// Take ownership of the P0 GPIO bank, configure the LED and the buttons.
pub fn init(p0_periph: P0) -> (Leds, Buttons) {
    // Гасим ВСЕ экземпляры ШИМ и отключаем их выходы. Загрузчик настраивает
    // аппаратный ШИМ для светодиода (`led_pwm_init`, «use PWM0 for LED RED» в
    // bl_boards.c) и перед прыжком в приложение его НЕ снимает — ШИМ продолжает
    // мигать сам, без участия процессора. Какой экземпляр он занял, из кода не
    // видно, поэтому гасим PWM0..PWM3 целиком: нашей прошивке ШИМ не нужен
    // (NFC идёт через NFCT + TIMER4). Наша правка, в апстриме её нет.
    macro_rules! kill_pwm {
        ($p:ty) => {{
            let r = unsafe { &*<$p>::ptr() };
            r.enable.write(|w| w.enable().disabled());
            for ch in 0..4 {
                r.psel.out[ch].write(|w| unsafe { w.bits(0x8000_0000) }); // PSEL: disconnected
            }
        }};
    }
    kill_pwm!(nrf52840_pac::PWM0);
    kill_pwm!(nrf52840_pac::PWM1);
    kill_pwm!(nrf52840_pac::PWM2);
    kill_pwm!(nrf52840_pac::PWM3);

    let parts = p0::Parts::new(p0_periph);

    let mut leds = Leds {
        led: parts.p0_15.into_push_pull_output(Level::Low).degrade(),
    };

    // Boot indication: 3 quick blinks, so "firmware alive" is visible without
    // a debug probe.
    const BLINK_CYCLES: u32 = 64_000_000 / 5 / 4; // ~200 ms at 64 MHz
    for _ in 0..3 {
        leds.set_brightness(255);
        cortex_m::asm::delay(BLINK_CYCLES);
        leds.set_brightness(0);
        cortex_m::asm::delay(BLINK_CYCLES);
    }

    let buttons = Buttons {
        btn1: parts.p0_11.into_pullup_input().degrade(),
        btn3: parts.p0_24.into_pullup_input().degrade(),
    };

    (leds, buttons)
}
