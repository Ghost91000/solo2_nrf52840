//! Board glue: trussed UserInterface + Syscall (shared); per-board pin
//! defs + HAL handles live in [`dk`] / [`solo`] (compiled in via the
//! `board-dk` / `board-solo` Cargo feature, mutually exclusive).
//!
//! Buttons and the user-presence LED live in the idle loop, not the
//! `UserInterface`: `poll_buttons` latches a gesture and `refresh_up_led`
//! lights LED4 while a sign or FIDO op waits for presence.

// Диагностика без отладчика (мигание светодиодом) — общая для всех плат,
// потому что вызывается из обработчиков прерываний/паники и раннего кода.
#[allow(dead_code)] // часть функций живёт только под фичей `led-diag`
mod diag;
pub use diag::{blink_digit, blink_raw, spin};
// Доклад о трассе EP0 нужен только сборкам с `led-diag`.
#[cfg(feature = "led-diag")]
pub use diag::{note_usb_events, report_tick};

#[cfg(feature = "board-dk")]
pub mod dk;
#[cfg(feature = "board-dk")]
pub use dk::{init, Buttons, Leds};

#[cfg(feature = "board-solo")]
pub mod solo;
#[cfg(feature = "board-solo")]
pub use solo::{init, Buttons, Leds};

#[cfg(feature = "board-supermini")]
pub mod supermini;
#[cfg(feature = "board-supermini")]
pub use supermini::{init, Buttons, Leds};

#[cfg(any(
    all(feature = "board-dk", feature = "board-solo"),
    all(feature = "board-dk", feature = "board-supermini"),
    all(feature = "board-solo", feature = "board-supermini")
))]
compile_error!("`board-dk`, `board-solo` and `board-supermini` are mutually exclusive — pick one");

#[cfg(not(any(
    feature = "board-dk",
    feature = "board-solo",
    feature = "board-supermini"
)))]
compile_error!("enable one of `board-dk` / `board-solo` / `board-supermini` (default = `board-dk`)");

use core::time::Duration;
use cortex_m::peripheral::SCB;
use trussed::platform::{consent, reboot, ui};

#[derive(Default)]
pub struct Syscall;

impl trussed::client::Syscall for Syscall {
    #[inline]
    fn syscall(&mut self) {
        rtic::pend(nrf52840_pac::Interrupt::SWI0_EGU0);
    }
}

// ── Gesture detector (shared, board-agnostic) ────────────────────────────────
//
// Per-board `Buttons` exposes three primitive signals — left, right,
// explicit_deny. The detector commits a gesture on RELEASE so it can
// distinguish "single press → approve" from "both held → deny".
// Explicit-deny fires immediately on press, debounced one-shot.

#[derive(Copy, Clone, PartialEq, Eq)]
pub enum Gesture {
    None,
    Approve,
    Deny,
}

const DEBOUNCE_MS: u32 = 30;
/// Minimum spacing between cap-touch reads. Each `buttons.{left,right,
/// explicit_deny}()` is a ~23 ms floating-pad read on the bare DK; `poll_buttons`
/// runs every idle pass, so without throttling the reads would starve the
/// idle/NFC loop. Cache the three booleans in between.
const CAP_THROTTLE_MS: u32 = 50;

pub struct GestureDetector {
    pending_started_ms: Option<u32>,
    both_seen: bool,
    last_explicit_deny: bool,
}

impl Default for GestureDetector {
    fn default() -> Self {
        Self::new()
    }
}

impl GestureDetector {
    pub const fn new() -> Self {
        Self {
            pending_started_ms: None,
            both_seen: false,
            last_explicit_deny: false,
        }
    }

    pub fn poll(&mut self, left: bool, right: bool, explicit_deny: bool, now_ms: u32) -> Gesture {
        if explicit_deny && !self.last_explicit_deny {
            self.last_explicit_deny = true;
            self.pending_started_ms = None;
            self.both_seen = false;
            return Gesture::Deny;
        }
        if !explicit_deny {
            self.last_explicit_deny = false;
        }

        match (left, right) {
            (false, false) => {
                let result = match self.pending_started_ms {
                    Some(start) if now_ms.wrapping_sub(start) >= DEBOUNCE_MS => {
                        if self.both_seen {
                            Gesture::Deny
                        } else {
                            Gesture::Approve
                        }
                    }
                    _ => Gesture::None,
                };
                self.pending_started_ms = None;
                self.both_seen = false;
                result
            }
            (l, r) => {
                if self.pending_started_ms.is_none() {
                    self.pending_started_ms = Some(now_ms);
                }
                if l && r {
                    self.both_seen = true;
                }
                Gesture::None
            }
        }
    }
}

// ── Latched button-gesture global ────────────────────────────────────────────
//
// The buttons live outside trussed's `UserInterface`: the idle loop polls them
// via `poll_buttons` and latches any committed Approve/Deny here. Both
// consumers — `check_user_presence` (FIDO, called from trussed's UP loop) and
// `confirm_user_present_non_blocking` (wallet) — drain it via `take_gesture`
// (one-shot consume). They never run concurrently (one consent at a time), so a
// single latch is enough.

use core::sync::atomic::{AtomicBool, AtomicU8, Ordering};

const GESTURE_NONE: u8 = 0;
const GESTURE_APPROVE: u8 = 1;
const GESTURE_DENY: u8 = 2;

static BUTTON_GESTURE: AtomicU8 = AtomicU8::new(GESTURE_NONE);

/// Latch a committed gesture. A fresh Approve/Deny replaces whatever is there;
/// a `None` never overwrites an un-consumed Approve/Deny (so a gesture survives
/// until a consumer drains it).
fn latch_gesture(g: Gesture) {
    match g {
        Gesture::Approve => BUTTON_GESTURE.store(GESTURE_APPROVE, Ordering::Release),
        Gesture::Deny => BUTTON_GESTURE.store(GESTURE_DENY, Ordering::Release),
        Gesture::None => {}
    }
}

/// One-shot consume: read the latched gesture and reset to None.
fn take_gesture() -> Gesture {
    match BUTTON_GESTURE.swap(GESTURE_NONE, Ordering::AcqRel) {
        GESTURE_APPROVE => Gesture::Approve,
        GESTURE_DENY => Gesture::Deny,
        _ => Gesture::None,
    }
}

// ── Shared user-presence INPUT read ──────────────────────────────────────────
//
// The single place that reads the idle-reachable presence inputs (the test
// UP_CONTROL override, the NFC field, and the latched button gesture). Both the
// blocking trussed `check_user_presence` and the non-blocking
// `confirm_user_present_non_blocking` wrap this; neither touches the LED — the
// idle loop owns that (`refresh_up_led`).
enum Presence {
    Grant(consent::Level),
    Deny,
    Pending,
}

fn read_presence() -> Presence {
    #[cfg(feature = "test-up-control")]
    {
        let val = unsafe { core::ptr::read_volatile(&raw const UP_CONTROL) };
        match val {
            1 => {
                unsafe { core::ptr::write_volatile(&raw mut UP_CONTROL, 0) };
                return Presence::Grant(consent::Level::Normal);
            }
            2 => {
                unsafe { core::ptr::write_volatile(&raw mut UP_CONTROL, 0) };
                return Presence::Grant(consent::Level::Strong);
            }
            129 => return Presence::Grant(consent::Level::Normal),
            130 => return Presence::Grant(consent::Level::Strong),
            128 => return Presence::Pending, // deny-sticky = keep waiting
            0 => {}                          // fall through
            _ => return Presence::Pending,
        }
    }
    // ── НАШ ФИКС (в апстриме его нет) ───────────────────────────────────
    // Пока FIDO ждёт касания, trussed крутит свой UP-цикл ВНУТРИ задачи
    // `syscall` (приоритет выше idle), поэтому idle-цикл заморожен: ни
    // `poll_buttons`, ни `refresh_up_led` там не выполняются. Защёлкнутый
    // жест поэтому не появляется, и ожидание висит вечно. Читаем пин
    // напрямую — это работает из любого контекста. Пины с внутренним pull-up
    // (см. board/supermini.rs), активный уровень низкий: замкнуто на GND =
    // нажато. PIN8 (P0.24) = отказ, PIN10 (P0.11) = подтверждение.
    #[cfg(feature = "board-supermini")]
    {
        let pins = unsafe { (&*nrf52840_pac::P0::ptr()).in_.read().bits() };
        if pins & (1 << 24) == 0 {
            return Presence::Deny;
        }
        if pins & (1 << 11) == 0 {
            return Presence::Grant(consent::Level::Normal);
        }
    }

    if crate::nfct::field_on() && !ndef_app::has_override() {
        return Presence::Grant(consent::Level::Normal);
    }
    match take_gesture() {
        Gesture::Approve => Presence::Grant(consent::Level::Normal),
        Gesture::Deny => Presence::Deny,
        Gesture::None => Presence::Pending,
    }
}

/// `now_ms` of the last cap-touch read (idle-loop throttle).
static CAP_LAST_MS: AtomicU32 = AtomicU32::new(0);
/// Cached (left, right, deny) cap-touch read between throttled reads.
static CAP_CACHE: AtomicU8 = AtomicU8::new(0);

use core::sync::atomic::AtomicU32;

/// Idle-loop button poll: throttle the (~23 ms) cap-touch read to once per
/// `CAP_THROTTLE_MS`, run the gesture detector, and latch any committed
/// Approve/Deny. Cheap on the throttled passes (just an atomic compare).
pub fn poll_buttons(buttons: &Buttons, gesture: &mut GestureDetector, now_ms: u32) {
    let last = CAP_LAST_MS.load(Ordering::Relaxed);
    let (left, right, deny) = if now_ms.wrapping_sub(last) >= CAP_THROTTLE_MS {
        let l = buttons.left();
        let r = buttons.right();
        let d = buttons.explicit_deny();
        CAP_CACHE.store(
            (l as u8) | ((r as u8) << 1) | ((d as u8) << 2),
            Ordering::Relaxed,
        );
        CAP_LAST_MS.store(now_ms, Ordering::Relaxed);
        (l, r, d)
    } else {
        let c = CAP_CACHE.load(Ordering::Relaxed);
        (c & 1 != 0, c & 2 != 0, c & 4 != 0)
    };
    let g = gesture.poll(left, right, deny, now_ms);
    latch_gesture(g);
}

// ── Wallet consent: runner-driven, non-blocking user-presence ────────────────
//
// A wallet sign arms `wallet_app::consent` (request) and polls the result. The
// idle loop calls `confirm_user_present_non_blocking(now_ms)` once per pass to fill the result — the
// idle-reachable inputs (the monotonic clock + the NFC field) live here, not in
// the wallet. This is the non-blocking analogue of `check_user_presence` for
// the wallet path only (FIDO keeps using `check_user_presence`).
//
// A button gesture latched by the idle `poll_buttons` is consumed here too: an
// Approve grants, a Deny times out (same global as `check_user_presence`).
#[cfg(feature = "wallet")]
pub fn confirm_user_present_non_blocking(now_ms: u32) {
    use wallet_app::consent;

    // No sign waiting — reset the start sentinel so the next request re-arms.
    if !consent::is_up_requested() {
        CONSENT_START.store(u32::MAX, core::sync::atomic::Ordering::Relaxed);
        return;
    }

    // Record the first-seen instant (sentinel u32::MAX = not yet started).
    let start = match CONSENT_START.compare_exchange(
        u32::MAX,
        now_ms,
        core::sync::atomic::Ordering::Relaxed,
        core::sync::atomic::Ordering::Relaxed,
    ) {
        Ok(_) => now_ms,
        Err(existing) => existing,
    };

    match read_presence() {
        Presence::Grant(_) => consent::set_up_result(consent::GRANTED),
        Presence::Deny => consent::set_up_result(consent::TIMED_OUT),
        Presence::Pending => {
            // 30 s budget (wrapping compare).
            if now_ms.wrapping_sub(start) > 30_000 {
                consent::set_up_result(consent::TIMED_OUT);
            } else {
                consent::set_up_result(consent::WAITING);
            }
        }
    }
}

/// First-seen `now_ms` of the in-flight wallet consent request; `u32::MAX` =
/// none in flight (sentinel).
#[cfg(feature = "wallet")]
static CONSENT_START: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(u32::MAX);

// ── Test hook: probe-rs-driven user-presence override ───────────────────────
//
// Value semantics match `runners/pc/tests/support/up.rs`:
//   0   = no override (fall through to real button polling)
//   1   = approve once (Normal), consumed after one read
//   2   = approve once (Strong), consumed after one read
//   128 = deny sticky (every check returns None until reset to 0)
//   129 = approve sticky (every check returns Normal until reset to 0)
//
// Placed in `.uninit` with `no_mangle` so its address is stable and
// discoverable via the ELF symbol table. NEVER enable this feature on
// firmware shipped to users — it bypasses the real consent gate.
#[cfg(feature = "test-up-control")]
#[unsafe(link_section = ".uninit")]
#[unsafe(no_mangle)]
// 0 = штатное поведение (ждём касание по пину). Значения 1/2/129/130
// работают только в диагностической сборке с фичей `test-up-control`.
pub static mut UP_CONTROL: u8 = 0;

// ── UP-indicator LED, in the idle loop ───────────────────────────────────────
//
// The idle loop owns the `Leds` and drives the "waiting for user presence" LED
// each pass, ORing two sources: trussed's status (`UI_WAITING`, set in
// `set_status` for FIDO) and a wallet sign waiting
// (`wallet_app::consent::is_up_requested()`). It lives here, not in
// `UserInterface`, because a wallet sign waits via the runner-driven signal and
// never reaches trussed's `set_status`.

/// True when trussed wants the UP indicator on (set in `set_status`).
static UI_WAITING: AtomicBool = AtomicBool::new(false);

/// Idle-loop UP-LED refresh: on when trussed is waiting (`UI_WAITING`) or a
/// wallet sign is waiting (`is_up_requested`), else off. Cheap (atomic loads).
pub fn refresh_up_led(leds: &mut Leds) {
    let waiting = UI_WAITING.load(Ordering::Relaxed) || {
        #[cfg(feature = "wallet")]
        {
            wallet_app::consent::is_up_requested()
        }
        #[cfg(not(feature = "wallet"))]
        {
            false
        }
    };
    // Диагностика: пока канал занят узорами (`LED_CHANNEL_RESERVED`), штатный
    // индикатор молчит, иначе он замаскирует диагностические миги.
    if !diag::LED_CHANNEL_RESERVED.load(core::sync::atomic::Ordering::Relaxed) {
        leds.set_brightness(if waiting { 255 } else { 0 });
    }
}

// ── Диагностика разводки кнопок (фича `pin-diag`, наша) ──────────────────────
//
// Зачем: кнопок на плате нет, user presence — это замыкание пина на GND, и
// проверить, что прошивка видит именно тот пин, иначе нечем: логи только по
// RTT с пробником. Здесь светодиод в простое повторяет состояние пинов,
// поэтому проверка не требует ни браузера, ни WebAuthn:
//
//   D7 / PIN10 (P0.11) замкнут на GND → светодиод горит ровно;
//   D5 / PIN8  (P0.24) замкнут на GND → светодиод мигает 5 раз в секунду.
//
// Пока идёт ожидание касания, диагностика молчит: светодиодом в этот момент
// владеет штатная индикация (`set_status` зажигает пин напрямую). Заодно это
// тест подтяжки: если пин «висит в воздухе», светодиод будет мерцать сам —
// значит внутренний pull-up не поднялся.
#[cfg(all(feature = "pin-diag", feature = "board-supermini"))]
pub fn diag_pin_mirror(leds: &mut Leds, now_ms: u32) {
    if UI_WAITING.load(Ordering::Relaxed) {
        return;
    }
    let pins = unsafe { (&*nrf52840_pac::P0::ptr()).in_.read().bits() };
    let approve = pins & (1 << 11) == 0; // активный уровень низкий (pull-up)
    let deny = pins & (1 << 24) == 0;
    let on = if deny {
        (now_ms / 100) % 2 == 0 // отказ — частое мигание, отличимо от «ровно горит»
    } else {
        approve
    };
    leds.set_brightness(if on { 255 } else { 0 });
}

// ── Диагностика NFC (фичи `nfc-fix` / `nfc-diag`, наши) ──────────────────────
//
// Состояние UICR.NFCPINS читает `crate::uicr` (PAC его моделирует, поэтому там
// типизированное чтение). Если пины P0.09/P0.10 отобраны под GPIO предыдущей
// прошивкой (ZMK/QMK на Zephyr), NFC не заработает ни при какой катушке —
// поэтому это проверяется до намотки.
//
// Показываем БЕЗ мигов и кодов, ровно в одном бите: «горит или нет». Считать
// миги на плате без кнопок и без консоли слишком легко ошибиться, а различить
// надо всего два состояния. У каждой сборки свой ОДИН вопрос:
//
//   `nfc-fix` без `nfc-diag` — «пины P0.09/P0.10 отданы NFCT?»
//       горит ровно → да: можно мотать катушку и заливать сборку поля;
//       не горит     → нет: UICR всё ещё в режиме GPIO (правка не прошла).
//       Светодиод статичен и на поднесённый телефон НЕ реагирует.
//
//   `nfc-diag` (собирается вместе с `nfc-fix`) — «есть ли поле считывателя?»
//       горит, пока телефон рядом → катушка и контур работают;
//       не горит → поля нет: катушки нет, контур расстроен или пины не NFC.
//       Светодиод реагирует на телефон — этим сборка и отличается от первой.
//
// Оба признака идут через `diag::led_raw`: он бронирует канал, поэтому штатная
// индикация (`refresh_up_led`) замолкает и не подмешивается в наш сигнал.

/// «Пины P0.09/P0.10 отданы NFCT» — ровным горением, без мигов и счёта.
/// Состояние перепроверяется на каждом проходе idle, но пин трогается только
/// при изменении: так индикация остаётся актуальной после любой правки UICR.
#[cfg(all(feature = "nfc-fix", not(feature = "nfc-diag")))]
pub fn diag_nfc_pins_steady() {
    use core::sync::atomic::{AtomicU8, Ordering as AtomOrd};
    /// 2 — «ещё ничего не показывали»: на первом проходе пин выставляется
    /// в любом случае, включая состояние «не горит».
    static SHOWN: AtomicU8 = AtomicU8::new(2);
    let now = u8::from(crate::uicr::nfcpins_is_nfc());
    if SHOWN.swap(now, AtomOrd::Relaxed) != now {
        diag::led_force_output();
        diag::led_raw(now != 0);
    }
}

/// Диагностика поля: светодиод горит, пока считыватель (телефон) даёт поле.
/// `field_on()` — состояние t4t-стека, который поднимается в `nfct.rs`
/// (`nfc_t4t_emulation_start`). Пока идёт ожидание касания, молчим: в этот
/// момент светодиодом владеет штатная индикация UP-запроса, и она важнее.
#[cfg(all(feature = "nfc-diag", feature = "board-supermini"))]
pub fn diag_nfc_field() {
    if UI_WAITING.load(Ordering::Relaxed) {
        return;
    }
    diag::led_raw(crate::nfct::field_on());
}

// ── Trussed UserInterface ────────────────────────────────────────────────────

pub struct UserInterface {
    status: ui::Status,
}

impl Default for UserInterface {
    fn default() -> Self {
        Self::new()
    }
}

impl UserInterface {
    pub fn new() -> Self {
        Self {
            status: ui::Status::Idle,
        }
    }
}

impl trussed::platform::UserInterface for UserInterface {
    fn check_user_presence(&mut self) -> consent::Level {
        match read_presence() {
            Presence::Grant(level) => level,
            Presence::Deny => {
                // ctaphid-dispatch sets the active app's InterruptFlag
                // to Working before calling us, so the CAS in interrupt()
                // succeeds and trussed's UP loop bails on the next iter.
                crate::types::interrupt_all_apps();
                consent::Level::None
            }
            Presence::Pending => consent::Level::None,
        }
    }

    fn set_status(&mut self, status: ui::Status) {
        self.status = status;
        // Record trussed's wish for the UP indicator; the idle loop
        // (`refresh_up_led`) drives the LED, since the `Leds` live there.
        UI_WAITING.store(
            matches!(status, ui::Status::WaitingForUserPresence),
            Ordering::Relaxed,
        );
        // Тот же корень, что и в `read_presence`: во время ожидания UP idle
        // заморожен, поэтому `refresh_up_led` не успевает. Зажигаем пин
        // светодиода напрямую (P0.15, active-high — см. board/supermini.rs).
        #[cfg(feature = "board-supermini")]
        unsafe {
            let p0 = &*nrf52840_pac::P0::ptr();
            let waiting = matches!(status, ui::Status::WaitingForUserPresence);
            if waiting {
                p0.outset.write(|w| w.bits(1 << 15));
            } else {
                p0.outclr.write(|w| w.bits(1 << 15));
            }
        }
    }

    fn status(&self) -> ui::Status {
        self.status
    }

    fn refresh(&mut self) {}

    fn uptime(&mut self) -> Duration {
        // Read RTIC SysTick monotonic — 64-bit ms count, doesn't wrap.
        // Was DWT cycle counter (32-bit, wraps at ~67 s). Trussed's UP loop
        // computes `nowtime - starttime` which panicked on wrap.
        use rtic_monotonics::Monotonic;
        let now = crate::app::Mono::now();
        Duration::from_millis(u64::from(now.duration_since_epoch().to_millis()))
    }

    fn reboot(&mut self, _to: reboot::To) -> ! {
        SCB::sys_reset()
    }

    fn wink(&mut self, _: Duration) {}
}
