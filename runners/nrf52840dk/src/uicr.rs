//! UICR: чтение и починка `NFCPINS` — возврат пинов P0.09/P0.10 под NFCT.
//!
//! Проблема: предыдущая прошивка (ZMK/QMK на Zephyr) может выставить
//! `NFCPINS.PROTECT = 0`, и тогда P0.09/P0.10 работают как обычные GPIO, а
//! NFCT ими не управляет — NFC не заработает ни при какой катушке.
//!
//! Сложность в том, что UICR — это флеш, и стирается он только целой
//! страницей. Порядок поэтому такой же, как в Zephyr: прочитать все слова
//! UICR в RAM, стереть страницу, записать обратно всё, что было записано,
//! выставив `NFCPINS = 1` (NFC).
//!
//! Почему это безопасно на нашей плате:
//!   * флеш не трогается — теряется только UICR; бутлоадер и UF2 остаются
//!     рабочими, откат возможен двойным тапом RST;
//!   * `REGOUT0` стирается в DEFAULT (1.8 В), но на этой плате выход
//!     регулятора не используется: питание 3.3 В идёт прямо на VDD;
//!   * `PSELRESET` стирается, а для nRF52840 это штатный nRESET на P0.18 —
//!     как раз тот RST-пятак, что есть на плате;
//!   * `APPROTECT` после стирания остаётся выключенным, SWD не теряем.
//!
//! Модуль подключается под фичами `nfc-diag` / `nfc-fix` и в боевую
//! прошивку не попадает.

#[cfg(feature = "nfc-fix")]
use nrf52840_pac::NVMC;
use nrf52840_pac::UICR;

/// Слов в странице UICR: 1 КБ (`0x10001000..0x10001400`).
#[cfg(feature = "nfc-fix")]
const UICR_WORDS: usize = 0x400 / 4;
/// Слово, в котором лежит `NFCPINS` (offset 0x20C).
#[cfg(feature = "nfc-fix")]
const NFCPINS_WORD: usize = 0x20C / 4;
/// Значение стёртого слова флеша.
#[cfg(feature = "nfc-fix")]
const ERASED: u32 = 0xFFFF_FFFF;

#[cfg(feature = "nfc-fix")]
fn nvmc() -> &'static nrf52840_pac::nvmc::RegisterBlock {
    unsafe { &*NVMC::ptr() }
}

/// Ждём, пока NVMC закончит операцию. `false` — не дождались (таймаут).
#[cfg(feature = "nfc-fix")]
fn wait_ready() -> bool {
    for _ in 0..4_000_000 {
        if nvmc().ready.read().ready().is_ready() {
            return true;
        }
    }
    false
}

#[cfg(feature = "nfc-fix")]
fn mode_erase() -> bool {
    nvmc().config.write(|w| w.wen().een());
    wait_ready()
}

#[cfg(feature = "nfc-fix")]
fn mode_write() -> bool {
    nvmc().config.write(|w| w.wen().wen());
    wait_ready()
}

#[cfg(feature = "nfc-fix")]
fn mode_read() {
    nvmc().config.write(|w| w.wen().ren());
}

#[cfg(feature = "nfc-fix")]
fn erase_uicr() -> bool {
    if !mode_erase() {
        return false;
    }
    nvmc().eraseuicr.write(|w| w.eraseuicr().erase());
    let ok = wait_ready();
    mode_read();
    ok
}

/// true — пины P0.09/P0.10 отданы NFCT (`NFCPINS.PROTECT = 1`; так же читается
/// и полностью стёртый UICR, потому что reset-значение 0xFFFFFFFF).
pub fn nfcpins_is_nfc() -> bool {
    unsafe { &*UICR::ptr() }.nfcpins.read().protect().is_nfc()
}

/// Один проход починки. Код возврата — для доклада миганиями:
///   1 — уже NFC, ничего не делали;
///   2 — починено: UICR стёрт и записан заново с `NFCPINS = NFC`;
///   3 — UICR был пуст, выставили только `NFCPINS`;
///   4 — ошибка NVMC (стирание или запись не подтвердились).
///
/// Перезагрузку не делаем намеренно: NFCPINS действует со следующего сброса,
/// а автосброс при неудаче дал бы цикл перезагрузок.
#[cfg(feature = "nfc-fix")]
pub fn fix_nfcpins() -> u32 {
    if nfcpins_is_nfc() {
        return 1;
    }

    // Образ UICR: после стирания возвращаем всё, что было записано (NRFFW,
    // NRFHW, CUSTOMER, PSELRESET, APPROTECT, DEBUGCTRL, REGOUT0), а NFCPINS
    // переопределяем в NFC.
    let mut words = [ERASED; UICR_WORDS];
    let src = UICR::ptr() as *const u32;
    for (i, w) in words.iter_mut().enumerate() {
        *w = unsafe { core::ptr::read_volatile(src.add(i)) };
    }
    let had_content = words.iter().any(|w| *w != ERASED);
    words[NFCPINS_WORD] = 1;

    if had_content && !erase_uicr() {
        return 4;
    }
    if !mode_write() {
        return 4;
    }
    let dst = UICR::ptr() as *mut u32;
    for (i, w) in words.iter().enumerate() {
        if *w != ERASED {
            unsafe { core::ptr::write_volatile(dst.add(i), *w) };
        }
    }
    let ok = wait_ready();
    mode_read();

    if !ok || !nfcpins_is_nfc() {
        return 4;
    }
    if had_content { 2 } else { 3 }
}
