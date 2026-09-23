//! Диагностика без отладчика: «язык» мигания светодиодом.
//!
//! В этой прошивке нет USB CDC, а RTT доступен только через J-Link, поэтому
//! единственный канал наблюдаемости — светодиод. Помощники ниже работают из
//! любого контекста (обработчик прерывания, обработчик паники, ранний код до
//! старта RTIC), где ресурсы RTIC недоступны, и не требуют экземпляра `Leds`.
//!
//! Пин зашит: P0.15 — это светодиод и на nRF52840-DK (LED3), и на
//! nice!nano-совместимых платах (синий LED). У DK светодиоды включаются
//! низким уровнем, у клона — высоким; полярность учтена.

use core::sync::atomic::{AtomicBool, Ordering};

use nrf52840_pac::P0;

/// Видели ли событие USB-reset (хост перезапускал шину).
pub static SAW_USB_RESET: AtomicBool = AtomicBool::new(false);
/// Видели ли EP0 SETUP: хост действительно запрашивает дескрипторы.
pub static SAW_EP0_SETUP: AtomicBool = AtomicBool::new(false);
/// Завершали ли передачу данных через EP0 (значит, ответ ушёл хосту).
pub static SAW_EP0DATADONE: AtomicBool = AtomicBool::new(false);

/// Отметить «сырые» события USBD *до* того, как драйвер их погасит.
/// Вызывается и из обработчика прерывания USBD, и из idle-цикла перед
/// собственным `poll()` — кто первый, тот и увидит событие.
#[inline]
pub fn note_usb_events() {
    let r = unsafe { &*nrf52840_pac::USBD::ptr() };
    if r.events_usbreset.read().events_usbreset().bit_is_set() {
        SAW_USB_RESET.store(true, Ordering::Relaxed);
    }
    if r.events_ep0setup.read().events_ep0setup().bit_is_set() {
        SAW_EP0_SETUP.store(true, Ordering::Relaxed);
    }
    if r.events_ep0datadone
        .read()
        .events_ep0datadone()
        .bit_is_set()
    {
        SAW_EP0DATADONE.store(true, Ordering::Relaxed);
    }
}

/// Бит P0.15 в регистрах OUT/OUTSET/OUTCLR.
const LED_BIT: u32 = 1 << 15;

/// Занят ли светодиод диагностикой: как только прошивка мигнула своим узором,
/// штатный индикатор (`refresh_up_led`) замолкает, чтобы не смешивать каналы.
pub static LED_CHANNEL_RESERVED: AtomicBool = AtomicBool::new(false);

/// У DK светодиоды active-low, у nice!nano-совместимых плат — active-high.
#[cfg(feature = "board-dk")]
const LED_ACTIVE_HIGH: bool = false;
#[cfg(not(feature = "board-dk"))]
const LED_ACTIVE_HIGH: bool = true;

/// Принудительно сделать P0.15 выходом — нужно, если мигать приходится до
/// `board::init()` (например паника в самом раннем коде).
#[inline]
pub fn led_force_output() {
    unsafe {
        let p0 = &*P0::ptr();
        p0.pin_cnf[15].write(|w| {
            w.dir()
                .output()
                .input()
                .disconnect()
                .pull()
                .disabled()
                .drive()
                .s0s1()
        });
    }
}

/// Зажечь (`on = true`) или погасить светодиод в обход `Leds`.
/// Заодно «бронирует» канал за диагностикой — см. `LED_CHANNEL_RESERVED`.
#[inline]
pub fn led_raw(on: bool) {
    LED_CHANNEL_RESERVED.store(true, Ordering::Relaxed);
    let phys_high = on == LED_ACTIVE_HIGH;
    unsafe {
        let p0 = &*P0::ptr();
        if phys_high {
            p0.outset.write(|w| w.bits(LED_BIT));
        } else {
            p0.outclr.write(|w| w.bits(LED_BIT));
        }
    }
}

/// Пауза в циклах CPU (64 МГц) — только для узоров на светодиоде.
#[inline]
pub fn spin(cycles: u32) {
    cortex_m::asm::delay(cycles);
}

/// Мигнуть `times` раз: `on_cycles` горит, `off_cycles` пауза.
#[inline]
pub fn blink_raw(times: u32, on_cycles: u32, off_cycles: u32) {
    led_force_output();
    for _ in 0..times {
        led_raw(true);
        spin(on_cycles);
        led_raw(false);
        spin(off_cycles);
    }
}

/// «Продиктовать» одну цифру: `d` быстрых мигов и длинная пауза.
/// `d == 0` — пауза без мигов (пустая группа читается как ноль).
#[inline]
pub fn blink_digit(d: u32) {
    blink_raw(d, 4_000_000, 4_000_000);
    spin(96_000_000);
}

// ── Неблокируемый доклад о трассе EP0 ───────────────────────────────────────
//
// Драйвер считает таймаут EP0-IN по пяти SOF-кадрам (5 мс), поэтому блокировать
// цикл `spin`-паузами нельзя: «пульс» из 4 мигов = 500 мс без опроса USB и
// сорванный обмен. Доклад работает по слотам: `report_tick` вызывается в idle
// каждый проход, сам считает время и трогает GPIO на микросекунды.

use core::sync::atomic::{AtomicBool as BoolFlag, AtomicU32};

// ── Доклад о трассе EP0 ──────────────────────────────────────────────────────
//
// Протокол чтения (отладчика нет, светодиод — единственный канал):
//   • 3 с непрерывного горения — начало сообщения;
//   • число — N коротких вспышек (300 мс горения / 300 мс паузы);
//   • 1 с тишины отделяет числа друг от друга.
// Числа идут в фиксированном порядке (см. `report_values`). Ноль читается как
// удвоенная тишина: между двумя разделителями вспышек нет.
//
// Всё считается по монотонику в idle-цикле и НЕ блокирует его: драйвер считает
// таймаут EP0-IN по 5 SOF-кадрам (5 мс), поэтому `spin`-паузы здесь запрещены.

/// 3 с горения — начало сообщения.
const MSG_ON_MS: u32 = 3000;
/// Длительность вспышки и паузы между вспышками внутри числа.
const BLINK_MS: u32 = 300;
/// Длительность длинной вспышки: так кодируется ноль (900 мс).
const LONG_MS: u32 = 900;
/// Тишина между числами.
const NUM_GAP_MS: u32 = 1000;

const PH_START: u32 = 0; // горит 3 с: начало сообщения
const PH_ON: u32 = 1; // вспышка
const PH_OFF: u32 = 2; // пауза между вспышками
const PH_GAP: u32 = 3; // тишина между числами

static RPT_STARTED: BoolFlag = BoolFlag::new(false);
static RPT_T0: AtomicU32 = AtomicU32::new(0);
static RPT_PHASE: AtomicU32 = AtomicU32::new(PH_START);
static RPT_NUM: AtomicU32 = AtomicU32::new(0);
static RPT_LEFT: AtomicU32 = AtomicU32::new(0);
/// Текущее число кодируется длинной вспышкой (это ноль).
static RPT_LONG: BoolFlag = BoolFlag::new(false);

/// Что показываем. Счётчики обрезаны до 3 («3» = три и больше), `bRequest` —
/// точный: 6 = GET_DESCRIPTOR, 5 = SET_ADDRESS, 9 = SET_CONFIGURATION.
///   [0] попытки стека отдать данные хосту через EP0;
///   [1] из них отказ по busy-флагу драйвера (WouldBlock);
///   [2] из них отказ по `EPSTATUS.EPIN[0]` — «железо занято» (WouldBlock);
///   [3] из них успешно отданы периферии;
///   [4] сколько раз периферия доложила об окончании передачи (байты ушли);
///   [5] последний `bRequest` запроса хоста.
pub fn report_values() -> [u32; 6] {
    use nrf_usbd::trace;
    [
        trace::get(&trace::WRITE_EP0_CALLS).min(3),
        trace::get(&trace::WRITE_EP0_BUSY_BIT).min(3),
        trace::get(&trace::WRITE_EP0_EPSTATUS).min(3),
        trace::get(&trace::WRITE_EP0_OK).min(3),
        trace::get(&trace::EP0_DATADONE_IN).min(3),
        trace::get(&trace::LAST_SETUP_REQ).min(9),
    ]
}

/// Неблокирующий шаг доклада. Вызывается каждый проход idle-цикла; всё, что он
/// делает — сравнивает монотоник с моментом старта текущей фазы.
pub fn report_tick(now_ms: u32) {
    if !RPT_STARTED.load(Ordering::Relaxed) {
        RPT_STARTED.store(true, Ordering::Relaxed);
        RPT_NUM.store(0, Ordering::Relaxed);
        RPT_PHASE.store(PH_START, Ordering::Relaxed);
        RPT_T0.store(now_ms, Ordering::Relaxed);
        led_raw(true);
        return;
    }

    let elapsed = now_ms.wrapping_sub(RPT_T0.load(Ordering::Relaxed));
    let phase = RPT_PHASE.load(Ordering::Relaxed);
    let left = RPT_LEFT.load(Ordering::Relaxed);
    // Ноль кодируется одной длинной вспышкой, а не тишиной: тишину нельзя
    // отличить от разделителя, а длинную вспышку — можно.
    let dur = if RPT_LONG.load(Ordering::Relaxed) {
        LONG_MS
    } else {
        BLINK_MS
    };

    match phase {
        // Конец 3-секундного маркера — секунда тишины, чтобы первая вспышка
        // первого числа не слилась с маркером.
        PH_START => {
            if elapsed >= MSG_ON_MS {
                RPT_PHASE.store(PH_GAP, Ordering::Relaxed);
                RPT_T0.store(now_ms, Ordering::Relaxed);
                led_raw(false);
            }
        }
        PH_ON => {
            if elapsed >= dur {
                RPT_PHASE.store(PH_OFF, Ordering::Relaxed);
                RPT_T0.store(now_ms, Ordering::Relaxed);
                led_raw(false);
            }
        }
        PH_OFF => {
            if elapsed >= BLINK_MS {
                if left > 0 {
                    RPT_LEFT.store(left - 1, Ordering::Relaxed);
                    RPT_PHASE.store(PH_ON, Ordering::Relaxed);
                    RPT_T0.store(now_ms, Ordering::Relaxed);
                    led_raw(true);
                } else {
                    RPT_LONG.store(false, Ordering::Relaxed);
                    RPT_PHASE.store(PH_GAP, Ordering::Relaxed);
                    RPT_T0.store(now_ms, Ordering::Relaxed);
                    led_raw(false);
                }
            }
        }
        // Тишина кончилась: либо следующее число, либо новый кадр.
        _ => {
            if elapsed >= NUM_GAP_MS {
                let vals = report_values();
                let idx = RPT_NUM.load(Ordering::Relaxed) as usize;
                if idx >= vals.len() {
                    RPT_NUM.store(0, Ordering::Relaxed);
                    RPT_PHASE.store(PH_START, Ordering::Relaxed);
                    RPT_T0.store(now_ms, Ordering::Relaxed);
                    led_raw(true);
                } else {
                    RPT_NUM.store((idx + 1) as u32, Ordering::Relaxed);
                    start_number(vals[idx], now_ms);
                }
            }
        }
    }
}

/// Начать число: ноль — одна длинная вспышка, иначе N коротких.
fn start_number(n: u32, now_ms: u32) {
    RPT_T0.store(now_ms, Ordering::Relaxed);
    if n == 0 {
        RPT_LONG.store(true, Ordering::Relaxed);
        RPT_LEFT.store(0, Ordering::Relaxed);
        RPT_PHASE.store(PH_ON, Ordering::Relaxed);
        led_raw(true);
    } else {
        RPT_LONG.store(false, Ordering::Relaxed);
        RPT_LEFT.store(n - 1, Ordering::Relaxed);
        RPT_PHASE.store(PH_ON, Ordering::Relaxed);
        led_raw(true);
    }
}
