//! Трассировка путей EP0 внутри драйвера.
//!
//! Это ВЕНДОРЕННАЯ копия `nrf-usbd` — модуль наш, в апстриме его нет.
//! Отладчика нет, единственный канал наблюдаемости — светодиод, поэтому счётчики
//! держим в статиках, а раннер их читает и «диктует» миганием.
//!
//! Пишем только `Relaxed`-атомики: ни блокировок, ни пауз, вызовы безопасны
//! из обработчика прерывания USBD.

use core::sync::atomic::{AtomicU32, Ordering};

/// Сколько раз `poll()` драйвера видел установленный `EVENTS_EP0SETUP`.
pub static POLL_SETUP_SEEN: AtomicU32 = AtomicU32::new(0);
/// Сколько раз стек запросил у драйвера разбор setup-пакета.
pub static READ_SETUP_CALLS: AtomicU32 = AtomicU32::new(0);
/// Последний `bmRequestType` (0x80 = IN, standard, device).
pub static LAST_SETUP_TYPE: AtomicU32 = AtomicU32::new(0);
/// Последний `bRequest` (6 = GET_DESCRIPTOR, 5 = SET_ADDRESS, 9 = SET_CONFIG).
pub static LAST_SETUP_REQ: AtomicU32 = AtomicU32::new(0);
/// Последний `wValue` (для GET_DESCRIPTOR: старший байт = тип дескриптора).
pub static LAST_SETUP_VALUE: AtomicU32 = AtomicU32::new(0);
/// Последний `wLength`.
pub static LAST_SETUP_LEN: AtomicU32 = AtomicU32::new(0);
/// Вызовы `write()` для EP0 IN — то есть попытки ответить хосту.
pub static WRITE_EP0_CALLS: AtomicU32 = AtomicU32::new(0);
/// Из них успешных.
pub static WRITE_EP0_OK: AtomicU32 = AtomicU32::new(0);
/// Из них `WouldBlock` — «EP0 IN занят».
pub static WRITE_EP0_BUSY: AtomicU32 = AtomicU32::new(0);
/// Из них отказ по busy-флагу драйвера (`WouldBlock`).
pub static WRITE_EP0_BUSY_BIT: AtomicU32 = AtomicU32::new(0);
/// Из них отказ по `EPSTATUS.EPIN[0]` — «железо занято» (`WouldBlock`).
pub static WRITE_EP0_EPSTATUS: AtomicU32 = AtomicU32::new(0);
/// Из них прочие ошибки.
pub static WRITE_EP0_ERR: AtomicU32 = AtomicU32::new(0);
/// Длина последней попытки записать в EP0.
pub static WRITE_EP0_LAST_LEN: AtomicU32 = AtomicU32::new(0);
/// Сколько раз путь таймаута EP0-IN отправлял лишний status stage.
pub static EP0_TIMEOUT_STATUS: AtomicU32 = AtomicU32::new(0);
/// Сколько раз драйвер видел USB-reset.
pub static RESETS: AtomicU32 = AtomicU32::new(0);

/// Попытки записать в EP0, которые не влезли в буфер (`BufferOverflow`).
pub static WRITE_EP0_OVERFLOW: AtomicU32 = AtomicU32::new(0);
/// Сколько раз периферия доложила об окончании передачи данных из EP0 (IN).
/// Это и есть доказательство, что байты реально ушли на шину.
pub static EP0_DATADONE_IN: AtomicU32 = AtomicU32::new(0);

/// Увеличить счётчик (без переполнения).
#[inline]
pub fn bump(c: &AtomicU32) {
    c.store(c.load(Ordering::Relaxed).saturating_add(1), Ordering::Relaxed);
}

/// Прочитать счётчик (раннер показывает его светодиодом).
#[inline]
pub fn get(c: &AtomicU32) -> u32 {
    c.load(Ordering::Relaxed)
}
