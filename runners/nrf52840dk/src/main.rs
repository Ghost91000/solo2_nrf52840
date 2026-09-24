//! Solo2 firmware port for the Nordic nRF52840-DK.
//!
//! All transports run together: CTAPHID + CCID over USB, plus NFC. Every
//! app (admin/fido/ndef/secrets/piv/opcard) is reachable over both the
//! contact (USB-CCID) and contactless (NFC) interfaces — validated on the
//! DK with CTAP2.3 over NFC while PIV runs over USB-CCID simultaneously.

#![no_std]
#![no_main]

use defmt_rtt as _;

// Свой обработчик паники вместо `panic-halt`: без J-Link единственный способ
// узнать, ГДЕ упало, — «продиктовать» номер строки исходника миганием.
// Формат: 5 медленных мигов (маркер паники) → пауза → группа «сотни» →
// группа «десятки» → группа «единицы» → длинная пауза, по кругу.
// Группа = N быстрых мигов; пустая группа (ноль мигов) = цифра 0.
#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    let line = info.location().map(|l| l.line()).unwrap_or(0);
    loop {
        board::blink_raw(5, 16_000_000, 16_000_000); // маркер: это паника
        board::spin(96_000_000);
        board::blink_digit(line / 100);
        board::blink_digit((line / 10) % 10);
        board::blink_digit(line % 10);
        board::spin(192_000_000);
    }
}

mod board;
// No capacitive-touch pads on the SuperMini/nice!nano footprint, so the
// driver is only compiled for boards that actually have them (the crate
// denies dead_code, and an unused module is an error, not a warning).
#[cfg(not(feature = "board-supermini"))]
mod cap_touch;
mod device_config;
mod dispatch;
mod flash;
mod initializer;
mod nfct;
mod types;
// UICR (починка NFCPINS) — только для диагностики и разовой правки.
#[cfg(any(feature = "nfc-diag", feature = "nfc-fix"))]
mod uicr;

// `nfc-diag` (индикация поля) собирается только вместе с `nfc-fix`: обеим
// фичам нужен `crate::uicr`, а по отдельности модуль остался бы с мёртвым
// кодом, что при `-D warnings` = ошибка сборки. Пусть падает с внятным
// текстом, а не с «unused function».
#[cfg(all(feature = "nfc-diag", not(feature = "nfc-fix")))]
compile_error!("nfc-diag собирается вместе с nfc-fix: FEATURES=board-supermini,nfc-fix,nfc-diag");

// HardFault diagnostic handler. Logs the stacked exception frame +
// fault-status registers via defmt RTT, then halts. Replaces the
// silent `loop {}` cortex-m-rt installs by default, which leaves the
// chip dead with no clue why. flip-link's stack-bottom-fence triggers
// a MemManage fault on stack overflow, which escalates to HardFault —
// so this is also our stack-overflow detector.
#[cortex_m_rt::exception]
unsafe fn HardFault(ef: &cortex_m_rt::ExceptionFrame) -> ! {
    let scb = unsafe { &*cortex_m::peripheral::SCB::PTR };
    let hfsr = scb.hfsr.read();
    let cfsr = scb.cfsr.read();
    let mmfar = scb.mmfar.read();
    let bfar = scb.bfar.read();
    defmt::error!(
        "HardFault! pc={=u32:08x} lr={=u32:08x} r0={=u32:08x} r1={=u32:08x} \
         r2={=u32:08x} r3={=u32:08x} r12={=u32:08x} psr={=u32:08x} | \
         HFSR={=u32:08x} CFSR={=u32:08x} MMFAR={=u32:08x} BFAR={=u32:08x}",
        ef.pc(),
        ef.lr(),
        ef.r0(),
        ef.r1(),
        ef.r2(),
        ef.r3(),
        ef.r12(),
        ef.xpsr(),
        hfsr,
        cfsr,
        mmfar,
        bfar,
    );
    // wfi(), not bkpt(). Without a debugger attached, BKPT #0 raises
    // another exception inside the HardFault handler, which the Cortex-M
    // escalates to a CPU LOCKUP → silent chip reset (RESETREAS bit 3) →
    // we lose the fault context entirely. wfi() is safe to spin on.
    //
    // Без пробника defmt выше не виден, поэтому подпись HardFault — быстрое
    // непрерывное мигание (отличимо от узора паники: там 3 медленных).
    loop {
        board::blink_raw(1, 2_000_000, 2_000_000);
    }
}

use ctaphid_dispatch::DEFAULT_MESSAGE_SIZE as CTAPHID_MESSAGE_SIZE;

// nRF52840 high-frequency clock is 64 MHz; SysTick runs from it.
const SYSTICK_FREQ_HZ: u32 = 64_000_000;

#[rtic::app(device = nrf52840_pac, peripherals = true, dispatchers = [SWI3_EGU3, SWI4_EGU4])]
mod app {
    use super::SYSTICK_FREQ_HZ;
    use rtic_monotonics::systick::prelude::*;
    systick_monotonic!(Mono, 1000);
    use crate::initializer::{init_board, CcidClass, CtapHidClass, UsbBus, WalletHidClass};
    use crate::nfct;
    use crate::types::{Apps, Trussed, WalletSlot};
    use apdu_dispatch::dispatch::ApduDispatch;
    use apdu_dispatch::interchanges as apdu_interchanges;
    use ctaphid_dispatch::DefaultDispatch as CtaphidDispatchDefault;
    use embedded_time::duration::Milliseconds;
    use nrf52840_pac::POWER;
    use rtic_sync::channel::{Receiver, Sender};
    use rtic_sync::make_channel;
    use usb_device::device::UsbDevice;

    #[shared]
    struct Shared {
        trussed: Trussed,
        apps: Apps,
        ctaphid_dispatch: CtaphidDispatchDefault<'static, 'static>,
        apdu_dispatch: ApduDispatch<'static>,
        // NFC APDU interchange — t4t bridge in nfct.rs pushes inbound APDUs
        // from the reader here and pulls responses to send back.
        nfc_apdu_rq: apdu_interchanges::Requester<'static>,
        usbd: UsbDevice<'static, UsbBus>,
        ctaphid: CtapHidClass,
        ccid: CcidClass,
        wallet_hid: WalletHidClass,
        wallet: WalletSlot,
        // CTAPHID KEEPALIVE channel — mirrors the LPC55 runner's
        // pattern. Idle + on_usb call `ctaphid.did_start_processing()`
        // when a new CBOR command arrives; that returns
        // `Status::ReceivedData(ms)` indicating when the first
        // keepalive frame is due. Send that to the keepalive task,
        // which sleeps `ms` then transmits the keepalive. Without
        // these frames the chip is silent during long crypto
        // operations (e.g. allow_list iteration with 3× chacha8
        // decrypt) and CTAPHID hosts time out — most visibly inside
        // UTM, where the Linux hidapi timeout is tighter than
        // macOS's.
        ctaphid_keepalive_sender: Sender<'static, Milliseconds, 1>,
    }

    #[local]
    struct Local {
        power: POWER,
        ctaphid_keepalive_receiver: Receiver<'static, Milliseconds, 1>,
        // Buttons hoisted out of trussed's UserInterface: the idle loop polls
        // them via `board::poll_buttons` and latches gestures into a global
        // that both `check_user_presence` (FIDO) and `confirm_user_present_non_blocking` (wallet)
        // consume. Owned exclusively by idle, so plain Local (no lock).
        buttons: crate::board::Buttons,
        gesture: crate::board::GestureDetector,
        // UP-indicator LEDs hoisted out of trussed's UserInterface: the idle
        // loop drives them via `board::refresh_up_led` (ORing trussed status
        // and a waiting wallet sign). Owned exclusively by idle, so plain Local.
        leds: crate::board::Leds,
    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local) {
        // РАЗОВАЯ ПОЧИНКА UICR (только с фичей `nfc-fix`, см. src/uicr.rs).
        // Никаких мигов: результат виден по светодиоду в idle-цикле — он горит
        // ровно, если пины P0.09/P0.10 отданы NFCT (см. `diag_nfc_pins_steady`).
        // Перезагрузку не делаем: NFCPINS действует со следующего сброса, а
        // автосброс при неудаче дал бы цикл перезагрузок.
        #[cfg(feature = "nfc-fix")]
        let _ = crate::uicr::fix_nfcpins();

        // ДИАГНОСТИКА (только с фичей `test-up-control`). Статик `UP_CONTROL`
        // лежит в `.uninit` — секция не инициализируется при старте, и значение
        // из исходника в RAM не попадает (её пишет отладчик через JTAG). Поэтому
        // пишем вручную: 129 = «approve sticky», каждый запрос UP удовлетворён.
        #[cfg(feature = "test-up-control")]
        unsafe {
            core::ptr::write_volatile(&raw mut crate::board::UP_CONTROL, 129);
        }

        // Диагностика без пробника: приветствие до всего остального. Две
        // длинные вспышки (~600 мс) с паузой означают «наш код получил
        // управление». Если их не видно — до прошивки дело не дошло.
        #[cfg(feature = "led-diag")]
        {
            crate::board::blink_raw(2, 40_000_000, 12_000_000);
            crate::board::spin(24_000_000);
        }
        // Подпись владельца светодиода: 2 длинных + 7 коротких. Никакая чужая
        // схема (зарядник, загрузчик) такого узора не даёт — по нему видно,
        // что светодиодом управляет НАША прошивка.
        #[cfg(feature = "led-diag")]
        {
            crate::board::blink_raw(7, 4_000_000, 4_000_000);
            crate::board::spin(48_000_000);
        }

        // Причина ПРОШЛОЙ перезагрузки — по RESETREAS (загрузчик её не чистит,
        // это записано в его же коде). Гасим регистр, чтобы следующий старт был
        // чистым, и мигаем кодом: 1 = reset-пин, 2 = watchdog, 3 = программный
        // сброс (NVIC_SystemReset), 4 = LOCKUP (ядро упало: HardFault → сброс),
        // 5 = выход из System OFF.
        {
            let p = unsafe { &*nrf52840_pac::POWER::ptr() };
            let reas = p.resetreas.read().bits();
            p.resetreas.write(|w| unsafe { w.bits(reas) }); // write-1-to-clear
            #[cfg(feature = "led-diag")]
            if reas != 0 {
                let code = (reas.trailing_zeros() + 1).min(5);
                crate::board::blink_raw(code, 8_000_000, 8_000_000);
                crate::board::spin(24_000_000);
            }
            #[cfg(not(feature = "led-diag"))]
            let _ = reas;
            let _ = &p;
        }

        // SysTick monotonic — 1 kHz, used by UserInterface::uptime. (DWT
        // would wrap at ~67 s and panic trussed's user-presence loop.)
        Mono::start(ctx.core.SYST, SYSTICK_FREQ_HZ);

        let board = init_board(ctx.device);

        let (ctaphid_keepalive_sender, ctaphid_keepalive_receiver) = make_channel!(Milliseconds, 1);
        ctaphid_keepalive::spawn().unwrap();
        ndef_clock::spawn().unwrap();

        (
            Shared {
                trussed: board.trussed,
                apps: board.apps,
                ctaphid_dispatch: board.ctaphid_dispatch,
                apdu_dispatch: board.apdu_dispatch,
                nfc_apdu_rq: board.nfc_apdu_rq,
                usbd: board.usbd,
                ctaphid: board.ctaphid,
                ccid: board.ccid,
                wallet_hid: board.wallet_hid,
                wallet: board.wallet,
                ctaphid_keepalive_sender,
            },
            Local {
                power: board.power,
                ctaphid_keepalive_receiver,
                buttons: board.buttons,
                gesture: crate::board::GestureDetector::new(),
                leds: board.leds,
            },
        )
    }

    /// Idle: drain NFC + APDU + CTAPHID + USB. The contactless drain also
    /// runs in a separate `nfc_drain` task so reader reads don't have to
    /// wait for the idle loop to come around.
    #[idle(shared = [apps, ctaphid_dispatch, apdu_dispatch, nfc_apdu_rq, usbd, ctaphid, #[cfg(feature = "ccid")] ccid, #[cfg(feature = "wallet")] wallet, #[cfg(feature = "wallet")] wallet_hid, ctaphid_keepalive_sender], local = [buttons, gesture, leds])]
    fn idle(mut ctx: idle::Context) -> ! {
        loop {
            // Диагностика NFC (фича `nfc-fix` без `nfc-diag`): ровное горение =
            // пины P0.09/P0.10 отданы NFCT; «не горит» = UICR всё ещё в режиме
            // GPIO. Ни мигов, ни кодов: считать их на плате без консоли слишком
            // легко ошибиться. Идёт после `refresh_up_led`, чтобы перебить его.
            #[cfg(all(feature = "nfc-fix", not(feature = "nfc-diag")))]
            crate::board::diag_nfc_pins_steady();

            // Диагностика (только с фичей `led-diag`): события USBD читаем ДО
            // любого poll(), чтобы не пропустить их, плюс неблокирующий доклад
            // о трассе EP0. Блокировать цикл нельзя: драйвер считает таймаут
            // EP0-IN по 5 SOF-кадрам (5 мс). Выключено — светодиод принадлежит
            // штатной индикации «коснись ключа» (`refresh_up_led`).
            #[cfg(feature = "led-diag")]
            {
                crate::board::note_usb_events();
                use rtic_monotonics::Monotonic;
                let now_ms = crate::app::Mono::now().duration_since_epoch().to_millis();
                crate::board::report_tick(now_ms);
            }

            // Poll the hoisted buttons every pass and latch any committed
            // gesture into the global consumed by `check_user_presence` (FIDO)
            // and `confirm_user_present_non_blocking` (wallet). Cheap except the throttled cap-touch
            // read. Runs regardless of `wallet` — FIDO needs fresh gestures too.
            {
                use rtic_monotonics::Monotonic;
                let now_ms = crate::app::Mono::now().duration_since_epoch().to_millis();
                crate::board::poll_buttons(ctx.local.buttons, ctx.local.gesture, now_ms);
            }

            // Drive the UP/"waiting" LED: on when trussed is waiting (FIDO) or
            // a wallet sign is waiting. Cheap (atomic loads + one GPIO write).
            crate::board::refresh_up_led(ctx.local.leds);

            // Диагностика разводки кнопок (фича `pin-diag`): в простое
            // светодиод показывает состояние пинов касания. Идёт ПОСЛЕ
            // `refresh_up_led`, чтобы перебить его «выключено» в простое, и
            // молчит во время ожидания касания (см. `diag_pin_mirror`).
            #[cfg(all(feature = "pin-diag", feature = "board-supermini"))]
            {
                use rtic_monotonics::Monotonic;
                let now_ms = crate::app::Mono::now().duration_since_epoch().to_millis();
                crate::board::diag_pin_mirror(ctx.local.leds, now_ms);
            }

            // Диагностика NFC (фича `nfc-diag`): светодиод горит, пока на
            // катушке есть поле считывателя. Единственный способ проверить
            // катушку и настройку контура без LCR/nanoVNA (см. `diag_nfc_field`).
            #[cfg(all(feature = "nfc-diag", feature = "board-supermini"))]
            crate::board::diag_nfc_field();

            // Run the contactless drain inline once per loop too, in
            // case nfc_drain raced and an APDU is sitting in the
            // mailbox without an IRQ pending it.
            ctx.shared.nfc_apdu_rq.lock(nfct::fido_poll);

            let _ = ctx.shared.apps.lock(|apps| {
                ctx.shared
                    .apdu_dispatch
                    .lock(|disp| apps.apdu_dispatch(|app_slice| disp.poll(app_slice)))
            });

            #[cfg(feature = "ccid")]
            ctx.shared.ccid.lock(|ccid| ccid.check_for_app_response());

            let pending = ctx.shared.apps.lock(|apps| {
                ctx.shared
                    .ctaphid_dispatch
                    .lock(|disp| apps.ctaphid_dispatch(|app_slice| disp.poll(app_slice)))
            });
            if pending {
                rtic::pend(nrf52840_pac::Interrupt::USBD);
            }

            // Fill the wallet consent result from the idle-reachable inputs
            // (monotonic clock + NFC field) — the non-blocking UP check.
            #[cfg(feature = "wallet")]
            {
                use rtic_monotonics::Monotonic;
                let now_ms = crate::app::Mono::now().duration_since_epoch().to_millis();
                crate::board::confirm_user_present_non_blocking(now_ms);
            }

            // Drive the wallet HID transport (its own dispatch, outside `apps`).
            #[cfg(feature = "wallet")]
            {
                let wallet_pending = ctx
                    .shared
                    .wallet
                    .lock(|w| w.as_mut().map(|s| s.poll()).unwrap_or(false));
                if wallet_pending {
                    rtic::pend(nrf52840_pac::Interrupt::USBD);
                }
            }

            let ka_status = ctx.shared.usbd.lock(|usbd| {
                ctx.shared.ctaphid.lock(|ctaphid| {
                    ctaphid.check_for_app_response();
                    #[cfg(feature = "wallet")]
                    ctx.shared.wallet_hid.lock(|wallet_hid| {
                        wallet_hid.check_for_app_response();
                        #[cfg(feature = "ccid")]
                        let _ = ctx
                            .shared
                            .ccid
                            .lock(|ccid| usbd.poll(&mut [ctaphid, ccid, wallet_hid]));
                        #[cfg(not(feature = "ccid"))]
                        let _ = usbd.poll(&mut [ctaphid, wallet_hid]);
                    });
                    #[cfg(not(feature = "wallet"))]
                    {
                        #[cfg(feature = "ccid")]
                        let _ = ctx.shared.ccid.lock(|ccid| usbd.poll(&mut [ctaphid, ccid]));
                        #[cfg(not(feature = "ccid"))]
                        let _ = usbd.poll(&mut [ctaphid]);
                    }
                    ctaphid.did_start_processing()
                })
            });
            if let usbd_ctaphid::types::Status::ReceivedData(ms) = ka_status {
                ctx.shared
                    .ctaphid_keepalive_sender
                    .lock(|s| s.try_send(ms).ok());
            }
        }
    }

    #[task(binds = USBD, priority = 6, shared = [usbd, ctaphid, #[cfg(feature = "ccid")] ccid, #[cfg(feature = "wallet")] wallet_hid, ctaphid_keepalive_sender])]
    fn on_usb(mut ctx: on_usb::Context) {
        // Диагностика: отмечаем «сырые» события USBD раньше, чем драйвер их
        // погасит. Видно в «пульсе» idle-цикла: +1 миг = был USB-reset,
        // ещё +1 = приходил EP0 SETUP (хост реально запрашивал дескриптор).
        #[cfg(feature = "led-diag")]
        crate::board::note_usb_events();

        let ka_status = ctx.shared.usbd.lock(|usbd| {
            ctx.shared.ctaphid.lock(|ctaphid| {
                #[cfg(feature = "wallet")]
                ctx.shared.wallet_hid.lock(|wallet_hid| {
                    #[cfg(feature = "ccid")]
                    let _ = ctx
                        .shared
                        .ccid
                        .lock(|ccid| usbd.poll(&mut [ctaphid, ccid, wallet_hid]));
                    #[cfg(not(feature = "ccid"))]
                    let _ = usbd.poll(&mut [ctaphid, wallet_hid]);
                });
                #[cfg(not(feature = "wallet"))]
                {
                    #[cfg(feature = "ccid")]
                    let _ = ctx.shared.ccid.lock(|ccid| usbd.poll(&mut [ctaphid, ccid]));
                    #[cfg(not(feature = "ccid"))]
                    let _ = usbd.poll(&mut [ctaphid]);
                }
                ctaphid.did_start_processing()
            })
        });
        if let usbd_ctaphid::types::Status::ReceivedData(ms) = ka_status {
            ctx.shared
                .ctaphid_keepalive_sender
                .lock(|s| s.try_send(ms).ok());
        }
    }

    /// CTAPHID KEEPALIVE pump. Receives a deadline from `did_start_processing`
    /// (when a new CBOR command arrives), waits that long, then transmits
    /// a `KEEPALIVE` frame to the host so it doesn't time out mid-operation.
    /// If `send_keepalive` itself returns `ReceivedData(ms)` (i.e. another
    /// command landed during transmit) we re-arm via the channel.
    ///
    /// Mirrors the LPC55 runner's `ctaphid_keepalive` task.
    #[task(shared = [ctaphid, ctaphid_keepalive_sender], local = [ctaphid_keepalive_receiver], priority = 6)]
    async fn ctaphid_keepalive(mut ctx: ctaphid_keepalive::Context) {
        loop {
            let ms = ctx.local.ctaphid_keepalive_receiver.recv().await.unwrap();
            Mono::delay(ms.0.millis()).await;
            let next = ctx
                .shared
                .ctaphid
                .lock(|ctaphid| ctaphid.send_keepalive(false));
            if let usbd_ctaphid::types::Status::ReceivedData(ms) = next {
                let _ = ctx.local.ctaphid_keepalive_receiver.try_recv();
                ctx.shared
                    .ctaphid_keepalive_sender
                    .lock(|s| s.try_send(ms).ok());
            }
        }
    }

    /// Free-running ms clock for NDEF suppression. Advances a counter so the
    /// `NdefFidoGate` wrapper can refuse NDEF SELECT for `NDEF_SUPPRESS_MS`
    /// after any FIDO command — stops phones popping the URL during a WebAuthn
    /// ceremony (the OS selects the FIDO applet first, which arms the window).
    #[task(priority = 1)]
    async fn ndef_clock(_: ndef_clock::Context) {
        const REFRESH_MS: u32 = 50;
        loop {
            Mono::delay(REFRESH_MS.millis()).await;
            crate::types::NDEF_CLOCK_MS
                .fetch_add(REFRESH_MS, core::sync::atomic::Ordering::Relaxed);
        }
    }

    /// Contactless dispatch — pended from the NFCT IRQ whenever an
    /// inbound APDU lands in the t4t bridge mailbox. Sits at priority
    /// 2, above the trussed syscall ISR (priority 1). The syscall ISR
    /// busy-loops inside `confirm_user_present`; if nfc_drain were
    /// below it, NDEF reads would never get serviced during a long
    /// user-presence wait. Below USBD (3) and NFCT (4) so USB polling
    /// and chip IRQs still preempt promptly.
    #[task(binds = SWI1_EGU1, priority = 2, shared = [apps, apdu_dispatch, nfc_apdu_rq])]
    fn nfc_drain(mut ctx: nfc_drain::Context) {
        defmt::info!("nfc_drain: enter");
        // 1. Push any newly-arrived APDU from INCOMING into the
        //    contactless interchange.
        ctx.shared.nfc_apdu_rq.lock(nfct::fido_poll);
        // 2. Run apdu_dispatch so ndef-app (or whoever owns the AID)
        //    builds a response.
        let r = ctx.shared.apps.lock(|apps| {
            ctx.shared
                .apdu_dispatch
                .lock(|disp| apps.apdu_dispatch(|app_slice| disp.poll(app_slice)))
        });
        defmt::info!("nfc_drain: dispatch={:?}", defmt::Debug2Format(&r));
        // 3. Drain the response back through t4t to the reader.
        ctx.shared.nfc_apdu_rq.lock(nfct::fido_poll);
    }

    /// Trussed syscall — pended by `Syscall::syscall()` via
    /// `rtic::pend(SWI0_EGU0)`. Must run at a priority HIGHER than
    /// any task that locks resources `idle` is holding when an app
    /// inside ctaphid_dispatch / apdu_dispatch hits
    /// `syscall!(trussed.…)`. The shared resource ceiling for those
    /// (`apps`, `apdu_dispatch`, `ctaphid_dispatch`) is `nfc_drain`'s
    /// priority (= 2): locking them sets BASEPRI to NVIC-prio-6,
    /// which masks any task ≤ priority 2. A syscall task at priority
    /// 1 was therefore never able to preempt — apps busy-polled in
    /// `<ClientImplementation as PollClient>::poll`, the service
    /// never ran, every CTAPHID_MSG / CTAPHID_CBOR timed out.
    ///
    /// Priority 3 sits above `nfc_drain` (2) and at the same level
    /// as `USBD` (3) — they share no resources, so RTIC schedules
    /// them cooperatively without preemption hazards. Matches the
    /// pattern the lpc55 runner uses (its Trussed syscall is at
    /// priority 5).
    #[task(binds = SWI0_EGU0, priority = 3, shared = [trussed])]
    fn syscall(mut ctx: syscall::Context) {
        use rtic_monotonics::Monotonic;
        let t0 = Mono::now();
        ctx.shared.trussed.lock(|t| t.process());
        let dt = (Mono::now() - t0).to_millis();
        if dt > 30 {
            defmt::warn!("syscall: process took {=u32}ms", dt);
        }
    }

    #[task(binds = POWER_CLOCK, priority = 5, local = [power])]
    fn on_power(ctx: on_power::Context) {
        let p = ctx.local.power;
        if p.events_usbdetected.read().bits() != 0 {
            p.events_usbdetected.write(|w| unsafe { w.bits(0) });
        }
        if p.events_usbpwrrdy.read().bits() != 0 {
            p.events_usbpwrrdy.write(|w| unsafe { w.bits(0) });
        }
        if p.events_usbremoved.read().bits() != 0 {
            p.events_usbremoved.write(|w| unsafe { w.bits(0) });
        }
    }

    // NFCT + TIMER4 IRQ trampolines into libnrfx_nfct.a. TIMER instance
    // 4 is hardwired by NRFX_NFCT_CONFIG_TIMER_INSTANCE_ID in the .a's
    // build config (see components/nrf-nfc/Makefile).
    //
    // Priority 7 (highest): NFCT frame RX/TX is hard real-time (ISO-14443
    // FDT is ~86 us). It MUST sit above USBD (6) and POWER_CLOCK (5) — if
    // the USB ISR preempts NFCT mid-frame under CCID/CTAPHID load, the
    // anti-collision/RATS reply is corrupted or late and the reader
    // abandons activation (symptom: NFCT IRQ storms, t4t never latches
    // FIELD_ON, PC/SC reader sees no card).
    #[task(binds = NFCT, priority = 7)]
    fn nfct_irq(_: nfct_irq::Context) {
        unsafe { nrf_nfc::nrfx_nfct::nrfx_nfct_irq_handler() };
        // Wake the contactless drain. The .a's IRQ handler may have
        // pushed an APDU into INCOMING via the t4t callback; pending
        // nfc_drain here lets it run even when idle is parked in a
        // user-presence spin.
        rtic::pend(nrf52840_pac::Interrupt::SWI1_EGU1);
    }

    #[task(binds = TIMER4, priority = 7)]
    fn nfct_timer(_: nfct_timer::Context) {
        unsafe { nrf_nfc::nrfx_nfct::nrfx_nfct_workaround_timer_handler() };
    }
}
