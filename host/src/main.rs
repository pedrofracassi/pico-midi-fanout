//! HOST PICO — USB MIDI host on the native USB port → UART fan-out.
//!
//! The fader box plugs into this Pico's micro-USB port through an OTG
//! adapter (5 V fed into VBUS, pin 40). Every MIDI message it sends is
//! written as plain serial MIDI to UART0 TX on GP0, which is wired to the
//! RX pin of both device Picos.
//!
//! Status LED (GP25): slow blink = waiting for controller,
//! solid = controller connected, flicker = MIDI flowing.

#![no_std]
#![no_main]

use core::sync::atomic::{AtomicBool, AtomicU8, Ordering};

use embassy_executor::Spawner;
use embassy_futures::select::{Either, select};
use embassy_rp::bind_interrupts;
use embassy_rp::gpio::{Level, Output};
use embassy_rp::peripherals::{UART0, USB};
use embassy_rp::uart::{self, BufferedInterruptHandler, BufferedUartTx};
use embassy_rp::watchdog::Watchdog;
use embassy_time::{Duration, Timer};
use embassy_usb_driver::host::DeviceEvent;
use embassy_usb_host::class::midi::{MidiHost, UsbMidiEventPacket};
use embassy_usb_host::{BusRoute, BusState};
use embedded_io_async::Write;
use panic_reset as _; // on panic: reboot, don't hang
use static_cell::StaticCell;

/// Must match `LINK_BAUD` in the device firmware.
const LINK_BAUD: u32 = 500_000;

bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => embassy_rp::usb::host::InterruptHandler<USB>;
    UART0_IRQ => BufferedInterruptHandler<UART0>;
});

const STATE_WAITING: u8 = 0;
const STATE_CONNECTED: u8 = 1;
static STATE: AtomicU8 = AtomicU8::new(STATE_WAITING);
static ACTIVITY: AtomicBool = AtomicBool::new(false);

#[embassy_executor::main(executor = "embassy_rp::executor::Executor", entry = "cortex_m_rt::entry")]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    // Watchdog + status LED run in their own task. If the executor ever
    // wedges, the watchdog stops being fed and the chip reboots.
    let mut watchdog = Watchdog::new(p.WATCHDOG);
    watchdog.start(Duration::from_secs(2));
    spawner.spawn(housekeeping(watchdog, Output::new(p.PIN_25, Level::Low)).unwrap());

    // UART link to both device Picos (TX only).
    let mut uart_cfg = uart::Config::default();
    uart_cfg.baudrate = LINK_BAUD;
    static TX_BUF: StaticCell<[u8; 512]> = StaticCell::new();
    let mut link = BufferedUartTx::new(p.UART0, p.PIN_0, Irqs, &mut TX_BUF.init([0; 512])[..], uart_cfg);

    // Native USB controller in host mode.
    let driver = embassy_rp::usb::host::Driver::new(p.USB, Irqs);
    static BUS_STATE: BusState = BusState::new();
    let (mut bus_ctrl, bus) = embassy_usb_host::bus(driver, &BUS_STATE);

    loop {
        STATE.store(STATE_WAITING, Ordering::Relaxed);
        let speed = bus_ctrl.wait_for_connection().await;

        // MIDI config descriptors are often longer than 256 bytes.
        let mut config_buf = [0u8; 1024];
        let (enum_info, config_len) = match bus.enumerate(BusRoute::Direct(speed), &mut config_buf).await {
            Ok(r) => r,
            Err(_) => reboot_after_delay().await,
        };
        let addr = enum_info.device_address;

        let midi = match MidiHost::new(&bus, &config_buf[..config_len], &enum_info) {
            Ok(m) => m,
            // Not a class-compliant MIDI device (or an unsupported layout).
            Err(_) => reboot_after_delay().await,
        };
        let (_to_controller, mut from_controller) = midi.split();
        STATE.store(STATE_CONNECTED, Ordering::Relaxed);

        let forward = async {
            let mut buf = [0u8; 64];
            let mut consecutive_errors = 0u32;
            loop {
                match from_controller.read_transfer(&mut buf).await {
                    Ok(n) => {
                        consecutive_errors = 0;
                        for chunk in buf[..n].chunks_exact(4) {
                            let packet = UsbMidiEventPacket::new([chunk[0], chunk[1], chunk[2], chunk[3]]);
                            // `data()` is None for padding / reserved packets.
                            if let Some(bytes) = packet.data() {
                                let _ = link.write_all(bytes).await; // all virtual cables merged
                                ACTIVITY.store(true, Ordering::Relaxed);
                            }
                        }
                    }
                    Err(_) => {
                        consecutive_errors += 1;
                        if consecutive_errors > 50 {
                            return; // something is badly wrong; caller reboots
                        }
                        Timer::after_millis(2).await;
                    }
                }
            }
        };

        let unplugged = async {
            loop {
                if let DeviceEvent::Disconnected = bus_ctrl.wait_for_device_event().await {
                    return;
                }
            }
        };

        match select(forward, unplugged).await {
            // Persistent transfer errors with the controller still attached:
            // a clean reboot re-enumerates it from scratch.
            Either::First(()) => reboot_after_delay().await,
            Either::Second(()) => {
                drop(from_controller);
                drop(_to_controller);
                bus.free_address(addr);
            }
        }
    }
}

/// Give the device a moment, then do a full reset so enumeration starts clean.
async fn reboot_after_delay() -> ! {
    Timer::after_millis(500).await;
    cortex_m::peripheral::SCB::sys_reset();
}

#[embassy_executor::task]
async fn housekeeping(mut watchdog: Watchdog<'static>, mut led: Output<'static>) -> ! {
    let mut tick = 0u32;
    loop {
        watchdog.feed(Duration::from_secs(2));
        // (No atomic swap on the Cortex-M0+, so load then clear.)
        let active = ACTIVITY.load(Ordering::Relaxed);
        ACTIVITY.store(false, Ordering::Relaxed);
        let on = match STATE.load(Ordering::Relaxed) {
            STATE_WAITING => (tick / 8) % 2 == 0, // ~400 ms blink
            _ => !active,                         // solid, flicker on data
        };
        led.set_level(if on { Level::High } else { Level::Low });
        tick = tick.wrapping_add(1);
        Timer::after_millis(50).await;
    }
}
