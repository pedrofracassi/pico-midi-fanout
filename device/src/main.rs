//! DEVICE PICO — UART in → USB MIDI device to one show PC.
//!
//! Flash the same binary onto both device Picos. Each enumerates as a
//! class-compliant USB MIDI device named `DEVICE_NAME`, so TouchDesigner's
//! MIDI mapping is identical on PC A and PC B. No serial number is set, so
//! a pre-flashed spare looks exactly the same to the PC.
//!
//! State replay: the last value of every CC and pitch bend is cached. When
//! the PC sends CC 127 on channel 16, all cached values are re-sent — have
//! TouchDesigner send that on project start.

#![no_std]
#![no_main]

use core::cell::RefCell;

use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::peripherals::{UART0, USB};
use embassy_rp::uart::{self, BufferedInterruptHandler, BufferedUartRx};
use embassy_rp::usb::{Driver, InterruptHandler};
use embassy_rp::watchdog::Watchdog;
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::{Duration, Timer};
use embassy_usb::class::midi::{MidiClass, MidiClassConfig, MidiClassState, Receiver, Sender};
use embassy_usb::{Builder, Config, UsbDevice};
use midi_link::{StateCache, StreamParser, is_replay_request};
use panic_reset as _; // on panic: reboot, don't hang
use static_cell::StaticCell;

/// Must match `LINK_BAUD` in the host firmware.
const LINK_BAUD: u32 = 500_000;
/// Name shown in TouchDesigner / Windows. Keep identical on A and B.
const DEVICE_NAME: &str = "ShowFaders";

bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => InterruptHandler<USB>;
    UART0_IRQ => BufferedInterruptHandler<UART0>;
});

type UsbDriver = Driver<'static, USB>;

/// Packets waiting to go to the PC. The UART task never blocks on it:
/// if the PC isn't reading, packets are dropped but the cache stays current.
static TO_PC: Channel<CriticalSectionRawMutex, [u8; 4], 256> = Channel::new();
static CACHE: Mutex<CriticalSectionRawMutex, RefCell<StateCache>> = Mutex::new(RefCell::new(StateCache::new()));

#[embassy_executor::main(executor = "embassy_rp::executor::Executor", entry = "cortex_m_rt::entry")]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    let mut watchdog = Watchdog::new(p.WATCHDOG);
    watchdog.start(Duration::from_secs(2));
    spawner.spawn(watchdog_task(watchdog).unwrap());

    // ---- USB MIDI device ----
    let driver = Driver::new(p.USB, Irqs);
    // 0x1209:0x0001 is the pid.codes test ID; fine for an in-house tool.
    let mut config = Config::new(0x1209, 0x0001);
    config.manufacturer = Some("Show Control");
    config.product = Some(DEVICE_NAME);
    config.max_power = 100;
    config.max_packet_size_0 = 64;

    static CONFIG_DESC: StaticCell<[u8; 256]> = StaticCell::new();
    static BOS_DESC: StaticCell<[u8; 256]> = StaticCell::new();
    static CONTROL_BUF: StaticCell<[u8; 64]> = StaticCell::new();
    static MIDI_STATE: StaticCell<MidiClassState<'static>> = StaticCell::new();

    let mut builder = Builder::new(
        driver,
        config,
        CONFIG_DESC.init([0; 256]),
        BOS_DESC.init([0; 256]),
        &mut [], // no MS OS descriptors
        CONTROL_BUF.init([0; 64]),
    );

    let mut midi_cfg = MidiClassConfig::default(); // 1 IN + 1 OUT jack, 64-byte packets
    midi_cfg.interface_name = Some(DEVICE_NAME);
    midi_cfg.in_jack_names = &[Some(DEVICE_NAME)];
    midi_cfg.out_jack_names = &[Some(DEVICE_NAME)];
    let class = MidiClass::new_with_names(&mut builder, MIDI_STATE.init(MidiClassState::new()), midi_cfg);
    let (sender, receiver) = class.split();
    let usb = builder.build();

    // ---- UART link from the host Pico (RX only) ----
    let mut uart_cfg = uart::Config::default();
    uart_cfg.baudrate = LINK_BAUD;
    static RX_BUF: StaticCell<[u8; 1024]> = StaticCell::new();
    let link = BufferedUartRx::new(p.UART0, p.PIN_1, Irqs, &mut RX_BUF.init([0; 1024])[..], uart_cfg);

    spawner.spawn(usb_task(usb).unwrap());
    spawner.spawn(uart_task(link).unwrap());
    spawner.spawn(to_pc_task(sender.unwrap()).unwrap());
    spawner.spawn(from_pc_task(receiver.unwrap()).unwrap());
}

#[embassy_executor::task]
async fn usb_task(mut usb: UsbDevice<'static, UsbDriver>) -> ! {
    usb.run().await
}

/// UART bytes → USB-MIDI packets → cache + outgoing queue.
#[embassy_executor::task]
async fn uart_task(mut link: BufferedUartRx<'static>) -> ! {
    let mut parser = StreamParser::new();
    let mut buf = [0u8; 64];
    loop {
        let n = match link.read(&mut buf).await {
            Ok(n) => n,
            Err(_) => {
                // Framing/overrun error: resync on the next status byte.
                parser = StreamParser::new();
                continue;
            }
        };
        for &byte in &buf[..n] {
            if let Some(packet) = parser.push(byte) {
                CACHE.lock(|c| c.borrow_mut().observe(&packet));
                let _ = TO_PC.try_send(packet);
            }
        }
    }
}

/// Drain the queue to the PC, batching up to 15 packets per USB transfer.
#[embassy_executor::task]
async fn to_pc_task(mut sender: Sender<'static, UsbDriver>) -> ! {
    let mut out = [0u8; 60]; // < 64 bytes, so a transfer never needs a ZLP
    loop {
        sender.wait_connection().await;
        loop {
            out[..4].copy_from_slice(&TO_PC.receive().await);
            let mut len = 4;
            while len < out.len() {
                let Ok(packet) = TO_PC.try_receive() else { break };
                out[len..len + 4].copy_from_slice(&packet);
                len += 4;
            }
            if sender.write_packet(&out[..len]).await.is_err() {
                break; // PC unplugged or USB reset: wait for reconnection
            }
        }
    }
}

/// Listen for the replay request from TouchDesigner.
#[embassy_executor::task]
async fn from_pc_task(mut receiver: Receiver<'static, UsbDriver>) -> ! {
    let mut buf = [0u8; 64];
    loop {
        receiver.wait_connection().await;
        loop {
            let Ok(n) = receiver.read_packet(&mut buf).await else { break };
            if buf[..n].chunks_exact(4).any(is_replay_request) {
                replay_state().await;
            }
        }
    }
}

async fn replay_state() {
    for ch in 0..16u8 {
        for cc in 0..128u8 {
            if let Some(v) = CACHE.lock(|c| c.borrow().cc(ch, cc)) {
                TO_PC.send([0x0B, 0xB0 | ch, cc, v]).await;
            }
        }
        if let Some(pb) = CACHE.lock(|c| c.borrow().pitch_bend(ch)) {
            TO_PC.send([0x0E, 0xE0 | ch, (pb & 0x7F) as u8, (pb >> 7) as u8]).await;
        }
    }
}

#[embassy_executor::task]
async fn watchdog_task(mut watchdog: Watchdog<'static>) -> ! {
    loop {
        watchdog.feed(Duration::from_secs(2));
        Timer::after_millis(500).await;
    }
}
