//! TEST FIRMWARE for a device Pico — sends a sine wave straight to the PC.
//!
//! Flash this instead of the normal device firmware to check the PC side
//! (USB, Windows, TouchDesigner mapping) without the host Pico or the fader
//! box. It enumerates as the same `ShowFaders` device and ignores the UART.
//!
//! On MIDI channel 1, every 10 ms:
//!   CC 1       = sine   (0..127)
//!   CC 2       = cosine (0..127, 90° ahead of CC 1)
//!   pitch bend = sine   (0..16383, full 14-bit resolution)
//!
//! Flash back with `cargo run --release -p device`.

#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::peripherals::USB;
use embassy_rp::usb::{Driver, InterruptHandler};
use embassy_rp::watchdog::Watchdog;
use embassy_time::{Duration, Ticker, Timer};
use embassy_usb::class::midi::{MidiClass, MidiClassConfig, MidiClassState, Sender};
use embassy_usb::{Builder, Config, UsbDevice};
use panic_reset as _;
use static_cell::StaticCell;

/// Keep identical to the normal device firmware so TouchDesigner's mapping works.
const DEVICE_NAME: &str = "ShowFaders";
/// One full sine cycle.
const PERIOD_MS: u32 = 4_000;
/// Time between updates.
const STEP_MS: u32 = 10;
/// MIDI channel 1 (0-based on the wire).
const CHANNEL: u8 = 0;

bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => InterruptHandler<USB>;
});

type UsbDriver = Driver<'static, USB>;

/// One cycle of sine, 14-bit (0..=16383, centre 8192). Built at compile
/// time, because the M0+ has no FPU and `core` has no `sin`.
static SINE: [u16; 256] = {
    let mut table = [0u16; 256];
    let mut i = 0;
    while i < 256 {
        let s = sin_turns(i as f32 / 256.0);
        table[i] = (8192.0 + 8191.0 * s + 0.5) as u16;
        i += 1;
    }
    table
};

/// sin(2π·t) for t in [0, 1): fold to a quarter wave, then Taylor series.
const fn sin_turns(t: f32) -> f32 {
    let (t, sign) = if t >= 0.5 { (t - 0.5, -1.0) } else { (t, 1.0) };
    let t = if t > 0.25 { 0.5 - t } else { t };
    let x = t * 2.0 * core::f32::consts::PI; // 0..=π/2
    let x2 = x * x;
    // x - x³/3! + x⁵/5! - x⁷/7! + x⁹/9!
    let s = x * (1.0 - x2 / 6.0 * (1.0 - x2 / 20.0 * (1.0 - x2 / 42.0 * (1.0 - x2 / 72.0))));
    sign * s
}

#[embassy_executor::main(executor = "embassy_rp::executor::Executor", entry = "cortex_m_rt::entry")]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    let mut watchdog = Watchdog::new(p.WATCHDOG);
    watchdog.start(Duration::from_secs(2));
    spawner.spawn(watchdog_task(watchdog).unwrap());

    // Same USB identity as the normal device firmware.
    let driver = Driver::new(p.USB, Irqs);
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
        &mut [],
        CONTROL_BUF.init([0; 64]),
    );

    let mut midi_cfg = MidiClassConfig::default();
    midi_cfg.interface_name = Some(DEVICE_NAME);
    midi_cfg.in_jack_names = &[Some(DEVICE_NAME)];
    midi_cfg.out_jack_names = &[Some(DEVICE_NAME)];
    let class = MidiClass::new_with_names(&mut builder, MIDI_STATE.init(MidiClassState::new()), midi_cfg);
    let (sender, _receiver) = class.split();
    let usb = builder.build();

    spawner.spawn(usb_task(usb).unwrap());
    sine_loop(sender.unwrap()).await
}

async fn sine_loop(mut sender: Sender<'static, UsbDriver>) -> ! {
    // 32-bit phase accumulator; the top 8 bits index the table.
    const PHASE_STEP: u32 = ((1u64 << 32) * STEP_MS as u64 / PERIOD_MS as u64) as u32;
    let mut phase: u32 = 0;

    loop {
        sender.wait_connection().await;
        let mut ticker = Ticker::every(Duration::from_millis(STEP_MS as u64));
        loop {
            let sine = SINE[(phase >> 24) as usize];
            let cosine = SINE[(phase.wrapping_add(1 << 30) >> 24) as usize];

            // Three USB-MIDI event packets in one transfer (cable 0).
            let packets = [
                0x0B, 0xB0 | CHANNEL, 1, (sine >> 7) as u8,
                0x0B, 0xB0 | CHANNEL, 2, (cosine >> 7) as u8,
                0x0E, 0xE0 | CHANNEL, (sine & 0x7F) as u8, (sine >> 7) as u8,
            ];
            if sender.write_packet(&packets).await.is_err() {
                break; // PC unplugged or USB reset: wait for reconnection
            }

            phase = phase.wrapping_add(PHASE_STEP);
            ticker.next().await;
        }
    }
}

#[embassy_executor::task]
async fn usb_task(mut usb: UsbDevice<'static, UsbDriver>) -> ! {
    usb.run().await
}

#[embassy_executor::task]
async fn watchdog_task(mut watchdog: Watchdog<'static>) -> ! {
    loop {
        watchdog.feed(Duration::from_secs(2));
        Timer::after_millis(500).await;
    }
}
