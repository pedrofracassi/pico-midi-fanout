# pico-midi-fanout

One USB MIDI controller → two show PCs, using three Raspberry Pi Picos, in Rust
([Embassy](https://embassy.dev)).

```
fader box ─USB─► HOST PICO ─UART (GP0)─┬─► DEVICE PICO A ─USB─► PC A
                                       └─► DEVICE PICO B ─USB─► PC B
```

| Crate       | Runs on          | Does                                                                 |
|-------------|------------------|----------------------------------------------------------------------|
| `host`      | host Pico        | USB MIDI **host** on the native USB port; forwards serial MIDI to GP0 |
| `device`    | both device Picos | Serial MIDI on GP1 → USB MIDI **device** named `ShowFaders`          |
| `midi-link` | shared library   | Serial-MIDI → USB-MIDI packet parser, fader-state cache (unit tested) |

## Build and flash

```sh
rustup show                  # installs the pinned toolchain from rust-toolchain.toml
cargo install elf2uf2-rs     # or use probe-rs / picotool

# linux
cargo test -p midi-link --target x86_64-unknown-linux-gnu   # (or your host triple)
# windows
cargo test -p midi-link --target x86_64-pc-windows-msvc

# Hold BOOTSEL, plug the Pico into your computer, then:
cargo run --release -p host      # host Pico
cargo run --release -p device    # each device Pico (same binary on both)

# Test a PC on its own: flash this onto a device Pico instead. It sends
# a 4 s sine wave on channel 1 (CC 1, CC 2 = cosine, pitch bend) over USB.
cargo run --release -p device --bin sine
```

The host Pico's USB port is used for the controller, so to reflash it later,
remove the OTG adapter, hold BOOTSEL, and plug it into your computer.

All embassy crates are pinned to one git commit because the USB host stack is
not on crates.io yet (the MIDI host class was merged on 2026-09-27).

## Wiring

```
HOST PICO                           DEVICE PICOS (A and B)
pin 1  GP0 (UART0 TX) ─┬─ 1 kΩ ───► pin 2 GP1 (UART0 RX) on A
                       └─ 1 kΩ ───► pin 2 GP1 (UART0 RX) on B
pin 3  GND ───────────────────────► GND on A and B
pin 40 VBUS ◄── 5 V (diode-OR from A's and B's pin 40 VBUS, plus optional PSU)
micro-USB ◄── OTG adapter ◄── fader box
```

* The 1 kΩ resistors stop the host's TX line back-powering an unpowered device Pico.
* Diode-OR: one Schottky (e.g. 1N5819) from each source to the host's VBUS, so
  the host Pico and controller stay powered if either PC turns off.

## TouchDesigner

* Map the `ShowFaders` MIDI device identically on both PCs.
* On project start, send **CC 127 on channel 16** to `ShowFaders` (e.g. a MIDI
  Out CHOP fired from an Execute DAT's `onStart`). The device Pico replays the
  last value of every CC and pitch bend it has seen, so a rebooted PC is in sync
  immediately.

## Robustness features

* Watchdog on all three boards (2 s) and `panic-reset`: any hang or panic reboots.
* Host reboots itself if enumeration fails or the controller stops responding,
  which forces a clean re-enumeration.
* Device never blocks the UART path on USB: if the PC isn't reading, packets are
  dropped but the state cache stays current.

## Changing things

* **Baud rate**: `LINK_BAUD` in both `main.rs` files (must match).
* **Device name**: `DEVICE_NAME` in `device/src/main.rs`.
* **Pico W**: the LED is on the wireless chip, not GP25; remove or change the LED
  code in `host`.
* **Pico 2 (RP2350)**: the embassy host driver has RP235x support, but this is
  untested here. Switch the `embassy-rp` feature to `rp235xa`, the target to
  `thumbv8m.main-none-eabihf`, and use an RP2350 `memory.x`.
