# OpenShift

**An open-source controller platform for automatic transmissions.**

[![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](LICENSE)

OpenShift is a fork and continuation of Markus Kovero's
[`7226ctrl`](https://github.com/mkovero/7226ctrl) project — a standalone controller
for the Mercedes/Chrysler **722.6** ("5G-Tronic") automatic transmission. The design
is similar to more recent (7G-Tronic++) gearboxes, so with care it may be adapted to
those as well.

This fork keeps the proven Arduino/Teensy firmware while adding a new, clean-room
**Rust control platform (`transictrl`)** aimed at portability, safety, and legal
interoperability across a wider range of transmissions and microcontrollers.

> ⚠️ **Not fully functional / experimental — use at your own risk.**
> This project controls safety-critical vehicle hardware. It is not intended for
> on-road use without full compliance with your local laws and regulations.

---

## Repository layout

| Path           | Description                                                                 |
| -------------- | --------------------------------------------------------------------------- |
| `main/`        | Original C++/Arduino firmware for the 722.6 controller (Teensy/Mega).        |
| `transictrl/`  | Rust-based controller platform. Core logic, HAL, host tools, and per-board firmware (RP2350, STM32F4) are implemented; Teensy 4 skeleton remains. |
| `datalog/`     | Datalogging format, method, and setup notes.                                |
| `dataviz/`     | Legacy Node.js backend + React frontend for live/remote visualization. A Rust egui dataviz app is in `transictrl/host/dataviz`. |
| `pcb/`         | Gerber files and BOMs for the controller PCBs (revisions H, I, K).          |

---

## The classic firmware (`main/`)

Originally intended as a reverse-engineered 722.6 controller, the firmware grew to
also control boost, speedometer, RPM meter, fuel pumps, and even the horn. It can
communicate with an electric steering wheel for menus and manual button shifts. A
CAN bus interface is present on the PCB.

**Working and tested:**

- Automatic up/down-shift based on vehicle speed and throttle position
- Manual shifting
- Throttle position sensor
- Pressure maps for shifts and modulation pressure (oil temperature + load, incl. P/N/R)
- W202 gearstick reading
- SPI OLED data output
- Boost sensor reading and boost-limit maps
- Staged boost limiting (e.g. drop boost on pre-shift, then continue shift)
- W124 wire-speedometer
- Fuel pump control on idle
- Vehicle speed from tire size and diff ratio (with and without engine RPM)
- RPM reading
- Electric steering wheel support (manual shifts, menu control)
- Oil temperature
- N2/N3 speed sensors
- Gear evaluation (confirm which gear is engaged)
- Torque converter lock (default: 100% lock in 5th, >80 km/h, low load)
- Percentual map reads (preferably without floats)
- Exhaust pressure sensor and intake/exhaust pressure ratio calculation
- Adaptive shift pressure (transloppy/maps)

**Implemented, needs real-world testing:**

- W124 RPM meter (works in theory)

**Work in progress:**

- External datalogging (hardware implementation, visualization)

Schematics target the Teensy and can be used with a Mega to some extent, though Mega
support is being deprecated for performance reasons.

---

## The new platform (`transictrl`)

`transictrl` is a ground-up rewrite in **Rust** with a strong focus on safety,
portability, and lawful interoperability.

- **Core logic** — hardware-independent, calibration-driven shift state machine,
  gear-ratio evaluation, map interpolation, and safety/fault handling (`#![no_std]`,
  `#![forbid(unsafe_code)]`, unit-testable on the host)
- **transictrl-722** — GPL-3.0-derived crate that ports the 722.6-specific shift
  planning, sensor scaling, W202 gearstick decoding, and map data from `main/`
- **transictrl-hal** — board-agnostic hardware abstraction traits for solenoids,
  PWM, ADC, frequency capture, CAN, persistent storage, serial, and display
- **Firmware** — portable HAL + embassy runtime for RP2040/RP2350 and STM32F4
  (`fw-rp2350`, `fw-stm32`); Teensy 4 (`fw-teensy4`) is a skeleton awaiting a
  suitable embassy-compatible HAL
- **Host** — axum web server with a serial→WebSocket telemetry bridge, a device
  simulator, and a calibration validation endpoint; Rust egui dataviz app in
  `transictrl/host/dataviz`
- **Protocol** — open serialization with serde/postcard, plus a no-std binary
  framing layer for calibration push/pull over serial
- **Calibration** — open TOML/JSON schema; optional user-supplied importers;
  command-line manager in `transictrl/tools/calibration-cli`
- **Map conversion** — `transictrl/tools/maps-converter/convert_maps.py` ports the
  legacy `main/include/maps.h` PROGMEM tables into Rust const map modules

See [`transictrl/README.md`](transictrl/README.md) for details.

```bash
cd transictrl
cargo test --workspace          # run host-side logic tests
cargo run -p server             # launch the host web UI (simulated telemetry)
cargo run -p server -- -s COM3  # bridge telemetry from a serial port
cargo run -p calibration-cli -- --help

# Build firmware for your board
cargo build -p fw-rp2350 --target thumbv8m.main-none-eabihf
cargo build -p fw-stm32 --target thumbv7em-none-eabihf

# Regenerate map tables from the original C++ maps.h
python transictrl/tools/maps-converter/convert_maps.py \
    main/include/maps.h \
    transictrl/crates/transictrl-722/src/generated_maps.rs
```

---

## Datalogging & visualization

Sensor data is streamed over USB serial and pushed into a local time-series database,
replicated remotely, and rendered by a web frontend for both live and historical
analysis. See [`datalog/README.md`](datalog/README.md) and the `dataviz/` app.

Logged channels include vehicle speed, RPM, throttle position, coolant/oil/ATF
temperature, calculated load, intake/exhaust pressure and their difference, current
boost limit, N2/N3 sensors, evaluated gear, real gear ratio, slippage, and battery
voltage. External datalogging is planned to add GPS location and 3-axis acceleration.

---

## Hardware / PCB

Gerber files and BOMs are available under `pcb/` with three tested, working revisions
(H, I, K). Firmware is compatible across all revisions. See [`pcb/README.md`](pcb/README.md)
for revision notes and assembly caveats. Assembly instructions are welcome as
contributions.

- Working PCB (rev H): <http://www.mui.fi/pcb-valmis.jpg>
- SMD revision (rev K): <http://www.mui.fi/smd-revk.jpg>

---

## License

OpenShift is licensed under the **GNU General Public License v3.0** (GPL-3.0), the same
license as the upstream `7226ctrl` project. See [LICENSE](LICENSE) for the full text.

---

## Credits

Original `7226ctrl` work by **Markus Kovero** (mui@mui.fi), with thanks to:

- Tuomas Kantola — maps and related math
- Tommi Otsavaara — guidance in electronics
- Mikko Kovero and Pete — mechanical side of things
- Jami Karvanen — datalogging and frontend
- Liia Ahola — PCB tracing
- Joosep Vahar — testing
- Toni Lassila and Jan Blonski — ideas
- Krzysztof Dymianiuk — enclosure solution and PCB design

OpenShift fork and the `transictrl` platform maintained by the OpenShift contributors.
