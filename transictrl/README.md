# transictrl

An open, lawful, safety-first transmission controller platform. Licensed under GPL-3.0-only.

- **Core** — hardware-independent shift state machine, gear-ratio evaluation, map
  interpolation, and safety/fault handling (`transictrl-core`)
- **722.6 logic** — GPL-3.0-derived port of the original `7226ctrl` control
  strategy, sensor scaling, and map data (`transictrl-722`)
- **HAL** — board-agnostic traits for solenoids, PWM, ADC, frequency capture,
  CAN, persistent storage, serial, and display (`transictrl-hal`)
- **Firmware** — embassy-based concrete HALs and main loops for RP2350
  (`fw-rp2350`) and STM32F4 (`fw-stm32`); Teensy 4 (`fw-teensy4`) is a skeleton
- **Host** — web UI served by Rust (axum), a native egui dataviz app, a device
  simulator, and a calibration CLI (`host/server`, `host/dataviz`, `host/sim`,
  `tools/calibration-cli`). The server can bridge a real serial telemetry stream
  to WebSocket clients.
- **Protocol** — open serialization with serde/postcard, plus a no-std binary
  framing layer for calibration push/pull over serial
- **Calibration** — open TOML/JSON schema, optional user-supplied importers

## Workspace layout

| Path | Crate / app | Purpose |
| ---- | ----------- | ------- |
| `crates/core` | `transictrl-core` | Generic, `#![no_std]` shift controller logic |
| `crates/hal` | `transictrl-hal` | Board-agnostic hardware traits and serial framing |
| `crates/protocol` | `protocol` | Calibration and telemetry serialization |
| `crates/transictrl-722` | `transictrl-722` | 722.6-specific strategy, maps, and inputs |
| `fw/fw-rp2350` | `fw-rp2350` | Embassy firmware for RP2040/RP2350 |
| `fw/fw-stm32` | `fw-stm32` | Embassy firmware for STM32F4 |
| `fw/fw-teensy4` | `fw-teensy4` | Skeleton for Teensy 4 / i.MX RT |
| `host/server` | `server` | axum HTTP + WebSocket host; serial bridge |
| `host/dataviz` | `dataviz` | Native egui telemetry viewer |
| `host/sim` | `sim` | Mock-HAL controller simulator |
| `tools/calibration-cli` | `calibration-cli` | CLI for calibration validate/pack/import/push/pull |
| `tools/maps-converter` | — | Python helper to port `main/include/maps.h` |

Legal: No proprietary code or reverse-engineered material included.
US Shift format compatibility is via user-provided mapping/import modules loaded at runtime; we do not ship proprietary parsers.

Not for on-road use without compliance with local laws.

## Quick start

```bash
cd transictrl
cargo test --workspace          # run host-side logic tests
cargo run -p server             # launch the host web UI (simulated telemetry)
cargo run -p server -- -s COM3  # bridge telemetry from a serial port
cargo run -p calibration-cli -- --help
cargo build -p fw-rp2350 --target thumbv8m.main-none-eabihf
cargo build -p fw-stm32 --target thumbv7em-none-eabihf
```
