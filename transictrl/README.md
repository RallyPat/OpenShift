# transictrl

An open, lawful, safety-first transmission controller platform. Licensed under GPL-2.0-only.

- Firmware: Rust-based, portable HAL (e.g., RP2040/RP2350, STM32, ESP32)
- Host: Web UI served by Rust (axum), no proprietary dependencies
- Protocol: Open serialization with serde/postcard
- Calibration: Open TOML/JSON schema, optional user-supplied importers

Legal: No proprietary code or reverse-engineered material included.
US Shift format compatibility is via user-provided mapping/import modules loaded at runtime; we do not ship proprietary parsers.

Not for on-road use without compliance with local laws.
