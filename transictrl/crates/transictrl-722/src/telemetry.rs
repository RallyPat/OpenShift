// SPDX-License-Identifier: GPL-3.0-only
// Portions derived from 7226ctrl (c) Markus Kovero et al., GPL-3.0

//! Telemetry / datalogging output.
//!
//! Streams runtime state over the serial link using a compact CSV-style
//! format. This is the Rust equivalent of the serial logging in
//! `main/serial_config.cpp`.

use transictrl_hal::SerialLink;

use crate::sensor_input::RawReadings;

/// Compact telemetry snapshot (no_std friendly).
#[derive(Debug, Clone, Copy, Default)]
pub struct TelemetryFrame {
    pub vehicle_speed_kmh: f32,
    pub engine_rpm: f32,
    pub tps_percent: f32,
    pub oil_temp_c: f32,
    pub atf_temp_c: f32,
    pub load_percent: f32,
    pub boost_kpa: f32,
    pub exhaust_temp_c: f32,
    pub battery_v: f32,
    pub current_gear: u8,
    pub target_gear: u8,
    pub shifting: bool,
}

/// Telemetry streamer.
pub struct Telemetry<S: SerialLink> {
    link: S,
    /// Output interval in milliseconds.
    pub interval_ms: u32,
}

impl<S: SerialLink> Telemetry<S> {
    pub fn new(link: S) -> Self {
        Self {
            link,
            interval_ms: 100,
        }
    }

    /// Send one CSV line of raw sensor data.
    pub fn report(&mut self, r: &RawReadings) -> Result<(), S::Error> {
        let mut buf = heapless::String::<256>::new();
        use core::fmt::Write;
        let _ = writeln!(
            buf,
            "{},{},{},{},{},{},{},{},{},{},{}\r\n",
            r.n2_hz,
            r.n3_hz,
            r.vehicle_speed_hz,
            r.engine_rpm,
            r.tps_adc,
            r.atf_adc,
            r.oil_adc,
            r.boost_adc,
            r.exhaust_adc,
            r.battery_adc,
            r.stick.encode()
        );
        self.link.write(buf.as_bytes())
    }
}
