// SPDX-License-Identifier: GPL-3.0-only
// Portions derived from 7226ctrl (c) Markus Kovero et al., GPL-3.0

//! Sensor-input pipeline.
//!
//! Reads the HAL peripherals once per tick and converts raw ADC counts / pulse
//! frequencies into the [`transictrl_core::Sensors`] snapshot used by the
//! controller. This mirrors `main/sensors.cpp`.

use transictrl_core::Sensors;

use crate::config::Config722;
use crate::maps;
use crate::stick::{DigitalStick, ResistiveStick, SelectorPos};

/// Raw peripheral readings from the board.
pub struct RawReadings {
    pub n2_hz: f32,
    pub n3_hz: f32,
    pub vehicle_speed_hz: f32,
    pub engine_rpm: f32,
    pub tps_adc: u16,
    pub atf_adc: u16,
    pub oil_adc: u16,
    pub boost_adc: u16,
    pub exhaust_adc: u16,
    pub battery_adc: u16,
    pub stick: StickReading,
}

/// Gearstick reading: either digital W202 lines or a single resistive ADC.
pub enum StickReading {
    Digital { white: bool, blue: bool, green: bool, yellow: bool, auto_switch: bool },
    Resistive(u16),
}

impl RawReadings {
    /// Convert raw readings into the generic sensor snapshot.
    ///
    /// `cfg` provides scaling constants (tire size, diff ratio, etc.).
    pub fn to_sensors(&self, cfg: &Config722) -> Sensors {
        let tps_pct = if cfg.flags.tps_sensor {
            // TODO(port): read calibrated TPS min/max from EEPROM/persist.
            maps::tps_percent_from_adc(self.tps_adc, 0, 1023)
        } else {
            100.0
        };

        let atf_temp_c = maps::atf_temp_from_adc(self.atf_adc) as f32;
        let _oil_temp_c = maps::oil_temp_from_adc(self.oil_adc);
        let battery_v = maps::battery_voltage_from_adc(self.battery_adc, 4095, 3.3);

        let vehicle_speed_kmh = cfg.speed_from_diff_hz(self.vehicle_speed_hz);

        Sensors {
            input_rpm: self.engine_rpm, // TODO(port): compute turbine RPM from N2/N3.
            output_rpm: self.vehicle_speed_hz * 60.0 / cfg.rear_diff_teeth as f32,
            engine_rpm: self.engine_rpm,
            tps_pct,
            vehicle_speed: vehicle_speed_kmh,
            atf_temp_c,
            line_pressure: 0.0, // TODO(port): add pressure transducer support.
            battery_v,
            brake: false, // TODO(port): brake switch input.
            selector: crate::controller::selector_to_generic(self.stick.decode(cfg), self.manual_mode()),
        }
    }

    fn manual_mode(&self) -> bool {
        match &self.stick {
            StickReading::Digital { auto_switch, .. } => *auto_switch,
            StickReading::Resistive(_) => false,
        }
    }
}

impl StickReading {
    /// Encode the stick as a small integer for telemetry.
    pub fn encode(&self) -> u8 {
        match self {
            StickReading::Digital { white, blue, green, yellow, auto_switch } => {
                let mut v = 0u8;
                if *white { v |= 1 << 0; }
                if *blue { v |= 1 << 1; }
                if *green { v |= 1 << 2; }
                if *yellow { v |= 1 << 3; }
                if *auto_switch { v |= 1 << 4; }
                v
            }
            StickReading::Resistive(v) => ((*v).min(255) as u8) | 0x80,
        }
    }

    /// Decode the stick into a selector position.
    pub fn decode(&self, _cfg: &Config722) -> SelectorPos {
        match self {
            StickReading::Digital {
                white,
                blue,
                green,
                yellow,
                ..
            } => DigitalStick {
                white: *white,
                blue: *blue,
                green: *green,
                yellow: *yellow,
                auto_switch: false,
            }
            .decode(),
            StickReading::Resistive(v) => ResistiveStick(*v).decode(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn raw_default() -> RawReadings {
        RawReadings {
            n2_hz: 0.0,
            n3_hz: 0.0,
            vehicle_speed_hz: 0.0,
            engine_rpm: 0.0,
            tps_adc: 0,
            atf_adc: 1450,
            oil_adc: 435,
            boost_adc: 0,
            exhaust_adc: 0,
            battery_adc: 4095,
            stick: StickReading::Digital {
                white: true,
                blue: true,
                green: true,
                yellow: false,
                auto_switch: false,
            },
        }
    }

    #[test]
    fn park_decoder() {
        let raw = raw_default();
        let cfg = Config722::default();
        let s = raw.to_sensors(&cfg);
        assert_eq!(s.selector, transictrl_core::sensors::Selector::Park);
    }
}
