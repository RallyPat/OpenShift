// SPDX-License-Identifier: GPL-3.0-only
// Portions derived from 7226ctrl (c) Markus Kovero et al., GPL-3.0

//! 722.6 sensor scaling and evaluation.
//!
//! This module ports the analog-sensor scaling, N2/N3 ratio evaluation, and
//! slip monitoring from `main/sensors.cpp` and the latter half of `core.cpp`.

use crate::elements::{gear_from_ratio, ratio_from_gear};
use crate::Config722;

/// Evaluate the currently engaged gear from N2/N3 speed sensors and vehicle
/// speed, matching `evaluateGear()` in `core.cpp`.
///
/// - If N3 is zero and vehicle speed is below ~10 km/h, assume 1st.
/// - Otherwise compute shaft ratio and look it up.
pub fn evaluate_gear(n2_speed_hz: f32, n3_speed_hz: f32, vehicle_speed_kmh: f32) -> Option<u8> {
    if n3_speed_hz == 0.0 && vehicle_speed_kmh < 10.0 {
        return Some(1);
    }

    let incoming_shaft_speed = if n3_speed_hz == 0.0 {
        n2_speed_hz * 1.64
    } else {
        n2_speed_hz
    };

    // Avoid divide-by-zero.
    let vehicle_revs = vehicle_speed_kmh / 3.6;
    if vehicle_revs <= 0.0 {
        return None;
    }

    let ratio = incoming_shaft_speed / vehicle_revs;
    gear_from_ratio(ratio)
}

/// Compute converter/gear slip from the measured ratio and the expected ratio
/// for the commanded gear. The original firmware tracked max/min observed ratio
/// per gear over time; this simplified version returns the instantaneous delta.
pub fn gear_slip(commanded_gear: u8, n2_speed_hz: f32, n3_speed_hz: f32, vehicle_speed_kmh: f32) -> f32 {
    let expected_ratio = ratio_from_gear(commanded_gear);
    if expected_ratio <= 0.0 {
        return 0.0;
    }

    let incoming = if n3_speed_hz == 0.0 {
        n2_speed_hz * 1.64
    } else {
        n2_speed_hz
    };

    let vehicle_revs = vehicle_speed_kmh / 3.6;
    if vehicle_revs <= 0.0 {
        return 0.0;
    }

    let measured_ratio = incoming / vehicle_revs;
    (measured_ratio - expected_ratio).abs()
}

/// Check whether observed slip exceeds the configured threshold, matching the
/// logic in `faultMon()`.
pub fn slip_fault(cfg: &Config722, slip: f32, engine_rpm: f32) -> bool {
    slip > cfg.max_slip && engine_rpm > cfg.stall_speed as f32
}

/// Convert battery ADC reading to volts.
///
/// The Teensy uses a voltage divider; on the PCB the battery pin is read with
/// a 1k / 380 divider (12 V -> ~3.15 V at the ADC). The exact divisor is
/// calibration-dependent; this default matches the original wiring.
pub fn battery_voltage_from_adc(count: u16, full_scale_counts: u16, full_scale_voltage: f32) -> f32 {
    let ratio = count as f32 / full_scale_counts.max(1) as f32;
    ratio * full_scale_voltage * ((1000.0 + 380.0) / 380.0)
}

/// Convert throttle-position ADC count to percent.
///
/// Assumes a 5 V / 3 V divider at the ADC, so full throttle is the lower of
/// the ADC reference or the scaled max.
pub fn tps_percent_from_adc(count: u16, min_count: u16, max_count: u16) -> f32 {
    if max_count <= min_count {
        return 0.0;
    }
    let clamped = count.clamp(min_count, max_count);
    let pct = (clamped - min_count) as f32 / (max_count - min_count) as f32 * 100.0;
    pct.clamp(0.0, 100.0)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn evaluate_first_at_standstill() {
        assert_eq!(evaluate_gear(0.0, 0.0, 0.0), Some(1));
        assert_eq!(evaluate_gear(100.0, 0.0, 5.0), Some(1));
    }

    #[test]
    fn tps_scaling() {
        assert_eq!(tps_percent_from_adc(0, 0, 1000), 0.0);
        assert_eq!(tps_percent_from_adc(500, 0, 1000), 50.0);
        assert_eq!(tps_percent_from_adc(1200, 0, 1000), 100.0);
    }
}
