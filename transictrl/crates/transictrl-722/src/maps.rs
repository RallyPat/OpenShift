// SPDX-License-Identifier: GPL-3.0-only
// Portions derived from 7226ctrl (c) Markus Kovero et al., GPL-3.0

//! 722.6 calibration map data, ported from `main/include/maps.h`.
//!
//! `generated_maps.rs` is produced automatically by
//! `transictrl/tools/maps-converter/convert_maps.py`.

include!("generated_maps.rs");

/// ATF temperature sensor: ADC count -> degrees Celsius.
pub const ATF_SENSOR_MAP: &[(u16, i16)] = &[
    (564, -40),
    (624, -30),
    (686, -20),
    (750, -10),
    (817, 0),
    (886, 10),
    (957, 20),
    (1032, 30),
    (1109, 40),
    (1189, 50),
    (1273, 60),
    (1306, 70),
    (1450, 80),
    (1545, 90),
    (1644, 100),
    (1747, 110),
    (1855, 120),
    (1968, 130),
    (2087, 140),
    (2211, 150),
    (2276, 160),
    (2479, 170),
];

/// Engine oil temperature sensor: ADC count -> degrees Celsius.
pub const OIL_SENSOR_MAP: &[(u16, i16)] = &[
    (130, 120),
    (145, 110),
    (185, 100),
    (245, 90),
    (325, 80),
    (435, 70),
    (600, 60),
    (830, 50),
    (1170, 40),
    (1700, 30),
    (2500, 20),
    (3700, 10),
    (5900, 0),
    (10000, -10),
    (15700, -20),
];

/// Look up a value from a 2-D table whose first row is column breakpoints,
/// first column is row breakpoints, and remaining cells are the data.
///
/// `x` is looked up along columns, `y` along rows. This uses nearest-neighbor
/// with clamping, matching the behavior of the original `readMap`.
pub fn lookup_2d(table: &[[i16; 12]], x: i16, y: i16) -> i16 {
    lookup_2d_n(table, x, y)
}

/// Generic 2-D table lookup for any fixed inner array width.
pub fn lookup_2d_n<const N: usize>(table: &[[i16; N]], x: i16, y: i16) -> i16 {
    if table.len() < 2 {
        return 0;
    }

    let cols = table[0].len();
    let rows = table.len();

    // Find x column.
    let mut xidx = 1usize;
    for i in 1..cols {
        if x <= table[0][i] {
            xidx = i;
            break;
        }
    }

    // Find y row.
    let mut yidx = 1usize;
    for i in 1..rows {
        if y <= table[i][0] {
            yidx = i;
            break;
        }
    }

    table[yidx][xidx]
}

/// Look up a value from a 1-D lookup table using linear interpolation.
pub fn lookup_1d(samples: &[(u16, i16)], key: u16) -> i16 {
    if samples.is_empty() {
        return 0;
    }
    if key <= samples[0].0 {
        return samples[0].1;
    }
    let last = samples[samples.len() - 1];
    if key >= last.0 {
        return last.1;
    }
    for window in samples.windows(2) {
        let (x0, y0) = (window[0].0 as i32, window[0].1 as i32);
        let (x1, y1) = (window[1].0 as i32, window[1].1 as i32);
        let key_i = key as i32;
        if key_i >= x0 && key_i <= x1 {
            let t = (key_i - x0) as f32 / (x1 - x0).max(1) as f32;
            let y = y0 as f32 + t * (y1 - y0) as f32;
            return y as i16;
        }
    }
    last.1
}

/// Convert an ADC count to ATF temperature (°C).
pub fn atf_temp_from_adc(count: u16) -> i16 {
    lookup_1d(ATF_SENSOR_MAP, count)
}

/// Convert an ADC count to oil temperature (°C).
pub fn oil_temp_from_adc(count: u16) -> i16 {
    lookup_1d(OIL_SENSOR_MAP, count)
}

/// Read the original automatic gear map (`gearMap`) at a given TPS/speed.
pub fn decide_auto_gear(tps_pct: u8, speed_kmh: u16) -> u8 {
    lookup_2d(GEARMAP, tps_pct as i16, speed_kmh as i16) as u8
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
    fn atf_lookup_sanity() {
        assert_eq!(atf_temp_from_adc(1450), 80);
        assert_eq!(atf_temp_from_adc(564), -40);
        assert_eq!(atf_temp_from_adc(3000), 170); // clamped high
    }

    #[test]
    fn oil_lookup_sanity() {
        assert_eq!(oil_temp_from_adc(130), 120);
        assert_eq!(oil_temp_from_adc(435), 70);
    }

    #[test]
    fn gear_map_sanity() {
        assert_eq!(decide_auto_gear(0, 0), 2);
        assert_eq!(decide_auto_gear(0, 120), 5);
    }

    #[test]
    fn tps_scaling() {
        assert_eq!(tps_percent_from_adc(0, 0, 1000), 0.0);
        assert_eq!(tps_percent_from_adc(500, 0, 1000), 50.0);
        assert_eq!(tps_percent_from_adc(1200, 0, 1000), 100.0);
    }
}
