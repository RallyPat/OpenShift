//! Automatic gear-selection decision.
//!
//! Ported from the `decideGear` logic in the original `main/core.cpp`. The
//! desired gear is looked up from a 2-D *shift map* indexed by road speed
//! (rows) and throttle position (columns); the map's stored value is the gear
//! number to be in. Hysteresis is applied so the controller does not oscillate
//! (hunt) between two gears at a boundary.

use crate::config::TransConfig;
use crate::interp::Map2d;
use crate::sensors::{Selector, Sensors};

/// Inputs needed to decide the desired gear in automatic mode.
pub struct DecisionInput<'a> {
    /// 2-D shift map: X axis = throttle %, Y axis = road speed (km/h), value =
    /// desired gear number.
    pub shift_map: Map2d<'a>,
    /// Gear currently engaged (for hysteresis).
    pub current_gear: u8,
}

/// Decide the desired gear for the current conditions.
///
/// Returns a clamped, hysteresis-filtered 1-based gear number. In `Manual`
/// mode the caller drives gear changes directly, so this returns
/// `current_gear` unchanged.
pub fn decide_gear(cfg: &TransConfig, sensors: &Sensors, input: &DecisionInput) -> u8 {
    if cfg.gear_count == 0 {
        return 0;
    }
    // Only auto-select in Drive; other selectors are handled by the controller.
    if sensors.selector != Selector::Drive {
        return input.current_gear.clamp(1, cfg.gear_count);
    }

    let raw = input
        .shift_map
        .get(sensors.tps_pct, sensors.vehicle_speed);
    // Round to nearest gear and clamp to the valid range.
    let mut desired = round_to_gear(raw, cfg.gear_count);

    // Hysteresis: require the map to ask for a change of a full gear before
    // acting, preventing hunting right at a cell boundary.
    let cur = input.current_gear;
    if cur >= 1 && cur <= cfg.gear_count {
        if desired > cur && raw < (cur as f32) + 1.0 - HYST {
            desired = cur;
        } else if desired < cur && raw > (cur as f32) - 1.0 + HYST {
            desired = cur;
        }
    }
    desired
}

/// Hysteresis band, in gears, applied around an integer boundary.
const HYST: f32 = 0.3;

#[inline]
fn round_to_gear(raw: f32, gear_count: u8) -> u8 {
    if raw <= 1.0 {
        return 1;
    }
    let n = (raw + 0.5) as i32;
    n.clamp(1, gear_count as i32) as u8
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::test_support::cfg_4l60e;

    // Shift map for a 4-speed: X = throttle (0,100), Y = speed (0..120).
    // Low speed -> 1st, rising through to 4th at speed. Two throttle columns.
    fn map_data() -> ([f32; 2], [f32; 5], [f32; 10]) {
        let x = [0.0, 100.0]; // throttle columns
        let y = [0.0, 30.0, 60.0, 90.0, 120.0]; // speed rows
                                                 // value = desired gear; wide-open throttle holds lower gears longer.
                                                 // row-major: each row has 2 entries (light, heavy throttle)
        let v = [
            1.0, 1.0, // 0 km/h
            2.0, 1.0, // 30
            3.0, 2.0, // 60
            4.0, 3.0, // 90
            4.0, 4.0, // 120
        ];
        (x, y, v)
    }

    #[test]
    fn light_throttle_cruise_picks_top_gear() {
        let cfg = cfg_4l60e();
        let (x, y, v) = map_data();
        let map = Map2d::new(&x, &y, &v).unwrap();
        let mut s = Sensors::default();
        s.selector = Selector::Drive;
        s.tps_pct = 0.0;
        s.vehicle_speed = 120.0;
        let g = decide_gear(&cfg, &s, &DecisionInput { shift_map: map, current_gear: 4 });
        assert_eq!(g, 4);
    }

    #[test]
    fn wide_open_throttle_holds_lower_gear() {
        let cfg = cfg_4l60e();
        let (x, y, v) = map_data();
        let map = Map2d::new(&x, &y, &v).unwrap();
        let mut s = Sensors::default();
        s.selector = Selector::Drive;
        s.tps_pct = 100.0;
        s.vehicle_speed = 60.0;
        let g = decide_gear(&cfg, &s, &DecisionInput { shift_map: map, current_gear: 3 });
        assert_eq!(g, 2); // heavy throttle at 60 km/h -> 2nd
    }

    #[test]
    fn manual_selector_holds_current() {
        let cfg = cfg_4l60e();
        let (x, y, v) = map_data();
        let map = Map2d::new(&x, &y, &v).unwrap();
        let mut s = Sensors::default();
        s.selector = Selector::Manual;
        s.vehicle_speed = 120.0;
        let g = decide_gear(&cfg, &s, &DecisionInput { shift_map: map, current_gear: 2 });
        assert_eq!(g, 2);
    }
}
