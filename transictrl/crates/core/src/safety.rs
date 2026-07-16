//! Safety and fault detection.
//!
//! Encapsulates the protective checks that, in the original firmware, were
//! scattered through `core.cpp`/`sensors.cpp` (slip faults, battery faults,
//! over-temperature, over-rev). When any check trips the controller enters a
//! limp state: it commands a safe gear with full line pressure and unlocks the
//! torque converter, so a sensor or wiring failure degrades gracefully rather
//! than damaging the transmission.

use crate::config::TransConfig;
use crate::gear::gear_from_ratio;
use crate::math::absf;
use crate::sensors::Sensors;

/// A detected fault condition.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Fault {
    /// Battery/system voltage below the configured minimum.
    LowVoltage,
    /// ATF temperature above the configured maximum.
    OverTemp,
    /// Engine speed above the configured maximum.
    OverRev,
    /// Measured ratio does not match the commanded gear (slip / mis-engagement).
    Slip,
    /// Speed sensor implausibility (e.g. output moving but no input, or input
    /// implausibly high).
    SensorImplausible,
}

/// Evaluate all safety checks for the current tick.
///
/// `commanded_gear` is the gear the controller believes is engaged and
/// `settled` indicates the shift machine is idle (slip checks only make sense
/// once a shift has completed).
pub fn check(
    cfg: &TransConfig,
    sensors: &Sensors,
    commanded_gear: u8,
    settled: bool,
) -> Option<Fault> {
    let lim = &cfg.limits;

    if sensors.battery_v < lim.min_battery_v {
        return Some(Fault::LowVoltage);
    }
    if sensors.atf_temp_c > lim.max_atf_temp_c {
        return Some(Fault::OverTemp);
    }
    if sensors.engine_rpm > lim.max_engine_rpm {
        return Some(Fault::OverRev);
    }

    // Speed-sensor sanity: output spinning meaningfully but input dead is a
    // wiring/sensor failure (or a fully open converter, which we also flag).
    if sensors.output_rpm > 300.0 && sensors.input_rpm < 50.0 {
        return Some(Fault::SensorImplausible);
    }

    // Slip / mis-engagement: only when settled in a real gear and both shafts
    // are turning enough to give a trustworthy ratio.
    if settled && commanded_gear >= 1 {
        if let Some(measured) = sensors.measured_ratio() {
            let eval = gear_from_ratio(cfg, measured);
            match eval.gear {
                // Engaged gear differs from what we commanded -> slip/fault.
                Some(g) if g != commanded_gear => return Some(Fault::Slip),
                // No gear matched at all -> excessive slip.
                None => return Some(Fault::Slip),
                Some(_) => {
                    if let Some(slip) = eval.slip {
                        if absf(slip) > lim.max_slip {
                            return Some(Fault::Slip);
                        }
                    }
                }
            }
        }
    }

    None
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::test_support::cfg_4l60e;
    use crate::sensors::Selector;

    fn healthy(cfg: &TransConfig) -> Sensors {
        let mut s = Sensors::default();
        s.selector = Selector::Drive;
        s.battery_v = 13.8;
        s.atf_temp_c = 80.0;
        s.engine_rpm = 2000.0;
        // 3rd gear ratio 1.00 -> input == output.
        s.input_rpm = 2000.0;
        s.output_rpm = 2000.0;
        let _ = cfg;
        s
    }

    #[test]
    fn healthy_state_no_fault() {
        let cfg = cfg_4l60e();
        let s = healthy(&cfg);
        assert_eq!(check(&cfg, &s, 3, true), None);
    }

    #[test]
    fn low_voltage_trips() {
        let cfg = cfg_4l60e();
        let mut s = healthy(&cfg);
        s.battery_v = 8.0;
        assert_eq!(check(&cfg, &s, 3, true), Some(Fault::LowVoltage));
    }

    #[test]
    fn overtemp_trips() {
        let cfg = cfg_4l60e();
        let mut s = healthy(&cfg);
        s.atf_temp_c = 160.0;
        assert_eq!(check(&cfg, &s, 3, true), Some(Fault::OverTemp));
    }

    #[test]
    fn overrev_trips() {
        let cfg = cfg_4l60e();
        let mut s = healthy(&cfg);
        s.engine_rpm = 7000.0;
        assert_eq!(check(&cfg, &s, 3, true), Some(Fault::OverRev));
    }

    #[test]
    fn wrong_gear_ratio_is_slip() {
        let cfg = cfg_4l60e();
        let mut s = healthy(&cfg);
        // Commanded 3rd (1.00) but shafts show ~1.63 (2nd).
        s.input_rpm = 3260.0;
        s.output_rpm = 2000.0;
        assert_eq!(check(&cfg, &s, 3, true), Some(Fault::Slip));
    }

    #[test]
    fn slip_not_checked_mid_shift() {
        let cfg = cfg_4l60e();
        let mut s = healthy(&cfg);
        s.input_rpm = 3260.0;
        s.output_rpm = 2000.0;
        // Not settled -> ratio mismatch tolerated.
        assert_eq!(check(&cfg, &s, 3, false), None);
    }

    #[test]
    fn dead_input_sensor_trips() {
        let cfg = cfg_4l60e();
        let mut s = healthy(&cfg);
        s.input_rpm = 0.0;
        s.output_rpm = 1500.0;
        assert_eq!(check(&cfg, &s, 3, true), Some(Fault::SensorImplausible));
    }
}
