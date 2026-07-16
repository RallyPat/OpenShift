// SPDX-License-Identifier: GPL-3.0-only
// Portions derived from 7226ctrl (c) Markus Kovero et al., GPL-3.0

//! 722.6 runtime configuration, ported from `main/include/config.h`.
//!
//! These parameters live *alongside* the generic `transictrl_core::TransConfig`
//! and describe vehicle/engine/722.6-specific constants: tire size, diff ratio,
//! RPM limits, boost limits, per-shift timing defaults, and feature flags.

use core::fmt;

/// Error returned when validating a 722.6 configuration.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ConfigError {
    /// Tire dimensions must be positive.
    InvalidTireSize,
    /// Diff ratio must be positive.
    InvalidDiffRatio,
    /// High RPM limit must be greater than low RPM limit.
    InvalidRpmLimits,
    /// Battery low-voltage threshold must be positive.
    InvalidBatteryLimit,
}

impl fmt::Display for ConfigError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            ConfigError::InvalidTireSize => write!(f, "tire dimensions must be positive"),
            ConfigError::InvalidDiffRatio => write!(f, "diff ratio must be positive"),
            ConfigError::InvalidRpmLimits => write!(f, "high RPM limit must exceed low RPM limit"),
            ConfigError::InvalidBatteryLimit => write!(f, "battery limit must be positive"),
        }
    }
}

/// Vehicle/engine constants and feature flags derived from the original
/// `ConfigParam` plus global feature flags.
#[derive(Debug, Clone, Copy, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Config722 {
    /// Maximum allowed boost pressure (kPa).
    pub boost_max: u16,
    /// Boost drop target before a shift is allowed to proceed.
    pub boost_drop: u16,
    /// Boost pressure achieved with wastegate spring only (kPa).
    pub boost_spring: u16,
    /// Maximum allowed engine RPM.
    pub fuel_max_rpm: u16,
    /// Redline / high RPM downshift protection threshold.
    pub max_rpm: u16,
    /// Tire width in mm.
    pub tire_width: u16,
    /// Tire profile (aspect ratio, e.g. 45 for 225/45).
    pub tire_profile: u16,
    /// Wheel diameter in inches.
    pub tire_inches: u16,
    /// Number of teeth on the rear diff speed sensor ring.
    pub rear_diff_teeth: u16,
    /// Minimum delay between successive shifts (ms).
    pub next_shift_delay_ms: u32,
    /// Engine RPM below which the torque converter is considered stalled.
    pub stall_speed: u16,
    /// Battery voltage below which limp mode is triggered (Volts * 10).
    pub battery_limit_v10: u16,
    /// First gear that is allowed to use torque-converter lockup.
    pub first_tcc_gear: u8,
    /// Number of teeth on the crank trigger wheel.
    pub trigger_wheel_teeth: u16,
    /// TPS agreement threshold (%).
    pub tps_agree: u8,
    /// RPM above which automatic downshifts are blocked.
    pub high_rpm_shift_limit: u16,
    /// RPM below which automatic upshifts are blocked.
    pub low_rpm_shift_limit: u16,
    /// Final drive ratio.
    pub diff_ratio: f32,
    /// Maximum allowed converter/gear slip before a fault is declared.
    pub max_slip: f32,
    /// "Sloppiness" factor used by the adaptive SPC routine.
    pub trans_sloppy: f32,
    /// Per-shift timing defaults (when not using the full map tables).
    pub shift_timings: ShiftTimings,
    /// Feature flags.
    pub flags: Flags,
}

/// Default fixed shift timings (percent of full pressure), used when the
/// TPS/ATF-based pressure maps are disabled.
#[derive(Debug, Clone, Copy, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct ShiftTimings {
    pub one_to_two: u8,
    pub two_to_three: u8,
    pub three_to_four: u8,
    pub four_to_five: u8,
    pub five_to_four: u8,
    pub four_to_three: u8,
    pub three_to_two: u8,
    pub two_to_one: u8,
}

/// Feature flags from the original firmware.
#[derive(Debug, Clone, Copy, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub struct Flags {
    /// Transmission control enabled.
    pub trans: bool,
    /// Sensor reading enabled.
    pub sensors: bool,
    /// TPS sensor installed.
    pub tps_sensor: bool,
    /// Boost limit feature enabled.
    pub boost_limit: bool,
    /// Drop/boost-limit shifts.
    pub boost_limit_shift: bool,
    /// Adaptive SPC enabled.
    pub adaptive: bool,
    /// Evaluate gear from N2/N3/speed before allowing a shift.
    pub eval_gear: bool,
    /// Use resistive (analog) gearstick instead of digital W202.
    pub resistive_stick: bool,
    /// Stick-control / manual mode active.
    pub stick_ctrl: bool,
    /// Full automatic shift mode active.
    pub full_auto: bool,
    /// Debug serial output enabled.
    pub debug: bool,
    /// CAN bus gearstick/control enabled.
    pub canbus: bool,
}

impl Default for Config722 {
    fn default() -> Self {
        Self {
            boost_max: 200,
            boost_drop: 20,
            boost_spring: 50,
            fuel_max_rpm: 6500,
            max_rpm: 6500,
            tire_width: 225,
            tire_profile: 45,
            tire_inches: 17,
            rear_diff_teeth: 12,
            next_shift_delay_ms: 1000,
            stall_speed: 800,
            battery_limit_v10: 105,
            first_tcc_gear: 5,
            trigger_wheel_teeth: 60,
            tps_agree: 5,
            high_rpm_shift_limit: 5500,
            low_rpm_shift_limit: 1200,
            diff_ratio: 3.27,
            max_slip: 0.15,
            trans_sloppy: 0.05,
            shift_timings: ShiftTimings::default(),
            flags: Flags::default(),
        }
    }
}

impl Default for ShiftTimings {
    fn default() -> Self {
        Self {
            one_to_two: 50,
            two_to_three: 50,
            three_to_four: 50,
            four_to_five: 50,
            five_to_four: 50,
            four_to_three: 50,
            three_to_two: 50,
            two_to_one: 50,
        }
    }
}

impl Default for Flags {
    fn default() -> Self {
        Self {
            trans: true,
            sensors: true,
            tps_sensor: true,
            boost_limit: false,
            boost_limit_shift: false,
            adaptive: false,
            eval_gear: false,
            resistive_stick: false,
            stick_ctrl: false,
            full_auto: true,
            debug: false,
            canbus: false,
        }
    }
}

impl Config722 {
    /// Validate the configuration, returning the first error found.
    pub fn validate(&self) -> Result<(), ConfigError> {
        if self.tire_width == 0 || self.tire_profile == 0 || self.tire_inches == 0 {
            return Err(ConfigError::InvalidTireSize);
        }
        if self.diff_ratio <= 0.0 {
            return Err(ConfigError::InvalidDiffRatio);
        }
        if self.high_rpm_shift_limit <= self.low_rpm_shift_limit {
            return Err(ConfigError::InvalidRpmLimits);
        }
        if self.battery_limit_v10 == 0 {
            return Err(ConfigError::InvalidBatteryLimit);
        }
        Ok(())
    }

    /// Tire rolling radius in millimetres.
    pub fn tire_radius_mm(&self) -> f32 {
        // (width * profile / 100) + (rim inches * 25.4 / 2)
        let sidewall = self.tire_width as f32 * self.tire_profile as f32 / 100.0;
        let rim = self.tire_inches as f32 * 25.4 / 2.0;
        sidewall + rim
    }

    /// Convert rear-diff Hz to vehicle speed (km/h).
    pub fn speed_from_diff_hz(&self, hz: f32) -> f32 {
        if hz <= 0.0 {
            return 0.0;
        }
        let revs_per_second = hz / self.rear_diff_teeth as f32;
        let circumference_m = 2.0 * core::f32::consts::PI * self.tire_radius_mm() / 1000.0;
        revs_per_second * circumference_m * 3.6
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn default_config_validates() {
        let cfg = Config722::default();
        assert!(cfg.validate().is_ok());
    }

    #[test]
    fn speed_calculation_is_nonzero() {
        let cfg = Config722::default();
        let speed = cfg.speed_from_diff_hz(1000.0);
        assert!(speed > 0.0);
    }
}
