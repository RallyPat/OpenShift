//! Runtime transmission configuration.
//!
//! This is the *decoded* form of a calibration file used by the control logic.
//! It is deliberately fixed-capacity (no heap allocation) so it can live on an
//! embedded target. The host side reads a richer TOML/JSON calibration and
//! lowers it into this structure.
//!
//! ## The shift-element matrix
//!
//! The single abstraction that lets one engine drive very different gearboxes
//! is the per-gear **shift-element matrix**. Each transmission has up to
//! [`MAX_ELEMENTS`] logical *shift elements*. For a direct-solenoid gearbox an
//! element is a shift solenoid (e.g. GM 4L60E `SS-A`/`SS-B`); for a
//! clutch-to-clutch gearbox an element is a clutch or brake (e.g. ZF 8HP
//! `Brake A/B`, `Clutch C/D/E`). Every gear declares the state each element
//! must be in. The firmware maps each element to a physical output (a GPIO, a
//! PWM channel or a pressure-control valve).
//!
//! Example — GM 4L60E (2 elements: SS-A, SS-B):
//! ```text
//!   Gear 1: [Applied,  Applied ]
//!   Gear 2: [Released, Applied ]
//!   Gear 3: [Released, Released]
//!   Gear 4: [Applied,  Released]
//! ```

/// Maximum number of forward gears supported (covers 10-speed units and more).
pub const MAX_GEARS: usize = 10;

/// Maximum number of logical shift elements (solenoids / clutches / brakes).
pub const MAX_ELEMENTS: usize = 6;

/// State a single shift element must be in for a given gear, or the state the
/// controller is commanding for an output.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ElementState {
    /// Element engaged (solenoid energized / clutch or brake applied).
    Applied,
    /// Element disengaged (solenoid de-energized / clutch or brake released).
    Released,
    /// State irrelevant for this gear (controller leaves it as-is).
    DontCare,
}

impl ElementState {
    /// Convenience: is this element energized/applied?
    #[inline]
    pub fn is_applied(self) -> bool {
        matches!(self, ElementState::Applied)
    }
}

/// Description of a single forward gear.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GearProfile {
    /// Theoretical gear ratio (input speed / output speed).
    pub ratio: f32,
    /// Required state of every shift element while in this gear.
    pub elements: [ElementState; MAX_ELEMENTS],
    /// Whether torque-converter lockup is *allowed* in this gear.
    pub lockup_allowed: bool,
}

impl GearProfile {
    /// A neutral profile (used to pad the fixed-size gear array).
    pub const fn empty() -> Self {
        Self {
            ratio: 0.0,
            elements: [ElementState::DontCare; MAX_ELEMENTS],
            lockup_allowed: false,
        }
    }
}

/// Limits and tuning knobs that are not part of any lookup table.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Limits {
    /// Fractional tolerance used when identifying the engaged gear from the
    /// measured input/output speed ratio (e.g. `0.08` = ±8 %).
    pub ratio_tolerance: f32,
    /// Maximum allowed slip (1 - achieved/expected ratio) before a fault.
    pub max_slip: f32,
    /// Minimum healthy battery voltage; below this the controller faults.
    pub min_battery_v: f32,
    /// Engine over-rev limit used to inhibit downshifts.
    pub max_engine_rpm: f32,
    /// Minimum dwell time (ms) between consecutive shifts.
    pub shift_lockout_ms: u32,
    /// ATF temperature (°C) above which the unit is considered overheating.
    pub max_atf_temp_c: f32,
}

impl Default for Limits {
    fn default() -> Self {
        Self {
            ratio_tolerance: 0.08,
            max_slip: 0.4,
            min_battery_v: 9.0,
            max_engine_rpm: 6500.0,
            shift_lockout_ms: 800,
            max_atf_temp_c: 150.0,
        }
    }
}

/// Timing for a single shift event, in milliseconds.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ShiftTiming {
    /// Pre-shift phase: prepare (e.g. drop boost, pre-fill) before the swap.
    pub preshift_ms: u32,
    /// Shift phase: elements are swapped and shift pressure applied.
    pub shift_ms: u32,
    /// Post-shift phase: settle and confirm the new gear is engaged.
    pub postshift_ms: u32,
}

impl Default for ShiftTiming {
    fn default() -> Self {
        Self {
            preshift_ms: 80,
            shift_ms: 450,
            postshift_ms: 120,
        }
    }
}

/// Complete runtime configuration for one transmission.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TransConfig {
    /// Number of valid forward gears (`1..=MAX_GEARS`).
    pub gear_count: u8,
    /// Number of logical shift elements actually used (`0..=MAX_ELEMENTS`).
    pub element_count: u8,
    /// Per-gear profiles. Only the first `gear_count` entries are valid.
    pub gears: [GearProfile; MAX_GEARS],
    /// Final-drive / axle ratio, used to relate output-shaft and wheel speed.
    pub final_drive: f32,
    /// Limits and protections.
    pub limits: Limits,
    /// Default shift timing (tables may refine this per-shift later).
    pub timing: ShiftTiming,
}

impl TransConfig {
    /// An empty (invalid) configuration with sensible defaults; callers fill in
    /// `gear_count`, `element_count` and `gears`.
    pub fn empty() -> Self {
        Self {
            gear_count: 0,
            element_count: 0,
            gears: [GearProfile::empty(); MAX_GEARS],
            final_drive: 3.42,
            limits: Limits::default(),
            timing: ShiftTiming::default(),
        }
    }

    /// Returns the profile for a 1-based gear number, if valid.
    #[inline]
    pub fn gear(&self, gear: u8) -> Option<&GearProfile> {
        if gear >= 1 && gear <= self.gear_count {
            Some(&self.gears[(gear - 1) as usize])
        } else {
            None
        }
    }

    /// Theoretical ratio for a 1-based gear number.
    #[inline]
    pub fn ratio_for_gear(&self, gear: u8) -> Option<f32> {
        self.gear(gear).map(|g| g.ratio)
    }

    /// Highest valid gear number.
    #[inline]
    pub fn top_gear(&self) -> u8 {
        self.gear_count
    }

    /// Basic structural validation of the configuration.
    pub fn validate(&self) -> Result<(), ConfigError> {
        if self.gear_count == 0 || self.gear_count as usize > MAX_GEARS {
            return Err(ConfigError::GearCount);
        }
        if self.element_count as usize > MAX_ELEMENTS {
            return Err(ConfigError::ElementCount);
        }
        // Ratios must be strictly decreasing (gear 1 highest numerical ratio).
        let mut prev = f32::INFINITY;
        for g in 1..=self.gear_count {
            let r = self.gears[(g - 1) as usize].ratio;
            if r <= 0.0 {
                return Err(ConfigError::Ratio(g));
            }
            if r >= prev {
                return Err(ConfigError::RatioOrder(g));
            }
            prev = r;
        }
        Ok(())
    }
}

/// Errors produced by [`TransConfig::validate`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ConfigError {
    /// `gear_count` is zero or exceeds [`MAX_GEARS`].
    GearCount,
    /// `element_count` exceeds [`MAX_ELEMENTS`].
    ElementCount,
    /// Gear `n` has a non-positive ratio.
    Ratio(u8),
    /// Gear `n` does not have a strictly smaller ratio than the gear below it.
    RatioOrder(u8),
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::test_support::cfg_4l60e;

    #[test]
    fn valid_config_passes() {
        assert_eq!(cfg_4l60e().validate(), Ok(()));
    }

    #[test]
    fn rejects_bad_ratio_order() {
        let mut cfg = cfg_4l60e();
        cfg.gears[1].ratio = 5.0; // 2nd higher than 1st
        assert_eq!(cfg.validate(), Err(ConfigError::RatioOrder(2)));
    }

    #[test]
    fn rejects_zero_gears() {
        let cfg = TransConfig::empty();
        assert_eq!(cfg.validate(), Err(ConfigError::GearCount));
    }

    #[test]
    fn gear_lookup_is_one_based() {
        let cfg = cfg_4l60e();
        assert_eq!(cfg.ratio_for_gear(1), Some(3.06));
        assert_eq!(cfg.ratio_for_gear(4), Some(0.70));
        assert_eq!(cfg.ratio_for_gear(0), None);
        assert_eq!(cfg.ratio_for_gear(5), None);
    }
}
