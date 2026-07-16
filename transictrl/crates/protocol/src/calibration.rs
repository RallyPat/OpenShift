use serde::{Deserialize, Serialize};
use serde_with::{serde_as, DisplayFromStr};
use semver::Version;

#[serde_as]
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct CalibrationFile {
    #[serde_as(as = "DisplayFromStr")]
    pub schema_version: Version,
    pub metadata: CalibrationMetadata,
    pub transmission: TransmissionInfo,
    pub scalars: Vec<Scalar>,
    pub tables: Vec<Table>,
    pub io_mapping: IoMapping,
    /// Gearbox definition: ratios, shift-element matrix, limits and timing.
    /// Optional for backward compatibility with early minimal calibrations.
    #[serde(default)]
    pub gearbox: Gearbox,
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct CalibrationMetadata {
    pub id: String,
    pub name: String,
    pub author: Option<String>,
    pub description: Option<String>,
    pub created_utc: Option<String>,
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct TransmissionInfo {
    pub model: String,
    pub gear_count: u8,
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct Scalar {
    pub name: String,
    pub unit: Option<String>,
    pub value: f64,
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct Table {
    pub name: String,
    pub x_axis: Axis,
    pub y_axis: Option<Axis>,
    pub values: Vec<f64>,
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct Axis {
    pub label: String,
    pub unit: Option<String>,
    pub points: Vec<f64>,
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Default)]
pub struct IoMapping {
    pub analog_inputs: Vec<AnalogInput>,
    pub digital_inputs: Vec<DigitalInput>,
    pub pwm_outputs: Vec<PwmOutput>,
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct AnalogInput {
    pub name: String,
    pub channel: u8,
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct DigitalInput {
    pub name: String,
    pub channel: u8,
    pub active_high: bool,
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct PwmOutput {
    pub name: String,
    pub channel: u8,
    pub frequency_hz: u32,
}

/// State a shift element must be in for a given gear.
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "lowercase")]
pub enum ElementStateCal {
    /// Engaged (solenoid energized / clutch or brake applied).
    Applied,
    /// Disengaged (solenoid de-energized / clutch or brake released).
    Released,
    /// State irrelevant for this gear.
    DontCare,
}

/// One forward gear: its ratio and the required state of every shift element.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct GearCal {
    /// 1-based gear number.
    pub number: u8,
    /// Theoretical ratio (input speed / output speed).
    pub ratio: f32,
    /// Required state per shift element, in the same order as
    /// [`Gearbox::shift_elements`].
    #[serde(default)]
    pub elements: Vec<ElementStateCal>,
    /// Whether torque-converter lockup is permitted in this gear.
    #[serde(default)]
    pub lockup_allowed: bool,
}

/// Protective limits for the transmission.
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
pub struct LimitsCal {
    pub ratio_tolerance: f32,
    pub max_slip: f32,
    pub min_battery_v: f32,
    pub max_engine_rpm: f32,
    pub shift_lockout_ms: u32,
    pub max_atf_temp_c: f32,
}

impl Default for LimitsCal {
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

/// Default per-shift timing (milliseconds).
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
pub struct ShiftTimingCal {
    pub preshift_ms: u32,
    pub shift_ms: u32,
    pub postshift_ms: u32,
}

impl Default for ShiftTimingCal {
    fn default() -> Self {
        Self {
            preshift_ms: 80,
            shift_ms: 450,
            postshift_ms: 120,
        }
    }
}

/// Complete gearbox definition.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Default)]
pub struct Gearbox {
    /// Final-drive / axle ratio.
    #[serde(default)]
    pub final_drive: f32,
    /// Names of the logical shift elements (solenoids / clutches / brakes).
    #[serde(default)]
    pub shift_elements: Vec<String>,
    /// Per-gear definitions.
    #[serde(default)]
    pub gears: Vec<GearCal>,
    /// Protective limits.
    #[serde(default)]
    pub limits: LimitsCal,
    /// Default shift timing.
    #[serde(default)]
    pub timing: ShiftTimingCal,
}

/// Validation problems found in a calibration file.
#[derive(Debug, Clone, PartialEq)]
pub enum ValidationError {
    /// `transmission.gear_count` disagrees with the number of gear entries.
    GearCountMismatch { declared: u8, found: usize },
    /// A gear's `number` is out of the `1..=gear_count` range.
    BadGearNumber(u8),
    /// Gears are not numbered 1..=gear_count exactly once each.
    NonContiguousGears,
    /// A gear has a non-positive ratio.
    BadRatio(u8),
    /// Ratios are not strictly decreasing from gear 1 upward.
    RatioOrder(u8),
    /// A gear's element count differs from `shift_elements.len()`.
    ElementCountMismatch { gear: u8, expected: usize, found: usize },
}

impl CalibrationFile {
    pub fn new_minimal(id: impl Into<String>, name: impl Into<String>) -> Self {
        Self {
            schema_version: Version::new(0, 1, 0),
            metadata: CalibrationMetadata {
                id: id.into(),
                name: name.into(),
                author: None,
                description: None,
                created_utc: None,
            },
            transmission: TransmissionInfo { model: "unknown".into(), gear_count: 0 },
            scalars: Vec::new(),
            tables: Vec::new(),
            io_mapping: IoMapping::default(),
            gearbox: Gearbox::default(),
        }
    }

    /// Validate the gearbox definition for internal consistency. Calibrations
    /// without a gearbox section (legacy minimal files) are accepted as-is.
    pub fn validate(&self) -> Result<(), ValidationError> {
        let gb = &self.gearbox;
        if gb.gears.is_empty() {
            return Ok(());
        }

        let declared = self.transmission.gear_count;
        if declared as usize != gb.gears.len() {
            return Err(ValidationError::GearCountMismatch {
                declared,
                found: gb.gears.len(),
            });
        }

        // Gears must be numbered 1..=declared exactly once, with strictly
        // decreasing ratios and matching element counts.
        let elem_count = gb.shift_elements.len();
        let mut seen = [false; 256];
        let mut prev_ratio = f32::INFINITY;
        // Walk in declared order to validate ratio ordering by gear number.
        let mut sorted: Vec<&GearCal> = gb.gears.iter().collect();
        sorted.sort_by_key(|g| g.number);

        for g in &sorted {
            if g.number < 1 || g.number > declared {
                return Err(ValidationError::BadGearNumber(g.number));
            }
            if seen[g.number as usize] {
                return Err(ValidationError::NonContiguousGears);
            }
            seen[g.number as usize] = true;

            if g.ratio <= 0.0 {
                return Err(ValidationError::BadRatio(g.number));
            }
            if g.ratio >= prev_ratio {
                return Err(ValidationError::RatioOrder(g.number));
            }
            prev_ratio = g.ratio;

            if !g.elements.is_empty() && g.elements.len() != elem_count {
                return Err(ValidationError::ElementCountMismatch {
                    gear: g.number,
                    expected: elem_count,
                    found: g.elements.len(),
                });
            }
        }

        for n in 1..=declared {
            if !seen[n as usize] {
                return Err(ValidationError::NonContiguousGears);
            }
        }

        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn cal_4l60e() -> CalibrationFile {
        use ElementStateCal::{Applied, Released};
        let mut cal = CalibrationFile::new_minimal("4l60e", "GM 4L60E");
        cal.transmission = TransmissionInfo {
            model: "GM 4L60E".into(),
            gear_count: 4,
        };
        cal.gearbox.final_drive = 3.42;
        cal.gearbox.shift_elements = vec!["SS-A".into(), "SS-B".into()];
        cal.gearbox.gears = vec![
            GearCal { number: 1, ratio: 3.06, elements: vec![Applied, Applied], lockup_allowed: false },
            GearCal { number: 2, ratio: 1.63, elements: vec![Released, Applied], lockup_allowed: false },
            GearCal { number: 3, ratio: 1.00, elements: vec![Released, Released], lockup_allowed: true },
            GearCal { number: 4, ratio: 0.70, elements: vec![Applied, Released], lockup_allowed: true },
        ];
        cal
    }

    #[test]
    fn valid_4l60e_passes() {
        assert_eq!(cal_4l60e().validate(), Ok(()));
    }

    #[test]
    fn legacy_minimal_is_accepted() {
        assert_eq!(CalibrationFile::new_minimal("x", "y").validate(), Ok(()));
    }

    #[test]
    fn gear_count_mismatch_detected() {
        let mut cal = cal_4l60e();
        cal.transmission.gear_count = 5;
        assert_eq!(
            cal.validate(),
            Err(ValidationError::GearCountMismatch { declared: 5, found: 4 })
        );
    }

    #[test]
    fn ratio_order_detected() {
        let mut cal = cal_4l60e();
        cal.gearbox.gears[1].ratio = 4.0; // 2nd higher than 1st
        assert_eq!(cal.validate(), Err(ValidationError::RatioOrder(2)));
    }

    #[test]
    fn element_count_mismatch_detected() {
        let mut cal = cal_4l60e();
        cal.gearbox.gears[0].elements = vec![ElementStateCal::Applied]; // only 1 of 2
        assert_eq!(
            cal.validate(),
            Err(ValidationError::ElementCountMismatch { gear: 1, expected: 2, found: 1 })
        );
    }

    #[test]
    fn roundtrips_through_toml() {
        let cal = cal_4l60e();
        let s = toml::to_string_pretty(&cal).unwrap();
        let back: CalibrationFile = toml::from_str(&s).unwrap();
        assert_eq!(cal, back);
    }
}
