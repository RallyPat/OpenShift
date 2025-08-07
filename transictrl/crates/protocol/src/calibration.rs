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
        }
    }
}
