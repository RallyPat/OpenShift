use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum HostToDevice {
    ReadDeviceInfo,
    PushCalibration(super::calibration::CalibrationFile),
    RequestLogSnapshot,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum DeviceToHost {
    DeviceInfo { firmware_version: String, hardware: String },
    Ack,
    LogLine { level: String, message: String },
}
