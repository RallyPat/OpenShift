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
    Telemetry(TelemetryFrame),
}

/// A single telemetry snapshot from the device.
///
/// Fields are kept close to the legacy CSV format so existing front-ends can
/// continue to consume `[value, ...]` arrays while richer clients can use the
/// named fields.
#[derive(Debug, Clone, Copy, Default, Serialize, Deserialize)]
pub struct TelemetryFrame {
    pub vehicle_speed_kmh: f32,
    pub engine_rpm: f32,
    pub tps_percent: f32,
    pub oil_temp_c: f32,
    pub atf_temp_c: f32,
    pub load_percent: f32,
    pub boost_kpa: f32,
    pub exhaust_temp_c: f32,
    pub battery_v: f32,
    pub current_gear: u8,
    pub target_gear: u8,
    pub shifting: bool,
}

impl TelemetryFrame {
    /// Serialize the frame as the legacy 12-element JSON array expected by the
    /// original React dataviz app.
    pub fn to_legacy_array(&self) -> [f32; 12] {
        [
            self.vehicle_speed_kmh,
            self.engine_rpm,
            self.tps_percent,
            self.oil_temp_c,
            self.atf_temp_c,
            self.load_percent,
            self.boost_kpa,
            self.exhaust_temp_c,
            self.battery_v,
            self.current_gear as f32,
            self.target_gear as f32,
            if self.shifting { 1.0 } else { 0.0 },
        ]
    }
}
