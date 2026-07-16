//! Sensor snapshot consumed by the control logic.
//!
//! The firmware reads ADCs, speed-capture timers and digital inputs, fills in a
//! [`Sensors`] value once per control tick and hands it to the
//! [`crate::Controller`]. Keeping this as a plain data struct makes the logic
//! trivially testable: a unit test can synthesize any driving scenario.

/// Gear-selector (PRNDL) position requested by the driver.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Selector {
    Park,
    Reverse,
    Neutral,
    /// Full automatic drive.
    Drive,
    /// Manual / sport mode (driver requests gears).
    Manual,
}

/// One coherent snapshot of every input the controller cares about.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Sensors {
    /// Transmission input-shaft / turbine speed (RPM). `0` if not fitted.
    pub input_rpm: f32,
    /// Transmission output-shaft speed (RPM).
    pub output_rpm: f32,
    /// Engine speed (RPM).
    pub engine_rpm: f32,
    /// Throttle position, 0..=100 %.
    pub tps_pct: f32,
    /// Road speed (km/h), derived from output speed and final drive/tyre size.
    pub vehicle_speed: f32,
    /// Automatic-transmission-fluid temperature (°C).
    pub atf_temp_c: f32,
    /// Measured line pressure (kPa); `0` if no pressure transducer.
    pub line_pressure: f32,
    /// System/battery voltage (V).
    pub battery_v: f32,
    /// Brake pedal pressed.
    pub brake: bool,
    /// Current gear-selector position.
    pub selector: Selector,
}

impl Default for Sensors {
    fn default() -> Self {
        Self {
            input_rpm: 0.0,
            output_rpm: 0.0,
            engine_rpm: 0.0,
            tps_pct: 0.0,
            vehicle_speed: 0.0,
            atf_temp_c: 80.0,
            line_pressure: 0.0,
            battery_v: 13.8,
            brake: false,
            selector: Selector::Park,
        }
    }
}

impl Sensors {
    /// Measured input/output speed ratio, if both shafts are turning fast
    /// enough to be meaningful. Returns `None` near standstill where the ratio
    /// is undefined.
    #[inline]
    pub fn measured_ratio(&self) -> Option<f32> {
        const MIN_RPM: f32 = 150.0;
        if self.input_rpm >= MIN_RPM && self.output_rpm >= MIN_RPM {
            Some(self.input_rpm / self.output_rpm)
        } else {
            None
        }
    }
}
