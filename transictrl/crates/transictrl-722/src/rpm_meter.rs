// SPDX-License-Identifier: GPL-3.0-only
// Portions derived from 7226ctrl (c) Markus Kovero et al., GPL-3.0

//! W124 tachometer / RPM meter output, ported from `main/core.cpp`.
//!
//! The W124 cluster RPM input expects a pulsed signal whose frequency is
//! proportional to engine RPM. On a 4-cylinder signal this is typically
//! 2 pulses per crank revolution.

use transictrl_hal::PwmOutput;

/// RPM-meter (tachometer) driver.
pub struct RpmMeter<P: PwmOutput> {
    output: P,
    /// Pulses per engine revolution.
    pub pulses_per_rev: u8,
}

impl<P: PwmOutput> RpmMeter<P> {
    pub fn new(output: P) -> Self {
        Self {
            output,
            pulses_per_rev: 2,
        }
    }

    /// Set the RPM meter to display `rpm`.
    pub fn set_rpm(&mut self, rpm: f32) -> Result<(), P::Error> {
        if rpm <= 0.0 {
            return self.output.set_duty_percent(0);
        }
        let _hz = rpm / 60.0 * self.pulses_per_rev as f32;
        // TODO(port): drive a frequency output rather than plain PWM.
        self.output.set_duty_percent(50)
    }
}
