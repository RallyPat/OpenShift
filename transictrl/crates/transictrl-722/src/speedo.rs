// SPDX-License-Identifier: GPL-3.0-only
// Portions derived from 7226ctrl (c) Markus Kovero et al., GPL-3.0

//! W124 cable-speedometer output, ported from `main/core.cpp`.
//!
//! The W124 cluster expects a pulsed signal whose frequency is proportional to
//! road speed. The controller generates that pulse train via a PWM or
//! frequency output.

use transictrl_hal::PwmOutput;

/// Cable-speedometer driver.
pub struct Speedometer<P: PwmOutput> {
    output: P,
    /// Pulses per kilometre for the W124 cluster.
    pub pulses_per_km: u16,
}

impl<P: PwmOutput> Speedometer<P> {
    pub fn new(output: P) -> Self {
        Self {
            output,
            pulses_per_km: 4545, // Typical W124 value; calibration-dependent.
        }
    }

    /// Set the speedometer to display `speed_kmh`.
    pub fn set_speed(&mut self, speed_kmh: f32) -> Result<(), P::Error> {
        if speed_kmh <= 0.0 {
            return self.output.set_duty_percent(0);
        }
        // Hz = km/h * pulses/km / 3600
        let _hz = speed_kmh * self.pulses_per_km as f32 / 3600.0;
        // TODO(port): drive a frequency output rather than plain PWM.
        self.output.set_duty_percent(50)
    }
}
