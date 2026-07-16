// SPDX-License-Identifier: GPL-3.0-only
// Portions derived from 7226ctrl (c) Markus Kovero et al., GPL-3.0

//! Fuel pump control, ported from `main/input.cpp`.
//!
//! The original firmware reduces fuel pump duty at idle to keep fuel pressure
//! sensible when demand is low.

use transictrl_hal::PwmOutput;

/// Fuel-pump controller.
pub struct FuelPumpController<P: PwmOutput> {
    output: P,
    /// Pump duty at idle (0..100).
    pub idle_duty: u8,
    /// Pump duty above idle (0..100).
    pub run_duty: u8,
    /// Engine RPM threshold below which the pump runs at idle duty.
    pub idle_threshold_rpm: u16,
}

impl<P: PwmOutput> FuelPumpController<P> {
    pub fn new(output: P) -> Self {
        Self {
            output,
            idle_duty: 30,
            run_duty: 100,
            idle_threshold_rpm: 900,
        }
    }

    /// Update pump duty based on engine RPM.
    pub fn update(&mut self, engine_rpm: f32) -> Result<(), P::Error> {
        let duty = if engine_rpm < self.idle_threshold_rpm as f32 {
            self.idle_duty
        } else {
            self.run_duty
        };
        self.output.set_duty_percent(duty)
    }
}
