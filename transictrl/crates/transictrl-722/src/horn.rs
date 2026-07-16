// SPDX-License-Identifier: GPL-3.0-only
// Portions derived from 7226ctrl (c) Markus Kovero et al., GPL-3.0

//! Horn output helper.
//!
//! The original firmware can sound the horn via a dedicated output pin.

use transictrl_hal::OutputPin;

/// Horn controller.
pub struct Horn<P: OutputPin> {
    output: P,
}

impl<P: OutputPin> Horn<P> {
    pub fn new(output: P) -> Self {
        Self { output }
    }

    /// Sound the horn.
    pub fn on(&mut self) -> Result<(), P::Error> {
        self.output.set_high()
    }

    /// Silence the horn.
    pub fn off(&mut self) -> Result<(), P::Error> {
        self.output.set_low()
    }
}
