//! Output command produced by the control logic.
//!
//! The controller emits an [`Outputs`] every tick describing the desired state
//! of every actuator in *logical* terms. The firmware translates these into
//! physical signals (GPIO levels, PWM duty cycles, current targets or CAN
//! frames) according to the calibration's I/O mapping.

use crate::config::{ElementState, MAX_ELEMENTS};

/// Desired actuator state for one control tick.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Outputs {
    /// Commanded state of each logical shift element (solenoid / clutch / brake).
    /// Only the first `element_count` entries are meaningful.
    pub elements: [ElementState; MAX_ELEMENTS],
    /// Target pressure-control (EPC / line-pressure) command, 0..=1000 (‰ of
    /// full scale). The firmware converts this to a solenoid current target.
    pub epc_permille: u16,
    /// Torque-converter-clutch duty cycle, 0..=100 %.
    pub tcc_duty_pct: u8,
    /// 1-based gear the controller currently considers engaged.
    pub current_gear: u8,
    /// 1-based gear the controller is shifting toward (== `current_gear` when
    /// not shifting).
    pub target_gear: u8,
    /// True while a shift is in progress.
    pub shifting: bool,
    /// True when the controller is in a protective limp state.
    pub limp: bool,
}

impl Default for Outputs {
    fn default() -> Self {
        Self {
            elements: [ElementState::DontCare; MAX_ELEMENTS],
            epc_permille: 1000, // default to full line pressure (safe / firm)
            tcc_duty_pct: 0,
            current_gear: 0,
            target_gear: 0,
            shifting: false,
            limp: false,
        }
    }
}
