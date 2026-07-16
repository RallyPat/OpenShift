//! transictrl-core
//!
//! Hardware-independent, calibration-driven transmission control logic.
//!
//! This crate contains the "brain" of the controller: the shift state machine,
//! gear-ratio evaluation, map interpolation, automatic-gear decision logic and
//! safety/fault handling. It deliberately knows nothing about a specific
//! microcontroller, pin, solenoid or transmission model — everything that is
//! transmission-specific is described by a [`config::TransConfig`] that is
//! loaded from a calibration file at runtime.
//!
//! Because the logic is pure (a function of sensor inputs, configuration and
//! elapsed time), it can be exercised on the host with `cargo test` and reused
//! unchanged inside the embedded firmware control loop.
//!
//! ## Layering
//! ```text
//!   calibration file (TOML/JSON)  ->  config::TransConfig (runtime form)
//!                                         |
//!   sensors::Sensors  ->  Controller::update(dt, sensors)  ->  outputs::Outputs
//!                                         |
//!                          firmware maps Outputs -> physical solenoids / PWM / CAN
//! ```

#![cfg_attr(not(test), no_std)]
#![forbid(unsafe_code)]

pub mod config;
pub mod controller;
pub mod decision;
pub mod gear;
pub mod interp;
pub mod safety;
pub mod sensors;
pub mod shift;

pub mod math;

pub use config::{ElementState, GearProfile, Limits, ShiftTiming, TransConfig, MAX_ELEMENTS, MAX_GEARS};
pub use controller::{Controller, DriveMode};
pub use outputs::Outputs;
pub use safety::Fault;
pub use sensors::Sensors;

pub mod outputs;

#[cfg(test)]
pub(crate) mod test_support;
