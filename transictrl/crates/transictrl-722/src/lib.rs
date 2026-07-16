// SPDX-License-Identifier: GPL-3.0-only
// Portions derived from 7226ctrl (c) Markus Kovero et al., GPL-3.0

//! `transictrl-722` — 722.6 transmission-specific port.
//!
//! This crate contains logic derived from Markus Kovero's `7226ctrl` project.
//! It is intentionally separate from the generic `transictrl-core` crate so
//! that the core can remain a clean-room, calibration-driven transmission
//! controller library while this crate preserves the upstream GPL-3.0-derived
//! implementation.
//!
//! The crate provides:
//! - Mercedes/Chrysler 722.6 gear ratios, element mapping, and solenoid
//!   selection for upshifts/downshifts.
//! - Sensor scaling (ATF temperature, oil temperature, TPS, battery, etc.).
//! - Shift timing and pressure tables derived from the original `maps.h`.
//! - W202 gearstick decoding (digital and resistive modes).
//! - The top-level [`Controller722`] that wires `transictrl-core` pieces
//!   together with the 722.6 specifics.

#![cfg_attr(not(test), no_std)]
#![forbid(unsafe_code)]

pub mod boost;
pub mod can;
pub mod config;
pub mod controller;
pub mod device_link;

// Re-export the no_std framing utilities so host tools can share the same
// protocol definitions without adding a direct HAL dependency.
pub use transictrl_hal::framing;
pub mod elements;
pub mod fuel;
pub mod horn;
pub mod maps;
pub mod persist;
pub mod rpm_meter;
pub mod sensor_input;
pub mod sensors;
pub mod serial_config;
pub mod shift;
pub mod speedo;
pub mod stick;
pub mod telemetry;
pub mod ui;

pub use config::Config722;
pub use controller::{selector_to_generic, ActuatorCommand, Controller722};
pub use shift::{ShiftCommand722, ShiftPlanner722};
