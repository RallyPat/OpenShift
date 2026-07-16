// SPDX-License-Identifier: GPL-3.0-only

//! Placeholder HAL implementation for Teensy 4.x.
//!
//! Will provide concrete types for OutputPin, PwmOutput, AdcChannel,
//! FrequencyCapture, PersistentStore, CanBus, SerialLink and Display using the
//! Teensy 4 BSP.

#![cfg(target_arch = "arm")]
