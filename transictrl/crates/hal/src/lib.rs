// SPDX-License-Identifier: GPL-3.0-only
// Portions derived from 7226ctrl (c) Markus Kovero et al., GPL-3.0

//! `transictrl-hal` — board-agnostic hardware abstraction layer for the
//! OpenShift transmission controller.
//!
//! This crate defines *domain-level* traits that describe the sensors and
//! actuators a transmission controller needs. Concrete board support packages
//! (`fw-teensy4`, `fw-rp2350`, `fw-stm32`, etc.) implement these traits using
//! the target HAL of choice (embassy, RTIC, or bare-metal).
//!
//! Keeping the traits in their own crate keeps the GPL-3.0-derived control
//! logic portable and independent of any one microcontroller vendor.

#![no_std]
#![forbid(unsafe_code)]

use core::fmt;

pub mod framing;

/// A single digital output line (e.g. shift solenoid enable).
pub trait OutputPin {
    type Error: fmt::Debug;

    /// Set the pin high or low.
    fn set_high(&mut self) -> Result<(), Self::Error>;
    fn set_low(&mut self) -> Result<(), Self::Error>;
}

/// A PWM output with 0..=100 % duty resolution.
pub trait PwmOutput {
    type Error: fmt::Debug;

    /// Set duty cycle as a percentage in [0, 100].
    fn set_duty_percent(&mut self, duty: u8) -> Result<(), Self::Error>;
}

/// Variable solenoid current target, expressed as 0..=1000 ‰ of full scale.
/// Implementers translate this to PWM duty or closed-loop current as needed.
pub trait SolenoidCurrent: PwmOutput {
    /// Drive the solenoid at `permille / 1000` of full current.
    fn set_current_permille(&mut self, permille: u16) -> Result<(), Self::Error> {
        let duty = ((permille.min(1000) * 100) / 1000) as u8;
        self.set_duty_percent(duty)
    }
}

/// Analog-to-digital input returning a normalized [0.0, 1.0] ratio of reference
/// voltage, plus raw counts for diagnostics.
pub trait AdcChannel {
    type Error: fmt::Debug;

    /// Read the channel and return (normalized ratio, raw counts).
    fn read_ratio(&mut self) -> Result<(f32, u16), Self::Error>;
}

/// Frequency capture for N2/N3 turbine speed sensors and vehicle speed / RPM.
/// Implementers use input capture, pulse counting, or polled edge timing.
pub trait FrequencyCapture {
    type Error: fmt::Debug;

    /// Frequency in Hz, if available.
    fn frequency_hz(&self) -> Result<f32, Self::Error>;

    /// Reset the capture (useful after a gear change).
    fn reset(&mut self);
}

/// Non-volatile storage for calibration and learned adaption data.
pub trait PersistentStore {
    type Error: fmt::Debug;

    /// Load a raw byte blob from storage.
    fn load(&mut self, buf: &mut [u8]) -> Result<(), Self::Error>;

    /// Save a raw byte blob to storage.
    fn save(&mut self, buf: &[u8]) -> Result<(), Self::Error>;
}

/// Controller Area Network interface, if present.
pub trait CanBus {
    type Error: fmt::Debug;

    /// Attempt to receive a frame without blocking.
    fn try_recv(&mut self) -> Result<Option<CanFrame>, Self::Error>;

    /// Transmit a frame.
    fn send(&mut self, frame: &CanFrame) -> Result<(), Self::Error>;
}

/// A CAN frame.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CanFrame {
    pub id: u32,
    pub len: u8,
    pub data: [u8; 8],
}

/// Serial link to the host (USB-CDC, UART, etc.).
pub trait SerialLink {
    type Error: fmt::Debug;

    /// Read bytes into `buf`, returning how many were read.
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error>;

    /// Write all bytes in `buf`.
    fn write(&mut self, buf: &[u8]) -> Result<(), Self::Error>;
}

/// System clock, used when the firmware needs wall/relative time but does not
/// have an async executor available.
pub trait MonotonicClock {
    type Error: fmt::Debug;

    /// Return elapsed milliseconds since boot.
    fn now_ms(&self) -> Result<u64, Self::Error>;
}

/// Small display abstraction for the on-device OLED.
pub trait Display {
    type Error: fmt::Debug;

    /// Clear the display.
    fn clear(&mut self) -> Result<(), Self::Error>;

    /// Render a text line at the given row.
    fn draw_text(&mut self, row: u8, text: &str) -> Result<(), Self::Error>;

    /// Flush pending draw commands.
    fn flush(&mut self) -> Result<(), Self::Error>;
}
