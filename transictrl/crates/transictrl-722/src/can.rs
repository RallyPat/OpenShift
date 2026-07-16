// SPDX-License-Identifier: GPL-3.0-only
// Portions derived from 7226ctrl (c) Markus Kovero et al., GPL-3.0

//! CAN-bus gearstick/control handler, ported from `input.cpp`.
//!
//! Handles the W220-style CAN messages that request P/R/N/D and manual
//! upshift/downshift. Uses the board-agnostic `CanBus` trait.

use transictrl_hal::{CanBus, CanFrame};

/// CAN input state.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct CanStickState {
    /// 8 = P, 7 = R, 6 = N, 5 = D, etc.
    pub wanted_mode: u8,
    /// Upshift requested this tick.
    pub upshift: bool,
    /// Downshift requested this tick.
    pub downshift: bool,
}

/// Decodes CAN frames for gearstick commands.
pub struct CanStickDecoder;

impl CanStickDecoder {
    pub fn new() -> Self {
        Self
    }

    /// Process one incoming frame, updating `state`.
    pub fn on_frame(&self, frame: &CanFrame, state: &mut CanStickState) {
        if frame.id != 560 {
            return;
        }
        if frame.len == 0 {
            return;
        }
        let byte = frame.data[0];
        state.wanted_mode = byte;
        state.upshift = byte == 9;
        state.downshift = byte == 10;
    }

    /// Poll the CAN bus for any pending frames.
    pub fn poll<B: CanBus>(&self, bus: &mut B, state: &mut CanStickState) -> Result<(), B::Error> {
        while let Some(frame) = bus.try_recv()? {
            self.on_frame(&frame, state);
        }
        Ok(())
    }
}
