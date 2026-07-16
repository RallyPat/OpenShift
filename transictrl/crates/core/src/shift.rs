//! Shift state machine.
//!
//! Ported from the `doPreShift` / `doShift` / `doPostShift` sequence in the
//! original `main/core.cpp`, restructured as an explicit, time-driven state
//! machine that is a pure function of elapsed time and configuration. A shift
//! progresses through three phases:
//!
//! ```text
//!   Idle --request(target)--> PreShift --> Shift --> PostShift --> Idle
//! ```
//!
//! * **PreShift** – prepare for the swap (e.g. ask the engine to drop torque,
//!   begin pre-filling the on-coming element). Outputs still reflect the old
//!   gear's element pattern.
//! * **Shift** – command the new gear's element pattern and apply the shift
//!   pressure. This is where the clutch/solenoid swap actually happens.
//! * **PostShift** – hold and settle; the controller confirms (via
//!   [`crate::gear`]) that the new ratio has been reached before returning to
//!   `Idle`.

use crate::config::{ElementState, GearProfile, ShiftTiming, TransConfig, MAX_ELEMENTS};

/// Current phase of the shift state machine.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Phase {
    /// No shift in progress; holding a steady gear.
    Idle,
    /// Preparing to shift.
    PreShift,
    /// Actively swapping shift elements.
    Shift,
    /// Settling and confirming the new gear.
    PostShift,
}

/// The element pattern and pressure the firmware should drive this tick.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ShiftCommand {
    /// Commanded element states.
    pub elements: [ElementState; MAX_ELEMENTS],
    /// Shift-pressure command 0..=1000 (‰); higher = firmer shift.
    pub pressure_permille: u16,
    /// Whether a shift is currently in progress.
    pub shifting: bool,
}

/// Time-driven shift sequencer.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ShiftMachine {
    phase: Phase,
    /// Gear we are coming from (1-based).
    from_gear: u8,
    /// Gear we are going to (1-based).
    to_gear: u8,
    /// Milliseconds elapsed in the current phase.
    elapsed_ms: u32,
    timing: ShiftTiming,
    /// Shift-pressure command used during the `Shift` phase.
    shift_pressure: u16,
}

impl ShiftMachine {
    /// Create an idle machine sitting in `gear`.
    pub fn new(gear: u8, timing: ShiftTiming) -> Self {
        Self {
            phase: Phase::Idle,
            from_gear: gear,
            to_gear: gear,
            elapsed_ms: 0,
            timing,
            shift_pressure: 1000,
        }
    }

    /// Current phase.
    #[inline]
    pub fn phase(&self) -> Phase {
        self.phase
    }

    /// Gear considered engaged right now (the destination gear once the swap
    /// has been commanded, otherwise the gear we started from).
    #[inline]
    pub fn current_gear(&self) -> u8 {
        match self.phase {
            Phase::Idle | Phase::PreShift => self.from_gear,
            Phase::Shift | Phase::PostShift => self.to_gear,
        }
    }

    /// Gear being shifted toward.
    #[inline]
    pub fn target_gear(&self) -> u8 {
        self.to_gear
    }

    /// True when not actively shifting.
    #[inline]
    pub fn is_idle(&self) -> bool {
        self.phase == Phase::Idle
    }

    /// Request a shift to `target`. Ignored if a shift is already in progress
    /// or the target equals the current gear. `pressure_permille` sets the
    /// firmness of this shift (typically read from a pressure map).
    pub fn request(&mut self, target: u8, pressure_permille: u16) {
        if self.phase == Phase::Idle && target != self.from_gear && target != 0 {
            self.to_gear = target;
            self.shift_pressure = pressure_permille;
            self.phase = Phase::PreShift;
            self.elapsed_ms = 0;
        }
    }

    /// Force the machine to consider `gear` engaged and return to idle (used on
    /// init or after a fault recovery).
    pub fn force_gear(&mut self, gear: u8) {
        self.phase = Phase::Idle;
        self.from_gear = gear;
        self.to_gear = gear;
        self.elapsed_ms = 0;
    }

    /// Advance the machine by `dt_ms` and produce the element/pressure command
    /// for this tick. `cfg` supplies the per-gear element patterns.
    pub fn update(&mut self, dt_ms: u32, cfg: &TransConfig) -> ShiftCommand {
        self.elapsed_ms = self.elapsed_ms.saturating_add(dt_ms);

        match self.phase {
            Phase::Idle => {}
            Phase::PreShift => {
                if self.elapsed_ms >= self.timing.preshift_ms {
                    self.phase = Phase::Shift;
                    self.elapsed_ms = 0;
                }
            }
            Phase::Shift => {
                if self.elapsed_ms >= self.timing.shift_ms {
                    self.phase = Phase::PostShift;
                    self.elapsed_ms = 0;
                }
            }
            Phase::PostShift => {
                if self.elapsed_ms >= self.timing.postshift_ms {
                    // Shift complete: destination becomes the resting gear.
                    self.from_gear = self.to_gear;
                    self.phase = Phase::Idle;
                    self.elapsed_ms = 0;
                }
            }
        }

        self.command(cfg)
    }

    /// Build the output command for the current phase.
    fn command(&self, cfg: &TransConfig) -> ShiftCommand {
        // During PreShift we still present the *old* gear's pattern; from the
        // Shift phase onward we present the *new* gear's pattern.
        let gear = match self.phase {
            Phase::Idle | Phase::PreShift => self.from_gear,
            Phase::Shift | Phase::PostShift => self.to_gear,
        };
        let profile = cfg
            .gear(gear)
            .copied()
            .unwrap_or_else(GearProfile::empty);

        let shifting = self.phase != Phase::Idle;
        // Apply the calibrated shift pressure during the active swap; otherwise
        // command full line pressure for a firm hold.
        let pressure_permille = if self.phase == Phase::Shift {
            self.shift_pressure
        } else {
            1000
        };

        ShiftCommand {
            elements: profile.elements,
            pressure_permille,
            shifting,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::test_support::cfg_4l60e;
    use crate::config::ElementState::{Applied, Released};

    fn fast_timing() -> ShiftTiming {
        ShiftTiming {
            preshift_ms: 100,
            shift_ms: 300,
            postshift_ms: 100,
        }
    }

    #[test]
    fn full_shift_sequence_progresses_through_phases() {
        let cfg = cfg_4l60e();
        let mut m = ShiftMachine::new(1, fast_timing());
        assert!(m.is_idle());

        // 1st gear pattern is [Applied, Applied].
        let c = m.update(10, &cfg);
        assert_eq!(c.elements[0], Applied);
        assert_eq!(c.elements[1], Applied);

        // Request 1 -> 2 at 60% firmness.
        m.request(2, 600);
        assert_eq!(m.phase(), Phase::PreShift);

        // During PreShift we still show 1st gear's pattern.
        m.update(50, &cfg);
        assert_eq!(m.phase(), Phase::PreShift);
        assert_eq!(m.current_gear(), 1);

        // Cross into Shift.
        m.update(60, &cfg);
        assert_eq!(m.phase(), Phase::Shift);
        // 2nd gear pattern is [Released, Applied] and shift pressure applies.
        let c = m.update(10, &cfg);
        assert_eq!(c.elements[0], Released);
        assert_eq!(c.elements[1], Applied);
        assert_eq!(c.pressure_permille, 600);
        assert!(c.shifting);

        // Finish Shift -> PostShift -> Idle.
        m.update(300, &cfg);
        assert_eq!(m.phase(), Phase::PostShift);
        let c = m.update(100, &cfg);
        assert_eq!(m.phase(), Phase::Idle);
        assert_eq!(m.current_gear(), 2);
        assert!(!c.shifting);
        assert_eq!(c.pressure_permille, 1000); // firm hold once settled
    }

    #[test]
    fn request_ignored_while_shifting() {
        let cfg = cfg_4l60e();
        let mut m = ShiftMachine::new(1, fast_timing());
        m.update(10, &cfg);
        m.request(2, 600);
        assert_eq!(m.target_gear(), 2);
        // A second request mid-shift must not change the target.
        m.update(50, &cfg);
        m.request(3, 600);
        assert_eq!(m.target_gear(), 2);
    }

    #[test]
    fn request_to_same_gear_is_noop() {
        let mut m = ShiftMachine::new(2, fast_timing());
        m.request(2, 600);
        assert!(m.is_idle());
    }
}
