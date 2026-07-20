// SPDX-License-Identifier: GPL-3.0-only

//! Anti-windup PI controller for closed-loop solenoid current regulation.
//!
//! The revL carrier reads each proportional solenoid's current through an
//! INA240 sense amplifier into an external ADC (MCP3208). A background task
//! calls [`CurrentPi::update`] at a fixed rate to servo the PWM duty so the
//! measured current tracks the commanded set-point.
//!
//! This module is pure arithmetic (no I/O, no allocator) so it is fully
//! unit-testable on the host. The firmware task in `fw-rp2350` owns the
//! hardware and feeds real samples in.
//!
//! ## Units
//!
//! Both set-point and measurement are expressed as **per-mille of full-scale
//! current** (0..=1000), matching the `SolenoidCurrent::set_current_permille`
//! trait. The control output is a duty percentage in 0..=100.

/// Anti-windup proportional-integral current controller.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CurrentPi {
    /// Proportional gain (duty-% per permille of error).
    pub kp: f32,
    /// Integral gain (duty-% per permille-second).
    pub ki: f32,
    /// Integral term bounds, in duty-%. Clamps wind-up.
    pub i_min: f32,
    pub i_max: f32,
    /// Output duty bounds.
    pub out_min: f32,
    pub out_max: f32,
    /// Accumulated integral term (duty-%).
    integral: f32,
}

impl CurrentPi {
    /// Create a controller with the given gains and default bounds.
    ///
    /// Integral is clamped to `[i_min, i_max]` (duty-%) and the output duty to
    /// `[out_min, out_max]`. The anti-windup clamp keeps the integrator from
    /// running away when the output saturates.
    pub const fn new(kp: f32, ki: f32, i_min: f32, i_max: f32, out_min: f32, out_max: f32) -> Self {
        Self {
            kp,
            ki,
            i_min,
            i_max,
            out_min,
            out_max,
            integral: 0.0,
        }
    }

    /// A conservative default for a low-inductance solenoid driven at ~1 kHz.
    /// Tuning happens on the bench against the actual load.
    pub const fn default_1khz() -> Self {
        Self::new(0.20, 1.5, 0.0, 100.0, 0.0, 100.0)
    }

    /// Current integral term (duty-%), for diagnostics/telemetry.
    pub fn integral(&self) -> f32 {
        self.integral
    }

    /// Reset the controller state (e.g. when enabling a channel).
    pub fn reset(&mut self) {
        self.integral = 0.0;
    }

    /// Advance one control step.
    ///
    /// - `setpoint_permille`: commanded current, 0..=1000.
    /// - `measured_permille`: latest current-sense reading, 0..=1000.
    /// - `dt_ms`: elapsed time since the last update (milliseconds).
    ///
    /// Returns the new duty percentage in `[out_min, out_max]`.
    pub fn update(&mut self, setpoint_permille: f32, measured_permille: f32, dt_ms: u32) -> f32 {
        let err = setpoint_permille - measured_permille;
        let dt_s = (dt_ms.max(1) as f32) / 1000.0;

        // Tentative integral update.
        let i_new = self.integral + err * self.ki * dt_s;
        let i_clamped = clampf(i_new, self.i_min, self.i_max);

        // Compute the raw output with the clamped integral.
        let out = self.kp * err + i_clamped;
        let out_clamped = clampf(out, self.out_min, self.out_max);

        // Conditional anti-windup: only commit the integral if the output is
        // not saturating, or if committing would move it back inside the
        // saturation (i.e. the error opposes the saturation direction).
        let saturated = out < self.out_min || out > self.out_max;
        let unwinding = saturated && (i_clamped - self.integral) * err <= 0.0;
        if !saturated || unwinding {
            self.integral = i_clamped;
        }

        out_clamped
    }
}

#[inline]
fn clampf(x: f32, lo: f32, hi: f32) -> f32 {
    if x < lo {
        lo
    } else if x > hi {
        hi
    } else {
        x
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Drive a first-order plant model (tau = 5 ms, gain = 0.1 duty-%/permille)
    /// and confirm the controller converges to the set-point.
    #[test]
    fn closed_loop_converges_to_setpoint() {
        let mut pi = CurrentPi::default_1khz();
        let setpoint = 600.0_f32; // 60% current target
        let mut measured = 0.0_f32;
        // Plant: measured += (duty * 10 - measured) * (dt/tau)
        let tau_ms = 5.0_f32;
        let plant_gain = 10.0_f32; // permille per duty-%

        for _ in 0..2000 {
            let duty = pi.update(setpoint, measured, 1);
            let alpha = 1.0_f32 / tau_ms; // dt = 1ms
            measured += (duty * plant_gain - measured) * alpha;
        }
        // Steady-state error well under 2% (i.e. within ±20 permille).
        assert!(
            (measured - setpoint).abs() < 20.0,
            "did not converge: measured = {measured}, setpoint = {setpoint}"
        );
    }

    #[test]
    fn zero_setpoint_drives_to_zero() {
        let mut pi = CurrentPi::default_1khz();
        let mut measured = 500.0;
        for _ in 0..1000 {
            let duty = pi.update(0.0, measured, 1);
            // plant with no drive decays toward 0
            measured += (duty * 10.0 - measured) / 5.0;
        }
        assert!(measured.abs() < 20.0, "measured = {measured}");
        assert_eq!(pi.update(0.0, 0.0, 1), 0.0);
    }

    #[test]
    fn output_clamps_to_bounds() {
        let mut pi = CurrentPi::new(10.0, 0.0, 0.0, 0.0, 0.0, 60.0);
        // Huge error should saturate at the configured out_max (60%), and the
        // integrator must not wind up (it's disabled here by i bounds = 0).
        let duty = pi.update(1000.0, 0.0, 1);
        assert_eq!(duty, 60.0);
    }

    #[test]
    fn anti_windup_freezes_integral_during_saturation() {
        // Conditional-integration anti-windup: while the proportional term alone
        // saturates the output, the integrator must freeze (not accumulate). Once
        // the output leaves saturation, the integral resumes accumulating.
        let mut pi = CurrentPi::new(0.2, 5.0, 0.0, 100.0, 0.0, 100.0);

        // Large error -> P saturates (0.2 * 1000 = 200% > 100%). Integral frozen.
        let _ = pi.update(1000.0, 0.0, 1);
        let i_after_sat = pi.integral();
        assert_eq!(i_after_sat, 0.0, "integral must be frozen while P saturates");

        // Keep saturating; integral stays frozen.
        for _ in 0..50 {
            let _ = pi.update(1000.0, 0.0, 1);
        }
        assert_eq!(pi.integral(), 0.0, "integral must remain frozen under sustained saturation");

        // Now drop the error so the output is no longer saturated (0.2 * 100 = 20%).
        // The integral should start accumulating again.
        for _ in 0..10 {
            let _ = pi.update(100.0, 0.0, 1);
        }
        assert!(
            pi.integral() > 0.0,
            "integral must resume after leaving saturation"
        );
    }

    #[test]
    fn reset_clears_state() {
        let mut pi = CurrentPi::default_1khz();
        // Use a modest error so the proportional term does not saturate and the
        // integral is allowed to accumulate (conditional anti-windup).
        let _ = pi.update(100.0, 0.0, 100);
        assert!(pi.integral() > 0.0);
        pi.reset();
        assert_eq!(pi.integral(), 0.0);
    }
}
