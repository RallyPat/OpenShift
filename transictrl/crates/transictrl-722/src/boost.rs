// SPDX-License-Identifier: GPL-3.0-only
// Portions derived from 7226ctrl (c) Markus Kovero et al., GPL-3.0

//! Boost control, ported from `main/input.cpp`.
//!
//! Implements a simple PID wastegate controller and staged boost limiting
//! (drop boost before a shift, then resume).

use transictrl_hal::PwmOutput;

/// Boost PID state.
pub struct BoostController<P: PwmOutput> {
    /// Proportional gain.
    pub kp: f32,
    /// Integral gain.
    pub ki: f32,
    /// Derivative gain.
    pub kd: f32,
    /// Target boost limit (kPa).
    pub target: u16,
    integral: f32,
    last_error: f32,
    output: P,
}

impl<P: PwmOutput> BoostController<P> {
    /// Create a boost controller with default 722.6 tuning.
    pub fn new(output: P) -> Self {
        Self {
            kp: 21.0,
            ki: 7.0,
            kd: 0.0,
            target: 200,
            integral: 0.0,
            last_error: 0.0,
            output,
        }
    }

    /// Update the controller at `dt_ms` intervals.
    ///
    /// `boost_kpa` is the measured intake pressure. `limit_kpa` is the current
    /// allowed maximum (may be reduced during a shift).
    pub fn update(&mut self, dt_ms: u32, boost_kpa: u16, limit_kpa: u16) -> Result<u8, P::Error> {
        let target = self.target.min(limit_kpa);
        let error = target as f32 - boost_kpa as f32;
        let dt = dt_ms as f32 / 1000.0;

        self.integral += error * dt;
        let derivative = if dt > 0.0 {
            (error - self.last_error) / dt
        } else {
            0.0
        };
        self.last_error = error;

        let mut duty = self.kp * error + self.ki * self.integral + self.kd * derivative;
        duty = duty.clamp(0.0, 100.0);
        let duty_u8 = duty as u8;
        self.output.set_duty_percent(duty_u8)?;
        Ok(duty_u8)
    }
}

/// Determine the allowed boost pressure for the current ATF temperature and
/// gear, using the original `boostControlPressureMap`.
pub fn boost_limit_for_gear(atf_temp_c: i16, gear: u8) -> u16 {
    use crate::maps::{lookup_2d_n, BOOSTCONTROLPRESSUREMAP};
    let col = gear.min(5).max(1) as i16;
    lookup_2d_n(BOOSTCONTROLPRESSUREMAP, col, atf_temp_c) as u16
}
