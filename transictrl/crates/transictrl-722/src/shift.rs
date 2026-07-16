// SPDX-License-Identifier: GPL-3.0-only
// Portions derived from 7226ctrl (c) Markus Kovero et al., GPL-3.0

//! 722.6 shift planning, ported from `core.cpp`.
//!
//! This module bridges the generic `transictrl_core::Controller` with the
//! 722.6-specific solenoid and pressure choices made by `gearchangeUp` and
//! `gearchangeDown` in the original firmware.

use crate::elements::{ratio_from_gear, solenoid_for_shift};
use crate::maps::{
    lookup_2d, MPCMAP12, MPCMAP21, MPCMAP23, MPCMAP32, MPCMAP34, MPCMAP43, MPCMAP45, MPCMAP54,
    SPCMAP12, SPCMAP21, SPCMAP23, SPCMAP32, SPCMAP34, SPCMAP43, SPCMAP45, SPCMAP54,
};
use crate::Config722;

/// A concrete 722.6 shift command, suitable for execution by the board-specific
/// firmware.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ShiftCommand722 {
    pub from: u8,
    pub to: u8,
    pub solenoid: crate::elements::Solenoid,
    /// SPC pressure in percent (0..100).
    pub spc_percent: u8,
    /// MPC pressure in percent (0..100).
    pub mpc_percent: u8,
}

/// Planner that turns a desired gear change into 722.6 solenoid + pressure
/// choices, matching the original `gearchangeUp`/`gearchangeDown` logic.
#[derive(Debug, Clone, Copy)]
pub struct ShiftPlanner722 {
    cfg: Config722,
}

impl ShiftPlanner722 {
    pub fn new(cfg: &Config722) -> Self {
        Self { cfg: *cfg }
    }

    /// Plan an upshift from `from` to `to` (must be `to == from + 1`).
    /// Returns `None` if no solenoid change is required.
    pub fn plan_upshift(
        &self,
        from: u8,
        to: u8,
        load: u8,
        atf_temp_c: i16,
    ) -> Option<ShiftCommand722> {
        let solenoid = solenoid_for_shift(from, to)?;
        let (spc, mpc) = if self.cfg.flags.tps_sensor {
            (
                read_shift_pressure_map(from, to, load, atf_temp_c, true),
                read_shift_pressure_map(from, to, load, atf_temp_c, false),
            )
        } else {
            let fixed = fixed_timing(&self.cfg, from, to)?;
            (fixed, fixed)
        };
        Some(ShiftCommand722 {
            from,
            to,
            solenoid,
            spc_percent: spc,
            mpc_percent: mpc,
        })
    }

    /// Plan a downshift from `from` to `to` (must be `to == from - 1`).
    pub fn plan_downshift(
        &self,
        from: u8,
        to: u8,
        load: u8,
        atf_temp_c: i16,
    ) -> Option<ShiftCommand722> {
        let solenoid = solenoid_for_shift(from, to)?;
        let (spc, mpc) = if self.cfg.flags.tps_sensor {
            (
                read_shift_pressure_map(from, to, load, atf_temp_c, true),
                read_shift_pressure_map(from, to, load, atf_temp_c, false),
            )
        } else {
            let fixed = fixed_timing(&self.cfg, from, to)?;
            (fixed, fixed)
        };
        Some(ShiftCommand722 {
            from,
            to,
            solenoid,
            spc_percent: spc,
            mpc_percent: mpc,
        })
    }

    /// Look up the expected shaft ratio for the current gear.
    pub fn expected_ratio(&self, gear: u8) -> f32 {
        ratio_from_gear(gear)
    }

    /// Plan a single-gear change in either direction.
    pub fn plan(
        &self,
        from: u8,
        to: u8,
        load: u8,
        atf_temp_c: i16,
    ) -> Option<ShiftCommand722> {
        if to == from + 1 {
            self.plan_upshift(from, to, load, atf_temp_c)
        } else if to + 1 == from {
            self.plan_downshift(from, to, load, atf_temp_c)
        } else {
            None
        }
    }
}

/// Return the fixed shift timing from `Config722` for a 1-gear change.
fn fixed_timing(cfg: &Config722, from: u8, to: u8) -> Option<u8> {
    match (from, to) {
        (1, 2) => Some(cfg.shift_timings.one_to_two),
        (2, 3) => Some(cfg.shift_timings.two_to_three),
        (3, 4) => Some(cfg.shift_timings.three_to_four),
        (4, 5) => Some(cfg.shift_timings.four_to_five),
        (2, 1) => Some(cfg.shift_timings.two_to_one),
        (3, 2) => Some(cfg.shift_timings.three_to_two),
        (4, 3) => Some(cfg.shift_timings.four_to_three),
        (5, 4) => Some(cfg.shift_timings.five_to_four),
        _ => None,
    }
}

/// Select the correct SPC/MPC table for a 1-gear shift.
fn shift_table(from: u8, to: u8, is_spc: bool) -> &'static [[i16; 12]] {
    match (from, to) {
        (1, 2) => {
            if is_spc {
                &SPCMAP12
            } else {
                &MPCMAP12
            }
        }
        (2, 3) => {
            if is_spc {
                &SPCMAP23
            } else {
                &MPCMAP23
            }
        }
        (3, 4) => {
            if is_spc {
                &SPCMAP34
            } else {
                &MPCMAP34
            }
        }
        (4, 5) => {
            if is_spc {
                &SPCMAP45
            } else {
                &MPCMAP45
            }
        }
        (2, 1) => {
            if is_spc {
                &SPCMAP21
            } else {
                &MPCMAP21
            }
        }
        (3, 2) => {
            if is_spc {
                &SPCMAP32
            } else {
                &MPCMAP32
            }
        }
        (4, 3) => {
            if is_spc {
                &SPCMAP43
            } else {
                &MPCMAP43
            }
        }
        (5, 4) => {
            if is_spc {
                &SPCMAP54
            } else {
                &MPCMAP54
            }
        }
        _ => {
            if is_spc {
                &SPCMAP12
            } else {
                &MPCMAP12
            }
        }
    }
}

/// Read a 722.6 shift pressure map for the given shift.
fn read_shift_pressure_map(from: u8, to: u8, load: u8, atf_temp_c: i16, is_spc: bool) -> u8 {
    let table = shift_table(from, to, is_spc);
    lookup_2d(table, load as i16, atf_temp_c).clamp(0, 100) as u8
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn planner_upshift_yields_solenoid() {
        let cfg = Config722::default();
        let planner = ShiftPlanner722::new(&cfg);
        let cmd = planner.plan_upshift(1, 2, 50, 80).unwrap();
        assert_eq!(cmd.from, 1);
        assert_eq!(cmd.to, 2);
    }

    #[test]
    fn pressure_map_returns_sane_value() {
        let pct = read_shift_pressure_map(1, 2, 50, 80, true);
        assert!(pct > 0 && pct <= 100);
    }
}
