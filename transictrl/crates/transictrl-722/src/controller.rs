// SPDX-License-Identifier: GPL-3.0-only
// Portions derived from 7226ctrl (c) Markus Kovero et al., GPL-3.0

//! 722.6 controller assembly.
//!
//! [`Controller722`] owns a generic [`transictrl_core::Controller`] configured
//! for the 722.6 and layers on the 722.6-specific behavior:
//! - boost-drop pre-shift,
//! - adaptive SPC correction,
//! - pressure normalization against battery voltage,
//! - mapping logical outputs to the physical Y3/Y4/Y5 solenoids plus MPC/SPC/TCC.

use transictrl_core::{Controller as GenericController, Outputs, Sensors};
use transictrl_core::controller::Maps;
use transictrl_core::shift::Phase;

use crate::config::Config722;
use crate::elements::GEAR_PROFILES;
use crate::shift::{ShiftCommand722, ShiftPlanner722};
use crate::stick::SelectorPos;

/// High-level actuator command produced for one 722.6 control tick.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ActuatorCommand {
    /// Which shift solenoid to pulse, if any.
    pub solenoid: Option<crate::elements::Solenoid>,
    /// SPC duty, 0..=1000 ‰.
    pub spc_permille: u16,
    /// MPC duty, 0..=1000 ‰.
    pub mpc_permille: u16,
    /// TCC duty, 0..=100 %.
    pub tcc_duty_pct: u8,
    /// Commanded gear (1-based).
    pub current_gear: u8,
    /// Target gear (1-based).
    pub target_gear: u8,
    /// Whether a shift is in progress.
    pub shifting: bool,
    /// True in limp/fault mode.
    pub limp: bool,
}

/// 722.6 controller wrapper.
pub struct Controller722 {
    cfg: Config722,
    generic: GenericController,
    planner: ShiftPlanner722,
    maps: MapStore,
    adaption: Adaption,
    last_phase: Phase,
    active_shift: Option<ShiftCommand722>,
}

impl Controller722 {
    /// Build a 722.6 controller starting in `start_gear` (typically 2).
    pub fn new(cfg: Config722, start_gear: u8) -> Self {
        let generic_cfg = build_trans_config(&cfg);
        let generic = GenericController::new(generic_cfg, start_gear.max(1));
        let planner = ShiftPlanner722::new(&cfg);
        let maps = MapStore::new(&cfg);
        Self {
            cfg,
            generic,
            planner,
            maps,
            adaption: Adaption::default(),
            last_phase: Phase::Idle,
            active_shift: None,
        }
    }

    /// Request a manual upshift.
    pub fn request_upshift(&mut self) {
        self.generic.request_upshift();
    }

    /// Request a manual downshift.
    pub fn request_downshift(&mut self) {
        self.generic.request_downshift();
    }

    /// Current fault, if any.
    pub fn fault(&self) -> Option<transictrl_core::Fault> {
        self.generic.fault()
    }

    /// Current gear the controller considers engaged.
    pub fn current_gear(&self) -> u8 {
        self.generic.current_gear()
    }

    /// Replace the running configuration and adaption from a saved blob.
    pub fn load_blob(&mut self, blob: &crate::persist::CalibrationBlob) {
        let gear = self.current_gear();
        self.cfg = blob.config;
        self.adaption.table = blob.adaption;
        self.generic = GenericController::new(build_trans_config(&self.cfg), gear);
    }

    /// Extract the current configuration and adaption into a blob for saving.
    pub fn to_blob(&self) -> crate::persist::CalibrationBlob {
        crate::persist::CalibrationBlob {
            config: self.cfg,
            adaption: self.adaption.table,
            tps_min: 0,
            tps_max: 1023,
        }
    }

    /// Advance by `dt_ms` and produce actuator outputs.
    pub fn update(&mut self, dt_ms: u32, sensors: &Sensors) -> ActuatorCommand {
        let maps = self.maps.maps();
        let out = self.generic.update(dt_ms, sensors, &maps);
        let phase = self.generic.phase();
        self.apply_adaption(dt_ms, sensors, phase);
        let cmd = self.to_722_outputs(out, sensors, phase);
        self.last_phase = phase;
        cmd
    }

    fn to_722_outputs(&mut self, out: Outputs, sensors: &Sensors, phase: Phase) -> ActuatorCommand {
        let (spc, mpc, sol) = match phase {
            Phase::Idle => (
                out.epc_permille,
                self.normal_mpc_pressure(out.epc_permille, sensors),
                None,
            ),
            Phase::PreShift => {
                // Begin a new 722.6-specific shift plan when entering PreShift.
                if self.last_phase == Phase::Idle {
                    self.active_shift = self.planner.plan(
                        out.current_gear,
                        out.target_gear,
                        sensors.tps_pct as u8,
                        sensors.atf_temp_c as i16,
                    );
                }
                let shift = self.active_shift;
                (
                    self.shift_spc(&shift, sensors),
                    self.shift_mpc(&shift, sensors),
                    shift.map(|s| s.solenoid),
                )
            }
            Phase::Shift => {
                let shift = self.active_shift;
                (
                    self.shift_spc(&shift, sensors),
                    self.shift_mpc(&shift, sensors),
                    shift.map(|s| s.solenoid),
                )
            }
            Phase::PostShift => {
                // Keep solenoid active briefly at the start of PostShift to
                // ensure the mechanical change has completed, then release.
                let sol = if self.last_phase == Phase::Shift {
                    self.active_shift.map(|s| s.solenoid)
                } else {
                    None
                };
                (
                    out.epc_permille,
                    self.normal_mpc_pressure(out.epc_permille, sensors),
                    sol,
                )
            }
        };

        ActuatorCommand {
            solenoid: sol,
            spc_permille: spc,
            mpc_permille: mpc,
            tcc_duty_pct: out.tcc_duty_pct,
            current_gear: out.current_gear,
            target_gear: out.target_gear,
            shifting: out.shifting,
            limp: out.limp,
        }
    }

    /// Compute the steady-state MPC pressure outside of shifts.
    fn normal_mpc_pressure(&self, _epc: u16, sensors: &Sensors) -> u16 {
        // The original `mpcNormalMap` is indexed by load % and oil temperature.
        // We use ATF temperature as a proxy until a dedicated oil-temp sensor
        // field is threaded through the generic Sensors struct.
        use crate::maps::{lookup_2d, MPCNORMALMAP};
        let load = sensors.tps_pct as i16;
        let temp = sensors.atf_temp_c as i16;
        let pct = lookup_2d(MPCNORMALMAP, load, temp).clamp(0, 100) as u16;
        self.voltage_compensated(pct * 10, sensors.battery_v)
    }

    /// SPC pressure during a shift, including adaptive correction.
    fn shift_spc(&self, shift: &Option<ShiftCommand722>, sensors: &Sensors) -> u16 {
        let base = shift
            .map(|s| s.spc_percent as u16 * 10)
            .unwrap_or(1000);
        let corr = self.adaption.correction_for_shift(
            shift.map(|s| (s.from, s.to)).unwrap_or((0, 0)),
        );
        let adjusted = (base as i32 * corr as i32 / 100).clamp(0, 1000) as u16;
        self.voltage_compensated(adjusted, sensors.battery_v)
    }

    /// MPC pressure during a shift.
    fn shift_mpc(&self, shift: &Option<ShiftCommand722>, sensors: &Sensors) -> u16 {
        let base = shift
            .map(|s| s.mpc_percent as u16 * 10)
            .unwrap_or(1000);
        self.voltage_compensated(base, sensors.battery_v)
    }

    /// Compensate a pressure command for battery voltage.
    fn voltage_compensated(&self, permille: u16, battery_v: f32) -> u16 {
        let nominal = self.cfg.battery_limit_v10 as f32 / 10.0;
        if battery_v <= 0.0 || nominal <= 0.0 {
            return permille;
        }
        let factor = nominal / battery_v;
        ((permille as f32 * factor).clamp(0.0, 1000.0)) as u16
    }

    /// Adjust adaptive SPC correction after a shift completes.
    fn apply_adaption(&mut self, _dt_ms: u32, sensors: &Sensors, phase: Phase) {
        if self.last_phase == Phase::PostShift && phase == Phase::Idle {
            if let Some(shift) = self.active_shift {
                self.adaption.learn(shift.from, shift.to, sensors);
                self.active_shift = None;
            }
        }
    }
}

/// Convert a W202/selector position to the generic selector enum.
pub fn selector_to_generic(pos: SelectorPos, manual_mode: bool) -> transictrl_core::sensors::Selector {
    use transictrl_core::sensors::Selector as S;
    match pos {
        SelectorPos::Park => S::Park,
        SelectorPos::Reverse => S::Reverse,
        SelectorPos::Neutral => S::Neutral,
        SelectorPos::Drive => {
            if manual_mode {
                S::Manual
            } else {
                S::Drive
            }
        }
        // Positions 3/2/1 on the W202 stick are treated as manual mode.
        SelectorPos::Three | SelectorPos::Two | SelectorPos::One => S::Manual,
        SelectorPos::Unknown => S::Park,
    }
}

/// Build the generic transmission configuration from the 722.6-specific one.
fn build_trans_config(cfg: &Config722) -> transictrl_core::TransConfig {
    use transictrl_core::{GearProfile, Limits, TransConfig};

    let mut gears = [GearProfile::empty(); transictrl_core::MAX_GEARS];
    for (i, src) in GEAR_PROFILES.iter().enumerate() {
        gears[i] = *src;
    }

    TransConfig {
        gear_count: 5,
        element_count: 5,
        gears,
        final_drive: cfg.diff_ratio,
        limits: Limits {
            ratio_tolerance: 0.12,
            max_slip: cfg.max_slip,
            min_battery_v: cfg.battery_limit_v10 as f32 / 10.0,
            max_engine_rpm: cfg.max_rpm as f32,
            shift_lockout_ms: cfg.next_shift_delay_ms,
            max_atf_temp_c: 150.0,
        },
        timing: transictrl_core::ShiftTiming {
            preshift_ms: 80,
            shift_ms: 450,
            postshift_ms: 120,
        },
    }
}

/// Adaptive SPC correction, indexed by shift pair.
#[derive(Debug, Clone, Copy, PartialEq)]
struct Adaption {
    /// Percent correction around the base value (100 = no change).
    table: [[u8; 6]; 6],
}

impl Default for Adaption {
    fn default() -> Self {
        Self {
            table: [[100; 6]; 6],
        }
    }
}

impl Adaption {
    fn correction_for_shift(&self, shift: (u8, u8)) -> u8 {
        if shift.0 == 0 || shift.1 == 0 {
            return 100;
        }
        self.table
            .get(shift.0 as usize)
            .and_then(|row| row.get(shift.1 as usize))
            .copied()
            .unwrap_or(100)
    }

    fn learn(&mut self, from: u8, to: u8, sensors: &Sensors) {
        use crate::sensors::evaluate_gear;
        let actual = evaluate_gear(sensors.input_rpm, sensors.output_rpm, sensors.vehicle_speed);
        let target_ratio = crate::elements::ratio_from_gear(to);
        let tolerance = 0.12;

        if let Some(actual_gear) = actual {
            if actual_gear == to {
                // Shift completed at the right ratio; slightly soften.
                self.adjust(from, to, -1);
            } else if actual_gear != to {
                // Shift was too soft / incomplete; firm up.
                self.adjust(from, to, 2);
            }
        } else {
            // Ratio not recognised; likely still slipping — firm up.
            let actual_ratio = if sensors.output_rpm.abs() > 1.0 {
                sensors.input_rpm / sensors.output_rpm
            } else {
                0.0
            };
            if (actual_ratio - target_ratio).abs() > tolerance {
                self.adjust(from, to, 3);
            }
        }
    }

    fn adjust(&mut self, from: u8, to: u8, delta: i8) {
        if from as usize >= self.table.len() || to as usize >= self.table[from as usize].len() {
            return;
        }
        let v = self.table[from as usize][to as usize] as i16 + delta as i16;
        self.table[from as usize][to as usize] = v.clamp(50, 150) as u8;
    }
}

/// Owned map buffers for the generic controller.
struct MapStore {
    sx: [f32; 2],
    sy: [f32; 13],
    sv: [f32; 26],
    px: [f32; 2],
    py: [f32; 2],
    pv: [f32; 4],
}

impl MapStore {
    fn new(_cfg: &Config722) -> Self {
        Self {
            sx: [0.0, 100.0],
            sy: [0.0, 10.0, 20.0, 30.0, 45.0, 50.0, 60.0, 70.0, 80.0, 90.0, 100.0, 110.0, 120.0],
            sv: [
                2.0, 2.0, // 0
                2.0, 2.0, // 10
                3.0, 3.0, // 20
                3.0, 3.0, // 30
                4.0, 4.0, // 45
                4.0, 4.0, // 50
                5.0, 5.0, // 60
                5.0, 5.0, // 70
                5.0, 5.0, // 80
                5.0, 5.0, // 90
                5.0, 5.0, // 100
                5.0, 5.0, // 110
                5.0, 5.0, // 120
            ],
            px: [0.0, 100.0],
            py: [0.0, 120.0],
            pv: [500.0, 800.0, 500.0, 800.0],
        }
    }

    fn maps(&self) -> Maps<'_> {
        use transictrl_core::interp::Map2d;
        Maps {
            shift_map: Map2d::new(&self.sx, &self.sy, &self.sv).unwrap(),
            line_pressure: Map2d::new(&self.px, &self.py, &self.pv).unwrap(),
            shift_pressure: Map2d::new(&self.px, &self.py, &self.pv).unwrap(),
            tcc_lockup_speed: 80.0,
            tcc_max_throttle: 30.0,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use transictrl_core::sensors::{Selector, Sensors};

    fn cruising_sensor(speed: f32, gear: u8) -> Sensors {
        let ratio = crate::elements::ratio_from_gear(gear);
        Sensors {
            selector: Selector::Drive,
            vehicle_speed: speed,
            input_rpm: speed / 3.6 * ratio,
            output_rpm: speed / 3.6,
            engine_rpm: speed / 3.6 * ratio,
            tps_pct: 20.0,
            atf_temp_c: 80.0,
            battery_v: 13.8,
            brake: false,
            line_pressure: 0.0,
        }
    }

    #[test]
    fn controller_starts_in_gear() {
        let cfg = Config722::default();
        let ctrl = Controller722::new(cfg, 2);
        assert_eq!(ctrl.current_gear(), 2);
    }

    #[test]
    fn selector_mapping() {
        assert_eq!(selector_to_generic(SelectorPos::Park, false), Selector::Park);
        assert_eq!(selector_to_generic(SelectorPos::Drive, false), Selector::Drive);
        assert_eq!(selector_to_generic(SelectorPos::Drive, true), Selector::Manual);
    }

    #[test]
    fn shift_phases_emit_solenoid_and_pressures() {
        let cfg = Config722::default();
        let mut ctrl = Controller722::new(cfg, 1);
        let mut sensors = cruising_sensor(30.0, 1);
        sensors.selector = Selector::Manual;

        // Idle in 1st.
        let idle = ctrl.update(10, &sensors);
        assert_eq!(idle.current_gear, 1);
        assert!(idle.solenoid.is_none());

        // Request a manual 1->2 upshift.
        ctrl.request_upshift();

        // PreShift.
        let pre = ctrl.update(10, &sensors);
        assert!(pre.shifting);
        assert_eq!(pre.current_gear, 1);
        assert_eq!(pre.target_gear, 2);
        assert!(pre.solenoid.is_some());
        assert!(pre.spc_permille > 0);
        assert!(pre.mpc_permille > 0);

        // Advance through shift.
        let shift = ctrl.update(100, &sensors);
        assert!(shift.shifting);

        // PostShift.
        let post = ctrl.update(500, &sensors);
        assert!(post.shifting || post.current_gear == 2);

        // Settle.
        let settled = ctrl.update(200, &sensors);
        assert_eq!(settled.current_gear, 2);
        assert!(!settled.shifting);
        assert!(settled.solenoid.is_none());
    }
}
