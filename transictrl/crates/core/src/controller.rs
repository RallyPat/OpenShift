//! Top-level controller.
//!
//! Ties the sub-systems together into the single `update` call the firmware
//! runs every control tick: read [`Sensors`], run safety checks, decide the
//! desired gear (automatic or manual), drive the [`ShiftMachine`], compute
//! pressure and torque-converter commands, and emit [`Outputs`].
//!
//! The controller owns a copy of the [`TransConfig`]; per-tick lookup tables
//! (shift map, pressure maps) are passed in via [`Maps`] because they borrow
//! slices from the loaded calibration.

use crate::config::{ElementState, TransConfig};
use crate::decision::{decide_gear, DecisionInput};
use crate::interp::Map2d;
use crate::math::clampf;
use crate::outputs::Outputs;
use crate::safety::{check, Fault};
use crate::sensors::{Selector, Sensors};
use crate::shift::ShiftMachine;

/// High-level operating mode derived from the selector.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DriveMode {
    Park,
    Reverse,
    Neutral,
    /// Full automatic.
    Auto,
    /// Driver-commanded gears.
    Manual,
}

/// Per-tick lookup tables borrowed from the active calibration.
pub struct Maps<'a> {
    /// Desired gear from throttle (x) × road speed (y).
    pub shift_map: Map2d<'a>,
    /// Steady-state line pressure (‰) from load (x) × ATF temp (y).
    pub line_pressure: Map2d<'a>,
    /// Shift firmness (‰) from load (x) × ATF temp (y).
    pub shift_pressure: Map2d<'a>,
    /// Road speed (km/h) above which torque-converter lockup may engage.
    pub tcc_lockup_speed: f32,
    /// Throttle (%) below which lockup may engage (avoid lockup under load).
    pub tcc_max_throttle: f32,
}

/// The transmission controller.
pub struct Controller {
    cfg: TransConfig,
    machine: ShiftMachine,
    fault: Option<Fault>,
    /// Driver-selected gear when in manual mode.
    manual_gear: u8,
    /// Gear commanded while limping (typically a mid gear for safety).
    limp_gear: u8,
}

impl Controller {
    /// Build a controller for `cfg`, starting in `start_gear`.
    pub fn new(cfg: TransConfig, start_gear: u8) -> Self {
        let timing = cfg.timing;
        let limp_gear = limp_gear_for(&cfg);
        Self {
            cfg,
            machine: ShiftMachine::new(start_gear.max(1), timing),
            fault: None,
            manual_gear: start_gear.max(1),
            limp_gear,
        }
    }

    /// The active configuration.
    #[inline]
    pub fn config(&self) -> &TransConfig {
        &self.cfg
    }

    /// The current fault, if any.
    #[inline]
    pub fn fault(&self) -> Option<Fault> {
        self.fault
    }

    /// Current shift phase.
    #[inline]
    pub fn phase(&self) -> crate::shift::Phase {
        self.machine.phase()
    }

    /// Gear currently considered engaged.
    #[inline]
    pub fn current_gear(&self) -> u8 {
        self.machine.current_gear()
    }

    /// Request a manual upshift (only acts in manual mode).
    pub fn request_upshift(&mut self) {
        if self.manual_gear < self.cfg.gear_count {
            self.manual_gear += 1;
        }
    }

    /// Request a manual downshift (only acts in manual mode).
    pub fn request_downshift(&mut self) {
        if self.manual_gear > 1 {
            self.manual_gear -= 1;
        }
    }

    /// Map the selector to a high-level drive mode.
    fn mode(sensors: &Sensors) -> DriveMode {
        match sensors.selector {
            Selector::Park => DriveMode::Park,
            Selector::Reverse => DriveMode::Reverse,
            Selector::Neutral => DriveMode::Neutral,
            Selector::Drive => DriveMode::Auto,
            Selector::Manual => DriveMode::Manual,
        }
    }

    /// Advance the controller by `dt_ms` and produce actuator outputs.
    pub fn update(&mut self, dt_ms: u32, sensors: &Sensors, maps: &Maps) -> Outputs {
        // Load proxy for pressure maps: throttle position (0..100).
        let load = clampf(sensors.tps_pct, 0.0, 100.0);

        // --- Safety first -----------------------------------------------------
        let settled = self.machine.is_idle();
        self.fault = check(&self.cfg, sensors, self.machine.current_gear(), settled);
        if self.fault.is_some() {
            return self.limp_outputs();
        }

        let mode = Self::mode(sensors);

        // --- Desired gear -----------------------------------------------------
        let desired = match mode {
            DriveMode::Auto => decide_gear(
                &self.cfg,
                sensors,
                &DecisionInput {
                    shift_map: maps.shift_map,
                    current_gear: self.machine.current_gear(),
                },
            ),
            DriveMode::Manual => {
                // Keep the manual target in sync with the engaged gear so a
                // switch into manual does not jump gears.
                self.manual_gear = self.manual_gear.clamp(1, self.cfg.gear_count);
                self.manual_gear
            }
            // Non-driving modes: do not command forward shifts.
            DriveMode::Park | DriveMode::Neutral | DriveMode::Reverse => {
                return self.non_driving_outputs(mode);
            }
        };

        // --- Drive the shift machine -----------------------------------------
        if self.machine.is_idle() && desired != self.machine.current_gear() {
            let firmness = permille(maps.shift_pressure.get(load, sensors.atf_temp_c));
            self.machine.request(desired, firmness);
        }
        let cmd = self.machine.update(dt_ms, &self.cfg);

        // --- Pressure ---------------------------------------------------------
        // During a shift use the machine's shift pressure; otherwise the steady
        // line-pressure map for the current conditions.
        let epc_permille = if cmd.shifting {
            cmd.pressure_permille
        } else {
            permille(maps.line_pressure.get(load, sensors.atf_temp_c))
        };

        // --- Torque-converter lockup -----------------------------------------
        let tcc_duty = self.tcc_duty(sensors, maps, cmd.shifting);

        Outputs {
            elements: cmd.elements,
            epc_permille,
            tcc_duty_pct: tcc_duty,
            current_gear: self.machine.current_gear(),
            target_gear: self.machine.target_gear(),
            shifting: cmd.shifting,
            limp: false,
        }
    }

    /// Torque-converter-clutch duty for the current conditions.
    fn tcc_duty(&self, sensors: &Sensors, maps: &Maps, shifting: bool) -> u8 {
        if shifting {
            return 0; // never lock during a shift
        }
        let gear = self.machine.current_gear();
        let allowed = self
            .cfg
            .gear(gear)
            .map(|g| g.lockup_allowed)
            .unwrap_or(false);
        if allowed
            && sensors.vehicle_speed >= maps.tcc_lockup_speed
            && sensors.tps_pct <= maps.tcc_max_throttle
        {
            100
        } else {
            0
        }
    }

    /// Outputs while in Park/Neutral/Reverse (no automatic forward control).
    fn non_driving_outputs(&mut self, mode: DriveMode) -> Outputs {
        // Keep the machine consistent with a resting first gear so re-entry to
        // Drive starts cleanly.
        self.machine.force_gear(1);
        self.manual_gear = 1;
        let mut out = Outputs::default();
        out.current_gear = if mode == DriveMode::Reverse { 0 } else { 1 };
        out.target_gear = out.current_gear;
        out.epc_permille = 1000;
        out
    }

    /// Safe outputs while limping after a fault.
    fn limp_outputs(&mut self) -> Outputs {
        self.machine.force_gear(self.limp_gear);
        self.manual_gear = self.limp_gear;
        let profile = self.cfg.gear(self.limp_gear).copied();
        let elements = profile
            .map(|p| p.elements)
            .unwrap_or([ElementState::DontCare; crate::config::MAX_ELEMENTS]);
        Outputs {
            elements,
            epc_permille: 1000, // full line pressure: firm, protect the clutches
            tcc_duty_pct: 0,    // unlock converter
            current_gear: self.limp_gear,
            target_gear: self.limp_gear,
            shifting: false,
            limp: true,
        }
    }
}

/// Pick a sensible limp-home gear: a mid gear that lets the vehicle move
/// without over-revving or lugging. Uses the gear closest to a 1.5 ratio,
/// falling back to the top gear.
fn limp_gear_for(cfg: &TransConfig) -> u8 {
    if cfg.gear_count == 0 {
        return 1;
    }
    // Prefer a direct-ish gear (ratio near 1.0..1.6); else middle of the box.
    let mid = (cfg.gear_count + 1) / 2;
    mid.clamp(1, cfg.gear_count)
}

/// Clamp a raw map value to a 0..=1000 per-mille command.
#[inline]
fn permille(v: f32) -> u16 {
    clampf(v, 0.0, 1000.0) as u16
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::test_support::cfg_4l60e;

    // Minimal map set for tests. All slices are owned by the caller.
    struct MapStore {
        sx: [f32; 2],
        sy: [f32; 5],
        sv: [f32; 10],
        px: [f32; 2],
        py: [f32; 2],
        pv: [f32; 4],
    }
    impl MapStore {
        fn new() -> Self {
            Self {
                // shift map: throttle(0,100) x speed(0,30,60,90,120) -> gear
                sx: [0.0, 100.0],
                sy: [0.0, 30.0, 60.0, 90.0, 120.0],
                sv: [
                    1.0, 1.0, // 0
                    2.0, 1.0, // 30
                    3.0, 2.0, // 60
                    4.0, 3.0, // 90
                    4.0, 4.0, // 120
                ],
                // pressure maps: load(0,100) x temp(0,120) -> permille
                px: [0.0, 100.0],
                py: [0.0, 120.0],
                pv: [500.0, 800.0, 500.0, 800.0],
            }
        }
        fn maps(&self) -> Maps<'_> {
            Maps {
                shift_map: Map2d::new(&self.sx, &self.sy, &self.sv).unwrap(),
                line_pressure: Map2d::new(&self.px, &self.py, &self.pv).unwrap(),
                shift_pressure: Map2d::new(&self.px, &self.py, &self.pv).unwrap(),
                tcc_lockup_speed: 70.0,
                tcc_max_throttle: 40.0,
            }
        }
    }

    fn drive_sensors() -> Sensors {
        let mut s = Sensors::default();
        s.selector = Selector::Drive;
        s.battery_v = 13.8;
        s.atf_temp_c = 80.0;
        s.engine_rpm = 1500.0;
        s
    }

    #[test]
    fn upshifts_from_first_to_top_when_speed_rises() {
        let cfg = cfg_4l60e();
        let store = MapStore::new();
        let mut ctrl = Controller::new(cfg, 1);
        let mut s = drive_sensors();
        s.tps_pct = 0.0;
        s.vehicle_speed = 120.0;
        // Make the engaged ratio consistent so no slip fault fires.
        // Run many ticks; the controller should climb to 4th.
        for _ in 0..400 {
            // Keep shafts consistent with the engaged gear to avoid slip faults.
            let g = ctrl.current_gear();
            let ratio = cfg.ratio_for_gear(g).unwrap_or(1.0);
            s.output_rpm = 2000.0;
            s.input_rpm = 2000.0 * ratio;
            let _ = ctrl.update(20, &s, &store.maps());
        }
        assert_eq!(ctrl.current_gear(), 4);
        assert!(ctrl.fault().is_none());
    }

    #[test]
    fn fault_forces_limp_mode() {
        let cfg = cfg_4l60e();
        let store = MapStore::new();
        let mut ctrl = Controller::new(cfg, 3);
        let mut s = drive_sensors();
        s.battery_v = 7.0; // low voltage fault
        let out = ctrl.update(20, &s, &store.maps());
        assert!(out.limp);
        assert_eq!(out.epc_permille, 1000);
        assert_eq!(out.tcc_duty_pct, 0);
        assert_eq!(ctrl.fault(), Some(Fault::LowVoltage));
    }

    #[test]
    fn manual_mode_obeys_driver() {
        let cfg = cfg_4l60e();
        let store = MapStore::new();
        let mut ctrl = Controller::new(cfg, 1);
        let mut s = drive_sensors();
        s.selector = Selector::Manual;
        s.vehicle_speed = 40.0;
        // Driver asks for 2nd, then 3rd.
        ctrl.request_upshift();
        // Run enough ticks to complete the shift.
        for _ in 0..60 {
            let g = ctrl.current_gear();
            let ratio = cfg.ratio_for_gear(g).unwrap_or(1.0);
            s.output_rpm = 1500.0;
            s.input_rpm = 1500.0 * ratio;
            ctrl.update(20, &s, &store.maps());
        }
        assert_eq!(ctrl.current_gear(), 2);
    }

    #[test]
    fn tcc_locks_only_when_cruising_in_lockup_gear() {
        let cfg = cfg_4l60e();
        let store = MapStore::new();
        let mut ctrl = Controller::new(cfg, 4);
        let mut s = drive_sensors();
        s.selector = Selector::Manual;
        ctrl.manual_gear = 4;
        s.vehicle_speed = 110.0;
        s.tps_pct = 10.0; // light load
        s.output_rpm = 2500.0;
        s.input_rpm = 2500.0 * cfg.ratio_for_gear(4).unwrap();
        // settle
        let mut out = Outputs::default();
        for _ in 0..10 {
            out = ctrl.update(20, &s, &store.maps());
        }
        assert_eq!(out.tcc_duty_pct, 100);

        // Heavy throttle should unlock.
        s.tps_pct = 90.0;
        out = ctrl.update(20, &s, &store.maps());
        assert_eq!(out.tcc_duty_pct, 0);
    }
}
