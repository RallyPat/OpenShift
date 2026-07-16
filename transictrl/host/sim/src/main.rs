// SPDX-License-Identifier: GPL-3.0-only

//! Host-side 722.6 controller simulator.
//!
//! Runs the Rust port of the transmission controller on the developer machine
//! with fake HAL peripherals. It prints actuator commands at 100 Hz and can
//! stream telemetry over TCP for the dataviz front-end.

mod hal;

use std::time::Duration;

use transictrl_722::{
    fuel::FuelPumpController,
    sensor_input::{RawReadings, StickReading},
    telemetry::{Telemetry, TelemetryFrame},
    ui::{UiManager, UiPage, UiState},
    Config722, Controller722,
};
use transictrl_core::sensors::{Selector, Sensors};

use transictrl_hal::MonotonicClock;

use crate::hal::{MockClock, MockDisplayDrawer, MockPwm, MockSerial, SimState};

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    tracing_subscriber::fmt::init();

    let state = SimState::new();
    let fuel_pwm = MockPwm::new("fuel_pump", &state);
    let mut fuel_pump = FuelPumpController::new(fuel_pwm);

    let serial = MockSerial::new(&state);
    let mut telemetry = Telemetry::new(serial);

    let clock = MockClock::new(&state);

    let mut ui = UiManager::new(MockDisplayDrawer);

    let cfg = Config722::default();
    let mut ctrl = Controller722::new(cfg, 2);

    let mut sensors = Sensors {
        selector: Selector::Drive,
        vehicle_speed: 40.0,
        input_rpm: 1200.0,
        output_rpm: 400.0,
        engine_rpm: 1200.0,
        tps_pct: 25.0,
        atf_temp_c: 80.0,
        battery_v: 13.8,
        brake: false,
        line_pressure: 0.0,
    };

    let mut raw = RawReadings {
        n2_hz: 100.0,
        n3_hz: 0.0,
        vehicle_speed_hz: 40.0,
        engine_rpm: 1200.0,
        tps_adc: 512,
        atf_adc: 1450,
        oil_adc: 435,
        boost_adc: 0,
        exhaust_adc: 0,
        battery_adc: 4095,
        stick: StickReading::Digital {
            white: true,
            blue: true,
            green: true,
            yellow: false,
            auto_switch: false,
        },
    };

    let mut tick = 0u64;
    loop {
        tick += 1;
        state.advance_ms(10);

        // Slowly accelerate to exercise upshifts.
        sensors.vehicle_speed = 40.0 + (tick as f32 * 0.02);
        sensors.output_rpm = sensors.vehicle_speed / 3.6;
        sensors.input_rpm = sensors.output_rpm * 3.5;
        sensors.engine_rpm = sensors.input_rpm;
        raw.vehicle_speed_hz = sensors.vehicle_speed / 3.6;
        raw.engine_rpm = sensors.engine_rpm;

        fuel_pump.update(sensors.engine_rpm).unwrap();
        let cmd = ctrl.update(10, &sensors);

        let ui_state = UiState {
            page: UiPage::Main,
            wanted_gear: cmd.target_gear,
            actual_gear: cmd.current_gear,
            full_auto: cfg.flags.full_auto,
            shift_pending: cmd.shifting,
            pre_shift: false,
            post_shift: false,
            telemetry: TelemetryFrame {
                vehicle_speed_kmh: sensors.vehicle_speed,
                engine_rpm: sensors.engine_rpm,
                tps_percent: sensors.tps_pct,
                oil_temp_c: 90.0,
                atf_temp_c: sensors.atf_temp_c,
                load_percent: sensors.tps_pct,
                boost_kpa: 150.0,
                exhaust_temp_c: 0.0,
                battery_v: sensors.battery_v,
                current_gear: cmd.current_gear,
                target_gear: cmd.target_gear,
                shifting: cmd.shifting,
            },
            boost_limit_active: cfg.flags.boost_limit,
            boost_override: cfg.boost_drop,
            diff_speed_enabled: true,
            rpm_speed_enabled: true,
            slip_fault: false,
            battery_fault: false,
        };
        ui.update(&ui_state).unwrap();
        let _ = telemetry.report(&raw);

        if tick % 10 == 0 {
            tracing::info!(
                "tick={:5} gear={}/{} shifting={} sol={:?} spc={} mpc={} tcc={}% fuel={:?}",
                tick,
                cmd.current_gear,
                cmd.target_gear,
                cmd.shifting,
                cmd.solenoid,
                cmd.spc_permille,
                cmd.mpc_permille,
                cmd.tcc_duty_pct,
                state.pwm_duty.lock().unwrap().get("fuel_pump")
            );
            let _ = clock.now_ms();
        }

        tokio::time::sleep(Duration::from_millis(10)).await;
    }
}
