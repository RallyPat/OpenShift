// SPDX-License-Identifier: GPL-3.0-only

//! Raspberry Pi RP2350 firmware for OpenShift.
//!
//! Wires the transictrl-722 controller to RP2350 peripherals using
//! `embassy-rp`. Pin assignments are defined in [`board::Rp2350PinConfig`].

#![no_std]
#![no_main]

#[cfg(target_arch = "arm")]
mod board;
#[cfg(target_arch = "arm")]
mod can;
#[cfg(target_arch = "arm")]
mod hal_impl;
#[cfg(target_arch = "arm")]
mod rev_l;

#[cfg(target_arch = "arm")]
use panic_halt as _;

#[cfg(target_arch = "arm")]
#[embassy_executor::main]
async fn main(_spawner: embassy_executor::Spawner) {
    use core::fmt::Write as _;

    use embassy_rp::adc::Adc;
    use embassy_rp::gpio::{Input, Level, Output, Pull};
    use embassy_rp::pwm::{Config as PwmConfig, Pwm};
    use embassy_rp::uart::{Config as UartConfig, Uart};
    use transictrl_722::elements::Solenoid;
    use transictrl_722::sensor_input::{RawReadings, StickReading};
    use transictrl_722::{persist, Config722, Controller722};
    use transictrl_hal::{AdcChannel, FrequencyCapture, MonotonicClock, OutputPin, PwmOutput, SerialLink};

    let _p = embassy_rp::init(Default::default());
    let _pins = board::Rp2350PinConfig::prototype();
    let clock = hal_impl::RpClock::new();
    let cfg = Config722::default();
    let mut controller = Controller722::new(cfg, 2);

    // ADC inputs: TPS, ATF temp, oil pressure, boost.
    let adc = Adc::new_blocking(_p.ADC, Default::default());
    hal_impl::RpAdcChannel::init(adc);
    let mut tps_adc = hal_impl::RpAdcChannel::new(_p.PIN_26);
    let mut atf_adc = hal_impl::RpAdcChannel::new(_p.PIN_27);
    let mut oil_adc = hal_impl::RpAdcChannel::new(_p.PIN_28);
    let mut boost_adc = hal_impl::RpAdcChannel::new(_p.PIN_29);

    // Frequency-capture inputs (N2/N3 turbine speeds, vehicle speed, engine RPM).
    static N2_STATE: hal_impl::FrequencyState = hal_impl::FrequencyState::new();
    static N3_STATE: hal_impl::FrequencyState = hal_impl::FrequencyState::new();
    static VSS_STATE: hal_impl::FrequencyState = hal_impl::FrequencyState::new();
    static RPM_STATE: hal_impl::FrequencyState = hal_impl::FrequencyState::new();
    _spawner.spawn(hal_impl::freq_capture_task(Input::new(_p.PIN_18, Pull::Up), &N2_STATE).unwrap());
    _spawner.spawn(hal_impl::freq_capture_task(Input::new(_p.PIN_19, Pull::Up), &N3_STATE).unwrap());
    _spawner.spawn(hal_impl::freq_capture_task(Input::new(_p.PIN_20, Pull::Up), &VSS_STATE).unwrap());
    _spawner.spawn(hal_impl::freq_capture_task(Input::new(_p.PIN_21, Pull::Up), &RPM_STATE).unwrap());
    let n2_freq = hal_impl::RpFreq::new(&N2_STATE);
    let n3_freq = hal_impl::RpFreq::new(&N3_STATE);
    let vss_freq = hal_impl::RpFreq::new(&VSS_STATE);
    let rpm_freq = hal_impl::RpFreq::new(&RPM_STATE);

    // Load persisted calibration/adaption if available.
    let mut store = hal_impl::RpFlashStore::new(_p.FLASH);
    if let Ok(Some(blob)) = persist::load(&mut store) {
        controller.load_blob(&blob);
    }

    // Shift solenoids (digital outputs).
    let mut y3 = hal_impl::RpOutput::new(Output::new(_p.PIN_2, Level::Low));
    let mut y4 = hal_impl::RpOutput::new(Output::new(_p.PIN_3, Level::Low));
    let mut y5 = hal_impl::RpOutput::new(Output::new(_p.PIN_4, Level::Low));

    // Pressure / TCC PWM outputs.
    let pwm_mpc = Pwm::new_output_b(_p.PWM_SLICE2, _p.PIN_5, PwmConfig::default());
    let (mpc_out, _) = pwm_mpc.split();
    let mut mpc = hal_impl::RpPwm::new(mpc_out.unwrap());

    let pwm_spc = Pwm::new_output_a(_p.PWM_SLICE3, _p.PIN_6, PwmConfig::default());
    let (spc_out, _) = pwm_spc.split();
    let mut spc = hal_impl::RpPwm::new(spc_out.unwrap());

    let pwm_tcc = Pwm::new_output_b(_p.PWM_SLICE4, _p.PIN_9, PwmConfig::default());
    let (_, tcc_out) = pwm_tcc.split();
    let mut tcc = hal_impl::RpPwm::new(tcc_out.unwrap());

    // Auxiliary outputs.
    let mut _fuel_pump = hal_impl::RpOutput::new(Output::new(_p.PIN_11, Level::Low));
    let mut _boost_ctrl = hal_impl::RpOutput::new(Output::new(_p.PIN_10, Level::Low));

    // Telemetry / calibration UART.
    let uart = Uart::new_blocking(_p.UART0, _p.PIN_0, _p.PIN_1, UartConfig::default());
    let serial = hal_impl::RpSerial::new(uart);
    let mut link: transictrl_722::device_link::Link<hal_impl::RpSerial, 512> =
        transictrl_722::device_link::Link::new(serial);

    // Gearstick digital inputs.
    let stick_white = Input::new(_p.PIN_13, Pull::Up);
    let stick_blue = Input::new(_p.PIN_14, Pull::Up);
    let stick_green = Input::new(_p.PIN_15, Pull::Up);
    let stick_yellow = Input::new(_p.PIN_16, Pull::Up);
    let auto_switch = Input::new(_p.PIN_17, Pull::Up);

    let mut last_ms = clock.now_ms().unwrap_or(0);
    let mut last_saved_blob = controller.to_blob();
    let mut next_save_ms = last_ms.saturating_add(30_000);
    loop {
        embassy_time::Timer::after(embassy_time::Duration::from_millis(10)).await;

        let now_ms = clock.now_ms().unwrap_or(last_ms);
        let dt_ms = (now_ms - last_ms) as u32;
        last_ms = now_ms;

        // Persist calibration/adaption every 30 seconds when it has changed.
        if now_ms >= next_save_ms {
            let blob = controller.to_blob();
            if blob != last_saved_blob && persist::save(&mut store, &blob).is_ok() {
                last_saved_blob = blob;
            }
            next_save_ms = now_ms.saturating_add(30_000);
        }

        // Read ADC channels (ignore errors and use last-good defaults).
        let tps = tps_adc.read_ratio().unwrap_or((0.0, 0));
        let atf = atf_adc.read_ratio().unwrap_or((0.0, 0));
        let oil = oil_adc.read_ratio().unwrap_or((0.0, 0));
        let boost = boost_adc.read_ratio().unwrap_or((0.0, 0));

        let raw = RawReadings {
            n2_hz: n2_freq.frequency_hz().unwrap_or(0.0),
            n3_hz: n3_freq.frequency_hz().unwrap_or(0.0),
            vehicle_speed_hz: vss_freq.frequency_hz().unwrap_or(0.0),
            engine_rpm: rpm_freq.frequency_hz().unwrap_or(0.0),
            tps_adc: tps.1,
            atf_adc: atf.1,
            oil_adc: oil.1,
            boost_adc: boost.1,
            exhaust_adc: 0,
            battery_adc: 0,
            stick: StickReading::Digital {
                white: stick_white.is_high(),
                blue: stick_blue.is_high(),
                green: stick_green.is_high(),
                yellow: stick_yellow.is_high(),
                auto_switch: auto_switch.is_high(),
            },
        };

        let sensors = raw.to_sensors(&cfg);
        let cmd = controller.update(dt_ms, &sensors);

        // Drive outputs from the controller command.
        let _ = _fuel_pump.set_low();
        let _ = _boost_ctrl.set_low();
        let _ = mpc.set_duty_percent((cmd.mpc_permille / 10) as u8);
        let _ = spc.set_duty_percent((cmd.spc_permille / 10) as u8);
        let _ = tcc.set_duty_percent(cmd.tcc_duty_pct);

        // Shift solenoid pulsing.
        let _ = y3.set_low();
        let _ = y4.set_low();
        let _ = y5.set_low();
        match cmd.solenoid {
            Some(Solenoid::Y3) => {
                let _ = y3.set_high();
            }
            Some(Solenoid::Y4) => {
                let _ = y4.set_high();
            }
            Some(Solenoid::Y5) => {
                let _ = y5.set_high();
            }
            None => {}
        }

        // Stream a CSV telemetry line over UART0.
        let mut line: heapless::String<256> = heapless::String::new();
        let _ = writeln!(
            line,
            "{},{},{},{},{},{},{},{},{},{},{},{}",
            raw.n2_hz as u32,
            raw.n3_hz as u32,
            raw.vehicle_speed_hz as u32,
            raw.engine_rpm as u32,
            raw.tps_adc,
            raw.atf_adc,
            raw.oil_adc,
            raw.boost_adc,
            raw.battery_adc,
            cmd.current_gear,
            cmd.target_gear,
            cmd.shifting as u8
        );
        let _ = link.serial().write(line.as_bytes());

        // Handle any pending host calibration commands.
        let _ = link.poll(&mut controller, &mut store);
    }
}

#[cfg(not(target_arch = "arm"))]
fn main() {
    // Host placeholder so the crate parses on x86_64 during workspace checks.
}
