// SPDX-License-Identifier: GPL-3.0-only

//! STM32F4 firmware for OpenShift.
//!
//! Wires the transictrl-722 controller to an STM32F407 using `embassy-stm32`.

#![no_std]
#![no_main]

#[cfg(target_arch = "arm")]
mod hal_impl;

#[cfg(target_arch = "arm")]
use panic_halt as _;

#[cfg(target_arch = "arm")]
#[embassy_executor::main]
async fn main(_spawner: embassy_executor::Spawner) {
    use embassy_stm32::gpio::{Input, Level, Output, Pull, Speed};
    use embassy_stm32::time::Hertz;
    use embassy_stm32::timer::low_level::CountingMode;
    use embassy_stm32::timer::simple_pwm::{PwmPin, SimplePwm};
    use embassy_stm32::usart::{Config as UartConfig, Uart};
    use embassy_time::{Duration, Timer};
    use transictrl_722::sensor_input::{RawReadings, StickReading};
    use transictrl_722::{persist, Config722, Controller722};
    use transictrl_hal::{AdcChannel, FrequencyCapture, MonotonicClock, OutputPin, PwmOutput, SerialLink};

    let p = embassy_stm32::init(Default::default());
    let clock = hal_impl::StmClock::new();
    let cfg = Config722::default();
    let mut controller = Controller722::new(cfg, 2);

    // Load persisted calibration/adaption if available.
    let mut store = hal_impl::StmFlashStore::new(p.FLASH);
    if let Ok(Some(blob)) = persist::load(&mut store) {
        controller.load_blob(&blob);
    }

    // PWM for MPC/SPC/TCC on TIM3 channels 1/2/3 (PA6/PA7/PB0).
    let pwm = SimplePwm::new(
        p.TIM3,
        Some(PwmPin::new(p.PA6, embassy_stm32::gpio::OutputType::PushPull)),
        Some(PwmPin::new(p.PA7, embassy_stm32::gpio::OutputType::PushPull)),
        Some(PwmPin::new(p.PB0, embassy_stm32::gpio::OutputType::PushPull)),
        None,
        Hertz(1000),
        CountingMode::EdgeAlignedUp,
    );
    let pwm_channels = pwm.split();
    let mut _mpc = hal_impl::StmPwm::new(pwm_channels.ch1);
    let mut _spc = hal_impl::StmPwm::new(pwm_channels.ch2);
    let mut _tcc = hal_impl::StmPwm::new(pwm_channels.ch3);

    // Example outputs on a Nucleo F407-like pinout. Remap for the carrier board.
    let mut _fuel_pump = hal_impl::StmOutput::new(Output::new(p.PA0, Level::Low, Speed::Low));
    let mut _boost_ctrl = hal_impl::StmOutput::new(Output::new(p.PA8, Level::Low, Speed::Low));
    let mut _y3 = hal_impl::StmOutput::new(Output::new(p.PB12, Level::Low, Speed::Low));
    let mut _y4 = hal_impl::StmOutput::new(Output::new(p.PB13, Level::Low, Speed::Low));
    let mut _y5 = hal_impl::StmOutput::new(Output::new(p.PB14, Level::Low, Speed::Low));

    // ADC1 for TPS/ATF/oil/boost/battery.
    let adc = embassy_stm32::adc::Adc::new(p.ADC1);
    hal_impl::StmAdcChannel::init(adc);
    let mut tps_adc = hal_impl::StmAdcChannel::new(p.PA1);
    let mut atf_adc = hal_impl::StmAdcChannel::new(p.PA4);
    let mut oil_adc = hal_impl::StmAdcChannel::new(p.PA5);
    let mut boost_adc = hal_impl::StmAdcChannel::new(p.PC0);
    let mut battery_adc = hal_impl::StmAdcChannel::new(p.PC1);

    // Frequency capture inputs (polled in software on plain GPIO inputs).
    static N2_STATE: hal_impl::FrequencyState = hal_impl::FrequencyState::new();
    static N3_STATE: hal_impl::FrequencyState = hal_impl::FrequencyState::new();
    static SPD_STATE: hal_impl::FrequencyState = hal_impl::FrequencyState::new();
    let n2_input = Input::new(p.PB6, Pull::Up);
    let n3_input = Input::new(p.PB7, Pull::Up);
    let spd_input = Input::new(p.PC6, Pull::Up);
    let _n2 = hal_impl::StmFreq::new(&N2_STATE);
    let _n3 = hal_impl::StmFreq::new(&N3_STATE);
    let _spd = hal_impl::StmFreq::new(&SPD_STATE);
    // Spawn background edge-capture tasks.
    let _ = _spawner.spawn(hal_impl::freq_capture_task(n2_input, &N2_STATE).unwrap());
    let _ = _spawner.spawn(hal_impl::freq_capture_task(n3_input, &N3_STATE).unwrap());
    let _ = _spawner.spawn(hal_impl::freq_capture_task(spd_input, &SPD_STATE).unwrap());

    // Gearstick digital inputs.
    let _stick_white = Input::new(p.PD0, Pull::Up);
    let _stick_blue = Input::new(p.PD1, Pull::Up);
    let _stick_green = Input::new(p.PD2, Pull::Up);
    let _stick_yellow = Input::new(p.PD3, Pull::Up);
    let _auto_switch = Input::new(p.PD4, Pull::Up);

    // Serial telemetry/calibration on USART2 (PA2 TX / PA3 RX).
    let uart = Uart::new_blocking(p.USART2, p.PA3, p.PA2, UartConfig::default()).expect("usart2");
    let serial = hal_impl::StmSerial::new(uart);
    let mut link: transictrl_722::device_link::Link<hal_impl::StmSerial, 512> =
        transictrl_722::device_link::Link::new(serial);

    let mut last_ms = clock.now_ms().unwrap_or(0);
    let mut telem_timer_ms: u64 = 0;
    let mut save_timer_ms: u64 = 0;
    loop {
        Timer::after(Duration::from_millis(10)).await;

        let now_ms = clock.now_ms().unwrap_or(last_ms);
        let dt_ms = (now_ms - last_ms) as u32;
        last_ms = now_ms;

        let tps = tps_adc.read_ratio().map(|(_, r)| r).unwrap_or(0);
        let atf = atf_adc.read_ratio().map(|(_, r)| r).unwrap_or(0);
        let oil = oil_adc.read_ratio().map(|(_, r)| r).unwrap_or(0);
        let boost = boost_adc.read_ratio().map(|(_, r)| r).unwrap_or(0);
        let battery = battery_adc.read_ratio().map(|(_, r)| r).unwrap_or(0);

        let raw = RawReadings {
            n2_hz: _n2.frequency_hz().unwrap_or(0.0),
            n3_hz: _n3.frequency_hz().unwrap_or(0.0),
            vehicle_speed_hz: _spd.frequency_hz().unwrap_or(0.0),
            engine_rpm: 0.0, // no engine RPM input on this prototype pinout
            tps_adc: tps,
            atf_adc: atf,
            oil_adc: oil,
            boost_adc: boost,
            exhaust_adc: 0,
            battery_adc: battery,
            stick: StickReading::Digital {
                white: _stick_white.is_high(),
                blue: _stick_blue.is_high(),
                green: _stick_green.is_high(),
                yellow: _stick_yellow.is_high(),
                auto_switch: _auto_switch.is_high(),
            },
        };

        let sensors = raw.to_sensors(&cfg);
        let cmd = controller.update(dt_ms, &sensors);

        let _ = _fuel_pump.set_low();
        let _ = _boost_ctrl.set_low();
        let _ = _mpc.set_duty_percent((cmd.mpc_permille / 10) as u8);
        let _ = _spc.set_duty_percent((cmd.spc_permille / 10) as u8);
        let _ = _tcc.set_duty_percent(cmd.tcc_duty_pct);

        let _ = _y3.set_low();
        let _ = _y4.set_low();
        let _ = _y5.set_low();
        // TODO(port): pulse the active shift solenoid during shifts.

        // Stream CSV telemetry at 10 Hz.
        telem_timer_ms = telem_timer_ms.saturating_add(dt_ms as u64);
        if telem_timer_ms >= 100 {
            telem_timer_ms = 0;
            let mut line: heapless::String<256> = heapless::String::new();
            use core::fmt::Write;
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
        }

        // Handle any pending host calibration commands.
        let _ = link.poll(&mut controller, &mut store);

        // Persist adaption every 30 s.
        save_timer_ms = save_timer_ms.saturating_add(dt_ms as u64);
        if save_timer_ms >= 30_000 {
            save_timer_ms = 0;
            let blob = controller.to_blob();
            let _ = persist::save(&mut store, &blob);
        }
    }
}

#[cfg(not(target_arch = "arm"))]
fn main() {
    // Host placeholder so the crate parses on x86_64 during workspace checks.
}
