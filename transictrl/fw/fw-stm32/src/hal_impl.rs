// SPDX-License-Identifier: GPL-3.0-only

//! STM32F4 HAL implementation for the OpenShift transmission controller.
//!
//! Wires `embassy-stm32` peripherals to the board-agnostic `transictrl-hal`
//! traits. PWM is provided by general-purpose timers, ADC by ADC1, frequency
//! capture by GPIO input + EXTI edge-count tasks, flash by the on-chip FLASH,
//! and serial by USART2.

#![cfg(target_arch = "arm")]

use core::cell::RefCell;
use core::sync::atomic::{AtomicU32, Ordering};

use embassy_stm32::adc::{Adc, AdcChannel, AnyAdcChannel, SampleTime};
use embassy_stm32::flash::{Blocking as FlashBlocking, Flash, FLASH_SIZE};
use embassy_stm32::gpio::{Input, Output};
use embassy_stm32::timer::simple_pwm::SimplePwmChannel;
use embassy_stm32::Peri;
use embassy_sync::blocking_mutex::{raw::CriticalSectionRawMutex, Mutex};
use embassy_time::Instant;
use transictrl_hal as hal;

const STORE_SECTOR_OFFSET: u32 = (FLASH_SIZE - 128 * 1024) as u32;
const STORE_LEN: usize = 512;

/// Digital output wrapper around `embassy_stm32::gpio::Output`.
pub struct StmOutput {
    pin: Output<'static>,
}

impl StmOutput {
    pub fn new(pin: Output<'static>) -> Self {
        Self { pin }
    }
}

impl hal::OutputPin for StmOutput {
    type Error = core::convert::Infallible;

    fn set_high(&mut self) -> Result<(), Self::Error> {
        self.pin.set_high();
        Ok(())
    }

    fn set_low(&mut self) -> Result<(), Self::Error> {
        self.pin.set_low();
        Ok(())
    }
}

/// PWM output wrapper around `embassy_stm32::timer::simple_pwm::SimplePwmChannel`.
pub struct StmPwm {
    inner: SimplePwmChannel<'static, embassy_stm32::peripherals::TIM3>,
}

impl StmPwm {
    pub fn new(inner: SimplePwmChannel<'static, embassy_stm32::peripherals::TIM3>) -> Self {
        Self { inner }
    }
}

impl hal::PwmOutput for StmPwm {
    type Error = ();

    fn set_duty_percent(&mut self, duty: u8) -> Result<(), Self::Error> {
        self.inner.set_duty_cycle_percent(duty.min(100));
        Ok(())
    }
}

/// Shared ADC instance, populated during board init.
static ADC: Mutex<CriticalSectionRawMutex, RefCell<Option<Adc<'static, embassy_stm32::peripherals::ADC1>>>> =
    Mutex::new(RefCell::new(None));

/// ADC channel wrapper around `embassy_stm32::adc::AnyAdcChannel`.
pub struct StmAdcChannel {
    ch: AnyAdcChannel<'static, embassy_stm32::peripherals::ADC1>,
}

impl StmAdcChannel {
    pub fn new<P>(pin: Peri<'static, P>) -> Self
    where
        P: embassy_stm32::PeripheralType,
        Peri<'static, P>: embassy_stm32::adc::AdcChannel<embassy_stm32::peripherals::ADC1>,
    {
        Self {
            ch: pin.degrade_adc(),
        }
    }

    /// Install the shared `Adc` driver. Must be called once before any reads.
    pub fn init(adc: Adc<'static, embassy_stm32::peripherals::ADC1>) {
        critical_section::with(|cs| {
            ADC.borrow(cs).replace(Some(adc));
        });
    }
}

impl hal::AdcChannel for StmAdcChannel {
    type Error = ();

    fn read_ratio(&mut self) -> Result<(f32, u16), Self::Error> {
        critical_section::with(|cs| {
            let mut adc_ref = ADC.borrow(cs).borrow_mut();
            let adc = adc_ref.as_mut().ok_or(())?;
            let raw = adc.blocking_read(&mut self.ch, SampleTime::CYCLES144);
            // STM32F4 ADC is 12-bit.
            let ratio = raw as f32 / 4095.0;
            Ok((ratio, raw))
        })
    }
}

/// Shared frequency-capture state updated by a background edge-count task.
pub struct FrequencyState {
    period_us: AtomicU32,
    last_update_ticks: AtomicU32,
}

impl FrequencyState {
    pub const fn new() -> Self {
        Self {
            period_us: AtomicU32::new(0),
            last_update_ticks: AtomicU32::new(0),
        }
    }

    pub fn store(&self, period_us: u32, now: Instant) {
        self.period_us.store(period_us.max(1), Ordering::Relaxed);
        self.last_update_ticks.store(now.as_ticks() as u32, Ordering::Relaxed);
    }

    fn stale_timeout_ticks() -> u64 {
        embassy_time::Duration::from_millis(500).as_ticks()
    }
}

/// Frequency capture reading from a shared [`FrequencyState`].
pub struct StmFreq {
    state: &'static FrequencyState,
}

impl StmFreq {
    pub fn new(state: &'static FrequencyState) -> Self {
        Self { state }
    }
}

impl hal::FrequencyCapture for StmFreq {
    type Error = ();

    fn frequency_hz(&self) -> Result<f32, Self::Error> {
        let period = self.state.period_us.load(Ordering::Relaxed);
        if period == 0 {
            return Ok(0.0);
        }
        let elapsed = Instant::now()
            .as_ticks()
            .wrapping_sub(self.state.last_update_ticks.load(Ordering::Relaxed) as u64);
        if elapsed > FrequencyState::stale_timeout_ticks() {
            return Ok(0.0);
        }
        Ok(1_000_000.0 / period as f32)
    }

    fn reset(&mut self) {
        self.state.period_us.store(0, Ordering::Relaxed);
    }
}

/// Background task that measures the period between rising edges on `input`
/// and publishes the result to `state`.
#[embassy_executor::task]
pub async fn freq_capture_task(input: Input<'static>, state: &'static FrequencyState) {
    let mut last = Instant::now();
    let mut last_level = input.is_high();
    loop {
        embassy_time::Timer::after(embassy_time::Duration::from_micros(10)).await;
        let level = input.is_high();
        if level && !last_level {
            let now = Instant::now();
            let dt = now.duration_since(last).as_micros() as u32;
            if dt > 0 {
                state.store(dt, now);
            }
            last = now;
        }
        last_level = level;
    }
}

/// Monotonic clock backed by `embassy_time::Instant::now()`.
pub struct StmClock;

impl StmClock {
    pub fn new() -> Self {
        Self
    }
}

impl hal::MonotonicClock for StmClock {
    type Error = ();

    fn now_ms(&self) -> Result<u64, Self::Error> {
        Ok(Instant::now().as_millis())
    }
}

/// Serial link over USART2 (blocking).
pub struct StmSerial {
    tx: embassy_stm32::usart::UartTx<'static, embassy_stm32::mode::Blocking>,
    rx: embassy_stm32::usart::UartRx<'static, embassy_stm32::mode::Blocking>,
}

impl StmSerial {
    pub fn new(uart: embassy_stm32::usart::Uart<'static, embassy_stm32::mode::Blocking>) -> Self {
        let (tx, rx) = uart.split();
        Self { tx, rx }
    }
}

impl hal::SerialLink for StmSerial {
    type Error = embassy_stm32::usart::Error;

    fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        self.rx.blocking_read(buf)?;
        Ok(buf.len())
    }

    fn write(&mut self, buf: &[u8]) -> Result<(), Self::Error> {
        self.tx.blocking_write(buf)
    }
}

/// Persistent storage using on-chip flash.
pub struct StmFlashStore {
    flash: Flash<'static, FlashBlocking>,
}

impl StmFlashStore {
    pub fn new(flash: Peri<'static, embassy_stm32::peripherals::FLASH>) -> Self {
        Self {
            flash: Flash::new_blocking(flash),
        }
    }
}

impl hal::PersistentStore for StmFlashStore {
    type Error = ();

    fn load(&mut self, buf: &mut [u8]) -> Result<(), Self::Error> {
        let mut sector = [0u8; STORE_LEN];
        self.flash.blocking_read(STORE_SECTOR_OFFSET, &mut sector).map_err(|_| ())?;
        let len = buf.len().min(STORE_LEN);
        buf[..len].copy_from_slice(&sector[..len]);
        Ok(())
    }

    fn save(&mut self, buf: &[u8]) -> Result<(), Self::Error> {
        let mut sector = [0u8; STORE_LEN];
        let len = buf.len().min(STORE_LEN);
        sector[..len].copy_from_slice(buf);
        self.flash
            .blocking_erase(STORE_SECTOR_OFFSET, STORE_SECTOR_OFFSET + 128 * 1024)
            .map_err(|_| ())?;
        self.flash.blocking_write(STORE_SECTOR_OFFSET, &sector).map_err(|_| ())?;
        Ok(())
    }
}
