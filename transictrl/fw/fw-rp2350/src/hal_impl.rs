// SPDX-License-Identifier: GPL-3.0-only

//! RP2350 HAL implementation for the OpenShift transmission controller.
//!
//! Uses `embassy-rp` drivers. Pin assignments come from
//! [`crate::board::Rp2350PinConfig`].

#![cfg(target_arch = "arm")]

use core::cell::RefCell;
use core::sync::atomic::{AtomicU32, Ordering};

use critical_section::Mutex;
use embassy_rp::adc::{Adc, Blocking as AdcBlocking, Channel as AdcChannelHw};
use embassy_rp::flash::{Blocking as FlashBlocking, Flash, ERASE_SIZE};
use embassy_rp::gpio::{Input, Output, Pull};
use embassy_rp::pwm::{PwmOutput as RpPwmOutput, SetDutyCycle};
use embassy_rp::Peri;
use embassy_time::Instant;
use transictrl_hal as hal;

#[cfg(feature = "flash-2m")]
const FLASH_SIZE: usize = 2 * 1024 * 1024;
#[cfg(feature = "flash-4m")]
const FLASH_SIZE: usize = 4 * 1024 * 1024;
#[cfg(feature = "flash-8m")]
const FLASH_SIZE: usize = 8 * 1024 * 1024;

const STORE_SECTOR_OFFSET: u32 = (FLASH_SIZE - ERASE_SIZE) as u32;
const STORE_LEN: usize = 512;

/// Digital output wrapper around `embassy_rp::gpio::Output`.
pub struct RpOutput {
    pin: Output<'static>,
}

impl RpOutput {
    pub fn new(pin: Output<'static>) -> Self {
        Self { pin }
    }
}

impl hal::OutputPin for RpOutput {
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

/// PWM output wrapper around a single `embassy_rp::pwm::PwmOutput` channel.
pub struct RpPwm {
    inner: RpPwmOutput<'static>,
}

impl RpPwm {
    pub fn new(inner: RpPwmOutput<'static>) -> Self {
        Self { inner }
    }
}

impl hal::PwmOutput for RpPwm {
    type Error = embassy_rp::pwm::PwmError;

    fn set_duty_percent(&mut self, duty: u8) -> Result<(), Self::Error> {
        self.inner.set_duty_cycle_percent(duty.min(100))
    }
}

/// Shared ADC instance, populated during board init.
static ADC: Mutex<RefCell<Option<Adc<'static, AdcBlocking>>>> = Mutex::new(RefCell::new(None));

/// ADC channel wrapper around `embassy_rp::adc::Channel`.
pub struct RpAdcChannel {
    ch: AdcChannelHw<'static>,
}

impl RpAdcChannel {
    pub fn new<P: embassy_rp::adc::AdcPin>(pin: Peri<'static, P>) -> Self {
        Self {
            ch: AdcChannelHw::new_pin(pin, Pull::None),
        }
    }

    /// Install the shared `Adc` driver. Must be called once before any reads.
    pub fn init(adc: Adc<'static, AdcBlocking>) {
        critical_section::with(|cs| {
            ADC.borrow(cs).replace(Some(adc));
        });
    }
}

impl hal::AdcChannel for RpAdcChannel {
    type Error = embassy_rp::adc::Error;

    fn read_ratio(&mut self) -> Result<(f32, u16), Self::Error> {
        critical_section::with(|cs| {
            let mut adc_ref = ADC.borrow(cs).borrow_mut();
            let adc = adc_ref.as_mut().ok_or(embassy_rp::adc::Error::ConversionFailed)?;
            let raw = adc.blocking_read(&mut self.ch)?;
            // RP2350 ADC is 12-bit.
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
pub struct RpFreq {
    state: &'static FrequencyState,
}

impl RpFreq {
    pub fn new(state: &'static FrequencyState) -> Self {
        Self { state }
    }
}

impl hal::FrequencyCapture for RpFreq {
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
pub async fn freq_capture_task(mut input: Input<'static>, state: &'static FrequencyState) {
    let mut last = Instant::now();
    loop {
        input.wait_for_rising_edge().await;
        let now = Instant::now();
        let dt = now.duration_since(last).as_micros() as u32;
        if dt > 0 {
            state.store(dt, now);
        }
        last = now;
    }
}

/// Monotonic clock backed by `embassy_time::Instant::now()`.
pub struct RpClock;

impl RpClock {
    pub fn new() -> Self {
        Self
    }
}

impl hal::MonotonicClock for RpClock {
    type Error = ();

    fn now_ms(&self) -> Result<u64, Self::Error> {
        Ok(Instant::now().as_millis())
    }
}

/// Serial link over UART0 (blocking).
pub struct RpSerial {
    tx: embassy_rp::uart::UartTx<'static, embassy_rp::uart::Blocking>,
    rx: embassy_rp::uart::UartRx<'static, embassy_rp::uart::Blocking>,
}

impl RpSerial {
    pub fn new(uart: embassy_rp::uart::Uart<'static, embassy_rp::uart::Blocking>) -> Self {
        let (tx, rx) = uart.split();
        Self { tx, rx }
    }
}

impl hal::SerialLink for RpSerial {
    type Error = embassy_rp::uart::Error;

    fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        self.rx.blocking_read(buf)?;
        Ok(buf.len())
    }

    fn write(&mut self, buf: &[u8]) -> Result<(), Self::Error> {
        self.tx.blocking_write(buf)
    }
}

/// Persistent storage using on-board flash.
pub struct RpFlashStore {
    flash: Flash<'static, embassy_rp::peripherals::FLASH, FlashBlocking, FLASH_SIZE>,
}

impl RpFlashStore {
    pub fn new(flash: Peri<'static, embassy_rp::peripherals::FLASH>) -> Self {
        Self {
            flash: Flash::new_blocking(flash),
        }
    }
}

impl hal::PersistentStore for RpFlashStore {
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
            .blocking_erase(STORE_SECTOR_OFFSET, STORE_SECTOR_OFFSET + ERASE_SIZE as u32)
            .map_err(|_| ())?;
        self.flash.blocking_write(STORE_SECTOR_OFFSET, &sector).map_err(|_| ())?;
        Ok(())
    }
}

