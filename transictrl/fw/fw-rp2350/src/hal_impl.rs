// SPDX-License-Identifier: GPL-3.0-only

//! RP2350 HAL implementation for the OpenShift transmission controller.
//!
//! Uses `embassy-rp` drivers. Pin assignments come from
//! [`crate::board::Rp2350PinConfig`] (722.6 prototyping) or
//! [`crate::rev_l::RevLPinConfig`] (revL universal carrier).

#![cfg(target_arch = "arm")]

use core::cell::RefCell;
use core::sync::atomic::{AtomicBool, AtomicU16, AtomicU32, Ordering};

use critical_section::Mutex;
use embassy_rp::adc::{Adc, Blocking as AdcBlocking, Channel as AdcChannelHw};
use embassy_rp::flash::{Blocking as FlashBlocking, Flash, ERASE_SIZE};
use embassy_rp::gpio::{Input, Output, Pull};
use embassy_rp::pwm::{PwmOutput as RpPwmOutput, SetDutyCycle};
use embassy_rp::spi::{Blocking as SpiBlocking, Spi};
use embassy_rp::Peri;
use embassy_time::Instant;
use transictrl_hal as hal;
use transictrl_hal::current_pi::CurrentPi;

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

// =============================================================================
// Closed-loop current-controlled solenoid driver (revL Group A channels).
//
// A proportional solenoid (4L60E EPC, 8HP EDS) needs closed-loop current
// regulation rather than open-loop duty. Each Group A channel pairs a PWM
// output with an INA240 sense-amplifier channel on the external MCP3208 ADC.
// The [`current_control_task`] reads all sense channels at a fixed rate and
// runs a `CurrentPi` per channel to servo the measured current onto the target
// commanded via [`RpCurrentSolenoid::set_current_permille`].
// =============================================================================

/// INA240 gain (V/V) used on the revL current-sense front-end.
pub const INA240_GAIN: f32 = 50.0;
/// Sense resistor value (ohms) in series with each solenoid low-side FET.
pub const SENSE_R_OHM: f32 = 0.050;
/// Solenoid current (A) that maps to ADC full-scale (permille 1000).
/// Determined by the INA240 gain, sense resistor and MCP3208 Vref:
///   I_full = Vref / (gain * R) = 3.3 / (50 * 0.050) = 1.32 A.
pub const FULL_SCALE_CURRENT_A: f32 = 3.3 / (INA240_GAIN * SENSE_R_OHM);

/// Per-channel closed-loop state shared between the trait wrapper and the
/// background control task. Targets and feedback cross atomically.
pub struct CurrentSolenoidState {
    /// Commanded current target, 0..=1000 permille (0 = channel off).
    target_permille: AtomicU16,
    /// Latest measured current, 0..=1000 permille (published by the task).
    measured_permille: AtomicU16,
    /// True once `set_current_permille` has been called at least once.
    enabled: AtomicBool,
}

impl CurrentSolenoidState {
    pub const fn new() -> Self {
        Self {
            target_permille: AtomicU16::new(0),
            measured_permille: AtomicU16::new(0),
            enabled: AtomicBool::new(false),
        }
    }

    /// Command a new current target (permille). Enabling the channel.
    pub fn set_target(&self, permille: u16) {
        self.target_permille.store(permille.min(1000), Ordering::Relaxed);
        self.enabled.store(true, Ordering::Relaxed);
    }

    /// Disable the channel (target -> 0, control loop ramps duty down).
    pub fn disable(&self) {
        self.target_permille.store(0, Ordering::Relaxed);
    }

    /// Latest commanded target (permille).
    pub fn target(&self) -> u16 {
        self.target_permille.load(Ordering::Relaxed)
    }

    /// Latest measured current (permille), for telemetry/diagnostics.
    pub fn measured(&self) -> u16 {
        self.measured_permille.load(Ordering::Relaxed)
    }

    /// Publish a fresh measurement (called by the control task).
    pub(crate) fn publish(&self, permille: u16) {
        self.measured_permille.store(permille.min(1000), Ordering::Relaxed);
    }
}

/// Closed-loop current solenoid wrapper. Implements [`hal::SolenoidCurrent`].
///
/// The wrapper only records the target into shared state; the actual PWM duty
/// is driven by [`current_control_task`], which closes the loop against the
/// INA240/MCP3208 feedback. `set_duty_percent` bypasses the loop for explicit
/// open-loop control (e.g. PWM TCC/3-2 on a Group A superset channel).
pub struct RpCurrentSolenoid {
    pwm: RpPwmOutput<'static>,
    state: &'static CurrentSolenoidState,
}

impl RpCurrentSolenoid {
    pub fn new(pwm: RpPwmOutput<'static>, state: &'static CurrentSolenoidState) -> Self {
        Self { pwm, state }
    }
}

impl hal::PwmOutput for RpCurrentSolenoid {
    type Error = embassy_rp::pwm::PwmError;

    fn set_duty_percent(&mut self, duty: u8) -> Result<(), Self::Error> {
        // Open-loop bypass: take the channel out of closed-loop mode and drive
        // the PWM directly. Used for PWM (non-current) solenoids like TCC/3-2.
        self.state.disable();
        self.pwm.set_duty_cycle_percent(duty.min(100))
    }
}

impl hal::SolenoidCurrent for RpCurrentSolenoid {
    fn set_current_permille(&mut self, permille: u16) -> Result<(), Self::Error> {
        self.state.set_target(permille);
        Ok(())
    }
}

/// MCP3208 8-channel 12-bit SPI ADC. Reads the INA240 sense-amplifier outputs
/// for the 6 Group A solenoid channels plus 2 spare inputs (line pressure, EGT).
pub struct Mcp3208 {
    cs: Output<'static>,
}

impl Mcp3208 {
    /// Create the ADC driver. `cs` is the chip-select pin, driven high at rest.
    pub fn new(mut cs: Output<'static>) -> Self {
        cs.set_high();
        Self { cs }
    }

    /// Read a single channel (0..=7) and return the 12-bit code (0..=4095).
    ///
    /// MCP3208 single-ended read sequence (mode 0.0, MSB-first):
    ///   send [start, single, ch2, ch1, ch0], then read a null bit + 12 data bits.
    /// We transmit 3 bytes and interpret the response accordingly.
    pub fn read_raw<S>(
        &mut self,
        spi: &mut Spi<'static, S, SpiBlocking>,
        channel: u8,
    ) -> Result<u16, embassy_rp::spi::Error>
    where
        S: embassy_rp::spi::Instance,
    {
        debug_assert!(channel < 8);
        // Start bit (1) + single-ended (1) + channel (3 bits) packed into the
        // high bits of the first TX byte; remaining TX bytes are don't-care.
        let cmd: u8 = 0b0000_0110 | (channel >> 1);
        let cmd2: u8 = (channel & 0b1) << 7;
        let tx = [cmd, cmd2, 0x00];
        let mut rx = [0u8; 3];

        self.cs.set_low();
        // Blocking transfer: a single 3-byte transaction keeps CS asserted
        // across the whole conversion, as the MCP3208 requires.
        spi.blocking_transfer(&mut rx, &tx)?;
        self.cs.set_high();

        // Data lands in rx[1] (low 2 bits) and rx[2] (8 bits).
        let raw: u16 = (((rx[1] as u16) & 0b11) << 8) | (rx[2] as u16);
        Ok(raw)
    }

    /// Read a channel and convert to permille of full-scale current.
    ///
    /// `raw` 0..=4095 maps to 0..=`FULL_SCALE_CURRENT_A`, expressed as permille.
    pub fn read_current_permille<S>(
        &mut self,
        spi: &mut Spi<'static, S, SpiBlocking>,
        channel: u8,
    ) -> Result<u16, embassy_rp::spi::Error>
    where
        S: embassy_rp::spi::Instance,
    {
        let raw = self.read_raw(spi, channel)?;
        // 0..=4095 -> 0..=1000 permille (linear, clamped).
        let permille = (raw as u32 * 1000 + 2047) / 4095;
        Ok(permille.min(1000) as u16)
    }
}

/// Number of Group A current-controlled channels on revL.
pub const NUM_CURRENT_CHANNELS: usize = 6;

/// One current-loop entry: a shared state + the PWM output it drives + the
/// MCP3208 sense channel index that feeds its PI controller.
pub struct CurrentLoopEntry {
    pub state: &'static CurrentSolenoidState,
    pub pwm: RpPwmOutput<'static>,
    pub sense_channel: u8,
}

/// All six PI controllers, one per Group A channel. Held by the control task.
pub struct CurrentLoopBank {
    pub pi: [CurrentPi; NUM_CURRENT_CHANNELS],
    pub entries: [Option<CurrentLoopEntry>; NUM_CURRENT_CHANNELS],
}

impl CurrentLoopBank {
    /// Create an empty bank (channels added via `wire`). Tuning comes from
    /// [`CurrentPi::default_1khz`]; override per-channel if needed.
    pub fn new() -> Self {
        Self {
            pi: [
                CurrentPi::default_1khz(),
                CurrentPi::default_1khz(),
                CurrentPi::default_1khz(),
                CurrentPi::default_1khz(),
                CurrentPi::default_1khz(),
                CurrentPi::default_1khz(),
            ],
            entries: [None, None, None, None, None, None],
        }
    }

    /// Attach a channel to the bank.
    pub fn wire(&mut self, index: usize, entry: CurrentLoopEntry) {
        if index < NUM_CURRENT_CHANNELS {
            self.entries[index] = Some(entry);
        }
    }

    /// Run one control step for every wired channel.
    ///
    /// Reads each channel's sense feedback from the MCP3208, advances its PI
    /// controller, and applies the resulting duty. Channels whose target is 0
    /// are driven to 0% duty and their PI state is reset.
    pub fn step<S>(
        &mut self,
        adc: &mut Mcp3208,
        spi: &mut Spi<'static, S, SpiBlocking>,
        dt_ms: u32,
    ) where
        S: embassy_rp::spi::Instance,
    {
        for (ctrl, entry) in self.pi.iter_mut().zip(self.entries.iter_mut()) {
            let Some(e) = entry.as_mut() else { continue };
            let target = e.state.target();
            if target == 0 {
                // Channel off: force duty 0 and reset the integrator so the
                // next enable starts cleanly.
                let _ = e.pwm.set_duty_cycle_percent(0);
                e.state.publish(0);
                ctrl.reset();
                continue;
            }
            // Measure (fall back to 0 on a failed read -> loop winds down safely).
            let measured = adc.read_current_permille(spi, e.sense_channel).unwrap_or(0);
            e.state.publish(measured);
            let duty = ctrl.update(target as f32, measured as f32, dt_ms);
            let _ = e.pwm.set_duty_cycle_percent(duty as u8);
        }
    }
}

/// Background task that closes the current loop for the wired Group A channels.
///
/// Runs at a fixed 1 ms period. The caller constructs the [`CurrentLoopBank`]
/// (wiring each channel's state + PWM + sense index) and the shared SPI bus +
/// MCP3208, then spawns this task.
#[embassy_executor::task]
pub async fn current_control_task(
    mut bank: CurrentLoopBank,
    mut adc: Mcp3208,
    spi: Spi<'static, embassy_rp::peripherals::SPI0, SpiBlocking>,
) {
    let period = embassy_time::Duration::from_millis(1);
    let mut spi = spi;
    loop {
        let _ = bank.step(&mut adc, &mut spi, 1);
        embassy_time::Timer::after(period).await;
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

