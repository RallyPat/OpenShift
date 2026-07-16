// SPDX-License-Identifier: GPL-3.0-only

//! Mock hardware-abstraction layer for host simulation.
//!
//! Implements the `transictrl-hal` traits with in-memory state so the
//! simulator can run the 722.6 controller without real hardware. All outputs
//! are logged; all inputs are driven by the simulation scenario.

use std::sync::{Arc, Mutex};

use transictrl_hal as hal;

/// Shared simulation state that the mock peripherals read and write.
#[derive(Debug, Default, Clone)]
#[allow(dead_code)]
pub struct SimState {
    /// Last commanded duty for each named PWM output (0..100).
    pub pwm_duty: Arc<Mutex<std::collections::HashMap<String, u8>>>,
    /// Last commanded level for each named digital output.
    pub digital_level: Arc<Mutex<std::collections::HashMap<String, bool>>>,
    /// Stored non-volatile memory.
    pub eeprom: Arc<Mutex<Vec<u8>>>,
    /// Incoming CAN frames.
    pub can_rx: Arc<Mutex<Vec<hal::CanFrame>>>,
    /// Outgoing CAN frames.
    pub can_tx: Arc<Mutex<Vec<hal::CanFrame>>>,
    /// Serial TX buffer.
    pub serial_tx: Arc<Mutex<Vec<u8>>>,
    /// Simulated elapsed milliseconds since boot.
    pub elapsed_ms: Arc<Mutex<u64>>,
}

#[allow(dead_code)]
impl SimState {
    pub fn new() -> Self {
        Self {
            eeprom: Arc::new(Mutex::new(vec![0u8; 4096])),
            ..Default::default()
        }
    }

    pub fn set_pwm(&self, name: &str, duty: u8) {
        self.pwm_duty.lock().unwrap().insert(name.to_string(), duty);
    }

    pub fn set_digital(&self, name: &str, level: bool) {
        self.digital_level.lock().unwrap().insert(name.to_string(), level);
    }

    pub fn advance_ms(&self, ms: u64) {
        let mut e = self.elapsed_ms.lock().unwrap();
        *e += ms;
    }
}

#[allow(dead_code)]
pub struct MockOutputPin {
    name: String,
    state: SimState,
}

#[allow(dead_code)]
impl MockOutputPin {
    pub fn new(name: &str, state: &SimState) -> Self {
        Self {
            name: name.to_string(),
            state: state.clone(),
        }
    }
}

impl hal::OutputPin for MockOutputPin {
    type Error = ();

    fn set_high(&mut self) -> Result<(), Self::Error> {
        self.state.set_digital(&self.name, true);
        Ok(())
    }

    fn set_low(&mut self) -> Result<(), Self::Error> {
        self.state.set_digital(&self.name, false);
        Ok(())
    }
}

#[allow(dead_code)]
pub struct MockPwm {
    name: String,
    state: SimState,
}

impl MockPwm {
    pub fn new(name: &str, state: &SimState) -> Self {
        Self {
            name: name.to_string(),
            state: state.clone(),
        }
    }
}

impl hal::PwmOutput for MockPwm {
    type Error = ();

    fn set_duty_percent(&mut self, duty: u8) -> Result<(), Self::Error> {
        self.state.set_pwm(&self.name, duty);
        Ok(())
    }
}

#[allow(dead_code)]
pub struct MockAdc {
    value: f32,
    raw: u16,
}

#[allow(dead_code)]
impl MockAdc {
    pub fn new(value: f32, raw: u16) -> Self {
        Self { value, raw }
    }
}

impl hal::AdcChannel for MockAdc {
    type Error = ();

    fn read_ratio(&mut self) -> Result<(f32, u16), Self::Error> {
        Ok((self.value, self.raw))
    }
}

#[allow(dead_code)]
pub struct MockFreq {
    hz: f32,
}

#[allow(dead_code)]
impl MockFreq {
    pub fn new(hz: f32) -> Self {
        Self { hz }
    }
}

impl hal::FrequencyCapture for MockFreq {
    type Error = ();

    fn frequency_hz(&self) -> Result<f32, Self::Error> {
        Ok(self.hz)
    }

    fn reset(&mut self) {}
}

#[allow(dead_code)]
pub struct MockStore {
    state: SimState,
}

#[allow(dead_code)]
impl MockStore {
    pub fn new(state: &SimState) -> Self {
        Self { state: state.clone() }
    }
}

impl hal::PersistentStore for MockStore {
    type Error = ();

    fn load(&mut self, buf: &mut [u8]) -> Result<(), Self::Error> {
        let eeprom = self.state.eeprom.lock().unwrap();
        let len = buf.len().min(eeprom.len());
        buf[..len].copy_from_slice(&eeprom[..len]);
        Ok(())
    }

    fn save(&mut self, buf: &[u8]) -> Result<(), Self::Error> {
        let mut eeprom = self.state.eeprom.lock().unwrap();
        let len = buf.len().min(eeprom.len());
        eeprom[..len].copy_from_slice(&buf[..len]);
        Ok(())
    }
}

#[allow(dead_code)]
pub struct MockCan {
    state: SimState,
}

#[allow(dead_code)]
impl MockCan {
    pub fn new(state: &SimState) -> Self {
        Self { state: state.clone() }
    }
}

impl hal::CanBus for MockCan {
    type Error = ();

    fn try_recv(&mut self) -> Result<Option<hal::CanFrame>, Self::Error> {
        let mut rx = self.state.can_rx.lock().unwrap();
        Ok(rx.pop())
    }

    fn send(&mut self, frame: &hal::CanFrame) -> Result<(), Self::Error> {
        self.state.can_tx.lock().unwrap().push(*frame);
        Ok(())
    }
}

#[allow(dead_code)]
pub struct MockSerial {
    state: SimState,
    rx: Vec<u8>,
}

#[allow(dead_code)]
impl MockSerial {
    pub fn new(state: &SimState) -> Self {
        Self {
            state: state.clone(),
            rx: Vec::new(),
        }
    }

    pub fn inject_rx(&mut self, bytes: &[u8]) {
        self.rx.extend_from_slice(bytes);
    }
}

impl hal::SerialLink for MockSerial {
    type Error = ();

    fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        let len = buf.len().min(self.rx.len());
        buf[..len].copy_from_slice(&self.rx[..len]);
        self.rx.drain(..len);
        Ok(len)
    }

    fn write(&mut self, buf: &[u8]) -> Result<(), Self::Error> {
        self.state.serial_tx.lock().unwrap().extend_from_slice(buf);
        Ok(())
    }
}

#[allow(dead_code)]
pub struct MockClock {
    state: SimState,
}

impl MockClock {
    pub fn new(state: &SimState) -> Self {
        Self { state: state.clone() }
    }
}

impl hal::MonotonicClock for MockClock {
    type Error = ();

    fn now_ms(&self) -> Result<u64, Self::Error> {
        Ok(*self.state.elapsed_ms.lock().unwrap())
    }
}

/// Mock display drawer that logs UI pages to stdout via tracing.
pub struct MockDisplayDrawer;

impl transictrl_722::ui::DisplayDrawer for MockDisplayDrawer {
    type Error = ();

    fn draw_main(&mut self, state: &transictrl_722::ui::UiState) -> Result<(), Self::Error> {
        tracing::debug!("UI main: gear={}/{} auto={}", state.actual_gear, state.wanted_gear, state.full_auto);
        Ok(())
    }

    fn draw_max_values(&mut self, state: &transictrl_722::ui::UiState) -> Result<(), Self::Error> {
        tracing::debug!("UI max values: speed={} boost={}", state.telemetry.vehicle_speed_kmh, state.telemetry.boost_kpa);
        Ok(())
    }

    fn draw_info(&mut self, state: &transictrl_722::ui::UiState, code: u8) -> Result<(), Self::Error> {
        tracing::debug!("UI info code={} state={:?}", code, state);
        Ok(())
    }
}
