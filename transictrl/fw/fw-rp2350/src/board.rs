// SPDX-License-Identifier: GPL-3.0-only

//! RP2350 board support for OpenShift.
//!
//! This module maps the logical transmission-controller signals to the
//! physical GPIO/PWM/ADC peripherals of an RP2350-based board. Pin numbers
//! here are placeholders for the specific carrier board; adjust them in the
//! board configuration TOML or source before flashing.

#![cfg(target_arch = "arm")]

/// Pin assignment for the RP2350 variant.
///
/// All fields are `Option<usize>` so unused peripherals can be left unassigned.
#[derive(Debug, Clone, Copy)]
pub struct Rp2350PinConfig {
    // Shift solenoids (digital outputs).
    pub y3: Option<u8>,
    pub y4: Option<u8>,
    pub y5: Option<u8>,

    // Pressure / TCC (PWM outputs).
    pub mpc: Option<u8>,
    pub spc: Option<u8>,
    pub tcc: Option<u8>,

    // Auxiliary outputs.
    pub speedo: Option<u8>,
    pub rpm_meter: Option<u8>,
    pub boost_ctrl: Option<u8>,
    pub fuel_pump: Option<u8>,
    pub horn: Option<u8>,

    // Gearstick digital inputs.
    pub stick_white: Option<u8>,
    pub stick_blue: Option<u8>,
    pub stick_green: Option<u8>,
    pub stick_yellow: Option<u8>,
    pub auto_switch: Option<u8>,

    // ADC inputs.
    pub tps_adc: Option<u8>,
    pub atf_adc: Option<u8>,
    pub oil_adc: Option<u8>,
    pub boost_adc: Option<u8>,
    pub battery_adc: Option<u8>,

    // Frequency capture inputs.
    pub n2_freq: Option<u8>,
    pub n3_freq: Option<u8>,
    pub vehicle_speed_freq: Option<u8>,
    pub engine_rpm_freq: Option<u8>,

    // Display SPI.
    pub display_sck: Option<u8>,
    pub display_mosi: Option<u8>,
    pub display_cs: Option<u8>,
    pub display_dc: Option<u8>,
    pub display_rst: Option<u8>,
}

impl Rp2350PinConfig {
    /// A reasonable default mapping for an RP2350 prototyping board. This is
    /// **not** the final OpenShift carrier pinout; it is intended for bench
    /// bring-up only.
    pub fn prototype() -> Self {
        Self {
            y3: Some(2),
            y4: Some(3),
            y5: Some(4),
            mpc: Some(5),
            spc: Some(6),
            tcc: Some(9),
            speedo: Some(8),
            rpm_meter: None,
            boost_ctrl: Some(10),
            fuel_pump: Some(11),
            horn: Some(12),
            stick_white: Some(13),
            stick_blue: Some(14),
            stick_green: Some(15),
            stick_yellow: Some(16),
            auto_switch: Some(17),
            tps_adc: Some(26),
            atf_adc: Some(27),
            oil_adc: Some(28),
            boost_adc: Some(29),
            battery_adc: None,
            n2_freq: Some(18),
            n3_freq: Some(19),
            vehicle_speed_freq: Some(20),
            engine_rpm_freq: Some(21),
            display_sck: Some(22),
            display_mosi: Some(23),
            display_cs: Some(24),
            display_dc: Some(25),
            display_rst: None,
        }
    }
}
