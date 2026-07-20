// SPDX-License-Identifier: GPL-3.0-only

//! revL board pin assignment for the **universal** multi-transmission carrier.
//!
//! Unlike [`crate::board::Rp2350PinConfig`] (which is 722.6-shaped), this
//! config exposes a superset of channels so a single board drives any
//! transmission the `transictrl` platform supports: 4L60E/4L80E, 8HP, 722.6,
//! 6L80, 4R70W and GM/Ford 10-speed units.
//!
//! **Bring-up priority: 4L60E/4L80E first** (only test unit on hand), then 8HP.
//! 4L60E mapping (see `pcb/revL/DESIGN.md` §10.1 and `calibrations/4l60e.toml`):
//!   - SS-A -> ch6, SS-B -> ch7  (on/off, Group B)
//!   - EPC  -> ch5               (closed-loop current, Group A)
//!   - TCC  -> ch0, 3-2 -> ch1   (open-loop PWM, Group A)
//!
//! Pin numbers below are the **target netlist** from `pcb/revL/DESIGN.md`
//! (§8). They are provisional and must be confirmed against the RP2350B
//! datasheet and any boot/strap constraints during layout. GPIO 40–47 are
//! reserved for QSPI external flash and are intentionally omitted.
//!
//! See `pcb/revL/DESIGN.md`, `pcb/revL/NETLIST.tsv` and `pcb/revL/revL.BOM`.

#![cfg(target_arch = "arm")]

/// Pin assignment for the revL universal carrier (RP2350B, 48 GPIO).
///
/// All fields are `Option<u8>` so unused peripherals can be left unassigned.
/// Channels are addressed by the calibration `io_mapping` model, so the same
/// physical board serves every `CalibrationFile`.
#[derive(Debug, Clone, Copy)]
pub struct RevLPinConfig {
    // Group A — current-controlled solenoid drivers (PWM, ch0..5).
    /// Shift-element PWM channels 0..4 (8HP clutches/brakes; shift elements).
    pub sol_pwm: [Option<u8>; 5],
    /// EPC / line-pressure PWM channel (ch5).
    pub epc_pwm: Option<u8>,

    // Group B — on/off low-side drivers (ch6..9).
    pub sol_on: [Option<u8>; 4],

    // Speed-sensor frequency-capture inputs (VR/Hall via MAX9926).
    pub iss_freq: Option<u8>,
    pub oss_freq: Option<u8>,
    pub rpm_vss_freq: Option<u8>,

    // Shared SPI0 bus: MCP2518FD CAN-FD controller + MCP3208 external ADC.
    pub spi0_sck: Option<u8>,
    pub spi0_mosi: Option<u8>,
    pub spi0_miso: Option<u8>,
    pub can_cs: Option<u8>,
    pub can_int: Option<u8>,
    pub adc_cs: Option<u8>,

    // OLED on SPI1.
    pub oled_sck: Option<u8>,
    pub oled_mosi: Option<u8>,
    pub oled_cs: Option<u8>,
    pub oled_dc: Option<u8>,
    pub oled_rst: Option<u8>,

    // I2C: PCA9555 LED expander + optional FRAM / sensors.
    pub i2c_sda: Option<u8>,
    pub i2c_scl: Option<u8>,

    // RP2350 internal ADC (slow sensor signals).
    pub tps_adc: Option<u8>,
    pub atf_adc: Option<u8>,
    pub battery_adc: Option<u8>,
    pub oil_boost_adc: Option<u8>,

    // Gearstick / general digital inputs.
    pub stick: [Option<u8>; 5],

    // Rotary encoder + button for on-device menus.
    pub enc_a: Option<u8>,
    pub enc_b: Option<u8>,
    pub enc_sw: Option<u8>,

    // Status / breakout.
    pub run_led: Option<u8>,
    pub spare: Option<u8>,
}

impl RevLPinConfig {
    /// The revL production netlist (matches `pcb/revL/DESIGN.md` §8).
    ///
    /// Provisional — confirm against the RP2350B datasheet before flashing.
    pub fn revl() -> Self {
        Self {
            // Group A: current-controlled (INA240 + sense resistor, MCP3208 feedback).
            sol_pwm: [Some(0), Some(1), Some(2), Some(3), Some(4)],
            epc_pwm: Some(5),

            // Group B: on/off low-side.
            sol_on: [Some(6), Some(7), Some(8), Some(9)],

            // Speed capture (VR via MAX9926, N2/N3 strap fix provisioned on the board).
            iss_freq: Some(10),
            oss_freq: Some(11),
            rpm_vss_freq: Some(12),

            // SPI0: CAN-FD controller + external ADC.
            spi0_sck: Some(13),
            spi0_mosi: Some(14),
            spi0_miso: Some(15),
            can_cs: Some(16),
            can_int: Some(17),
            adc_cs: Some(18),

            // SPI1: OLED.
            oled_sck: Some(19),
            oled_mosi: Some(20),
            oled_cs: Some(21),
            oled_dc: Some(22),
            oled_rst: Some(23),

            // I2C: expander + optional FRAM.
            i2c_sda: Some(24),
            i2c_scl: Some(25),

            // Internal ADC: slow sensors.
            tps_adc: Some(26),
            atf_adc: Some(27),
            battery_adc: Some(28),
            oil_boost_adc: Some(29),

            // Gearstick / digital inputs.
            stick: [Some(30), Some(31), Some(32), Some(33), Some(34)],

            // Rotary encoder.
            enc_a: Some(35),
            enc_b: Some(36),
            enc_sw: Some(37),

            // Status / spare.
            run_led: Some(38),
            spare: Some(39),
        }
    }
}
