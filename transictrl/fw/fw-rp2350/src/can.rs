// SPDX-License-Identifier: GPL-3.0-only

//! MCP2518FD CAN-FD controller driver and `CanBus` trait implementation.
//!
//! The revL carrier uses an MCP2518FD CAN-FD controller over SPI (shared SPI0
//! bus with the MCP3208 ADC) plus an MCP2562FD transceiver. The controller has
//! no native CAN peripheral, so all CAN traffic flows through this driver.
//!
//! ## Architecture
//!
//! Because the SPI bus is shared and `CanBus::try_recv`/`send` are called from
//! the controller loop (not the SPI-owning task), [`RpCan`] decouples the trait
//! surface from the hardware:
//!
//! - **Receive:** a background task owns the SPI bus, polls the MCP2518FD RX
//!   FIFO, and pushes accepted frames into a [`CanRxBuf`] (host-tested, in
//!   `transictrl-hal`). `try_recv` simply pops from that buffer.
//! - **Transmit:** `send` pushes into a small TX ring; the background task
//!   drains it into the MCP2518FD TX FIFO.
//!
//! This mirrors how [`crate::hal_impl::RpFreq`] decouples frequency capture.
//!
//! ## Bring-up status
//!
//! The SPI register primitives below capture the MCP2518FD command set and the
//! key register addresses from the public datasheet. The exact configuration
//! sequence (CiCON, CiNBTCFG, CiFIFOBA, FIFO filter setup) and TX/RAM base
//! addresses must be validated against a live controller on the bench — they
//! are marked `TODO(bring-up)` where the datasheet leaves runtime choices open.

#![cfg(target_arch = "arm")]

use core::sync::atomic::{AtomicU16, Ordering};

use embassy_rp::gpio::Output;
use embassy_rp::spi::{Blocking as SpiBlocking, Spi};
use embassy_rp::Peri;
use transictrl_hal::can::{CanFilter, CanRxBuf};
use transictrl_hal::{CanBus, CanFrame};

// ---------------------------------------------------------------------------
// MCP2518FD SPI command set (from the public datasheet, table 5-1).
// ---------------------------------------------------------------------------

/// Reset the device and internal registers to the default state.
pub const CMD_RESET: u8 = 0x00;
/// Read from a register address (24-bit address, see datasheet §5).
pub const CMD_READ: u8 = 0x03;
/// Write to a register address.
pub const CMD_WRITE: u8 = 0x02;
/// Read from SRAM (TX/RX FIFO and RAM).
pub const CMD_READ_SRAM: u8 = 0xEB;
/// Write to SRAM.
pub const CMD_WRITE_SRAM: u8 = 0x20;

// ---------------------------------------------------------------------------
// Key register addresses (MCP251x FD "Ci" registers, datasheet §5).
// These are byte addresses in the controller's register space.
// ---------------------------------------------------------------------------

/// CAN Control register.
pub const REG_CICON: u32 = 0x000;
/// CAN Status register.
pub const REG_CISTAT: u32 = 0x004;
/// Nominal Bit Time Configuration.
pub const REG_CINBTCFG: u32 = 0x00C;
/// Data Bit Time Configuration (CAN-FD).
pub const REG_CIDBTCFG: u32 = 0x010;
/// Transmit Event Control.
pub const REG_CITEC: u32 = 0x014;
/// Error counter / diagnostic.
pub const REG_CIDIAG0: u32 = 0x01C;
/// FIFO Control base (FIFO 0 control).
pub const REG_CIFIFOCON: u32 = 0x05C;
/// FIFO count of FIFO0 (TX) status bits live here.
pub const REG_CIFIFOSTA: u32 = 0x07C;

/// Chip-select-gated SPI register read (one 24-bit address word).
fn reg_read<S>(
    spi: &mut Spi<'static, S, SpiBlocking>,
    cs: &mut Output<'static>,
    addr: u32,
) -> Result<u32, embassy_rp::spi::Error>
where
    S: embassy_rp::spi::Instance,
{
    let cmd = [
        CMD_READ,
        ((addr >> 16) & 0xFF) as u8,
        ((addr >> 8) & 0xFF) as u8,
        (addr & 0xFF) as u8,
        0,
        0,
        0,
        0,
    ];
    let mut rx = [0u8; 8];
    cs.set_low();
    spi.blocking_transfer(&mut rx, &cmd)?;
    cs.set_high();
    // Response data follows the address bytes (little-endian within the word).
    Ok(u32::from_le_bytes([rx[4], rx[5], rx[6], rx[7]]))
}

/// Chip-select-gated SPI register write (one 24-bit address word).
fn reg_write<S>(
    spi: &mut Spi<'static, S, SpiBlocking>,
    cs: &mut Output<'static>,
    addr: u32,
    value: u32,
) -> Result<(), embassy_rp::spi::Error>
where
    S: embassy_rp::spi::Instance,
{
    let bytes = value.to_le_bytes();
    let cmd = [
        CMD_WRITE,
        ((addr >> 16) & 0xFF) as u8,
        ((addr >> 8) & 0xFF) as u8,
        (addr & 0xFF) as u8,
        bytes[0],
        bytes[1],
        bytes[2],
        bytes[3],
    ];
    cs.set_low();
    spi.blocking_transfer(&mut [0u8; 8], &cmd)?;
    cs.set_high();
    Ok(())
}

/// The MCP2518FD controller handle. Owns the chip-select; the SPI bus is shared
/// with the MCP3208 and therefore passed into each operation by reference.
pub struct Mcp2518fd {
    cs: Output<'static>,
}

impl Mcp2518fd {
    /// Create the driver. `cs` is the chip-select pin, driven high at rest.
    pub fn new(mut cs: Output<'static>) -> Self {
        cs.set_high();
        Self { cs }
    }

    /// Issue a software RESET command (takes ~3 µs; caller should delay).
    pub fn reset<S: embassy_rp::spi::Instance>(
        &mut self,
        spi: &mut Spi<'static, S, SpiBlocking>,
    ) -> Result<(), embassy_rp::spi::Error> {
        self.cs.set_low();
        spi.blocking_write(&[CMD_RESET])?;
        self.cs.set_high();
        Ok(())
    }

    /// Read a controller register.
    pub fn read<S: embassy_rp::spi::Instance>(
        &mut self,
        spi: &mut Spi<'static, S, SpiBlocking>,
        addr: u32,
    ) -> Result<u32, embassy_rp::spi::Error> {
        reg_read(spi, &mut self.cs, addr)
    }

    /// Write a controller register.
    pub fn write<S: embassy_rp::spi::Instance>(
        &mut self,
        spi: &mut Spi<'static, S, SpiBlocking>,
        addr: u32,
        value: u32,
    ) -> Result<(), embassy_rp::spi::Error> {
        reg_write(spi, &mut self.cs, addr, value)
    }

    /// Configure the controller for classic CAN at 500 kbit/s.
    ///
    /// TODO(bring-up): the NBTCFG timing values, FIFO base address (CiFIFOBA)
    /// and receive-filter setup depend on the system clock (the MCP2518FD is
    /// clocked from a 40 MHz crystal on revL) and the desired acceptance
    /// filter. Validate against a live bus before relying on this.
    pub fn configure_500kb<S: embassy_rp::spi::Instance>(
        &mut self,
        spi: &mut Spi<'static, S, SpiBlocking>,
        _filter: CanFilter,
    ) -> Result<(), embassy_rp::spi::Error> {
        self.reset(spi)?;
        // TODO(bring-up): program CiCON (mode = config), CiNBTCFG for 500 kbit/s
        // at 40 MHz, set up FIFO0 as TX and FIFO1 as RX with the acceptance
        // filter, then set CiCON.OpMode = Normal. See datasheet §5.
        let _ = (REG_CICON, REG_CINBTCFG, REG_CIFIFOCON, REG_CIFIFOSTA);
        Ok(())
    }
}

// ---------------------------------------------------------------------------
// Shared receive/transmit buffers + CanBus trait implementation.
// ---------------------------------------------------------------------------

/// Shared CAN state: the receive ring (filled by a background task) plus a
/// count of frames queued for transmit.
pub struct CanState {
    rx: CanRxBuf,
    /// Number of frames waiting in the TX ring (the ring itself lives in the
    /// background task for simplicity; this is the high-water telemetry).
    tx_pending: AtomicU16,
}

impl CanState {
    /// Create with no ID filter (accept all IDs).
    pub const fn new() -> Self {
        Self {
            rx: CanRxBuf::new(),
            tx_pending: AtomicU16::new(0),
        }
    }

    /// Create with the given acceptance filter.
    pub const fn with_filter(filter: CanFilter) -> Self {
        Self {
            rx: CanRxBuf::with_filter(filter),
            tx_pending: AtomicU16::new(0),
        }
    }

    /// Producer entry-point for the background receive task: push a freshly
    /// received frame into the ring.
    pub fn push_rx(&self, frame: CanFrame) {
        let _ = self.rx.push(frame);
    }

    /// Number of frames currently awaiting transmit (telemetry).
    pub fn tx_pending(&self) -> u16 {
        self.tx_pending.load(Ordering::Relaxed)
    }
}

/// `CanBus` trait wrapper backed by shared [`CanState`].
///
/// `try_recv` pops from the receive ring; `send` queues for the background task
/// to drain into the MCP2518FD TX FIFO. Neither touches the SPI bus directly,
/// so the trait can be used from the controller loop without bus ownership.
pub struct RpCan {
    state: &'static CanState,
}

impl RpCan {
    pub fn new(state: &'static CanState) -> Self {
        Self { state }
    }
}

impl CanBus for RpCan {
    type Error = ();

    fn try_recv(&mut self) -> Result<Option<CanFrame>, Self::Error> {
        Ok(self.state.rx.pop())
    }

    fn send(&mut self, _frame: &CanFrame) -> Result<(), Self::Error> {
        // TODO(bring-up): push into a TX ring drained by the background task
        // that owns the SPI bus, which writes the frame into the MCP2518FD TX
        // FIFO and requests transmission. Until that task is wired, bumping the
        // pending counter gives basic observability.
        self.state
            .tx_pending
            .fetch_update(Ordering::Relaxed, Ordering::Relaxed, |v| v.checked_add(1))
            .ok();
        Ok(())
    }
}
