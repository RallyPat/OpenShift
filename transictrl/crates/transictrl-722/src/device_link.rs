// SPDX-License-Identifier: GPL-3.0-only
// Portions derived from 7226ctrl (c) Markus Kovero et al., GPL-3.0

//! Host-device link for calibration push/pull over a serial port.
//!
//! Implements the binary framing protocol defined in
//! `protocol::transport::framing`. The firmware reads bytes from a
//! [`SerialLink`], parses frames, and either loads/saves a [`CalibrationBlob`]
//! or emits the current blob back to the host.
//!
//! This module is `#![no_std]` and uses a fixed-size buffer so it can run on
//! the microcontroller without an allocator.

use transictrl_hal::framing;
use transictrl_hal::{PersistentStore, SerialLink};

use crate::persist::CalibrationBlob;
use crate::Controller722;

/// Byte buffer used for both the incoming frame accumulator and the outgoing
/// frame payload. `BUF` must be at least `framing::OVERHEAD` bytes.
pub struct Link<B, const BUF: usize> {
    serial: B,
    rx_buf: [u8; BUF],
    rx_len: usize,
}

impl<B: SerialLink, const BUF: usize> Link<B, BUF> {
    /// Create a new device link wrapping `serial`.
    pub fn new(serial: B) -> Self {
        Self {
            serial,
            rx_buf: [0; BUF],
            rx_len: 0,
        }
    }

    /// Borrow the underlying serial link (e.g. to stream telemetry between
    /// command polls).
    pub fn serial(&mut self) -> &mut B {
        &mut self.serial
    }

    /// Poll the serial port for host commands and handle them.
    ///
    /// Call this from the firmware's main loop whenever time permits. It reads
    /// available bytes, looks for a complete frame, and dispatches. On success
    /// it returns the number of bytes consumed; on a serial error it returns
    /// the error.
    pub fn poll<S: PersistentStore>(
        &mut self,
        controller: &mut Controller722,
        store: &mut S,
    ) -> Result<usize, B::Error> {
        // Read whatever is currently available.
        let mut tmp = [0u8; 64];
        match self.serial.read(&mut tmp) {
            Ok(0) => return Ok(0),
            Ok(n) => {
                let free = BUF.saturating_sub(self.rx_len);
                let take = n.min(free);
                self.rx_buf[self.rx_len..self.rx_len + take]
                    .copy_from_slice(&tmp[..take]);
                self.rx_len += take;
            }
            Err(e) => return Err(e),
        }

        // Look for a sync byte and try to decode a complete frame.
        let consumed = if let Some(pos) = framing::find_sync(&self.rx_buf[..self.rx_len]) {
            let window = &self.rx_buf[pos..self.rx_len];
            let mut payload = [0u8; 512];
            match framing::decode(window, &mut payload) {
                Ok((opcode, len)) => {
                    let frame_len = len + framing::OVERHEAD;
                    self.dispatch(opcode, &payload[..len], controller, store);
                    frame_len + pos
                }
                Err(_) => {
                    // Frame incomplete or corrupt; drop bytes up to (and
                    // including) the sync to allow resync on next byte.
                    pos + 1
                }
            }
        } else {
            // No sync byte in buffer; discard everything.
            self.rx_len
        };

        self.rx_buf.copy_within(consumed..self.rx_len, 0);
        self.rx_len -= consumed;
        Ok(consumed)
    }

    fn dispatch<S: PersistentStore>(
        &mut self,
        opcode: u8,
        payload: &[u8],
        controller: &mut Controller722,
        store: &mut S,
    ) {
        match opcode {
            framing::host_op::PUSH_BLOB => {
                let mut out = [0u8; 512];
                if let Ok(blob) = postcard::from_bytes::<CalibrationBlob>(payload) {
                    controller.load_blob(&blob);
                    let response = match postcard::to_slice(&blob, &mut out) {
                        Ok(bytes) => {
                            if store.save(bytes).is_ok() {
                                framing::device_op::ACK
                            } else {
                                framing::device_op::NAK
                            }
                        }
                        Err(_) => framing::device_op::NAK,
                    };
                    let _ = self.send_simple(response);
                } else {
                    let _ = self.send_simple(framing::device_op::NAK);
                }
            }
            framing::host_op::PULL_BLOB => {
                let blob = controller.to_blob();
                let mut out = [0u8; 512];
                match postcard::to_slice(&blob, &mut out) {
                    Ok(bytes) => {
                        let mut frame = [0u8; 520];
                        if frame.len() >= bytes.len() + framing::OVERHEAD {
                            let n = framing::encode(framing::device_op::BLOB, bytes, &mut frame);
                            let _ = self.serial.write(&frame[..n]);
                        } else {
                            let _ = self.send_simple(framing::device_op::NAK);
                        }
                    }
                    Err(_) => {
                        let _ = self.send_simple(framing::device_op::NAK);
                    }
                }
            }
            _ => {
                let _ = self.send_simple(framing::device_op::NAK);
            }
        }
    }

    fn send_simple(&mut self, opcode: u8) -> Result<(), B::Error> {
        let mut frame = [0u8; framing::OVERHEAD];
        framing::encode(opcode, &[], &mut frame);
        self.serial.write(&frame)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    struct DummySerial {
        rx: heapless::Vec<u8, 256>,
        tx: heapless::Vec<u8, 256>,
    }

    impl DummySerial {
        fn feed(&mut self, bytes: &[u8]) {
            self.rx.extend_from_slice(bytes).unwrap();
        }
    }

    impl SerialLink for DummySerial {
        type Error = ();

        fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
            let n = buf.len().min(self.rx.len());
            buf[..n].copy_from_slice(&self.rx[..n]);
            self.rx.copy_within(n.., 0);
            self.rx.truncate(self.rx.len() - n);
            Ok(n)
        }

        fn write(&mut self, buf: &[u8]) -> Result<(), Self::Error> {
            self.tx.extend_from_slice(buf).unwrap();
            Ok(())
        }
    }

    struct DummyStore;

    impl PersistentStore for DummyStore {
        type Error = ();
        fn load(&mut self, _buf: &mut [u8]) -> Result<(), Self::Error> {
            Err(())
        }
        fn save(&mut self, _buf: &[u8]) -> Result<(), Self::Error> {
            Ok(())
        }
    }

    #[test]
    fn pull_blob_returns_valid_blob() {
        let serial = DummySerial {
            rx: heapless::Vec::new(),
            tx: heapless::Vec::new(),
        };
        let mut link: Link<DummySerial, 256> = Link::new(serial);
        let mut frame = [0u8; framing::OVERHEAD];
        framing::encode(framing::host_op::PULL_BLOB, &[], &mut frame);
        link.serial.feed(&frame);

        let mut controller = Controller722::new(crate::Config722::default(), 2);
        let mut store = DummyStore;
        link.poll(&mut controller, &mut store).unwrap();

        assert!(!link.serial.tx.is_empty());
        let sync = framing::find_sync(&link.serial.tx).unwrap();
        let mut payload = [0u8; 512];
        let (op, len) = framing::decode(&link.serial.tx[sync..], &mut payload).unwrap();
        assert_eq!(op, framing::device_op::BLOB);
        let blob: CalibrationBlob = postcard::from_bytes(&payload[..len]).unwrap();
        assert_eq!(blob.config, controller.to_blob().config);
    }
}
