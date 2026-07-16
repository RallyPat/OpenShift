// SPDX-License-Identifier: GPL-3.0-only

//! Simple length-delimited binary framing for host/device calibration transfers.
//!
//! Works without an allocator: the caller supplies a fixed-size buffer. A frame
//! is `[SYNC, OPCODE, LEN_HI, LEN_LO, payload..., CRC]` where CRC is the XOR of
//! all preceding bytes.

pub const SYNC: u8 = 0xAA;
pub const HEADER_LEN: usize = 4;
pub const CRC_LEN: usize = 1;
pub const OVERHEAD: usize = HEADER_LEN + CRC_LEN;

/// Host -> Device opcodes.
pub mod host_op {
    pub const PUSH_BLOB: u8 = 0x01;
    pub const PULL_BLOB: u8 = 0x02;
    pub const PUSH_CALIBRATION: u8 = 0x03;
}

/// Device -> Host opcodes.
pub mod device_op {
    pub const ACK: u8 = 0x81;
    pub const BLOB: u8 = 0x82;
    pub const NAK: u8 = 0x83;
}

/// Encode a frame into `out`. Returns the number of bytes written.
///
/// Panics if `out` is smaller than `payload.len() + OVERHEAD`.
pub fn encode(opcode: u8, payload: &[u8], out: &mut [u8]) -> usize {
    assert!(out.len() >= payload.len() + OVERHEAD);
    out[0] = SYNC;
    out[1] = opcode;
    let len = payload.len() as u16;
    out[2] = (len >> 8) as u8;
    out[3] = len as u8;
    out[HEADER_LEN..HEADER_LEN + payload.len()].copy_from_slice(payload);
    let crc = out[..HEADER_LEN + payload.len()]
        .iter()
        .fold(0u8, |a, &b| a ^ b);
    out[HEADER_LEN + payload.len()] = crc;
    payload.len() + OVERHEAD
}

/// Decode a frame from `buf`. Returns `(opcode, payload_len)` on success,
/// writing the payload into `payload_out`.
///
/// `payload_out` must be at least as large as the frame's length field. The
/// caller usually reads bytes until a full frame is available, then calls
/// `decode`.
pub fn decode(buf: &[u8], payload_out: &mut [u8]) -> Result<(u8, usize), DecodeError> {
    if buf.len() < OVERHEAD {
        return Err(DecodeError::TooShort);
    }
    if buf[0] != SYNC {
        return Err(DecodeError::BadSync);
    }
    let opcode = buf[1];
    let len = ((buf[2] as usize) << 8) | buf[3] as usize;
    if len > payload_out.len() {
        return Err(DecodeError::BufferTooSmall);
    }
    if buf.len() < len + OVERHEAD {
        return Err(DecodeError::TooShort);
    }
    let crc = buf[..HEADER_LEN + len]
        .iter()
        .fold(0u8, |a, &b| a ^ b);
    if crc != buf[HEADER_LEN + len] {
        return Err(DecodeError::BadCrc);
    }
    payload_out[..len].copy_from_slice(&buf[HEADER_LEN..HEADER_LEN + len]);
    Ok((opcode, len))
}

/// Scan `buf` for a sync byte and return the offset of the first candidate.
///
/// This is useful for resynchronizing a byte stream after a framing error.
pub fn find_sync(buf: &[u8]) -> Option<usize> {
    buf.iter().position(|&b| b == SYNC)
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DecodeError {
    TooShort,
    BadSync,
    BadCrc,
    BufferTooSmall,
}

impl core::fmt::Display for DecodeError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            DecodeError::TooShort => write!(f, "frame too short"),
            DecodeError::BadSync => write!(f, "bad sync byte"),
            DecodeError::BadCrc => write!(f, "CRC mismatch"),
            DecodeError::BufferTooSmall => write!(f, "payload buffer too small"),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn roundtrip_empty_payload() {
        let mut buf = [0u8; 16];
        let n = encode(host_op::PULL_BLOB, &[], &mut buf);
        assert_eq!(n, OVERHEAD);
        let mut payload = [0u8; 16];
        let (op, len) = decode(&buf[..n], &mut payload).unwrap();
        assert_eq!(op, host_op::PULL_BLOB);
        assert_eq!(len, 0);
    }

    #[test]
    fn roundtrip_blob_payload() {
        let data = [1u8, 2, 3, 4, 5];
        let mut buf = [0u8; 32];
        let n = encode(host_op::PUSH_BLOB, &data, &mut buf);
        let mut payload = [0u8; 32];
        let (op, len) = decode(&buf[..n], &mut payload).unwrap();
        assert_eq!(op, host_op::PUSH_BLOB);
        assert_eq!(len, data.len());
        assert_eq!(&payload[..len], &data[..]);
    }

    #[test]
    fn bad_crc_fails() {
        let mut buf = [0u8; 32];
        let n = encode(host_op::PULL_BLOB, &[1, 2, 3], &mut buf);
        buf[n - 1] ^= 0xFF;
        let mut payload = [0u8; 32];
        assert_eq!(decode(&buf[..n], &mut payload), Err(DecodeError::BadCrc));
    }

    #[test]
    fn find_sync_skips_garbage() {
        let garbage = [0x00, 0x11, SYNC, 0x01, 0x00, 0x02];
        assert_eq!(find_sync(&garbage), Some(2));
    }
}
