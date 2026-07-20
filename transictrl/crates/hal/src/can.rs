// SPDX-License-Identifier: GPL-3.0-only

//! CAN receive buffering and filtering.
//!
//! The revL carrier talks CAN-FD through an MCP2518FD SPI controller. Frames
//! arrive at interrupt rate (potentially hundreds per second on a busy vehicle
//! bus), while the transmission controller polls much more slowly. This module
//! provides a small single-producer / single-consumer ring buffer to decouple
//! those rates, plus an optional ID filter so irrelevant frames (e.g. the bulk
//! of body-domain traffic) are dropped before they ever enter the queue.
//!
//! Everything here is plain arithmetic + atomics (no allocator, no `heapless`),
//! so the crate stays dependency-free and the logic is unit-testable on host.

use core::cell::Cell;
use core::sync::atomic::{AtomicUsize, Ordering};

use crate::CanFrame;

/// Capacity of the receive ring (power-of-two for cheap modulo). The ring
/// holds at most `CAPACITY - 1` frames at once (one slot stays empty to
/// distinguish full from empty); when a push would overflow, the **oldest**
/// frame is evicted so the newest traffic is always kept. Sized to ride out a
/// short burst on a moderately busy CAN bus without evicting between controller
/// ticks.
pub const CAPACITY: usize = 16;

/// Mask: `CAPACITY - 1`. `CAPACITY` must be a power of two for the modular
/// indexing used below to be correct.
const MASK: usize = CAPACITY - 1;

/// A CAN ID acceptance filter: a frame passes when
/// `(frame.id & mask) == id & mask`. The default (mask 0) accepts everything.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CanFilter {
    pub id: u32,
    pub mask: u32,
}

impl CanFilter {
    /// Accept every ID (no filtering).
    pub const fn accept_all() -> Self {
        Self { id: 0, mask: 0 }
    }

    /// Accept exactly one 11-bit standard ID.
    pub const fn accept_id(id: u32) -> Self {
        Self { id, mask: 0x7FF }
    }

    /// Does `frame` match this filter?
    #[inline]
    pub fn matches(&self, frame: &CanFrame) -> bool {
        (frame.id & self.mask) == (self.id & self.mask)
    }
}

impl Default for CanFilter {
    fn default() -> Self {
        Self::accept_all()
    }
}

/// Lock-free single-producer / single-consumer ring buffer of CAN frames.
///
/// The MCP2518FD receive path pushes from one context (the SPI/interrupt task)
/// and the controller polls from another. `head` is written by the producer and
/// read by the consumer; `tail` is written by the consumer and read by the
/// producer — the classic SPSC queue.
///
/// `CanFrame` is `Copy`, so the slots use `Cell` for safe interior mutability
/// behind a shared `&self` reference without any `unsafe` code.
pub struct CanRxBuf {
    slots: [Cell<CanFrame>; CAPACITY],
    head: AtomicUsize, // producer writes here
    tail: AtomicUsize, // consumer writes here
    filter: CanFilter,
}

impl CanRxBuf {
    /// Create an empty buffer with no ID filter (accepts everything).
    pub const fn new() -> Self {
        Self {
            slots: [const { Cell::new(CAN_FRAME_ZERO) }; CAPACITY],
            head: AtomicUsize::new(0),
            tail: AtomicUsize::new(0),
            filter: CanFilter::accept_all(),
        }
    }

    /// Create an empty buffer with the given acceptance filter.
    pub const fn with_filter(filter: CanFilter) -> Self {
        Self {
            slots: [const { Cell::new(CAN_FRAME_ZERO) }; CAPACITY],
            head: AtomicUsize::new(0),
            tail: AtomicUsize::new(0),
            filter,
        }
    }

    /// Install/replace the acceptance filter. Only meaningful while the queue
    /// is empty (existing contents are not retroactively filtered).
    pub fn set_filter(&mut self, filter: CanFilter) {
        self.filter = filter;
    }

    /// Current acceptance filter.
    pub fn filter(&self) -> CanFilter {
        self.filter
    }

    /// Producer: enqueue a frame, evicting the oldest if the ring is full.
    ///
    /// Always returns `Ok(())`: frames that fail the ID filter are dropped
    /// silently, and a full ring evicts its oldest entry rather than rejecting
    /// the new one (a controller is more interested in fresh commands than
    /// stale ones).
    pub fn push(&self, frame: CanFrame) -> Result<(), CanFrame> {
        if !self.filter.matches(&frame) {
            // Filtered out: not an error, just ignored.
            return Ok(());
        }
        let head = self.head.load(Ordering::Acquire);
        let tail = self.tail.load(Ordering::Acquire);
        let next = (head + 1) & MASK;
        if next == tail {
            // Queue full: drop the oldest entry by advancing tail so the newest
            // traffic is preserved (a controller is more interested in fresh
            // commands than stale ones).
            self.tail.store((tail + 1) & MASK, Ordering::Release);
        }
        // Safe write: only the producer touches `slots[head]` (SPSC contract),
        // and `Cell` provides interior mutability without `unsafe`.
        self.slots[head].set(frame);
        self.head.store(next, Ordering::Release);
        Ok(())
    }

    /// Consumer: dequeue the oldest frame, if any.
    pub fn pop(&self) -> Option<CanFrame> {
        let tail = self.tail.load(Ordering::Acquire);
        let head = self.head.load(Ordering::Acquire);
        if tail == head {
            return None;
        }
        // Safe read: only the consumer touches `slots[tail]` (SPSC contract).
        let frame = self.slots[tail].get();
        self.tail.store((tail + 1) & MASK, Ordering::Release);
        Some(frame)
    }

    /// Number of frames currently buffered (advisory; may be stale by design).
    pub fn len(&self) -> usize {
        let head = self.head.load(Ordering::Relaxed);
        let tail = self.tail.load(Ordering::Relaxed);
        (head + CAPACITY - tail) & MASK
    }

    /// Whether the buffer is empty (advisory).
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }
}

const CAN_FRAME_ZERO: CanFrame = CanFrame { id: 0, len: 0, data: [0; 8] };

#[cfg(test)]
mod tests {
    use super::*;

    fn frame(id: u32, data: u8) -> CanFrame {
        let mut f = CanFrame { id, len: 1, data: [0; 8] };
        f.data[0] = data;
        f
    }

    #[test]
    fn filter_accept_all_passes_everything() {
        let f = CanFilter::accept_all();
        assert!(f.matches(&frame(0x123, 1)));
        assert!(f.matches(&frame(0x7FF, 2)));
        assert!(f.matches(&frame(0, 3)));
    }

    #[test]
    fn filter_exact_id_matches_only_that_id() {
        let f = CanFilter::accept_id(0x220); // the 722.6 gearstick ID is 560 dec
        assert!(f.matches(&frame(0x220, 1)));
        assert!(!f.matches(&frame(0x221, 2)));
        assert!(!f.matches(&frame(0, 3)));
    }

    #[test]
    fn push_then_pop_roundtrips() {
        let buf = CanRxBuf::new();
        assert!(buf.is_empty());
        buf.push(frame(0x100, 10)).unwrap();
        buf.push(frame(0x100, 20)).unwrap();
        assert_eq!(buf.len(), 2);

        let a = buf.pop().unwrap();
        let b = buf.pop().unwrap();
        assert_eq!(a.data[0], 10); // FIFO order preserved
        assert_eq!(b.data[0], 20);
        assert!(buf.is_empty());
    }

    #[test]
    fn filter_drops_unwanted_ids_silently() {
        let buf = CanRxBuf::with_filter(CanFilter::accept_id(560));
        buf.push(frame(560, 1)).unwrap(); // accepted
        assert!(buf.push(frame(999, 2)).is_ok()); // filtered, not an error
        assert_eq!(buf.len(), 1);
        assert_eq!(buf.pop().unwrap().id, 560);
    }

    #[test]
    fn overflow_drops_oldest_preserves_newest() {
        let buf = CanRxBuf::new();
        // The one-empty-slot ring holds CAPACITY-1 items before evicting. Push
        // enough that several overflows occur, then confirm the oldest frames
        // were evicted while the newest survive.
        for i in 0..(CAPACITY + 2) {
            buf.push(frame(0x100, i as u8)).unwrap();
        }
        // After pushing CAPACITY+2 items with eviction, CAPACITY-1 remain; the
        // oldest survivor is the (CAPACITY+2) - (CAPACITY-1) = 3rd item pushed.
        let expected_oldest = ((CAPACITY + 2) - (CAPACITY - 1)) as u8;
        let first = buf.pop().unwrap();
        assert_eq!(first.data[0], expected_oldest);
        assert_eq!(buf.len(), CAPACITY - 2);
    }

    #[test]
    fn spsc_queue_drains_in_order() {
        let buf = CanRxBuf::new();
        // Push exactly CAPACITY-1 items (the non-evicting capacity) and drain.
        for i in 0..(CAPACITY - 1) {
            buf.push(frame(0x10, i as u8)).unwrap();
        }
        for i in 0..(CAPACITY - 1) {
            assert_eq!(buf.pop().unwrap().data[0], i as u8);
        }
        assert!(buf.is_empty());
        assert!(buf.pop().is_none());
    }
}
