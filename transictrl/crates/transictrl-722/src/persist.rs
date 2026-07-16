// SPDX-License-Identifier: GPL-3.0-only
// Portions derived from 7226ctrl (c) Markus Kovero et al., GPL-3.0

//! Calibration / adaption persistence.
//!
//! Replaces the Arduino `EEPROM` usage in `main/eeprom.cpp`. The runtime
//! configuration is serialized with `postcard` and stored through the
//! `PersistentStore` HAL trait.

use transictrl_hal::PersistentStore;

use crate::config::Config722;

/// Persisted calibration blob.
#[derive(Debug, Clone, Copy, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct CalibrationBlob {
    pub config: Config722,
    /// Per-shift adaption modifiers, indexed as `[from_gear][to_gear]`.
    /// 100 means no correction; valid range 50..150.
    pub adaption: [[u8; 6]; 6],
    /// TPS calibration min/max ADC counts.
    pub tps_min: u16,
    pub tps_max: u16,
}

impl Default for CalibrationBlob {
    fn default() -> Self {
        Self {
            config: Config722::default(),
            adaption: [[100; 6]; 6],
            tps_min: 0,
            tps_max: 1023,
        }
    }
}

/// Load a calibration blob from persistent storage.
///
/// Returns `Ok(None)` if the stored data is invalid or uninitialized.
pub fn load<S: PersistentStore>(store: &mut S) -> Result<Option<CalibrationBlob>, S::Error> {
    let mut buf = [0u8; 512];
    store.load(&mut buf)?;

    // Simple uninitialized check: if all zeros, treat as empty.
    if buf.iter().all(|&b| b == 0) {
        return Ok(None);
    }

    match postcard::from_bytes::<CalibrationBlob>(&buf) {
        Ok(blob) => Ok(Some(blob)),
        Err(_) => Ok(None),
    }
}

/// Save a calibration blob to persistent storage.
pub fn save<S: PersistentStore>(store: &mut S, blob: &CalibrationBlob) -> Result<(), S::Error> {
    let mut buf = [0u8; 512];
    match postcard::to_slice(blob, &mut buf) {
        Ok(bytes) => store.save(bytes),
        Err(_) => Ok(()),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    struct VecStore(Vec<u8>);

    impl PersistentStore for VecStore {
        type Error = ();

        fn load(&mut self, buf: &mut [u8]) -> Result<(), Self::Error> {
            let len = buf.len().min(self.0.len());
            buf[..len].copy_from_slice(&self.0[..len]);
            Ok(())
        }

        fn save(&mut self, buf: &[u8]) -> Result<(), Self::Error> {
            self.0.resize(buf.len(), 0);
            self.0.copy_from_slice(buf);
            Ok(())
        }
    }

    #[test]
    fn roundtrip_config_and_adaption() {
        let mut blob = CalibrationBlob::default();
        blob.config.boost_max = 350;
        blob.adaption[1][2] = 110;
        blob.adaption[2][3] = 90;

        let mut store = VecStore(vec![0u8; 512]);
        save(&mut store, &blob).unwrap();
        let loaded = load(&mut store).unwrap().expect("valid blob loads");
        assert_eq!(loaded.config.boost_max, 350);
        assert_eq!(loaded.adaption[1][2], 110);
        assert_eq!(loaded.adaption[2][3], 90);
    }

    #[test]
    fn empty_store_returns_none() {
        let mut store = VecStore(vec![0u8; 512]);
        assert!(load(&mut store).unwrap().is_none());
    }
}
