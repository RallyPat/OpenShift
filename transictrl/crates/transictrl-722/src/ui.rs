// SPDX-License-Identifier: GPL-3.0-only
// Portions derived from 7226ctrl (c) Markus Kovero et al., GPL-3.0

//! On-device OLED UI, ported from `main/ui.cpp`.
//!
//! The original code is tightly coupled to U8g2 and the exact 128×64 layout.
//! This module decouples the *page/state model* from the display driver by
//! accepting a [`DisplayDrawer`] trait. Board support crates provide the
//! concrete renderer.

use crate::telemetry::TelemetryFrame;

/// Which top-level page the UI is showing.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum UiPage {
    #[default]
    Main,
    MaxValues,
    /// `infoDisplay` codes from the original firmware: 1=lamp, 2=boost, etc.
    Info(u8),
}

/// Snapshot of everything the UI wants to draw.
#[derive(Debug, Clone, Copy, Default)]
pub struct UiState {
    pub page: UiPage,
    pub wanted_gear: u8,
    pub actual_gear: u8,
    pub full_auto: bool,
    pub shift_pending: bool,
    pub pre_shift: bool,
    pub post_shift: bool,
    pub telemetry: TelemetryFrame,
    pub boost_limit_active: bool,
    pub boost_override: u16,
    pub diff_speed_enabled: bool,
    pub rpm_speed_enabled: bool,
    pub slip_fault: bool,
    pub battery_fault: bool,
}

/// Trait implemented by the board-specific display driver.
pub trait DisplayDrawer {
    type Error: core::fmt::Debug;

    /// Clear and present a full page.
    fn draw_main(&mut self, state: &UiState) -> Result<(), Self::Error>;
    fn draw_max_values(&mut self, state: &UiState) -> Result<(), Self::Error>;
    fn draw_info(&mut self, state: &UiState, code: u8) -> Result<(), Self::Error>;
}

/// Simple UI manager.
pub struct UiManager<D: DisplayDrawer> {
    drawer: D,
    max: MaxValues,
}

#[derive(Debug, Clone, Copy, Default)]
#[allow(dead_code)]
struct MaxValues {
    speed: f32,
    boost: f32,
    exhaust_pres: f32,
    oil_temp: f32,
    atf_temp: f32,
    rpm: f32,
    pres_diff: f32,
}

impl<D: DisplayDrawer> UiManager<D> {
    pub fn new(drawer: D) -> Self {
        Self {
            drawer,
            max: MaxValues::default(),
        }
    }

    /// Call once per tick to update peaks and redraw.
    pub fn update(&mut self, state: &UiState) -> Result<(), D::Error> {
        self.update_max(state);

        match state.page {
            UiPage::Main => self.drawer.draw_main(state),
            UiPage::MaxValues => self.drawer.draw_max_values(state),
            UiPage::Info(code) => self.drawer.draw_info(state, code),
        }
    }

    fn update_max(&mut self, state: &UiState) {
        let t = state.telemetry;
        self.max.speed = self.max.speed.max(t.vehicle_speed_kmh);
        self.max.boost = self.max.boost.max(t.boost_kpa);
        self.max.exhaust_pres = self.max.exhaust_pres.max(t.exhaust_temp_c);
        self.max.oil_temp = self.max.oil_temp.max(t.oil_temp_c);
        self.max.atf_temp = self.max.atf_temp.max(t.atf_temp_c);
        self.max.rpm = self.max.rpm.max(t.engine_rpm);
        self.max.pres_diff = self.max.pres_diff.max(t.boost_kpa - t.oil_temp_c);
    }
}
