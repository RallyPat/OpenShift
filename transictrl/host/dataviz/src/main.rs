// SPDX-License-Identifier: GPL-3.0-only

//! Rust native data visualization front-end for OpenShift.
//!
//! This replaces the legacy React dataviz app. It connects to the host server
//! WebSocket and displays vehicle speed, RPM, boost, and temperatures using
//! egui gauges and plots.

use std::sync::mpsc::{channel, Receiver};

use futures::StreamExt;
use protocol::transport::TelemetryFrame;

fn main() {
    let (tx, rx) = channel();
    std::thread::spawn(move || telemetry_task(tx));

    let native_options = eframe::NativeOptions::default();
    eframe::run_native(
        "OpenShift DataViz",
        native_options,
        Box::new(|_cc| Ok(Box::new(DatavizApp::new(rx)))),
    )
    .unwrap();
}

fn telemetry_task(tx: std::sync::mpsc::Sender<TelemetryFrame>) {
    let rt = tokio::runtime::Runtime::new().unwrap();
    rt.block_on(async {
        let url = url::Url::parse("ws://localhost:8080/ws/telemetry").unwrap();
        let Ok((mut socket, _)) = tokio_tungstenite::connect_async(url).await else {
            eprintln!("failed to connect to telemetry server");
            return;
        };

        while let Some(Ok(msg)) = socket.next().await {
            if let tokio_tungstenite::tungstenite::Message::Text(text) = msg {
                if let Ok(arr) = serde_json::from_str::<[f32; 12]>(&text) {
                    let frame = TelemetryFrame {
                        vehicle_speed_kmh: arr[0],
                        engine_rpm: arr[1],
                        tps_percent: arr[2],
                        oil_temp_c: arr[3],
                        atf_temp_c: arr[4],
                        load_percent: arr[5],
                        boost_kpa: arr[6],
                        exhaust_temp_c: arr[7],
                        battery_v: arr[8],
                        current_gear: arr[9] as u8,
                        target_gear: arr[10] as u8,
                        shifting: arr[11] != 0.0,
                    };
                    let _ = tx.send(frame);
                }
            }
        }
    });
}

struct DatavizApp {
    rx: Receiver<TelemetryFrame>,
    history: Vec<TelemetryFrame>,
}

impl DatavizApp {
    fn new(rx: Receiver<TelemetryFrame>) -> Self {
        Self {
            rx,
            history: Vec::with_capacity(500),
        }
    }

    fn latest(&self) -> TelemetryFrame {
        self.history.last().copied().unwrap_or_default()
    }
}

impl eframe::App for DatavizApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        while let Ok(frame) = self.rx.try_recv() {
            self.history.push(frame);
            if self.history.len() > 500 {
                self.history.remove(0);
            }
        }

        let frame = self.latest();

        egui::CentralPanel::default().show(ctx, |ui| {
            ui.heading("OpenShift Telemetry");

            ui.horizontal(|ui| {
                ui.vertical(|ui| {
                    ui.label(format!("Speed: {:.1} km/h", frame.vehicle_speed_kmh));
                    ui.label(format!("RPM: {:.0}", frame.engine_rpm));
                    ui.label(format!("TPS: {:.1} %", frame.tps_percent));
                    ui.label(format!("Gear: {}/{}", frame.current_gear, frame.target_gear));
                });
                ui.vertical(|ui| {
                    ui.label(format!("ATF: {:.1} °C", frame.atf_temp_c));
                    ui.label(format!("Oil: {:.1} °C", frame.oil_temp_c));
                    ui.label(format!("Boost: {:.1} kPa", frame.boost_kpa));
                    ui.label(format!("Battery: {:.1} V", frame.battery_v));
                });
            });

            ui.separator();

            let speed_points: egui_plot::PlotPoints = self
                .history
                .iter()
                .enumerate()
                .map(|(i, f)| [i as f64, f.vehicle_speed_kmh as f64])
                .collect();
            let rpm_points: egui_plot::PlotPoints = self
                .history
                .iter()
                .enumerate()
                .map(|(i, f)| [i as f64, f.engine_rpm as f64])
                .collect();
            let boost_points: egui_plot::PlotPoints = self
                .history
                .iter()
                .enumerate()
                .map(|(i, f)| [i as f64, f.boost_kpa as f64])
                .collect();

            egui_plot::Plot::new("telemetry_plot")
                .height(300.0)
                .show(ui, |plot_ui| {
                    plot_ui.line(egui_plot::Line::new(speed_points).name("speed"));
                    plot_ui.line(egui_plot::Line::new(rpm_points).name("rpm"));
                    plot_ui.line(egui_plot::Line::new(boost_points).name("boost"));
                });
        });
    }
}
