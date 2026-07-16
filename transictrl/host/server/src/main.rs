use std::{net::SocketAddr, path::PathBuf, time::Duration};

use axum::{
    extract::{State, WebSocketUpgrade},
    response::IntoResponse,
    routing::{get, post},
    Json, Router,
};
use clap::Parser;
use tokio::sync::broadcast;
use tower_http::services::ServeDir;
use tracing_subscriber::{layer::SubscriberExt, util::SubscriberInitExt};

#[derive(Clone)]
struct AppState {
    /// Broadcast channel carrying telemetry frames to all WebSocket clients.
    telemetry_tx: broadcast::Sender<protocol::transport::TelemetryFrame>,
}

#[derive(Parser)]
#[command(author, version, about = "TransiCtrl host server", long_about = None)]
struct Cli {
    /// Stream telemetry from a serial port instead of simulating it.
    #[arg(short, long)]
    serial_port: Option<String>,
    /// Baud rate for the serial port.
    #[arg(short, long, default_value_t = 115_200)]
    baud: u32,
}

#[tokio::main]
async fn main() {
    let cli = Cli::parse();

    tracing_subscriber::registry()
        .with(tracing_subscriber::EnvFilter::new(
            std::env::var("RUST_LOG").unwrap_or_else(|_| "server=debug,tower_http=debug".into()),
        ))
        .with(tracing_subscriber::fmt::layer())
        .init();

    let (telemetry_tx, _telemetry_rx) = broadcast::channel::<protocol::transport::TelemetryFrame>(64);
    let state = AppState {
        telemetry_tx: telemetry_tx.clone(),
    };

    if let Some(port) = cli.serial_port {
        spawn_serial_reader(port, cli.baud, telemetry_tx.clone());
    } else {
        spawn_simulator(telemetry_tx.clone());
    }

    let ui_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("ui");

    let app = Router::new()
        .route("/api/ping", get(|| async { "ok" }))
        .route("/api/device/info", get(get_device_info))
        .route("/api/calibration/validate", post(validate_calibration))
        .route("/ws/telemetry", get(telemetry_ws))
        .nest_service("/", ServeDir::new(ui_dir))
        .with_state(state);

    let addr: SocketAddr = "0.0.0.0:8080".parse().unwrap();
    tracing::info!(%addr, "listening");
    axum::serve(tokio::net::TcpListener::bind(addr).await.unwrap(), app)
        .await
        .unwrap();
}

async fn get_device_info() -> Json<serde_json::Value> {
    Json(serde_json::json!({
        "firmware_version": "dev",
        "hardware": "simulated",
    }))
}

async fn validate_calibration(
    State(_state): State<AppState>,
    Json(value): Json<serde_json::Value>,
) -> Json<serde_json::Value> {
    let res = serde_json::from_value::<protocol::calibration::CalibrationFile>(value)
        .map(|v| serde_json::to_value(v).unwrap());
    match res {
        Ok(ok) => Json(serde_json::json!({"ok": true, "normalized": ok})),
        Err(err) => Json(serde_json::json!({"ok": false, "error": err.to_string()})),
    }
}

async fn telemetry_ws(State(state): State<AppState>, ws: WebSocketUpgrade) -> impl IntoResponse {
    ws.on_upgrade(move |socket| handle_telemetry_socket(socket, state.telemetry_tx))
}

async fn handle_telemetry_socket(
    mut socket: axum::extract::ws::WebSocket,
    telemetry_tx: broadcast::Sender<protocol::transport::TelemetryFrame>,
) {
    let mut rx = telemetry_tx.subscribe();
    loop {
        let frame = match rx.recv().await {
            Ok(f) => f,
            Err(tokio::sync::broadcast::error::RecvError::Lagged(_)) => continue,
            Err(tokio::sync::broadcast::error::RecvError::Closed) => break,
        };
        let text = serde_json::to_string(&frame.to_legacy_array()).unwrap();
        if socket.send(axum::extract::ws::Message::Text(text)).await.is_err() {
            break;
        }
    }
}

fn spawn_simulator(telemetry_tx: broadcast::Sender<protocol::transport::TelemetryFrame>) {
    tokio::spawn(async move {
        let mut t = 0.0f32;
        loop {
            let frame = protocol::transport::TelemetryFrame {
                vehicle_speed_kmh: 60.0 + 20.0 * t.sin(),
                engine_rpm: 2500.0 + 500.0 * (t * 1.3).sin(),
                tps_percent: 30.0 + 20.0 * (t * 0.7).sin(),
                oil_temp_c: 90.0,
                atf_temp_c: 80.0 + 5.0 * t.sin(),
                load_percent: 40.0,
                boost_kpa: 150.0 + 50.0 * (t * 2.0).sin(),
                exhaust_temp_c: 400.0,
                battery_v: 13.8,
                current_gear: 4,
                target_gear: 4,
                shifting: false,
            };
            let _ = telemetry_tx.send(frame);
            tokio::time::sleep(Duration::from_millis(100)).await;
            t += 0.1;
        }
    });
}

fn spawn_serial_reader(
    port: String,
    baud: u32,
    telemetry_tx: broadcast::Sender<protocol::transport::TelemetryFrame>,
) {
    std::thread::spawn(move || {
        let mut port = match serialport::new(&port, baud)
            .timeout(Duration::from_millis(100))
            .open()
        {
            Ok(p) => p,
            Err(e) => {
                tracing::error!(port, error = %e, "failed to open serial port");
                return;
            }
        };

        let mut line = String::new();
        let mut buf = [0u8; 256];
        loop {
            match port.read(&mut buf) {
                Ok(0) => {}
                Ok(n) => {
                    for &b in &buf[..n] {
                        if b == b'\n' {
                            if let Some(frame) = parse_csv_telemetry(&line) {
                                let _ = telemetry_tx.send(frame);
                            }
                            line.clear();
                        } else if b != b'\r' {
                            line.push(b as char);
                        }
                    }
                }
                Err(e) if e.kind() == std::io::ErrorKind::TimedOut => {}
                Err(e) => {
                    tracing::error!(error = %e, "serial read error");
                    break;
                }
            }
        }
    });
}

/// Parse the legacy CSV telemetry line emitted by the firmware simulators.
///
/// Expected order (12 values):
/// n2_hz, n3_hz, vehicle_speed_hz, engine_rpm, tps_adc, atf_adc, oil_adc,
/// boost_adc, battery_adc, current_gear, target_gear, shifting
fn parse_csv_telemetry(line: &str) -> Option<protocol::transport::TelemetryFrame> {
    let mut vals = [0.0f32; 12];
    let mut count = 0usize;
    for (i, part) in line.split(',').enumerate() {
        if i >= 12 {
            return None;
        }
        vals[i] = part.parse().unwrap_or(0.0);
        count = i + 1;
    }
    if count != 12 {
        return None;
    }
    Some(protocol::transport::TelemetryFrame {
        vehicle_speed_kmh: vals[2], // firmware emits Hz; convert loosely here
        engine_rpm: vals[3],
        tps_percent: vals[4],
        oil_temp_c: vals[6],
        atf_temp_c: vals[5],
        load_percent: vals[4],
        boost_kpa: vals[7],
        exhaust_temp_c: 0.0,
        battery_v: vals[8],
        current_gear: vals[9] as u8,
        target_gear: vals[10] as u8,
        shifting: vals[11] != 0.0,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parse_csv_telemetry_valid_line() {
        let line = "100,0,11.111,1200,512,1450,435,0,4095,3,4,1";
        let frame = parse_csv_telemetry(line).unwrap();
        assert_eq!(frame.vehicle_speed_kmh, 11.111);
        assert_eq!(frame.engine_rpm, 1200.0);
        assert_eq!(frame.current_gear, 3);
        assert_eq!(frame.target_gear, 4);
        assert!(frame.shifting);
    }

    #[test]
    fn parse_csv_telemetry_short_line_is_none() {
        assert!(parse_csv_telemetry("1,2,3").is_none());
    }
}
