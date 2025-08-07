use std::{net::SocketAddr, path::PathBuf};

use axum::{extract::State, routing::{get, post}, Json, Router};
use tower_http::services::ServeDir;
use tracing_subscriber::{layer::SubscriberExt, util::SubscriberInitExt};

#[derive(Clone)]
struct AppState;

#[tokio::main]
async fn main() {
    tracing_subscriber::registry()
        .with(tracing_subscriber::EnvFilter::new(
            std::env::var("RUST_LOG").unwrap_or_else(|_| "server=debug,tower_http=debug".into()),
        ))
        .with(tracing_subscriber::fmt::layer())
        .init();

    let state = AppState;

    let ui_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("ui");

    let app = Router::new()
        .route("/api/ping", get(|| async { "ok" }))
        .route("/api/device/info", get(get_device_info))
        .route("/api/calibration/validate", post(validate_calibration))
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
