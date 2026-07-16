# Data visualization

This directory contains the **legacy** Node.js + React live telemetry stack.
A new Rust-native dataviz app is also available under `transictrl/host/dataviz`.

## Legacy stack (`dataviz/backend` + `dataviz/frontend`)

A Node.js backend (`BufferReader`) reads serial data from the controller, pushes
it into InfluxDB, and serves a React frontend over WebSocket. Designed for local
live dashboards plus remote replication.

```bash
cd dataviz/backend
npm install
npm start

cd dataviz/frontend
npm install
npm start
```

See [`datalog/README.md`](../datalog/README.md) for the legacy message format.

## New Rust dataviz (`transictrl/host/dataviz`)

A native [egui](https://github.com/emilk/egui)/eframe application that connects to
the `transictrl/host/server` WebSocket telemetry endpoint (`ws://localhost:8080/ws/telemetry`).
It can also be compiled to WASM for in-browser use.

```bash
cd transictrl
cargo run -p dataviz
```

Run the server first so the app has telemetry to display:

```bash
cargo run -p server             # simulated telemetry
# or
cargo run -p server -- -s COM3  # from a real device
```
