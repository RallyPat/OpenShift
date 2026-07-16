# Datalog
This document describes datalogging format and method.

<http://www.mui.fi/7226ui.png>

Idea is to push sensor data to local time series database, replicate it remotely and process accordingly
for local visualization and remote historical data evaluation purposes.

## Hardware
Teensy outputs serial data to Surface Pro 2 running linux for data gathering.
GPS location data is gathered using NMEA/gpsd from Samsung J1 mobile phone also providing network capabilities.

## Method
All the data is brought together in nodeJS backend "BufferReader" also handling input to influxDB, which is internally replicated for remote influxDB instance.
Local and remote frontend connect backend using websocket for data visualization.

## Message format (legacy)
The original `datalog-testing` code outputs a semicolon-separated frame at 115200 baud on USB serial:

`vehicleSpeed(kmh);rpmSensor(rpm);tpsSensor(%);gear(1-5);oilTemp(celcius);atfTemp(celcius);load(%);boostSensor(kPa);boostLimit(kPa);lastSPC(int);modVal(int)`

For example:

`120;4300;30;4;80;90;50;200;250;110;5`

## New Rust telemetry format
`transictrl/host/server` consumes a comma-separated 12-field CSV line streamed over serial from the Rust firmware. Fields:

`n2_hz,n3_hz,vehicle_speed_hz,engine_rpm,tps_adc,atf_adc,oil_adc,boost_adc,battery_adc,current_gear,target_gear,shifting`

Example: `100,0,11.111,1200,512,1450,435,0,4095,3,4,1`

This is parsed into `protocol::transport::TelemetryFrame` and broadcast to `/ws/telemetry` WebSocket clients.


