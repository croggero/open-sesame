# Open Sesame

ESP32-S3 firmware (Rust + esp-idf) for an assistive sliding door controller.

## Features

- Velocity-based assistive open/close
- Auto end-stop calibration and optional close-side homing
- Stall detection (obstacle / end-stop)
- Home Assistant MQTT discovery; commands, state and telemetry topics
- Captive-portal provisioning, factory reset via GPIO 0
- Build-time hardware/tuning configuration in `config.toml`

## Hardware

- ESP32-S3
- DRV8874 H-bridge (PhEn or InIn mode, set in `config.toml`)
- Quadrature encoder on the door drive
- Status LED, factory-reset button, optional toggle button

Pin assignments live under `[pins]` in `config.toml`.

## Building

Requires the Espressif Rust toolchain. See https://docs.esp-rs.org/book/installation/ for setup details.

1. Install `espup` and the `esp` toolchain:
   ```bash
   cargo install espup
   espup install
   ```
2. Source the export script in your shell (path printed by `espup install`):
   ```bash
   . $HOME/export-esp.sh
   ```
3. Install the flashing tools:
   ```bash
   cargo install espflash ldproxy
   ```
4. Edit `config.toml` to match your hardware (pins, motor mode, tuning).
5. Build:
   ```bash
   cargo build --release
   ```
6. Flash a connected ESP32 and open a serial monitor:
   ```bash
   cargo run --release
   ```
   Or, to flash with probe-rs and attach a debugger:
   ```bash
   cargo embed --release
   ```
  Or
  ```bash
  cargo flash --release --chip esp32s3
  ```

## First boot

1. Power the board. The status LED flashes to indicate the captive portal is active.
2. Connect to the `Open Sesame` SoftAP from a phone or laptop.
3. Submit WiFi and MQTT credentials in the provisioning form. The device reboots and connects.
4. The door is now visible in Home Assistant via MQTT discovery.

To re-provision, hold the factory-reset button (GPIO 0) for 5 seconds.

## Layout

- `src/main.rs` — boot sequence and control loop
- `src/door_controller.rs` — state machine, calibration, stall handling
- `src/motor.rs` — DRV8874 PWM driver (LEDC, 20 kHz)
- `src/encoder.rs` — PCNT quadrature decoder with 32-bit overflow tracking
- `src/mqtt_client.rs` — MQTT wrapper and command thread
- `src/captive_portal.rs` — SoftAP + HTTP provisioning
- `src/config.rs` — NVS-backed runtime config and build-time constants
- `config.toml` — hardware and tuning constants, parsed by `build.rs`
