// src/main.rs
//
// Open Sesame Door Controller
//
// Boot sequence:
//   1. Check factory-reset button (GPIO 0, hold 5 s)
//   2. Load config from NVS
//   3. If no config → run captive portal, save, reboot
//   4. Connect WiFi → connect MQTT → publish HA discovery
//   5. Start assistive door control loop on this thread
//   6. On MQTT config/set message → save new config, reboot
//
// Pin map — see src/pins.rs to remap all GPIO assignments.

use anyhow::Result;
use esp_idf_hal::{
    gpio::{AnyIOPin, AnyInputPin, AnyOutputPin, PinDriver, Pull},
    peripherals::Peripherals,
};
use esp_idf_svc::{
    eventloop::EspSystemEventLoop,
    nvs::{EspDefaultNvsPartition, EspNvs, NvsDefault},
    wifi::{BlockingWifi, ClientConfiguration, Configuration, EspWifi},
};
use log::info;
use std::time::Duration;

mod captive_portal;
mod config;
mod door_controller;
mod encoder;
mod motor;
mod mqtt_client;

use captive_portal::{needs_provisioning, run_captive_portal};
use config::Config;
use door_controller::DoorController;
use encoder::Encoder;
use motor::MotorDriver;
use mqtt_client::{json_str_field, MqttCommand, MqttHandle, TELEMETRY_EVERY_N_TICKS};

// ─────────────────────────────────────────────────────────────────────────────
// Entry point
// ─────────────────────────────────────────────────────────────────────────────

fn main() -> Result<()> {
    // Required by esp-idf-svc — patches std hooks
    esp_idf_svc::sys::link_patches();

    // Initialise the ESP-IDF logger (output via UART0 / USB-JTAG)
    esp_idf_svc::log::EspLogger::initialize_default();

    info!("=== Open Sesame booting ===");

    let peripherals = Peripherals::take()?;
    let sysloop = EspSystemEventLoop::take()?;
    let nvs_partition = EspDefaultNvsPartition::take()?;

    // Open (or create) our NVS namespace
    let mut nvs: EspNvs<NvsDefault> = EspNvs::new(nvs_partition.clone(), "door_cfg", true)?;

    // ── Status LED (on at boot) ─────────────────────────────────────────────
    let mut led = PinDriver::output(unsafe { AnyIOPin::new(config::PIN_LED) })?;
    led.set_low()?;

    // ── Factory reset ─────────────────────────────────────────────────────────
    {
        let mut reset_btn = PinDriver::input(unsafe { AnyIOPin::new(config::PIN_FACTORY_RESET) })?;
        reset_btn.set_pull(Pull::Up)?;

        if reset_btn.is_low() {
            info!("Reset button held — waiting 5 s for factory reset confirmation...");
            std::thread::sleep(Duration::from_secs(5));
            if reset_btn.is_low() {
                info!("Factory reset triggered! Erasing config.");
                Config::erase(&mut nvs)?;
                info!("Config erased. Restarting into captive portal...");
                std::thread::sleep(Duration::from_millis(200));
                esp_idf_hal::reset::restart();
            }
            info!("Button released — skipping factory reset.");
        }
    }

    // ── Provisioning ──────────────────────────────────────────────────────────
    if needs_provisioning(&nvs) {
        info!("No config found — starting captive portal.");

        // Flash LED in a background thread while provisioning
        let flash_active = std::sync::Arc::new(std::sync::atomic::AtomicBool::new(true));
        let flash_flag = flash_active.clone();
        std::thread::spawn(move || {
            while flash_flag.load(std::sync::atomic::Ordering::Relaxed) {
                let _ = led.toggle();
                std::thread::sleep(Duration::from_millis(500));
            }
            let _ = led.set_high();
        });

        run_captive_portal(peripherals.modem, sysloop.clone(), &mut nvs)?;
        flash_active.store(false, std::sync::atomic::Ordering::Relaxed);

        info!("Config saved. Restarting...");
        std::thread::sleep(Duration::from_millis(500));
        esp_idf_hal::reset::restart();
    }

    let config = Config::load(&nvs);
    info!(
        "Config loaded: ssid={}, broker={}",
        config.wifi_ssid, config.mqtt_broker
    );

    // ── WiFi ──────────────────────────────────────────────────────────────────
    info!("Connecting to Wifi");
    let wifi = connect_wifi(peripherals.modem, sysloop.clone(), &config)?;
    info!("WiFi connected");

    // ── Encoder (hardware PCNT) ─────────────────────────────────────────────
    info!("Initializing Encoder");
    let encoder = Encoder::new(
        peripherals.pcnt0,
        unsafe { AnyInputPin::new(config::PIN_ENC_A) },
        unsafe { AnyInputPin::new(config::PIN_ENC_B) },
    )?;

    // ── Motor driver ──────────────────────────────────────────────────────────
    info!("Initializing Motor Driver");
    let motor = MotorDriver::new(
        peripherals.ledc.timer0,
        peripherals.ledc.channel0,
        unsafe { AnyOutputPin::new(config::PIN_MOTOR_IN1) },
        unsafe { AnyOutputPin::new(config::PIN_MOTOR_IN2) },
        unsafe { AnyOutputPin::new(config::PIN_MOTOR_NSLP) },
        config::MOTOR_MODE,
    )?;

    // ── MQTT ──────────────────────────────────────────────────────────────────
    info!("Starting MQTT Client");
    let (mut mqtt, connection) = MqttHandle::connect(&config)?;
    let cmd_rx = mqtt.spawn_receive_thread(connection);

    // ── Door controller ───────────────────────────────────────────────────────
    info!("Starting Door Controller");
    let mut door = DoorController::new(motor, encoder);

    info!("Entering control loop");

    let mut tick_count: u32 = 0;

    loop {
        // ── Control tick ──────────────────────────────────────────────────────
        door.tick()?;

        // ── Periodic state publish (state + position + velocity) ──────────────
        tick_count = tick_count.wrapping_add(1);
        if tick_count % TELEMETRY_EVERY_N_TICKS == 0 {
            mqtt.publish_state(door.state(), door.position(), door.velocity(), door.power());
        }

        // ── MQTT commands ─────────────────────────────────────────────────────
        if let Ok(cmd) = cmd_rx.try_recv() {
            match cmd {
                MqttCommand::Connected => {
                    info!("MQTT connected — subscribing and publishing discovery");
                    mqtt.subscribe_commands()?;
                    mqtt.publish_ha_discovery(&config.mqtt_client_id);
                }
                MqttCommand::ConfigUpdate(json) => {
                    info!("Remote config update received: {}", json);
                    apply_remote_config(&json, &mut nvs)?;
                    info!("New config saved. Restarting...");
                    std::thread::sleep(Duration::from_millis(500));
                    drop(wifi); // graceful disconnect
                    esp_idf_hal::reset::restart();
                }
                MqttCommand::ResetEncoder => {
                    info!("Encoder reset requested via MQTT");
                    door.reset_position();
                }
                MqttCommand::SetPower { power } => {
                    info!("MQTT set power: {}", power);
                    door.set_power(power)?;
                }
                MqttCommand::Open => door.drive_open(),
                MqttCommand::Close => door.drive_close(),
                MqttCommand::Stop => door.stop()?,
            }
        }

        std::thread::sleep(DoorController::loop_period());
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// WiFi helper
// ─────────────────────────────────────────────────────────────────────────────

fn connect_wifi(
    modem: impl esp_idf_hal::peripheral::Peripheral<P = esp_idf_hal::modem::Modem> + 'static,
    sysloop: EspSystemEventLoop,
    cfg: &Config,
) -> Result<BlockingWifi<EspWifi<'static>>> {
    let mut wifi = BlockingWifi::wrap(EspWifi::new(modem, sysloop.clone(), None)?, sysloop)?;

    wifi.set_configuration(&Configuration::Client(ClientConfiguration {
        ssid: cfg
            .wifi_ssid
            .as_str()
            .try_into()
            .map_err(|_| anyhow::anyhow!("SSID too long"))?,
        password: cfg
            .wifi_password
            .as_str()
            .try_into()
            .map_err(|_| anyhow::anyhow!("WiFi password too long"))?,
        ..Default::default()
    }))?;

    wifi.start()?;

    // Retry up to 3 times before giving up and launching the captive portal
    for attempt in 1..=3 {
        info!("WiFi connect attempt {attempt}/3...");
        match wifi.connect() {
            Ok(_) => break,
            Err(e) if attempt == 3 => {
                return Err(anyhow::anyhow!("WiFi failed after 3 attempts: {:?}", e));
            }
            Err(e) => {
                warn_wifi(e);
                std::thread::sleep(Duration::from_secs(2));
            }
        }
    }

    wifi.wait_netif_up()?;
    Ok(wifi)
}

fn warn_wifi(e: impl std::fmt::Debug) {
    log::warn!("WiFi connect error: {:?}", e);
}

// ─────────────────────────────────────────────────────────────────────────────
// Remote config update via MQTT
// ─────────────────────────────────────────────────────────────────────────────

/// Parse a partial JSON config payload and update only the supplied keys.
///
/// Example payload:
/// ```json
/// {"mqtt_broker":"mqtt://192.168.1.200:1883","mqtt_id":"door-v2"}
/// ```
fn apply_remote_config(json: &str, nvs: &mut EspNvs<NvsDefault>) -> Result<()> {
    // Load current config so we don't overwrite fields not in the payload
    let mut cfg = Config::load(nvs);

    if let Some(v) = json_str_field(json, "ssid") {
        cfg.wifi_ssid = v.into();
    }
    if let Some(v) = json_str_field(json, "wifi_pass") {
        cfg.wifi_password = v.into();
    }
    if let Some(v) = json_str_field(json, "mqtt_broker") {
        cfg.mqtt_broker = v.into();
    }
    if let Some(v) = json_str_field(json, "mqtt_id") {
        cfg.mqtt_client_id = v.into();
    }
    if let Some(v) = json_str_field(json, "mqtt_user") {
        cfg.mqtt_username = v.into();
    }
    if let Some(v) = json_str_field(json, "mqtt_pass") {
        cfg.mqtt_password = v.into();
    }

    cfg.save(nvs)?;
    Ok(())
}
