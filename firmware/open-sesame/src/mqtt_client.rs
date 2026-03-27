// src/mqtt_client.rs
//
// MQTT client wrapper.

use crate::config::Config;
use crate::door_controller::DoorState;
use anyhow::Result;
use esp_idf_svc::mqtt::client::{
    EspMqttClient, EspMqttConnection, EventPayload, MqttClientConfiguration, QoS,
};
use log::{info, warn};

// Subscribed Topics
pub const TOPIC_CONFIG: &str = "set_config";
pub const TOPIC_ENCODER_RESET: &str = "reset_encoder";
pub const TOPIC_SET_STATE: &str = "set_state";
pub const TOPIC_COMMAND: &str = "command"; // payload: "open" | "close" | "stop"

// Published Topics
pub const TOPIC_STATE: &str = "state";

// ─────────────────────────────────────────────────────────────────────────────
// Commands routed from MQTT to the main loop
// ─────────────────────────────────────────────────────────────────────────────

pub enum MqttCommand {
    /// Broker connection established — subscribe and publish discovery.
    Connected,
    /// JSON payload with partial device config — apply and reboot.
    ConfigUpdate(String),
    /// Zero the encoder position counter.
    ResetEncoder,
    /// Raw motor power override (-255 to 255, 0 = stop).
    SetPower { power: i32 },
    /// Drive door to the open endstop.
    Open,
    /// Drive door to the closed endstop.
    Close,
    /// Stop motor and clear any remote target.
    Stop,
}

// How many control ticks between telemetry publishes (10 ms × 50 = 500 ms).
pub const TELEMETRY_EVERY_N_TICKS: u32 = 50;

// ─────────────────────────────────────────────────────────────────────────────
// MqttHandle
// ─────────────────────────────────────────────────────────────────────────────

pub struct MqttHandle {
    client: EspMqttClient<'static>,
    topic_prefix: String,
}

impl MqttHandle {
    /// Create the MQTT client, start the background receive thread, and publish
    /// the Home Assistant discovery payload.
    pub fn connect(cfg: &Config) -> Result<(Self, EspMqttConnection)> {
        let mqtt_cfg = MqttClientConfiguration {
            client_id: Some(cfg.mqtt_client_id.as_str()),
            username: if cfg.mqtt_username.is_empty() {
                None
            } else {
                Some(cfg.mqtt_username.as_str())
            },
            password: if cfg.mqtt_password.is_empty() {
                None
            } else {
                Some(cfg.mqtt_password.as_str())
            },
            keep_alive_interval: Some(std::time::Duration::from_secs(15)),
            ..Default::default()
        };

        let (client, connection) = EspMqttClient::new(&cfg.mqtt_broker, &mqtt_cfg)?;
        info!("MQTT connecting to {}", cfg.mqtt_broker);

        let topic_prefix = cfg.mqtt_client_id.clone();
        Ok((
            Self {
                client,
                topic_prefix,
            },
            connection,
        ))
    }

    /// Spawn the receive thread. Must be called once after `connect()`.
    /// The thread keeps the connection alive and routes incoming messages
    /// back to the main loop as typed `MqttCommand` values.
    pub fn spawn_receive_thread(
        &mut self,
        mut connection: EspMqttConnection,
    ) -> std::sync::mpsc::Receiver<MqttCommand> {
        let (tx, rx) = std::sync::mpsc::channel::<MqttCommand>();

        let prefix = self.topic_prefix.clone();

        std::thread::Builder::new()
            .name("mqtt-rx".into())
            .stack_size(4096)
            .spawn(move || loop {
                match connection.next() {
                    Ok(msg) => match msg.payload() {
                        EventPayload::Received { topic, data, .. } => {
                            let suffix = match topic {
                                Some(t) => {
                                    t.strip_prefix(&prefix).and_then(|s| s.strip_prefix("/"))
                                }
                                None => None,
                            };

                            let cmd = match suffix {
                                Some(TOPIC_CONFIG) => std::str::from_utf8(data)
                                    .ok()
                                    .map(|s| MqttCommand::ConfigUpdate(s.to_string())),
                                Some(TOPIC_ENCODER_RESET) => Some(MqttCommand::ResetEncoder),
                                Some(TOPIC_SET_STATE) => {
                                    std::str::from_utf8(data).ok().and_then(|json| {
                                        json_num_field(json, "power")
                                            .map(|p| MqttCommand::SetPower { power: p })
                                    })
                                }
                                Some(TOPIC_COMMAND) => {
                                    std::str::from_utf8(data).ok().and_then(|s| match s {
                                        "open" => Some(MqttCommand::Open),
                                        "close" => Some(MqttCommand::Close),
                                        "stop" => Some(MqttCommand::Stop),
                                        other => {
                                            warn!("Unknown command: {}", other);
                                            None
                                        }
                                    })
                                }
                                _ => None,
                            };

                            if let Some(cmd) = cmd {
                                let _ = tx.send(cmd);
                            }
                        }
                        EventPayload::Connected(_) => {
                            info!("MQTT connected");
                            let _ = tx.send(MqttCommand::Connected);
                        }
                        EventPayload::Disconnected => warn!("MQTT disconnected"),
                        _ => {}
                    },
                    Err(e) => {
                        warn!("MQTT rx error: {:?}", e);
                        break;
                    }
                }
            })
            .expect("failed to spawn mqtt-rx thread");

        rx
    }

    // ── Publishing ────────────────────────────────────────────────────────────

    /// Publish door state, position, and velocity as a single JSON payload.
    pub fn publish_state(&mut self, state: &DoorState, position: i32, velocity: f32, power: i32) {
        let full_topic = format!("{}/{}", self.topic_prefix, TOPIC_STATE);
        let payload = format!(
            r#"{{"state":"{}","position":{},"velocity":{:.2},"power":{}}}"#,
            state.as_str(),
            position,
            velocity,
            power
        );

        if let Err(e) = self.client.publish(
            &full_topic,
            QoS::AtLeastOnce,
            true, // retain
            payload.as_bytes(),
        ) {
            warn!("Failed to publish state: {:?}", e);
        }
    }

    /// Publish Home Assistant MQTT Discovery config (call once after connecting).
    pub fn publish_ha_discovery(&mut self, client_id: &str) {
        // Door state binary sensor
        let sensor_topic = format!("homeassistant/sensor/{}/config", client_id);
        let sensor_payload = format!(
            r#"{{
                "name": "Sliding Door",
                "unique_id": "{id}",
                "state_topic": "{state}",
                "device_class": "door",
                "device": {{
                    "identifiers": ["{id}"],
                    "name": "Open Sesame",
                    "model": "ESP32-S3 + DRV8874",
                    "manufacturer": "DIY"
                }}
            }}"#,
            id = client_id,
            state = &format!("{}/{}", self.topic_prefix, TOPIC_STATE)
        );

        if let Err(e) = self.client.publish(
            &sensor_topic,
            QoS::AtLeastOnce,
            true,
            sensor_payload.as_bytes(),
        ) {
            warn!("HA discovery publish failed: {:?}", e);
        } else {
            info!("HA discovery published");
        }
    }

    /// Subscribe to all command topics (config updates, encoder reset, etc.).
    pub fn subscribe_commands(&mut self) -> Result<()> {
        for topic in [
            TOPIC_CONFIG,
            TOPIC_ENCODER_RESET,
            TOPIC_SET_STATE,
            TOPIC_COMMAND,
        ] {
            let full_topic = format!("{}/{}", self.topic_prefix, topic);
            self.client.subscribe(&full_topic, QoS::AtLeastOnce)?;
            info!("Subscribed to {}", topic);
        }
        Ok(())
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Simple JSON field extractor (no external crate needed for our tiny payloads)
// ─────────────────────────────────────────────────────────────────────────────

/// Extract an unquoted numeric value of `key` from a flat JSON object.
/// e.g. `json_num_field(r#"{"power":100}"#, "power")` → Some(100)
pub fn json_num_field(json: &str, key: &str) -> Option<i32> {
    let needle = format!("\"{}\"", key);
    let start = json.find(&needle)? + needle.len();
    let after_colon = json[start..].find(':')? + start + 1;
    let trimmed = json[after_colon..].trim_start();
    let end = trimmed
        .find(|c: char| !c.is_ascii_digit() && c != '-')
        .unwrap_or(trimmed.len());
    trimmed[..end].parse::<i32>().ok()
}

/// Extract the string value of `key` from a flat JSON object.
/// e.g. `json_str_field(r#"{"mqtt_broker":"mqtt://x:1883"}"#, "mqtt_broker")`
pub fn json_str_field<'a>(json: &'a str, key: &str) -> Option<&'a str> {
    let needle = format!("\"{}\"", key);
    let start = json.find(&needle)? + needle.len();
    let after_colon = json[start..].find(':')? + start + 1;
    let trimmed = json[after_colon..].trim_start();
    if trimmed.starts_with('"') {
        let inner = &trimmed[1..];
        let end = inner.find('"')?;
        Some(&inner[..end])
    } else {
        None
    }
}
