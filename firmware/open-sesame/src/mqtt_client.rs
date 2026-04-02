// src/mqtt_client.rs
//
// MQTT client wrapper.

use crate::{config::Config, door_controller::DoorController};
use anyhow::Result;
use esp_idf_svc::mqtt::client::{
    EspMqttClient, EspMqttConnection, EventPayload, MqttClientConfiguration, QoS,
};
use log::{info, warn};

// Subscribed Topics
pub const TOPIC_CONFIG: &str = "config/set";
pub const TOPIC_COMMAND: &str = "command"; // payload: "open" | "close" | "stop" | "calibrate" | "home"
pub const TOPIC_SET_POSITION: &str = "position/set";

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
    /// Drive door to the open endstop.
    Open,
    /// Drive door to the closed endstop.
    Close,
    /// Set door to position from 0-100
    Position(i32),
    /// Stop motor and clear any remote target.
    Stop,
    /// Auto-detect endstops by driving to each end and watching for stall.
    Calibrate,
    /// Perform home on door requested.
    Home,
}

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
                                Some(TOPIC_SET_POSITION) => std::str::from_utf8(data)
                                    .ok()
                                    .and_then(|s| s.parse::<i32>().ok())
                                    .map(|p| MqttCommand::Position(p)),
                                Some(TOPIC_COMMAND) => {
                                    std::str::from_utf8(data).ok().and_then(|s| match s {
                                        "open" => Some(MqttCommand::Open),
                                        "close" => Some(MqttCommand::Close),
                                        "stop" => Some(MqttCommand::Stop),
                                        "calibrate" => Some(MqttCommand::Calibrate),
                                        "home" => Some(MqttCommand::Home),
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
    /// `pos_pct` (0–100) is included for the HA cover position template.
    pub fn publish_state(&mut self, door: &DoorController) {
        let position = door.position();
        let state = door.state();
        let velocity = door.velocity();
        let power = door.power();
        let pos_pct = door.position_pct();

        let full_topic = format!("{}/{}", self.topic_prefix, TOPIC_STATE);
        let payload = format!(
            r#"{{"state":"{}","position":{},"pos_pct":{},"velocity":{:.2},"power":{}}}"#,
            state.as_str(),
            position,
            pos_pct,
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
        let state_topic = format!("{}/{}", self.topic_prefix, TOPIC_STATE);
        let command_topic = format!("{}/{}", self.topic_prefix, TOPIC_COMMAND);
        let set_position_topic = format!("{}/{}", self.topic_prefix, TOPIC_SET_POSITION);

        let device = format!(
            r#"{{
                "identifiers":["{id}"],
                "name":"Open Sesame",
                "model":"Open Sesame v0.1",
                "manufacturer":"croggero"
            }}"#,
            id = client_id
        );

        // ── Cover (open / close / stop) ───────────────────────────────────────
        let cover_topic = format!("homeassistant/cover/{}/config", client_id);
        let cover_payload = format!(
            r#"{{
                "name":"Sliding Door",
                "unique_id":"{id}_cover",
                "device_class":"door",
                "state_topic":"{state}",
                "value_template":"{{{{value_json.state}}}}",
                "position_topic":"{state}",
                "position_template":"{{{{value_json.pos_pct}}}}",
                "set_position_topic":"{set_pos}",
                "command_topic":"{cmd}",
                "payload_open":"open",
                "payload_close":"close",
                "payload_stop":"stop",
                "state_open":"open",
                "state_closed":"closed",
                "state_opening":"opening",
                "state_closing":"closing",
                "device":{device}
            }}"#,
            id = client_id,
            state = state_topic,
            cmd = command_topic,
            set_pos = set_position_topic,
            device = device,
        );

        // ── Button: calibrate ─────────────────────────────────────────────────
        let calibrate_topic = format!("homeassistant/button/{}_calibrate/config", client_id);
        let calibrate_payload = format!(
            r#"{{
                "name":"Calibrate Door",
                "unique_id":"{id}_calibrate",
                "command_topic":"{cmd}",
                "payload_press":"calibrate",
                "entity_category":"config",
                "device":{device}
            }}"#,
            id = client_id,
            cmd = command_topic,
            device = device,
        );

        // ── Button: home ─────────────────────────────────────────────────
        let home_topic = format!("homeassistant/button/{}_home/config", client_id);
        let home_payload = format!(
            r#"{{
                "name":"Home Door",
                "unique_id":"{id}_home",
                "command_topic":"{cmd}",
                "payload_press":"home",
                "entity_category":"config",
                "device":{device}
            }}"#,
            id = client_id,
            cmd = command_topic,
            device = device,
        );

        for (topic, payload) in [
            (cover_topic, cover_payload),
            (calibrate_topic, calibrate_payload),
            (home_topic, home_payload),
        ] {
            if let Err(e) = self
                .client
                .publish(&topic, QoS::AtLeastOnce, true, payload.as_bytes())
            {
                warn!("HA discovery publish failed on {}: {:?}", topic, e);
            } else {
                info!("HA discovery published: {}", topic);
            }
        }
    }

    /// Subscribe to all command topics (config updates, encoder reset, etc.).
    pub fn subscribe_commands(&mut self) -> Result<()> {
        for topic in [TOPIC_CONFIG, TOPIC_COMMAND, TOPIC_SET_POSITION] {
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

// /// Extract an unquoted numeric value of `key` from a flat JSON object.
// /// e.g. `json_num_field(r#"{"power":100}"#, "power")` → Some(100)
// pub fn json_num_field(json: &str, key: &str) -> Option<i32> {
//     let needle = format!("\"{}\"", key);
//     let start = json.find(&needle)? + needle.len();
//     let after_colon = json[start..].find(':')? + start + 1;
//     let trimmed = json[after_colon..].trim_start();
//     let end = trimmed
//         .find(|c: char| !c.is_ascii_digit() && c != '-')
//         .unwrap_or(trimmed.len());
//     trimmed[..end].parse::<i32>().ok()
// }

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
