// src/config.rs
//
// Hardware pin assignments, tuning constants, and device configuration.
//
// Static constants come from config.toml (compiled by build.rs).
// Runtime device config (WiFi, MQTT) is stored in NVS with secrets.toml defaults.

// ─────────────────────────────────────────────────────────────────────────────
// Build-time constants from config.toml
// ─────────────────────────────────────────────────────────────────────────────

include!(concat!(env!("OUT_DIR"), "/config_generated.rs"));

// ─────────────────────────────────────────────────────────────────────────────
// Runtime device config (NVS-backed)
// ─────────────────────────────────────────────────────────────────────────────

use anyhow::Result;
use esp_idf_svc::nvs::{EspNvs, NvsDefault};

/// Compile-time defaults from secrets.toml (empty string if not present).
macro_rules! secret {
    ($name:expr) => {
        option_env!($name).unwrap_or("")
    };
}

#[derive(Debug, Clone)]
pub struct Config {
    pub wifi_ssid: String,
    pub wifi_password: String,
    pub mqtt_broker: String, // e.g. "mqtt://192.168.1.100:1883"
    pub mqtt_client_id: String,
    pub mqtt_username: String, // empty = no auth
    pub mqtt_password: String,
    pub opening_counts: i32,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            wifi_ssid: secret!("DEF_WIFI_SSID").into(),
            wifi_password: secret!("DEF_WIFI_PASSWORD").into(),
            mqtt_broker: secret!("DEF_MQTT_BROKER").into(),
            mqtt_client_id: secret!("DEF_MQTT_CLIENT_ID").into(),
            mqtt_username: secret!("DEF_MQTT_USERNAME").into(),
            mqtt_password: secret!("DEF_MQTT_PASSWORD").into(),
            opening_counts: 0,
        }
    }
}

impl Config {
    /// Load all fields from NVS, falling back to compile-time defaults from
    /// secrets.toml for any missing keys.
    pub fn load(nvs: &EspNvs<NvsDefault>) -> Self {
        let defaults = Self::default();

        fn read_string(nvs: &EspNvs<NvsDefault>, key: &str, fallback: &str) -> String {
            let mut buf = [0u8; 128];
            nvs.get_str(key, &mut buf)
                .ok()
                .flatten()
                .filter(|s: &&str| !s.is_empty())
                .map(|s| s.to_string())
                .unwrap_or_else(|| fallback.to_string())
        }

        fn read_i32(nvs: &EspNvs<NvsDefault>, key: &str, fallback: i32) -> i32 {
            nvs.get_i32(key).ok().flatten().unwrap_or_else(|| fallback)
        }

        Self {
            wifi_ssid: read_string(nvs, "wifi_ssid", &defaults.wifi_ssid),
            wifi_password: read_string(nvs, "wifi_pass", &defaults.wifi_password),
            mqtt_broker: read_string(nvs, "mqtt_broker", &defaults.mqtt_broker),
            mqtt_client_id: read_string(nvs, "mqtt_id", &defaults.mqtt_client_id),
            mqtt_username: read_string(nvs, "mqtt_user", &defaults.mqtt_username),
            mqtt_password: read_string(nvs, "mqtt_pass", &defaults.mqtt_password),
            opening_counts: read_i32(nvs, "opening_counts", defaults.opening_counts),
        }
    }

    /// Persist all fields to NVS.
    pub fn save(&self, nvs: &mut EspNvs<NvsDefault>) -> Result<()> {
        nvs.set_str("wifi_ssid", &self.wifi_ssid)?;
        nvs.set_str("wifi_pass", &self.wifi_password)?;
        nvs.set_str("mqtt_broker", &self.mqtt_broker)?;
        nvs.set_str("mqtt_id", &self.mqtt_client_id)?;
        nvs.set_str("mqtt_user", &self.mqtt_username)?;
        nvs.set_str("mqtt_pass", &self.mqtt_password)?;
        nvs.set_i32("opening_counts", self.opening_counts)?;
        Ok(())
    }

    /// Erase all config keys from NVS (factory reset).
    pub fn erase(nvs: &mut EspNvs<NvsDefault>) -> Result<()> {
        for key in [
            "wifi_ssid",
            "wifi_pass",
            "mqtt_broker",
            "mqtt_id",
            "mqtt_user",
            "mqtt_pass",
            "opening_counts",
        ] {
            let _ = nvs.remove(key); // ignore "not found" errors
        }
        Ok(())
    }
}
