// src/captive_portal.rs
//
// WiFi provisioning via a SoftAP captive portal.
//
// Flow:
//   1. ESP32 boots as an open WiFi AP ("OpenSesame-Setup")
//   2. User's phone connects and is redirected to http://192.168.4.1/
//   3. User fills in WiFi + MQTT credentials and submits
//   4. Credentials are saved to NVS
//   5. Function returns — caller should call restart()

use crate::config::Config;

use anyhow::Result;
use embedded_svc::http::Method;
use esp_idf_hal::peripheral::Peripheral;
use esp_idf_svc::{
    eventloop::EspSystemEventLoop,
    http::server::{Configuration as HttpConfig, EspHttpServer},
    nvs::{EspNvs, NvsDefault},
    wifi::{AccessPointConfiguration, BlockingWifi, Configuration, EspWifi},
};
use log::info;
use std::sync::{Arc, Mutex};

// ─────────────────────────────────────────────────────────────────────────────
// HTML pages
// ─────────────────────────────────────────────────────────────────────────────

const HTML_FORM: &str = r#"<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Open Sesame Setup</title>
  <style>
    *, *::before, *::after { box-sizing: border-box; margin: 0; padding: 0; }
    body {
      font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
      background: #0f172a; color: #e2e8f0;
      min-height: 100vh; display: flex;
      align-items: center; justify-content: center; padding: 1rem;
    }
    .card {
      background: #1e293b; border: 1px solid #334155;
      border-radius: 12px; padding: 2rem; width: 100%; max-width: 420px;
      box-shadow: 0 20px 60px rgba(0,0,0,.5);
    }
    .logo { text-align:center; margin-bottom:1.5rem; }
    .logo svg { width:48px; height:48px; color:#38bdf8; }
    h1 { font-size:1.4rem; font-weight:600; text-align:center; margin-bottom:.25rem; color:#f1f5f9; }
    .subtitle { text-align:center; font-size:.85rem; color:#94a3b8; margin-bottom:1.75rem; }
    .section-title {
      font-size:.7rem; font-weight:600; letter-spacing:.1em;
      text-transform:uppercase; color:#38bdf8; margin-bottom:.75rem; margin-top:1.25rem;
    }
    label { display:block; font-size:.8rem; color:#94a3b8; margin-bottom:.3rem; }
    input {
      display:block; width:100%; padding:.6rem .75rem;
      background:#0f172a; border:1px solid #334155; border-radius:7px;
      color:#f1f5f9; font-size:.9rem; margin-bottom:.9rem;
      transition:border-color .15s; outline:none;
    }
    input:focus { border-color:#38bdf8; }
    input::placeholder { color:#475569; }
    hr { border:none; border-top:1px solid #334155; margin:1rem 0; }
    button {
      width:100%; padding:.75rem; background:#0284c7; color:#fff;
      border:none; border-radius:8px; font-size:.95rem; font-weight:600;
      cursor:pointer; margin-top:.5rem; transition:background .15s;
    }
    button:hover  { background:#0369a1; }
    button:active { background:#075985; }
    .note { font-size:.75rem; color:#64748b; text-align:center; margin-top:1rem; }
  </style>
</head>
<body>
<div class="card">
  <div class="logo">
    <svg viewBox="0 0 24 24" fill="none" stroke="currentColor"
         stroke-width="1.5" stroke-linecap="round" stroke-linejoin="round">
      <rect x="2" y="3" width="20" height="18" rx="2"/>
      <line x1="12" y1="3" x2="12" y2="21"/>
      <polyline points="8 10 5 12 8 14"/>
    </svg>
  </div>
  <h1>Open Sesame Setup</h1>
  <p class="subtitle">Configure WiFi and MQTT connection</p>
  <form method="POST" action="/save">
    <div class="section-title">WiFi</div>
    <label for="ssid">Network Name (SSID)</label>
    <input type="text"     id="ssid"      name="ssid"      placeholder="MyHomeNetwork" required autocomplete="off">
    <label for="wpass">Password</label>
    <input type="password" id="wpass"     name="wpass"     placeholder="••••••••" autocomplete="off">
    <hr>
    <div class="section-title">MQTT Broker</div>
    <label for="broker">Broker URL</label>
    <input type="text"     id="broker"    name="broker"    placeholder="mqtt://192.168.1.100:1883" required>
    <label for="client_id">Client ID</label>
    <input type="text"     id="client_id" name="client_id" placeholder="sliding-door" value="sliding-door">
    <label for="muser">Username <span style="color:#475569">(optional)</span></label>
    <input type="text"     id="muser"     name="muser"     placeholder="homeassistant" autocomplete="off">
    <label for="mpass">Password <span style="color:#475569">(optional)</span></label>
    <input type="password" id="mpass"     name="mpass"     placeholder="••••••••" autocomplete="off">
    <button type="submit">Save &amp; Restart</button>
  </form>
  <p class="note">The device will restart automatically after saving.</p>
</div>
</body></html>
"#;

const HTML_SUCCESS: &str = r#"<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>Saved!</title>
  <style>
    body { font-family:sans-serif; background:#0f172a; color:#e2e8f0;
           min-height:100vh; display:flex; align-items:center; justify-content:center; }
    .card { background:#1e293b; border:1px solid #334155; border-radius:12px;
            padding:2.5rem 2rem; text-align:center; max-width:360px; }
    .icon { font-size:3rem; margin-bottom:1rem; }
    h1 { font-size:1.3rem; margin-bottom:.5rem; color:#f1f5f9; }
    p  { color:#94a3b8; font-size:.9rem; line-height:1.5; }
    .badge { display:inline-block; margin-top:1.25rem; padding:.3rem .9rem;
             background:#022c22; color:#4ade80; border:1px solid #166534;
             border-radius:99px; font-size:.8rem; font-weight:600; }
  </style>
</head>
<body>
<div class="card">
  <div class="icon">✅</div>
  <h1>Configuration Saved</h1>
  <p>Your settings have been stored.<br>
     Open Sesame is restarting and will join your network shortly.</p>
  <div class="badge">You can close this page</div>
</div>
</body></html>
"#;

const HTML_ERROR: &str = r#"<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>Error</title>
  <style>
    body { font-family:sans-serif; background:#0f172a; color:#e2e8f0;
           min-height:100vh; display:flex; align-items:center; justify-content:center; }
    .card { background:#1e293b; border:1px solid #7f1d1d; border-radius:12px;
            padding:2rem; text-align:center; max-width:360px; }
    .icon { font-size:3rem; margin-bottom:1rem; }
    h1 { color:#fca5a5; margin-bottom:.5rem; }
    p  { color:#94a3b8; font-size:.9rem; }
    a  { color:#38bdf8; }
  </style>
</head>
<body>
<div class="card">
  <div class="icon">⚠️</div>
  <h1>Missing Required Fields</h1>
  <p>SSID and MQTT Broker URL are required.<br>
     <a href="/">Go back and try again</a>.</p>
</div>
</body></html>
"#;

// ─────────────────────────────────────────────────────────────────────────────
// URL / form helpers
// ─────────────────────────────────────────────────────────────────────────────

fn url_decode(input: &str) -> String {
    let mut out = String::with_capacity(input.len());
    let mut chars = input.chars().peekable();
    while let Some(c) = chars.next() {
        match c {
            '+' => out.push(' '),
            '%' => {
                let h1 = chars.next().unwrap_or('0');
                let h2 = chars.next().unwrap_or('0');
                if let Ok(b) = u8::from_str_radix(&format!("{h1}{h2}"), 16) {
                    out.push(b as char);
                }
            }
            _ => out.push(c),
        }
    }
    out
}

fn parse_form(body: &str) -> Vec<(String, String)> {
    body.split('&')
        .filter_map(|pair| {
            let mut it = pair.splitn(2, '=');
            let key = it.next().map(url_decode)?;
            let val = it.next().map(url_decode).unwrap_or_default();
            Some((key, val))
        })
        .collect()
}

fn field<'a>(fields: &'a [(String, String)], name: &str) -> &'a str {
    fields
        .iter()
        .find(|(k, _)| k == name)
        .map(|(_, v)| v.as_str())
        .unwrap_or("")
}

// ─────────────────────────────────────────────────────────────────────────────
// Public API
// ─────────────────────────────────────────────────────────────────────────────

/// Returns true when no WiFi SSID is available (neither in NVS nor secrets.toml).
pub fn needs_provisioning(nvs: &EspNvs<NvsDefault>) -> bool {
    Config::load(nvs).wifi_ssid.is_empty()
}

/// Start the captive portal and block until credentials are submitted.
/// Saves config to NVS before returning. Caller must call `restart()`.
pub fn run_captive_portal(
    modem: impl Peripheral<P = esp_idf_hal::modem::Modem> + 'static,
    sysloop: EspSystemEventLoop,
    nvs: &mut EspNvs<NvsDefault>,
) -> Result<()> {
    info!("Starting captive portal AP...");

    // ── SoftAP ───────────────────────────────────────────────────────────────
    let mut wifi = BlockingWifi::wrap(EspWifi::new(modem, sysloop.clone(), None)?, sysloop)?;

    wifi.set_configuration(&Configuration::AccessPoint(AccessPointConfiguration {
        ssid: "OpenSesame-Setup".try_into().unwrap(),
        password: "".try_into().unwrap(), // open network
        channel: 6,
        max_connections: 4,
        ..Default::default()
    }))?;

    wifi.start()?;
    info!("AP up — connect to 'OpenSesame-Setup', visit http://192.168.4.1");

    // ── Shared submission slot ────────────────────────────────────────────────
    let submitted: Arc<Mutex<Option<Config>>> = Arc::new(Mutex::new(None));
    let sub_get = submitted.clone();
    let sub_post = submitted.clone();

    // ── HTTP server ───────────────────────────────────────────────────────────
    let mut server = EspHttpServer::new(&HttpConfig::default())?;

    // GET /
    server.fn_handler("/", Method::Get, move |req| {
        let done = sub_get.lock().unwrap().is_some();
        let body = if done { HTML_SUCCESS } else { HTML_FORM };
        let mut r = req.into_response(
            200,
            Some("OK"),
            &[("Content-Type", "text/html; charset=utf-8")],
        )?;
        r.write(body.as_bytes())?;
        Ok::<(), anyhow::Error>(())
    })?;

    // Captive-portal detection endpoints (iOS, Android, Windows)
    for path in [
        "/generate_204",
        "/hotspot-detect.html",
        "/ncsi.txt",
        "/connecttest.txt",
    ] {
        server.fn_handler(path, Method::Get, |req| {
            let mut r =
                req.into_response(302, Some("Found"), &[("Location", "http://192.168.4.1/")])?;
            r.write(b"")?;
            Ok::<(), anyhow::Error>(())
        })?;
    }

    // POST /save
    server.fn_handler("/save", Method::Post, move |mut req| {
        let mut buf = [0u8; 1024];
        let n = req.read(&mut buf).unwrap_or(0);
        let body = std::str::from_utf8(&buf[..n]).unwrap_or("");
        let fields = parse_form(body);

        let ssid = field(&fields, "ssid").trim().to_string();
        let broker = field(&fields, "broker").trim().to_string();

        let (html, status) = if ssid.is_empty() || broker.is_empty() {
            (HTML_ERROR, 400u16)
        } else {
            let cfg = Config {
                wifi_ssid: ssid,
                wifi_password: field(&fields, "wpass").to_string(),
                mqtt_broker: broker,
                mqtt_client_id: {
                    let id = field(&fields, "client_id").trim().to_string();
                    if id.is_empty() {
                        "sliding-door".into()
                    } else {
                        id
                    }
                },
                mqtt_username: field(&fields, "muser").to_string(),
                mqtt_password: field(&fields, "mpass").to_string(),
                opening_counts: 0,
            };
            *sub_post.lock().unwrap() = Some(cfg);
            (HTML_SUCCESS, 200u16)
        };

        let reason = if status == 200 { "OK" } else { "Bad Request" };
        let mut r = req.into_response(
            status,
            Some(reason),
            &[("Content-Type", "text/html; charset=utf-8")],
        )?;
        r.write(html.as_bytes())?;
        Ok::<(), anyhow::Error>(())
    })?;

    // ── Wait for submission ───────────────────────────────────────────────────
    info!("Waiting for user to submit config...");
    let config = loop {
        std::thread::sleep(std::time::Duration::from_millis(200));
        if let Some(cfg) = submitted.lock().unwrap().take() {
            break cfg;
        }
    };

    // Let the success page render before killing the server
    std::thread::sleep(std::time::Duration::from_millis(1500));

    drop(server);
    drop(wifi);

    config.save(nvs)?;
    info!(
        "Config saved — ssid={}, broker={}",
        config.wifi_ssid, config.mqtt_broker
    );
    Ok(())
}
