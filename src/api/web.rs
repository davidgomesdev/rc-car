//! API: HTTP + WebSocket server. Serves the controller UI, proxies camera
//! status, and turns inbound WebSocket text frames into [`RemoteCmd`]s on the
//! shared command bus.

use anyhow::Result;
use std::sync::Arc;

use embedded_svc::io::Write as _;
use esp_idf_svc::http::client::{Configuration as HttpClientConfig, EspHttpConnection};
use esp_idf_svc::http::server::{Configuration as ServerConfig, EspHttpServer};
use esp_idf_svc::http::Method;
use esp_idf_svc::sys::EspError;

use crate::api::motors::CommandBus;
use crate::app::controller::{parse_cmd, RemoteCmd};

static INDEX_HTML: &str = include_str!("../controller.html");

/// Register all HTTP/WebSocket handlers and return the running server handle.
///
/// The handle **must be kept alive** for the duration of the program; dropping
/// it tears down the server. `shared` is the command bus that the WS handler
/// writes into and the motor control loop reads from.
pub fn start(shared: CommandBus) -> Result<EspHttpServer<'static>> {
    let server_cfg = ServerConfig {
        stack_size: 10240,
        ..Default::default()
    };
    let mut server = EspHttpServer::new(&server_cfg)?;

    // Serve the controller UI at /.
    server.fn_handler("/", Method::Get, |req| {
        log::info!("User visited page");
        req.into_ok_response()?
            .write_all(INDEX_HTML.as_bytes())
            .map(|_| ())
    })?;

    // Proxy: fetch camera status server-side to avoid CORS issues in the browser.
    // GET /camerastatus?ip=<camera-ip> → 200 if camera is up, 502 otherwise.
    server.fn_handler("/camerastatus", Method::Get, |req| {
        let uri = req.uri();
        let ip = uri
            .split_once("ip=")
            .map(|(_, v)| v.split('&').next().unwrap_or(v).trim())
            .unwrap_or("");

        let ok = if ip.is_empty() {
            false
        } else {
            let url = format!("http://{}:8080/videostatus", ip);
            let cfg = HttpClientConfig::default();
            EspHttpConnection::new(&cfg)
                .and_then(|mut conn| {
                    conn.initiate_request(Method::Get, &url, &[])?;
                    conn.initiate_response()?;
                    Ok(conn.status() == 200)
                })
                .unwrap_or(false)
        };

        let mut resp = req.into_response(
            if ok { 200 } else { 502 },
            None,
            &[("Content-Type", "text/plain")],
        )?;
        resp.write_all(if ok { b"ok" } else { b"error" }).map(|_| ())
    })?;

    // WebSocket handler: parse incoming text frames and push to shared state.
    // Only RemoteCmd (Copy) crosses the thread boundary — motor types stay on main.
    let shared_ws = Arc::clone(&shared);
    server.ws_handler("/ws", None, move |ws| {
        if ws.is_new() {
            log::info!("WS: client connected (session {})", ws.session());
            return Ok(());
        }
        if ws.is_closed() {
            log::info!("WS: client disconnected — stopping motors");
            *shared_ws.lock().unwrap() = RemoteCmd::Stop;
            return Ok(());
        }

        // ESP-IDF WS requires two recv calls: first with empty buf to get length,
        // then with a sized buf to read the payload.
        let (_frame_type, len) = ws.recv(&mut [])?;
        if len == 0 || len > 32 {
            return Ok(());
        }
        let mut buf = [0u8; 32];
        ws.recv(&mut buf[..len])?;

        let s = std::str::from_utf8(&buf[..len])
            .unwrap_or("")
            .trim_matches(|c: char| c.is_ascii_control() || c.is_whitespace());

        log::info!("WS rx: '{s}'");
        *shared_ws.lock().unwrap() = parse_cmd(s);

        Ok::<(), EspError>(())
    })?;

    Ok(server)
}
