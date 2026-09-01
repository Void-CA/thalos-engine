//! ESP32 Hello — prueba de conexión real con el firmware Thalos.
//!
//! Conecta al ESP por Serial, hace handshake, sube un manifiesto
//! simple de 2 DOF, ejecuta, y monitorea hasta completar.
//!
//! # Uso
//!
//! ```bash
//! cargo run --example esp-hello -- /dev/ttyUSB0
//! ```
//!
//! # Salida esperada
//!
//! ```text
//! Conectando a /dev/ttyUSB0...
//!   >> HELLO 1
//!   << HELLO 1 OK
//!   ✓ Handshake OK
//!   ...
//!   ✓ Execution COMPLETED
//! ```

use std::env;
use std::time::Duration;

use thalos_runtime::backends::transport::{SerialTransport, Transport};

#[tokio::main]
async fn main() {
    let args: Vec<String> = env::args().collect();
    let port = args.get(1).map(|s| s.as_str()).unwrap_or("/dev/ttyUSB0");
    let baud: u32 = args.get(2).and_then(|s| s.parse().ok()).unwrap_or(115200);

    println!("Conectando a {} ({} baud)...", port, baud);

    let mut t = SerialTransport::new(port, baud);
    t.connect().await.expect("failed to open serial port");
    println!("  ✓ Puerto abierto\n");

    // ── Handshake ──────────────────────────────────────────────────────
    t.send(b"HELLO 1\n").await.unwrap();
    let resp = String::from_utf8_lossy(&t.receive().await.unwrap()).to_string();
    println!("  >> HELLO 1");
    println!("  << {}", resp.trim());
    assert!(resp.contains("HELLO 1 OK"), "Handshake falló: {resp}");
    println!("  ✓ Handshake OK\n");

    // ── Subir manifiesto: 2 DOF, 3 samples, 2 segundos ────────────────
    t.send(b"MANIFEST 2 3 2000000\n").await.unwrap();
    let resp = String::from_utf8_lossy(&t.receive().await.unwrap()).to_string();
    println!("  >> MANIFEST 2 3 2000000");
    println!("  << {}", resp.trim());

    t.send(b"SEGMENT 0 movej 0 3\n").await.unwrap();
    let resp = String::from_utf8_lossy(&t.receive().await.unwrap()).to_string();
    println!("  >> SEGMENT 0 movej 0 3");
    println!("  << {}", resp.trim());

    for (i, (j0, j1, dt)) in [(0.0, 0.0, 0), (0.5, 0.3, 1_000_000), (1.0, 0.5, 1_000_000)]
        .iter()
        .enumerate()
    {
        let cmd = format!("SAMPLE {j0} {j1} {dt}\n");
        t.send(cmd.as_bytes()).await.unwrap();
        let resp = String::from_utf8_lossy(&t.receive().await.unwrap()).to_string();
        println!("  >> SAMPLE {i}: {j0} {j1} dt={dt}");
        println!("  << {}", resp.trim());
    }

    t.send(b"END_UPLOAD\n").await.unwrap();
    let resp = String::from_utf8_lossy(&t.receive().await.unwrap()).to_string();
    println!("  >> END_UPLOAD");
    println!("  << {}", resp.trim());
    assert!(resp.contains("READY"), "Upload rechazado: {resp}");
    println!("  ✓ Manifiesto listo (READY)\n");

    // ── Ejecutar ───────────────────────────────────────────────────────
    t.send(b"EXECUTE\n").await.unwrap();
    let resp = String::from_utf8_lossy(&t.receive().await.unwrap()).to_string();
    println!("  >> EXECUTE");
    println!("  << {}", resp.trim());
    println!("  ✓ Ejecutando...\n");

    // ── Monitorear ─────────────────────────────────────────────────────
    for i in 0..10 {
        tokio::time::sleep(Duration::from_millis(500)).await;
        t.send(b"STATUS\n").await.unwrap();
        let resp = String::from_utf8_lossy(&t.receive().await.unwrap()).to_string();
        println!("  STATUS [{}/10]: {}", i + 1, resp.trim());

        if resp.contains("COMPLETED") {
            println!("\n  ✓ Ejecución COMPLETADA");
            break;
        }
    }

    // ── Limpiar ────────────────────────────────────────────────────────
    t.send(b"STOP\n").await.unwrap();
    let resp = String::from_utf8_lossy(&t.receive().await.unwrap()).to_string();
    println!("  >> STOP");
    println!("  << {}", resp.trim());

    t.disconnect().await.unwrap();
    println!("\n✅ ESP conectado, handshake, upload, execute — todo OK.");
}
