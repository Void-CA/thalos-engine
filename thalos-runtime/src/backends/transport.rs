//! Transport abstraction — comunicación con hardware real.
//!
//! Separa el protocolo de aplicación (comando → wire format) del
//! medio físico (serial, TCP, MQTT, etc.).
//!
//! # Ejemplo
//!
//! ```ignore
//! let transport = SerialTransport::new("/dev/ttyUSB0", 115200)?;
//! transport.send(b"CMD MOVEJ 0.5 -0.3 0.1\n")?;
//! let response = transport.receive()?;
//! ```

use async_trait::async_trait;

/// Error de transporte.

/// Error de transporte.
#[derive(Debug)]
pub enum TransportError {
    Io(std::io::Error),
    Timeout,
    Disconnected,
    InvalidResponse(String),
}

impl std::fmt::Display for TransportError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            TransportError::Io(e) => write!(f, "IO error: {}", e),
            TransportError::Timeout => write!(f, "Transport timeout"),
            TransportError::Disconnected => write!(f, "Transport disconnected"),
            TransportError::InvalidResponse(s) => write!(f, "Invalid response: {}", s),
        }
    }
}

impl std::error::Error for TransportError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            TransportError::Io(e) => Some(e),
            _ => None,
        }
    }
}

impl From<std::io::Error> for TransportError {
    fn from(e: std::io::Error) -> Self {
        TransportError::Io(e)
    }
}

/// Medio de transporte entre Thalos y un backend físico.
///
/// No conoce el formato de los mensajes — solo envía y recibe bytes.
#[async_trait]
pub trait Transport: Send + Sync {
    /// Conectar al dispositivo.
    async fn connect(&mut self) -> Result<(), TransportError>;

    /// Desconectar.
    async fn disconnect(&mut self) -> Result<(), TransportError>;

    /// Enviar datos. Bloquea hasta que se envíen todos los bytes.
    async fn send(&mut self, data: &[u8]) -> Result<(), TransportError>;

    /// Recibir datos. Bloquea hasta recibir al menos 1 byte.
    async fn receive(&mut self) -> Result<Vec<u8>, TransportError>;

    /// Descartar líneas residuales sin destinatario (defensa contra desync de
    /// protocolo). Real transports (serial/TCP) la implementan leyendo con un
    /// timeout corto hasta que no queda nada; test fakes la dejan como no-op
    /// porque su cola de respuestas pertenece a comandos futuros.
    async fn drain(&mut self) -> Result<(), TransportError> {
        Ok(())
    }
}

/// Transporte TCP — conecta a un ESP32 (o simulador) por socket.
pub struct TcpTransport {
    addr: String,
    stream: Option<tokio::sync::Mutex<tokio::net::tcp::OwnedWriteHalf>>,
    /// REL-04 / RES-05: the read buffer persists across `receive()` calls —
    /// a fresh `BufReader` per call threw away partially-buffered line bytes
    /// on timeout and desynced the protocol permanently.
    reader: Option<tokio::io::BufReader<tokio::net::tcp::OwnedReadHalf>>,
    /// In-progress response line carried across `receive()` calls so a
    /// timed-out partial line is never lost. NOTE: `read_until` appends
    /// DIRECTLY to this Vec (cancellation-safe), unlike `read_line`, which
    /// mem::takes the String into a future-owned buffer that is dropped on
    /// timeout.
    partial_line: Option<Vec<u8>>,
    /// Max time to wait for a response line, in milliseconds (S1.3).
    receive_timeout_ms: u64,
}

impl TcpTransport {
    pub fn new(addr: impl Into<String>) -> Self {
        Self::with_receive_timeout(addr, 500)
    }

    /// Create a TCP transport with an explicit receive timeout (ms).
    /// `receive()` returns `Error::Timeout` if no data arrives in time.
    pub fn with_receive_timeout(addr: impl Into<String>, receive_timeout_ms: u64) -> Self {
        Self {
            addr: addr.into(),
            stream: None,
            reader: None,
            partial_line: Some(Vec::new()),
            receive_timeout_ms,
        }
    }
}

#[async_trait]
impl Transport for TcpTransport {
    async fn connect(&mut self) -> Result<(), TransportError> {
        let stream = tokio::net::TcpStream::connect(&self.addr).await?;
        // Split the socket: the owned read half feeds the persistent BufReader
        // (which keeps partially-read line bytes across receive() calls); the
        // owned write half stays behind the mutex for send().
        let (read_half, write_half) = stream.into_split();
        self.reader = Some(tokio::io::BufReader::new(read_half));
        self.stream = Some(tokio::sync::Mutex::new(write_half));
        Ok(())
    }

    async fn disconnect(&mut self) -> Result<(), TransportError> {
        self.stream = None;
        self.reader = None;
        self.partial_line = None;
        Ok(())
    }

    async fn send(&mut self, data: &[u8]) -> Result<(), TransportError> {
        use tokio::io::AsyncWriteExt;
        let stream = self.stream.as_ref().ok_or(TransportError::Disconnected)?;
        let mut guard = stream.lock().await;
        guard.write_all(data).await?;
        guard.flush().await?;
        Ok(())
    }

    async fn receive(&mut self) -> Result<Vec<u8>, TransportError> {
        use tokio::io::AsyncBufReadExt;
        let reader = self.reader.as_mut().ok_or(TransportError::Disconnected)?;
        let line = self.partial_line.as_mut().ok_or(TransportError::Disconnected)?;
        // S1.3: bound the read — a silent peer must surface `Timeout` instead
        // of blocking the request forever (mirrors SerialTransport R4-002).
        // REL-04: `read_until` accumulates into `line` directly, so on timeout
        // the partially-read prefix SURVIVES in `line` and is resumed by the
        // next call (unlike `read_line`, whose future-owned buffer is dropped).
        match tokio::time::timeout(
            std::time::Duration::from_millis(self.receive_timeout_ms),
            reader.read_until(b'\n', line),
        )
        .await
        {
            Err(_) => return Err(TransportError::Timeout),
            Ok(Err(e)) => return Err(TransportError::Io(e)),
            Ok(Ok(0)) => return Err(TransportError::Disconnected),
            Ok(Ok(_)) => {}
        }
        if line.is_empty() {
            return Err(TransportError::Disconnected);
        }
        // Take the completed line; the next call starts a fresh line.
        Ok(std::mem::take(line))
    }
}

/// Transporte serial — conexión USB/UART a un ESP32 (o cualquier MCU).
///
/// Lee línea por línea usando un `BufReader` interno. La velocidad y
/// configuración del puerto se definen en `new()`.
///
/// # Timeout de lectura (R4-002)
///
/// `receive()` espera una línea hasta `read_timeout`; si el dispositivo no
/// responde (TTY silencioso) devuelve `TransportError::Timeout` en vez de
/// bloquear para siempre. Sin esto, un `POST /backends/esp32/connect` contra
/// un puerto sin firmware cuelga la request y deja el dispositivo abierto
/// (el retry choca con `port_in_use` hasta reiniciar el proceso).
pub struct SerialTransport {
    port: String,
    baud: u32,
    /// Persistent buffered reader — OWNS the serial stream. The previous
    /// design created a fresh `BufReader` per `receive()` call, so excess
    /// bytes buffered from a multi-line burst (or a cancelled read) were
    /// DROPPED with the local reader → the next read returned a line
    /// FRAGMENT ("0.000000 0.000000…") that desynced the protocol (real
    /// repro: "unexpected response: 0.000000 0.000000" during upload).
    /// Keeping the reader across calls preserves those bytes (mirrors the
    /// TCP transport's persistent `reader`). `send()` writes through
    /// `BufReader::get_mut()`.
    reader: Option<tokio::sync::Mutex<tokio::io::BufReader<tokio_serial::SerialStream>>>,
    /// Max wait for a response line in `receive`. Defaults to 2s — short
    /// enough to beat the frontend 10s timeout and return `no_firmware` fast,
    /// long enough for a real device to answer the HELLO handshake.
    read_timeout: std::time::Duration,
    /// In-progress response line carried across `receive()` calls so a
    /// timed-out partial line is never lost (mirrors the TCP `partial_line`,
    /// REL-04). `read_until` appends DIRECTLY to this Vec (cancellation-safe),
    /// unlike `read_line`, which mem::takes the String into a future-owned
    /// buffer that is dropped on timeout.
    partial_line: Option<Vec<u8>>,
}

impl SerialTransport {
    /// Crear un nuevo transporte serial.
    ///
    /// `port` es el path al dispositivo (ej: `"/dev/ttyUSB0"`).
    /// `baud` es la velocidad en baudios (ej: `115200`).
    pub fn new(port: impl Into<String>, baud: u32) -> Self {
        Self {
            port: port.into(),
            baud,
            reader: None,
            read_timeout: std::time::Duration::from_secs(2),
            partial_line: Some(Vec::new()),
        }
    }

    /// Override the receive read timeout (R4-002). Tests use a short value so
    /// the silent-device path is exercised fast; production keeps the 2s default.
    pub fn with_read_timeout(mut self, timeout: std::time::Duration) -> Self {
        self.read_timeout = timeout;
        self
    }
}

#[cfg(test)]
impl SerialTransport {
    /// Build a transport over an already-open stream (test seam): the virtual
    /// serial pair from `SerialStream::pair()` exercises the REAL read path
    /// without a physical device.
    pub fn from_stream(stream: tokio_serial::SerialStream, read_timeout: std::time::Duration) -> Self {
        Self {
            port: String::new(),
            baud: 0,
            reader: Some(tokio::sync::Mutex::new(tokio::io::BufReader::new(stream))),
            read_timeout,
            partial_line: Some(Vec::new()),
        }
    }
}

#[async_trait]
impl Transport for SerialTransport {
    async fn connect(&mut self) -> Result<(), TransportError> {
        // Idempotent: a reader already present (test seam) stays open.
        if self.reader.is_some() {
            return Ok(());
        }
        let builder = tokio_serial::new(&self.port, self.baud);
        let port = tokio_serial::SerialStream::open(&builder)
            .map_err(|e| TransportError::Io(std::io::Error::new(std::io::ErrorKind::Other, e)))?;
        self.reader = Some(tokio::sync::Mutex::new(tokio::io::BufReader::new(port)));
        // A fresh device connection must not inherit a stale partial line.
        self.partial_line = Some(Vec::new());
        Ok(())
    }

    async fn disconnect(&mut self) -> Result<(), TransportError> {
        self.reader = None;
        self.partial_line = None;
        Ok(())
    }

    async fn send(&mut self, data: &[u8]) -> Result<(), TransportError> {
        use tokio::io::AsyncWriteExt;
        let reader = self.reader.as_ref().ok_or(TransportError::Disconnected)?;
        let mut guard = reader.lock().await;
        guard.get_mut().write_all(data).await?;
        guard.get_mut().flush().await?;
        Ok(())
    }

    async fn receive(&mut self) -> Result<Vec<u8>, TransportError> {
        use tokio::io::AsyncBufReadExt;
        let reader = self.reader.as_ref().ok_or(TransportError::Disconnected)?;
        let mut guard = reader.lock().await;
        // REL-04 (serial): `read_until` accumulates into the persistent
        // `partial_line`, so a partial line buffered when the timeout fires
        // SURVIVES and is resumed by the next call (unlike `read_line`, whose
        // future-owned String buffer is dropped on timeout, losing the prefix).
        // The reader itself is ALSO persistent: excess bytes buffered from a
        // multi-line burst stay in the reader and are returned by the next
        // call, instead of being dropped with a per-call `BufReader` (the
        // desync that produced "unexpected response: 0.000000 0.000000").
        let line = self
            .partial_line
            .as_mut()
            .ok_or(TransportError::Disconnected)?;
        // R4-002: bound the read — a silent device must surface `Timeout`
        // (→ `no_firmware`) instead of blocking the request forever.
        match tokio::time::timeout(self.read_timeout, guard.read_until(b'\n', line)).await {
            Err(_) => return Err(TransportError::Timeout),
            Ok(Err(e)) => return Err(TransportError::Io(e)),
            Ok(Ok(0)) => return Err(TransportError::Disconnected),
            Ok(Ok(_)) => {}
        }
        if line.is_empty() {
            return Err(TransportError::Disconnected);
        }
        // Take the completed line; the next call starts a fresh one.
        let mut bytes = std::mem::take(line);
        // Strip trailing \r\n or \n (ESP firmware envía \r\n), then restore a
        // single \n for the protocol parser.
        if bytes.last() == Some(&b'\n') {
            bytes.pop();
        }
        if bytes.last() == Some(&b'\r') {
            bytes.pop();
        }
        bytes.push(b'\n');
        Ok(bytes)
    }

    async fn drain(&mut self) -> Result<(), TransportError> {
        use tokio::io::AsyncBufReadExt;
        let reader = self.reader.as_ref().ok_or(TransportError::Disconnected)?;
        let mut guard = reader.lock().await;
        let line = self
            .partial_line
            .as_mut()
            .ok_or(TransportError::Disconnected)?;
        // Consume stale complete lines with a short bound. A partial line
        // (timeout mid-line) stays in `partial_line` for the next receive to
        // resume — the persistent reader guarantees no bytes are lost.
        for _ in 0..8 {
            match tokio::time::timeout(
                std::time::Duration::from_millis(20),
                guard.read_until(b'\n', line),
            )
            .await
            {
                Err(_) => break,   // nothing more buffered
                Ok(Err(_)) => break,
                Ok(Ok(0)) => break,
                Ok(Ok(_)) => {
                    line.clear(); // consumed one stale line; read the next
                }
            }
        }
        Ok(())
    }
}

/// Transporte simulado — para tests sin hardware real.
pub struct FakeTransport {
    sent: std::sync::Mutex<Vec<Vec<u8>>>,
    responses: std::sync::Mutex<Vec<Vec<u8>>>,
    connected: std::sync::atomic::AtomicBool,
    /// When set, the next `receive` that finds an empty response queue reports
    /// the transport disconnected (R4-001 test seam: simulate a device that
    /// drops mid-operation AFTER answering the HELLO handshake).
    disconnect_on_empty: std::sync::atomic::AtomicBool,
}

impl FakeTransport {
    pub fn new() -> Self {
        Self {
            sent: std::sync::Mutex::new(Vec::new()),
            responses: std::sync::Mutex::new(Vec::new()),
            connected: std::sync::atomic::AtomicBool::new(false),
            disconnect_on_empty: std::sync::atomic::AtomicBool::new(false),
        }
    }

    /// Arm the transport to report `TransportError::Disconnected` on the next
    /// `receive` that has no queued response — i.e. right after the injected
    /// HELLO response is consumed. Test seam for the ConnectionLost path.
    pub fn disconnect_on_empty_queue(&self) {
        self.disconnect_on_empty
            .store(true, std::sync::atomic::Ordering::SeqCst);
    }

    /// Inyectar una respuesta que se devolverá en el próximo `receive()`.
    pub fn inject_response(&self, data: Vec<u8>) {
        self.responses.lock().unwrap().push(data);
    }

    /// Comandos enviados hasta ahora.
    pub fn sent_commands(&self) -> Vec<Vec<u8>> {
        self.sent.lock().unwrap().clone()
    }

    /// Limpiar el historial de comandos.
    pub fn clear_sent(&self) {
        self.sent.lock().unwrap().clear();
    }
}

#[async_trait]
impl Transport for FakeTransport {
    async fn connect(&mut self) -> Result<(), TransportError> {
        self.connected
            .store(true, std::sync::atomic::Ordering::SeqCst);
        Ok(())
    }

    async fn disconnect(&mut self) -> Result<(), TransportError> {
        self.connected
            .store(false, std::sync::atomic::Ordering::SeqCst);
        Ok(())
    }

    async fn send(&mut self, data: &[u8]) -> Result<(), TransportError> {
        self.sent.lock().unwrap().push(data.to_vec());
        Ok(())
    }

    async fn receive(&mut self) -> Result<Vec<u8>, TransportError> {
        let mut responses = self.responses.lock().unwrap();
        if responses.is_empty() {
            if self
                .disconnect_on_empty
                .swap(false, std::sync::atomic::Ordering::SeqCst)
            {
                return Err(TransportError::Disconnected);
            }
            return Err(TransportError::Timeout);
        }
        Ok(responses.remove(0))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::Duration;

    #[tokio::test]
    async fn fake_transport_roundtrip() {
        let mut transport = FakeTransport::new();
        transport.inject_response(b"STATE 1.0 2.0\n".to_vec());
        transport.connect().await.unwrap();
        transport.send(b"CMD MOVEJ 1.0 2.0\n").await.unwrap();
        let resp = transport.receive().await.unwrap();
        assert_eq!(String::from_utf8(resp).unwrap(), "STATE 1.0 2.0\n");
    }

    /// R4-002: `SerialTransport::receive` on a SILENT device must time out
    /// instead of blocking forever — the no_firmware handshake depends on it
    /// (a silent TTY currently hangs the connect request indefinitely).
    #[tokio::test]
    async fn serial_receive_times_out_on_silent_port() {
        let (master, _slave) = tokio_serial::SerialStream::pair().unwrap();
        let mut transport =
            SerialTransport::from_stream(master, Duration::from_millis(150));
        let start = std::time::Instant::now();
        let err = transport.receive().await.unwrap_err();
        assert!(matches!(err, TransportError::Timeout), "got {err:?}");
        assert!(
            start.elapsed() < Duration::from_secs(5),
            "receive must not hang forever"
        );
    }

    /// R4-002: a device that DOES answer must still read its line — the timeout
    /// must not break the healthy path.
    #[tokio::test]
    async fn serial_receive_reads_line_when_data_arrives() {
        let (master, slave) = tokio_serial::SerialStream::pair().unwrap();
        let mut transport =
            SerialTransport::from_stream(master, Duration::from_secs(2));
        // Write from the OTHER end of the virtual pair; the transport reads it.
        use tokio::io::AsyncWriteExt;
        let mut slave = slave;
        slave.write_all(b"HELLO 2 OK\r\n").await.unwrap();
        slave.flush().await.unwrap();
        let resp = transport.receive().await.unwrap();
        assert_eq!(String::from_utf8(resp).unwrap(), "HELLO 2 OK\n");
    }

    /// REL-04 / RES-05 (RED): a partial line buffered by `read_line` when the
    /// receive timeout fires must NOT be lost — the BufReader AND the
    /// in-progress line persist across `receive()` calls, so a slow/partial
    /// write never desyncs the protocol permanently. (A fresh BufReader per
    /// call throws the prefix away and the next line starts mid-token.)
    #[tokio::test]
    async fn tcp_partial_line_survives_receive_timeout() {
        use tokio::io::AsyncWriteExt;
        use tokio::net::TcpListener;
        let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
        let addr = listener.local_addr().unwrap();
        let mut transport = TcpTransport::with_receive_timeout(addr.to_string(), 100);
        transport.connect().await.unwrap();

        let (mut socket, _) = listener.accept().await.unwrap();
        // Half a line, then silence: the first receive() times out mid-line.
        socket.write_all(b"STATUS RUN").await.unwrap();
        let err = transport.receive().await.unwrap_err();
        assert!(matches!(err, TransportError::Timeout), "got {err:?}");

        // The rest arrives later — the buffered prefix must be kept.
        socket.write_all(b"NING 0.5\n").await.unwrap();
        let resp = transport.receive().await.unwrap();
        assert_eq!(String::from_utf8(resp).unwrap(), "STATUS RUNNING 0.5\n");
    }

    /// S1.3/S1.5 (RED): `TcpTransport::receive()` on a silent peer MUST return
    /// `Error::Timeout` after `receive_timeout_ms` instead of blocking forever.
    #[tokio::test]
    async fn tcp_receive_times_out_without_data() {
        use tokio::net::TcpListener;
        let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
        let addr = listener.local_addr().unwrap();

        // 100ms timeout so the test is fast; default is 500ms.
        let mut transport = TcpTransport::with_receive_timeout(addr.to_string(), 100);
        transport.connect().await.unwrap();

        // Do NOT accept/write on the listener: the peer stays silent.
        let start = std::time::Instant::now();
        let err = transport.receive().await.unwrap_err();
        assert!(matches!(err, TransportError::Timeout), "got {err:?}");
        assert!(
            start.elapsed() < Duration::from_secs(5),
            "receive must not block forever"
        );
    }

    /// Fragmented writes: a line split across several `write` calls with a gap
    /// between them must still be delivered COMPLETE by a single `receive()`.
    /// The read path (kernel buffering + `read_until`) coalesces the fragments
    /// instead of returning a truncated line.
    #[tokio::test]
    async fn serial_receive_coalesces_fragmented_writes() {
        use tokio::io::AsyncWriteExt;
        let (master, mut slave) = tokio_serial::SerialStream::pair().unwrap();
        let mut transport = SerialTransport::from_stream(master, Duration::from_secs(2));

        // "PART1" arrives first; a byte-count read would return early here.
        slave.write_all(b"PART1").await.unwrap();
        slave.flush().await.unwrap();
        // Gap between fragments, then the rest of the line.
        tokio::time::sleep(Duration::from_millis(80)).await;
        slave.write_all(b"PART2\n").await.unwrap();
        slave.flush().await.unwrap();

        let resp = transport.receive().await.unwrap();
        assert_eq!(String::from_utf8(resp).unwrap(), "PART1PART2\n");
    }

    /// RED: a partial line buffered when the receive timeout fires must NOT be
    /// lost — the in-progress line must survive across `receive()` calls so a
    /// slow/partial write never desyncs the protocol permanently. Mirrors the
    /// REL-04 guarantee the TCP transport already has.
    #[tokio::test]
    async fn serial_partial_line_survives_receive_timeout() {
        use tokio::io::AsyncWriteExt;
        let (master, mut slave) = tokio_serial::SerialStream::pair().unwrap();
        let mut transport = SerialTransport::from_stream(master, Duration::from_millis(150));

        // Half a line, then silence: the first receive() times out mid-line.
        slave.write_all(b"STATUS RUN").await.unwrap();
        slave.flush().await.unwrap();
        let err = transport.receive().await.unwrap_err();
        assert!(matches!(err, TransportError::Timeout), "got {err:?}");

        // The rest arrives later — the buffered prefix must be kept.
        slave.write_all(b"NING 0.5\n").await.unwrap();
        slave.flush().await.unwrap();
        let resp = transport.receive().await.unwrap();
        assert_eq!(String::from_utf8(resp).unwrap(), "STATUS RUNNING 0.5\n");
    }

    /// EOF/disconnect: when the peer end of the virtual serial pair is dropped,
    /// `receive()` must surface a TERMINAL error instead of blocking until the
    /// read timeout. Note: Linux PTYs report EIO when the slave side closes,
    /// so the drop maps to `Io` here; macOS reports EOF → `Disconnected`. A
    /// disconnect must NEVER masquerade as a `Timeout`.
    #[tokio::test]
    async fn serial_receive_reports_disconnect_when_peer_dropped() {
        let (master, slave) = tokio_serial::SerialStream::pair().unwrap();
        let mut transport = SerialTransport::from_stream(master, Duration::from_millis(150));

        // Drop the peer end of the PTY pair.
        drop(slave);

        let start = std::time::Instant::now();
        let err = transport.receive().await.unwrap_err();
        assert!(
            !matches!(err, TransportError::Timeout),
            "peer drop must NOT surface as a timeout, got {err:?}"
        );
        assert!(
            matches!(
                err,
                TransportError::Disconnected | TransportError::Io(_)
            ),
            "peer drop must surface as Disconnected or Io, got {err:?}"
        );
        assert!(
            start.elapsed() < Duration::from_secs(5),
            "receive must not hang after the peer is dropped"
        );
    }

    /// Sizing: a line longer than 1KB must be read intact — `read_until` grows
    /// its buffer dynamically, so a long line (e.g. a verbose error) must not
    /// be truncated at a fixed chunk. This is a HOST-side bound; the firmware
    /// 256-byte buffer is a separate device limit.
    #[tokio::test]
    async fn serial_receive_handles_long_line() {
        use tokio::io::AsyncWriteExt;
        let (master, mut slave) = tokio_serial::SerialStream::pair().unwrap();
        let mut transport = SerialTransport::from_stream(master, Duration::from_secs(2));

        let mut long_line = vec![b'A'; 4096];
        long_line.push(b'\n');
        slave.write_all(&long_line).await.unwrap();
        slave.flush().await.unwrap();

        let resp = transport.receive().await.unwrap();
        assert_eq!(resp.len(), 4097);
        assert!(
            resp.iter().take(4096).all(|&b| b == b'A'),
            "long line must be read intact"
        );
        assert_eq!(resp[4096], b'\n');
    }
}
