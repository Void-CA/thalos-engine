use async_trait::async_trait;
use crate::common::{Transport, TransportError};

/// Transporte TCP — conecta a un socket (ESP32 o simulator).
pub struct TcpTransport {
    addr: String,
    stream: Option<tokio::sync::Mutex<tokio::net::tcp::OwnedWriteHalf>>,
    reader: Option<tokio::io::BufReader<tokio::net::tcp::OwnedReadHalf>>,
    partial_line: Option<Vec<u8>>,
    receive_timeout_ms: u64,
}

impl TcpTransport {
    pub fn new(addr: impl Into<String>) -> Self {
        Self::with_receive_timeout(addr, 500)
    }

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
        Ok(std::mem::take(line))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[tokio::test]
    async fn tcp_partial_line_survives_receive_timeout() {
        use tokio::io::AsyncWriteExt;
        use tokio::net::TcpListener;
        let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
        let addr = listener.local_addr().unwrap();
        let mut transport = TcpTransport::with_receive_timeout(addr.to_string(), 100);
        transport.connect().await.unwrap();

        let (mut socket, _) = listener.accept().await.unwrap();
        socket.write_all(b"STATUS RUN").await.unwrap();
        let err = transport.receive().await.unwrap_err();
        assert!(matches!(err, TransportError::Timeout), "got {err:?}");

        socket.write_all(b"NING 0.5\n").await.unwrap();
        let resp = transport.receive().await.unwrap();
        assert_eq!(String::from_utf8(resp).unwrap(), "STATUS RUNNING 0.5\n");
    }

    #[tokio::test]
    async fn tcp_receive_times_out_without_data() {
        use tokio::net::TcpListener;
        let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
        let addr = listener.local_addr().unwrap();

        let mut transport = TcpTransport::with_receive_timeout(addr.to_string(), 100);
        transport.connect().await.unwrap();

        let start = std::time::Instant::now();
        let err = transport.receive().await.unwrap_err();
        assert!(matches!(err, TransportError::Timeout), "got {err:?}");
        assert!(
            start.elapsed() < std::time::Duration::from_secs(5),
            "receive must not block forever"
        );
    }
}
