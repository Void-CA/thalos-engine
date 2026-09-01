use async_trait::async_trait;
use crate::common::{Transport, TransportError};

/// Transporte serial — conexión USB/UART a un MCU.
pub struct SerialTransport {
    port: String,
    baud: u32,
    reader: Option<tokio::sync::Mutex<tokio::io::BufReader<tokio_serial::SerialStream>>>,
    read_timeout: std::time::Duration,
    partial_line: Option<Vec<u8>>,
}

impl SerialTransport {
    pub fn new(port: impl Into<String>, baud: u32) -> Self {
        Self {
            port: port.into(),
            baud,
            reader: None,
            read_timeout: std::time::Duration::from_secs(2),
            partial_line: Some(Vec::new()),
        }
    }

    pub fn with_read_timeout(mut self, timeout: std::time::Duration) -> Self {
        self.read_timeout = timeout;
        self
    }

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
        if self.reader.is_some() {
            return Ok(());
        }
        let builder = tokio_serial::new(&self.port, self.baud);
        let port = tokio_serial::SerialStream::open(&builder)
            .map_err(|e| TransportError::Io(std::io::Error::new(std::io::ErrorKind::Other, e)))?;
        self.reader = Some(tokio::sync::Mutex::new(tokio::io::BufReader::new(port)));
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
        let line = self
            .partial_line
            .as_mut()
            .ok_or(TransportError::Disconnected)?;

        match tokio::time::timeout(self.read_timeout, guard.read_until(b'\n', line)).await {
            Err(_) => return Err(TransportError::Timeout),
            Ok(Err(e)) => return Err(TransportError::Io(e)),
            Ok(Ok(0)) => return Err(TransportError::Disconnected),
            Ok(Ok(_)) => {}
        }
        if line.is_empty() {
            return Err(TransportError::Disconnected);
        }
        let mut bytes = std::mem::take(line);
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
        for _ in 0..8 {
            match tokio::time::timeout(
                std::time::Duration::from_millis(20),
                guard.read_until(b'\n', line),
            )
            .await
            {
                Err(_) => break,
                Ok(Err(_)) => break,
                Ok(Ok(0)) => break,
                Ok(Ok(_)) => {
                    line.clear();
                }
            }
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::Duration;

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

    #[tokio::test]
    async fn serial_receive_reads_line_when_data_arrives() {
        let (master, slave) = tokio_serial::SerialStream::pair().unwrap();
        let mut transport =
            SerialTransport::from_stream(master, Duration::from_secs(2));
        use tokio::io::AsyncWriteExt;
        let mut slave = slave;
        slave.write_all(b"HELLO 2 OK\r\n").await.unwrap();
        slave.flush().await.unwrap();
        let resp = transport.receive().await.unwrap();
        assert_eq!(String::from_utf8(resp).unwrap(), "HELLO 2 OK\n");
    }

    #[tokio::test]
    async fn serial_receive_coalesces_fragmented_writes() {
        use tokio::io::AsyncWriteExt;
        let (master, mut slave) = tokio_serial::SerialStream::pair().unwrap();
        let mut transport = SerialTransport::from_stream(master, Duration::from_secs(2));

        slave.write_all(b"PART1").await.unwrap();
        slave.flush().await.unwrap();
        tokio::time::sleep(Duration::from_millis(80)).await;
        slave.write_all(b"PART2\n").await.unwrap();
        slave.flush().await.unwrap();

        let resp = transport.receive().await.unwrap();
        assert_eq!(String::from_utf8(resp).unwrap(), "PART1PART2\n");
    }

    #[tokio::test]
    async fn serial_partial_line_survives_receive_timeout() {
        use tokio::io::AsyncWriteExt;
        let (master, mut slave) = tokio_serial::SerialStream::pair().unwrap();
        let mut transport = SerialTransport::from_stream(master, Duration::from_millis(150));

        slave.write_all(b"STATUS RUN").await.unwrap();
        slave.flush().await.unwrap();
        let err = transport.receive().await.unwrap_err();
        assert!(matches!(err, TransportError::Timeout), "got {err:?}");

        slave.write_all(b"NING 0.5\n").await.unwrap();
        slave.flush().await.unwrap();
        let resp = transport.receive().await.unwrap();
        assert_eq!(String::from_utf8(resp).unwrap(), "STATUS RUNNING 0.5\n");
    }

    #[tokio::test]
    async fn serial_receive_reports_disconnect_when_peer_dropped() {
        let (master, slave) = tokio_serial::SerialStream::pair().unwrap();
        let mut transport = SerialTransport::from_stream(master, Duration::from_millis(150));

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
