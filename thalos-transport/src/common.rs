use async_trait::async_trait;

/// Error de transporte físico (I/O, timeout, desconexión).
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

/// Medio de transporte físico entre Thalos y un dispositivo (I/O de bytes).
#[async_trait]
pub trait Transport: Send + Sync {
    /// Conectar al dispositivo.
    async fn connect(&mut self) -> Result<(), TransportError>;

    /// Desconectar.
    async fn disconnect(&mut self) -> Result<(), TransportError>;

    /// Enviar datos (bytes).
    async fn send(&mut self, data: &[u8]) -> Result<(), TransportError>;

    /// Recibir datos (bytes).
    async fn receive(&mut self) -> Result<Vec<u8>, TransportError>;

    /// Descartar líneas residuales en el buffer de entrada.
    async fn drain(&mut self) -> Result<(), TransportError> {
        Ok(())
    }
}

/// Transporte simulado de bytes para pruebas de infraestructura.
pub struct FakeTransport {
    sent: std::sync::Mutex<Vec<Vec<u8>>>,
    responses: std::sync::Mutex<Vec<Vec<u8>>>,
    connected: std::sync::atomic::AtomicBool,
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

    pub fn disconnect_on_empty_queue(&self) {
        self.disconnect_on_empty
            .store(true, std::sync::atomic::Ordering::SeqCst);
    }

    pub fn inject_response(&self, data: Vec<u8>) {
        self.responses.lock().unwrap().push(data);
    }

    pub fn sent_commands(&self) -> Vec<Vec<u8>> {
        self.sent.lock().unwrap().clone()
    }

    pub fn clear_sent(&self) {
        self.sent.lock().unwrap().clear();
    }
}

impl Default for FakeTransport {
    fn default() -> Self {
        Self::new()
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

    #[tokio::test]
    async fn fake_transport_roundtrip() {
        let mut transport = FakeTransport::new();
        transport.inject_response(b"STATE 1.0 2.0\n".to_vec());
        transport.connect().await.unwrap();
        transport.send(b"CMD MOVEJ 1.0 2.0\n").await.unwrap();
        let resp = transport.receive().await.unwrap();
        assert_eq!(String::from_utf8(resp).unwrap(), "STATE 1.0 2.0\n");
    }
}
