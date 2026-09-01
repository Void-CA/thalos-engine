use thiserror::Error;

#[derive(Debug, Error, PartialEq, Eq)]
pub enum CodecError {
    #[error("empty line")]
    EmptyLine,
    #[error("malformed response: {0}")]
    MalformedResponse(String),
}

/// Raw parsed response frame from ESP32 text wire protocol.
#[derive(Debug, Clone, PartialEq)]
pub enum Esp32Frame {
    Ok,
    Ready,
    HandshakeOk(u32),
    Error(String),
    StatusIdle,
    StatusReceiving,
    StatusReady,
    StatusRunning { progress: f64, values: Vec<f64> },
    StatusCompleted { sample_count: u32 },
    SampleFrame { timestamp_us: u64, values: Vec<f64> },
}

/// Pure parser and encoder for ESP32 text wire format (ADR-014).
pub struct Esp32Codec;

impl Esp32Codec {
    /// Format HELLO handshake request.
    pub fn encode_hello(version: u32) -> String {
        format!("HELLO {}\n", version)
    }

    /// Format MANIFEST initialization header.
    pub fn encode_manifest(dof: usize, count: usize, duration_us: u64) -> String {
        format!("MANIFEST {} {} {}\n", dof, count, duration_us)
    }

    /// Format SAMPLE command frame for motion playback: `SAMPLE <v0> <v1> ... <dt_us>`
    pub fn encode_sample(values: &[f64], dt_us: u32) -> String {
        let mut parts = vec!["SAMPLE".to_string()];
        for v in values {
            parts.push(format!("{:.6}", v));
        }
        parts.push(dt_us.to_string());
        parts.join(" ") + "\n"
    }

    /// Format EXECUTE command.
    pub fn encode_execute() -> String {
        "EXECUTE\n".to_string()
    }

    /// Format STOP command.
    pub fn encode_stop() -> String {
        "STOP\n".to_string()
    }

    /// Format STATUS query command.
    pub fn encode_status() -> String {
        "STATUS\n".to_string()
    }

    /// Parse a single response line into an `Esp32Frame`.
    pub fn parse_response(line: &str) -> Result<Esp32Frame, CodecError> {
        let trimmed = line.trim();
        if trimmed.is_empty() {
            return Err(CodecError::EmptyLine);
        }
        let parts: Vec<&str> = trimmed.split_whitespace().collect();
        if parts.is_empty() {
            return Err(CodecError::EmptyLine);
        }

        match parts[0] {
            "HELLO" => {
                if parts.len() >= 3 && parts[2] == "OK" {
                    let version: u32 = parts[1]
                        .parse()
                        .map_err(|_| CodecError::MalformedResponse(line.to_string()))?;
                    Ok(Esp32Frame::HandshakeOk(version))
                } else {
                    Err(CodecError::MalformedResponse(line.to_string()))
                }
            }
            "OK" => Ok(Esp32Frame::Ok),
            "READY" => Ok(Esp32Frame::Ready),
            "ERROR" => {
                let reason = if parts.len() > 1 {
                    parts[1..].join(" ")
                } else {
                    "unknown".into()
                };
                Ok(Esp32Frame::Error(reason))
            }
            "STATUS" => {
                if parts.len() < 2 {
                    return Err(CodecError::MalformedResponse(line.to_string()));
                }
                match parts[1] {
                    "IDLE" => Ok(Esp32Frame::StatusIdle),
                    "RECEIVING" => Ok(Esp32Frame::StatusReceiving),
                    "READY" => Ok(Esp32Frame::StatusReady),
                    "RUNNING" => {
                        let progress: f64 = match parts.get(2) {
                            Some(raw) => raw.parse().map_err(|_| {
                                CodecError::MalformedResponse(line.to_string())
                            })?,
                            None => 0.0,
                        };
                        if !progress.is_finite() {
                            return Err(CodecError::MalformedResponse(line.to_string()));
                        }
                        let values = parts
                            .get(3..)
                            .unwrap_or(&[])
                            .iter()
                            .map(|s| {
                                s.parse::<f64>()
                                    .map_err(|_| CodecError::MalformedResponse(line.to_string()))
                            })
                            .collect::<Result<Vec<f64>, _>>()?;
                        Ok(Esp32Frame::StatusRunning { progress, values })
                    }
                    "COMPLETED" => {
                        let sample_count: u32 = match parts.get(2) {
                            Some(raw) => raw.parse().map_err(|_| {
                                CodecError::MalformedResponse(line.to_string())
                            })?,
                            None => 0,
                        };
                        Ok(Esp32Frame::StatusCompleted { sample_count })
                    }
                    "ERROR" => {
                        let reason = if parts.len() > 2 {
                            parts[2..].join(" ")
                        } else {
                            "unknown".into()
                        };
                        Ok(Esp32Frame::Error(reason))
                    }
                    _ => Err(CodecError::MalformedResponse(line.to_string())),
                }
            }
            "SAMPLE" => {
                if parts.len() < 3 {
                    return Err(CodecError::MalformedResponse(line.to_string()));
                }
                let timestamp: u64 = parts[1]
                    .parse()
                    .map_err(|_| CodecError::MalformedResponse(line.to_string()))?;
                let values = parts[2..]
                    .iter()
                    .map(|s| {
                        s.parse::<f64>()
                            .map_err(|_| CodecError::MalformedResponse(line.to_string()))
                    })
                    .collect::<Result<Vec<f64>, _>>()?;
                Ok(Esp32Frame::SampleFrame {
                    timestamp_us: timestamp,
                    values,
                })
            }
            _ => Err(CodecError::MalformedResponse(line.to_string())),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parse_hello_ok() {
        let frame = Esp32Codec::parse_response("HELLO 2 OK\n").unwrap();
        assert_eq!(frame, Esp32Frame::HandshakeOk(2));
    }

    #[test]
    fn parse_status_running() {
        let frame = Esp32Codec::parse_response("STATUS RUNNING 0.50 0.1 0.2 0.3\n").unwrap();
        assert_eq!(
            frame,
            Esp32Frame::StatusRunning {
                progress: 0.50,
                values: vec![0.1, 0.2, 0.3],
            }
        );
    }

    #[test]
    fn parse_sample_frame() {
        let frame = Esp32Codec::parse_response("SAMPLE 1000 25.5 60.1\n").unwrap();
        assert_eq!(
            frame,
            Esp32Frame::SampleFrame {
                timestamp_us: 1000,
                values: vec![25.5, 60.1],
            }
        );
    }
}
