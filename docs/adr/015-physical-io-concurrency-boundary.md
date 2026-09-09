# ADR: Physical I/O Concurrency Boundary

## Status

Proposed

## Context

Thalos needs to execute physical robot commands through `RobotTransport`, which is a **sync trait**. However, the underlying transport implementations (`SerialTransport`, `TcpTransport`) are **async** because they use Tokio's I/O primitives.

The current `Esp32RobotAdapter` bridges this gap using `tokio::task::block_in_place + block_on`, which:
1. Requires the `rt-multi-thread` Tokio feature (currently only in dev-dependencies)
2. Holds a blocking thread while waiting for async I/O
3. Cannot be cancelled from outside
4. May deadlock if called from a non-Tokio context

## Decision

**Introduce an async `RobotTransport` trait at the port boundary, with `HardwareExecutor` being the sync consumer that runs inside a Tokio task.**

### Architecture

```
┌─────────────────────────────────────────────────┐
│                   Tokio Runtime                  │
│                                                  │
│  ┌──────────────────────────────────────────┐   │
│  │          HardwareExecutor (sync)          │   │
│  │  ┌────────────────────────────────────┐  │   │
│  │  │  tick() → RobotTransport methods   │  │   │
│  │  └────────────────────────────────────┘  │   │
│  └──────────────────┬───────────────────────┘   │
│                     │                            │
│  ┌──────────────────▼───────────────────────┐   │
│  │     AsyncRobotTransport (async bridge)    │   │
│  │  Wraps async Transport, exposes sync API  │   │
│  │  Uses spawn_blocking for I/O operations   │   │
│  └──────────────────┬───────────────────────┘   │
│                     │                            │
│  ┌──────────────────▼───────────────────────┐   │
│  │          Transport (async)                │   │
│  │  SerialTransport / TcpTransport           │   │
│  └──────────────────────────────────────────┘   │
└─────────────────────────────────────────────────┘
```

### Why not `block_in_place`?

| Issue | `block_in_place` | `spawn_blocking` |
|-------|------------------|------------------|
| Requires `rt-multi-thread` | Yes | Yes |
| Can deadlock if called from async | Yes (panics) | No |
| Cancellation | Impossible | Via `JoinHandle::abort()` |
| Thread pool | Uses worker thread | Dedicated blocking pool |
| Multiple sessions | Shares worker threads | Independent blocking threads |

`block_in_place` is designed for when you **already have a long-running sync computation** inside an async task and want to temporarily use the worker thread for blocking I/O. It's not meant for wrapping arbitrary async I/O from sync code.

### Why `spawn_blocking` is better for Thalos

1. **Cancellation**: When a user cancels execution, we can abort the `JoinHandle` and the blocking operation eventually completes or times out.

2. **Isolation**: Each physical transport gets its own blocking thread, preventing one slow robot from blocking another.

3. **Pool saturation**: Tokio's blocking pool has a configurable limit (default: 512 threads). This prevents unbounded thread creation.

4. **Safety**: No risk of deadlocking the Tokio runtime.

## Implementation

### Option A: Make `RobotTransport` async (breaking change)

```rust
#[async_trait]
pub trait RobotTransport: Send + Sync {
    fn state(&self) -> TransportState;
    async fn send(&mut self, command: RobotCommand) -> Result<(), TransportError>;
    async fn stop(&mut self) -> Result<(), TransportError>;
    async fn try_receive_observation(&mut self) -> Result<Option<RobotObservation>, TransportError>;
}
```

**Pros**: Clean, idiomatic, allows proper cancellation.
**Cons**: Major refactor of `HardwareExecutor`, `SimulationRunner`, all test doubles.

### Option B: Keep `RobotTransport` sync, use `spawn_blocking` at call site

```rust
// In HardwareExecutor or execution service
let transport = Arc::new(Mutex::new(esp32_adapter));

// When sending a command
let transport_clone = transport.clone();
let cmd = command.clone();
let result = tokio::task::spawn_blocking(move || {
    let mut guard = transport_clone.lock().unwrap();
    guard.send(cmd)
}).await??;
```

**Pros**: Minimal change to existing traits, `HardwareExecutor` stays sync.
**Cons**: Lock contention if multiple operations need the transport simultaneously.

### Option C: Hybrid — sync trait with async interior (recommended)

```rust
// Keep RobotTransport sync
pub trait RobotTransport: Send + Sync {
    fn state(&self) -> TransportState;
    fn send(&mut self, command: RobotCommand) -> Result<(), TransportError>;
    fn stop(&mut self) -> Result<(), TransportError>;
    fn try_receive_observation(&mut self) -> Result<Option<RobotObservation>, TransportError>;
}

// But Esp32RobotAdapter uses a channel-based interior
pub struct Esp32RobotAdapter {
    // Async I/O happens in a dedicated task
    command_tx: mpsc::Sender<RobotCommand>,
    observation_rx: mpsc::Receiver<RobotObservation>,
    state_rx: watch::Receiver<TransportState>,
}

impl RobotTransport for Esp32RobotAdapter {
    fn send(&mut self, command: RobotCommand) -> Result<(), TransportError> {
        // Non-blocking channel send
        self.command_tx.try_send(command)
            .map_err(|_| TransportError::CommunicationFailure("Channel full".into()))?;
        Ok(())
    }

    fn try_receive_observation(&mut self) -> Result<Option<RobotObservation>, TransportError> {
        // Non-blocking channel receive
        Ok(self.observation_rx.try_recv().ok())
    }
}
```

**Pros**: Sync trait preserved, I/O happens in dedicated async task, proper cancellation.
**Cons**: More complex adapter, requires channel infrastructure.

## Recommendation

**Option B for now, Option C for production.**

Reasoning:
1. Option B is the smallest change that fixes the compilation issue and enables physical testing.
2. Option C is the long-term architecture but requires more design work.
3. We can migrate from B to C without changing the `RobotTransport` trait.

### Immediate action (Step 4)

1. Add `rt-multi-thread` to `thalos-transport` production dependencies
2. Replace `block_in_place` with `spawn_blocking` in `Esp32RobotAdapter`
3. Verify the transport compiles and tests pass

### Future action (after physical E2E)

1. Evaluate Option C based on real-world latency measurements
2. If Option B works well enough, defer Option C

## Consequences

### Positive
- Transport crate compiles without `block_in_place` issues
- Physical I/O is properly isolated from the async runtime
- Cancellation becomes possible (via timeout or abort)

### Negative
- `spawn_blocking` adds a small overhead per I/O operation (~1-5μs)
- Blocking thread pool has a finite size; very many robots could exhaust it

### Risks
- If a physical transport blocks indefinitely (e.g., serial port hang), the blocking thread is stuck. Mitigation: use `tokio::time::timeout` around the `spawn_blocking` call.
