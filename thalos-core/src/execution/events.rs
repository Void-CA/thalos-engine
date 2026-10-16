use serde::{Deserialize, Serialize};

use crate::deviation::TickDeviation;
use crate::execution::session::{ExecutionSessionState, TickResult};
use crate::ids::ExecutionSessionId;

/// Invariante temporal del tick: distingue los CUATRO instantes del recorrido
/// `observación → evaluación` y declara el RELOJ que origina cada marca.
///
/// ```text
/// Tramo                    Campo              Reloj
/// -----------------------  -----------------  --------------------------------
/// muestreo del dispositivo `sampled_at_us`    reloj del ORIGEN  (puede faltar)
/// recepción en el bridge   `bridged_at_us`    reloj del BRIDGE  (puede faltar)
/// ingesta en el runtime    `ingested_at_us`   reloj del HOST    (siempre presente)
/// evaluación del tick      `evaluated_at_us`  reloj del HOST    (siempre presente)
/// ```
///
/// Reglas:
/// - `sampled_at_us`/`bridged_at_us` son `None` cuando la marca **no existe o no
///   es fiable**. NUNCA se rellenan con la hora del host: una marca ausente no es
///   una marca cero.
/// - Las marcas de origen y de bridge pertenecen a relojes que **no** son el del
///   host y por tanto NO son comparables con él sin una correlación de reloj
///   explícita. En particular, `sampled_at_us` no participa en comparaciones de
///   frescura contra el reloj del host hasta que exista esa correlación.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct TemporalInvariants {
    /// Instante de muestreo en el ORIGEN (reloj del dispositivo). `None` = el
    /// dispositivo no ofrece una marca fiable.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub sampled_at_us: Option<u64>,
    /// Instante de recepción en el BRIDGE de transporte. `None` = sin bridge o
    /// el bridge no marca la recepción.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub bridged_at_us: Option<u64>,
    /// Instante de ingesta en la capa de adquisición del RUNTIME (reloj del host).
    pub ingested_at_us: u64,
    /// Instante de evaluación del tick de control (reloj del host).
    pub evaluated_at_us: u64,
}

impl TemporalInvariants {
    /// Marcas exclusivas del runtime: dispositivo y bridge DESCONOCIDOS.
    ///
    /// Es el constructor honesto cuando la fuente no aporta marca temporal.
    /// Sustituye a la forma anterior `(sampled, received, evaluated)`, que
    /// fabricaba un `received` igual a la evaluación.
    pub fn new(ingested_at_us: u64, evaluated_at_us: u64) -> Self {
        Self {
            sampled_at_us: None,
            bridged_at_us: None,
            ingested_at_us,
            evaluated_at_us,
        }
    }

    /// Adjunta el instante de muestreo del dispositivo (reloj del origen). Sólo
    /// debe llamarse cuando la fuente proporciona una marca fiable.
    pub fn with_device_sample(mut self, sampled_at_us: u64) -> Self {
        self.sampled_at_us = Some(sampled_at_us);
        self
    }

    /// Adjunta el instante de recepción del bridge. Sólo debe llamarse cuando un
    /// bridge marca realmente la recepción.
    pub fn with_bridge_receipt(mut self, bridged_at_us: u64) -> Self {
        self.bridged_at_us = Some(bridged_at_us);
        self
    }
}

/// Contrato semántico de eventos de dominio emitidos por el runtime.
///
/// Principio rector: "El estado de dominio es autoritativo; los eventos son observaciones de sus transiciones".
// `TickEvaluated` carries the full `TickResult` by value on purpose: this is a
// public event-vocabulary type and boxing it would change its shape for every
// consumer (runtime bus, persistence, DTO mapping). The size difference is accepted.
#[allow(clippy::large_enum_variant)]
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum ExecutionEvent {
    /// Instanciación de una nueva sesión de ejecución.
    SessionCreated {
        session_id: ExecutionSessionId,
        program_id: String,
        timestamp_us: u64,
    },
    /// Transición autoritativa en la máquina de estados de ciclo de vida.
    LifecycleChanged {
        session_id: ExecutionSessionId,
        previous: ExecutionSessionState,
        current: ExecutionSessionState,
        timestamp_us: u64,
    },
    /// Agregado completo de observabilidad emitido tras la evaluación y ejecución (post-act) del tick k.
    TickEvaluated {
        session_id: ExecutionSessionId,
        result: TickResult,
        temporal: TemporalInvariants,
    },
    /// Evidencia de que una expectativa declarada NO se cumplió en el tick.
    ///
    /// Se emite sólo cuando la comparación expected↔observed (A4) produce al
    /// menos una desviación (A5). `Unknown` nunca llega aquí: la ausencia de
    /// evidencia no es una violación. Es evidencia, no una respuesta.
    DeviationDetected {
        session_id: ExecutionSessionId,
        deviation: TickDeviation,
        timestamp_us: u64,
    },
    /// Fallo crítico durante el ciclo de vida o la ejecución de una acción.
    SessionFaulted {
        session_id: ExecutionSessionId,
        reason: String,
        timestamp_us: u64,
    },
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The runtime-only constructor must NOT fabricate device/bridge marks: an
    /// absent mark is `None`, never the host clock.
    #[test]
    fn runtime_only_marks_leave_device_and_bridge_unknown() {
        let t = TemporalInvariants::new(100, 200);
        assert_eq!(t.sampled_at_us, None);
        assert_eq!(t.bridged_at_us, None);
        assert_eq!(t.ingested_at_us, 100);
        assert_eq!(t.evaluated_at_us, 200);
    }

    /// Device and bridge marks are attached explicitly, never inferred.
    #[test]
    fn device_and_bridge_marks_are_attached_explicitly() {
        let t = TemporalInvariants::new(300, 400)
            .with_device_sample(100)
            .with_bridge_receipt(250);
        assert_eq!(t.sampled_at_us, Some(100));
        assert_eq!(t.bridged_at_us, Some(250));
        assert_eq!(t.ingested_at_us, 300);
        assert_eq!(t.evaluated_at_us, 400);
    }

    /// Unknown marks are omitted on the wire and decode back to `None`, so an
    /// absent timestamp is distinguishable from a zero timestamp.
    #[test]
    fn unknown_marks_are_omitted_and_decode_to_none() {
        let t = TemporalInvariants::new(10, 20);
        let json = serde_json::to_string(&t).unwrap();
        assert!(
            !json.contains("sampled_at_us"),
            "an absent device mark must not be serialized: {json}"
        );
        assert!(
            !json.contains("bridged_at_us"),
            "an absent bridge mark must not be serialized: {json}"
        );

        let back: TemporalInvariants = serde_json::from_str(&json).unwrap();
        assert_eq!(back, t);
        assert_eq!(back.sampled_at_us, None);
    }
}
