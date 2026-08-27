/// Estado de convergencia del solver de cinemática inversa.
#[derive(Debug, Clone, PartialEq)]
pub enum IKStatus {
    Converged,
    MaxIterations,
}

impl IKStatus {
    pub fn is_converged(&self) -> bool {
        matches!(self, IKStatus::Converged)
    }
}

#[derive(Debug, Clone)]
pub struct IKResult {
    pub q: Vec<f64>,
    pub status: IKStatus,
    pub iterations: usize,
    pub final_error: f64,
    pub error_history: Option<Vec<f64>>,
}

impl IKResult {
    /// Construye un resultado con estado `Converged`.
    pub fn converged(
        q: Vec<f64>,
        iterations: usize,
        final_error: f64,
        error_history: Option<Vec<f64>>,
    ) -> Self {
        Self {
            q,
            status: IKStatus::Converged,
            iterations,
            final_error,
            error_history,
        }
    }

    /// Construye un resultado con estado `MaxIterations`.
    pub fn max_iterations(
        q: Vec<f64>,
        iterations: usize,
        final_error: f64,
        error_history: Option<Vec<f64>>,
    ) -> Self {
        Self {
            q,
            status: IKStatus::MaxIterations,
            iterations,
            final_error,
            error_history,
        }
    }
}
