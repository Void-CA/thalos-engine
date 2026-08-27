use crate::kinematics::jacobian::Jacobian;
#[derive(Debug, Clone)]
pub struct SingularityReport {
    pub det_jtj: f64,
    pub condition_number: f64,
    pub rank: usize,
    pub singular_values: Vec<f64>,
}

impl SingularityReport {
    /// Analiza el Jacobiano lineal (3×n) para detectar singularidades.
    ///
    /// Usa SVD sobre la matriz Jacobiano directamente para obtener
    /// valores singulares, rango efectivo y número de condición.
    pub fn analyze(jacobian: &Jacobian) -> Self {
        let j_lin = jacobian.linear();

        // SVD del Jacobiano (3×n). Nos da exactamente min(3,n)
        // valores singulares.
        let raw_sv = j_lin.singular_values();

        let n = j_lin.ncols();
        let m = j_lin.nrows();

        // Tomamos los primeros min(m,n) valores singulares.
        let n_sv = m.min(n);
        let singular_values: Vec<f64> = raw_sv.iter().take(n_sv).copied().collect();

        // Rango efectivo: valores singulares > 1e-10
        let rank = singular_values.iter().filter(|&&s| s > 1e-10).count();

        // Número de condición: σ_max / σ_min
        let condition_number = if rank > 1 {
            let max_sv = singular_values[0]; // ya vienen ordenados descendente
            let min_sv = singular_values[rank - 1]; // último no-cero
            if min_sv > 1e-14 {
                max_sv / min_sv
            } else {
                f64::INFINITY
            }
        } else if rank == 1 {
            // Un solo valor singular no-cero → condition number ∞
            f64::INFINITY
        } else {
            f64::INFINITY
        };

        // det(JᵀJ) = ∏ σᵢ² para TODOS los valores singulares
        // (incluyendo ceros, que hacen el producto = 0 si hay
        // deficiencia de rango). Para n×n es exactamente det(JᵀJ).
        let det_jtj: f64 = singular_values.iter().map(|s| s * s).product();

        Self {
            det_jtj,
            condition_number,
            rank,
            singular_values,
        }
    }
}
