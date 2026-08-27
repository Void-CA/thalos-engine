use crate::DynamicVector;
use nalgebra as na;
use std::ops::{Add, Index, IndexMut, Mul};

/// Matriz dinámica (heap-allocated) que envuelve `nalgebra::DMatrix<f64>`.
///
/// Es el equivalente a `DMatrix<f64>` pero dentro del dominio de Thalos,
/// para no exponer nalgebra directamente en la API pública.
#[derive(Debug, Clone)]
pub struct DynamicMatrix(na::DMatrix<f64>);

impl DynamicMatrix {
    /// Crea una matriz de `rows × cols` llena de ceros.
    pub fn zeros(rows: usize, cols: usize) -> Self {
        Self(na::DMatrix::<f64>::zeros(rows, cols))
    }

    /// Crea una matriz identidad de `n × n`.
    pub fn identity(n: usize, m: usize) -> Self {
        assert_eq!(n, m, "identity matrix must be square");
        Self(na::DMatrix::<f64>::identity(n, n))
    }

    /// Cantidad de filas.
    pub fn nrows(&self) -> usize {
        self.0.nrows()
    }

    /// Cantidad de columnas.
    pub fn ncols(&self) -> usize {
        self.0.ncols()
    }

    /// Transpuesta.
    pub fn transpose(&self) -> Self {
        Self(self.0.transpose())
    }

    /// Clona la matriz (igual que `.clone()`, nombre explícito para
    /// compatibilidad con APIs existentes).
    pub fn clone_owned(&self) -> Self {
        self.clone()
    }

    /// Inversa de la matriz (solo cuadradas).
    /// Devuelve `None` si la matriz es singular.
    pub fn try_inverse(&self) -> Option<Self> {
        self.0.clone().try_inverse().map(Self)
    }

    /// Determinante (solo cuadradas).
    pub fn determinant(&self) -> f64 {
        self.0.determinant()
    }

    /// Descomposición SVD completa.
    /// Devuelve los valores singulares como `Vec<f64>`.
    pub fn singular_values(&self) -> Vec<f64> {
        let svd = self.0.clone().svd(true, true);
        svd.singular_values.iter().copied().collect()
    }

    /// Moore-Penrose pseudo-inverse via SVD.
    ///
    /// Computes `J⁺ = V · Σ⁺ · Uᵀ` where singular values above `tolerance`
    /// are inverted (`1/σᵢ`) and all others are treated as zero.
    /// Returns `None` when ALL singular values are at or below `tolerance`
    /// (effectively a zero matrix).
    pub fn pseudo_inverse(&self, tolerance: f64) -> Option<Self> {
        let svd = self.0.clone().svd(true, true);
        let u = svd.u.as_ref()?;
        let v_t = svd.v_t.as_ref()?;
        let s = &svd.singular_values;

        let k = s.len();

        // Build Σ⁺: k×k diagonal with 1/σᵢ for σᵢ > tolerance
        let mut sigma_plus = DynamicMatrix::zeros(k, k);
        let mut any_above_tolerance = false;
        for i in 0..k {
            if s[i] > tolerance {
                sigma_plus.0[(i, i)] = 1.0 / s[i];
                any_above_tolerance = true;
            }
        }
        if !any_above_tolerance {
            return None;
        }

        // J⁺ = V · Σ⁺ · Uᵀ
        // v_t is Vᵀ (k×n), so V = (Vᵀ)ᵀ = v_tᵀ (n×k)
        let v = v_t.transpose();
        Some(DynamicMatrix(v * sigma_plus.0 * u.transpose()))
    }

    /// Acceso inmutable al `na::DMatrix<f64>` interno.
    pub fn inner(&self) -> &na::DMatrix<f64> {
        &self.0
    }

    /// Consume el wrapper y devuelve el `na::DMatrix<f64>` interno.
    pub fn into_inner(self) -> na::DMatrix<f64> {
        self.0
    }
}

// ─── Indexing ───────────────────────────────────────────────────

impl Index<(usize, usize)> for DynamicMatrix {
    type Output = f64;

    fn index(&self, (row, col): (usize, usize)) -> &Self::Output {
        &self.0[(row, col)]
    }
}

impl IndexMut<(usize, usize)> for DynamicMatrix {
    fn index_mut(&mut self, (row, col): (usize, usize)) -> &mut Self::Output {
        &mut self.0[(row, col)]
    }
}

// ─── Operaciones aritméticas ───────────────────────────────────

/// Matriz * Matriz
impl Mul for DynamicMatrix {
    type Output = Self;

    fn mul(self, rhs: Self) -> Self::Output {
        Self(self.0 * rhs.0)
    }
}

/// &Matriz * &Matriz
impl Mul<&DynamicMatrix> for &DynamicMatrix {
    type Output = DynamicMatrix;

    fn mul(self, rhs: &DynamicMatrix) -> Self::Output {
        DynamicMatrix(self.0.clone() * &rhs.0)
    }
}

/// &Matriz * Matriz
impl Mul<DynamicMatrix> for &DynamicMatrix {
    type Output = DynamicMatrix;

    fn mul(self, rhs: DynamicMatrix) -> Self::Output {
        DynamicMatrix(self.0.clone() * rhs.0)
    }
}

/// Matriz * &Matriz
impl Mul<&DynamicMatrix> for DynamicMatrix {
    type Output = DynamicMatrix;

    fn mul(self, rhs: &DynamicMatrix) -> Self::Output {
        DynamicMatrix(self.0 * &rhs.0)
    }
}

/// Matriz * Vector (columna)
impl Mul<DynamicVector> for DynamicMatrix {
    type Output = DynamicVector;

    fn mul(self, rhs: DynamicVector) -> Self::Output {
        DynamicVector(self.0 * rhs.into_inner())
    }
}

/// &Matriz * Vector
impl Mul<DynamicVector> for &DynamicMatrix {
    type Output = DynamicVector;

    fn mul(self, rhs: DynamicVector) -> Self::Output {
        DynamicVector(self.0.clone() * rhs.into_inner())
    }
}

/// &Matriz * &Vector
impl Mul<&DynamicVector> for &DynamicMatrix {
    type Output = DynamicVector;

    fn mul(self, rhs: &DynamicVector) -> Self::Output {
        DynamicVector(self.0.clone() * rhs.inner())
    }
}

/// Escalar * Matriz
impl Mul<f64> for DynamicMatrix {
    type Output = Self;

    fn mul(self, rhs: f64) -> Self::Output {
        Self(self.0 * rhs)
    }
}

/// f64 * Matriz
impl Mul<DynamicMatrix> for f64 {
    type Output = DynamicMatrix;

    fn mul(self, rhs: DynamicMatrix) -> Self::Output {
        DynamicMatrix(self * rhs.0)
    }
}

impl Mul<f64> for &DynamicMatrix {
    type Output = DynamicMatrix;

    fn mul(self, rhs: f64) -> Self::Output {
        DynamicMatrix(self.0.clone() * rhs)
    }
}

/// Matriz + Matriz
impl Add for DynamicMatrix {
    type Output = Self;

    fn add(self, rhs: Self) -> Self::Output {
        Self(self.0 + rhs.0)
    }
}

/// &Matriz + Matriz
impl Add<DynamicMatrix> for &DynamicMatrix {
    type Output = DynamicMatrix;

    fn add(self, rhs: DynamicMatrix) -> Self::Output {
        DynamicMatrix(self.0.clone() + rhs.0)
    }
}

/// Matriz + &Matriz
impl Add<&DynamicMatrix> for DynamicMatrix {
    type Output = DynamicMatrix;

    fn add(self, rhs: &DynamicMatrix) -> Self::Output {
        DynamicMatrix(self.0 + &rhs.0)
    }
}

/// f64 * &Matriz
impl Mul<&DynamicMatrix> for f64 {
    type Output = DynamicMatrix;

    fn mul(self, rhs: &DynamicMatrix) -> Self::Output {
        DynamicMatrix(self * &rhs.0)
    }
}

/// &Matriz + &Matriz
impl Add<&DynamicMatrix> for &DynamicMatrix {
    type Output = DynamicMatrix;

    fn add(self, rhs: &DynamicMatrix) -> Self::Output {
        DynamicMatrix(self.0.clone() + &rhs.0)
    }
}

// ─── Conversion desde na::DMatrix ──────────────────────────────

impl From<na::DMatrix<f64>> for DynamicMatrix {
    fn from(m: na::DMatrix<f64>) -> Self {
        Self(m)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::constants::EPS;

    // ─── pseudo_inverse tests (Strict TDD: RED → GREEN → Triangulate) ─────

    #[test]
    fn pseudo_inverse_full_rank_square() {
        // Task 1.2: Full-rank square → true inverse
        // 3×3 invertible matrix with det ≠ 0
        let a = DynamicMatrix::from(na::DMatrix::<f64>::from_row_slice(
            3,
            3,
            &[2.0, 1.0, 1.0, 1.0, 3.0, 2.0, 1.0, 0.0, 2.0],
        ));
        let pinv = a
            .pseudo_inverse(1e-12)
            .expect("full-rank square should have pseudo-inverse");

        // A · A⁺ ≈ I₃
        let a_pinv = &a * &pinv;
        // A⁺ · A ≈ I₃
        let pinv_a = &pinv * &a;

        let i3 = DynamicMatrix::identity(3, 3);
        for i in 0..3 {
            for j in 0..3 {
                let diff_a = (a_pinv[(i, j)] - i3[(i, j)]).abs();
                let diff_b = (pinv_a[(i, j)] - i3[(i, j)]).abs();
                assert!(
                    diff_a < 1e-10,
                    "A·A⁺[{i},{j}] = {}, expected ~{}",
                    a_pinv[(i, j)],
                    i3[(i, j)]
                );
                assert!(
                    diff_b < 1e-10,
                    "A⁺·A[{i},{j}] = {}, expected ~{}",
                    pinv_a[(i, j)],
                    i3[(i, j)]
                );
            }
        }
    }

    #[test]
    fn pseudo_inverse_full_rank_tall() {
        // Task 1.3: Full-rank tall (4×3) → left pseudo-inverse: A⁺ · A ≈ I₃
        let a = DynamicMatrix::from(na::DMatrix::<f64>::from_row_slice(
            4,
            3,
            &[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 0.0],
        ));
        let pinv = a
            .pseudo_inverse(1e-12)
            .expect("full-rank tall should have pseudo-inverse");

        // A⁺ · A ≈ I₃
        let pinv_a = &pinv * &a;
        let i3 = DynamicMatrix::identity(3, 3);
        for i in 0..3 {
            for j in 0..3 {
                let diff = (pinv_a[(i, j)] - i3[(i, j)]).abs();
                assert!(
                    diff < 1e-10,
                    "A⁺·A[{i},{j}] = {}, expected ~{}",
                    pinv_a[(i, j)],
                    i3[(i, j)]
                );
            }
        }
    }

    #[test]
    fn pseudo_inverse_penrose_conditions() {
        // Task 1.4: Verify all four Penrose conditions for a 3×2 matrix
        let a = DynamicMatrix::from(na::DMatrix::<f64>::from_row_slice(
            3,
            2,
            &[1.0, 2.0, 3.0, 4.0, 5.0, 6.0],
        ));
        let pinv = a
            .pseudo_inverse(1e-12)
            .expect("full-rank rectangular should have pseudo-inverse");

        // 1. A·A⁺·A = A
        let a_pinv_a = &a * (&pinv * &a);
        for i in 0..3 {
            for j in 0..2 {
                let diff = (a_pinv_a[(i, j)] - a[(i, j)]).abs();
                assert!(
                    diff < 1e-10,
                    "A·A⁺·A[{i},{j}] = {}, expected ~{}",
                    a_pinv_a[(i, j)],
                    a[(i, j)]
                );
            }
        }

        // 2. A⁺·A·A⁺ = A⁺
        let pinv_a_pinv = &pinv * (&a * &pinv);
        for i in 0..2 {
            for j in 0..3 {
                let diff = (pinv_a_pinv[(i, j)] - pinv[(i, j)]).abs();
                assert!(
                    diff < 1e-10,
                    "A⁺·A·A⁺[{i},{j}] = {}, expected ~{}",
                    pinv_a_pinv[(i, j)],
                    pinv[(i, j)]
                );
            }
        }

        // 3. (A·A⁺)ᵀ = A·A⁺  (symmetry in the row space)
        let a_pinv_t = (&a * &pinv).transpose();
        let a_pinv = &a * &pinv;
        for i in 0..3 {
            for j in 0..3 {
                let diff = (a_pinv_t[(i, j)] - a_pinv[(i, j)]).abs();
                assert!(
                    diff < 1e-10,
                    "(A·A⁺)ᵀ[{i},{j}] = {}, expected ~{}",
                    a_pinv_t[(i, j)],
                    a_pinv[(i, j)]
                );
            }
        }

        // 4. (A⁺·A)ᵀ = A⁺·A  (symmetry in the column space)
        let pinv_a = &pinv * &a;
        let pinv_a_t = pinv_a.transpose();
        for i in 0..2 {
            for j in 0..2 {
                let diff = (pinv_a_t[(i, j)] - pinv_a[(i, j)]).abs();
                assert!(
                    diff < 1e-10,
                    "(A⁺·A)ᵀ[{i},{j}] = {}, expected ~{}",
                    pinv_a_t[(i, j)],
                    pinv_a[(i, j)]
                );
            }
        }
    }

    #[test]
    fn pseudo_inverse_singular_matrix() {
        // Task 1.5: All singular values below tolerance → None
        // Matrix with values so small that no singular value exceeds tolerance
        let a = DynamicMatrix::from(na::DMatrix::<f64>::from_row_slice(
            3,
            3,
            &[1e-15, 0.0, 0.0, 0.0, 1e-15, 0.0, 0.0, 0.0, 1e-15],
        ));
        assert!(
            a.pseudo_inverse(1e-12).is_none(),
            "matrix with all singular values below tolerance should return None"
        );
    }

    #[test]
    fn pseudo_inverse_zero_matrix() {
        // Task 1.6: Zero matrix → None
        let a = DynamicMatrix::zeros(3, 3);
        assert!(
            a.pseudo_inverse(1e-12).is_none(),
            "zero matrix should return None"
        );
    }

    #[test]
    fn zeros_creates_correct_dimensions() {
        let m = DynamicMatrix::zeros(3, 4);
        assert_eq!(m.nrows(), 3);
        assert_eq!(m.ncols(), 4);
        for i in 0..3 {
            for j in 0..4 {
                assert!((m[(i, j)] - 0.0).abs() < EPS);
            }
        }
    }

    #[test]
    fn identity() {
        let i = DynamicMatrix::identity(3, 3);
        assert!((i[(0, 0)] - 1.0).abs() < EPS);
        assert!((i[(1, 1)] - 1.0).abs() < EPS);
        assert!((i[(2, 2)] - 1.0).abs() < EPS);
        assert!((i[(0, 1)] - 0.0).abs() < EPS);
        assert!((i[(1, 0)] - 0.0).abs() < EPS);
    }

    #[test]
    fn index_mut() {
        let mut m = DynamicMatrix::zeros(2, 2);
        m[(0, 1)] = 42.0;
        assert!((m[(0, 1)] - 42.0).abs() < EPS);
    }

    #[test]
    fn transpose() {
        let m = DynamicMatrix::from(na::DMatrix::<f64>::from_row_slice(
            2,
            3,
            &[1.0, 2.0, 3.0, 4.0, 5.0, 6.0],
        ));
        let t = m.transpose();
        assert_eq!(t.nrows(), 3);
        assert_eq!(t.ncols(), 2);
        assert!((t[(0, 1)] - 4.0).abs() < EPS);
    }

    #[test]
    fn matrix_vector_mul() {
        let m = DynamicMatrix::from(na::DMatrix::<f64>::from_row_slice(
            2,
            2,
            &[1.0, 2.0, 3.0, 4.0],
        ));
        let v = DynamicVector::from_vec(vec![2.0, 3.0]);
        let r = m * v;
        assert!((r[0] - 8.0).abs() < EPS); // 1*2 + 2*3 = 8
        assert!((r[1] - 18.0).abs() < EPS); // 3*2 + 4*3 = 18
    }

    #[test]
    fn matrix_matrix_mul() {
        let a = DynamicMatrix::from(na::DMatrix::<f64>::from_row_slice(
            2,
            3,
            &[1.0, 2.0, 3.0, 4.0, 5.0, 6.0],
        ));
        let b = DynamicMatrix::from(na::DMatrix::<f64>::from_row_slice(
            3,
            2,
            &[7.0, 8.0, 9.0, 10.0, 11.0, 12.0],
        ));
        let c = a * b;
        assert_eq!(c.nrows(), 2);
        assert_eq!(c.ncols(), 2);
        // c[0,0] = 1*7 + 2*9 + 3*11 = 58
        assert!((c[(0, 0)] - 58.0).abs() < EPS);
        // c[1,0] = 4*7 + 5*9 + 6*11 = 139
        assert!((c[(1, 0)] - 139.0).abs() < EPS);
    }

    #[test]
    fn try_inverse() {
        let m = DynamicMatrix::from(na::DMatrix::<f64>::from_row_slice(
            2,
            2,
            &[4.0, 7.0, 2.0, 6.0],
        ));
        let inv = m.try_inverse().expect("should be invertible");
        // Identidad: m * inv ≈ I
        let prod = DynamicMatrix::from(na::DMatrix::<f64>::from_row_slice(
            2,
            2,
            &[4.0, 7.0, 2.0, 6.0],
        )) * inv;
        assert!((prod[(0, 0)] - 1.0).abs() < EPS);
        assert!((prod[(1, 1)] - 1.0).abs() < EPS);
        assert!((prod[(0, 1)] - 0.0).abs() < EPS);
        assert!((prod[(1, 0)] - 0.0).abs() < EPS);
    }

    #[test]
    fn singular_values() {
        let m = DynamicMatrix::from(na::DMatrix::<f64>::from_row_slice(
            2,
            2,
            &[1.0, 0.0, 0.0, 2.0],
        ));
        let sv = m.singular_values();
        assert_eq!(sv.len(), 2);
        assert!((sv[0] - 2.0).abs() < EPS);
        assert!((sv[1] - 1.0).abs() < EPS);
    }

    #[test]
    fn scalar_mul() {
        let m = DynamicMatrix::from(na::DMatrix::<f64>::from_row_slice(
            2,
            2,
            &[1.0, 2.0, 3.0, 4.0],
        ));
        let r = m * 2.0;
        assert!((r[(0, 0)] - 2.0).abs() < EPS);
        assert!((r[(1, 1)] - 8.0).abs() < EPS);
    }
}
