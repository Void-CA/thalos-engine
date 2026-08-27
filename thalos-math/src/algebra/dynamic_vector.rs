use nalgebra as na;
use std::ops::{Add, AddAssign, Index, IndexMut, Mul, Neg, Sub};

// DynamicVector se construye desde nalgebra internamente.
// El campo es pub(crate) para que DynamicMatrix (en otro archivo
// del mismo módulo) pueda construir DynamicVector desde na::DVector.
#[derive(Debug, Clone)]
pub struct DynamicVector(pub(crate) na::DVector<f64>);

impl DynamicVector {
    /// Crea un vector columna de `n` ceros.
    pub fn zeros(n: usize) -> Self {
        Self(na::DVector::<f64>::zeros(n))
    }

    /// Crea un vector desde un slice (copia los datos).
    pub fn from_column_slice(data: &[f64]) -> Self {
        Self(na::DVector::<f64>::from_column_slice(data))
    }

    /// Crea un vector desde un `Vec<f64>`.
    pub fn from_vec(data: Vec<f64>) -> Self {
        Self(na::DVector::<f64>::from_vec(data))
    }

    /// Cantidad de elementos.
    pub fn len(&self) -> usize {
        self.0.len()
    }

    /// Devuelve `true` si el vector está vacío.
    pub fn is_empty(&self) -> bool {
        self.0.is_empty()
    }

    /// Norma euclídea (L2).
    pub fn magnitude(&self) -> f64 {
        self.0.norm()
    }

    /// Norma euclídea al cuadrado.
    pub fn magnitude_squared(&self) -> f64 {
        self.0.norm_squared()
    }

    /// Devuelve una vista del slice subyacente.
    pub fn as_slice(&self) -> &[f64] {
        self.0.as_slice()
    }

    /// Versión mutable del slice subyacente.
    pub fn as_mut_slice(&mut self) -> &mut [f64] {
        self.0.as_mut_slice()
    }

    /// Acceso inmutable al `na::DVector<f64>` interno.
    pub fn inner(&self) -> &na::DVector<f64> {
        &self.0
    }

    /// Consume el wrapper y devuelve el `na::DVector<f64>` interno.
    pub fn into_inner(self) -> na::DVector<f64> {
        self.0
    }
}

// ─── Indexing ───────────────────────────────────────────────────

impl Index<usize> for DynamicVector {
    type Output = f64;

    fn index(&self, index: usize) -> &Self::Output {
        &self.0[index]
    }
}

impl IndexMut<usize> for DynamicVector {
    fn index_mut(&mut self, index: usize) -> &mut Self::Output {
        &mut self.0[index]
    }
}

// ─── Operaciones aritméticas ───────────────────────────────────

impl Add for DynamicVector {
    type Output = Self;

    fn add(self, rhs: Self) -> Self::Output {
        Self(self.0 + rhs.0)
    }
}

impl Add<&DynamicVector> for DynamicVector {
    type Output = DynamicVector;

    fn add(self, rhs: &DynamicVector) -> Self::Output {
        DynamicVector(self.0 + &rhs.0)
    }
}

impl AddAssign for DynamicVector {
    fn add_assign(&mut self, rhs: Self) {
        self.0 += rhs.0;
    }
}

impl AddAssign<&DynamicVector> for DynamicVector {
    fn add_assign(&mut self, rhs: &DynamicVector) {
        self.0 += &rhs.0;
    }
}

impl Sub for DynamicVector {
    type Output = Self;

    fn sub(self, rhs: Self) -> Self::Output {
        Self(self.0 - rhs.0)
    }
}

impl Neg for DynamicVector {
    type Output = Self;

    fn neg(self) -> Self::Output {
        Self(-self.0)
    }
}

// Escalar * Vector
impl Mul<f64> for DynamicVector {
    type Output = Self;

    fn mul(self, rhs: f64) -> Self::Output {
        Self(self.0 * rhs)
    }
}

// Vector * Escalar
impl Mul<DynamicVector> for f64 {
    type Output = DynamicVector;

    fn mul(self, rhs: DynamicVector) -> Self::Output {
        DynamicVector(self * rhs.0)
    }
}

impl Mul<f64> for &DynamicVector {
    type Output = DynamicVector;

    fn mul(self, rhs: f64) -> Self::Output {
        DynamicVector(self.0.clone() * rhs)
    }
}

impl Mul<&DynamicVector> for f64 {
    type Output = DynamicVector;

    fn mul(self, rhs: &DynamicVector) -> Self::Output {
        DynamicVector(self * &rhs.0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn zeros_creates_correct_length() {
        let v = DynamicVector::zeros(5);
        assert_eq!(v.len(), 5);
        for i in 0..5 {
            assert!((v[i] - 0.0).abs() < 1e-15);
        }
    }

    #[test]
    fn from_column_slice() {
        let data = vec![1.0, 2.0, 3.0];
        let v = DynamicVector::from_column_slice(&data);
        assert_eq!(v.len(), 3);
        assert!((v[0] - 1.0).abs() < 1e-15);
        assert!((v[1] - 2.0).abs() < 1e-15);
        assert!((v[2] - 3.0).abs() < 1e-15);
    }

    #[test]
    fn from_vec() {
        let v = DynamicVector::from_vec(vec![4.0, 5.0, 6.0, 7.0]);
        assert_eq!(v.len(), 4);
        assert!((v[3] - 7.0).abs() < 1e-15);
    }

    #[test]
    fn magnitude() {
        let v = DynamicVector::from_vec(vec![3.0, 4.0]);
        assert!((v.magnitude() - 5.0).abs() < 1e-15);
    }

    #[test]
    fn add() {
        let a = DynamicVector::from_vec(vec![1.0, 2.0]);
        let b = DynamicVector::from_vec(vec![3.0, 4.0]);
        let c = a + b;
        assert!((c[0] - 4.0).abs() < 1e-15);
        assert!((c[1] - 6.0).abs() < 1e-15);
    }

    #[test]
    fn add_assign() {
        let mut a = DynamicVector::from_vec(vec![1.0, 2.0]);
        let b = DynamicVector::from_vec(vec![3.0, 4.0]);
        a += b;
        assert!((a[0] - 4.0).abs() < 1e-15);
        assert!((a[1] - 6.0).abs() < 1e-15);
    }

    #[test]
    fn index_mut() {
        let mut v = DynamicVector::zeros(3);
        v[1] = 42.0;
        assert!((v[1] - 42.0).abs() < 1e-15);
    }

    #[test]
    fn scalar_multiplication() {
        let v = DynamicVector::from_vec(vec![1.0, 2.0, 3.0]);
        let r = v * 2.0;
        assert!((r[0] - 2.0).abs() < 1e-15);
        assert!((r[1] - 4.0).abs() < 1e-15);
        assert!((r[2] - 6.0).abs() < 1e-15);
    }

    #[test]
    fn scalar_left_multiplication() {
        let v = DynamicVector::from_vec(vec![1.0, 2.0]);
        let r = 3.0 * v;
        assert!((r[0] - 3.0).abs() < 1e-15);
        assert!((r[1] - 6.0).abs() < 1e-15);
    }

    #[test]
    fn as_slice() {
        let v = DynamicVector::from_vec(vec![1.0, 2.0, 3.0]);
        assert_eq!(v.as_slice(), &[1.0, 2.0, 3.0]);
    }

    #[test]
    fn from_column_slice_mutability() {
        let mut v = DynamicVector::from_column_slice(&[1.0, 2.0]);
        v[0] = 10.0;
        assert!((v[0] - 10.0).abs() < 1e-15);
    }
}
