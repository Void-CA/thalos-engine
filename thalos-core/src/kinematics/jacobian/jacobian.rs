use thalos_math::DynamicMatrix;

pub trait JacobianSolver {
    fn evaluate(&self, q: &[f64]) -> Jacobian;
}

pub struct Jacobian {
    pub linear: DynamicMatrix,
    pub angular: DynamicMatrix,
}

impl Jacobian {
    pub fn new(linear: DynamicMatrix, angular: DynamicMatrix) -> Self {
        Self { linear, angular }
    }

    pub fn linear(&self) -> &DynamicMatrix {
        &self.linear
    }

    pub fn position(&self) -> &DynamicMatrix {
        &self.linear
    }

    pub fn angular(&self) -> &DynamicMatrix {
        &self.angular
    }

    /// Jacobiano completo 6×n: stackea linear (3×n) + angular (3×n).
    ///
    /// Útil para solvers que usan error de pose completo
    /// (posición + orientación), como `DampedLeastSquaresSolver`.
    pub fn full(&self) -> DynamicMatrix {
        let mut full = DynamicMatrix::zeros(6, self.linear.ncols());
        for j in 0..self.linear.ncols() {
            for i in 0..3 {
                full[(i, j)] = self.linear[(i, j)];
                full[(i + 3, j)] = self.angular[(i, j)];
            }
        }
        full
    }
}
