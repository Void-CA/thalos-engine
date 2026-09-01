pub struct VisualPrecision {
    pub epsilon_zero: f64,
    pub decimal_places: usize,
}

impl Default for VisualPrecision {
    fn default() -> Self {
        Self {
            epsilon_zero: 1e-10,
            decimal_places: 6,
        }
    }
}

impl VisualPrecision {
    pub fn normalize(&self, val: f64) -> f64 {
        if val.abs() < self.epsilon_zero {
            0.0
        } else {
            let factor = 10_f64.powi(self.decimal_places as i32);
            (val * factor).round() / factor
        }
    }

    pub fn normalize_3(&self, arr: &mut [f64; 3]) {
        for v in arr.iter_mut() {
            *v = self.normalize(*v);
        }
    }

    pub fn normalize_4(&self, arr: &mut [f64; 4]) {
        for v in arr.iter_mut() {
            *v = self.normalize(*v);
        }
    }
}
