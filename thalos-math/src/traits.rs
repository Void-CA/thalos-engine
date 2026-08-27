pub trait Dot<Rhs = Self> {
    type Output;
    fn dot(self, rhs: Rhs) -> Self::Output;
}

pub trait Cross<Rhs = Self> {
    type Output;
    fn cross(self, rhs: Rhs) -> Self::Output;
}
