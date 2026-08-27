use crate::robot::link::LinkId;
use std::collections::HashSet;

#[derive(Debug, Clone, Default)]
pub struct CollisionMatrix {
    ignored_pairs: HashSet<(LinkId, LinkId)>,
}

impl CollisionMatrix {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn ignore(&mut self, a: LinkId, b: LinkId) {
        let (lo, hi) = if a < b { (a, b) } else { (b, a) };
        self.ignored_pairs.insert((lo, hi));
    }

    pub fn is_ignored(&self, a: LinkId, b: LinkId) -> bool {
        let (lo, hi) = if a < b { (a, b) } else { (b, a) };
        self.ignored_pairs.contains(&(lo, hi))
    }

    pub fn ignored_pairs(&self) -> impl Iterator<Item = &(LinkId, LinkId)> {
        self.ignored_pairs.iter()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ignore_pair_is_symmetric() {
        let mut matrix = CollisionMatrix::new();
        matrix.ignore(1, 2);
        assert!(matrix.is_ignored(1, 2));
        assert!(matrix.is_ignored(2, 1));
    }

    #[test]
    fn non_ignored_pair_returns_false() {
        let matrix = CollisionMatrix::new();
        assert!(!matrix.is_ignored(0, 1));
    }
}
