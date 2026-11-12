use serde::{Deserialize, Serialize};

use crate::span::Span;

/// A syntactic type annotation, exactly as written in the source.
///
/// `TypeExpr` is the parser's output and the resolver's input. It keeps the
/// written structure (name + optional type arguments) and the source span, so
/// the semantic layer never needs to re-parse a string. Resolution to a
/// semantic [`crate::ast::TypeExpr`] target lives in `thalos-language-service`.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum TypeExpr {
    /// A plain type name: `Position`, `Length`, `Joints`.
    Name { name: String, span: Span },
    /// A type applied to arguments: `Joints<4>`.
    Applied {
        name: String,
        args: Vec<TypeArg>,
        span: Span,
    },
}

/// One argument of an applied type expression.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum TypeArg {
    /// A literal dimension/parameter: `4` in `Joints<4>`.
    Integer(usize),
    /// A nested type expression (reserved for future parameterised types).
    Type(TypeExpr),
}

impl TypeExpr {
    pub fn name(&self) -> &str {
        match self {
            TypeExpr::Name { name, .. } | TypeExpr::Applied { name, .. } => name,
        }
    }

    pub fn span(&self) -> Span {
        match self {
            TypeExpr::Name { span, .. } | TypeExpr::Applied { span, .. } => *span,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn name_and_span_are_preserved_for_both_shapes() {
        let span = Span::new(3, 9);
        let plain = TypeExpr::Name {
            name: "Position".to_string(),
            span,
        };
        let applied = TypeExpr::Applied {
            name: "Joints".to_string(),
            args: vec![TypeArg::Integer(4)],
            span,
        };

        assert_eq!(plain.name(), "Position");
        assert_eq!(plain.span(), span);
        assert_eq!(applied.name(), "Joints");
        assert_eq!(applied.span(), span);
    }
}
