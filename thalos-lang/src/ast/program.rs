use serde::{Deserialize, Serialize};
use crate::ast::item::Item;

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Program {
    pub items: Vec<Item>,
}
