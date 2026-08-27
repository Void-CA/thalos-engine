/// RGBA colour.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Color {
    pub r: f64,
    pub g: f64,
    pub b: f64,
    pub a: f64,
}

impl Color {
    pub const fn new(r: f64, g: f64, b: f64, a: f64) -> Self {
        Self { r, g, b, a }
    }
}

/// Visual material for a link.
///
/// Corresponds to `<material>` in URDF.
#[derive(Debug, Clone, PartialEq)]
pub struct Material {
    pub name: String,
    pub color: Option<Color>,
    pub texture: Option<String>,
}

impl Material {
    pub fn new(name: impl Into<String>) -> Self {
        Self {
            name: name.into(),
            color: None,
            texture: None,
        }
    }

    pub fn with_color(mut self, color: Color) -> Self {
        self.color = Some(color);
        self
    }
}
