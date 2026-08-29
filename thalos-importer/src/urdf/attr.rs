use quick_xml::events::BytesStart;
use crate::urdf::error::UrdfError;
use thalos_math::{UnitQuaternion, Vector3};
use thalos_models::material::Color;

pub fn attr(elem: &BytesStart<'_>, name: &[u8]) -> Result<Option<String>, UrdfError> {
    match elem.try_get_attribute(name) {
        Ok(Some(a)) => Ok(Some(
            a.unescape_value()
                .map_err(|e| UrdfError::Xml(e.to_string()))?
                .into_owned(),
        )),
        Ok(None) => Ok(None),
        Err(e) => Err(UrdfError::Xml(e.to_string())),
    }
}

pub fn required_attr(
    elem: &BytesStart<'_>,
    name: &[u8],
    element_name: &str,
) -> Result<String, UrdfError> {
    attr(elem, name)?.ok_or_else(|| UrdfError::MissingAttribute {
        element: element_name.to_string(),
        attribute: String::from_utf8_lossy(name).into_owned(),
    })
}

pub fn parse_n_floats(s: &str, n: usize, context: &str) -> Result<Vec<f64>, UrdfError> {
    let parts: Vec<&str> = s.split_whitespace().collect();
    if parts.len() != n {
        return Err(UrdfError::TupleLength {
            element: context.to_string(),
            expected: n,
            got: parts.len(),
        });
    }
    parts
        .iter()
        .map(|p| {
            p.parse::<f64>().map_err(|e| UrdfError::ParseFloat {
                value: (*p).to_string(),
                source: e.to_string(),
            })
        })
        .collect()
}

pub fn parse_xyz(s: &str, context: &str) -> Result<Vector3, UrdfError> {
    let v = parse_n_floats(s, 3, context)?;
    Ok(Vector3::new(v[0], v[1], v[2]))
}

pub fn parse_rpy(s: &str, context: &str) -> Result<UnitQuaternion, UrdfError> {
    let v = parse_n_floats(s, 3, context)?;
    Ok(UnitQuaternion::from_euler(v[0], v[1], v[2]))
}

pub fn parse_rgba(s: &str, context: &str) -> Result<Color, UrdfError> {
    let v = parse_n_floats(s, 4, context)?;
    Ok(Color::new(v[0], v[1], v[2], v[3]))
}

pub fn parse_origin(elem: &BytesStart<'_>) -> Result<thalos_math::Transform3D, UrdfError> {
    let translation = match attr(elem, b"xyz")? {
        Some(s) => parse_xyz(&s, "origin")?,
        None => Vector3::zero(),
    };
    let rotation = match attr(elem, b"rpy")? {
        Some(s) => parse_rpy(&s, "origin")?,
        None => UnitQuaternion::identity(),
    };
    Ok(thalos_math::Transform3D {
        translation,
        rotation,
    })
}
