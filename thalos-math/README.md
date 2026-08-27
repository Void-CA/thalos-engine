# thalos-math

**Pregunta que responde:** ¿Cómo se representan y operan vectores,
rotaciones y transformaciones?

Tipos matemáticos fundamentales desacoplados del dominio robótico.
Originalmente parte de `thalos-core`, separados para permitir reuso en
contextos no robóticos.

### Tipos

- `Vector3`, `UnitVector3`
- `Quaternion`, `UnitQuaternion`
- `Transform3D`, `Transform<From, To>` (PhantomData type safety)
- `DynamicMatrix`, `DynamicVector`
- `Matrix4x4`
- Traits `Cross`, `Dot`
- Parámetros DH (`dh`)

**No debe contener:** estado mutable, HTTP, conceptos robóticos, visualización.
