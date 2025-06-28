use crate::vec2::{FixedPoint, FixedVec2, FloatVec2};




pub enum PrimShape{
    Circle{
        position: FloatVec2,
        diameter: f32,
    },
    Rectangle{
        position: FixedVec2,
        width: f32,
        height: f32,
        rotation: cgmath::Rad<f32>, // Rotation in radians
    },
}