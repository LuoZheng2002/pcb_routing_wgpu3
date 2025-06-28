use crate::vec2::{FixedPoint, FixedVec2, FloatVec2};

#[derive(Debug, Clone)]
pub enum PadShape{
    Circle{
        diameter: f32,
    },
    Square{
        side_length: f32,
    },
    Rectangle{
        width: f32,
        height: f32,
    },
    RoundedRect{
        width: f32,
        height: f32,
        radius: f32, // Radius for the rounded corners
    }
}

#[derive(Debug, Clone)]
pub struct Pad{
    pub position: FloatVec2,
    pub shape: PadShape,
    pub rotation: cgmath::Deg<f32>, // Rotation in degrees
}