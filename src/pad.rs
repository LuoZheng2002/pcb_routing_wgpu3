use crate::{
    prim_shape::PrimShape,
    vec2::{FixedPoint, FixedVec2, FloatVec2},
};

#[derive(Debug, Clone)]
pub enum PadShape {
    Circle {
        diameter: f32,
    },
    Square {
        side_length: f32,
    },
    Rectangle {
        width: f32,
        height: f32,
    },
    RoundedRect {
        width: f32,
        height: f32,
        radius: f32, // Radius for the rounded corners
    },
}

#[derive(Debug, Clone)]
pub struct Pad {
    pub position: FloatVec2,
    pub shape: PadShape,
    pub rotation: cgmath::Deg<f32>, // Rotation in degrees
    pub clearance: f32,             // Clearance around the pad
}

impl Pad {
    pub fn to_shapes(&self) -> Vec<PrimShape> {
        match &self.shape {
            PadShape::Circle { diameter } => vec![PrimShape::Circle {
                position: self.position,
                diameter: *diameter,
            }],
            PadShape::Square { side_length } => vec![PrimShape::Rectangle {
                position: self.position,
                width: *side_length,
                height: *side_length,
                rotation: self.rotation,
            }],
            PadShape::Rectangle { width, height } => vec![PrimShape::Rectangle {
                position: self.position,
                width: *width,
                height: *height,
                rotation: self.rotation,
            }],
            PadShape::RoundedRect {
                width,
                height,
                radius,
            } => todo!(),
        }
    }
    pub fn to_clearance_shapes(&self) -> Vec<PrimShape> {
        todo!()
    }
}
