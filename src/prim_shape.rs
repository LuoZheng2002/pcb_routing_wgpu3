use crate::vec2::{FixedPoint, FixedVec2, FloatVec2};




pub enum PrimShape{
    Circle{
        position: FloatVec2,
        diameter: f32,
    },
    Rectangle{
        position: FloatVec2,
        width: f32,
        height: f32,
        rotation: cgmath::Deg<f32>, // Rotation in radians
    },
}


impl PrimShape{
    pub fn collides_with(&self, other: &PrimShape) -> bool {
        match (self, other) {
            (PrimShape::Circle { position: pos1, diameter: d1 }, PrimShape::Circle { position: pos2, diameter: d2 }) => {
                let radius1 = d1 / 2.0;
                let radius2 = d2 / 2.0;
                let distance_squared = (pos1.x - pos2.x).powi(2) + (pos1.y - pos2.y).powi(2);
                distance_squared < (radius1 + radius2).powi(2)
            },
            // Add more collision detection logic for Rectangle and mixed shapes if needed
            _ => {
                todo!("Collision detection for other shapes not implemented yet");
            }
        }
    }
}