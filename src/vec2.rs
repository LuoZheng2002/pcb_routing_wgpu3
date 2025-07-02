use fixed::traits::Fixed;
use ordered_float::Float;

pub type FixedPoint = fixed::types::I24F8;

#[derive(Debug, Clone, PartialEq, Hash, Eq, Copy, PartialOrd, Ord)]
pub struct FixedFloatVec2 {
    pub x: FixedPoint,
    pub y: FixedPoint,
}

impl FixedFloatVec2 {
    pub fn to_float(&self) -> FloatVec2 {
        FloatVec2 {
            x: self.x.to_num(),
            y: self.y.to_num(),
        }
    }
    pub fn length(&self) -> FixedPoint {
        (self.x * self.x + self.y * self.y).sqrt()
    }
}

#[derive(Debug, Clone, Copy)]
pub struct FloatVec2 {
    pub x: f32,
    pub y: f32,
}

impl FloatVec2 {
    pub fn to_fixed(&self) -> FixedFloatVec2 {
        FixedFloatVec2 {
            x: FixedPoint::from_num(self.x),
            y: FixedPoint::from_num(self.y),
        }
    }
    pub fn dot(self, other: FloatVec2) -> f32 {
        self.x * other.x + self.y * other.y
    }

    pub fn sub(self, other: FloatVec2) -> FloatVec2 {
        FloatVec2 {
            x: self.x - other.x,
            y: self.y - other.y,
        }
    }

    /// Returns a vector perpendicular to self (normal to edge)
    pub fn perp(self) -> FloatVec2 {
        FloatVec2 {
            x: -self.y,
            y: self.x,
        }
    }

    /// Normalize the vector (used to prevent numerical issues)
    pub fn normalize(self) -> FloatVec2 {
        let len = (self.x * self.x + self.y * self.y).sqrt();
        if len > f32::EPSILON {
            FloatVec2 {
                x: self.x / len,
                y: self.y / len,
            }
        } else {
            self
        }
    }
    pub fn magnitude2(self) -> f32 {
        self.x * self.x + self.y * self.y
    }
}