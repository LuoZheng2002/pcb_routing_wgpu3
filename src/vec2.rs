use fixed::traits::Fixed;
use ordered_float::Float;

pub type FixedPoint = fixed::types::I24F8;

#[derive(Debug, Clone, PartialEq, Hash, Eq, Copy, PartialOrd, Ord)]
pub struct FixedVec2 {
    pub x: FixedPoint,
    pub y: FixedPoint,
}

impl FixedVec2 {
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
    pub fn to_fixed(&self) -> FixedVec2 {
        FixedVec2 {
            x: FixedPoint::from_num(self.x),
            y: FixedPoint::from_num(self.y),
        }
    }
}