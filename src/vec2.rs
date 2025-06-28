use fixed::traits::Fixed;


pub type FixedPoint = fixed::types::I24F8;

#[derive(Debug, Clone, PartialEq, Hash, Eq, Copy, PartialOrd, Ord)]
pub struct FixedVec2 {
    pub x: FixedPoint,
    pub y: FixedPoint,
}

impl FixedVec2{
    pub fn to_float(&self) -> FloatVec2 {
        FloatVec2 {
            x: self.x.to_num(),
            y: self.y.to_num(),
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct FloatVec2{
    pub x: f32,
    pub y: f32,
}