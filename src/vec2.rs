
pub type FixedPoint = fixed::types::I24F8;

#[derive(Debug, Clone, PartialEq, Hash, Eq, Copy, PartialOrd, Ord)]
pub struct FixedVec2 {
    pub x: FixedPoint,
    pub y: FixedPoint,
}

#[derive(Debug, Clone, Copy)]
pub struct FloatVec2{
    pub x: f32,
    pub y: f32,
}