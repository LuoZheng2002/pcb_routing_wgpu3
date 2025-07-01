use crate::{prim_shape::PrimShape, trace_path::TracePath, vec2::FixedVec2};




pub struct AStarModel{
    pub width: f32,
    pub height: f32,
    pub obstacle_shapes: Vec<PrimShape>,
    pub obstacle_clearance_shapes: Vec<PrimShape>,
    pub start: FixedVec2,
    pub end: FixedVec2,
}

impl AStarModel{
    pub fn run(&self) -> Result<AStarResult, String> {
        todo!("Implement A* algorithm logic here");
    }
}

pub struct AStarResult{
    pub trace_path: TracePath,
}