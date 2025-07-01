use crate::{
    prim_shape::PrimShape,
    vec2::{FixedVec2, FloatVec2},
};

#[derive(Debug, Clone, PartialEq, Hash, Eq)]
pub enum Direction {
    Up,
    Down,
    Left,
    Right,
    UpRight,
    UpLeft,
    DownRight,
    DownLeft,
}
impl Direction {
    pub fn to_degree_angle(&self) -> f32 {
        match self {
            Direction::Up => 90.0,
            Direction::Down => 270.0,
            Direction::Left => 180.0,
            Direction::Right => 0.0,
            Direction::UpRight => 45.0,
            Direction::UpLeft => 135.0,
            Direction::DownRight => 315.0,
            Direction::DownLeft => 225.0,
        }
    }
}

#[derive(Debug, Clone)]
pub struct TraceSegment {
    pub start: FixedVec2,     // Start point of the trace segment
    pub end: FixedVec2,       // End point of the trace segment
    pub direction: Direction, // Direction of the trace segment
    pub width: f32,           // Width of the trace segment
    pub clearance: f32, // Clearance around the trace segment
}

impl TraceSegment {
    pub fn to_shapes(&self) -> Vec<PrimShape> {
        // a trace segment is composed of two circles and a rectangle
        let start = self.start.to_float();
        let end = self.end.to_float();
        let start_circle = PrimShape::Circle {
            position: start,
            diameter: self.width,
        };
        let end_circle = PrimShape::Circle {
            position: end,
            diameter: self.width,
        };
        let segment_length = ((end.x - start.x).powi(2) + (end.y - start.y).powi(2)).sqrt();
        let segment_rect = PrimShape::Rectangle {
            position: FloatVec2 {
                x: (start.x + end.x) / 2.0,
                y: (start.y + end.y) / 2.0,
            },
            width: segment_length,
            height: self.width,
            rotation: cgmath::Deg(self.direction.to_degree_angle()),
        };
        vec![start_circle, end_circle, segment_rect]
    }
    pub fn to_clearance_shapes(&self) -> Vec<PrimShape> {
        // Clearance is represented by a larger rectangle around the segment
        let start = self.start.to_float();
        let end = self.end.to_float();
        let segment_length = ((end.x - start.x).powi(2) + (end.y - start.y).powi(2)).sqrt();
        let new_width = self.width + self.clearance * 2.0;
        let new_diameter = new_width;
        let clearance_start_circle = PrimShape::Circle {
            position: start,
            diameter: new_diameter,
        };
        let clearance_end_circle = PrimShape::Circle {
            position: end,
            diameter: new_diameter,
        };
        let clearance_rect = PrimShape::Rectangle {
            position: FloatVec2 {
                x: (start.x + end.x) / 2.0,
                y: (start.y + end.y) / 2.0,
            },
            width: segment_length,
            height: new_width,
            rotation: cgmath::Deg(self.direction.to_degree_angle()),
        };
        vec![clearance_start_circle, clearance_end_circle, clearance_rect]
    }
    pub fn collides_with(&self, other: &TraceSegment) -> bool {
        let self_shapes = self.to_shapes();
        let self_clearance_shapes = self.to_clearance_shapes();
        let other_shapes = other.to_shapes();
        let other_clearance_shapes = other.to_clearance_shapes();
        for self_shape in self_shapes {
            for other_clearance_shape in &other_clearance_shapes {
                if self_shape.collides_with(other_clearance_shape) {
                    return true;
                }
            }
        }
        for self_clearance_shape in self_clearance_shapes {
            for other_shape in &other_shapes {
                if self_clearance_shape.collides_with(other_shape) {
                    return true;
                }
            }
        }
        false
    }
}

#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord)]
pub struct TraceAnchors(pub Vec<FixedVec2>); // List of turning points in the trace path, excluding start and end

#[derive(Debug, Clone)]
pub struct TracePath {
    pub anchors: TraceAnchors, // List of turning points in the trace path, excluding start and end
    pub segments: Vec<TraceSegment>, // List of segments in the trace path
    pub length: f64,
}
// shrink?

impl TracePath {
    pub fn collides_with(&self, other: &TracePath) -> bool {
        for segment_self in &self.segments {
            for segment_other in &other.segments {
                if segment_self.collides_with(segment_other) {
                    return true;
                }
            }
        }
        false
    }
}
