use crate::{
    hyperparameters::HALF_PROBABILITY_RAW_SCORE, pcb_render_model::{RenderableBatch, ShapeRenderable}, prim_shape::{CircleShape, PrimShape, RectangleShape}, vec2::{FixedPoint, FixedVec2, FloatVec2}
};

#[derive(Debug, Clone, Copy, PartialEq, Hash, Eq, PartialOrd, Ord)]
pub enum Direction {
    Up,
    Down,
    Left,
    Right,
    TopRight,
    TopLeft,
    BottomRight,
    BottomLeft,
}
impl Direction {
    pub fn to_degree_angle(&self) -> f32 {
        match self {
            Direction::Up => 90.0,
            Direction::Down => 270.0,
            Direction::Left => 180.0,
            Direction::Right => 0.0,
            Direction::TopRight => 45.0,
            Direction::TopLeft => 135.0,
            Direction::BottomRight => 315.0,
            Direction::BottomLeft => 225.0,
        }
    }
    fn direction_to_int(&self) -> i32 {
        match self {
            Direction::Up => 0,
            Direction::TopRight => 1,
            Direction::Right => 2,
            Direction::BottomRight => 3,
            Direction::Down => 4,
            Direction::BottomLeft => 5,
            Direction::Left => 6,
            Direction::TopLeft => 7,
        }
    }
    fn int_to_direction(i: i32) -> Direction {
        match i {
            0 => Direction::Up,
            1 => Direction::TopRight,
            2 => Direction::Right,
            3 => Direction::BottomRight,
            4 => Direction::Down,
            5 => Direction::BottomLeft,
            6 => Direction::Left,
            7 => Direction::TopLeft,
            _ => panic!("Invalid direction index"),
        }
    }
    pub fn left_90_dir(&self) -> Direction {
        let new_index = (self.direction_to_int() + 6) % 8; // 6 is equivalent to -2 in mod 8
        Direction::int_to_direction(new_index)
    }
    pub fn right_90_dir(&self) -> Direction {
        let new_index = (self.direction_to_int() + 2) % 8; // 2 is equivalent to +2 in mod 8
        Direction::int_to_direction(new_index)
    }
    pub fn left_45_dir(&self) -> Direction {
        let new_index = (self.direction_to_int() + 7) % 8; // 7 is equivalent to -1 in mod 8
        Direction::int_to_direction(new_index)
    }
    pub fn right_45_dir(&self) -> Direction {
        let new_index = (self.direction_to_int() + 1) % 8; // 1 is equivalent to +1 in mod 8
        Direction::int_to_direction(new_index)
    }
    pub fn all_directions() -> Vec<Direction> {
        vec![
            Direction::Up,
            Direction::Down,
            Direction::Left,
            Direction::Right,
            Direction::TopRight,
            Direction::TopLeft,
            Direction::BottomRight,
            Direction::BottomLeft,
        ]
    }
    // pub fn to_fixed_vec2(&self) -> FixedVec2 {
    //     match self {
    //         Direction::Up => FloatVec2 { x: 0.0, y: 1.0 },
    //         Direction::Down => FloatVec2 { x: 0.0, y: -1.0 },
    //         Direction::Left => FloatVec2 { x: -1.0, y: 0.0 },
    //         Direction::Right => FloatVec2 { x: 1.0, y: 0.0 },
    //         Direction::TopRight => FloatVec2 { x: 1.0, y: 1.0 },
    //         Direction::TopLeft => FloatVec2 { x: -1.0, y: 1.0 },
    //         Direction::BottomRight => FloatVec2 { x: 1.0, y: -1.0 },
    //         Direction::BottomLeft => FloatVec2 { x: -1.0, y: -1.0 },
    //     }.to_fixed()
    // }
    pub fn to_int_vec2(&self) -> (i32, i32) {
        match self {
            Direction::Up => (0, 1),
            Direction::Down => (0, -1),
            Direction::Left => (-1, 0),
            Direction::Right => (1, 0),
            Direction::TopRight => (1, 1),
            Direction::TopLeft => (-1, 1),
            Direction::BottomRight => (1, -1),
            Direction::BottomLeft => (-1, -1),
        }
    }
    pub fn to_fixed_vec2(&self, scale: FixedPoint) -> FixedVec2 {
        let (dx, dy) = self.to_int_vec2();
        FixedVec2 {
            x: FixedPoint::from_num(dx) * scale,
            y: FixedPoint::from_num(dy) * scale,
        }
    }

    pub fn from_points(start: FixedVec2, end: FixedVec2) -> Self {
        let dx = end.x - start.x;
        let dy = end.y - start.y;

        match (dx.partial_cmp(&0.0), dy.partial_cmp(&0.0)) {
            (Some(std::cmp::Ordering::Equal), Some(std::cmp::Ordering::Greater)) => Direction::Up,
            (Some(std::cmp::Ordering::Equal), Some(std::cmp::Ordering::Less)) => Direction::Down,
            (Some(std::cmp::Ordering::Greater), Some(std::cmp::Ordering::Equal)) => Direction::Right,
            (Some(std::cmp::Ordering::Less), Some(std::cmp::Ordering::Equal)) => Direction::Left,

            (Some(std::cmp::Ordering::Greater), Some(std::cmp::Ordering::Greater)) => Direction::TopRight,
            (Some(std::cmp::Ordering::Less), Some(std::cmp::Ordering::Greater)) => Direction::TopLeft,
            (Some(std::cmp::Ordering::Greater), Some(std::cmp::Ordering::Less)) => Direction::BottomRight,
            (Some(std::cmp::Ordering::Less), Some(std::cmp::Ordering::Less)) => Direction::BottomLeft,

            _ => panic!("collision!!"),
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
        let segment_length = ((end.x - start.x).powi(2) + (end.y - start.y).powi(2)).sqrt();
        let start_circle = PrimShape::Circle(
            CircleShape {
                position: start,
                diameter: self.width,
            }
        );
        let end_circle = PrimShape::Circle(
            CircleShape {
                position: end,
                diameter: self.width,
            }
        );
        let segment_rect = PrimShape::Rectangle(
            RectangleShape {
                position: FloatVec2 {
                    x: (start.x + end.x) / 2.0,
                    y: (start.y + end.y) / 2.0,
                },
                width: segment_length,
                height: self.width,
                rotation: cgmath::Deg(self.direction.to_degree_angle()),
            }
        );
        vec![start_circle, end_circle, segment_rect]
    }
    pub fn to_clearance_shapes(&self) -> Vec<PrimShape> {
        // Clearance is represented by a larger rectangle around the segment
        let start = self.start.to_float();
        let end = self.end.to_float();
        let segment_length = ((end.x - start.x).powi(2) + (end.y - start.y).powi(2)).sqrt();
        let new_width = self.width + self.clearance * 2.0;
        let new_diameter = new_width;
        let clearance_start_circle = PrimShape::Circle(
            CircleShape {
                position: start,
                diameter: new_diameter,
            }
        );
        let clearance_end_circle = PrimShape::Circle(
            CircleShape {
                position: end,
                diameter: new_diameter,
            }
        );
        let clearance_rect = PrimShape::Rectangle(
            RectangleShape {
                position: FloatVec2 {
                    x: (start.x + end.x) / 2.0,
                    y: (start.y + end.y) / 2.0,
                },
                width: segment_length + self.clearance * 2.0,
                height: new_width,
                rotation: cgmath::Deg(self.direction.to_degree_angle()),
            }
        );
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
    pub fn to_renderables(&self, color: [f32;4])-> Vec<ShapeRenderable>{
        let shapes = self.to_shapes();
        shapes.into_iter().map(|shape| {
            ShapeRenderable {
                shape,
                color,
            }
        }).collect()
    }
    pub fn to_clearance_renderables(&self, color: [f32;4]) -> Vec<ShapeRenderable> {
        let clearance_shapes = self.to_clearance_shapes();
        clearance_shapes.into_iter().map(|shape| {
            ShapeRenderable {
                shape,
                color,
            }
        }).collect()
    }
}

#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord)]
pub struct TraceAnchors(pub Vec<FixedVec2>); // List of turning points in the trace path, including start and end

#[derive(Debug, Clone)]
pub struct TracePath {
    pub anchors: TraceAnchors, // List of turning points in the trace path, including start and end
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

    pub fn get_score(&self) -> f64 {
        // to do
        let score_raw = self.length; // placeholder for actual score calculation
        let k = f64::ln(2.0) / HALF_PROBABILITY_RAW_SCORE;
        let score = f64::exp(-k * score_raw);
        assert!(
            score >= 0.0 && score <= 1.0,
            "Score must be between 0 and 1, got: {}",
            score
        );
        score
    }

    pub fn to_renderables(&self, color: [f32; 4]) -> [RenderableBatch; 2]{
        let mut renderables = Vec::new();
        // Render the segments
        for segment in &self.segments {
            let segment_renderables = segment.to_renderables(color);
            renderables.extend(segment_renderables);
        }
        let mut clearance_renderables = Vec::new();
        let clearance_color = [color[0], color[1], color[2], color[3]/2.0]; // semi-transparent color
        for segment in &self.segments {
            let segment_clearance_renderables = segment.to_clearance_renderables(clearance_color); // semi-transparent color
            clearance_renderables.extend(segment_clearance_renderables);
        }
        [
            RenderableBatch(renderables),
            RenderableBatch(clearance_renderables),
        ]
    }
}
