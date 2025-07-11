use std::{
    cell::RefCell,
    cmp::Reverse,
    collections::{BinaryHeap, HashMap, HashSet},
    rc::Rc,
    sync::{Arc, Mutex},
};

use fixed::traits::Fixed;
use ordered_float::NotNan;

use crate::{
    binary_heap_item::BinaryHeapItem,
    block_or_sleep::{block_or_sleep, block_thread},
    hyperparameters::{ASTAR_STRIDE, DISPLAY_ASTAR, ESTIMATE_COEFFICIENT, TURN_PENALTY},
    pcb_render_model::{PcbRenderModel, RenderableBatch, ShapeRenderable, UpdatePcbRenderModel},
    prim_shape::{CircleShape, PrimShape, RectangleShape},
    trace_path::{Direction, TraceAnchors, TracePath, TraceSegment},
    vec2::{FixedPoint, FixedVec2, FloatVec2},
};

pub struct AStarModel {
    pub width: f32,
    pub height: f32,
    pub obstacle_shapes: Vec<PrimShape>,
    pub obstacle_clearance_shapes: Vec<PrimShape>,
    pub start: FixedVec2,
    pub end: FixedVec2,
    pub trace_width: f32,
    pub trace_clearance: f32,
    pub border_cache: RefCell<Option<Rc<Vec<PrimShape>>>>,
}

impl AStarModel {
    fn get_border_shapes(&self) -> Rc<Vec<PrimShape>> {
        if let Some(border_shapes) = self.border_cache.borrow().as_ref() {
            return border_shapes.clone();
        }
        let margin = 100.0; // margin around the border shapes

        let top_rectangle = PrimShape::Rectangle(RectangleShape {
            position: FloatVec2 {
                x: 0.0,
                y: self.height / 2.0 + margin / 2.0,
            },
            width: self.width + 2.0 * margin,
            height: margin,
            rotation: cgmath::Deg(0.0),
        });
        let bottom_rectangle = PrimShape::Rectangle(RectangleShape {
            position: FloatVec2 {
                x: 0.0,
                y: -self.height / 2.0 - margin / 2.0,
            },
            width: self.width + 2.0 * margin,
            height: margin,
            rotation: cgmath::Deg(0.0),
        });
        let left_rectangle = PrimShape::Rectangle(RectangleShape {
            position: FloatVec2 {
                x: -self.width / 2.0 - margin / 2.0,
                y: 0.0,
            },
            width: margin,
            height: self.height + 2.0 * margin,
            rotation: cgmath::Deg(0.0),
        });
        let right_rectangle = PrimShape::Rectangle(RectangleShape {
            position: FloatVec2 {
                x: self.width / 2.0 + margin / 2.0,
                y: 0.0,
            },
            width: margin,
            height: self.height + 2.0 * margin,
            rotation: cgmath::Deg(0.0),
        });
        let border_shapes = Rc::new(vec![
            top_rectangle,
            bottom_rectangle,
            left_rectangle,
            right_rectangle,
        ]);
        *self.border_cache.borrow_mut() = Some(border_shapes.clone());
        border_shapes
    }

    fn collides_with_border(&self, shapes: &Vec<PrimShape>) -> bool {
        // the allowed region is between (-width/2, -height/2) and (width/2, height/2)
        // create four overlapping rectangles that encapsulate the allowed region
        // the margin is sufficiently large
        let border_shapes = self.get_border_shapes();
        for border_shape in border_shapes.iter() {
            for shape in shapes.iter() {
                if border_shape.collides_with(shape) {
                    return true; // collision with the border
                }
            }
        }
        false
    }

    fn check_collision(&self, start_position: FixedVec2, end_position: FixedVec2, trace_width: f32, trace_clearance: f32) -> bool {
        let trace_segment = TraceSegment {
            start: start_position,
            end: end_position,
            direction: Direction::Up, // don't care about the direction here, just need the segment
            width: trace_width,
            clearance: trace_clearance,
        };
        // new trace segment may collide with obstacles or bounds
        let shapes = trace_segment.to_shapes();
        let clearance_shapes = trace_segment.to_clearance_shapes();
        if self.collides_with_border(&shapes) {
            return true; // collision with the border
        }
        for obstacle_shape in self.obstacle_shapes.iter() {
            for clearance_shape in clearance_shapes.iter() {
                if obstacle_shape.collides_with(clearance_shape) {
                    return true; // collision with an obstacle
                }
            }
        }
        for obstacle_clearance_shape in self.obstacle_clearance_shapes.iter() {
            for shape in shapes.iter() {
                if obstacle_clearance_shape.collides_with(shape) {
                    return true; // collision with an obstacle clearance shape
                }
            }
        }
        false // no collision
    }
    fn octile_distance(start: &FixedVec2, end: &FixedVec2) -> f64 {
        let start = start.to_float();
        let end = end.to_float();
        let dx = (end.x - start.x).abs() as f64;
        let dy = (end.y - start.y).abs() as f64;
        f64::max(dx, dy) + (f64::sqrt(2.0) - 1.0) * f64::min(dx, dy)
    }

    fn is_grid_point(&self, position: &FixedVec2) -> bool {
        position.x % *ASTAR_STRIDE == FixedPoint::ZERO
            && position.y % *ASTAR_STRIDE == FixedPoint::ZERO
    }

    /// outputs the pairs of direction and the grid point that the direction leads to
    /// not implemented the collision check yet
    fn directions_to_grid_points(&self, position: &FixedVec2) -> Vec<(Direction, FixedVec2)>{
        let mut result: Vec<(Direction, FixedVec2)> = Vec::new();
        // horizontal directions
        if position.y.rem_euclid(*ASTAR_STRIDE) == FixedPoint::ZERO {
            // left
            let left_grid_point_x = ((position.x - FixedPoint::DELTA) / *ASTAR_STRIDE).floor() * *ASTAR_STRIDE;
            let right_grid_point_x = ((position.x + FixedPoint::DELTA) / *ASTAR_STRIDE).ceil() * *ASTAR_STRIDE;
            result.push((Direction::Left, FixedVec2::new(left_grid_point_x, position.y)));
            result.push((Direction::Right, FixedVec2::new(right_grid_point_x, position.y)));
        }
        // vertical directions
        if position.x.rem_euclid(*ASTAR_STRIDE) == FixedPoint::ZERO {
            // up
            let up_grid_point_y = ((position.y + FixedPoint::DELTA) / *ASTAR_STRIDE).ceil() * *ASTAR_STRIDE;
            let down_grid_point_y = ((position.y - FixedPoint::DELTA) / *ASTAR_STRIDE).floor() * *ASTAR_STRIDE;
            result.push((Direction::Up, FixedVec2::new(position.x, up_grid_point_y)));
            result.push((Direction::Down, FixedVec2::new(position.x, down_grid_point_y)));
        }
        // top left to bottom right diagonal
        if (position.x + position.y).rem_euclid(*ASTAR_STRIDE) == FixedPoint::ZERO {
            let top_left_grid_point = FixedVec2::new(
                ((position.x - FixedPoint::DELTA) / *ASTAR_STRIDE).floor() * *ASTAR_STRIDE,
                ((position.y + FixedPoint::DELTA) / *ASTAR_STRIDE).ceil() * *ASTAR_STRIDE,
            );
            let bottom_right_grid_point = FixedVec2::new(
                ((position.x + FixedPoint::DELTA) / *ASTAR_STRIDE).ceil() * *ASTAR_STRIDE,
                ((position.y - FixedPoint::DELTA) / *ASTAR_STRIDE).floor() * *ASTAR_STRIDE,
            );
            result.push((Direction::TopLeft, top_left_grid_point));
            result.push((Direction::BottomRight, bottom_right_grid_point));
        }
        // top right to bottom left diagonal
        if (position.x - position.y).rem_euclid(*ASTAR_STRIDE) == FixedPoint::ZERO {
            let top_right_grid_point = FixedVec2::new(
                ((position.x + FixedPoint::DELTA) / *ASTAR_STRIDE).ceil() * *ASTAR_STRIDE,
                ((position.y + FixedPoint::DELTA) / *ASTAR_STRIDE).ceil() * *ASTAR_STRIDE,
            );
            let bottom_left_grid_point = FixedVec2::new(
                ((position.x - FixedPoint::DELTA) / *ASTAR_STRIDE).floor() * *ASTAR_STRIDE,
                ((position.y - FixedPoint::DELTA) / *ASTAR_STRIDE).floor() * *ASTAR_STRIDE,
            );
            result.push((Direction::TopRight, top_right_grid_point));
            result.push((Direction::BottomLeft, bottom_left_grid_point));
        }
        result
    }
    fn radial_directions_wrt_obstacles(&self, position: &FixedVec2) -> Vec<Direction>{
        let mut directions: Vec<Direction> = Vec::new();
        let mut collides_at_direction: HashMap<Direction, bool> = HashMap::new();
        let twice_delta = FixedPoint::DELTA * 2;
        for direction in Direction::all_directions() {
            let end_position = *position + direction.to_fixed_vec2(twice_delta);
            let collides = self.check_collision(*position, end_position, self.trace_width, self.trace_clearance);
            collides_at_direction.insert(direction, collides);
        }
        let is_valid_radial_direction = |left_90_dir: Direction, left_45_dir: Direction,
            dir: Direction, right_45_dir: Direction, right_90_dir: Direction| {
            // check if the direction is valid, i.e., it is not a 45-degree direction
            // or it is a 45-degree direction but both left and right directions are not valid
            let left_blocked = collides_at_direction[&left_90_dir] && 
                collides_at_direction[&left_45_dir];
            let right_blocked = collides_at_direction[&right_90_dir] && 
                collides_at_direction[&right_45_dir];
            let front_blocked = collides_at_direction[&dir];
            !front_blocked && (left_blocked || right_blocked)
        };
        for direction in Direction::all_directions(){
            let left_90_dir = direction.left_90_dir();
            let left_45_dir = direction.left_45_dir();
            let right_45_dir = direction.right_45_dir();
            let right_90_dir = direction.right_90_dir();
            if is_valid_radial_direction(left_90_dir, left_45_dir, direction, right_45_dir, right_90_dir) {
                directions.push(direction);
            }
        }
        directions
    }
    /// 将浮动点移动到稍微好一点的点
    fn to_nearest_one_step_point(&self, position: &FixedVec2, direction: Direction) -> FixedVec2 {
        match direction{
            Direction::Up=>{
                let new_y = ((position.y + FixedPoint::DELTA) / *ASTAR_STRIDE).ceil() * *ASTAR_STRIDE;
                FixedVec2::new(position.x, new_y)
            },
            Direction::Down=>{
                let new_y = ((position.y - FixedPoint::DELTA) / *ASTAR_STRIDE).floor() * *ASTAR_STRIDE;
                FixedVec2::new(position.x, new_y)
            },
            Direction::Left=>{
                let new_x = ((position.x - FixedPoint::DELTA) / *ASTAR_STRIDE).floor() * *ASTAR_STRIDE;
                FixedVec2::new(new_x, position.y)
            },
            Direction::Right=>{
                let new_x = ((position.x + FixedPoint::DELTA) / *ASTAR_STRIDE).ceil() * *ASTAR_STRIDE;
                FixedVec2::new(new_x, position.y)
            },
            Direction::TopLeft=>{
                // 左下到右上的线
                let current_difference = position.y - position.x;
                // new_position.y - new_position.x = target_difference
                // 左下到右上的线，往左上提
                let target_difference = ((current_difference + FixedPoint::DELTA) / *ASTAR_STRIDE).ceil() * *ASTAR_STRIDE;
                // 往左上走，x和y的和不变
                let sum = position.y + position.x;
                // y - x = target_difference
                // y + x = sum
                // 求线性方程组
                let new_x = (sum - target_difference) / 2;
                let new_y = (sum + target_difference) / 2;
                FixedVec2::new(new_x, new_y)
            },
            Direction::BottomRight=>{
                // 左下到右上的线
                let current_difference = position.y - position.x;
                // new_position.y - new_position.x = target_difference
                // 左下到右上的线，往右下按
                let target_difference = ((current_difference - FixedPoint::DELTA) / *ASTAR_STRIDE).floor() * *ASTAR_STRIDE;
                // 往左上走，x和y的和不变
                let sum = position.y + position.x;
                // y - x = target_difference
                // y + x = sum
                // 求线性方程组
                let new_x = (sum - target_difference) / 2;
                let new_y = (sum + target_difference) / 2;
                FixedVec2::new(new_x, new_y)
            },
            Direction::BottomLeft=>{
                // 左上到右下的线
                let current_sum = position.x + position.y;
                // new_position.y + new_position.x = target_difference
                // 左上到右下的线， 往左下按
                let target_sum = ((current_sum - FixedPoint::DELTA) / *ASTAR_STRIDE).floor() * *ASTAR_STRIDE;
                // 往左下走，y和x的差不变
                let difference = position.y - position.x;
                // y - x = difference
                // y + x = target_sum                
                // 求线性方程组
                let new_x = (target_sum - difference) / 2;
                let new_y = (target_sum + difference) / 2;
                FixedVec2::new(new_x, new_y)
            },
            Direction::TopRight=>{
                // 左上到右下的线
                let current_sum = position.x + position.y;
                // new_position.y + new_position.x = target_difference
                // 左上到右下的线， 往右上按
                let target_sum = ((current_sum + FixedPoint::DELTA) / *ASTAR_STRIDE).ceil() * *ASTAR_STRIDE;
                // 往左下走，y和x的差不变
                let difference = position.y - position.x;
                // y - x = difference
                // y + x = target_sum                
                // 求线性方程组
                let new_x = (target_sum - difference) / 2;
                let new_y = (target_sum + difference) / 2;
                FixedVec2::new(new_x, new_y)
            },
        }
    }
    /// 判断当前点是否与目标点对齐，返回对齐的方向
    fn is_aligned_with_end(&self, position: &FixedVec2) -> Option<Direction>{
        assert_ne!(position, &self.end, "调用该函数前应确保已经处理与end重合的情况");
        if position.x == self.end.x {
            if position.y < self.end.y {
                return Some(Direction::Up);
            } else {
                return Some(Direction::Down);
            }
        } else if position.y == self.end.y {
            if position.x < self.end.x {
                return Some(Direction::Right);
            } else {
                return Some(Direction::Left);
            }
        } else if (position.x + position.y) == (self.end.x + self.end.y) {
            if position.x < self.end.x {
                return Some(Direction::BottomRight);
            } else {
                return Some(Direction::TopLeft);
            }
        } else if (position.x - position.y) == (self.end.x - self.end.y) {
            if position.x < self.end.x {
                return Some(Direction::TopRight);
            } else {
                return Some(Direction::BottomLeft);
            }
        }
        None // not aligned with end
    }
    /// 获取与end对齐的交点，还是给定方向和线段长度，判断是否有交叉
    fn get_intersection_with_end_alignments(&self, position: &FixedVec2, direction: Direction, max_length: FixedPoint)->Option<FixedVec2>{
        assert_ne!(position, &self.end, "调用该函数前应确保已经处理与end重合的情况");
        assert!(self.is_aligned_with_end(position).is_none(), "调用该函数前应确保当前点不与end对齐");

        // 实现应该和之前差不多
        // 输出的点的距离应当比max_length小，不然与已有的点重复
        todo!()
    }


    // 1. 整点/走一步到整点 -> 整点，或被障碍物挡住
    // 2. 走两步到整点+贴着障碍物 -> 对每个方向，走到最近的“走一步到整点”，或被障碍物挡住
    // 3. 是否align with end，如果是，并且align成功了的话，将end放入frontier

    // 拦住：网格边缘，align with end，障碍物
    // 障碍物优先，


    // 4. 浮空（走两步到整点+不贴障碍物）-> 选择任意的方向，走到“走一步到整点”，如果被障碍物挡住，选下一个方向；如果所有都被障碍物挡住，选择自己的方向并撞上障碍物
    
    // 同时考虑1和2和3
    // 如果满足1或2或3则不用4，如果1和2和3都失败则考虑4
    // 这些性质可以在expand的时候计算，不用存储
    // align with end也可以在expand的时候计算
    // 可能产生浮空的条件：起点，或是贴着墙走后不再贴着墙走

    // 伪代码：
    // current node从frontier中取出
    // current node设为visited
    // 判断1, 2, 3, 算出它们的expand的集合，然后合并（最多可能有8个方向，一个方向又最多可能有2个position）
    // 如果1, 2, 3都失败了（没有任何的expand），执行“4”的逻辑，必然会expand出来一个可能不怎么好的点
    // 将所有的expand的点放入frontier


    pub fn run(&self, pcb_render_model: Arc<Mutex<PcbRenderModel>>) -> Result<AStarResult, String> {
        

        let start_estimated_cost = Self::octile_distance(&self.start, &self.end) * ESTIMATE_COEFFICIENT;

        
        let start_node = AstarNode {
            position: self.start,
            direction: None, // no direction for the start node
            actual_cost: 0.0,
            actual_length: 0.0, // no length for the start node
            estimated_cost: start_estimated_cost,
            total_cost: start_estimated_cost,
            prev_node: None, // no previous node for the start node
            status: AstarNodeStatus::Normal, // start node is normal
        };

        // frontier is a min heap
        let mut frontier: BinaryHeap<BinaryHeapItem<Reverse<NotNan<f64>>, Rc<AstarNode>>> =
            BinaryHeap::new();
        frontier.push(BinaryHeapItem {
            key: Reverse(NotNan::new(start_node.total_cost).unwrap()), // use Reverse to make it a min heap
            value: Rc::new(start_node),
        });
        let mut visited: HashSet<AstarNodeKey> = HashSet::new();

        let display_and_block =
            |frontier: &BinaryHeap<BinaryHeapItem<Reverse<NotNan<f64>>, Rc<AstarNode>>>| {
                let mut frontier_vec: Vec<BinaryHeapItem<Reverse<NotNan<f64>>, Rc<AstarNode>>> =
                    frontier.clone().drain().collect();
                frontier_vec.reverse();
                let mut lowest_total_cost = f64::MAX;
                let mut highest_total_cost: f64 = 0.0;

                for item in frontier_vec.iter() {
                    if item.key.0.into_inner() < lowest_total_cost {
                        lowest_total_cost = item.key.0.into_inner();
                    }
                    if item.key.0.into_inner() > highest_total_cost {
                        highest_total_cost = item.key.0.into_inner();
                    }
                }
                let mut render_model = PcbRenderModel {
                    width: self.width,
                    height: self.height,
                    trace_shape_renderables: Vec::new(),
                    pad_shape_renderables: Vec::new(),
                };

                let obstacle_renderables = self
                    .obstacle_shapes
                    .iter()
                    .map(|shape| {
                        ShapeRenderable {
                            shape: shape.clone(),
                            color: [0.7, 0.7, 0.7, 1.0], // gray obstacles
                        }
                    })
                    .collect::<Vec<_>>();
                render_model
                    .trace_shape_renderables
                    .push(RenderableBatch(obstacle_renderables));
                let obstacle_clearance_renderables = self
                    .obstacle_clearance_shapes
                    .iter()
                    .map(|shape| {
                        ShapeRenderable {
                            shape: shape.clone(),
                            color: [0.7, 0.7, 0.7, 0.5], // gray obstacle clearance
                        }
                    })
                    .collect::<Vec<_>>();
                render_model
                    .trace_shape_renderables
                    .push(RenderableBatch(obstacle_clearance_renderables));
                // render border
                let border_renderables = self
                    .get_border_shapes()
                    .iter()
                    .map(|shape| {
                        ShapeRenderable {
                            shape: shape.clone(),
                            color: [1.0, 0.0, 1.0, 0.5], // magenta border
                        }
                    })
                    .collect::<Vec<_>>();
                render_model
                    .trace_shape_renderables
                    .push(RenderableBatch(border_renderables));

                for item in frontier_vec.iter() {
                    let BinaryHeapItem {
                        key: total_cost,
                        value: astar_node,
                    } = item;
                    let total_cost = total_cost.0.into_inner();
                    assert!(
                        total_cost >= lowest_total_cost,
                        "Total cost should be greater than or equal to the lowest total cost"
                    );
                    assert!(
                        total_cost <= highest_total_cost,
                        "Total cost should be less than or equal to the highest total cost"
                    );
                    // let alpha = 1.0 - (0.2 + 0.8 * (total_cost - lowest_total_cost) / (highest_total_cost - lowest_total_cost));
                    let alpha = if highest_total_cost > lowest_total_cost {
                        1.0 - (0.2
                            + 0.8 * (total_cost - lowest_total_cost)
                                / (highest_total_cost - lowest_total_cost))
                    } else {
                        1.0 // if all costs are the same, use full opacity
                    };
                    let alpha = alpha.clamp(0.0, 1.0) as f32;
                    assert!(
                        alpha >= 0.0 && alpha <= 1.0,
                        "Alpha should be between 0.0 and 1.0, get: {}",
                        alpha
                    );
                    let color: [f32; 3] = [1.0 - alpha, alpha, 0.0]; // red to green gradient
                    let renderables =
                        astar_node.to_renderables(self.trace_width, self.trace_clearance, color);
                    render_model.trace_shape_renderables.extend(renderables);
                }
                // render the start and end nodes
                let start_renderable = ShapeRenderable {
                    shape: PrimShape::Circle(CircleShape {
                        position: self.start.to_float(),
                        diameter: self.trace_width,
                    }),
                    color: [0.0, 0.0, 1.0, 1.0], // blue start node
                };
                let end_renderable = ShapeRenderable {
                    shape: PrimShape::Circle(CircleShape {
                        position: self.end.to_float(),
                        diameter: self.trace_width,
                    }),
                    color: [0.0, 1.0, 0.0, 1.0], // green end node
                };
                render_model.pad_shape_renderables.push(start_renderable);
                render_model.pad_shape_renderables.push(end_renderable);
                pcb_render_model.update_pcb_render_model(render_model);
                block_or_sleep();
            };
        if DISPLAY_ASTAR {
            display_and_block(&frontier); // display the initial state of the frontier
        }

        let max_trials: usize = 200;
        let mut trial_count = 0;
        while !frontier.is_empty() {
            trial_count += 1;
            if trial_count > max_trials {
                return Err("A* search exceeded maximum trials".to_string());
            }

            let item = frontier.pop().unwrap();

            let current_node = item.value.clone();
            if current_node.position == self.end {
                frontier.push(item); // push the current node back to the frontier, so that it can be displayed
                if DISPLAY_ASTAR {
                    display_and_block(&frontier); // display the initial state of the frontier
                }
                // Reached the end node, construct the trace path
                let trace_path = current_node.to_trace_path(self.trace_width, self.trace_clearance);
                return Ok(AStarResult { trace_path });
            }

            // move to the visited set
            let current_key = AstarNodeKey {
                position: current_node.position,
            };
            if visited.contains(&current_key) {
                continue; // already visited this node
            }
            
            // why? 这个应该可以去掉了
            if current_node.align_with_end == false {
                visited.insert(current_key.clone());
                // expand
            }
            // new:
            // hoist the closure out of the directions loop for the aligned_with_end condition
            let try_push_node_to_frontier = |direction: Direction, end_position: FixedVec2, aligned_with_end: bool| {
                let astar_node_key = AstarNodeKey {
                    position: end_position,
                };
                // check if the new position is already visited
                if visited.contains(&astar_node_key) {
                    return;
                }
                // let length: f64 = (direction.to_fixed_vec2().length() * length).to_num();
                let length: f64 = (end_position - current_node.position).length().to_num();
                let actual_cost = current_node.actual_cost + length; // to do: add turn penalty
                let actual_length = current_node.actual_length + length;
                let estimated_cost =
                    AStarModel::octile_distance(&end_position, &self.end) * ESTIMATE_COEFFICIENT;
                let total_cost = actual_cost + estimated_cost;
                let new_node = AstarNode {
                    position: end_position,
                    direction: Some(direction),
                    actual_cost,
                    actual_length,
                    estimated_cost,
                    total_cost,
                    prev_node: Some(current_node.clone()), // link to the previous node
                    align_with_end: aligned_with_end,
                };
                // push directly to the frontier
                frontier.push(BinaryHeapItem {
                    key: Reverse(NotNan::new(new_node.total_cost).unwrap()), // use Reverse to make it a min heap
                    value: Rc::new(new_node),
                });
            };

            // new:
            // hoist the aligned_with_end condition out of the directions loop
            // 这边应该不用再判断align_with_end，到需要用到的时候求解即可
            if current_node.align_with_end{         
                if !self.check_collision(current_node.position, self.end, self.trace_width, self.trace_clearance) {
                    try_push_node_to_frontier(self.end, false);
                }
            }

            let directions;
            // let current_direction = current_node.direction.clone();
            if current_node.align_with_end == false {
                directions = Direction::all_directions();
            } else {
                // 这边可以直接把current和end连起来，然后判断形成的trace是否与任何障碍交叉
                directions = vec![Direction::from_points(current_node.position, self.end)];
            }
            for direction in directions {
                // calculate the next position
                let direction_vec2 = direction.to_fixed_vec2();

                // fn get_new_position(direction_vec2: FixedVec2, current_position: FixedVec2, length: FixedPoint) -> FixedVec2 {                    
                //     let delta_x = direction_vec2.x * length;
                //     let delta_y = direction_vec2.y * length;
                //     FixedVec2 {
                //         x: current_position.x + delta_x,
                //         y: current_position.y + delta_y,
                //     }
                // }

                
                // secured the position of the new node
                // first push this node to the frontier, then we also have to consider a midway related to the goal

                // this closure depends on visited, direction, current_node, self.end, frontier
                
                
                if current_node.align_with_end == false {
                    let final_length = if !self.check_collision(*ASTAR_STRIDE) {
                        *ASTAR_STRIDE
                    } else {
                        let mut lower_bound = FixedPoint::from_num(0.0);
                        let mut upper_bound = *ASTAR_STRIDE;
                        while lower_bound + FixedPoint::DELTA < upper_bound {
                            let mid_length = (lower_bound + upper_bound) / 2;
                            if check_collision(mid_length) {
                                upper_bound = mid_length; // collision found, search in the lower half
                            } else {
                                lower_bound = mid_length; // no collision, search in the upper half
                            }
                        }
                        // assert_eq!(lower_bound, upper_bound, "Binary search should converge to a single point");
                        assert!(
                            (upper_bound - lower_bound).abs() <= FixedPoint::DELTA,
                            "Binary search should converge to a single point"
                        );
                        lower_bound // this is the length that does not collide with any obstacles
                    };
                    if final_length == FixedPoint::from_num(0.0) {
                        continue; // no valid movement in this direction
                    }
                    let length_bound_by_obstacles = final_length; // the length that does not collide with any obstacles

                    // try_push_node_to_frontier(final_length); // push the node with the final length

                    let old_x = current_node.position.x;
                    let old_y = current_node.position.y;
                    let new_x = old_x + direction_vec2.x * *ASTAR_STRIDE;
                    let new_y = old_y + direction_vec2.y * *ASTAR_STRIDE;

                    let x_min = old_x.min(new_x);
                    let y_min = old_y.min(new_y);
                    let x_max = old_x.max(new_x);
                    let y_max = old_y.max(new_y);

                    let x_max_norm = x_max / *ASTAR_STRIDE;
                    let y_max_norm = y_max / *ASTAR_STRIDE;
                    // to do: check "%"
                    let x_new_clamped = if new_x == old_x || x_max_norm.floor() == x_max_norm {
                        new_x
                    } else {
                        if x_max_norm > 0 {
                            x_max_norm.floor() * *ASTAR_STRIDE
                        } else {
                            x_max_norm.ceil() * *ASTAR_STRIDE
                        }
                    };
                    let y_new_clamped = if new_y == old_y || y_max_norm.floor() == y_max_norm {
                        new_y
                    } else {
                        if y_max_norm > 0 {
                            y_max_norm.floor() * *ASTAR_STRIDE
                        } else {
                            y_max_norm.ceil() * *ASTAR_STRIDE
                        }
                    };

                    let x_new_length = (x_new_clamped - old_x).abs();
                    let y_new_length = (y_new_clamped - old_y).abs();
                    let length_bound_by_grid = FixedPoint::max(x_new_length, y_new_length);
                    if length_bound_by_grid == 0 {
                        println!(
                            "old_x:{:?}, x_max_norm: {:?}, x_new_clamped: {:?}",
                            old_x, x_max_norm, x_new_clamped
                        );
                        println!(
                            "old_y:{:?}, y_max_norm: {:?}, y_new_clamped: {:?}",
                            old_y, y_max_norm, y_new_clamped
                        );
                    }

                    let end_lines = [
                        (self.end.x, self.end.y, 1.0, 0.0),
                        (self.end.x, self.end.y, 0.0, 1.0),
                        (self.end.x, self.end.y, 1.0, 1.0),
                        (self.end.x, self.end.y, 1.0, -1.0),
                    ];

                    let mut min_distance = FixedPoint::MAX;

                    let move_dir_x = new_x - old_x;
                    let move_dir_y = new_y - old_y;

                    for line in &end_lines {
                        let (line_x, line_y, line_dir_x, line_dir_y) = *line;

                        if let Some((ix, iy)) = line_intersection_infinite(
                            (old_x, old_y),
                            (new_x, new_y),
                            (line_x, line_y),
                            (
                                line_x + FixedPoint::from_num(line_dir_x),
                                line_y + FixedPoint::from_num(line_dir_y),
                            ),
                        ) {
                            let dx = ix - old_x;
                            let dy = iy - old_y;

                            if (move_dir_x * dx >= 0.0) && (move_dir_y * dy >= 0.0) {
                                if FixedPoint::max(dx.abs(), dy.abs()) < min_distance {
                                    min_distance = FixedPoint::max(dx.abs(), dy.abs());
                                }
                            }
                        }
                    }
                    fn line_intersection_infinite(
                        a1: (FixedPoint, FixedPoint),
                        a2: (FixedPoint, FixedPoint),
                        b1: (FixedPoint, FixedPoint),
                        b2: (FixedPoint, FixedPoint),
                    ) -> Option<(FixedPoint, FixedPoint)> {
                        let a_diff_x = a2.0 - a1.0;
                        let a_diff_y = a2.1 - a1.1;
                        let b_diff_x = b2.0 - b1.0;
                        let b_diff_y = b2.1 - b1.1;

                        let denominator = a_diff_x * b_diff_y - a_diff_y * b_diff_x;

                        if denominator.abs() < f32::EPSILON {
                            return None;
                        }

                        let t = ((b1.0 - a1.0) * b_diff_y - (b1.1 - a1.1) * b_diff_x) / denominator;

                        if t > 0.0 && t <= 1.0 {
                            let ix = a1.0 + t * a_diff_x;
                            let iy = a1.1 + t * a_diff_y;
                            Some((ix, iy))
                        } else {
                            None
                        }
                    }

                    let length_bound_by_end_point = min_distance;
                    println!(
                        "ob: {:?}, grid: {:?}, end: {:?}",
                        length_bound_by_obstacles, length_bound_by_grid, length_bound_by_end_point
                    );
                    let result_length =
                        FixedPoint::min(length_bound_by_obstacles, length_bound_by_grid);
                    try_push_node_to_frontier(result_length, false); // push the node with the result_length
                    if length_bound_by_end_point < result_length {
                        try_push_node_to_frontier(length_bound_by_end_point, true);
                    }
                } 

                // if result_length == length_bound_by_end_point{ // todo: put end in frontier
                //     let new_position = get_new_position(result_length);
                //     let new_direction = Direction::from_points(new_position, self.end);
                //     let new_length = FixedPoint::max((self.end.x-new_position.x).abs(), (self.end.y-new_position.y).abs());
                //     let astar_node_key = AstarNodeKey {
                //         position: self.end,
                //     };
                //     // check if the new position is already visited
                //     if !visited.contains(&astar_node_key) {

                //         // calculate the cost to reach the next node
                //         let turn_penalty = if new_direction == direction {
                //             0.0 // no turn penalty if the direction is the same
                //         } else {
                //             TURN_PENALTY
                //         };

                //         let length: f64 = ((new_direction.to_fixed_vec2().length() * new_length)).to_num();
                //         let actual_cost = current_node.actual_cost + length + turn_penalty;
                //         let actual_length = current_node.actual_length + length;
                //         let estimated_cost = octile_distance(&new_position, &self.end) * ESTIMATE_COEFFICIENT;
                //         let total_cost = actual_cost + estimated_cost;
                //         let new_node = AstarNode {
                //             position: self.end,
                //             direction: Some(new_direction),
                //             actual_cost,
                //             actual_length,
                //             estimated_cost,
                //             total_cost,
                //             prev_node: Some(current_node.clone()), // link to the previous node
                //         };
                //         // push directly to the frontier
                //         frontier.push(BinaryHeapItem {
                //             key: Reverse(NotNan::new(new_node.total_cost).unwrap()), // use Reverse to make it a min heap
                //             value: Rc::new(new_node),
                //         });
                //     }

                // }

                // let midway_length: Option<FixedPoint> = if self.end.x > x_min && self.end.x <x_max{
                //     Some((self.end.x - old_x).abs())
                // }else if self.end.y > y_min && self.end.y < y_max{
                //     Some((self.end.y - old_y).abs())
                // }else{
                //     None // no midway length, the end point is not in the range of the current segment
                // };
                // // try midway length
                // if let Some(midway_length) = midway_length {
                //     assert!(midway_length > FixedPoint::from_num(0.0), "Midway length should be greater than 0.0");
                //     assert!(midway_length < final_length, "Midway length should be less than or equal to the final length");
                //     // println!("A midway node is triggered! Position: {:?}, Direction: {:?}, Midway Length: {}",
                //     //     get_new_position(midway_length),
                //     //     direction,
                //     //     midway_length
                //     // );
                //     // push the midway node to the frontier
                //     try_push_node_to_frontier(midway_length);
                // }
                if DISPLAY_ASTAR {
                    display_and_block(&frontier); // display the initial state of the frontier
                }
            }
        }
        Err("No path found".to_string()) // no path found
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct AstarNodeKey {
    pub position: FixedVec2,
}

pub struct AstarNode {
    pub position: FixedVec2,
    pub direction: Option<Direction>, // the direction from the previous node to this node
    pub actual_cost: f64,             // the actual cost to reach this node from the start node
    pub actual_length: f64,
    pub estimated_cost: f64, // the estimated cost to reach the end node from this node
    pub total_cost: f64, // the total cost to reach this node from the start node, including the estimated cost to reach the end node
    pub prev_node: Option<Rc<AstarNode>>, // the previous node in the path, used for backtracking
    // 这边的判断align with end的变量去掉了
}

impl AstarNode {
    pub fn to_trace_path(&self, width: f32, clearance: f32) -> TracePath {
        let mut anchors = vec![self.position];
        let mut directions = vec![self.direction.as_ref().unwrap().clone()]; // start with the direction of the first segment
        let mut current_node = self.prev_node.clone();
        while let Some(node) = current_node {
            anchors.push(node.position);
            if let Some(direction) = &node.direction {
                directions.push(direction.clone());
            }
            current_node = node.prev_node.clone();
        }
        assert!(
            anchors.len() == directions.len() + 1,
            "The number of anchors should be one more than the number of directions"
        );
        anchors.reverse(); // reverse the anchors to get the correct order
        directions.reverse(); // reverse the directions to get the correct order
        let mut segments: Vec<TraceSegment> = Vec::new();
        for i in 0..directions.len() {
            let start = anchors[i];
            let end = anchors[i + 1];
            let direction = directions[i].clone();
            let segment = TraceSegment {
                start,
                end,
                direction,
                width,
                clearance,
            };
            segments.push(segment);
        }
        let anchors = TraceAnchors(anchors);
        assert!(
            self.estimated_cost == 0.0,
            "The estimated cost should be 0.0 for the trace path"
        );
        TracePath {
            anchors,
            segments,
            length: self.actual_length,
        }
    }
    pub fn to_renderables(
        &self,
        width: f32,
        clearance: f32,
        color: [f32; 3],
    ) -> Vec<RenderableBatch> {
        // This function is used to convert the AstarNode to a TraceSegment
        // It assumes that the node has a direction and a position
        let opaque_color = [color[0], color[1], color[2], 1.0]; // make the color opaque
        let transparent_color = [color[0], color[1], color[2], 0.5]; // make the color transparent
        if let Some(direction) = &self.direction {
            // If the node has a direction, we can create a TraceSegment
            let trace_segment = TraceSegment {
                start: self.prev_node.as_ref().unwrap().position,
                end: self.position,
                direction: direction.clone(),
                width,
                clearance,
            };
            let renderables = trace_segment.to_renderables(opaque_color);
            let clearance_renderables = trace_segment.to_clearance_renderables(transparent_color);
            vec![
                RenderableBatch(renderables),
                RenderableBatch(clearance_renderables),
            ]
        } else {
            let shape_renderable = ShapeRenderable {
                shape: PrimShape::Circle(CircleShape {
                    position: self.position.to_float(),
                    diameter: width,
                }),
                color: opaque_color,
            };
            let shape_clearance_renderable = ShapeRenderable {
                shape: PrimShape::Circle(CircleShape {
                    position: self.position.to_float(),
                    diameter: width + clearance * 2.0,
                }),
                color: transparent_color,
            };
            vec![RenderableBatch(vec![
                shape_renderable,
                shape_clearance_renderable,
            ])]
        }
    }
}

pub struct AStarResult {
    pub trace_path: TracePath,
}
