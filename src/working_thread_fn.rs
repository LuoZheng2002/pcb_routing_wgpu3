use std::sync::{Arc, Mutex};

use cgmath::Deg;

use crate::{pad::{Pad, PadShape}, pcb_problem::{Color, PcbProblem}, pcb_render_model::{self, PcbRenderModel}, vec2::FloatVec2};





pub fn working_thread_fn(pcb_render_model: Arc<Mutex<PcbRenderModel>>){
    println!("Working thread started");
    let pcb_width = 15.0;
    let pcb_height = 10.0;
    let mut pcb_problem = PcbProblem::new(pcb_width, pcb_height);
    let red_net_id = pcb_problem.add_net(Color{r: 255, g: 0, b: 0});
    let _ = pcb_problem.add_connection(
        red_net_id, 
        Pad{
            position: FloatVec2{x: 0.0, y: 0.0},
            shape: PadShape::Circle { diameter: 1.2 },
            rotation: Deg(0.0),
            clearance: 0.2,
        },
        Pad { 
            position: FloatVec2 { x: 0.0, y: 5.0 }, 
            shape: PadShape::Square { 
                side_length: 1.0,
            },
            rotation: Deg(15.0),
            clearance: 0.3,
        }
    );
    let result = pcb_problem.solve(pcb_render_model.clone());
    match result {
        Ok(_) => {
            println!("PCB problem solved successfully");
        },
        Err(e) => {
            println!("Failed to solve PCB problem: {}", e);
        }
    }
}