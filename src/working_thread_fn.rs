use std::sync::{Arc, Mutex};

use cgmath::Deg;

use crate::{
    pad::{Pad, PadShape},
    pcb_problem::{self, Color, PcbProblem},
    pcb_render_model::{self, PcbRenderModel},
    test_pcb_problem::{pcb_problem1, pcb_problem2},
    vec2::FloatVec2,
};

pub fn working_thread_fn(pcb_render_model: Arc<Mutex<PcbRenderModel>>) {
    println!("Working thread started");
    let pcb_problem = pcb_problem1();
    let result = pcb_problem.solve(pcb_render_model.clone());
    match result {
        Ok(_) => {
            println!("PCB problem solved successfully");
        }
        Err(e) => {
            println!("Failed to solve PCB problem: {}", e);
        }
    }
}
