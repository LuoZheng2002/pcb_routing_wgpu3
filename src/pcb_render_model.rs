use std::sync::{Arc, Mutex};

use crate::{pad::PadShape, vec2::FloatVec2};

pub struct PadRenderable {
    pub position: FloatVec2,
    pub shape: PadShape,
    pub rotation: cgmath::Deg<f32>, // Rotation in degrees
    pub color: [f32; 4],            // RGBA color
}

pub struct TraceSegmentRenderable {
    pub start: FloatVec2,
    pub end: FloatVec2,
    pub width: f32,      // Width of the trace segment
    pub color: [f32; 4], // RGBA color
}

#[derive(Default)]
pub struct PcbRenderModel {
    pub width: f32,
    pub height: f32,
    pub pad_renderables: Vec<PadRenderable>,
    pub trace_segment_renderables: Vec<TraceSegmentRenderable>,
}

pub trait UpdatePcbRenderModel {
    fn update_pcb_render_model(&self, pcb_render_model: PcbRenderModel);
}

impl UpdatePcbRenderModel for Arc<Mutex<PcbRenderModel>> {
    fn update_pcb_render_model(&self, pcb_render_model: PcbRenderModel) {
        let mut model = self.lock().unwrap();
        *model = pcb_render_model;
    }
}
