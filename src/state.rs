use std::{
    collections::HashMap,
    sync::{Arc, Mutex},
    time::Instant,
};

use cgmath::{Euler, Quaternion};
use wgpu::util::DeviceExt;

use crate::{
    input_context::InputContext, orthographic_camera::OrthographicCamera,
    render_context::RenderContext, shape_instance::ShapeInstance, shape_mesh::ShapeMesh,
    transparent_pipeline::TransparentShapeBatch, vertex::Vertex,
};

// model path,
pub struct State {
    // camera stuff
    pub camera: OrthographicCamera,
    // accumulated time
    pub timer: Option<Instant>,
    pub prev_time: Option<f32>,
    pub fps_timer: Option<Instant>,
    pub cursor_timer: Option<Instant>,
    pub accumulated_frame_num: u32,
    pub transparent_shape_submissions: Option<Vec<TransparentShapeBatch>>,
    pub fps: u32,

    pub pcb_width: f32,
    pub pcb_height: f32,
}

impl State {
    pub fn init(&mut self) {}

    pub fn update(&mut self, render_context: &RenderContext) {
        // calculate fps every 1 second
        let fps_timer = self.fps_timer.get_or_insert_with(|| Instant::now());
        let cursor_timer = self.cursor_timer.get_or_insert_with(|| Instant::now());
        let current_fps_time = fps_timer.elapsed().as_secs_f32();
        if current_fps_time >= 1.0 {
            println!("FPS: {}", self.accumulated_frame_num);
            self.fps = self.accumulated_frame_num;
            self.accumulated_frame_num = 0;
            *fps_timer = Instant::now();
        } else {
            self.accumulated_frame_num += 1;
        }
        let current_cursor_time = cursor_timer.elapsed().as_secs_f32();
        let mut cursor_blink = false;
        if current_cursor_time >= 0.5 {
            // println!("cursor: {:?}", input_context.mouse_position());
            *cursor_timer = Instant::now();
            cursor_blink = true;
        }
        let timer = self.timer.get_or_insert_with(|| Instant::now());
        let current_time = timer.elapsed().as_secs_f32();
        let prev_time = self.prev_time.get_or_insert(current_time);
        let delta_time = current_time - *prev_time;
        assert!(delta_time >= 0.0);
        let speed = 0.1;
        let delta_angle = current_time * speed;

        // update camera
        let pcb_aspect_ratio = self.pcb_width / self.pcb_height;
        let screen_aspect_ratio = {
            let size = *render_context.size.borrow();
            size.width as f32 / size.height as f32
        };
        let pcb_margin_scale: f32 = 1.2;
        let (orthographic_width, orthographic_height) = {
            if pcb_aspect_ratio > screen_aspect_ratio {
                let orthographic_width = self.pcb_width * pcb_margin_scale;
                let orthographic_height = orthographic_width / screen_aspect_ratio;
                (orthographic_width, orthographic_height)
            } else {
                let orthographic_height = self.pcb_height * pcb_margin_scale;
                let orthographic_width = orthographic_height * screen_aspect_ratio;
                (orthographic_width, orthographic_height)
            }
        };
        // the top left corner of the PCB is (0, 0) in world space
        // self.camera.left = -orthographic_width / 2.0 + self.pcb_width / 2.0;
        // self.camera.right = orthographic_width / 2.0 + self.pcb_width / 2.0;
        // self.camera.bottom = -orthographic_height / 2.0 + self.pcb_height / 2.0;
        // self.camera.top = orthographic_height / 2.0 + self.pcb_height / 2.0;
        self.camera.left = -orthographic_width / 2.0;
        self.camera.right = orthographic_width / 2.0;
        self.camera.bottom = -orthographic_height / 2.0;
        self.camera.top = orthographic_height / 2.0;
        // render submissions
        let mesh1 = render_context.circle_mesh.clone();

        let instance1 = ShapeInstance {
            position: [0.0, 0.0, 0.0].into(),
            rotation: Quaternion::from(Euler::new(
                cgmath::Rad(0.0),
                cgmath::Rad(0.0),
                cgmath::Rad(0.0),
            )),
            scale: cgmath::Vector3::new(1.27, 1.27, 1.27),
            color: [1.0, 0.0, 0.0, 0.9],
        };

        let instance2 = ShapeInstance {
            position: [2.54, 0.0, 0.0].into(),
            rotation: Quaternion::from(Euler::new(
                cgmath::Rad(0.0),
                cgmath::Rad(0.0),
                cgmath::Rad(0.0),
            )),
            scale: cgmath::Vector3::new(1.27, 1.27, 1.27),
            color: [1.0, 0.0, 0.0, 0.9],
        };

        let pcb_rect_mesh = render_context.square_mesh.clone();
        let pcb_rect_instance = ShapeInstance {
            position: [0.0, 0.0, 0.0].into(),
            rotation: Quaternion::from(Euler::new(
                cgmath::Rad(0.0),
                cgmath::Rad(0.0),
                cgmath::Rad(0.0),
            )),
            scale: cgmath::Vector3::new(self.pcb_width, self.pcb_height, 1.0),
            color: [1.0, 1.0, 1.0, 0.3],
        };
        let pcb_rect_batch = TransparentShapeBatch(vec![(pcb_rect_mesh, vec![pcb_rect_instance])]);
        let pad_batch = TransparentShapeBatch(vec![(mesh1, vec![instance1, instance2])]);
        self.transparent_shape_submissions = Some(vec![pcb_rect_batch, pad_batch]);
        // self.transparent_shape_submissions = Some(vec![pcb_rect_batch]);
    }
}

impl Default for State {
    fn default() -> Self {
        State {
            camera: OrthographicCamera::new(
                cgmath::Point3::new(0.0, 0.0, 1.0),
                cgmath::Point3::new(0.0, 0.0, 0.0),
                cgmath::Vector3::new(0.0, 1.0, 0.0),
                -1.0,
                1.0,
                -1.0,
                1.0,
                -10.0,
                10.0,
            ),
            timer: None,
            prev_time: None,
            fps_timer: None,
            cursor_timer: None,
            accumulated_frame_num: 0,
            transparent_shape_submissions: None,
            fps: 0,
            pcb_width: 15.0,
            pcb_height: 10.0,
        }
    }
}
