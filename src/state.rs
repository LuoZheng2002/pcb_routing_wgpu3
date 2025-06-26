use std::{collections::HashMap, sync::{Arc, Mutex}, time::Instant};

use cgmath::{Euler, Quaternion};
use wgpu::util::DeviceExt;

use crate::{app::RENDER_CONTEXT, input_context::InputContext, orthographic_camera::OrthographicCamera, shape_instance::ShapeInstance, shape_mesh::ShapeMesh, transparent_pipeline::TransparentShapeBatch, vertex::Vertex};

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
    // pub model_render_submissions: HashMap<ModelMeta, Vec<ShapeInstance>>,
    // use Arc here because we need to map the container to another container
    // pub ui_render_submissions: HashMap<TextureMeta, Vec<UIInstance>>,
    // pub ui_render_instructions: Vec<UIRenderInstruction>,
    pub fps: u32,

    // pub canvas: Option<UISpan>,
    // pub text: Option<UIText>,
}

impl State {
    

    pub fn init(&mut self) {
        
    }

    pub fn update(&mut self, window_size: &winit::dpi::PhysicalSize<u32>) {
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

        let scale = 1.0;
        let speed = 0.1;
        let delta_angle = current_time * speed;

        let mesh1 = RENDER_CONTEXT.with_borrow(|rc| {
            rc.as_ref().unwrap().square_mesh.clone()
        });

        let instance1 = ShapeInstance {
            position: [0.0, 0.0, 0.0].into(),
            rotation: Quaternion::from(Euler::new(
                cgmath::Rad(delta_angle),
                cgmath::Rad(delta_angle),
                cgmath::Rad(delta_angle),
            )),
            scale: cgmath::Vector3::new(scale, scale, scale),
            color: [1.0, 0.0, 0.0, 0.5],
        };

        let transparent_shape_batch = TransparentShapeBatch(
            vec![(mesh1, vec![instance1])],
        );

        self.transparent_shape_submissions = Some(vec![transparent_shape_batch]);
        

        // let instance1 = ShapeInstance {
        //     position: [-1.0, 0.0, 0.0].into(),
        //     rotation: Quaternion::from(Euler::new(
        //         cgmath::Rad(delta_angle),
        //         cgmath::Rad(delta_angle),
        //         cgmath::Rad(delta_angle),
        //     )),
        //     scale: cgmath::Vector3::new(scale, scale, scale),
        // };
        // let instance2 = ShapeInstance {
        //     position: [1.0, 0.0, 0.0].into(),
        //     rotation: Quaternion::from(Euler::new(
        //         cgmath::Rad(-delta_angle),
        //         cgmath::Rad(delta_angle),
        //         cgmath::Rad(-delta_angle),
        //     )),
        //     scale: cgmath::Vector3::new(scale, scale, scale),
        // };
        // self.submit_renderable(model_meta.clone(), instance1);
        // self.submit_renderable(model_meta.clone(), instance2);

        // let ui_meta1 = UIRenderableMeta::Font { character: 'F' };
        // let ui_instance1 = UIInstance {
        //     color: cgmath::Vector4::new(1.0, 0.0, 1.0, 1.0),
        //     location: [-0.2, 0.9, 0.7, -0.1],
        //     sort_order: 0,
        //     use_texture: true,
        // };
        // self.submit_ui_renderable(ui_meta1, ui_instance1);

        // to do
        let screen_width = window_size.width;
        let screen_height = window_size.height;
        // panic!()

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
        }
    }
}



// if use event or callback, need mutable shared reference.
// if use traversal, do not need shared reference, but is hard to communicate between each component.
// mouse event needs to specify layers, distributive calculation.
// pure tree -> component models with states (here we update the states) -> renderables
// length formula + dependencies -> actual length, manual override
// update actual length

// traverse twice to send mouse events.

// the first time gathers all elements that responds to the mouse event
// the second time notifies the one that wins the bid
// (actually, we hardly have any circumstances where we have competing elements)

// priority: bound > coop > manual override = preferred

// manual override will report a length, and the elements on the direction of the modification will be affected.
// the total least bound of all the elements will be calculated, if the least bound is greater than the target one, it will limit the manual override
// if the target length is less than the least bound, the changes will be applied to all the elements as uniformly as possible.

// so we have actual start point, actual end point, etc. preferred length, lower bound, upper bound, ...

// if one bound is not satisfied (actual length too small), then it will try to subtract length from other elements.
// other elements will report the actual length they have reduced

// only parents are allowed to change size
// child does not need to react to parents
// the split of a span can be moved around (min, max, preferred for each cell of a span)
// so, a span should have its elements uniformly distributed
// parents should never try to fit children
// if we want parents to fit children, we can set the length of the children to be the same as the parents
// if the children fails to fit, ...
// do not consider different resolution / screen size
