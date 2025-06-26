use std::{cell::RefCell, process::exit, sync::Arc};

use winit::{
    application::ApplicationHandler,
    dpi::LogicalPosition,
    event::{DeviceEvent, DeviceId, WindowEvent},
    event_loop::ActiveEventLoop,
    window::{Window, WindowAttributes, WindowId},
};

use crate::{input_context::InputContext, render_context::RenderContext, state::State};

thread_local! {
    pub static RENDER_CONTEXT: RefCell<Option<RenderContext>> = RefCell::new(None);
    pub static STATE: RefCell<State> = RefCell::new(State::default());
    pub static INPUT_CONTEXT: RefCell<InputContext> = RefCell::new(InputContext::default());
}

#[derive(Default)]
pub struct App {
    pub window: Option<Arc<Window>>,
}

impl ApplicationHandler for App {
    fn resumed(&mut self, event_loop: &ActiveEventLoop) {
        let attributes = WindowAttributes::default()
            .with_inner_size(winit::dpi::LogicalSize::new(800.0, 600.0))
            .with_position(LogicalPosition::new(0, 0));
        let window = event_loop.create_window(attributes).unwrap();
        let window = Arc::new(window);
        RENDER_CONTEXT.with_borrow_mut(|rc| {
            *rc = Some(RenderContext::new(window.clone()));
        });
        STATE.with_borrow_mut(|state| {
            state.init();
        });
        self.window = Some(window);
    }
    fn device_event(
        &mut self,
        _event_loop: &ActiveEventLoop,
        _device_id: DeviceId,
        event: DeviceEvent,
    ) {
        INPUT_CONTEXT.with_borrow_mut(|input_context| {
            input_context.handle_device_event(&event);
        });
    }

    fn window_event(&mut self, event_loop: &ActiveEventLoop, id: WindowId, event: WindowEvent) {
        INPUT_CONTEXT.with_borrow_mut(|input_context| {
            input_context.handle_window_event(&event);
        });
        match event {
            WindowEvent::CloseRequested => {
                println!("The close button was pressed; stopping");
                std::process::abort();
                event_loop.exit();
            }
            WindowEvent::RedrawRequested => {
                self.window.as_ref().unwrap().request_redraw();
                let size = RENDER_CONTEXT.with_borrow(|rc| {
                    rc.as_ref().unwrap().size
                });
                STATE.with_borrow_mut(|state| {
                    state.update(&size);
                });
                match RENDER_CONTEXT.with_borrow_mut(|rc| {
                    rc.as_mut().unwrap().render()
                }){
                    Ok(_) => {}
                    // Reconfigure the surface if it's lost or outdated
                    Err(wgpu::SurfaceError::Lost | wgpu::SurfaceError::Outdated) => {
                        RENDER_CONTEXT.with_borrow_mut(|rc| {
                            rc.as_mut().unwrap().resize(size);
                        });
                    }
                    // The system is out of memory, we should probably quit
                    Err(wgpu::SurfaceError::OutOfMemory | wgpu::SurfaceError::Other) => {
                        log::error!("OutOfMemory");
                        event_loop.exit();
                    }

                    // This happens when the a frame takes too long to present
                    Err(wgpu::SurfaceError::Timeout) => {
                        log::warn!("Surface timeout")
                    }
                }
            }
            WindowEvent::Resized(new_size) => {
                RENDER_CONTEXT.with_borrow_mut(|rc| {
                    rc.as_mut().unwrap().resize(new_size);
                });
            }
            _ => (),
        }
    }
}
