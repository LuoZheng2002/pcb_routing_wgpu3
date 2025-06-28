use std::cell::RefCell;

use crate::{input_context::InputContext, render_context::RenderContext, state::State};


#[derive(Default)]
pub struct Context{
    pub render_context: Option<RenderContext>,
    pub state: RefCell<State>,
    pub input_context: RefCell<InputContext>,
}

