

use core::num;
use std::{cell::RefCell, collections::{BTreeSet, HashMap, HashSet}, num::NonZeroUsize, vec};

use crate::{pad::Pad, trace_path::{TraceAnchors, TracePath}, vec2::{FixedPoint, FixedVec2}};

// use shared::interface_types::{Color, ColorGrid};

// use crate::{grid::Point, hyperparameters::{HALF_PROBABILITY_RAW_SCORE, ITERATION_TO_PRIOR_PROBABILITY, LENGTH_PENALTY_RATE, TURN_PENALTY_RATE}};



#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct Color{
    pub r: u8,
    pub g: u8,
    pub b: u8,
}

#[derive(Debug, Clone)]
pub struct Connection{
    pub source: Pad,
    pub sink: Pad,
    pub traces: HashMap<TraceID, TraceInfo>, // List of traces connecting the source and sink pads
}

#[derive(Debug, Clone)]
pub struct NetInfo{
    pub net_id: usize,
    pub color: Color, // Color of the net
    pub connections: Vec<Connection>, // List of connections in the net, the source pad is the same
}

#[derive(Copy, Debug, Clone, PartialEq, Hash, Eq, PartialOrd, Ord)]
pub struct NetID(pub usize);
#[derive(Copy, Debug, Clone, PartialEq, Hash, Eq, PartialOrd, Ord)]
pub struct PadPairID(pub usize);
#[derive(Copy, Debug, Clone, PartialEq, Hash, Eq, PartialOrd, Ord)]
pub struct TraceID(pub usize);



#[derive(Debug, Clone)]
pub struct PadPair{
    pub net_id: NetID,
    pub pad_pair_id: PadPairID,
    pub start: FixedPoint, // Start point of the trace
    pub end: FixedPoint, // End point of the trace
}








pub struct PostProcessInput{
    pub width: usize,
    pub height: usize,
    pub nets: HashMap<NetID, NetInfo>,
    pub net_to_pads: HashMap<NetID, HashSet<FixedVec2>>, // NetID to list of pad coordinates
    // output
    pub net_to_pad_pairs: HashMap<NetID, HashSet<PadPairID>>, // NetID to PadPairToRouteID to PadPairToRoute
    pub pad_pairs: HashMap<PadPairID, PadPair>, // PadPairToRouteID to PadPairToRoute
    pub pad_pair_to_traces: HashMap<PadPairID, HashSet<TraceID>>, // PadPairID to TraceID
    pub traces: HashMap<TraceID, TraceInfo>, // TraceID to Trace
}

#[derive(Debug, Clone)]
pub struct TraceInfo{
    pub trace_path: TracePath, // The path of the trace
    pub trace_length: f64, // The length of the trace
    pub iteration: NonZeroUsize, // The iteration that the trace belongs to, starting from 1
    pub posterior_normalized: RefCell<Option<f64>>, // to be accessed in the next iteration
    pub temp_posterior: RefCell<Option<f64>>, // serve as a buffer for simultaneous updates
    pub collision_adjacency: HashSet<TraceID>, // The set of traces that collide with this trace
}



// impl TraceInfo{
//     fn calculate_score(&self)->f64{
//         // calculate turns
//         let mut turns = 0;
//         let mut last_direction = self.trace_directions.first().cloned().unwrap_or(Direction { x: 0, y: 0 });
//         if self.trace_directions.len() > 0{
//             for direction in self.trace_directions.iter().skip(1) {
//                 if direction.x != last_direction.x || direction.y != last_direction.y {
//                     turns += 1;
//                 }
//                 last_direction = direction.clone();
//             }
//         }
//         let score_raw = self.trace_length * LENGTH_PENALTY_RATE + turns as f64 * TURN_PENALTY_RATE;
//         let k = f64::ln(2.0)/ HALF_PROBABILITY_RAW_SCORE;
//         let score = f64::exp(-k * score_raw);
//         assert!(score >= 0.0 && score <= 1.0, "Score must be between 0 and 1, got: {}", score);
//         score
//     }
//     pub fn get_score(&self)->f64{
//         // let mut score_cache = self.score_cache.borrow_mut();
//         // *score_cache.get_or_insert_with(||{
//         //     self.calculate_score()
//         // })
//         // we do not use cache until there is performance issue
//         self.calculate_score()
//     }
//     fn calculate_normalized_prior_probability(&self, num_traces_in_the_same_iteration: usize)->f64{
//         let sum_probability = ITERATION_TO_PRIOR_PROBABILITY.get(&self.iteration)
//             .cloned()
//             .unwrap_or_else(|| panic!("No prior probability for iteration {:?}", self.iteration));
//         sum_probability / (num_traces_in_the_same_iteration as f64)
//     }
//     /// this prior probability is not normalized
//     pub fn get_normalized_prior_probability(&self, num_traces_in_the_same_iteration: usize)->f64{
//         // let mut prior_probability_cache = self.prior_probability_cache.borrow_mut();
//         // *prior_probability_cache.get_or_insert_with(||{
//         //     self.calculate_prior_probability()
//         // })
//         // we do not use cache until there is performance issue
//         self.calculate_normalized_prior_probability(num_traces_in_the_same_iteration)
//     }
    
//     // pub fn calculate_fallback_posterior_unnormalized(&self, num_traces_in_the_same_iteration: usize)->f64{
//     //     // calculate the posterior probability of the trace
//     //     // this is used when there is no old posterior probability available
//     //     let prior_probability = self.get_prior_probability();
//     //     let normalized_prior_probability = prior_probability / (num_traces_in_the_same_iteration as f64);

//     //     let score = self.get_score();
//     //     let posterior_unnormalized = prior_probability * score;
//     //     // we use the number of traces in the same iteration to normalize the posterior probability
//     //     let posterior_normalized = posterior_unnormalized / (num_traces_in_the_same_iteration as f64);
//     //     assert!(posterior_normalized >= 0.0 && posterior_normalized <= 1.0, "Posterior normalized must be between 0 and 1, got: {}", posterior_normalized);
//     //     posterior_normalized
//     // }
//     pub fn get_posterior_normalized_with_fallback(&self, num_traces_in_the_same_iteration: usize)->f64{
//         let posterior_normalized = self.posterior_normalized.borrow();
//         if let Some(old_posterior) = posterior_normalized.as_ref() {
//             *old_posterior
//         } else {            
//             self.get_normalized_prior_probability(num_traces_in_the_same_iteration)
//         }
//     }
//     // call stack: want to sample traces that block the way -> call get_posterior_normalized for other traces
//     // -> use num_traces_in_the_same_iteration to get the normalized prior probability
//     // -> use the normalized prior probability as the normalized posterior probability 

//     // want to sample traces -> should already have the posterior_normalized
// }


#[derive(Clone)]
pub struct ProbaGridProblem{
    pub width: usize,
    pub height: usize,
    pub nets: HashMap<NetID, NetInfo>,
    pub net_to_pads: HashMap<NetID, HashSet<FixedVec2>>, // NetID to list of pad coordinates
}


#[derive(Copy, Debug, Clone, PartialEq, Hash, Eq, PartialOrd, Ord)]
pub struct IterationNum(pub NonZeroUsize);


// (0, 0) center, up, right
pub struct PcbProblem{
    pub width: f32,
    pub height: f32,
    pub nets: HashMap<NetID, NetInfo>,
    pub visited_traces: BTreeSet<TraceAnchors>,    
    pub next_iteration: NonZeroUsize, // The next iteration to be processed, starting from 1
    pub trace_id_generator: Box<dyn Iterator<Item=TraceID> + Send + 'static>, // A generator for TraceID, starting from 0
}


impl PcbProblem{
    pub fn collides_with_border(&self)->bool{
        todo!()
    }

    // pub fn get_num_traces_in_the_same_iteration(&self, trace_id: TraceID)->usize{
    //     let trace_info = self.traces.get(&trace_id).unwrap();
    //     let pad_pair_id = trace_info.pad_pair_id;
    //     let iteration_num = trace_info.iteration;
    //     let num = self.pad_pair_to_traces.get(&pad_pair_id).unwrap()
    //         .get(&IterationNum(iteration_num)).unwrap()
    //         .len();
    //     assert!(num > 0, "There should be at least one trace in the same iteration");
    //     num
    // }
}