use core::panic;
use std::{
    cell::RefCell,
    collections::{BTreeSet, BinaryHeap, HashMap, HashSet},
    num::NonZeroUsize,
    rc::Rc,
    sync::{Arc, Mutex},
};

use ordered_float::NotNan;
use rand::distr::{weighted::WeightedIndex, Distribution};

use crate::{
    astar::AStarModel, binary_heap_item::BinaryHeapItem, hyperparameters::{CONSTANT_LEARNING_RATE, ITERATION_TO_NUM_TRACES, ITERATION_TO_PRIOR_PROBABILITY, LINEAR_LEARNING_RATE, MAX_GENERATION_ATTEMPTS, NEXT_ITERATION_TO_REMAINING_PROBABILITY, OPPORTUNITY_COST_WEIGHT, SCORE_WEIGHT}, pad::Pad, pcb_render_model::{PcbRenderModel, RenderableBatch, ShapeRenderable, UpdatePcbRenderModel}, prim_shape::PrimShape, trace_path::{TraceAnchors, TracePath}, vec2::FixedVec2
};

// use shared::interface_types::{Color, ColorGrid};

// use crate::{grid::Point, hyperparameters::{HALF_PROBABILITY_RAW_SCORE, ITERATION_TO_PRIOR_PROBABILITY, LENGTH_PENALTY_RATE, TURN_PENALTY_RATE}};

#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct Color {
    pub r: u8,
    pub g: u8,
    pub b: u8,
}

impl Color{
    pub fn to_float4(&self, alpha: f32) -> [f32; 4] {
        [
            self.r as f32 / 255.0,
            self.g as f32 / 255.0,
            self.b as f32 / 255.0,
            alpha,
        ]
    }
}

#[derive(Debug, Clone)]
pub struct Connection {
    pub net_id: NetID,               // The net that the connection belongs to
    pub connection_id: ConnectionID, // Unique identifier for the connection
    pub source: Pad,
    pub sink: Pad,
    pub trace_width: f32, // Width of the trace
    pub trace_clearance: f32, // Clearance around the trace
    // pub traces: HashMap<TraceID, TraceInfo>, // List of traces connecting the source and sink pads
}

#[derive(Debug, Clone)]
pub struct NetInfo {
    pub net_id: NetID,
    pub color: Color,                                   // Color of the net
    pub connections: HashMap<ConnectionID, Rc<Connection>>, // List of connections in the net, the source pad is the same
}

#[derive(Copy, Debug, Clone, PartialEq, Hash, Eq, PartialOrd, Ord)]
pub struct NetID(pub usize);
#[derive(Copy, Debug, Clone, PartialEq, Hash, Eq, PartialOrd, Ord)]
pub struct ConnectionID(pub usize);
#[derive(Copy, Debug, Clone, PartialEq, Hash, Eq, PartialOrd, Ord)]
pub struct ProbaTraceID(pub usize);

// #[derive(Debug, Clone)]
// pub struct PadPair{
//     pub net_id: NetID,
//     pub pad_pair_id: PadPairID,
//     pub start: FixedPoint, // Start point of the trace
//     pub end: FixedPoint, // End point of the trace
// }

// pub struct PostProcessInput{
//     pub width: usize,
//     pub height: usize,
//     pub nets: HashMap<NetID, NetInfo>,
//     pub net_to_pads: HashMap<NetID, HashSet<FixedVec2>>, // NetID to list of pad coordinates
//     // output
//     pub net_to_pad_pairs: HashMap<NetID, HashSet<PadPairID>>, // NetID to PadPairToRouteID to PadPairToRoute
//     pub pad_pairs: HashMap<PadPairID, PadPair>, // PadPairToRouteID to PadPairToRoute
//     pub pad_pair_to_traces: HashMap<PadPairID, HashSet<TraceID>>, // PadPairID to TraceID
//     pub traces: HashMap<TraceID, TraceInfo>, // TraceID to Trace
// }

#[derive(Debug)]
pub struct ProbaTrace {
    pub net_id: NetID,                        // The net that the trace belongs to
    pub connection_id: ConnectionID,          // The connection that the trace belongs to
    pub proba_trace_id: ProbaTraceID,         // Unique identifier for the trace
    pub trace_path: TracePath,                // The path of the trace
    pub iteration: NonZeroUsize, // The iteration that the trace belongs to, starting from 1
    pub posterior: RefCell<Option<f64>>, // to be accessed in the next iteration
    pub temp_posterior: RefCell<Option<f64>>, // serve as a buffer for simultaneous updates
}

impl ProbaTrace{
    fn get_normalized_prior(
        &self,
    ) -> f64 {
        *ITERATION_TO_PRIOR_PROBABILITY.get(&self.iteration)
            .expect(format!("No prior probability for iteration {:?}", self.iteration).as_str())
    }

    pub fn get_posterior_with_fallback(
        &self,
    ) -> f64 {
        let posterior = self.posterior.borrow();
        if let Some(posterior) = posterior.as_ref() {
            *posterior
        } else {
            self.get_normalized_prior()
        }
    }
}

#[derive(Debug, Clone)]
pub struct FixedTrace {
    pub net_id: NetID,               // The net that the trace belongs to
    pub connection_id: ConnectionID, // The connection that the trace belongs to
    pub trace_path: TracePath,
}

#[derive(Debug, Clone)]
pub enum Traces {
    Fixed(FixedTrace), // A trace that is fixed and does not change
    Probabilistic(HashMap<ProbaTraceID, Rc<ProbaTrace>>), // A trace that is probabilistic and can change
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
pub struct ProbaGridProblem {
    pub width: usize,
    pub height: usize,
    pub nets: HashMap<NetID, NetInfo>,
    pub net_to_pads: HashMap<NetID, HashSet<FixedVec2>>, // NetID to list of pad coordinates
}

#[derive(Copy, Debug, Clone, PartialEq, Hash, Eq, PartialOrd, Ord)]
pub struct IterationNum(pub NonZeroUsize);

// backtrack search:
// each node contains trace candidates, their rankings, and already determined traces
// coarse mode: sample multiple traces at a time
// fine mode: change the model immediately when one trace is determined

// separate the problem, the probabilistic model, and the solution

// (0, 0) center, up, right
pub struct PcbProblem {
    pub width: f32,
    pub height: f32,
    pub nets: HashMap<NetID, NetInfo>, // NetID to NetInfo
    pub net_id_generator: Box<dyn Iterator<Item = NetID> + Send + 'static>, // A generator for NetID, starting from 0
    pub connection_id_generator: Box<dyn Iterator<Item = ConnectionID> + Send + 'static>, // A generator for ConnectionID, starting from 0
}

// a connection can have either a determined trace or multiple probabilistic traces
pub struct ProbaModel {
    pub trace_id_generator: Box<dyn Iterator<Item = ProbaTraceID> + Send + 'static>, // A generator for TraceID, starting from 0
    pub connection_to_traces: HashMap<ConnectionID, Traces>, // ConnectionID to list of traces
    // pub visited_traces: BTreeSet<TraceAnchors>,
    pub collision_adjacency: HashMap<ProbaTraceID, HashSet<ProbaTraceID>>, // TraceID to set of colliding TraceIDs
    pub next_iteration: NonZeroUsize, // The next iteration to be processed, starting from 1
}

impl ProbaModel {
    pub fn create_and_solve(
        problem: &PcbProblem,
        fixed_traces: &HashMap<ConnectionID, FixedTrace>,
        pcb_render_model: Arc<Mutex<PcbRenderModel>>,
    ) -> Self {
        let mut connection_ids: Vec<ConnectionID> = Vec::new();
        for net_info in problem.nets.values() {
            for connection in net_info.connections.keys() {
                connection_ids.push(*connection);
            }
        }
        let mut connection_to_traces: HashMap<ConnectionID, Traces> = HashMap::new();
        for connection_id in connection_ids {
            let traces = if let Some(fixed_trace) = fixed_traces.get(&connection_id) {
                Traces::Fixed(fixed_trace.clone())
            } else {
                Traces::Probabilistic(HashMap::new())
            };
            connection_to_traces.insert(connection_id, traces);
        }
        let mut proba_model = ProbaModel {
            trace_id_generator: Box::new((0..).map(ProbaTraceID)),
            connection_to_traces,
            collision_adjacency: HashMap::new(),
            next_iteration: NonZeroUsize::new(1).expect("Next iteration must be non-zero"),
        };
        // display and block
        let display_and_block = |proba_model: &ProbaModel|{
            let render_model = proba_model.to_pcb_render_model(problem);
            pcb_render_model.update_pcb_render_model(render_model);
            println!("Press Enter to continue...");
            let mut input = String::new();
            let _ = std::io::stdin().read_line(&mut input).unwrap();
        };
        display_and_block(&proba_model);
        

        // sample and then update posterior
        for j in 0..4{
            println!("Sampling new traces for iteration {}", j + 1);
            proba_model.sample_new_traces(problem, pcb_render_model.clone());
            display_and_block(&proba_model);

            for i in 0..10{
                println!("Updating posterior for the {}th time", i + 1);
                proba_model.update_posterior();
                display_and_block(&proba_model);
            }
        }
        proba_model
    }

    fn sample_new_traces(&mut self, problem: &PcbProblem, pcb_render_model: Arc<Mutex<PcbRenderModel>>) {
        let mut new_proba_traces: Vec<Rc<ProbaTrace>> = Vec::new();

        // connection_id to connection
        let mut connections: HashMap<ConnectionID, Rc<Connection>> = HashMap::new();
        // connection_id to net_id
        let mut connection_to_net: HashMap<ConnectionID, NetID> = HashMap::new();
        for (net_id, net_info) in problem.nets.iter() {
            for (connection_id, connection) in net_info.connections.iter() {
                connections.insert(*connection_id, connection.clone());
                connection_to_net.insert(*connection_id, *net_id);
            }
        }
        

        // proba_trace_id to proba_trace
        let mut proba_traces: HashMap<ProbaTraceID, Rc<ProbaTrace>> = HashMap::new();
        // visited TraceAnchors
        let mut visited_traces: BTreeSet<TraceAnchors> = BTreeSet::new();
        for traces in self.connection_to_traces.values() {
            if let Traces::Probabilistic(trace_map) = traces {
                for (proba_trace_id, proba_trace) in trace_map.iter() {
                    proba_traces.insert(*proba_trace_id, proba_trace.clone());
                    visited_traces.insert(proba_trace.trace_path.anchors.clone());
                }
            }
        }

        // net_id to proba_trace_ids
        let mut net_to_proba_traces: HashMap<NetID, Vec<ProbaTraceID>> = problem
            .nets
            .keys()
            .map(|net_id| (*net_id, Vec::new()))
            .collect();
        for (connection_id, traces) in self.connection_to_traces.iter() {
            if let Traces::Probabilistic(trace_ids) = traces {
                let net_id = connection_to_net.get(connection_id).expect(
                    format!(
                        "ConnectionID {:?} not found in connection_id_to_net_id",
                        connection_id
                    )
                    .as_str(),
                );
                net_to_proba_traces
                    .get_mut(net_id)
                    .expect(format!("NetID {:?} not found in net_to_proba_traces", net_id).as_str())
                    .extend(trace_ids.keys().cloned());
            }
        }
        // temporary normalized posterior for each proba_trace
        let mut temp_normalized_posteriors: HashMap<ProbaTraceID, f64> = HashMap::new();
        for (_, traces) in self.connection_to_traces.iter() {
            if let Traces::Probabilistic(trace_ids) = traces {
                let mut sum_posterior: f64 = 0.0;
                for (_, proba_trace) in trace_ids.iter() {
                    let posterior = proba_trace.get_posterior_with_fallback();
                    sum_posterior += posterior;
                }
                sum_posterior += NEXT_ITERATION_TO_REMAINING_PROBABILITY.get(&self.next_iteration)
                    .expect(format!("No remaining probability for iteration {:?}", self.next_iteration).as_str());
                // normalize the posterior for each trace
                // divide each posterior by the sum of all posteriors
                for (proba_trace_id, proba_trace) in trace_ids.iter() {
                    let posterior = proba_trace.get_posterior_with_fallback();
                    let normalized_posterior = posterior / sum_posterior;
                    temp_normalized_posteriors.insert(*proba_trace_id, normalized_posterior);
                }
            }   
        }

        // the outer loop for generating the dijkstra model
        for net_id in problem.nets.keys() {
            // collect connections that are not in this net
            let obstacle_connections: HashSet<ConnectionID> = problem.nets
                .iter()
                .filter(|(other_net_id, _)| **other_net_id != *net_id)
                .flat_map(|(_, net_info)| net_info.connections.keys())
                .cloned()
                .collect();
            // initialize the number of generated traces for each connection
            let mut num_generated_traces: HashMap<ConnectionID, usize> = self
                .connection_to_traces
                .keys()
                .map(|connection_id| (*connection_id, 0))
                .collect();
            // initialize the number of generation attempts
            let mut num_generation_attempts: usize = 0;
            // the inner loop for generating traces for each connection in the net
            let max_num_traces = *ITERATION_TO_NUM_TRACES.get(&self.next_iteration).expect(
                format!(
                    "No number of traces for iteration {:?}",
                    self.next_iteration
                )
                .as_str(),
            );
            while num_generation_attempts < MAX_GENERATION_ATTEMPTS
                && num_generated_traces
                    .values()
                    .any(|&count| count < max_num_traces)
            {
                println!("Generation attempt: {}", num_generation_attempts + 1);
                num_generation_attempts += 1;
                let mut sampled_obstacle_traces: HashMap<ConnectionID, Option<ProbaTraceID>> = HashMap::new();
                // randomly generate a trace for each pad pair of other nets (in a rare case the trace will not be generated)
                for obstacle_connection_id in obstacle_connections.iter() {
                    // sample a trace from this connection
                    let traces = self.connection_to_traces.get(obstacle_connection_id).expect(
                        format!(
                            "ConnectionID {:?} not found in connection_to_traces",
                            obstacle_connection_id
                        )
                        .as_str(),
                    );
                    let trace_ids = if let Traces::Probabilistic(trace_ids) = traces {
                        trace_ids.keys().cloned().collect::<Vec<ProbaTraceID>>()
                    } else {
                        continue; // Skip fixed traces
                    };
                    let mut sum_normalized_posterior: f64 = 0.0;
                    let mut normalized_posteriors: Vec<f64> = Vec::new();
                    for trace_id in trace_ids.iter() {
                        let normalized_posterior = *temp_normalized_posteriors
                            .get(trace_id)
                            .expect(format!("No normalized posterior for trace ID {:?}", trace_id).as_str());
                        sum_normalized_posterior += normalized_posterior;
                        normalized_posteriors.push(normalized_posterior);
                    }
                    assert!(sum_normalized_posterior < 1.0, "Sum of normalized posteriors must be less than 1.0, got: {}", sum_normalized_posterior);
                    let num_trace_candidates = normalized_posteriors.len();
                    let remaining_probability = 1.0 - sum_normalized_posterior;
                    normalized_posteriors.push(remaining_probability);
                    let dist = WeightedIndex::new(normalized_posteriors).unwrap();
                    let mut rng = rand::rng();
                    let index = dist.sample(&mut rng);
                    let chosen_proba_trace_id: Option<ProbaTraceID> = if index < num_trace_candidates {
                        Some(trace_ids[index])
                    } else {
                        None
                    };
                    sampled_obstacle_traces.insert(
                        *obstacle_connection_id,
                        chosen_proba_trace_id,
                    );
                }
                let mut obstacle_shapes: Vec<PrimShape> = Vec::new();
                let mut obstacle_clearance_shapes: Vec<PrimShape> = Vec::new();
                // add all sampled traces to the obstacle shapes
                for (_, proba_trace_id) in sampled_obstacle_traces.iter() {
                    let proba_trace_id = if let Some(proba_trace_id) = proba_trace_id {
                        *proba_trace_id
                    } else {
                        continue; // Skip if no trace was sampled
                    };
                    let proba_trace = proba_traces.get(&proba_trace_id).expect(
                        format!("ProbaTraceID {:?} not found in proba_traces", proba_trace_id).as_str(),
                    );
                    let trace_segments = &proba_trace.trace_path.segments;
                    for segment in trace_segments.iter() {
                        let shapes = segment.to_shapes();
                        obstacle_shapes.extend(shapes);
                        // add clearance shapes
                        let clearance_shapes = segment.to_clearance_shapes();
                        obstacle_clearance_shapes.extend(clearance_shapes);
                    }
                }
                // add all pads in other nets to the obstacle shapes
                for obstacle_connection_id in obstacle_connections.iter() {
                    let connection = connections.get(obstacle_connection_id).expect(
                        format!(
                            "ConnectionID {:?} not found in connections",
                            obstacle_connection_id
                        )
                        .as_str(),
                    );
                    let source_pad_shapes = connection.source.to_shapes();
                    let sink_pad_shapes = connection.sink.to_shapes();
                    obstacle_shapes.extend(source_pad_shapes);
                    obstacle_shapes.extend(sink_pad_shapes);
                }
                let mut astar_model = AStarModel{
                    width: problem.width,
                    height: problem.height,
                    obstacle_shapes,
                    obstacle_clearance_shapes,
                    start: FixedVec2{x: Default::default(), y: Default::default()},
                    end: FixedVec2{x: Default::default(), y: Default::default()},
                    trace_width: 0.0, // This will be set later
                    trace_clearance: 0.0, // This will be set later
                    border_cache: RefCell::new(None), // Cache for border points, initialized to None
                };
                let connections = &problem.nets
                    .get(net_id)
                    .expect(format!("NetID {:?} not found in nets", net_id).as_str())
                    .connections;
                // only consider connections with probabilistic traces
                let connections: Vec<(ConnectionID, Rc<Connection>)> = connections
                    .iter()
                    .filter(|(connection_id, _)| {
                        let traces = self.connection_to_traces.get(connection_id).expect(
                            format!("ConnectionID {:?} not found in connection_to_traces", connection_id).as_str(),
                        );
                        if let Traces::Probabilistic(_) = traces {
                            true // Only consider connections with probabilistic traces
                        } else {
                            false // Skip fixed traces
                        }
                    })
                    .map(|(connection_id, connection)|{
                        (*connection_id, connection.clone())
                    })
                    .collect();

                for (connection_id, connection) in connections.iter() {
                    let connection_num_generated_traces = num_generated_traces
                        .get(connection_id)
                        .expect(format!("ConnectionID {:?} not found in num_generated_traces", connection_id).as_str());
                    if *connection_num_generated_traces >= max_num_traces {
                        println!("ConnectionID {:?} already has enough traces, skipping", connection_id);
                        continue; // Skip this connection if it already has enough traces
                    }
                    // sample a trace for this connection
                    astar_model.start = connection.source.position.to_fixed();
                    astar_model.end = connection.sink.position.to_fixed();
                    astar_model.trace_width = connection.trace_width;
                    astar_model.trace_clearance = connection.trace_clearance;

                    // run A* algorithm to find a path
                    let astar_result = astar_model.run(pcb_render_model.clone());
                    let astar_result = match astar_result {
                        Ok(result) => result,
                        Err(err) => {
                            println!("A* algorithm failed: {}", err);
                            continue; // Skip this connection if A* fails
                        }
                    };
                    let trace_path = astar_result.trace_path;
                    if visited_traces.contains(&trace_path.anchors) {
                        println!("Trace path {:?} already visited, skipping", trace_path.anchors);
                        continue;
                    }
                    visited_traces.insert(trace_path.anchors.clone());
                    let proba_trace_id = self
                        .trace_id_generator
                        .next()
                        .expect("TraceID generator exhausted");
                    let proba_trace = ProbaTrace {
                        net_id: *net_id,
                        connection_id: *connection_id,
                        proba_trace_id,
                        trace_path,
                        iteration: self.next_iteration,
                        posterior: RefCell::new(None), // Initialize with None, will be updated later
                        temp_posterior: RefCell::new(None), // Temporary posterior for simultaneous updates
                    };
                    new_proba_traces.push(Rc::new(proba_trace));
                    let num = num_generated_traces
                        .get_mut(connection_id)
                        .expect(format!("ConnectionID {:?} not found in num_generated_traces", connection_id).as_str());
                    *num += 1;
                }
            }
        }
        // add the new traces to the model
        for proba_trace in new_proba_traces {
            let proba_trace_id = proba_trace.proba_trace_id;
            let connection_id = proba_trace.connection_id;
            let traces = self.connection_to_traces.get_mut(&connection_id).expect(
                format!("ConnectionID {:?} not found in connection_to_traces", connection_id).as_str(),
            );
            // traces can only be probabilistic, if it is fixed, we panic
            let traces = if let Traces::Probabilistic(trace_map) = traces {
                trace_map
            } else {
                panic!(
                    "ConnectionID {:?} has fixed traces, cannot add probabilistic trace",
                    connection_id
                );
            };
            let old = traces.insert(proba_trace_id, proba_trace.clone());
            assert!(
                old.is_none(),
                "ProbaTraceID {:?} already exists for ConnectionID {:?}",
                proba_trace_id,
                connection_id
            );            
        }

        // update proba_traces to include the new traces
        let mut proba_traces: HashMap<ProbaTraceID, Rc<ProbaTrace>> = HashMap::new();
        for traces in self.connection_to_traces.values() {
            if let Traces::Probabilistic(trace_map) = traces {
                for (proba_trace_id, proba_trace) in trace_map.iter() {
                    proba_traces.insert(*proba_trace_id, proba_trace.clone());
                }
            }
        }
        // update the collision adjacency
        let mut collision_adjacency: HashMap<ProbaTraceID, HashSet<ProbaTraceID>> = proba_traces.iter()
            .map(|(proba_trace_id, _)| (*proba_trace_id, HashSet::new()))
            .collect();
        // update net_to_proba_traces to include the new traces
        let mut net_to_proba_traces: HashMap<NetID, Vec<ProbaTraceID>> = problem
            .nets
            .keys()
            .map(|net_id| (*net_id, Vec::new()))
            .collect();
        for (connection_id, traces) in self.connection_to_traces.iter() {
            if let Traces::Probabilistic(trace_ids) = traces {
                let net_id = connection_to_net.get(connection_id).expect(
                    format!(
                        "ConnectionID {:?} not found in connection_id_to_net_id",
                        connection_id
                    )
                    .as_str(),
                );
                net_to_proba_traces
                    .get_mut(net_id)
                    .expect(format!("NetID {:?} not found in net_to_proba_traces", net_id).as_str())
                    .extend(trace_ids.keys().cloned());
            }
        }
        // calculate the collisions between traces
        let proba_traces_vec: Vec<Vec<ProbaTraceID>> = net_to_proba_traces.into_values().collect();
        for i in 0..proba_traces_vec.len() {
            for j in (i + 1)..proba_traces_vec.len() {
                let net_i = &proba_traces_vec[i];
                let net_j = &proba_traces_vec[j];
                for trace_i in net_i.iter() {
                    for trace_j in net_j.iter() {
                        // check if the traces collide
                        let proba_trace_i = proba_traces.get(trace_i).expect(
                            format!("ProbaTraceID {:?} not found in proba_traces", trace_i).as_str(),
                        );
                        let proba_trace_j = proba_traces.get(trace_j).expect(
                            format!("ProbaTraceID {:?} not found in proba_traces", trace_j).as_str(),
                        );
                        if proba_trace_i.trace_path.collides_with(&proba_trace_j.trace_path) {
                            // add the collision to the adjacency
                            collision_adjacency
                                .get_mut(trace_i)
                                .expect(format!("ProbaTraceID {:?} not found in collision_adjacency", trace_i).as_str())
                                .insert(*trace_j);
                            collision_adjacency
                                .get_mut(trace_j)
                                .expect(format!("ProbaTraceID {:?} not found in collision_adjacency", trace_j).as_str())
                                .insert(*trace_i);
                        }
                    }
                }
            }
        }
        self.collision_adjacency = collision_adjacency;
        // update next_iteration
        self.next_iteration = NonZeroUsize::new(self.next_iteration.get() + 1).unwrap();
    }
    pub fn to_pcb_render_model(&self, problem: &PcbProblem) -> PcbRenderModel {
        let mut trace_shape_renderables: Vec<RenderableBatch> = Vec::new();
        let mut pad_shape_renderables: Vec<ShapeRenderable> = Vec::new();
        for (_, net_info) in problem.nets.iter() {
            for (_, connection) in net_info.connections.iter() {
                // Add fixed traces
                if let Some(Traces::Fixed(fixed_trace)) = self.connection_to_traces.get(&connection.connection_id) {
                    let color = net_info.color.to_float4(1.0);
                    let renderable_batch = fixed_trace.trace_path.to_renderables(color);
                    trace_shape_renderables.push(renderable_batch);
                }
                // Add probabilistic traces
                if let Some(Traces::Probabilistic(trace_map)) = self.connection_to_traces.get(&connection.connection_id) {
                    for proba_trace in trace_map.values() {
                        let posterior = proba_trace.get_posterior_with_fallback();
                        let posterior = posterior.clamp(0.0, 1.0); // Ensure posterior is between 0 and 1
                        let color = net_info.color.to_float4(posterior as f32);
                        let renderable_batch = proba_trace.trace_path.to_renderables(color);
                        trace_shape_renderables.push(renderable_batch);
                    }
                }
                // Add pads
                let source_renderables = connection.source.to_renderables(net_info.color.to_float4(1.0));
                let source_clearance_renderables = connection.source.to_clearance_renderables(net_info.color.to_float4(1.0));
                let sink_renderables = connection.sink.to_renderables(net_info.color.to_float4(1.0));
                let sink_clearance_renderables = connection.sink.to_clearance_renderables(net_info.color.to_float4(1.0));
                pad_shape_renderables.extend(source_renderables);
                pad_shape_renderables.extend(source_clearance_renderables);
                pad_shape_renderables.extend(sink_renderables);
                pad_shape_renderables.extend(sink_clearance_renderables);
            }
        }
        PcbRenderModel {
            width: problem.width,
            height: problem.height,
            trace_shape_renderables,
            pad_shape_renderables,
        }
    }

    pub fn update_posterior(&mut self){
        let proba_traces: HashMap<ProbaTraceID, Rc<ProbaTrace>> = self.connection_to_traces
            .values()
            .filter_map(|traces| {
                if let Traces::Probabilistic(trace_map) = traces {
                    Some(trace_map.clone())
                } else {
                    None // Skip fixed traces
                }
            })
            .flatten()
            .collect();
        // Update the posterior probabilities for all traces in the model
        for (proba_trace_id, proba_trace) in proba_traces.iter() {
            let adjacent_traces = self.collision_adjacency
                .get(proba_trace_id)
                .expect(format!("No adjacent traces for ProbaTraceID {:?}", proba_trace_id).as_str());
            let mut proba_product = 1.0;
            for adjacent_trace_id in adjacent_traces.iter() {
                let adjacent_trace = proba_traces.get(adjacent_trace_id).expect(
                    format!("No ProbaTraceID {:?} found in proba_traces", adjacent_trace_id).as_str(),
                );
                let posterior = adjacent_trace.get_posterior_with_fallback();
                // to do: update this
                let one_minus_posterior = (1.0 - posterior).max(0.0); // Ensure non-negative
                proba_product *= one_minus_posterior;
            }
            let target_posterior = proba_product;
            assert!(
                target_posterior >= 0.0 && target_posterior <= 1.0,
                "Target posterior must be between 0 and 1"
            );
            // get num traces in the same iteration
            let current_posterior = proba_trace.get_posterior_with_fallback();
            let opportunity_cost = target_posterior / current_posterior;
            let score = proba_trace.trace_path.get_score();
            let score_weight = *SCORE_WEIGHT.lock().unwrap();
            let opportunity_cost_weight = *OPPORTUNITY_COST_WEIGHT.lock().unwrap();
            let target_posterior_unnormalized = 1.0
                * f64::powf(score, score_weight)
                * f64::powf(opportunity_cost, opportunity_cost_weight);
            let target_posterior_normalized = proba_trace
                .get_normalized_prior()
                * target_posterior_unnormalized;            
            let mut temp_posterior = proba_trace.temp_posterior.borrow_mut();
            let target_greater_than_current = target_posterior_normalized > current_posterior;
            let constant_offset = if target_greater_than_current {
                CONSTANT_LEARNING_RATE
            } else {
                -CONSTANT_LEARNING_RATE
            };
            let new_posterior = current_posterior
                + (target_posterior_normalized - current_posterior) * LINEAR_LEARNING_RATE
                + constant_offset;
            let new_posterior = if target_greater_than_current{
                new_posterior.max(target_posterior_normalized)
            } else {
                new_posterior.min(target_posterior_normalized)
            };
            *temp_posterior = Some(new_posterior);
        }
        // move temp_posterior to posterior
        for(_, proba_trace) in proba_traces.iter() {
            let mut posterior = proba_trace.posterior.borrow_mut();
            let mut temp_posterior = proba_trace.temp_posterior.borrow_mut();
            let temp_posterior_val = temp_posterior.unwrap();
            *posterior = Some(temp_posterior_val);
            // reset temp_posterior
            *temp_posterior = None;
        }
    }
}

#[derive(Debug, Clone)]
pub struct Node {
    pub remaining_trace_candidates: BinaryHeap<BinaryHeapItem<NotNan<f64>, Rc<ProbaTrace>>>, // The remaining trace candidates to be processed, sorted by their scores)>
    pub fixed_traces: HashMap<ConnectionID, FixedTrace>,
    pub prob_up_to_date: bool, // Whether the probabilistic model is up to date
}



impl Node{
    fn from_proba_model(
        proba_model: &ProbaModel,
    )->Self{
        let mut fixed_traces: HashMap<ConnectionID, FixedTrace> = HashMap::new();
        let mut remaining_trace_candidates: BinaryHeap<BinaryHeapItem<NotNan<f64>, Rc<ProbaTrace>>> = BinaryHeap::new();
        for (connection_id, traces) in proba_model.connection_to_traces.iter() {
            match traces {
                Traces::Fixed(fixed_trace) => {
                    fixed_traces.insert(*connection_id, fixed_trace.clone());
                }
                Traces::Probabilistic(trace_map) => {
                    for (_proba_trace_id, proba_trace) in trace_map.iter() {
                        let posterior = proba_trace.get_posterior_with_fallback();
                        let not_nan_proba = NotNan::new(posterior).expect("Probability must be non-NaN");
                        remaining_trace_candidates.push(BinaryHeapItem{
                            key: not_nan_proba,
                            value: proba_trace.clone(),
                        });
                    }
                }
            }
        }
        Node {
            remaining_trace_candidates,
            fixed_traces,
            prob_up_to_date: true, // Initially, the probabilistic model is up to date
        }
    }
    fn fix_trace(&mut self, connection_id: ConnectionID, fixed_trace: FixedTrace) {
        // Add the fixed trace to the fixed traces
        self.fixed_traces.insert(connection_id, fixed_trace);
        // Remove all trace candidates for this connection from the remaining candidates
        let mut remaining_trace_candidates_copy = self.remaining_trace_candidates.clone();
        let mut new_remaining_trace_candidates: BinaryHeap<BinaryHeapItem<NotNan<f64>, Rc<ProbaTrace>>> = BinaryHeap::new();
        for candidate in remaining_trace_candidates_copy.drain() {
            if candidate.value.connection_id != connection_id {
                new_remaining_trace_candidates.push(candidate);
            }
        }
        self.remaining_trace_candidates = new_remaining_trace_candidates;
        // Mark the probabilistic model as no longer up to date
        self.prob_up_to_date = false;
    }
    /// If an attemp fails, return none; it will pop the priority queue in both scenarios
    /// assume there are still candidates in the priority queue
    pub fn try_fix_top_ranked_trace(&mut self)->Option<Self>{
        // for self, peek from the priority queue
        // if succeed, remove all traces from the same connection, and generate a new node with the same priority queue and a fixed trace
        // if fail, return error
        let top_ranked_candidate = self.remaining_trace_candidates.pop()
            .expect("No remaining trace candidates to fix");
        let top_ranked_trace_path = &top_ranked_candidate.value.trace_path;
        // check if the trace collides with any fixed trace
        for fixed_trace in self.fixed_traces.values() {
            if top_ranked_trace_path.collides_with(&fixed_trace.trace_path) {
                // If it collides, we cannot fix this trace
                println!("Top ranked trace collides with a fixed trace, cannot fix it");
                return None; // Return None to indicate failure
            }
        }
        // If it does not collide, we can fix this trace
        let connection_id = top_ranked_candidate.value.connection_id;
        // Create a new fixed trace
        let fixed_trace = FixedTrace {
            net_id: top_ranked_candidate.value.net_id,
            connection_id,
            trace_path: top_ranked_trace_path.clone(),
        };
        // delete all trace candidates for this connection in the new node
        let mut new_node = self.clone();
        new_node.fix_trace(connection_id, fixed_trace);
        Some(new_node) // Return the new node with the fixed trace
    }
    pub fn from_fixed_traces(
        problem: &PcbProblem,
        fixed_traces: &HashMap<ConnectionID, FixedTrace>,
        pcb_render_model: Arc<Mutex<PcbRenderModel>>,
    )->Self{
        let proba_model = ProbaModel::create_and_solve(problem, fixed_traces, pcb_render_model);
        Node::from_proba_model(&proba_model)
    }
    /// if self is already up to date, return none
    pub fn try_update_proba_model(&self, problem: &PcbProblem, pcb_render_model: Arc<Mutex<PcbRenderModel>>) -> Option<Self>{
        if self.prob_up_to_date {
            return None; // If the probabilistic model is already up to date, do nothing
        }
        let fixed_traces = &self.fixed_traces;
        let new_node = Node::from_fixed_traces(problem, fixed_traces, pcb_render_model);
        Some(new_node) // Return the new node with the updated probabilistic model
    }
    pub fn is_solution(&self, problem: &PcbProblem) -> bool {
        // Check if all connections in the problem have fixed traces in this node
        for net_info in problem.nets.values() {
            for connection_id in net_info.connections.keys() {
                if !self.fixed_traces.contains_key(connection_id) {
                    return false; // If any connection does not have a fixed trace, it's not a solution
                }
            }
        }
        true // All connections have fixed traces, so this is a solution
    }
    pub fn try_fix_any_trace(&mut self) -> Option<Self> {
        // Try to fix any trace from the remaining candidates
        while self.remaining_trace_candidates.len() > 0 {
            let top_ranked_candidate = self.remaining_trace_candidates.pop()
                .expect("No remaining trace candidates to fix");
            let top_ranked_trace_path = &top_ranked_candidate.value.trace_path;
            // Check if the trace collides with any fixed trace
            let mut collision_found = false;
            for fixed_trace in self.fixed_traces.values() {
                if top_ranked_trace_path.collides_with(&fixed_trace.trace_path) {
                    // If it collides, we cannot fix this trace
                    println!("Top ranked trace collides with a fixed trace, cannot fix it");
                    collision_found = true;
                    break; // No need to check further, we found a collision
                }
            }
            if !collision_found{
                // If it does not collide, we can fix this trace
                let connection_id = top_ranked_candidate.value.connection_id;
                // Create a new fixed trace
                let fixed_trace = FixedTrace {
                    net_id: top_ranked_candidate.value.net_id,
                    connection_id,
                    trace_path: top_ranked_trace_path.clone(),
                };
                let mut new_node = self.clone();
                new_node.fix_trace(connection_id, fixed_trace);
                return Some(new_node); // Return the new node with the fixed trace
            }
        }
        None
    }
}

pub struct PcbSolution {
    pub determined_traces: HashMap<ConnectionID, FixedTrace>, // NetID to ConnectionID to FixedTrace
}

impl PcbProblem {
    pub fn new(width: f32, height: f32) -> Self {
        let net_id_generator = Box::new((0..).map(NetID));
        let connection_id_generator = Box::new((0..).map(ConnectionID));
        PcbProblem {
            width,
            height,
            nets: HashMap::new(),
            net_id_generator,
            connection_id_generator,
        }
    }
    pub fn add_net(&mut self, color: Color) -> NetID {
        let duplicate_color = self.nets.values().any(|net_info| net_info.color == color);
        assert!(
            !duplicate_color,
            "Net with color {:?} already exists",
            color
        );
        let net_id = self
            .net_id_generator
            .next()
            .expect("NetID generator exhausted");
        let net_info = NetInfo {
            net_id,
            color,
            connections: HashMap::new(),
        };
        self.nets.insert(net_id, net_info);
        net_id
    }
    /// assert the sources in the same net are the same
    pub fn add_connection(&mut self, net_id: NetID, source: Pad, sink: Pad, trace_width: f32, trace_clearance: f32) -> ConnectionID {
        let net_info = self.nets.get_mut(&net_id).expect("NetID not found");
        let connection_id = self
            .connection_id_generator
            .next()
            .expect("ConnectionID generator exhausted");
        let connection = Connection {
            net_id,
            connection_id,
            source,
            sink,
            trace_width,
            trace_clearance,
        };
        net_info.connections.insert(connection_id, Rc::new(connection));
        connection_id
    }

    pub fn solve(&self, pcb_render_model: Arc<Mutex<PcbRenderModel>>) -> Result<PcbSolution, String> {
        let mut node_stack: Vec<Node> = Vec::new();

        fn last_updated_node_index(node_stack: &Vec<Node>) -> usize {
            for (index, node) in node_stack.iter().enumerate().rev() {
                if node.prob_up_to_date {
                    return index; // Return the index of the last updated node
                }
            }
            // because the first node is always up to date, it is impossible to reach here
            panic!("No updated node found in the stack");
        }

        fn print_current_stack(node_stack: &Vec<Node>) {
            println!("Current stack:");
            for (index, node) in node_stack.iter().enumerate() {
                println!("\tNode {}: up_to_date: {}, num fixed traces: {}, num remaining trace candidates: {}, ", 
                    index,
                    node.prob_up_to_date,
                    node.fixed_traces.len(),
                    node.remaining_trace_candidates.len()
                );
            }
        }
        
        let first_node = Node::from_fixed_traces(self, &HashMap::new(), pcb_render_model.clone());
        // assume the first node has trace candidates
        node_stack.push(first_node);

        while node_stack.len() > 0 {
            print_current_stack(&node_stack);
            let top_node = node_stack.last_mut().unwrap();
            if top_node.is_solution(self){
                println!("Found a solution!");
                // If the top node is a solution, we can return it
                let fixed_traces = top_node.fixed_traces.clone();
                let solution = PcbSolution {
                    determined_traces: fixed_traces,
                };
                return Ok(solution);
            }
            let new_node = top_node.try_fix_top_ranked_trace();
            match new_node{
                Some(new_node) => {
                    // If we successfully fixed a trace, push the new node onto the stack
                    println!("Successfully fixed the top ranked trace, pushing new node onto the stack");
                    assert!(new_node.prob_up_to_date, "New node must be up to date");
                    node_stack.push(new_node);
                }
                None => {
                    // If we failed to fix the top-ranked trace, we update the node in the middle between the current position and the last updated node
                    println!("Failed to fix the top ranked trace, trying to update the probabilistic model in the middle of the stack");
                    let current_node_index = node_stack.len() - 1;
                    let last_updated_index = last_updated_node_index(&node_stack);
                    let target_index = (current_node_index + last_updated_index + 1) / 2; // bias to right for consistency
                    let new_node = node_stack[target_index].try_update_proba_model(self, pcb_render_model.clone());
                    match new_node {
                        Some(new_node) => {
                            // If we successfully updated the probabilistic model, replace the node at the target index with the new node
                            assert!(target_index < node_stack.len() - 1, "target index cannot be the last node in the stack");
                            node_stack[target_index + 1] = new_node;
                            node_stack.truncate(target_index + 2); // Remove all nodes above the target index
                            println!("Successfully updated the probabilistic model, replacing node at index {}", target_index);
                        },
                        None => {
                            // If we failed to update the probabilistic model, we pop the current node from the stack
                            assert!(target_index == node_stack.len() - 1, "target index must be the last node in the stack");
                            node_stack.pop();
                            println!("Failed to update the probabilistic model, popping the current node from the stack");
                        }
                    }
                }
            }
        }
        Err("No solution found".to_string())
    }
}
