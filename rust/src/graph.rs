//! Weighted directed road network graph with pre-computed physical edge costs.
//!
//! Constructs an indexed adjacency structure over a set of spatial vertices and
//! directed road segments. All edge physical metrics (aerodynamic drag, slope force,
//! rolling resistance, fuel consumption, and per-mode costs) are evaluated once at
//! construction time via [`crate::physics::compute_edge_metrics`], making the graph
//! read-only and safe for concurrent multi-threaded query execution via `Rayon`.

use crate::models::{Config, Edge, Node, Vehicle};
use crate::physics::compute_edge_metrics;
use std::collections::HashMap;

/// Weighted directed road network graph with pre-computed multi-objective edge costs.
///
/// Adjacency entries store `(target_node_id, edge_index)` pairs rather than direct
/// edge references, keeping the adjacency list compact and enabling cache-efficient
/// edge lookup during priority queue relaxation steps.
#[derive(Debug, Clone)]
pub struct Graph {
    /// Mapping from node identifier to spatial Node geometry.
    pub nodes: HashMap<usize, Node>,
    /// Ordered vector of all directed road segments with synthesized cost fields.
    pub edges: Vec<Edge>,
    /// Vehicular physical constants used during edge metric computation.
    pub vehicle: Vehicle,
    /// Atmospheric constants and multi-objective weight parameters.
    pub config: Config,
    /// Forward adjacency index: `node_id → [(target_node_id, edge_index_in_edges)]`.
    pub adj: HashMap<usize, Vec<(usize, usize)>>,
}

impl Graph {
    /// Constructs the graph, executes full physics derivation on all edges,
    /// and builds the forward adjacency index.
    ///
    /// # Arguments
    /// * `nodes_list` - Complete list of spatial graph vertices.
    /// * `edges_list` - Complete list of directed road segments (mutated in-place).
    /// * `vehicle`    - Vehicular aerodynamic and thermodynamic constants.
    /// * `config`     - Atmospheric parameters and optimization weight coefficients.
    ///
    /// # Returns
    /// A fully constructed `Graph` ready for concurrent route query execution.
    pub fn new(
        nodes_list: Vec<Node>,
        mut edges_list: Vec<Edge>,
        vehicle: Vehicle,
        config: Config,
    ) -> Self {
        let mut nodes = HashMap::with_capacity(nodes_list.len());
        let mut adj: HashMap<usize, Vec<(usize, usize)>> = HashMap::with_capacity(nodes_list.len());

        for n in nodes_list {
            adj.insert(n.id, Vec::new());
            nodes.insert(n.id, n);
        }

        for (i, edge) in edges_list.iter_mut().enumerate() {
            compute_edge_metrics(edge, &vehicle, &config);
            if let Some(neighbors) = adj.get_mut(&edge.source) {
                neighbors.push((edge.target, i));
            }
        }

        Self {
            nodes,
            edges: edges_list,
            vehicle,
            config,
            adj,
        }
    }

    /// Returns the pre-computed objective cost for the specified optimization mode.
    ///
    /// # Arguments
    /// * `edge` - The road segment whose cost field is to be selected.
    /// * `mode` - Optimization strategy: `"FASTEST"`, `"CHEAPEST"`, or `"BALANCED"`.
    ///
    /// # Returns
    /// Scalar edge traversal cost for use in priority queue relaxation.
    pub fn get_edge_weight(&self, edge: &Edge, mode: &str) -> f64 {
        match mode {
            "FASTEST"  => edge.cost_fastest,
            "CHEAPEST" => edge.cost_cheapest,
            _          => edge.cost_balanced,
        }
    }
}
