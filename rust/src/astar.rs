//! A* informed search algorithm for heuristic-guided vehicle route optimization.
//!
//! Implements A* graph search with an admissible, mode-aware Euclidean distance
//! heuristic. The priority queue is ordered by f-score (g + h), guiding expansion
//! toward geometrically promising vertex neighborhoods.
//!
//! **Admissibility**: The heuristic returns a lower bound derived from the fastest
//! physically possible traversal speed (140 km/h). Since no real road segment can
//! be traversed faster under the physical model, h(u) ≤ h*(u) holds for all vertices.
//!
//! **Complexity**: O((V + E) log V) time. In practice, a well-placed heuristic
//! reduces the number of settled vertices substantially compared to Dijkstra on
//! geographically distributed queries.

use crate::graph::Graph;
use crate::models::{Config, Node, RouteResult};
use std::cmp::Ordering;
use std::collections::{BinaryHeap, HashMap};
use std::time::Instant;

/// Priority queue entry carrying f-score, g-score, and vertex identifier.
///
/// `Ord` is implemented with reversed f-score comparison so that `BinaryHeap`
/// acts as a min-heap ordered by f-score ascending.
#[derive(Copy, Clone, PartialEq)]
struct AStarState {
    /// f-score = g-score + heuristic estimate to target.
    f_score: f64,
    /// g-score = actual accumulated cost from source to this vertex.
    g_score: f64,
    /// Graph vertex identifier.
    node: usize,
}

impl Eq for AStarState {}

impl Ord for AStarState {
    fn cmp(&self, other: &Self) -> Ordering {
        other
            .f_score
            .partial_cmp(&self.f_score)
            .unwrap_or(Ordering::Equal)
    }
}

impl PartialOrd for AStarState {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

/// Computes an admissible lower-bound heuristic from `u_node` to `target_node`.
///
/// The Euclidean straight-line distance (km) between the two spatial coordinates
/// is converted to the appropriate objective cost unit based on the mode:
///
/// - `"FASTEST"` — minimum traversal time at 140 km/h (the physical speed ceiling).
/// - `"BALANCED"` — time component scaled by its objective weight coefficient.
/// - `"CHEAPEST"` — returns 0.0 (an admissible zero lower bound; Euclidean fuel
///   cost is not constructible without per-road-type topology knowledge).
///
/// # Arguments
/// * `u_node`      — Current vertex spatial position.
/// * `target_node` — Destination vertex spatial position.
/// * `mode`        — Optimization mode determining heuristic output unit.
/// * `config`      — Simulation configuration providing weight coefficients.
///
/// # Returns
/// Non-negative admissible lower bound on cost from `u_node` to `target_node`.
pub fn astar_heuristic(u_node: &Node, target_node: &Node, mode: &str, config: &Config) -> f64 {
    let dx = target_node.x - u_node.x;
    let dy = target_node.y - u_node.y;
    let dist_km = (dx * dx + dy * dy).sqrt();
    // Physical ceiling: no edge traversal can be faster than 140 km/h
    let min_time_min = (dist_km / 140.0) * 60.0;

    match mode {
        "FASTEST"  => min_time_min,
        "BALANCED" => config.time_weight * min_time_min,
        _          => 0.0, // CHEAPEST — admissible zero lower bound
    }
}

/// Computes the cost-optimal route between two vertices using A* graph search.
///
/// Maintains separate g-score (actual cost from source) and f-score (g + heuristic)
/// tracking structures. The `BinaryHeap` is ordered by f-score, directing expansion
/// toward geometrically closer vertices and reducing the settled vertex count
/// compared to uninformed Dijkstra, particularly on sparse long-range queries.
///
/// Stale heap entries (superseded by shorter g-scores) are discarded via lazy deletion.
/// Path reconstruction backtracks the predecessor edge map after the target is settled.
///
/// # Arguments
/// * `graph`  — Pre-built `Graph` with physics-evaluated edge costs.
/// * `source` — Origin vertex identifier.
/// * `target` — Destination vertex identifier.
/// * `mode`   — Optimization objective: `"FASTEST"`, `"CHEAPEST"`, or `"BALANCED"`.
///
/// # Returns
/// `Some(RouteResult)` containing the optimal path and aggregate physical metrics,
/// or `None` if no path from `source` to `target` exists in the graph.
pub fn solve_astar(graph: &Graph, source: usize, target: usize, mode: &str) -> Option<RouteResult> {
    let start_time = Instant::now();

    if !graph.nodes.contains_key(&source) || !graph.nodes.contains_key(&target) {
        return None;
    }

    let target_node = &graph.nodes[&target];

    if source == target {
        return Some(RouteResult {
            source,
            destination: target,
            optimization: mode.to_string(),
            algorithm: "ASTAR".to_string(),
            route: vec![source],
            distance_km: 0.0,
            travel_time_min: 0.0,
            fuel_liters: 0.0,
            fuel_cost: 0.0,
            toll_cost: 0.0,
            total_cost: 0.0,
            nodes_explored: 1,
            execution_time_us: start_time.elapsed().as_micros() as u64,
        });
    }

    // g_score[v] = best confirmed cost from source to vertex v
    let mut g_score: HashMap<usize, f64> = HashMap::new();
    // prev[v] = (predecessor_vertex, edge_index) for path reconstruction
    let mut prev: HashMap<usize, (usize, usize)> = HashMap::new();
    let mut heap: BinaryHeap<AStarState> = BinaryHeap::new();

    let h0 = astar_heuristic(&graph.nodes[&source], target_node, mode, &graph.config);
    g_score.insert(source, 0.0);
    heap.push(AStarState {
        f_score: h0,
        g_score: 0.0,
        node: source,
    });

    let mut nodes_explored = 0;
    let mut found = false;

    while let Some(AStarState { g_score: g, node, .. }) = heap.pop() {
        // Lazy deletion: discard entries superseded by a shorter g-score
        if let Some(&curr_g) = g_score.get(&node) {
            if g > curr_g {
                continue;
            }
        }

        nodes_explored += 1;

        if node == target {
            found = true;
            break;
        }

        if let Some(neighbors) = graph.adj.get(&node) {
            for &(next_node, edge_idx) in neighbors {
                let edge = &graph.edges[edge_idx];
                let weight = graph.get_edge_weight(edge, mode);
                let tentative_g = g + weight;

                let is_shorter = match g_score.get(&next_node) {
                    Some(&curr_dist) => tentative_g < curr_dist,
                    None => true,
                };

                if is_shorter {
                    g_score.insert(next_node, tentative_g);
                    prev.insert(next_node, (node, edge_idx));
                    let h = astar_heuristic(&graph.nodes[&next_node], target_node, mode, &graph.config);
                    heap.push(AStarState {
                        f_score: tentative_g + h,
                        g_score: tentative_g,
                        node: next_node,
                    });
                }
            }
        }
    }

    let exec_us = start_time.elapsed().as_micros() as u64;

    if !found {
        return None;
    }

    // Backtrack predecessor map to reconstruct the optimal node sequence
    let mut route_nodes = Vec::new();
    let mut route_edges = Vec::new();
    let mut curr = target;
    route_nodes.push(curr);

    while curr != source {
        let (p, edge_idx) = prev[&curr];
        route_edges.push(edge_idx);
        curr = p;
        route_nodes.push(curr);
    }

    route_nodes.reverse();
    route_edges.reverse();

    let mut tot_dist = 0.0_f64;
    let mut tot_time = 0.0_f64;
    let mut tot_fuel = 0.0_f64;
    let mut tot_fuel_cost = 0.0_f64;
    let mut tot_toll = 0.0_f64;

    for &idx in &route_edges {
        let e = &graph.edges[idx];
        tot_dist     += e.distance;
        tot_time     += e.travel_time_min;
        tot_fuel     += e.fuel_liters;
        tot_fuel_cost += e.fuel_cost;
        tot_toll     += e.toll_cost;
    }

    let total_cost = g_score[&target];

    Some(RouteResult {
        source,
        destination: target,
        optimization: mode.to_string(),
        algorithm: "ASTAR".to_string(),
        route: route_nodes,
        distance_km:    (tot_dist      * 10_000.0).round() / 10_000.0,
        travel_time_min:(tot_time      * 10_000.0).round() / 10_000.0,
        fuel_liters:    (tot_fuel      * 10_000.0).round() / 10_000.0,
        fuel_cost:      (tot_fuel_cost * 10_000.0).round() / 10_000.0,
        toll_cost:      (tot_toll      * 10_000.0).round() / 10_000.0,
        total_cost:     (total_cost    * 10_000.0).round() / 10_000.0,
        nodes_explored,
        execution_time_us: exec_us,
    })
}
