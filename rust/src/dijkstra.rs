//! Dijkstra's single-source shortest path algorithm for physical road networks.
//!
//! Implements a binary max-heap variant of Dijkstra's algorithm using Rust's
//! standard `BinaryHeap<State>` with reversed cost ordering to simulate a min-heap.
//! Stale heap entries are discarded with lazy deletion for efficiency on dense graphs.
//!
//! **Complexity**: O((V + E) log V) time, O(V) space.
//!
//! The implementation is read-only with respect to the graph, enabling safe
//! invocation across parallel threads via `Rayon` work-stealing.

use crate::graph::Graph;
use crate::models::RouteResult;
use std::cmp::Ordering;
use std::collections::{BinaryHeap, HashMap};
use std::time::Instant;

/// Priority queue entry representing a graph vertex with its accumulated cost.
///
/// Implements `Ord` with reversed comparison so that `BinaryHeap` acts as a min-heap,
/// always yielding the vertex with the smallest accumulated cost first.
#[derive(Copy, Clone, PartialEq)]
struct State {
    cost: f64,
    node: usize,
}

impl Eq for State {}

impl Ord for State {
    fn cmp(&self, other: &Self) -> Ordering {
        other
            .cost
            .partial_cmp(&self.cost)
            .unwrap_or(Ordering::Equal)
    }
}

impl PartialOrd for State {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

/// Computes the cost-optimal route between two vertices using Dijkstra's algorithm.
///
/// Executes a standard priority-queue-driven relaxation over the directed graph
/// adjacency structure using the pre-computed multi-objective edge costs. Stale
/// (superseded) heap entries are discarded via lazy deletion.
///
/// Path reconstruction proceeds via backtracking through the predecessor map,
/// accumulating physical metrics (distance, time, fuel, costs) along the route.
///
/// # Arguments
/// * `graph`  - Pre-built `Graph` with physics-evaluated edge costs.
/// * `source` - Origin vertex identifier.
/// * `target` - Destination vertex identifier.
/// * `mode`   - Optimization objective: `"FASTEST"`, `"CHEAPEST"`, or `"BALANCED"`.
///
/// # Returns
/// `Some(RouteResult)` with the optimal path and physical metrics,
/// or `None` if no path exists between source and target.
pub fn solve_dijkstra(graph: &Graph, source: usize, target: usize, mode: &str) -> Option<RouteResult> {
    let start_time = Instant::now();

    if !graph.nodes.contains_key(&source) || !graph.nodes.contains_key(&target) {
        return None;
    }

    if source == target {
        return Some(RouteResult {
            source,
            destination: target,
            optimization: mode.to_string(),
            algorithm: "DIJKSTRA".to_string(),
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

    // dist[v] = minimum accumulated cost recorded to vertex v
    let mut dist: HashMap<usize, f64> = HashMap::new();
    // prev[v] = (predecessor_vertex, edge_index) for path reconstruction
    let mut prev: HashMap<usize, (usize, usize)> = HashMap::new();
    let mut heap: BinaryHeap<State> = BinaryHeap::new();

    dist.insert(source, 0.0);
    heap.push(State { cost: 0.0, node: source });

    let mut nodes_explored = 0;
    let mut found = false;

    while let Some(State { cost, node }) = heap.pop() {
        if let Some(&d) = dist.get(&node) {
            if cost > d {
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
                let next_cost = cost + weight;

                let is_shorter = match dist.get(&next_node) {
                    Some(&curr_dist) => next_cost < curr_dist,
                    None => true,
                };

                if is_shorter {
                    dist.insert(next_node, next_cost);
                    prev.insert(next_node, (node, edge_idx));
                    heap.push(State {
                        cost: next_cost,
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

    let mut tot_dist = 0.0;
    let mut tot_time = 0.0;
    let mut tot_fuel = 0.0;
    let mut tot_fuel_cost = 0.0;
    let mut tot_toll = 0.0;

    for &idx in &route_edges {
        let e = &graph.edges[idx];
        tot_dist += e.distance;
        tot_time += e.travel_time_min;
        tot_fuel += e.fuel_liters;
        tot_fuel_cost += e.fuel_cost;
        tot_toll += e.toll_cost;
    }

    let total_cost = dist[&target];

    Some(RouteResult {
        source,
        destination: target,
        optimization: mode.to_string(),
        algorithm: "DIJKSTRA".to_string(),
        route: route_nodes,
        distance_km: (tot_dist * 10000.0).round() / 10000.0,
        travel_time_min: (tot_time * 10000.0).round() / 10000.0,
        fuel_liters: (tot_fuel * 10000.0).round() / 10000.0,
        fuel_cost: (tot_fuel_cost * 10000.0).round() / 10000.0,
        toll_cost: (tot_toll * 10000.0).round() / 10000.0,
        total_cost: (total_cost * 10000.0).round() / 10000.0,
        nodes_explored,
        execution_time_us: exec_us,
    })
}
