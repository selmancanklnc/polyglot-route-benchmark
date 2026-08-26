//! Benchmark execution engine: dataset I/O, query dispatch, and parallel execution.
//!
//! Provides buffered JSON deserialization via `serde_json`, query dispatch to
//! Dijkstra or A* search, and sequential / Rayon work-stealing parallel execution
//! harnesses used by both the CLI and the automated benchmark runner.
//!
//! # Parallelism Model
//!
//! Unlike CPython's GIL, Rust's ownership model allows the `Graph` to be shared
//! across threads without a lock. `Rayon`'s work-stealing scheduler distributes
//! queries across a configurable thread pool, achieving near-linear CPU scaling
//! for compute-bound shortest-path workloads (empirically verified at 4.93× on
//! 8 logical cores in this study).

use crate::astar::solve_astar;
use crate::dijkstra::solve_dijkstra;
use crate::graph::Graph;
use crate::models::{Dataset, Query, RouteResult};
use rayon::prelude::*;
use std::fs::File;
use std::io::BufReader;

/// Deserializes a benchmark JSON dataset and constructs the routing graph.
///
/// Opens the file with a `BufReader` for efficient streaming deserialization via
/// `serde_json`. Constructs a [`Graph`], which triggers full physics derivation on
/// all edges at build time.
///
/// # Arguments
/// * `filepath` — Path to the benchmark JSON dataset file.
///
/// # Returns
/// `Ok((Graph, Vec<Query>))` ready for query execution, or an error if the
/// file cannot be opened or the JSON schema is invalid.
pub fn load_dataset(filepath: &str) -> Result<(Graph, Vec<Query>), Box<dyn std::error::Error>> {
    let file = File::open(filepath)?;
    let reader = BufReader::new(file);
    let dataset: Dataset = serde_json::from_reader(reader)?;

    let graph = Graph::new(
        dataset.nodes,
        dataset.edges,
        dataset.vehicle,
        dataset.configuration,
    );

    Ok((graph, dataset.queries))
}

/// Dispatches a single route query to the appropriate pathfinding algorithm.
///
/// # Arguments
/// * `graph` — Shared graph reference (read-only, thread-safe).
/// * `query` — Route query specification.
///
/// # Returns
/// `Some(RouteResult)` with the optimal path and physical metrics,
/// or `None` if no path exists.
pub fn execute_query(graph: &Graph, query: &Query) -> Option<RouteResult> {
    if query.algorithm == "ASTAR" {
        solve_astar(graph, query.source, query.destination, &query.optimization)
    } else {
        solve_dijkstra(graph, query.source, query.destination, &query.optimization)
    }
}

/// Executes all queries serially in a single thread.
///
/// Provides a deterministic, reproducible baseline for single-threaded
/// throughput benchmarking. Results preserve query execution order.
///
/// # Arguments
/// * `graph`   — Pre-built Graph instance (read-only).
/// * `queries` — Slice of route query specifications.
///
/// # Returns
/// Vector of all successful `RouteResult` instances in execution order.
pub fn run_sequential(graph: &Graph, queries: &[Query]) -> Vec<RouteResult> {
    queries
        .iter()
        .filter_map(|q| execute_query(graph, q))
        .collect()
}

/// Executes all queries concurrently using a Rayon work-stealing thread pool.
///
/// The `Graph` is shared across threads via Rayon's `par_iter`. Rust's borrow
/// checker statically guarantees that concurrent read access is data-race-free
/// without requiring any runtime locks.
///
/// # Arguments
/// * `graph`       — Shared Graph reference (Sync + Send because all fields are Sync + Send).
/// * `queries`     — Slice of route query specifications.
/// * `num_workers` — Number of worker threads in the Rayon pool.
///
/// # Returns
/// Vector of successful `RouteResult` instances (order not guaranteed).
pub fn run_parallel(graph: &Graph, queries: &[Query], num_workers: usize) -> Vec<RouteResult> {
    let pool = rayon::ThreadPoolBuilder::new()
        .num_threads(num_workers)
        .build()
        .expect("Failed to construct Rayon thread pool");

    pool.install(|| {
        queries
            .par_iter()
            .filter_map(|q| execute_query(graph, q))
            .collect()
    })
}
