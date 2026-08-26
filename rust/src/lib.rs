//! Crate root for the Rust vehicle route optimization benchmark.
//!
//! Re-exports all public modules composing the routing library:
//!
//! - [`models`] — Domain entities: Node, Edge, Vehicle, Config, Query, RouteResult.
//! - [`physics`] — Vehicle dynamics: aerodynamic drag, slope force, rolling resistance, fuel.
//! - [`graph`]   — Weighted directed graph with pre-computed physical edge costs.
//! - [`dijkstra`]— Dijkstra's single-source shortest path (binary min-heap, lazy deletion).
//! - [`astar`]   — A* informed search with admissible Euclidean distance heuristic.
//! - [`engine`]  — Dataset I/O, query dispatch, and Rayon parallel execution harness.

pub mod astar;
pub mod dijkstra;
pub mod engine;
pub mod graph;
pub mod models;
pub mod physics;
