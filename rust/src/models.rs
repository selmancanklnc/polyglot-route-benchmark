//! Domain models and data transfer representations for the vehicle routing benchmark.
//!
//! Defines the complete type hierarchy for spatial vertices, road segments,
//! vehicular physical constants, atmospheric configuration, query specifications,
//! and route evaluation results. All types derive `serde` traits for transparent
//! JSON deserialization from the canonical benchmark dataset schema.

use serde::{Deserialize, Serialize};
use std::collections::HashMap;

/// Represents a discrete geographic vertex in the spatial road network.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Node {
    /// Unique integer identifier for this vertex.
    pub id: usize,
    /// Local Cartesian X coordinate in kilometers.
    pub x: f64,
    /// Local Cartesian Y coordinate in kilometers.
    pub y: f64,
    /// Elevation above sea level in meters.
    pub elevation: f64,
}

/// Thermodynamic and aerodynamic physical parameters of the benchmark vehicle.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Vehicle {
    /// Gross vehicle mass including nominal payload (kg).
    pub vehicle_mass: f64,
    /// Dimensionless aerodynamic drag coefficient (Cd).
    pub drag_coefficient: f64,
    /// Projected cross-sectional frontal area (m²).
    pub frontal_area: f64,
    /// Dimensionless tire-pavement rolling resistance coefficient (Crr).
    pub rolling_resistance_coefficient: f64,
    /// Powertrain thermal-to-mechanical conversion efficiency in (0.0, 1.0].
    pub engine_efficiency: f64,
    /// Specific volumetric energy density of fuel (J/Liter).
    pub fuel_energy_density: f64,
    /// Financial unit tariff per liter of fuel ($).
    pub fuel_price: f64,
}

/// Atmospheric constants, physical bounds, and multi-objective weight coefficients.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Config {
    /// Standard sea-level atmospheric air density (kg/m³).
    pub air_density: f64,
    /// Standard acceleration due to gravity (m/s²).
    pub gravity: f64,
    /// Optimization weight assigned to fuel expenditure in BALANCED mode.
    pub fuel_cost_weight: f64,
    /// Optimization weight assigned to travel duration in BALANCED mode.
    pub time_weight: f64,
    /// Optimization weight assigned to highway toll fees in BALANCED mode.
    pub toll_weight: f64,
    /// Speed calibration multiplier mapping from road classification string to factor.
    pub road_speed_factors: HashMap<String, f64>,
}

/// Directed road segment connecting two vertices with physical and economic properties.
///
/// Fields annotated with `#[serde(default)]` are computed post-deserialization
/// via [`crate::physics::compute_edge_metrics`] and are zero-initialized in JSON.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Edge {
    /// Unique edge identifier.
    pub id: usize,
    /// Origin vertex identifier.
    pub source: usize,
    /// Destination vertex identifier.
    pub target: usize,
    /// Longitudinal segment traversal distance (km).
    pub distance: f64,
    /// Statutory maximum speed limit (km/h).
    pub speed_limit: f64,
    /// Historical unconstrained average flow speed (km/h).
    pub average_speed: f64,
    /// Road classification string (e.g., `"HIGHWAY"`, `"MAIN_ROAD"`).
    pub road_type: String,
    /// Longitudinal gradient angle (radians).
    pub slope: f64,
    /// Vertical altitude delta from source to target (meters).
    pub elevation_difference: f64,
    /// Traffic congestion index coefficient in (0.0, 1.0].
    pub traffic_factor: f64,
    /// Ambient wind velocity magnitude (m/s).
    pub wind_speed: f64,
    /// Incident wind vector angle relative to vehicle heading (degrees).
    pub wind_direction: f64,
    /// Fixed highway toll tariff ($).
    pub toll_cost: f64,

    // --- Synthesized physical and algorithmic cost metrics ---

    /// Estimated segment transit duration (minutes). Computed post-load.
    #[serde(default)]
    pub travel_time_min: f64,
    /// Volumetric fuel consumption (liters). Computed post-load.
    #[serde(default)]
    pub fuel_liters: f64,
    /// Direct monetary cost of consumed fuel ($). Computed post-load.
    #[serde(default)]
    pub fuel_cost: f64,
    /// Objective cost in FASTEST mode (= travel_time_min). Computed post-load.
    #[serde(default)]
    pub cost_fastest: f64,
    /// Objective cost in CHEAPEST mode (= fuel_cost + toll_cost). Computed post-load.
    #[serde(default)]
    pub cost_cheapest: f64,
    /// Objective cost in BALANCED mode (weighted sum). Computed post-load.
    #[serde(default)]
    pub cost_balanced: f64,
}

/// Origin-destination route calculation request specification.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Query {
    /// Unique query index within the dataset.
    pub id: usize,
    /// Departure vertex identifier.
    pub source: usize,
    /// Target arrival vertex identifier.
    pub destination: usize,
    /// Optimization objective: `"FASTEST"`, `"CHEAPEST"`, or `"BALANCED"`.
    pub optimization: String,
    /// Pathfinding algorithm: `"DIJKSTRA"` or `"ASTAR"`. Defaults to `"DIJKSTRA"`.
    #[serde(default = "default_algorithm")]
    pub algorithm: String,
}

fn default_algorithm() -> String {
    "DIJKSTRA".to_string()
}

/// Top-level container representing a deserialized benchmark JSON dataset.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Dataset {
    /// Arbitrary metadata key-value pairs (dataset name, seed, generation date).
    pub metadata: HashMap<String, serde_json::Value>,
    /// Vehicular physical constants for this benchmark run.
    pub vehicle: Vehicle,
    /// Atmospheric and optimization configuration parameters.
    pub configuration: Config,
    /// Complete ordered list of spatial graph vertices.
    pub nodes: Vec<Node>,
    /// Complete ordered list of directed road segments.
    pub edges: Vec<Edge>,
    /// Query workload to execute against this graph.
    pub queries: Vec<Query>,
}

/// Comprehensive performance and physical evaluation result for a completed route query.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RouteResult {
    /// Origin vertex identifier.
    pub source: usize,
    /// Destination vertex identifier.
    pub destination: usize,
    /// Evaluated optimization strategy.
    pub optimization: String,
    /// Pathfinding algorithm utilized.
    pub algorithm: String,
    /// Ordered sequence of traversed node identifiers forming the optimal path.
    pub route: Vec<usize>,
    /// Total cumulative path distance (km).
    pub distance_km: f64,
    /// Total path traversal duration (minutes).
    pub travel_time_min: f64,
    /// Total volumetric fuel consumed across all segments (liters).
    pub fuel_liters: f64,
    /// Total financial cost of consumed fuel ($).
    pub fuel_cost: f64,
    /// Aggregate highway toll charges ($).
    pub toll_cost: f64,
    /// Final evaluated objective function cost value.
    pub total_cost: f64,
    /// Number of graph vertices settled (popped from priority queue) during search.
    pub nodes_explored: usize,
    /// Algorithmic search wall-clock runtime (microseconds).
    pub execution_time_us: u64,
}
