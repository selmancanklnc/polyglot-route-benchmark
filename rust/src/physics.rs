//! Thermodynamic and aerodynamic vehicle dynamics calculation engine.
//!
//! Implements the complete physical resistance model for a vehicle traversing
//! a road segment under real-world environmental conditions:
//!
//! 1. **Aerodynamic Drag**: `F_drag = 0.5 · ρ · Cd · A · v_apparent²`
//! 2. **Slope Gravitational Force**: `F_slope = m · g · sin(θ)`
//! 3. **Rolling Resistance**: `F_roll = Crr · m · g · cos(θ)`
//! 4. **Powertrain Energy Conversion**: `E_fuel = (F_total · v · t) / η`
//! 5. **Volumetric Fuel Consumption**: `V = E_fuel / ρ_fuel`
//!
//! All functions operate on shared references to domain models and are
//! pure (side-effect-free) except for `compute_edge_metrics`, which mutates
//! the target [`Edge`] in-place.

use crate::models::{Config, Edge, Vehicle};
use std::f64::consts::PI;

/// Computes the speed attenuation multiplier due to longitudinal road gradient.
///
/// Uphill gradients reduce effective speed; downhill gradients yield a modest
/// improvement from gravitational assist, bounded to prevent unrealistic acceleration.
///
/// # Arguments
/// * `slope_rad` - Longitudinal incline angle in radians.
///
/// # Returns
/// Dimensionless factor in [0.5, ∞) applied to nominal traversal speed.
pub fn calculate_slope_factor(slope_rad: f64) -> f64 {
    if slope_rad > 0.0 {
        (1.0 - 2.0 * slope_rad.sin()).max(0.5)
    } else {
        1.0 + 0.5 * slope_rad.sin().abs()
    }
}

/// Evaluates the effective steady-state traversal velocity along a road segment.
///
/// Accounts for statutory speed limits, historical average flow, traffic congestion
/// index, road typology classification multiplier, and gravitational slope effects.
/// Result is clamped to the physical domain `[10.0, 140.0]` km/h.
///
/// # Arguments
/// * `edge`   - Road segment specification.
/// * `config` - Simulation environment constants including road speed factors.
///
/// # Returns
/// Effective vehicle velocity in km/h.
pub fn calculate_effective_speed(edge: &Edge, config: &Config) -> f64 {
    let base_speed = edge.speed_limit.min(edge.average_speed);
    let road_factor = config
        .road_speed_factors
        .get(&edge.road_type)
        .copied()
        .unwrap_or(1.0);
    let slope_factor = calculate_slope_factor(edge.slope);
    let eff_speed = base_speed * edge.traffic_factor * road_factor * slope_factor;
    eff_speed.clamp(10.0, 140.0)
}

/// Evaluates aerodynamic drag force opposing the direction of vehicle motion.
///
/// Computes apparent vehicle speed relative to the wind vector using the incident
/// wind angle projection. Only the headwind component (cosine projection) is
/// subtracted from vehicle velocity; crosswinds are excluded from drag per the
/// simplified 1D longitudinal model.
///
/// Formula: `F_drag = 0.5 · ρ_air · Cd · A_front · max(0, v - v_wind·cos(φ))²`
///
/// # Arguments
/// * `v_ms`    - Vehicle velocity relative to road surface (m/s).
/// * `edge`    - Road segment environmental wind parameters.
/// * `vehicle` - Vehicle aerodynamic attributes (Cd, frontal area).
/// * `config`  - Atmospheric air density.
///
/// # Returns
/// Aerodynamic resistive force in Newtons.
pub fn calculate_aerodynamic_drag(v_ms: f64, edge: &Edge, vehicle: &Vehicle, config: &Config) -> f64 {
    let wind_rad = edge.wind_direction * (PI / 180.0);
    let wind_opposing = edge.wind_speed * wind_rad.cos();
    let apparent_speed = (v_ms - wind_opposing).max(0.0);
    0.5 * config.air_density * vehicle.drag_coefficient * vehicle.frontal_area * apparent_speed.powi(2)
}

/// Evaluates the longitudinal gravitational resistance component along the road incline.
///
/// Formula: `F_slope = m · g · sin(θ)`
///
/// Positive slope produces a resistive force (uphill); negative slope produces
/// a propulsive gravitational assist (handled via signed arithmetic).
///
/// # Arguments
/// * `edge`    - Road segment slope angle θ in radians.
/// * `vehicle` - Vehicle mass.
/// * `config`  - Standard gravitational acceleration.
///
/// # Returns
/// Slope gravitational force component in Newtons.
pub fn calculate_slope_force(edge: &Edge, vehicle: &Vehicle, config: &Config) -> f64 {
    vehicle.vehicle_mass * config.gravity * edge.slope.sin()
}

/// Evaluates Coulomb tire-pavement rolling resistance.
///
/// Formula: `F_roll = Crr · m · g · cos(θ)`
///
/// # Arguments
/// * `edge`    - Road segment slope angle θ in radians (affects normal force).
/// * `vehicle` - Vehicle mass and rolling resistance coefficient.
/// * `config`  - Standard gravitational acceleration.
///
/// # Returns
/// Rolling resistance force in Newtons.
pub fn calculate_rolling_resistance(edge: &Edge, vehicle: &Vehicle, config: &Config) -> f64 {
    vehicle.rolling_resistance_coefficient * vehicle.vehicle_mass * config.gravity * edge.slope.cos()
}

/// Performs complete physics derivation and multi-objective cost synthesis for an edge.
///
/// Mutates the target edge in-place with:
/// - Travel time in minutes
/// - Volumetric fuel consumption in liters
/// - Monetary fuel cost ($)
/// - Per-mode objective costs (FASTEST, CHEAPEST, BALANCED)
///
/// This function is called once per edge at graph construction time and results
/// are cached for the lifetime of the graph.
///
/// # Arguments
/// * `edge`    - Mutable reference to the road segment to populate.
/// * `vehicle` - Vehicular physical constants.
/// * `config`  - Atmospheric parameters and multi-objective weight coefficients.
pub fn compute_edge_metrics(edge: &mut Edge, vehicle: &Vehicle, config: &Config) {
    let v_kmh = calculate_effective_speed(edge, config);
    let v_ms = v_kmh / 3.6;

    let t_hours = edge.distance / v_kmh;
    edge.travel_time_min = t_hours * 60.0;
    let t_seconds = t_hours * 3600.0;

    let f_drag  = calculate_aerodynamic_drag(v_ms, edge, vehicle, config);
    let f_slope = calculate_slope_force(edge, vehicle, config);
    let f_roll  = calculate_rolling_resistance(edge, vehicle, config);

    let f_total = (f_drag + f_slope + f_roll).max(0.0);
    let power_watts        = f_total * v_ms;
    let energy_mech_joules = power_watts * t_seconds;

    let energy_fuel_joules = energy_mech_joules / vehicle.engine_efficiency;
    edge.fuel_liters = energy_fuel_joules / vehicle.fuel_energy_density;
    edge.fuel_cost   = edge.fuel_liters * vehicle.fuel_price;

    edge.cost_fastest  = edge.travel_time_min;
    edge.cost_cheapest = edge.fuel_cost + edge.toll_cost;
    edge.cost_balanced = config.fuel_cost_weight * edge.fuel_cost
        + config.time_weight * edge.travel_time_min
        + config.toll_weight * edge.toll_cost;
}
