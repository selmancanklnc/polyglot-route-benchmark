/**
 * @file physics.hpp
 * @brief Thermodynamic and aerodynamic vehicle dynamics calculation engine.
 * @author Systems & Algorithms Research Group
 */

#pragma once
#include "models.hpp"
#include <cmath>
#include <algorithm>

namespace route_sim {

/**
 * @brief Computes velocity scaling factor as a function of road gradient angle.
 * @param slope_rad Longitudinal slope in radians.
 * @return Multiplier factor for effective traversal speed.
 */
inline double calculate_slope_factor(double slope_rad) {
    if (slope_rad > 0.0) {
        return std::max(0.5, 1.0 - 2.0 * std::sin(slope_rad));
    } else {
        return 1.0 + 0.5 * std::abs(std::sin(slope_rad));
    }
}

/**
 * @brief Computes effective steady-state speed considering road limits, traffic, and incline.
 * @param edge Road segment specification.
 * @param config Simulation configuration parameters.
 * @return Effective traversal velocity bounded in [10.0, 140.0] km/h.
 */
inline double calculate_effective_speed(const Edge& edge, const Config& config) {
    double base_speed = std::min(edge.speed_limit, edge.average_speed);
    double road_factor = 1.0;
    auto it = config.road_speed_factors.find(edge.road_type);
    if (it != config.road_speed_factors.end()) {
        road_factor = it->second;
    }
    double slope_factor = calculate_slope_factor(edge.slope);
    double eff_speed = base_speed * edge.traffic_factor * road_factor * slope_factor;
    return std::max(10.0, std::min(140.0, eff_speed));
}

/**
 * @brief Evaluates aerodynamic drag force opposing vehicle motion.
 * Formula: F_drag = 0.5 * rho * Cd * A * (v - v_wind)^2
 * @param v_ms Vehicle forward velocity in meters per second.
 * @param edge Road segment environmental wind parameters.
 * @param vehicle Vehicle aerodynamic characteristics.
 * @param config Atmospheric physical parameters.
 * @return Aerodynamic resistive force in Newtons.
 */
inline double calculate_aerodynamic_drag(double v_ms, const Edge& edge, const Vehicle& vehicle, const Config& config) {
    constexpr double pi = 3.14159265358979323846;
    double wind_rad = edge.wind_direction * (pi / 180.0);
    double wind_opposing = edge.wind_speed * std::cos(wind_rad);
    double apparent_speed = std::max(0.0, v_ms - wind_opposing);
    return 0.5 * config.air_density * vehicle.drag_coefficient * vehicle.frontal_area * (apparent_speed * apparent_speed);
}

/**
 * @brief Evaluates gravitational force along road incline.
 * Formula: F_slope = m * g * sin(theta)
 * @param edge Road segment slope gradient.
 * @param vehicle Vehicle mass properties.
 * @param config Gravitational acceleration constant.
 * @return Gravitational force component in Newtons.
 */
inline double calculate_slope_force(const Edge& edge, const Vehicle& vehicle, const Config& config) {
    return vehicle.vehicle_mass * config.gravity * std::sin(edge.slope);
}

/**
 * @brief Evaluates mechanical rolling resistance force.
 * Formula: F_roll = Crr * m * g * cos(theta)
 * @param edge Road segment slope angle.
 * @param vehicle Rolling resistance coefficient and mass.
 * @param config Gravitational acceleration constant.
 * @return Rolling resistance force in Newtons.
 */
inline double calculate_rolling_resistance(const Edge& edge, const Vehicle& vehicle, const Config& config) {
    return vehicle.rolling_resistance_coefficient * vehicle.vehicle_mass * config.gravity * std::cos(edge.slope);
}

/**
 * @brief Performs full physics derivation and edge cost synthesis for an edge.
 * @param edge Target edge entity to mutate with computed costs.
 * @param vehicle Evaluated vehicle powertrain constants.
 * @param config Simulation weights and atmospheric parameters.
 */
inline void compute_edge_metrics(Edge& edge, const Vehicle& vehicle, const Config& config) {
    double v_kmh = calculate_effective_speed(edge, config);
    double v_ms = v_kmh / 3.6;

    double t_hours = edge.distance / v_kmh;
    edge.travel_time_min = t_hours * 60.0;
    double t_seconds = t_hours * 3600.0;

    double f_drag = calculate_aerodynamic_drag(v_ms, edge, vehicle, config);
    double f_slope = calculate_slope_force(edge, vehicle, config);
    double f_roll = calculate_rolling_resistance(edge, vehicle, config);

    double f_total = std::max(0.0, f_drag + f_slope + f_roll);
    double power_watts = f_total * v_ms;
    double energy_mech_joules = power_watts * t_seconds;

    double energy_fuel_joules = energy_mech_joules / vehicle.engine_efficiency;
    edge.fuel_liters = energy_fuel_joules / vehicle.fuel_energy_density;
    edge.fuel_cost = edge.fuel_liters * vehicle.fuel_price;

    edge.cost_fastest = edge.travel_time_min;
    edge.cost_cheapest = edge.fuel_cost + edge.toll_cost;
    edge.cost_balanced = (
        config.fuel_cost_weight * edge.fuel_cost +
        config.time_weight * edge.travel_time_min +
        config.toll_weight * edge.toll_cost
    );
}

} // namespace route_sim
