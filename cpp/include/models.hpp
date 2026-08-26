/**
 * @file models.hpp
 * @brief Core domain entities and data transfer representations for routing benchmarks.
 * @author Systems & Algorithms Research Group
 */

#pragma once
#include <string>
#include <vector>
#include <unordered_map>
#include <cstdint>

namespace route_sim {

/**
 * @brief Represents a physical geographic vertex in the road network graph.
 */
struct Node {
    int id;              ///< Unique integer identifier of the node
    double x;           ///< Local Cartesian X coordinate (km)
    double y;           ///< Local Cartesian Y coordinate (km)
    double elevation;   ///< Elevation above sea level (meters)
};

/**
 * @brief Physical and thermodynamic characteristics of the evaluated test vehicle.
 */
struct Vehicle {
    double vehicle_mass;                    ///< Total vehicle mass including nominal payload (kg)
    double drag_coefficient;                ///< Aerodynamic drag coefficient (Cd, dimensionless)
    double frontal_area;                    ///< Effective projected frontal area (m^2)
    double rolling_resistance_coefficient;  ///< Tire-road rolling resistance coefficient (Crr)
    double engine_efficiency;               ///< Powertrain thermal/mechanical conversion efficiency (0.0 - 1.0)
    double fuel_energy_density;             ///< Specific volumetric energy density of fuel (J/Liter)
    double fuel_price;                      ///< Financial unit cost of fuel per liter ($)
};

/**
 * @brief Global simulation environment constants and optimization weight parameters.
 */
struct Config {
    double air_density;                                         ///< Ambient atmospheric air density (kg/m^3)
    double gravity;                                             ///< Standard acceleration due to gravity (m/s^2)
    double fuel_cost_weight;                                    ///< Relative weight assigned to fuel expenditure
    double time_weight;                                         ///< Relative weight assigned to travel duration
    double toll_weight;                                         ///< Relative weight assigned to highway toll fees
    std::unordered_map<std::string, double> road_speed_factors; ///< Speed calibration multipliers by road type
};

/**
 * @brief Represents a directed road segment connecting two vertices with physical properties.
 */
struct Edge {
    int id;                         ///< Unique edge identifier
    int source;                     ///< Origin vertex identifier
    int target;                     ///< Destination vertex identifier
    double distance;                ///< Segment traversal distance (km)
    double speed_limit;             ///< Statutory maximum speed limit (km/h)
    double average_speed;           ///< Historical unconstrained average flow speed (km/h)
    std::string road_type;          ///< Classification (e.g. HIGHWAY, MAIN_ROAD, CITY_ROAD)
    double slope;                   ///< Longitudinal gradient angle (radians)
    double elevation_difference;    ///< Vertical elevation delta from source to target (meters)
    double traffic_factor;          ///< Congestion index coefficient in interval (0.0, 1.0]
    double wind_speed;              ///< Ambient wind velocity magnitude (m/s)
    double wind_direction;          ///< Relative wind vector angle (degrees, 0 = headwind)
    double toll_cost;               ///< Fixed monetary toll tariff ($)

    // Dynamically computed physical and algorithmic metrics
    double travel_time_min = 0.0;   ///< Estimated segment transit duration in minutes
    double fuel_liters = 0.0;       ///< Volumetric fuel consumption in liters
    double fuel_cost = 0.0;         ///< Direct monetary cost of consumed fuel ($)
    double cost_fastest = 0.0;      ///< Objective cost under FASTEST optimization mode
    double cost_cheapest = 0.0;     ///< Objective cost under CHEAPEST optimization mode
    double cost_balanced = 0.0;     ///< Objective cost under BALANCED multi-criteria mode
};

/**
 * @brief Route computation request specification.
 */
struct Query {
    int id;                         ///< Unique query identifier
    int source;                     ///< Departure node identifier
    int destination;                ///< Target arrival node identifier
    std::string optimization;       ///< Optimization criterion ("FASTEST", "CHEAPEST", "BALANCED")
    std::string algorithm;          ///< Selected pathfinding algorithm ("DIJKSTRA", "ASTAR")
};

/**
 * @brief Comprehensive evaluation result for an executed route query.
 */
struct RouteResult {
    int source;                     ///< Origin node identifier
    int destination;                ///< Destination node identifier
    std::string optimization;       ///< Optimization strategy evaluated
    std::string algorithm;          ///< Pathfinding algorithm utilized
    std::vector<int> route;         ///< Sequential ordered node IDs traversing the optimal path
    double distance_km = 0.0;       ///< Total cumulative route distance (km)
    double travel_time_min = 0.0;   ///< Total route traversal duration (minutes)
    double fuel_liters = 0.0;       ///< Total fuel consumed across all traversed segments (Liters)
    double fuel_cost = 0.0;         ///< Aggregate fuel expenditure ($)
    double toll_cost = 0.0;         ///< Aggregate highway toll charges ($)
    double total_cost = 0.0;        ///< Final objective function cost value
    int nodes_explored = 0;         ///< Cardinality of graph vertices popped from priority queue
    int64_t execution_time_us = 0;  ///< Algorithmic search runtime in microseconds
};

} // namespace route_sim
