/**
 * @file astar.hpp
 * @brief A* heuristic-guided shortest path algorithm for multi-objective vehicle routing.
 *
 * Implements A* graph search with an admissible, mode-aware Euclidean distance
 * heuristic. The priority queue is ordered by f-score (g + h), guiding expansion
 * toward geometrically promising vertex neighborhoods.
 *
 * Admissibility: The heuristic returns a lower bound derived from the fastest
 * physically possible traversal speed (140 km/h). Since no real road segment can
 * be traversed faster under the physical model, h(u) <= h*(u) holds for all vertices.
 *
 * Complexity: O((V + E) log V) time, O(V) space.
 * @author Systems & Algorithms Research Group
 */

#pragma once
#include "graph.hpp"
#include <queue>
#include <limits>
#include <chrono>
#include <optional>
#include <algorithm>
#include <cmath>

namespace route_sim {

/**
 * @brief Priority queue entry pairing f-score, g-score, and vertex identifier.
 *
 * Implements operator> for std::greater, enabling std::priority_queue to act
 * as a min-heap ordered by ascending f-score.
 */
struct AStarEntry {
    double f_score; ///< f-score = g-score + heuristic estimate to target.
    double g_score; ///< g-score = actual accumulated cost from source to this vertex.
    int node;       ///< Graph vertex identifier.

    bool operator>(const AStarEntry& other) const {
        return f_score > other.f_score;
    }
};

/**
 * @brief Computes an admissible lower-bound heuristic from u_node to target_node.
 *
 * The Euclidean straight-line distance (km) between the two spatial coordinates
 * is converted to the appropriate objective cost unit based on the mode:
 *
 * - "FASTEST"  -- minimum traversal time at 140 km/h (the physical speed ceiling).
 * - "BALANCED" -- time component scaled by its objective weight coefficient.
 * - "CHEAPEST" -- returns 0.0 (an admissible zero lower bound).
 *
 * @param u_node      Current vertex spatial position.
 * @param target_node Destination vertex spatial position.
 * @param mode        Optimization mode determining heuristic output unit.
 * @param config      Simulation configuration providing weight coefficients.
 * @return Non-negative admissible lower bound on cost from u_node to target_node.
 */
inline double astar_heuristic(const Node& u_node, const Node& target_node, OptMode mode, const Config& config) {
    double dx = target_node.x - u_node.x;
    double dy = target_node.y - u_node.y;
    double dist_km = std::sqrt(dx * dx + dy * dy);
    double min_time_min = (dist_km / 140.0) * 60.0;

    switch (mode) {
        case OptMode::FASTEST:  return min_time_min;
        case OptMode::BALANCED: return config.time_weight * min_time_min;
        default:                return 0.0;
    }
}

/**
 * @brief Computes the cost-optimal route between two vertices using A* search.
 *
 * Maintains g-score (actual cost from source) and f-score (g + heuristic) structures.
 * Priority queue ordering by f-score guides expansion toward geometrically promising
 * regions, reducing settled vertices compared to uninformed Dijkstra.
 *
 * @param graph  Pre-built Graph with physics-evaluated edge costs.
 * @param source Origin vertex identifier.
 * @param target Destination vertex identifier.
 * @param mode   Optimization objective: "FASTEST", "CHEAPEST", or "BALANCED".
 * @return std::optional<RouteResult> with path and physical metrics,
 *         or std::nullopt if unreachable.
 */
inline std::optional<RouteResult> solve_astar(const Graph& graph, int source, int target, const std::string& mode) {
    auto start_time = std::chrono::high_resolution_clock::now();

    if (!graph.has_node(source) || !graph.has_node(target)) {
        return std::nullopt;
    }

    const Node& target_node = graph.nodes[static_cast<size_t>(target)];

    if (source == target) {
        RouteResult res;
        res.source = source;
        res.destination = target;
        res.optimization = mode;
        res.algorithm = "ASTAR";
        res.route = {source};
        res.nodes_explored = 1;
        auto end_time = std::chrono::high_resolution_clock::now();
        res.execution_time_us = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count();
        return res;
    }

    const OptMode opt_mode = parse_mode(mode);

    std::vector<double> g_score(graph.nodes.size(), std::numeric_limits<double>::infinity());
    std::vector<std::pair<int, size_t>> prev(graph.nodes.size());
    std::priority_queue<AStarEntry, std::vector<AStarEntry>, std::greater<AStarEntry>> pq;

    double h0 = astar_heuristic(graph.nodes[static_cast<size_t>(source)], target_node, opt_mode, graph.config);
    g_score[static_cast<size_t>(source)] = 0.0;
    pq.push({h0, 0.0, source});

    int nodes_explored = 0;
    bool found = false;

    while (!pq.empty()) {
        auto [f, g, u] = pq.top();
        (void)f;
        pq.pop();

        if (g > g_score[static_cast<size_t>(u)]) {
            continue;
        }

        nodes_explored++;

        if (u == target) {
            found = true;
            break;
        }

        for (const auto& [v, edge_idx] : graph.adj[static_cast<size_t>(u)]) {
            const auto& edge = graph.edges[edge_idx];
            double weight = graph.get_edge_weight(edge, opt_mode);
            double tentative_g = g + weight;

            if (tentative_g < g_score[static_cast<size_t>(v)]) {
                g_score[static_cast<size_t>(v)] = tentative_g;
                prev[static_cast<size_t>(v)] = {u, edge_idx};
                double h = astar_heuristic(graph.nodes[static_cast<size_t>(v)], target_node, opt_mode, graph.config);
                pq.push({tentative_g + h, tentative_g, v});
            }
        }
    }

    auto end_time = std::chrono::high_resolution_clock::now();
    int64_t exec_us = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count();

    if (!found) return std::nullopt;

    // Backtrack predecessor map to reconstruct the optimal node sequence
    std::vector<int> route_nodes;
    std::vector<size_t> route_edges;
    int curr = target;
    route_nodes.push_back(curr);

    while (curr != source) {
        const auto& [p, edge_idx] = prev[static_cast<size_t>(curr)];
        route_edges.push_back(edge_idx);
        curr = p;
        route_nodes.push_back(curr);
    }

    std::reverse(route_nodes.begin(), route_nodes.end());
    std::reverse(route_edges.begin(), route_edges.end());

    double tot_dist = 0.0;
    double tot_time = 0.0;
    double tot_fuel = 0.0;
    double tot_fuel_cost = 0.0;
    double tot_toll = 0.0;

    for (size_t idx : route_edges) {
        const auto& e = graph.edges[idx];
        tot_dist += e.distance;
        tot_time += e.travel_time_min;
        tot_fuel += e.fuel_liters;
        tot_fuel_cost += e.fuel_cost;
        tot_toll += e.toll_cost;
    }

    RouteResult res;
    res.source = source;
    res.destination = target;
    res.optimization = mode;
    res.algorithm = "ASTAR";
    res.route = std::move(route_nodes);
    res.distance_km = std::round(tot_dist * 10000.0) / 10000.0;
    res.travel_time_min = std::round(tot_time * 10000.0) / 10000.0;
    res.fuel_liters = std::round(tot_fuel * 10000.0) / 10000.0;
    res.fuel_cost = std::round(tot_fuel_cost * 10000.0) / 10000.0;
    res.toll_cost = std::round(tot_toll * 10000.0) / 10000.0;
    res.total_cost = std::round(g_score[static_cast<size_t>(target)] * 10000.0) / 10000.0;
    res.nodes_explored = nodes_explored;
    res.execution_time_us = exec_us;

    return res;
}

} // namespace route_sim
