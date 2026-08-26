/**
 * @file dijkstra.hpp
 * @brief Dijkstra's single-source shortest path algorithm for physical road networks.
 *
 * Implements a binary min-heap (std::priority_queue with std::greater) variant of
 * Dijkstra's algorithm with lazy deletion of stale heap entries. Operates on
 * pre-computed multi-objective edge costs.
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

namespace route_sim {

/**
 * @brief Priority queue entry pairing a vertex identifier with its accumulated cost.
 *
 * Implements operator> for std::greater, enabling std::priority_queue to act
 * as a min-heap ordered by ascending cost.
 */
struct DijkstraEntry {
    double cost; ///< Accumulated objective cost from source to this vertex.
    int node;    ///< Graph vertex identifier.

    bool operator>(const DijkstraEntry& other) const {
        return cost > other.cost;
    }
};

/**
 * @brief Computes the cost-optimal route using Dijkstra's algorithm.
 *
 * Executes standard min-heap relaxation over pre-computed physical edge costs.
 * Stale heap entries are discarded via lazy deletion (cost > dist[u] guard).
 * Path reconstruction backtracks the predecessor map.
 *
 * @param graph  Pre-built Graph with physics-evaluated edge costs.
 * @param source Origin vertex identifier.
 * @param target Destination vertex identifier.
 * @param mode   Optimization objective: "FASTEST", "CHEAPEST", or "BALANCED".
 * @return std::optional<RouteResult> containing the optimal path and metrics,
 *         or std::nullopt if the target is unreachable.
 */
inline std::optional<RouteResult> solve_dijkstra(const Graph& graph, int source, int target, const std::string& mode) {
    auto start_time = std::chrono::high_resolution_clock::now();

    if (!graph.has_node(source) || !graph.has_node(target)) {
        return std::nullopt;
    }

    if (source == target) {
        RouteResult res;
        res.source = source;
        res.destination = target;
        res.optimization = mode;
        res.algorithm = "DIJKSTRA";
        res.route = {source};
        res.nodes_explored = 1;
        auto end_time = std::chrono::high_resolution_clock::now();
        res.execution_time_us = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count();
        return res;
    }

    const OptMode opt_mode = parse_mode(mode);

    // dist[v] = minimum accumulated cost recorded to vertex v
    std::vector<double> dist(graph.nodes.size(), std::numeric_limits<double>::infinity());
    // prev[v] = (predecessor_vertex, edge_index) for path reconstruction
    std::vector<std::pair<int, size_t>> prev(graph.nodes.size());
    std::priority_queue<DijkstraEntry, std::vector<DijkstraEntry>, std::greater<DijkstraEntry>> pq;

    dist[static_cast<size_t>(source)] = 0.0;
    pq.push({0.0, source});
    int nodes_explored = 0;
    bool found = false;

    while (!pq.empty()) {
        auto [d, u] = pq.top();
        pq.pop();

        if (d > dist[static_cast<size_t>(u)]) {
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
            double new_dist = d + weight;

            if (new_dist < dist[static_cast<size_t>(v)]) {
                dist[static_cast<size_t>(v)] = new_dist;
                prev[static_cast<size_t>(v)] = {u, edge_idx};
                pq.push({new_dist, v});
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
    res.algorithm = "DIJKSTRA";
    res.route = std::move(route_nodes);
    res.distance_km = std::round(tot_dist * 10000.0) / 10000.0;
    res.travel_time_min = std::round(tot_time * 10000.0) / 10000.0;
    res.fuel_liters = std::round(tot_fuel * 10000.0) / 10000.0;
    res.fuel_cost = std::round(tot_fuel_cost * 10000.0) / 10000.0;
    res.toll_cost = std::round(tot_toll * 10000.0) / 10000.0;
    res.total_cost = std::round(dist[static_cast<size_t>(target)] * 10000.0) / 10000.0;
    res.nodes_explored = nodes_explored;
    res.execution_time_us = exec_us;

    return res;
}

} // namespace route_sim
