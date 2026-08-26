/**
 * @file graph.hpp
 * @brief Weighted directed road network graph with pre-computed physical edge costs.
 * @author Systems & Algorithms Research Group
 */

#pragma once
#include "models.hpp"
#include "physics.hpp"
#include <vector>
#include <unordered_map>
#include <utility>

namespace route_sim {

/**
 * @brief Weighted directed road network with pre-computed multi-objective edge costs.
 *
 * Constructs an adjacency-list graph from raw vertex and edge datasets. All edge
 * physical metrics are evaluated once at construction time via compute_edge_metrics,
 * rendering the graph read-only and safe for concurrent query execution.
 *
 * Adjacency entries store (target_id, edge_index) pairs indexed directly into the
 * edges vector, avoiding per-query memory allocation during priority queue relaxation.
 */
class Graph {
public:
    /// Mapping from node identifier to spatial Node geometry.
    std::unordered_map<int, Node> nodes;
    /// Ordered vector of all directed road segments with synthesized cost fields.
    std::vector<Edge> edges;
    /// Vehicular physical constants for this benchmark run.
    Vehicle vehicle;
    /// Atmospheric parameters and multi-objective weight coefficients.
    Config config;
    /// Forward adjacency index: node_id -> [(target_id, edge_index_in_edges)].
    std::unordered_map<int, std::vector<std::pair<int, size_t>>> adj;

    /**
     * @brief Constructs the graph, triggers physics derivation on all edges,
     *        and builds the forward adjacency index.
     * @param n_list Complete list of spatial graph vertices.
     * @param e_list Complete list of directed road segments.
     * @param v      Vehicular aerodynamic and thermodynamic constants.
     * @param c      Atmospheric parameters and optimization weight coefficients.
     */
    Graph(std::vector<Node> n_list, std::vector<Edge> e_list, Vehicle v, Config c)
        : vehicle(std::move(v)), config(std::move(c)) {
        for (auto& n : n_list) {
            nodes.emplace(n.id, n);
            adj[n.id] = {};
        }

        edges = std::move(e_list);
        for (size_t i = 0; i < edges.size(); ++i) {
            compute_edge_metrics(edges[i], vehicle, config);
            adj[edges[i].source].emplace_back(edges[i].target, i);
        }
    }

    /**
     * @brief Returns the pre-computed objective cost for the specified optimization mode.
     * @param edge Road segment whose cost field is to be selected.
     * @param mode Optimization strategy: "FASTEST", "CHEAPEST", or "BALANCED".
     * @return Scalar edge traversal cost for priority queue relaxation.
     */
    double get_edge_weight(const Edge& edge, const std::string& mode) const {
        if (mode == "FASTEST")  return edge.cost_fastest;
        if (mode == "CHEAPEST") return edge.cost_cheapest;
        return edge.cost_balanced;
    }
};

} // namespace route_sim
