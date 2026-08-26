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
 * @brief Optimization objective, resolved once per query to avoid repeated
 *        string comparisons during priority queue relaxation.
 */
enum class OptMode { FASTEST, CHEAPEST, BALANCED };

/**
 * @brief Resolves an optimization mode string to its enum representation.
 * @param mode Optimization strategy: "FASTEST", "CHEAPEST", or "BALANCED".
 * @return Corresponding OptMode value; defaults to BALANCED.
 */
inline OptMode parse_mode(const std::string& mode) {
    if (mode == "FASTEST")  return OptMode::FASTEST;
    if (mode == "CHEAPEST") return OptMode::CHEAPEST;
    return OptMode::BALANCED;
}

/**
 * @brief Weighted directed road network with pre-computed multi-objective edge costs.
 *
 * Constructs an adjacency-list graph from raw vertex and edge datasets. All edge
 * physical metrics are evaluated once at construction time via compute_edge_metrics,
 * rendering the graph read-only and safe for concurrent query execution.
 *
 * Node identifiers are assumed dense over [0, nodes.size()), matching the
 * benchmark dataset contract; nodes and adjacency are stored as vectors indexed
 * directly by id, avoiding hashing during priority queue relaxation.
 */
class Graph {
public:
    /// Node geometry indexed directly by node id.
    std::vector<Node> nodes;
    /// Ordered vector of all directed road segments with synthesized cost fields.
    std::vector<Edge> edges;
    /// Vehicular physical constants for this benchmark run.
    Vehicle vehicle;
    /// Atmospheric parameters and multi-objective weight coefficients.
    Config config;
    /// Forward adjacency index: node_id -> [(target_id, edge_index_in_edges)].
    std::vector<std::vector<std::pair<int, size_t>>> adj;

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
        int max_id = -1;
        for (const auto& n : n_list) max_id = std::max(max_id, n.id);
        nodes.resize(static_cast<size_t>(max_id) + 1);
        for (const auto& n : n_list) nodes[static_cast<size_t>(n.id)] = n;
        adj.resize(nodes.size());

        edges = std::move(e_list);
        for (size_t i = 0; i < edges.size(); ++i) {
            compute_edge_metrics(edges[i], vehicle, config);
            adj[static_cast<size_t>(edges[i].source)].emplace_back(edges[i].target, i);
        }
    }

    /**
     * @brief Checks whether the given identifier refers to a vertex in the graph.
     * @param id Candidate node identifier.
     * @return true if id lies within the valid node index range.
     */
    bool has_node(int id) const noexcept {
        return id >= 0 && static_cast<size_t>(id) < nodes.size();
    }

    /**
     * @brief Returns the pre-computed objective cost for the specified optimization mode.
     * @param edge Road segment whose cost field is to be selected.
     * @param mode Resolved optimization strategy.
     * @return Scalar edge traversal cost for priority queue relaxation.
     */
    double get_edge_weight(const Edge& edge, OptMode mode) const noexcept {
        switch (mode) {
            case OptMode::FASTEST:  return edge.cost_fastest;
            case OptMode::CHEAPEST: return edge.cost_cheapest;
            default:                return edge.cost_balanced;
        }
    }
};

} // namespace route_sim
