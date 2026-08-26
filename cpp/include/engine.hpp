/**
 * @file engine.hpp
 * @brief Benchmark execution engine for C++20 route optimization.
 *
 * Handles JSON dataset deserialization, query dispatching, and sequential /
 * multi-threaded execution harnesses.
 * @author Systems & Algorithms Research Group
 */

#pragma once
#include "models.hpp"
#include "graph.hpp"
#include "dijkstra.hpp"
#include "astar.hpp"
#include "json.hpp"
#include <fstream>
#include <sstream>
#include <future>
#include <thread>
#include <iostream>

namespace route_sim {

/**
 * @brief Deserializes a benchmark JSON dataset into a Graph and query list.
 * @param filepath Path to the input JSON file.
 * @return std::pair containing the initialized Graph and query workload.
 */
inline std::pair<Graph, std::vector<Query>> load_dataset(const std::string& filepath) {
    std::ifstream f(filepath);
    if (!f.is_open()) {
        throw std::runtime_error("Could not open dataset file: " + filepath);
    }
    std::stringstream buffer;
    buffer << f.rdbuf();
    std::string content = buffer.str();

    JsonValue root = JsonParser::parse(content);

    const auto& v_obj = root["vehicle"].as_object();
    Vehicle vehicle{
        v_obj.at("vehicle_mass").as_double(),
        v_obj.at("drag_coefficient").as_double(),
        v_obj.at("frontal_area").as_double(),
        v_obj.at("rolling_resistance_coefficient").as_double(),
        v_obj.at("engine_efficiency").as_double(),
        v_obj.at("fuel_energy_density").as_double(),
        v_obj.at("fuel_price").as_double()
    };

    const auto& c_obj = root["configuration"].as_object();
    Config config;
    config.air_density = c_obj.at("air_density").as_double();
    config.gravity = c_obj.at("gravity").as_double();
    config.fuel_cost_weight = c_obj.at("fuel_cost_weight").as_double();
    config.time_weight = c_obj.at("time_weight").as_double();
    config.toll_weight = c_obj.at("toll_weight").as_double();

    for (const auto& [k, v] : c_obj.at("road_speed_factors").as_object()) {
        config.road_speed_factors[k] = v.as_double();
    }

    const auto& node_arr = root["nodes"].as_array();
    std::vector<Node> nodes;
    nodes.reserve(node_arr.size());
    for (const auto& item : node_arr) {
        nodes.push_back({
            item["id"].as_int(),
            item["x"].as_double(),
            item["y"].as_double(),
            item["elevation"].as_double()
        });
    }

    const auto& edge_arr = root["edges"].as_array();
    std::vector<Edge> edges;
    edges.reserve(edge_arr.size());
    for (const auto& item : edge_arr) {
        edges.push_back({
            item["id"].as_int(),
            item["source"].as_int(),
            item["target"].as_int(),
            item["distance"].as_double(),
            item["speed_limit"].as_double(),
            item["average_speed"].as_double(),
            item["road_type"].as_string(),
            item["slope"].as_double(),
            item["elevation_difference"].as_double(),
            item["traffic_factor"].as_double(),
            item["wind_speed"].as_double(),
            item["wind_direction"].as_double(),
            item["toll_cost"].as_double()
        });
    }

    const auto& query_arr = root["queries"].as_array();
    std::vector<Query> queries;
    queries.reserve(query_arr.size());
    for (const auto& item : query_arr) {
        queries.push_back({
            item["id"].as_int(),
            item["source"].as_int(),
            item["destination"].as_int(),
            item["optimization"].as_string(),
            item.contains("algorithm") ? item["algorithm"].as_string() : "DIJKSTRA"
        });
    }

    Graph graph(std::move(nodes), std::move(edges), std::move(vehicle), std::move(config));
    return {std::move(graph), std::move(queries)};
}

/**
 * @brief Dispatches a single query to Dijkstra or A*.
 * @param graph Read-only graph reference.
 * @param query Route query specification.
 * @return std::optional<RouteResult> with path results or nullopt.
 */
inline std::optional<RouteResult> execute_query(const Graph& graph, const Query& query) {
    if (query.algorithm == "ASTAR") {
        return solve_astar(graph, query.source, query.destination, query.optimization);
    } else {
        return solve_dijkstra(graph, query.source, query.destination, query.optimization);
    }
}

/**
 * @brief Executes all queries serially.
 * @param graph Pre-built Graph.
 * @param queries List of route queries.
 * @return Vector of successful RouteResult instances.
 */
inline std::vector<RouteResult> run_sequential(const Graph& graph, const std::vector<Query>& queries) {
    std::vector<RouteResult> results;
    results.reserve(queries.size());
    for (const auto& q : queries) {
        auto res = execute_query(graph, q);
        if (res.has_value()) {
            results.push_back(std::move(*res));
        }
    }
    return results;
}

/**
 * @brief Executes all queries concurrently using standard POSIX/C++ threads.
 * @param graph Pre-built Graph.
 * @param queries List of route queries.
 * @param num_workers Number of worker threads.
 * @return Vector of RouteResult instances.
 */
inline std::vector<RouteResult> run_parallel(const Graph& graph, const std::vector<Query>& queries, int num_workers = 8) {
    std::vector<RouteResult> results(queries.size());
    std::vector<std::thread> workers;
    size_t total = queries.size();
    size_t chunk_size = (total + num_workers - 1) / num_workers;

    for (int w = 0; w < num_workers; ++w) {
        size_t start_idx = w * chunk_size;
        size_t end_idx = std::min(start_idx + chunk_size, total);
        if (start_idx >= total) break;

        workers.emplace_back([&graph, &queries, &results, start_idx, end_idx]() {
            for (size_t i = start_idx; i < end_idx; ++i) {
                auto res = execute_query(graph, queries[i]);
                if (res.has_value()) {
                    results[i] = std::move(*res);
                }
            }
        });
    }

    for (auto& t : workers) {
        if (t.joinable()) t.join();
    }

    return results;
}

} // namespace route_sim
