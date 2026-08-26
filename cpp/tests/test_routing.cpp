#include "engine.hpp"
#include <cassert>
#include <iostream>
#include <cmath>

using namespace route_sim;

void test_routing_fixture(const std::string& fixture_path) {
    auto [graph, queries] = load_dataset(fixture_path);
    assert(graph.nodes.size() == 10);
    assert(graph.edges.size() == 26);
    assert(queries.size() == 6);

    for (const auto& q : queries) {
        auto res_d = solve_dijkstra(graph, q.source, q.destination, q.optimization);
        auto res_a = solve_astar(graph, q.source, q.destination, q.optimization);

        assert(res_d.has_value());
        assert(res_a.has_value());
        assert(std::abs(res_d->total_cost - res_a->total_cost) < 1e-4);
        assert(res_d->route == res_a->route);
    }
    std::cout << "[PASS] test_routing_fixture\n";
}

void test_edge_cases(const std::string& fixture_path) {
    auto [graph, queries] = load_dataset(fixture_path);

    // Unreachable
    auto res_unreachable = solve_dijkstra(graph, 0, 9999, "FASTEST");
    assert(!res_unreachable.has_value());

    // Same source and target
    auto res_same = solve_dijkstra(graph, 3, 3, "FASTEST");
    assert(res_same.has_value());
    assert(res_same->route.size() == 1 && res_same->route[0] == 3);
    assert(res_same->total_cost == 0.0);

    std::cout << "[PASS] test_edge_cases\n";
}

int main(int argc, char* argv[]) {
    std::string fixture_path = "../datasets/test_fixture.json";
    if (argc > 1) fixture_path = argv[1];

    std::cout << "--- Running C++ Routing Tests ---\n";
    test_routing_fixture(fixture_path);
    test_edge_cases(fixture_path);
    std::cout << "All C++ Routing Tests Passed Successfully!\n";
    return 0;
}
