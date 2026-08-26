/**
 * @file test_physics.cpp
 * @brief Unit verification suite for C++ vehicle dynamics and physical calculations.
 */

#include "physics.hpp"
#include <cassert>
#include <iostream>
#include <cmath>

using namespace route_sim;

void test_aerodynamic_drag() {
    Vehicle v{1400.0, 0.30, 2.2, 0.012, 0.30, 34.2e6, 40.0};
    Config c{1.225, 9.81, 0.40, 0.40, 0.20, {{"HIGHWAY", 1.05}}};
    Edge edge{1, 0, 1, 10.0, 100.0, 100.0, "HIGHWAY", 0.0, 0.0, 1.0, 0.0, 0.0, 0.0};

    double v_ms = 100.0 / 3.6;
    double drag = calculate_aerodynamic_drag(v_ms, edge, v, c);
    double expected = 0.5 * 1.225 * 0.30 * 2.2 * (v_ms * v_ms);
    if (std::abs(drag - expected) >= 1e-4) {
        std::cerr << "Drag calculation mismatch: got " << drag << ", expected " << expected << "\n";
        std::exit(1);
    }
    std::cout << "[PASS] test_aerodynamic_drag\n";
}

void test_slope_force() {
    Vehicle v{1400.0, 0.30, 2.2, 0.012, 0.30, 34.2e6, 40.0};
    Config c{1.225, 9.81, 0.40, 0.40, 0.20, {{"MAIN_ROAD", 0.95}}};
    constexpr double pi = 3.14159265358979323846;
    double slope_angle = 5.0 * (pi / 180.0);
    Edge edge{1, 0, 1, 1.0, 60.0, 60.0, "MAIN_ROAD", slope_angle, 87.0, 1.0, 0.0, 0.0, 0.0};

    double f_slope = calculate_slope_force(edge, v, c);
    double expected = 1400.0 * 9.81 * std::sin(slope_angle);
    if (std::abs(f_slope - expected) >= 1e-3) {
        std::cerr << "Slope force mismatch: got " << f_slope << ", expected " << expected << "\n";
        std::exit(1);
    }
    std::cout << "[PASS] test_slope_force\n";
}

void test_rolling_resistance() {
    Vehicle v{1400.0, 0.30, 2.2, 0.012, 0.30, 34.2e6, 40.0};
    Config c{1.225, 9.81, 0.40, 0.40, 0.20, {{"MAIN_ROAD", 0.95}}};
    Edge edge{1, 0, 1, 1.0, 60.0, 60.0, "MAIN_ROAD", 0.0, 0.0, 1.0, 0.0, 0.0, 0.0};

    double f_roll = calculate_rolling_resistance(edge, v, c);
    double expected = 0.012 * 1400.0 * 9.81 * 1.0;
    if (std::abs(f_roll - expected) >= 1e-3) {
        std::cerr << "Rolling resistance mismatch: got " << f_roll << ", expected " << expected << "\n";
        std::exit(1);
    }
    std::cout << "[PASS] test_rolling_resistance\n";
}

void test_edge_metrics_consistency() {
    Vehicle v{1400.0, 0.30, 2.2, 0.012, 0.30, 34.2e6, 40.0};
    Config c{1.225, 9.81, 0.40, 0.40, 0.20, {{"HIGHWAY", 1.05}}};
    Edge edge{1, 0, 1, 5.0, 120.0, 110.0, "HIGHWAY", 0.01, 50.0, 0.9, 5.0, 0.0, 15.0};

    compute_edge_metrics(edge, v, c);
    assert(edge.travel_time_min > 0.0);
    assert(edge.fuel_liters > 0.0);
    assert(std::abs(edge.cost_fastest - edge.travel_time_min) < 1e-5);
    assert(std::abs(edge.cost_cheapest - (edge.fuel_cost + 15.0)) < 1e-5);
    std::cout << "[PASS] test_edge_metrics_consistency\n";
}

int main() {
    std::cout << "--- Running C++ Physics Verification Suite ---\n";
    test_aerodynamic_drag();
    test_slope_force();
    test_rolling_resistance();
    test_edge_metrics_consistency();
    std::cout << "All C++ Physics Tests Passed Successfully with Zero Warnings.\n";
    return 0;
}
