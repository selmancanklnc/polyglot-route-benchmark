#!/usr/bin/env python3
"""Monte Carlo Stochastic Uncertainty Simulation for Optimal Route Risk Analysis.

This module executes N independent stochastic realizations of a deterministic
optimal route under physically-motivated random perturbations, providing a
rigorous uncertainty quantification (UQ) framework for vehicle routing systems.

Motivation:
    Real-world road conditions introduce continuous stochastic variability
    (traffic bursts, wind gusts, road surface degradation) that deterministic
    shortest-path algorithms cannot capture. Monte Carlo simulation provides
    empirical confidence intervals and worst-case risk bounds that are critical
    for applications such as freight logistics SLA planning and emergency routing.

Physical Perturbation Model:
    For each simulation realization i ∈ {1, ..., N}:
      - Traffic factor:  T_i ~ Clip(N(T_0, σ_T=0.08), 0.2, 1.0)
      - Wind speed:      W_i ~ Clip(N(W_0, σ_W=3.0),  0.0, ∞)
    All other physical laws (aerodynamic drag, slope force, rolling resistance,
    thermal efficiency) are evaluated deterministically at perturbed inputs.

Statistical Output:
    Computes mean, standard deviation, 5th/50th/95th percentile, and symmetric
    95% confidence intervals over travel time (minutes) and fuel cost ($).
"""

import sys
import os
import math
import random

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'python', 'src'))

from engine import load_dataset
from dijkstra import solve_dijkstra


def run_monte_carlo(
    graph,
    route_nodes: list,
    num_simulations: int = 10_000,
    seed: int = 42
) -> dict:
    """Executes stochastic Monte Carlo sampling over a fixed optimal route.

    For each simulation, stochastic noise is injected into traffic congestion
    and wind speed parameters, after which the full aerodynamic and thermodynamic
    physics chain is re-evaluated to obtain perturbed travel time and fuel cost.

    Args:
        graph: Constructed Graph instance providing vehicle, config, and adjacency data.
        route_nodes: Ordered list of node IDs comprising the optimal route path.
        num_simulations: Total number of Monte Carlo realizations to execute.
        seed: Deterministic RNG seed for full reproducibility across runs.

    Returns:
        Dictionary containing the following statistical summaries for both
        ``travel_time_min`` and ``fuel_cost`` keys:
          - ``mean``: Arithmetic mean across all realizations.
          - ``std``: Sample standard deviation.
          - ``p5_best_case``: 5th percentile (optimistic bound).
          - ``p50_median``: 50th percentile (median estimate).
          - ``p95_worst_case``: 95th percentile (pessimistic bound).
          - ``ci_95_low`` / ``ci_95_high``: 2.5th and 97.5th percentile bounds
            forming the symmetric 95% confidence interval.
    """
    rng = random.Random(seed)

    # Extract the Edge objects traversed along the optimal route
    route_edges = []
    for i in range(len(route_nodes) - 1):
        u, v = route_nodes[i], route_nodes[i + 1]
        for neighbor, edge in graph.adj[u]:
            if neighbor == v:
                route_edges.append(edge)
                break

    times_min = []
    fuel_costs = []

    for _ in range(num_simulations):
        tot_time = 0.0
        tot_fuel_cost = 0.0

        for edge in route_edges:
            # --- Stochastic traffic perturbation ---
            eff_traffic = max(0.2, min(1.0, edge.traffic_factor + rng.gauss(0.0, 0.08)))

            # --- Stochastic wind perturbation ---
            eff_wind = max(0.0, edge.wind_speed + rng.gauss(0.0, 3.0))

            # --- Re-evaluate effective vehicle speed ---
            base_speed = min(edge.speed_limit, edge.average_speed)
            road_factor = graph.config.road_speed_factors.get(edge.road_type, 1.0)
            if edge.slope > 0:
                slope_factor = max(0.5, 1.0 - 2.0 * math.sin(edge.slope))
            else:
                slope_factor = 1.0 + 0.5 * abs(math.sin(edge.slope))
            v_kmh = max(10.0, min(140.0, base_speed * eff_traffic * road_factor * slope_factor))
            v_ms = v_kmh / 3.6

            t_hours = edge.distance / v_kmh
            t_min = t_hours * 60.0
            t_sec = t_hours * 3600.0

            # --- Aerodynamic drag with perturbed apparent wind ---
            wind_rad = math.radians(edge.wind_direction)
            apparent_speed = max(0.0, v_ms - eff_wind * math.cos(wind_rad))
            f_drag = (0.5 * graph.config.air_density
                      * graph.vehicle.drag_coefficient
                      * graph.vehicle.frontal_area
                      * (apparent_speed ** 2))

            # --- Slope and rolling resistance (deterministic at given geometry) ---
            f_slope = (graph.vehicle.vehicle_mass
                       * graph.config.gravity
                       * math.sin(edge.slope))
            f_roll = (graph.vehicle.rolling_resistance_coefficient
                      * graph.vehicle.vehicle_mass
                      * graph.config.gravity
                      * math.cos(edge.slope))

            f_total = max(0.0, f_drag + f_slope + f_roll)
            power_w = f_total * v_ms
            e_mech = power_w * t_sec
            e_fuel = e_mech / graph.vehicle.engine_efficiency
            liters = e_fuel / graph.vehicle.fuel_energy_density
            cost = liters * graph.vehicle.fuel_price

            tot_time += t_min
            tot_fuel_cost += cost

        times_min.append(tot_time)
        fuel_costs.append(tot_fuel_cost)

    times_arr = np.array(times_min)
    costs_arr = np.array(fuel_costs)

    def _summarize(arr: np.ndarray) -> dict:
        return {
            "mean":          float(np.mean(arr)),
            "std":           float(np.std(arr, ddof=1)),
            "p5_best_case":  float(np.percentile(arr, 5)),
            "p50_median":    float(np.percentile(arr, 50)),
            "p95_worst_case":float(np.percentile(arr, 95)),
            "ci_95_low":     float(np.percentile(arr, 2.5)),
            "ci_95_high":    float(np.percentile(arr, 97.5)),
        }

    return {
        "num_simulations": num_simulations,
        "travel_time_min": _summarize(times_arr),
        "fuel_cost":       _summarize(costs_arr),
    }


def main() -> None:
    """CLI entry point: loads the test fixture, solves the baseline route, and runs simulation."""
    dataset_path = os.path.join(
        os.path.dirname(__file__), '..', 'datasets', 'test_fixture.json'
    )
    graph, _ = load_dataset(dataset_path)

    baseline = solve_dijkstra(graph, 0, 9, "BALANCED")
    if baseline is None:
        print("ERROR: No feasible route found between Node 0 and Node 9.", file=sys.stderr)
        sys.exit(1)

    stats = run_monte_carlo(graph, baseline.route, num_simulations=10_000)

    t = stats["travel_time_min"]
    c = stats["fuel_cost"]

    print("=" * 70)
    print("  MONTE CARLO STOCHASTIC UNCERTAINTY SIMULATION  (N = 10,000 Runs)")
    print("  Route: Origin Node 0  →  Destination Node 9")
    print("=" * 70)
    print(f"  Deterministic Baseline — Travel Time : {baseline.travel_time_min:6.2f} min"
          f"  |  Fuel Cost : ${baseline.fuel_cost:.2f}")
    print("-" * 70)
    print(f"  Stochastic Mean Travel Time          : {t['mean']:6.2f} ± {t['std']:.2f} min")
    print(f"  95% Confidence Interval (Time)       : [{t['ci_95_low']:.2f}  –  {t['ci_95_high']:.2f}] min")
    print(f"  5th / 95th Percentile   (Time)       : {t['p5_best_case']:.2f} min  /  {t['p95_worst_case']:.2f} min")
    print("-" * 70)
    print(f"  Stochastic Mean Fuel Cost            : ${c['mean']:6.2f} ± ${c['std']:.2f}")
    print(f"  95% Confidence Interval (Cost)       : [${c['ci_95_low']:.2f}  –  ${c['ci_95_high']:.2f}]")
    print(f"  5th / 95th Percentile   (Cost)       : ${c['p5_best_case']:.2f}  /  ${c['p95_worst_case']:.2f}")
    print("=" * 70)


if __name__ == "__main__":
    main()
