#!/usr/bin/env python3
"""Multi-Objective Pareto Frontier Analysis for Route Optimization Trade-offs.

This module identifies the set of Pareto-optimal (non-dominated) routes
within the road network by performing a systematic sweep over the objective
weight simplex and filtering solutions that cannot be improved in any
dimension without degrading another.

Background:
    Multi-objective optimization problems where objectives conflict
    (minimize travel time vs. minimize fuel expenditure vs. minimize toll fees)
    do not admit a unique optimal solution. The Pareto frontier — the set of
    non-dominated alternatives — represents the complete rational choice set
    from which a decision-maker selects according to their own preference ordering.

Methodology:
    1. Enumerate all integer weight combinations (w_fuel, w_time, w_toll)
       on the unit simplex with step size Δ = 0.1 (66 unique weight vectors).
    2. Solve Dijkstra's BALANCED-mode shortest path for each weight vector.
    3. Deduplicate solutions by route node sequence (many weight vectors
       yield the same physical path).
    4. Apply non-dominated sorting: solution x dominates y iff x is
       ≤ y in all objectives and strictly < y in at least one.
    5. Return the non-dominated subset as the Pareto frontier.

Objectives (minimization):
    - Travel time (minutes)
    - Fuel cost ($)
    - Highway toll charges ($)
"""

import sys
import os
from typing import List, Dict, Any

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'python', 'src'))

from engine import load_dataset
from dijkstra import solve_dijkstra


def _is_dominated(candidate: Dict[str, Any], population: List[Dict[str, Any]]) -> bool:
    """Determines whether a candidate solution is Pareto-dominated by any member of the population.

    A solution x dominates candidate c iff:
      x[obj] ≤ c[obj]  for all objectives, and
      x[obj] <  c[obj]  for at least one objective.

    Args:
        candidate: Solution dictionary with ``travel_time_min``, ``fuel_cost``,
                   and ``toll_cost`` fields.
        population: All candidate solutions to compare against.

    Returns:
        True if at least one solution in population strictly dominates the candidate.
    """
    for other in population:
        if other is candidate:
            continue
        dominated_in_all = (
            other["travel_time_min"] <= candidate["travel_time_min"] and
            other["fuel_cost"]       <= candidate["fuel_cost"]       and
            other["toll_cost"]       <= candidate["toll_cost"]
        )
        strictly_better_in_one = (
            other["travel_time_min"] < candidate["travel_time_min"] or
            other["fuel_cost"]       < candidate["fuel_cost"]       or
            other["toll_cost"]       < candidate["toll_cost"]
        )
        if dominated_in_all and strictly_better_in_one:
            return True
    return False


def compute_pareto_frontier(
    graph,
    source: int,
    destination: int,
    steps: int = 10
) -> List[Dict[str, Any]]:
    """Computes the Pareto-optimal frontier over the three-objective weight simplex.

    Systematically enumerates weight combinations on the unit simplex
    (w_fuel + w_time + w_toll = 1.0) and solves the BALANCED shortest-path
    problem for each, then filters the non-dominated set.

    Args:
        graph: Constructed Graph instance.
        source: Origin vertex ID.
        destination: Destination vertex ID.
        steps: Discretization granularity of the weight simplex. A value of 10
               yields 66 unique weight triplets. Default is 10.

    Returns:
        List of non-dominated solution dictionaries, each containing:
          ``weights``, ``route``, ``distance_km``, ``travel_time_min``,
          ``fuel_liters``, ``fuel_cost``, ``toll_cost``, ``total_cost``.
    """
    solutions: List[Dict[str, Any]] = []

    for i in range(steps + 1):
        for j in range(steps + 1 - i):
            k = steps - i - j
            w_fuel  = i / steps
            w_time  = j / steps
            w_toll  = k / steps

            # Inject weight vector into config for this evaluation
            graph.config.fuel_cost_weight = w_fuel
            graph.config.time_weight      = w_time
            graph.config.toll_weight      = w_toll

            # Re-derive balanced costs with updated weights
            for edge in graph.edges:
                edge.cost_balanced = (
                    w_fuel * edge.fuel_cost +
                    w_time * edge.travel_time_min +
                    w_toll * edge.toll_cost
                )

            result = solve_dijkstra(graph, source, destination, "BALANCED")
            if result is None:
                continue

            solution = {
                "weights":          (round(w_fuel, 2), round(w_time, 2), round(w_toll, 2)),
                "route":            result.route,
                "distance_km":      result.distance_km,
                "travel_time_min":  result.travel_time_min,
                "fuel_liters":      result.fuel_liters,
                "fuel_cost":        result.fuel_cost,
                "toll_cost":        result.toll_cost,
                "total_cost":       result.total_cost,
            }

            # Deduplicate by route node sequence
            if not any(s["route"] == solution["route"] for s in solutions):
                solutions.append(solution)

    # Non-dominated sort: retain only Pareto-optimal alternatives
    pareto_frontier = [s for s in solutions if not _is_dominated(s, solutions)]
    return pareto_frontier


def main() -> None:
    """CLI entry point: runs the Pareto frontier analysis and prints results."""
    dataset_path = os.path.join(
        os.path.dirname(__file__), '..', 'datasets', 'small.json'
    )
    graph, _ = load_dataset(dataset_path)

    source      = 0
    destination = 500

    print("=" * 70)
    print("  MULTI-OBJECTIVE PARETO FRONTIER ANALYSIS")
    print(f"  Origin: Node {source}  →  Destination: Node {destination}")
    print("=" * 70)

    frontier = compute_pareto_frontier(graph, source, destination)

    print(f"  Discovered {len(frontier)} Non-Dominated Pareto-Optimal Route Alternatives:\n")
    for idx, sol in enumerate(frontier, start=1):
        wf, wt, wl = sol["weights"]
        print(
            f"  [{idx:02d}] Time: {sol['travel_time_min']:6.2f} min  |  "
            f"Fuel: ${sol['fuel_cost']:6.2f}  |  "
            f"Tolls: ${sol['toll_cost']:5.2f}  |  "
            f"Distance: {sol['distance_km']:6.2f} km"
        )
        print(
            f"       Weights — Fuel={wf:.1f}, Time={wt:.1f}, Toll={wl:.1f}  |  "
            f"Nodes in path: {len(sol['route'])}"
        )

    print("=" * 70)


if __name__ == "__main__":
    main()
