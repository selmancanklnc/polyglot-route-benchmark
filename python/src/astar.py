"""A* informed search algorithm for heuristic-guided vehicle route optimization.

Implements the A* graph search algorithm with an admissible, mode-aware Euclidean
distance heuristic. A* is guaranteed to find an optimal solution when the heuristic
is consistent (monotone), which the Euclidean lower-bound on travel time satisfies
in this domain.

Complexity: O((V + E) log V) time in the best case with an effective heuristic,
with measured empirical improvements over Dijkstra on sparse queries with
well-separated source and target nodes.
"""

import heapq
import time
import math
from typing import Optional, Dict, Tuple, List

from models import RouteResult, Edge, Node
from graph import Graph


def _heuristic(u_node: Node, target_node: Node, mode: str, config) -> float:
    """Computes an admissible lower-bound heuristic from vertex u to the target.

    Uses Euclidean straight-line distance as the geometric lower bound on path
    length, then converts to the appropriate objective cost unit:
    - FASTEST: Minimum achievable travel time (assuming unconstrained speed of 140 km/h).
    - BALANCED: Time component multiplied by its objective weight coefficient.
    - CHEAPEST: Zero (Euclidean lower-bound on fuel cost is not reliably constructible
      without road-type knowledge; returns 0 to guarantee admissibility).

    Args:
        u_node: Current vertex spatial coordinates.
        target_node: Destination vertex spatial coordinates.
        mode: Optimization mode determining heuristic unit.
        config: Configuration object providing optimization weight coefficients.

    Returns:
        Non-negative lower bound on the cost from u_node to target_node.
    """
    dx = target_node.x - u_node.x
    dy = target_node.y - u_node.y
    dist_km = math.sqrt(dx * dx + dy * dy)

    # Physical lower bound: maximum legal effective speed is 140.0 km/h
    min_time_min = (dist_km / 140.0) * 60.0

    if mode == "FASTEST":
        return min_time_min
    elif mode == "BALANCED":
        return config.time_weight * min_time_min
    else:  # CHEAPEST
        return 0.0


def solve_astar(graph: Graph, source: int, target: int, mode: str) -> Optional[RouteResult]:
    """Computes the cost-optimal route between two vertices using A* graph search.

    Maintains separate g-score (actual cost from source) and f-score (g + heuristic)
    structures. The priority queue is ordered by f-score, directing search expansion
    toward geometrically promising vertex neighborhoods and reducing the number of
    vertices that must be settled compared to uninformed Dijkstra.

    Path reconstruction proceeds identically to Dijkstra via backtracking the
    predecessor edge map.

    Args:
        graph: Pre-built Graph instance with physics-evaluated edge costs.
        source: Integer ID of the origin vertex.
        target: Integer ID of the destination vertex.
        mode: Optimization objective. One of ``"FASTEST"``, ``"CHEAPEST"``, ``"BALANCED"``.

    Returns:
        A ``RouteResult`` containing the optimal path, aggregate physical metrics,
        and search statistics. Returns ``None`` if the target is unreachable.
    """
    start_time = time.perf_counter_ns()

    if source not in graph.nodes or target not in graph.nodes:
        return None

    target_node = graph.nodes[target]

    if source == target:
        return RouteResult(
            source=source, destination=target, optimization=mode, algorithm="ASTAR",
            route=[source], distance_km=0.0, travel_time_min=0.0, fuel_liters=0.0,
            fuel_cost=0.0, toll_cost=0.0, total_cost=0.0, nodes_explored=1,
            execution_time_us=int((time.perf_counter_ns() - start_time) / 1000)
        )

    # g_score[v] = best confirmed cost from source to vertex v
    g_score: Dict[int, float] = {source: 0.0}
    # prev_edge[v] = (predecessor_vertex, edge_traversed)
    prev_edge: Dict[int, Tuple[int, Edge]] = {}

    # Priority queue entries: (f_score, g_score, vertex_id)
    h0 = _heuristic(graph.nodes[source], target_node, mode, graph.config)
    pq = [(h0, 0.0, source)]
    nodes_explored = 0
    found = False

    while pq:
        f, g, u = heapq.heappop(pq)

        # Lazy deletion: skip entries that were superseded by better paths
        if g > g_score.get(u, float('inf')):
            continue

        nodes_explored += 1

        if u == target:
            found = True
            break

        u_node = graph.nodes[u]

        for v, edge in graph.adj.get(u, []):
            weight = graph.get_edge_weight(edge, mode)
            tentative_g = g + weight

            if tentative_g < g_score.get(v, float('inf')):
                g_score[v] = tentative_g
                prev_edge[v] = (u, edge)
                h = _heuristic(graph.nodes[v], target_node, mode, graph.config)
                f_score = tentative_g + h
                heapq.heappush(pq, (f_score, tentative_g, v))

    end_time = time.perf_counter_ns()
    exec_us = int((end_time - start_time) / 1000)

    if not found:
        return None

    # Backtrack predecessor map to reconstruct the optimal path
    curr = target
    route_nodes = [curr]
    route_edges: List[Edge] = []

    while curr != source:
        p, edge = prev_edge[curr]
        route_edges.append(edge)
        curr = p
        route_nodes.append(curr)

    route_nodes.reverse()
    route_edges.reverse()

    tot_distance = sum(e.distance for e in route_edges)
    tot_time = sum(e.travel_time_min for e in route_edges)
    tot_fuel = sum(e.fuel_liters for e in route_edges)
    tot_fuel_cost = sum(e.fuel_cost for e in route_edges)
    tot_toll = sum(e.toll_cost for e in route_edges)
    tot_cost = g_score[target]

    return RouteResult(
        source=source,
        destination=target,
        optimization=mode,
        algorithm="ASTAR",
        route=route_nodes,
        distance_km=round(tot_distance, 4),
        travel_time_min=round(tot_time, 4),
        fuel_liters=round(tot_fuel, 4),
        fuel_cost=round(tot_fuel_cost, 4),
        toll_cost=round(tot_toll, 4),
        total_cost=round(tot_cost, 4),
        nodes_explored=nodes_explored,
        execution_time_us=exec_us
    )
