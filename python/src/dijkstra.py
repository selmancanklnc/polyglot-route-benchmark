"""Dijkstra's single-source shortest path algorithm for physical road networks.

Implements a binary min-heap (via Python's ``heapq`` module) variant of
Dijkstra's algorithm with lazy-deletion for stale priority queue entries.
Operates on pre-computed physical edge costs across three optimization modes:
FASTEST, CHEAPEST, and BALANCED.

Complexity: O((V + E) log V) time, O(V) space.
"""

import heapq
import time
from typing import Optional, Dict, Tuple, List

from models import RouteResult, Edge
from graph import Graph


def solve_dijkstra(graph: Graph, source: int, target: int, mode: str) -> Optional[RouteResult]:
    """Computes the cost-optimal route between two vertices using Dijkstra's algorithm.

    Executes a standard priority-queue-driven relaxation over the directed graph
    adjacency structure. Stale (superseded) heap entries are discarded efficiently
    via a lazy-deletion guard on popped cost versus recorded minimum.

    Path reconstruction proceeds via backtracking through the predecessor edge map,
    accumulating all physical metrics (distance, time, fuel, cost) along the route.

    Args:
        graph: Pre-built Graph instance with physics-evaluated edge costs.
        source: Integer ID of the origin vertex.
        target: Integer ID of the destination vertex.
        mode: Optimization objective. One of ``"FASTEST"``, ``"CHEAPEST"``, ``"BALANCED"``.

    Returns:
        A ``RouteResult`` containing the optimal path, aggregate physical metrics,
        and search statistics. Returns ``None`` if the target is unreachable from source.
    """
    start_time = time.perf_counter_ns()

    if source not in graph.nodes or target not in graph.nodes:
        return None

    if source == target:
        return RouteResult(
            source=source, destination=target, optimization=mode, algorithm="DIJKSTRA",
            route=[source], distance_km=0.0, travel_time_min=0.0, fuel_liters=0.0,
            fuel_cost=0.0, toll_cost=0.0, total_cost=0.0, nodes_explored=1,
            execution_time_us=int((time.perf_counter_ns() - start_time) / 1000)
        )

    # dist[v] = minimum accumulated cost to reach vertex v
    dist: Dict[int, float] = {}
    # prev_edge[v] = (predecessor_vertex, edge_traversed) for path reconstruction
    prev_edge: Dict[int, Tuple[int, Edge]] = {}

    # Priority queue entries: (accumulated_cost, vertex_id)
    pq = [(0.0, source)]
    dist[source] = 0.0
    nodes_explored = 0
    found = False

    while pq:
        d, u = heapq.heappop(pq)

        # Lazy deletion: discard entries superseded by a shorter path discovery
        if d > dist.get(u, float('inf')):
            continue

        nodes_explored += 1

        if u == target:
            found = True
            break

        for v, edge in graph.adj.get(u, []):
            weight = graph.get_edge_weight(edge, mode)
            new_dist = d + weight

            if new_dist < dist.get(v, float('inf')):
                dist[v] = new_dist
                prev_edge[v] = (u, edge)
                heapq.heappush(pq, (new_dist, v))

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
    tot_cost = dist[target]

    return RouteResult(
        source=source,
        destination=target,
        optimization=mode,
        algorithm="DIJKSTRA",
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
