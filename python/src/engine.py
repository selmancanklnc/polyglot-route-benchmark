"""Benchmark execution engine for dataset loading and parallel query dispatch.

Provides dataset deserialization from the canonical JSON schema, query dispatching
to Dijkstra or A* search, and sequential / multi-threaded execution harnesses
used by both the CLI and the automated benchmark runner.

Note on Python parallelism:
    Due to the CPython Global Interpreter Lock (GIL), multi-threaded execution
    using ``ThreadPoolExecutor`` does not provide true CPU-bound parallelism.
    Threads yield effectively during I/O but contend during computation.
    This constraint is empirically verified in benchmark results and is an
    intentional part of the cross-language comparison study.
"""

import json
import time
from typing import List, Optional, Tuple
from concurrent.futures import ThreadPoolExecutor

from models import Node, Edge, Vehicle, Config, Query, RouteResult
from graph import Graph
from dijkstra import solve_dijkstra
from astar import solve_astar


def load_dataset(filepath: str) -> Tuple[Graph, List[Query]]:
    """Deserializes a benchmark JSON dataset into a Graph and query list.

    Parses vehicle constants, atmospheric configuration, spatial node geometry,
    directed road segment properties, and the query workload. Constructs a
    Graph instance, which triggers full physics derivation on all edges.

    Args:
        filepath: Absolute or relative path to the benchmark JSON dataset file.

    Returns:
        A tuple of (Graph, List[Query]) ready for route query execution.

    Raises:
        FileNotFoundError: If the specified dataset file does not exist.
        KeyError: If the JSON schema is missing required fields.
    """
    with open(filepath, 'r') as f:
        data = json.load(f)

    v_data = data["vehicle"]
    vehicle = Vehicle(
        vehicle_mass=float(v_data["vehicle_mass"]),
        drag_coefficient=float(v_data["drag_coefficient"]),
        frontal_area=float(v_data["frontal_area"]),
        rolling_resistance_coefficient=float(v_data["rolling_resistance_coefficient"]),
        engine_efficiency=float(v_data["engine_efficiency"]),
        fuel_energy_density=float(v_data["fuel_energy_density"]),
        fuel_price=float(v_data["fuel_price"])
    )

    c_data = data["configuration"]
    config = Config(
        air_density=float(c_data["air_density"]),
        gravity=float(c_data["gravity"]),
        fuel_cost_weight=float(c_data["fuel_cost_weight"]),
        time_weight=float(c_data["time_weight"]),
        toll_weight=float(c_data["toll_weight"]),
        road_speed_factors={k: float(v) for k, v in c_data["road_speed_factors"].items()}
    )

    nodes = [
        Node(id=int(n["id"]), x=float(n["x"]), y=float(n["y"]), elevation=float(n["elevation"]))
        for n in data["nodes"]
    ]

    edges = [
        Edge(
            id=int(e["id"]),
            source=int(e["source"]),
            target=int(e["target"]),
            distance=float(e["distance"]),
            speed_limit=float(e["speed_limit"]),
            average_speed=float(e["average_speed"]),
            road_type=str(e["road_type"]),
            slope=float(e["slope"]),
            elevation_difference=float(e["elevation_difference"]),
            traffic_factor=float(e["traffic_factor"]),
            wind_speed=float(e["wind_speed"]),
            wind_direction=float(e["wind_direction"]),
            toll_cost=float(e["toll_cost"])
        )
        for e in data["edges"]
    ]

    queries = [
        Query(
            id=int(q["id"]),
            source=int(q["source"]),
            destination=int(q["destination"]),
            optimization=str(q["optimization"]),
            algorithm=str(q.get("algorithm", "DIJKSTRA"))
        )
        for q in data["queries"]
    ]

    graph = Graph(nodes, edges, vehicle, config)
    return graph, queries


def execute_query(graph: Graph, query: Query) -> Optional[RouteResult]:
    """Dispatches a single route query to the appropriate pathfinding algorithm.

    Args:
        graph: The constructed Graph instance to search.
        query: Route query specification including source, destination,
               optimization mode, and algorithm selection.

    Returns:
        A ``RouteResult`` with optimal path and physical metrics,
        or ``None`` if no path exists.
    """
    if query.algorithm == "ASTAR":
        return solve_astar(graph, query.source, query.destination, query.optimization)
    else:
        return solve_dijkstra(graph, query.source, query.destination, query.optimization)


def run_sequential(graph: Graph, queries: List[Query]) -> List[RouteResult]:
    """Executes all queries serially in a single thread.

    Provides a deterministic, reproducible baseline measurement for
    single-threaded throughput benchmarking.

    Args:
        graph: The Graph instance to execute queries against.
        queries: Ordered list of route query specifications.

    Returns:
        List of successful RouteResult instances, preserving execution order.
    """
    results = []
    for q in queries:
        res = execute_query(graph, q)
        if res:
            results.append(res)
    return results


def run_parallel(graph: Graph, queries: List[Query], num_workers: int = 8) -> List[RouteResult]:
    """Executes all queries concurrently using a thread pool.

    Employs Python's ``ThreadPoolExecutor`` for concurrent dispatch. Note that
    due to the CPython GIL, this does not yield true CPU parallelism for
    compute-bound workloads. Empirical scaling results are provided in the
    benchmark report as part of the cross-language GIL impact analysis.

    Args:
        graph: The Graph instance to execute queries against (thread-safe for reads).
        queries: List of route query specifications to dispatch.
        num_workers: Number of worker threads in the pool. Default is 8.

    Returns:
        List of successful RouteResult instances (ordering not guaranteed).
    """
    with ThreadPoolExecutor(max_workers=num_workers) as executor:
        futures = [executor.submit(execute_query, graph, q) for q in queries]
        results = [f.result() for f in futures if f.result() is not None]
    return results
