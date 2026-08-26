#!/usr/bin/env python3
"""Command-line interface for the Python route optimization benchmark runner.

Accepts a dataset path, execution mode (single/multi-threaded), warmup iterations,
repetition count, and optional output path. Emits structured JSON benchmark results
to stdout, and optionally persists full route-level detail to a file.

Usage:
    python3 cli.py --dataset datasets/medium.json --mode single --runs 3
    python3 cli.py --dataset datasets/large.json --mode multi --workers 8 --runs 5 --output out.json
"""

import argparse
import json
import os
import sys
import time

from engine import load_dataset, run_sequential, run_parallel


def _build_argument_parser() -> argparse.ArgumentParser:
    """Constructs and returns the CLI argument parser with full option documentation."""
    parser = argparse.ArgumentParser(
        prog="route_benchmark_python",
        description=(
            "Vehicle Route Optimization Benchmark — Python CPython 3.x implementation.\n"
            "Evaluates Dijkstra and A* performance over physics-modeled road networks."
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument(
        "--dataset", required=True,
        help="Path to the input JSON benchmark dataset file."
    )
    parser.add_argument(
        "--mode", default="single", choices=["single", "multi"],
        help="Execution concurrency model: 'single' (serial) or 'multi' (thread pool)."
    )
    parser.add_argument(
        "--workers", type=int, default=8,
        help="Number of worker threads in multi-threaded mode (default: 8)."
    )
    parser.add_argument(
        "--warmup", type=int, default=0,
        help="Number of warmup query passes before timed measurement (default: 0)."
    )
    parser.add_argument(
        "--runs", type=int, default=1,
        help="Number of timed benchmark repetitions to average (default: 1)."
    )
    parser.add_argument(
        "--output",
        help="Optional path to write full route-level JSON results."
    )
    parser.add_argument(
        "--algorithm", choices=["DIJKSTRA", "ASTAR", "ALL"], default="ALL",
        help="Override all query algorithm selections (default: ALL — use per-query setting)."
    )
    return parser


def main() -> None:
    """Entry point for the benchmark CLI.

    Orchestrates dataset loading, optional warmup passes, timed benchmark
    repetitions, and JSON result serialization. Outputs a compact summary
    to stdout for programmatic ingestion by the benchmark harness.
    """
    parser = _build_argument_parser()
    args = parser.parse_args()

    # Phase 1: Dataset load with wall-clock timing
    t_load_start = time.perf_counter()
    graph, queries = load_dataset(args.dataset)
    t_load_end = time.perf_counter()
    load_time_ms = (t_load_end - t_load_start) * 1000.0

    if args.algorithm != "ALL":
        for q in queries:
            q.algorithm = args.algorithm

    # Phase 2: Optional warmup passes (not timed)
    for _ in range(args.warmup):
        if args.mode == "single":
            run_sequential(graph, queries)
        else:
            run_parallel(graph, queries, args.workers)

    # Phase 3: Timed benchmark repetitions
    run_times_ms = []
    last_results = []

    for _ in range(args.runs):
        t_start = time.perf_counter()
        if args.mode == "single":
            results = run_sequential(graph, queries)
        else:
            results = run_parallel(graph, queries, args.workers)
        t_end = time.perf_counter()
        run_times_ms.append((t_end - t_start) * 1000.0)
        last_results = results

    avg_time_ms = sum(run_times_ms) / len(run_times_ms)
    min_time_ms = min(run_times_ms)
    max_time_ms = max(run_times_ms)
    qps = round((len(queries) * 1000.0) / avg_time_ms, 2) if avg_time_ms > 0 else 0.0

    # Phase 4: Serialization
    serialized_routes = [
        {
            "source": r.source,
            "destination": r.destination,
            "optimization": r.optimization,
            "algorithm": r.algorithm,
            "route": r.route,
            "distance_km": r.distance_km,
            "travel_time_min": r.travel_time_min,
            "fuel_liters": r.fuel_liters,
            "fuel_cost": r.fuel_cost,
            "toll_cost": r.toll_cost,
            "total_cost": r.total_cost,
            "nodes_explored": r.nodes_explored,
            "execution_time_us": r.execution_time_us
        }
        for r in last_results
    ]

    output_data = {
        "language": "python",
        "dataset": os.path.basename(args.dataset),
        "mode": args.mode,
        "workers": args.workers if args.mode == "multi" else 1,
        "dataset_load_time_ms": round(load_time_ms, 3),
        "num_queries": len(queries),
        "runs": args.runs,
        "run_times_ms": [round(t, 3) for t in run_times_ms],
        "avg_time_ms": round(avg_time_ms, 3),
        "min_time_ms": round(min_time_ms, 3),
        "max_time_ms": round(max_time_ms, 3),
        "query_throughput_qps": qps,
        "results": serialized_routes
    }

    if args.output:
        with open(args.output, "w") as f:
            json.dump(output_data, f, indent=2)

    # Compact summary emitted to stdout for harness consumption
    print(json.dumps({
        "language": "python",
        "mode": args.mode,
        "num_queries": len(queries),
        "avg_time_ms": round(avg_time_ms, 3),
        "query_throughput_qps": qps
    }))


if __name__ == "__main__":
    main()
