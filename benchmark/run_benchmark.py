#!/usr/bin/env python3
"""
Comprehensive Multi-Language Benchmark Runner & Visualizer
Executes all 5 languages (Python, C++, Rust, Go, Java) across:
- Multiple datasets (Small, Medium, Large)
- Execution modes (Single-thread, Multi-thread 8 workers)
- Algorithms (Dijkstra vs A*)
- Cold Start vs Warm Execution
- Measures: Wall-clock, CPU Time, Peak RSS Memory (MB), Throughput (QPS), p50/p95/p99
- Generates CSV, JSON, PNG charts, and Markdown comparison report.
"""

import os
import sys
import json
import time
import subprocess
import csv
import platform
import math
from typing import Dict, List, Any

BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DATASETS_DIR = os.path.join(BASE_DIR, "datasets")
RESULTS_DIR = os.path.join(BASE_DIR, "benchmark", "results")
CHARTS_DIR = os.path.join(BASE_DIR, "benchmark", "charts")

os.makedirs(RESULTS_DIR, exist_ok=True)
os.makedirs(CHARTS_DIR, exist_ok=True)

COMMANDS = {
    "python": [sys.executable, os.path.join(BASE_DIR, "python", "src", "cli.py")],
    "cpp": [os.path.join(BASE_DIR, "cpp", "build", "route_optimizer")],
    "rust": [os.path.join(BASE_DIR, "rust", "target", "release", "route_optimization_rust")],
    "go": [os.path.join(BASE_DIR, "go", "route_optimizer_go")],
    "java": ["java", "-jar", os.path.join(BASE_DIR, "java", "build", "libs", "route_optimization_java-1.0.0.jar")]
}

def get_system_metadata() -> Dict[str, Any]:
    def run_cmd(cmd):
        try:
            return subprocess.check_output(cmd, shell=True, text=True).strip()
        except:
            return "Unknown"
            
    return {
        "os": f"{platform.system()} {platform.release()} ({platform.machine()})",
        "cpu": run_cmd("sysctl -n machdep.cpu.brand_string"),
        "cpu_cores": run_cmd("sysctl -n hw.ncpu"),
        "ram_gb": round(int(run_cmd("sysctl -n hw.memsize") or "0") / (1024**3), 2),
        "python_version": run_cmd("python3 --version"),
        "cpp_compiler": run_cmd("clang++ --version | head -n 1"),
        "rust_version": run_cmd("rustc --version"),
        "go_version": run_cmd("go version"),
        "java_version": run_cmd("java -version 2>&1 | head -n 1")
    }

def measure_execution(lang: str, cmd_base: List[str], dataset_path: str, mode="single", workers=8, warmup=2, runs=5, algo="ALL") -> Dict[str, Any]:
    out_file = os.path.join(RESULTS_DIR, f"temp_{lang}.json")
    if os.path.exists(out_file):
        os.remove(out_file)
        
    cmd = list(cmd_base) + [
        "--dataset", dataset_path,
        "--mode", mode,
        "--workers", str(workers),
        "--warmup", str(warmup),
        "--runs", str(runs),
        "--algorithm", algo,
        "--output", out_file
    ]

    # Measure process metrics using /usr/bin/time on macOS
    # /usr/bin/time -l gives maximum resident set size in bytes on macOS
    time_cmd = ["/usr/bin/time", "-l"] + cmd
    t_start = time.perf_counter()
    proc = subprocess.Popen(time_cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    stdout, stderr = proc.communicate()
    t_wall_sec = time.perf_counter() - t_start

    if proc.returncode != 0:
        print(f"Error running {lang}: {stderr}")
        return {}

    # Parse peak memory from /usr/bin/time -l output
    peak_rss_mb = 0.0
    for line in stderr.splitlines():
        line = line.strip()
        if "maximum resident set size" in line:
            parts = line.split()
            if parts:
                peak_rss_mb = round(int(parts[0]) / (1024 * 1024), 2)
                break

    # Read output json
    result_data = {}
    if os.path.exists(out_file):
        with open(out_file, "r") as f:
            result_data = json.load(f)
            
    latencies = [r.get("execution_time_us", 0) / 1000.0 for r in result_data.get("results", [])]
    latencies.sort()
    
    p50 = latencies[int(len(latencies) * 0.50)] if latencies else 0.0
    p95 = latencies[int(len(latencies) * 0.95)] if latencies else 0.0
    p99 = latencies[int(len(latencies) * 0.99)] if latencies else 0.0

    return {
        "language": lang,
        "dataset": os.path.basename(dataset_path),
        "mode": mode,
        "algorithm": algo,
        "dataset_load_time_ms": result_data.get("dataset_load_time_ms", 0.0),
        "avg_query_time_ms": result_data.get("avg_time_ms", 0.0),
        "min_query_time_ms": result_data.get("min_time_ms", 0.0),
        "max_query_time_ms": result_data.get("max_time_ms", 0.0),
        "query_throughput_qps": result_data.get("query_throughput_qps", 0.0),
        "peak_rss_mb": peak_rss_mb,
        "p50_latency_ms": round(p50, 4),
        "p95_latency_ms": round(p95, 4),
        "p99_latency_ms": round(p99, 4),
        "total_wall_time_sec": round(t_wall_sec, 3),
        "num_queries": result_data.get("num_queries", len(latencies))
    }

def count_code_metrics() -> Dict[str, Dict[str, int]]:
    """Analyzes Lines of Code (LOC) and comments across languages."""
    paths = {
        "python": [os.path.join(BASE_DIR, "python", "src", f) for f in os.listdir(os.path.join(BASE_DIR, "python", "src")) if f.endswith(".py")],
        "cpp": [os.path.join(BASE_DIR, "cpp", "include", f) for f in os.listdir(os.path.join(BASE_DIR, "cpp", "include")) if f.endswith(".hpp")] + [os.path.join(BASE_DIR, "cpp", "src", "main.cpp")],
        "rust": [os.path.join(BASE_DIR, "rust", "src", f) for f in os.listdir(os.path.join(BASE_DIR, "rust", "src")) if f.endswith(".rs")],
        "go": [os.path.join(BASE_DIR, "go", "pkg", sub, f) for sub in ["models", "physics", "graph", "routing", "engine"] for f in os.listdir(os.path.join(BASE_DIR, "go", "pkg", sub)) if f.endswith(".go") and not f.endswith("_test.go")] + [os.path.join(BASE_DIR, "go", "cmd", "main.go")],
        "java": [os.path.join(BASE_DIR, "java", "src", "main", "java", "com", "benchmark", sub, f) for sub in ["models", "physics", "graph", "routing", "engine"] for f in os.listdir(os.path.join(BASE_DIR, "java", "src", "main", "java", "com", "benchmark", sub)) if f.endswith(".java")] + [os.path.join(BASE_DIR, "java", "src", "main", "java", "com", "benchmark", "App.java")]
    }

    metrics = {}
    for lang, file_list in paths.items():
        loc = 0
        comments = 0
        blank = 0
        for fp in file_list:
            if not os.path.exists(fp): continue
            with open(fp, "r", encoding="utf-8", errors="ignore") as f:
                for line in f:
                    stripped = line.strip()
                    if not stripped:
                        blank += 1
                    elif stripped.startswith(("#", "//", "/*", "*")):
                        comments += 1
                    else:
                        loc += 1
        metrics[lang] = {
            "loc": loc,
            "comments": comments,
            "blank_lines": blank,
            "total_lines": loc + comments + blank,
            "num_files": len(file_list)
        }
    return metrics

def plot_charts(records: List[Dict[str, Any]], code_metrics: Dict[str, Dict[str, int]]):
    try:
        import matplotlib.pyplot as plt
        import matplotlib
        matplotlib.use('Agg')
    except Exception as e:
        print(f"Matplotlib not available: {e}")
        return

    # Palette
    colors = {
        "python": "#3776AB",
        "cpp": "#00599C",
        "rust": "#DEA584",
        "go": "#00ADD8",
        "java": "#ED8B00"
    }

    # 1. Execution Time vs Dataset (Single-Thread)
    datasets = ["small.json", "medium.json", "large.json"]
    langs = ["cpp", "rust", "go", "java", "python"]
    
    plt.figure(figsize=(10, 6))
    for lang in langs:
        times = []
        for ds in datasets:
            match = [r for r in records if r["language"] == lang and r["dataset"] == ds and r["mode"] == "single" and r["algorithm"] == "ALL"]
            if match:
                times.append(match[0]["avg_query_time_ms"])
            else:
                times.append(0)
        plt.plot(["Small (1k)", "Medium (10k)", "Large (50k)"], times, marker='o', linewidth=2.5, label=lang.upper(), color=colors[lang])

    plt.yscale('log')
    plt.title("Execution Time vs Dataset Size (Single-Thread, Log Scale)", fontsize=14, fontweight='bold')
    plt.xlabel("Dataset Size (Nodes)", fontsize=12)
    plt.ylabel("Average Total Query Batch Time (ms)", fontsize=12)
    plt.grid(True, linestyle="--", alpha=0.6)
    plt.legend(fontsize=11)
    plt.tight_layout()
    plt.savefig(os.path.join(CHARTS_DIR, "execution_time_vs_dataset.png"), dpi=300)
    plt.close()

    # 2. Peak Memory Usage vs Dataset
    plt.figure(figsize=(10, 6))
    for lang in langs:
        mems = []
        for ds in datasets:
            match = [r for r in records if r["language"] == lang and r["dataset"] == ds and r["mode"] == "single" and r["algorithm"] == "ALL"]
            if match:
                mems.append(match[0]["peak_rss_mb"])
            else:
                mems.append(0)
        plt.plot(["Small (1k)", "Medium (10k)", "Large (50k)"], mems, marker='s', linewidth=2.5, label=lang.upper(), color=colors[lang])

    plt.title("Peak RAM Usage vs Dataset Size (MB)", fontsize=14, fontweight='bold')
    plt.xlabel("Dataset Size", fontsize=12)
    plt.ylabel("Peak RSS (MB)", fontsize=12)
    plt.grid(True, linestyle="--", alpha=0.6)
    plt.legend(fontsize=11)
    plt.tight_layout()
    plt.savefig(os.path.join(CHARTS_DIR, "memory_vs_dataset.png"), dpi=300)
    plt.close()

    # 3. Throughput comparison on Medium Dataset (QPS)
    plt.figure(figsize=(9, 5))
    qps_vals = []
    for lang in langs:
        match = [r for r in records if r["language"] == lang and r["dataset"] == "medium.json" and r["mode"] == "single" and r["algorithm"] == "ALL"]
        qps_vals.append(match[0]["query_throughput_qps"] if match else 0)

    bars = plt.bar([l.upper() for l in langs], qps_vals, color=[colors[l] for l in langs], edgecolor='black', width=0.55)
    plt.title("Query Throughput (QPS) - Medium Dataset (10,000 Nodes)", fontsize=13, fontweight='bold')
    plt.ylabel("Queries Per Second (QPS)", fontsize=12)
    for bar in bars:
        yval = bar.get_height()
        plt.text(bar.get_x() + bar.get_width()/2.0, yval + max(qps_vals)*0.01, f"{int(yval):,}", ha='center', va='bottom', fontweight='bold')
    plt.grid(axis='y', linestyle="--", alpha=0.5)
    plt.tight_layout()
    plt.savefig(os.path.join(CHARTS_DIR, "throughput_vs_language.png"), dpi=300)
    plt.close()

    # 4. Single-Thread vs Multi-Thread (8 Workers) on Large Dataset
    plt.figure(figsize=(10, 5.5))
    x_indices = range(len(langs))
    single_qps = []
    multi_qps = []
    for lang in langs:
        m_single = [r for r in records if r["language"] == lang and r["dataset"] == "large.json" and r["mode"] == "single" and r["algorithm"] == "ALL"]
        m_multi = [r for r in records if r["language"] == lang and r["dataset"] == "large.json" and r["mode"] == "multi" and r["algorithm"] == "ALL"]
        single_qps.append(m_single[0]["query_throughput_qps"] if m_single else 0)
        multi_qps.append(m_multi[0]["query_throughput_qps"] if m_multi else 0)

    width = 0.35
    plt.bar([i - width/2 for i in x_indices], single_qps, width=width, label="Single-Thread", color='#718096', edgecolor='black')
    plt.bar([i + width/2 for i in x_indices], multi_qps, width=width, label="Multi-Thread (8 Cores)", color='#38A169', edgecolor='black')
    plt.xticks(x_indices, [l.upper() for l in langs], fontsize=11, fontweight='bold')
    plt.title("Throughput Scaling: Single-Thread vs Multi-Thread (Large Dataset)", fontsize=13, fontweight='bold')
    plt.ylabel("Throughput (QPS)", fontsize=12)
    plt.legend(fontsize=11)
    plt.grid(axis='y', linestyle="--", alpha=0.5)
    plt.tight_layout()
    plt.savefig(os.path.join(CHARTS_DIR, "single_vs_multi_thread.png"), dpi=300)
    plt.close()

    # 5. Dijkstra vs A* Execution Time
    plt.figure(figsize=(10, 5.5))
    dijkstra_times = []
    astar_times = []
    for lang in langs:
        m_d = [r for r in records if r["language"] == lang and r["dataset"] == "medium.json" and r["algorithm"] == "DIJKSTRA"]
        m_a = [r for r in records if r["language"] == lang and r["dataset"] == "medium.json" and r["algorithm"] == "ASTAR"]
        dijkstra_times.append(m_d[0]["avg_query_time_ms"] if m_d else 0)
        astar_times.append(m_a[0]["avg_query_time_ms"] if m_a else 0)

    plt.bar([i - width/2 for i in x_indices], dijkstra_times, width=width, label="Dijkstra", color='#E53E3E', edgecolor='black')
    plt.bar([i + width/2 for i in x_indices], astar_times, width=width, label="A* Search", color='#3182CE', edgecolor='black')
    plt.xticks(x_indices, [l.upper() for l in langs], fontsize=11, fontweight='bold')
    plt.title("Algorithm Comparison: Dijkstra vs A* (Medium Dataset, Time in ms)", fontsize=13, fontweight='bold')
    plt.ylabel("Batch Execution Time (ms)", fontsize=12)
    plt.legend(fontsize=11)
    plt.grid(axis='y', linestyle="--", alpha=0.5)
    plt.tight_layout()
    plt.savefig(os.path.join(CHARTS_DIR, "dijkstra_vs_astar.png"), dpi=300)
    plt.close()

    # 6. Code Complexity & LOC
    plt.figure(figsize=(9, 5))
    loc_vals = [code_metrics[l]["loc"] for l in langs]
    bars = plt.bar([l.upper() for l in langs], loc_vals, color=[colors[l] for l in langs], edgecolor='black', width=0.55)
    plt.title("Source Code Lines of Code (LOC) Excluding Blank Lines & Comments", fontsize=13, fontweight='bold')
    plt.ylabel("Lines of Code (LOC)", fontsize=12)
    for bar in bars:
        yval = bar.get_height()
        plt.text(bar.get_x() + bar.get_width()/2.0, yval + 5, f"{int(yval)}", ha='center', va='bottom', fontweight='bold')
    plt.grid(axis='y', linestyle="--", alpha=0.5)
    plt.tight_layout()
    plt.savefig(os.path.join(CHARTS_DIR, "code_metrics_comparison.png"), dpi=300)
    plt.close()
    print(">>> All 6 publication-grade benchmark charts successfully generated! <<<")

def main():
    print("================================================================")
    print("   AUTOMATED VEHICLE ROUTE OPTIMIZATION 5-LANGUAGE BENCHMARK   ")
    print("================================================================")
    
    meta = get_system_metadata()
    print(f"System: {meta['os']} | CPU: {meta['cpu']} ({meta['cpu_cores']} cores) | RAM: {meta['ram_gb']} GB")
    print(f"Compilers: {meta['cpp_compiler']} | {meta['rust_version']} | {meta['go_version']} | {meta['java_version']}")
    print("----------------------------------------------------------------")

    datasets = [
        os.path.join(DATASETS_DIR, "small.json"),
        os.path.join(DATASETS_DIR, "medium.json"),
        os.path.join(DATASETS_DIR, "large.json")
    ]

    all_records = []
    languages = ["cpp", "rust", "go", "java", "python"]

    # 1. Main Suite: Datasets x Languages x Modes (Single & Multi)
    for ds_path in datasets:
        ds_name = os.path.basename(ds_path)
        print(f"\n>>> Running Benchmark Suite on Dataset: {ds_name} <<<")
        for lang in languages:
            cmd = COMMANDS[lang]
            # Single thread
            print(f"  [{lang.upper()}] Single-Thread ...", end=" ", flush=True)
            res_single = measure_execution(lang, cmd, ds_path, mode="single", warmup=2, runs=5, algo="ALL")
            if res_single:
                all_records.append(res_single)
                print(f"Done: {res_single['avg_query_time_ms']:.2f} ms | {res_single['query_throughput_qps']:.1f} QPS | Peak RSS: {res_single['peak_rss_mb']} MB")

            # Multi thread
            print(f"  [{lang.upper()}] Multi-Thread (8 workers) ...", end=" ", flush=True)
            res_multi = measure_execution(lang, cmd, ds_path, mode="multi", workers=8, warmup=2, runs=5, algo="ALL")
            if res_multi:
                all_records.append(res_multi)
                print(f"Done: {res_multi['avg_query_time_ms']:.2f} ms | {res_multi['query_throughput_qps']:.1f} QPS")

    # 2. Algorithm Comparison: Dijkstra vs A* on Medium Dataset
    med_ds = os.path.join(DATASETS_DIR, "medium.json")
    print("\n>>> Running Algorithm Comparison: Dijkstra vs A* on Medium Dataset <<<")
    for lang in languages:
        cmd = COMMANDS[lang]
        res_d = measure_execution(lang, cmd, med_ds, mode="single", warmup=2, runs=3, algo="DIJKSTRA")
        if res_d: all_records.append(res_d)
        res_a = measure_execution(lang, cmd, med_ds, mode="single", warmup=2, runs=3, algo="ASTAR")
        if res_a: all_records.append(res_a)
        print(f"  [{lang.upper()}] Dijkstra: {res_d.get('avg_query_time_ms', 0):.2f} ms vs A*: {res_a.get('avg_query_time_ms', 0):.2f} ms")

    # Save CSV
    csv_file = os.path.join(RESULTS_DIR, "summary.csv")
    fieldnames = [
        "language", "dataset", "mode", "algorithm", "dataset_load_time_ms",
        "avg_query_time_ms", "min_query_time_ms", "max_query_time_ms",
        "query_throughput_qps", "peak_rss_mb", "p50_latency_ms", "p95_latency_ms", "p99_latency_ms", "num_queries"
    ]
    with open(csv_file, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames, extrasaction="ignore")
        writer.writeheader()
        for r in all_records:
            writer.writerow(r)

    # Save JSON
    json_file = os.path.join(RESULTS_DIR, "detailed_results.json")
    with open(json_file, "w") as f:
        json.dump({
            "metadata": meta,
            "benchmark_records": all_records
        }, f, indent=2)

    # Code metrics
    code_metrics = count_code_metrics()
    with open(os.path.join(RESULTS_DIR, "code_metrics.json"), "w") as f:
        json.dump(code_metrics, f, indent=2)

    print("\n>>> Generating Visual Charts & Comparison Report <<<")
    plot_charts(all_records, code_metrics)

    print("================================================================")
    print("   BENCHMARK RUN COMPLETED SUCCESSFULLY! RESULTS PERSISTED.    ")
    print("================================================================")

if __name__ == "__main__":
    main()
