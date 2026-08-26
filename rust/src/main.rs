use route_optimization_rust::engine::{load_dataset, run_parallel, run_sequential};
use route_optimization_rust::models::RouteResult;
use serde_json::json;
use std::env;
use std::fs::File;
use std::path::Path;
use std::time::Instant;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<String> = env::args().collect();
    let mut dataset_path = String::new();
    let mut mode = "single".to_string();
    let mut workers: usize = 8;
    let mut warmup: usize = 0;
    let mut runs: usize = 1;
    let mut output_path = String::new();
    let mut algo_override = "ALL".to_string();

    let mut i = 1;
    while i < args.len() {
        match args[i].as_str() {
            "--dataset" => {
                dataset_path = args[i + 1].clone();
                i += 2;
            }
            "--mode" => {
                mode = args[i + 1].clone();
                i += 2;
            }
            "--workers" => {
                workers = args[i + 1].parse()?;
                i += 2;
            }
            "--warmup" => {
                warmup = args[i + 1].parse()?;
                i += 2;
            }
            "--runs" => {
                runs = args[i + 1].parse()?;
                i += 2;
            }
            "--output" => {
                output_path = args[i + 1].clone();
                i += 2;
            }
            "--algorithm" => {
                algo_override = args[i + 1].clone();
                i += 2;
            }
            _ => {
                i += 1;
            }
        }
    }

    if dataset_path.is_empty() {
        eprintln!("Usage: route_optimization_rust --dataset <path> [--mode single|multi] [--workers N] [--warmup N] [--runs N] [--output <path>] [--algorithm DIJKSTRA|ASTAR|ALL]");
        std::process::exit(1);
    }

    let t_load_start = Instant::now();
    let (graph, mut queries) = load_dataset(&dataset_path)?;
    let load_time_ms = t_load_start.elapsed().as_secs_f64() * 1000.0;

    if algo_override != "ALL" {
        for q in queries.iter_mut() {
            q.algorithm = algo_override.clone();
        }
    }

    // Warmup
    for _ in 0..warmup {
        if mode == "single" {
            let _ = run_sequential(&graph, &queries);
        } else {
            let _ = run_parallel(&graph, &queries, workers);
        }
    }

    // Benchmark runs
    let mut run_times_ms = Vec::with_capacity(runs);
    let mut last_results: Vec<RouteResult> = Vec::new();

    for _ in 0..runs {
        let t_start = Instant::now();
        if mode == "single" {
            last_results = run_sequential(&graph, &queries);
        } else {
            last_results = run_parallel(&graph, &queries, workers);
        }
        let elapsed_ms = t_start.elapsed().as_secs_f64() * 1000.0;
        run_times_ms.push(elapsed_ms);
    }

    let avg_time_ms = run_times_ms.iter().sum::<f64>() / run_times_ms.len() as f64;
    let min_time_ms = run_times_ms.iter().cloned().fold(f64::INFINITY, f64::min);
    let max_time_ms = run_times_ms.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
    let throughput_qps = if avg_time_ms > 0.0 {
        (queries.len() as f64 * 1000.0) / avg_time_ms
    } else {
        0.0
    };

    if !output_path.is_empty() {
        let filename = Path::new(&dataset_path)
            .file_name()
            .unwrap_or_default()
            .to_string_lossy();
        let out_json = json!({
            "language": "rust",
            "dataset": filename,
            "mode": mode,
            "workers": if mode == "multi" { workers } else { 1 },
            "dataset_load_time_ms": (load_time_ms * 1000.0).round() / 1000.0,
            "num_queries": queries.len(),
            "runs": runs,
            "avg_time_ms": (avg_time_ms * 1000.0).round() / 1000.0,
            "min_time_ms": (min_time_ms * 1000.0).round() / 1000.0,
            "max_time_ms": (max_time_ms * 1000.0).round() / 1000.0,
            "query_throughput_qps": (throughput_qps * 100.0).round() / 100.0,
            "results": last_results
        });

        let file = File::create(&output_path)?;
        serde_json::to_writer_pretty(file, &out_json)?;
    }

    println!(
        "{{\"language\": \"rust\", \"mode\": \"{}\", \"num_queries\": {}, \"avg_time_ms\": {:.4}, \"query_throughput_qps\": {:.2}}}",
        mode,
        queries.len(),
        avg_time_ms,
        throughput_qps
    );

    Ok(())
}
