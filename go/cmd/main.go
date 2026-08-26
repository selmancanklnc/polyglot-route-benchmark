package main

import (
	"encoding/json"
	"flag"
	"fmt"
	"math"
	"os"
	"path/filepath"
	"route_optimization_go/pkg/engine"
	"route_optimization_go/pkg/models"
	"time"
)

type BenchmarkOutput struct {
	Language           string               `json:"language"`
	Dataset            string               `json:"dataset"`
	Mode               string               `json:"mode"`
	Workers            int                  `json:"workers"`
	DatasetLoadTimeMs  float64              `json:"dataset_load_time_ms"`
	NumQueries         int                  `json:"num_queries"`
	Runs               int                  `json:"runs"`
	AvgTimeMs          float64              `json:"avg_time_ms"`
	MinTimeMs          float64              `json:"min_time_ms"`
	MaxTimeMs          float64              `json:"max_time_ms"`
	QueryThroughputQps float64              `json:"query_throughput_qps"`
	Results            []models.RouteResult `json:"results"`
}

func main() {
	datasetPath := flag.String("dataset", "", "Path to dataset JSON file")
	mode := flag.String("mode", "single", "Execution mode: single or multi")
	workers := flag.Int("workers", 8, "Number of parallel workers")
	warmup := flag.Int("warmup", 0, "Number of warmup runs")
	runs := flag.Int("runs", 1, "Number of benchmark iterations")
	outputPath := flag.String("output", "", "Optional output path for results JSON")
	algoOverride := flag.String("algorithm", "ALL", "Override query algorithm (DIJKSTRA, ASTAR, ALL)")

	flag.Parse()

	if *datasetPath == "" {
		fmt.Fprintf(os.Stderr, "Usage: route_optimizer_go --dataset <path> [options]\n")
		os.Exit(1)
	}

	tStartLoad := time.Now()
	graph, queries, err := engine.LoadDataset(*datasetPath)
	if err != nil {
		fmt.Fprintf(os.Stderr, "Failed to load dataset: %v\n", err)
		os.Exit(1)
	}
	loadTimeMs := float64(time.Since(tStartLoad).Microseconds()) / 1000.0

	if *algoOverride != "ALL" {
		for i := range queries {
			queries[i].Algorithm = *algoOverride
		}
	}

	// Warmup
	for w := 0; w < *warmup; w++ {
		if *mode == "single" {
			_ = engine.RunSequential(graph, queries)
		} else {
			_ = engine.RunParallel(graph, queries, *workers)
		}
	}

	// Benchmark runs
	runTimesMs := make([]float64, 0, *runs)
	var lastResults []models.RouteResult

	for r := 0; r < *runs; r++ {
		tStart := time.Now()
		if *mode == "single" {
			lastResults = engine.RunSequential(graph, queries)
		} else {
			lastResults = engine.RunParallel(graph, queries, *workers)
		}
		elapsedMs := float64(time.Since(tStart).Microseconds()) / 1000.0
		runTimesMs = append(runTimesMs, elapsedMs)
	}

	var sumTime float64
	minTime := math.Inf(1)
	maxTime := math.Inf(-1)
	for _, t := range runTimesMs {
		sumTime += t
		if t < minTime {
			minTime = t
		}
		if t > maxTime {
			maxTime = t
		}
	}
	avgTimeMs := sumTime / float64(len(runTimesMs))
	throughputQps := 0.0
	if avgTimeMs > 0 {
		throughputQps = (float64(len(queries)) * 1000.0) / avgTimeMs
	}

	if *outputPath != "" {
		out := BenchmarkOutput{
			Language:           "go",
			Dataset:            filepath.Base(*datasetPath),
			Mode:               *mode,
			Workers:            *workers,
			DatasetLoadTimeMs:  math.Round(loadTimeMs*1000.0) / 1000.0,
			NumQueries:         len(queries),
			Runs:               *runs,
			AvgTimeMs:          math.Round(avgTimeMs*1000.0) / 1000.0,
			MinTimeMs:          math.Round(minTime*1000.0) / 1000.0,
			MaxTimeMs:          math.Round(maxTime*1000.0) / 1000.0,
			QueryThroughputQps: math.Round(throughputQps*100.0) / 100.0,
			Results:            lastResults,
		}
		data, _ := json.MarshalIndent(out, "", "  ")
		_ = os.WriteFile(*outputPath, data, 0644)
	}

	fmt.Printf("{\"language\": \"go\", \"mode\": \"%s\", \"num_queries\": %d, \"avg_time_ms\": %.4f, \"query_throughput_qps\": %.2f}\n",
		*mode, len(queries), avgTimeMs, throughputQps)
}
