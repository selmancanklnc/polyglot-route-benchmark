package com.benchmark;

import com.benchmark.engine.Engine;
import com.benchmark.models.Query;
import com.benchmark.models.RouteResult;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.SerializationFeature;

import java.io.File;
import java.util.*;

public class App {
    public static void main(String[] args) throws Exception {
        String datasetPath = null;
        String mode = "single";
        int workers = 8;
        int warmup = 0;
        int runs = 1;
        String outputPath = null;
        String algoOverride = "ALL";

        for (int i = 0; i < args.length; i++) {
            if ("--dataset".equals(args[i]) && i + 1 < args.length) datasetPath = args[++i];
            else if ("--mode".equals(args[i]) && i + 1 < args.length) mode = args[++i];
            else if ("--workers".equals(args[i]) && i + 1 < args.length) workers = Integer.parseInt(args[++i]);
            else if ("--warmup".equals(args[i]) && i + 1 < args.length) warmup = Integer.parseInt(args[++i]);
            else if ("--runs".equals(args[i]) && i + 1 < args.length) runs = Integer.parseInt(args[++i]);
            else if ("--output".equals(args[i]) && i + 1 < args.length) outputPath = args[++i];
            else if ("--algorithm".equals(args[i]) && i + 1 < args.length) algoOverride = args[++i];
        }

        if (datasetPath == null) {
            System.err.println("Usage: App --dataset <path> [--mode single|multi] [--workers N] [--warmup N] [--runs N] [--output <path>] [--algorithm DIJKSTRA|ASTAR|ALL]");
            System.exit(1);
        }

        long tLoadStart = System.nanoTime();
        Engine.LoadedData data = Engine.loadDataset(datasetPath);
        long tLoadEnd = System.nanoTime();
        double loadTimeMs = (tLoadEnd - tLoadStart) / 1_000_000.0;

        if (!"ALL".equals(algoOverride)) {
            for (Query q : data.queries) {
                q.algorithm = algoOverride;
            }
        }

        // Warmup
        for (int w = 0; w < warmup; w++) {
            if ("single".equals(mode)) {
                Engine.runSequential(data.graph, data.queries);
            } else {
                Engine.runParallel(data.graph, data.queries, workers);
            }
        }

        // Benchmark runs
        List<Double> runTimesMs = new ArrayList<>(runs);
        List<RouteResult> lastResults = null;

        for (int r = 0; r < runs; r++) {
            long tStart = System.nanoTime();
            if ("single".equals(mode)) {
                lastResults = Engine.runSequential(data.graph, data.queries);
            } else {
                lastResults = Engine.runParallel(data.graph, data.queries, workers);
            }
            long tEnd = System.nanoTime();
            double elapsedMs = (tEnd - tStart) / 1_000_000.0;
            runTimesMs.add(elapsedMs);
        }

        double sumTime = 0.0;
        double minTime = Double.MAX_VALUE;
        double maxTime = Double.MIN_VALUE;
        for (double t : runTimesMs) {
            sumTime += t;
            if (t < minTime) minTime = t;
            if (t > maxTime) maxTime = t;
        }
        double avgTimeMs = sumTime / runTimesMs.size();
        double throughputQps = avgTimeMs > 0.0 ? (data.queries.size() * 1000.0) / avgTimeMs : 0.0;

        if (outputPath != null) {
            Map<String, Object> outMap = new LinkedHashMap<>();
            outMap.put("language", "java");
            outMap.put("dataset", new File(datasetPath).getName());
            outMap.put("mode", mode);
            outMap.put("workers", "multi".equals(mode) ? workers : 1);
            outMap.put("dataset_load_time_ms", Math.round(loadTimeMs * 1000.0) / 1000.0);
            outMap.put("num_queries", data.queries.size());
            outMap.put("runs", runs);
            outMap.put("avg_time_ms", Math.round(avgTimeMs * 1000.0) / 1000.0);
            outMap.put("min_time_ms", Math.round(minTime * 1000.0) / 1000.0);
            outMap.put("max_time_ms", Math.round(maxTime * 1000.0) / 1000.0);
            outMap.put("query_throughput_qps", Math.round(throughputQps * 100.0) / 100.0);
            outMap.put("results", lastResults);

            ObjectMapper mapper = new ObjectMapper().enable(SerializationFeature.INDENT_OUTPUT);
            mapper.writeValue(new File(outputPath), outMap);
        }

        System.out.printf(Locale.US, "{\"language\": \"java\", \"mode\": \"%s\", \"num_queries\": %d, \"avg_time_ms\": %.4f, \"query_throughput_qps\": %.2f}%n",
                mode, data.queries.size(), avgTimeMs, throughputQps);
    }
}
