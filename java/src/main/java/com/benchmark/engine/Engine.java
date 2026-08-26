package com.benchmark.engine;

import com.benchmark.graph.Graph;
import com.benchmark.models.Dataset;
import com.benchmark.models.Query;
import com.benchmark.models.RouteResult;
import com.benchmark.routing.AStar;
import com.benchmark.routing.Dijkstra;
import com.fasterxml.jackson.databind.DeserializationFeature;
import com.fasterxml.jackson.databind.ObjectMapper;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.*;

/**
 * Benchmark execution engine providing dataset loading and parallel query dispatch.
 *
 * <p>Handles JSON dataset deserialization via Jackson, query routing to the appropriate
 * pathfinding algorithm, and sequential / multi-threaded execution strategies used
 * by both the CLI entry point and the automated benchmark harness.
 *
 * <p>Java's {@link ExecutorService} with a fixed-size thread pool provides genuine
 * CPU-level parallelism since the JVM does not impose a global interpreter lock,
 * enabling measurable multi-core scaling on compute-bound query workloads.
 */
public class Engine {

    /**
     * Shared Jackson {@link ObjectMapper} configured to tolerate unknown JSON fields
     * introduced by dataset format extensions.
     */
    private static final ObjectMapper mapper = new ObjectMapper()
            .configure(DeserializationFeature.FAIL_ON_UNKNOWN_PROPERTIES, false);

    /**
     * Container for the deserialized graph and associated query workload.
     */
    public static class LoadedData {
        /** Fully constructed graph with pre-computed physical edge costs. */
        public Graph graph;
        /** Query specifications to execute against this graph. */
        public List<Query> queries;

        /**
         * Constructs a LoadedData container.
         *
         * @param graph   Fully built Graph instance.
         * @param queries Query workload associated with this dataset.
         */
        public LoadedData(Graph graph, List<Query> queries) {
            this.graph   = graph;
            this.queries = queries;
        }
    }

    /**
     * Deserializes a benchmark JSON dataset and constructs the routing graph.
     *
     * <p>Reads vehicle constants, atmospheric configuration, node geometry, edge
     * segments, and the query workload from the canonical JSON schema. Constructs a
     * {@link Graph} instance, which triggers full physics derivation on all edges.
     *
     * @param filepath Absolute or relative path to the benchmark JSON dataset file.
     * @return A {@link LoadedData} instance with the constructed Graph and query list.
     * @throws IOException If the file cannot be read or the JSON is malformed.
     */
    public static LoadedData loadDataset(String filepath) throws IOException {
        Dataset ds = mapper.readValue(new File(filepath), Dataset.class);
        Graph g = new Graph(ds.nodes, ds.edges, ds.vehicle, ds.configuration);
        return new LoadedData(g, ds.queries);
    }

    /**
     * Dispatches a single route query to the appropriate pathfinding algorithm.
     *
     * @param graph Graph to search.
     * @param query Route query specifying source, destination, optimization, and algorithm.
     * @return A {@link RouteResult} with the optimal path and metrics,
     *         or {@code null} if no path exists.
     */
    public static RouteResult executeQuery(Graph graph, Query query) {
        if ("ASTAR".equals(query.algorithm)) {
            return AStar.solve(graph, query.source, query.destination, query.optimization);
        } else {
            return Dijkstra.solve(graph, query.source, query.destination, query.optimization);
        }
    }

    /**
     * Executes all queries serially in a single thread.
     *
     * <p>Provides a deterministic, reproducible baseline for single-threaded
     * throughput benchmarking. Results preserve query execution order.
     *
     * @param graph   Graph to search.
     * @param queries Ordered list of route query specifications.
     * @return List of non-null {@link RouteResult} instances in execution order.
     */
    public static List<RouteResult> runSequential(Graph graph, List<Query> queries) {
        List<RouteResult> results = new ArrayList<>(queries.size());
        for (Query q : queries) {
            RouteResult res = executeQuery(graph, q);
            if (res != null) {
                results.add(res);
            }
        }
        return results;
    }

    /**
     * Executes all queries concurrently using a fixed-size thread pool.
     *
     * <p>Unlike CPython's GIL-constrained ThreadPoolExecutor, Java's thread pool
     * achieves genuine CPU parallelism for compute-bound tasks, enabling measurable
     * multi-core throughput scaling (empirically verified in benchmark results).
     *
     * @param graph      Graph to search (thread-safe for concurrent reads).
     * @param queries    List of query specifications to dispatch.
     * @param numWorkers Number of threads in the executor pool.
     * @return List of successful {@link RouteResult} instances (order not guaranteed).
     * @throws RuntimeException wrapping {@link InterruptedException} or {@link ExecutionException}.
     */
    public static List<RouteResult> runParallel(Graph graph, List<Query> queries, int numWorkers) {
        ExecutorService executor = Executors.newFixedThreadPool(numWorkers);
        List<Future<RouteResult>> futures = new ArrayList<>(queries.size());

        for (Query q : queries) {
            futures.add(executor.submit(() -> executeQuery(graph, q)));
        }

        List<RouteResult> results = new ArrayList<>(queries.size());
        try {
            for (Future<RouteResult> f : futures) {
                RouteResult r = f.get();
                if (r != null) {
                    results.add(r);
                }
            }
        } catch (InterruptedException | ExecutionException e) {
            throw new RuntimeException("Parallel query execution failed.", e);
        } finally {
            executor.shutdown();
        }

        return results;
    }
}
