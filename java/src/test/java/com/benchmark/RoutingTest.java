package com.benchmark;

import com.benchmark.engine.Engine;
import com.benchmark.models.Query;
import com.benchmark.models.RouteResult;
import com.benchmark.routing.AStar;
import com.benchmark.routing.Dijkstra;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

/**
 * Integration tests for the Dijkstra and A* routing pipeline.
 *
 * <p>Exercises the complete routing stack — dataset loading, graph construction,
 * physics evaluation, and pathfinding — using the deterministic 10-node, 26-edge
 * {@code test_fixture.json} dataset generated with seed=42. This ensures that
 * all assertions are reproducible and environment-independent.
 */
public class RoutingTest {

    private Engine.LoadedData data;

    /**
     * Loads the canonical test fixture graph and query set before each test.
     *
     * @throws Exception If the dataset file cannot be located or deserialized.
     */
    @BeforeEach
    public void setUp() throws Exception {
        data = Engine.loadDataset("../datasets/test_fixture.json");
    }

    /**
     * Validates that the test fixture contains exactly 10 nodes, 26 edges, and 6 queries.
     */
    @Test
    @DisplayName("Test fixture graph dimensions: 10 nodes, 26 edges, 6 queries")
    public void testFixtureGraphDimensions() {
        assertEquals(10, data.graph.nodes.size(),
            "Test fixture should contain exactly 10 nodes.");
        assertEquals(26, data.graph.edges.size(),
            "Test fixture should contain exactly 26 directed edges.");
        assertEquals(6, data.queries.size(),
            "Test fixture should contain exactly 6 query specifications.");
    }

    /**
     * Asserts that Dijkstra and A* produce identical optimal costs and path sequences.
     *
     * <p>Since both algorithms are guaranteed-optimal with an admissible heuristic,
     * any discrepancy in route or cost constitutes a correctness regression.
     */
    @Test
    @DisplayName("Dijkstra and A* produce identical cost and path on all fixture queries")
    public void testDijkstraAStarEquivalence() {
        for (Query q : data.queries) {
            RouteResult resD = Dijkstra.solve(data.graph, q.source, q.destination, q.optimization);
            RouteResult resA = AStar.solve(data.graph, q.source, q.destination, q.optimization);

            assertNotNull(resD, "Dijkstra returned null for query " + q.id);
            assertNotNull(resA, "A* returned null for query " + q.id);
            assertEquals(resD.totalCost, resA.totalCost, 1e-4,
                "Cost mismatch between Dijkstra and A* on query " + q.id);
            assertEquals(resD.route, resA.route,
                "Path identity mismatch between Dijkstra and A* on query " + q.id);
        }
    }

    /**
     * Ensures that routing to a non-existent node returns null gracefully.
     */
    @Test
    @DisplayName("Routing to non-existent node 9999 returns null")
    public void testUnreachableDestinationReturnsNull() {
        RouteResult res = Dijkstra.solve(data.graph, 0, 9999, "FASTEST");
        assertNull(res, "Routing to a non-existent node must return null.");
    }

    /**
     * Verifies zero-cost degenerate route when source equals destination.
     */
    @Test
    @DisplayName("Self-route (source == target) returns single-node path with zero cost")
    public void testTrivialSameSourceTarget() {
        RouteResult res = Dijkstra.solve(data.graph, 3, 3, "FASTEST");
        assertNotNull(res, "Self-route should always return a valid RouteResult.");
        assertEquals(1, res.route.size(),
            "Self-route path must contain only the origin node.");
        assertEquals(3, res.route.get(0),
            "Self-route path must contain the origin node identifier.");
        assertEquals(0.0, res.totalCost,
            "Self-route total cost must be exactly 0.0.");
    }
}
