package com.benchmark.graph;

import com.benchmark.models.Config;
import com.benchmark.models.Edge;
import com.benchmark.models.Node;
import com.benchmark.models.Vehicle;
import com.benchmark.physics.Physics;

import java.util.*;

/**
 * Weighted directed road network graph with pre-computed physical edge costs.
 *
 * <p>Constructs an adjacency-list representation over spatial vertices and directed
 * road segments. All edge physical metrics (aerodynamic drag, slope resistance, rolling
 * friction, fuel consumption, and per-mode costs) are evaluated once at construction
 * time via {@link Physics#computeEdgeMetrics}, rendering the graph read-only and safe
 * for concurrent access from multiple threads.
 *
 * <p>Adjacency entries are stored as {@link HalfEdge} instances containing a target
 * node ID and an index into the edge list, avoiding edge data duplication.
 */
public class Graph {

    /**
     * Compact outgoing edge entry in the adjacency list.
     *
     * <p>Stores a target vertex identifier and an index into the graph's edge list
     * to avoid memory duplication during adjacency construction.
     */
    public static class HalfEdge {
        /** Identifier of the destination vertex. */
        public int targetNode;
        /** Index of this edge in the parent graph's {@code edges} list. */
        public int edgeIndex;

        /**
         * Constructs a HalfEdge with the specified target and edge index.
         *
         * @param targetNode Destination vertex identifier.
         * @param edgeIndex  Index of the corresponding full Edge in the graph's edge list.
         */
        public HalfEdge(int targetNode, int edgeIndex) {
            this.targetNode = targetNode;
            this.edgeIndex = edgeIndex;
        }
    }

    /** Maps vertex identifier to its spatial Node geometry. */
    public Map<Integer, Node> nodes;
    /** Ordered list of all directed road segments with synthesized cost fields. */
    public List<Edge> edges;
    /** Vehicular physical constants for this benchmark run. */
    public Vehicle vehicle;
    /** Atmospheric and optimization weight configuration parameters. */
    public Config config;
    /** Forward adjacency index: {@code node_id -> List<HalfEdge>}. */
    public Map<Integer, List<HalfEdge>> adj;

    /**
     * Constructs the graph, executes full physics derivation on all edges,
     * and builds the forward adjacency index.
     *
     * @param nodesList Complete list of spatial graph vertices.
     * @param edgesList Complete list of directed road segments (mutated in-place).
     * @param vehicle   Vehicular aerodynamic and thermodynamic constants.
     * @param config    Atmospheric parameters and multi-objective weight coefficients.
     */
    public Graph(List<Node> nodesList, List<Edge> edgesList, Vehicle vehicle, Config config) {
        this.vehicle = vehicle;
        this.config  = config;
        this.edges   = edgesList;
        this.nodes   = new HashMap<>(nodesList.size());
        this.adj     = new HashMap<>(nodesList.size());

        for (Node n : nodesList) {
            this.nodes.put(n.id, n);
            this.adj.put(n.id, new ArrayList<>());
        }

        for (int i = 0; i < edgesList.size(); i++) {
            Edge e = edgesList.get(i);
            Physics.computeEdgeMetrics(e, vehicle, config);
            List<HalfEdge> neighbors = this.adj.get(e.source);
            if (neighbors != null) {
                neighbors.add(new HalfEdge(e.target, i));
            }
        }
    }

    /**
     * Returns the pre-computed objective cost for the specified optimization mode.
     *
     * @param edge Road segment whose cost is to be selected.
     * @param mode Optimization strategy: {@code "FASTEST"}, {@code "CHEAPEST"}, or {@code "BALANCED"}.
     * @return Scalar edge traversal cost for priority queue relaxation.
     */
    public double getEdgeWeight(Edge edge, String mode) {
        if ("FASTEST".equals(mode)) {
            return edge.costFastest;
        } else if ("CHEAPEST".equals(mode)) {
            return edge.costCheapest;
        } else {
            return edge.costBalanced;
        }
    }
}
