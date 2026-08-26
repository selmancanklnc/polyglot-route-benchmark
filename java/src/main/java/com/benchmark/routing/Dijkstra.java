package com.benchmark.routing;

import com.benchmark.graph.Graph;
import com.benchmark.models.Edge;
import com.benchmark.models.RouteResult;

import java.util.*;

/**
 * Dijkstra's single-source shortest path algorithm for multi-objective vehicle routing.
 *
 * <p>Implements a binary min-heap (via {@link PriorityQueue}) with lazy deletion for
 * stale priority queue entries. Operates on pre-computed physical edge costs across
 * three optimization modes: FASTEST, CHEAPEST, and BALANCED.
 *
 * <p>All state is local to the static {@link #solve} method invocation; the class
 * holds no mutable instance state and is therefore trivially thread-safe.
 */
public class Dijkstra {

    /**
     * Priority queue entry pairing a vertex identifier with its accumulated objective cost.
     *
     * <p>Natural ordering compares by cost ascending so that {@link PriorityQueue}
     * acts as a min-heap.
     */
    public static class DijkstraEntry implements Comparable<DijkstraEntry> {
        public double cost;
        public int node;

        public DijkstraEntry(double cost, int node) {
            this.cost = cost;
            this.node = node;
        }

        @Override
        public int compareTo(DijkstraEntry o) {
            return Double.compare(this.cost, o.cost);
        }
    }

    /**
     * Predecessor map entry used for optimal path backtracking.
     */
    public static class PrevEdge {
        public int prevNode;
        public int edgeIndex;

        public PrevEdge(int prevNode, int edgeIndex) {
            this.prevNode = prevNode;
            this.edgeIndex = edgeIndex;
        }
    }

    /**
     * Computes the cost-optimal route using Dijkstra's algorithm.
     *
     * <p>Executes priority-queue-driven edge relaxation over pre-computed physical costs.
     * Stale heap entries are discarded via lazy deletion. Path reconstruction backtracks
     * the predecessor map, accumulating all physical route metrics.
     *
     * @param graph  Pre-built Graph with physics-evaluated edge costs.
     * @param source Origin vertex identifier.
     * @param target Destination vertex identifier.
     * @param mode   Optimization objective: {@code "FASTEST"}, {@code "CHEAPEST"}, or {@code "BALANCED"}.
     * @return A {@link RouteResult} containing the optimal path and metrics,
     *         or {@code null} if the target is unreachable.
     */
    public static RouteResult solve(Graph graph, int source, int target, String mode) {
        long startTime = System.nanoTime();

        if (!graph.nodes.containsKey(source) || !graph.nodes.containsKey(target)) {
            return null;
        }

        if (source == target) {
            RouteResult res = new RouteResult();
            res.source = source;
            res.destination = target;
            res.optimization = mode;
            res.algorithm = "DIJKSTRA";
            res.route = Collections.singletonList(source);
            res.nodesExplored = 1;
            res.executionTimeUs = (System.nanoTime() - startTime) / 1000;
            return res;
        }

        Map<Integer, Double> dist = new HashMap<>();
        Map<Integer, PrevEdge> prev = new HashMap<>();
        PriorityQueue<DijkstraEntry> pq = new PriorityQueue<>();

        dist.put(source, 0.0);
        pq.add(new DijkstraEntry(0.0, source));

        int nodesExplored = 0;
        boolean found = false;

        while (!pq.isEmpty()) {
            DijkstraEntry top = pq.poll();
            int u = top.node;
            double cost = top.cost;

            Double currDist = dist.get(u);
            if (currDist != null && cost > currDist) {
                continue;
            }

            nodesExplored++;

            if (u == target) {
                found = true;
                break;
            }

            List<Graph.HalfEdge> neighbors = graph.adj.get(u);
            if (neighbors != null) {
                for (Graph.HalfEdge halfEdge : neighbors) {
                    int v = halfEdge.targetNode;
                    Edge edge = graph.edges.get(halfEdge.edgeIndex);
                    double weight = graph.getEdgeWeight(edge, mode);
                    double newDist = cost + weight;

                    Double currV = dist.get(v);
                    if (currV == null || newDist < currV) {
                        dist.put(v, newDist);
                        prev.put(v, new PrevEdge(u, halfEdge.edgeIndex));
                        pq.add(new DijkstraEntry(newDist, v));
                    }
                }
            }
        }

        long execUs = (System.nanoTime() - startTime) / 1000;

        if (!found) {
            return null;
        }

        // Backtrack predecessor map to reconstruct the optimal node sequence
        List<Integer> routeNodes = new ArrayList<>();
        List<Integer> routeEdges = new ArrayList<>();
        int curr = target;
        routeNodes.add(curr);

        while (curr != source) {
            PrevEdge p = prev.get(curr);
            routeEdges.add(p.edgeIndex);
            curr = p.prevNode;
            routeNodes.add(curr);
        }

        Collections.reverse(routeNodes);
        Collections.reverse(routeEdges);

        double totDist = 0.0;
        double totTime = 0.0;
        double totFuel = 0.0;
        double totFuelCost = 0.0;
        double totToll = 0.0;

        for (int edgeIdx : routeEdges) {
            Edge e = graph.edges.get(edgeIdx);
            totDist += e.distance;
            totTime += e.travelTimeMin;
            totFuel += e.fuelLiters;
            totFuelCost += e.fuelCost;
            totToll += e.tollCost;
        }

        RouteResult res = new RouteResult();
        res.source = source;
        res.destination = target;
        res.optimization = mode;
        res.algorithm = "DIJKSTRA";
        res.route = routeNodes;
        res.distanceKm = Math.round(totDist * 10000.0) / 10000.0;
        res.travelTimeMin = Math.round(totTime * 10000.0) / 10000.0;
        res.fuelLiters = Math.round(totFuel * 10000.0) / 10000.0;
        res.fuelCost = Math.round(totFuelCost * 10000.0) / 10000.0;
        res.tollCost = Math.round(totToll * 10000.0) / 10000.0;
        res.totalCost = Math.round(dist.get(target) * 10000.0) / 10000.0;
        res.nodesExplored = nodesExplored;
        res.executionTimeUs = execUs;

        return res;
    }
}
