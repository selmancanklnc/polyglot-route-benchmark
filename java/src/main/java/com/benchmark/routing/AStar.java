package com.benchmark.routing;

import com.benchmark.graph.Graph;
import com.benchmark.models.Config;
import com.benchmark.models.Edge;
import com.benchmark.models.Node;
import com.benchmark.models.RouteResult;

import java.util.*;

/**
 * A* informed search algorithm for heuristic-guided vehicle route optimization.
 *
 * <p>Implements A* with an admissible Euclidean distance heuristic scaled to each
 * optimization mode. Since the heuristic is consistent (monotone), A* is guaranteed
 * to find the cost-optimal path while typically expanding fewer vertices than
 * uninformed Dijkstra on geometrically well-separated queries.
 *
 * <p>All state is local to the static {@link #solve} invocation; the class is
 * thread-safe with no mutable instance state.
 */
public class AStar {

    /**
     * Priority queue entry pairing f-score and g-score with a vertex identifier.
     *
     * <p>Natural ordering compares by f-score ascending, directing search expansion
     * towards geometrically promising vertex neighborhoods.
     */
    public static class AStarEntry implements Comparable<AStarEntry> {
        public double fScore;
        public double gScore;
        public int node;

        public AStarEntry(double fScore, double gScore, int node) {
            this.fScore = fScore;
            this.gScore = gScore;
            this.node = node;
        }

        @Override
        public int compareTo(AStarEntry o) {
            return Double.compare(this.fScore, o.fScore);
        }
    }

    /**
     * Computes an admissible lower-bound heuristic from vertex {@code uNode} to the target.
     *
     * <p>Uses Euclidean straight-line distance as the geometric lower bound, converted
     * to the appropriate objective cost unit per mode:
     * <ul>
     *   <li>FASTEST — minimum achievable time at unconstrained 140 km/h</li>
     *   <li>BALANCED — time component multiplied by its weight coefficient</li>
     *   <li>CHEAPEST — zero (Euclidean fuel lower bound is not constructible without road data)</li>
     * </ul>
     *
     * @param uNode      Current vertex spatial geometry.
     * @param targetNode Destination vertex spatial geometry.
     * @param mode       Optimization mode determining heuristic unit.
     * @param config     Configuration providing objective weight coefficients.
     * @return Non-negative admissible lower bound from uNode to target.
     */
    public static double heuristic(Node uNode, Node targetNode, String mode, Config config) {
        double dx = targetNode.x - uNode.x;
        double dy = targetNode.y - uNode.y;
        double distKm = Math.sqrt(dx * dx + dy * dy);
        double minTimeMin = (distKm / 140.0) * 60.0;

        if ("FASTEST".equals(mode)) {
            return minTimeMin;
        } else if ("BALANCED".equals(mode)) {
            return config.timeWeight * minTimeMin;
        } else {
            return 0.0;
        }
    }

    /**
     * Computes the cost-optimal route between two vertices using A* search.
     *
     * <p>Maintains g-score (actual cost from source) and f-score (g + heuristic) structures.
     * Priority queue ordering by f-score guides expansion toward geometrically promising
     * regions, reducing settled vertices compared to uninformed Dijkstra.
     *
     * @param graph  Pre-built Graph with physics-evaluated edge costs.
     * @param source Origin vertex identifier.
     * @param target Destination vertex identifier.
     * @param mode   Optimization objective: {@code "FASTEST"}, {@code "CHEAPEST"}, or {@code "BALANCED"}.
     * @return A {@link RouteResult} with path and physical metrics,
     *         or {@code null} if the target is unreachable.
     */
    public static RouteResult solve(Graph graph, int source, int target, String mode) {
        long startTime = System.nanoTime();

        Node targetNode = graph.nodes.get(target);
        Node sourceNode = graph.nodes.get(source);

        if (targetNode == null || sourceNode == null) {
            return null;
        }

        if (source == target) {
            RouteResult res = new RouteResult();
            res.source = source;
            res.destination = target;
            res.optimization = mode;
            res.algorithm = "ASTAR";
            res.route = Collections.singletonList(source);
            res.nodesExplored = 1;
            res.executionTimeUs = (System.nanoTime() - startTime) / 1000;
            return res;
        }

        Map<Integer, Double> gScore = new HashMap<>();
        Map<Integer, Dijkstra.PrevEdge> prev = new HashMap<>();
        PriorityQueue<AStarEntry> pq = new PriorityQueue<>();

        double h0 = heuristic(sourceNode, targetNode, mode, graph.config);
        gScore.put(source, 0.0);
        pq.add(new AStarEntry(h0, 0.0, source));

        int nodesExplored = 0;
        boolean found = false;

        while (!pq.isEmpty()) {
            AStarEntry top = pq.poll();
            int u = top.node;
            double gVal = top.gScore;

            Double currG = gScore.get(u);
            if (currG != null && gVal > currG) {
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
                    double tentativeG = gVal + weight;

                    Double currV = gScore.get(v);
                    if (currV == null || tentativeG < currV) {
                        gScore.put(v, tentativeG);
                        prev.put(v, new Dijkstra.PrevEdge(u, halfEdge.edgeIndex));
                        Node vNode = graph.nodes.get(v);
                        double h = heuristic(vNode, targetNode, mode, graph.config);
                        pq.add(new AStarEntry(tentativeG + h, tentativeG, v));
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
            Dijkstra.PrevEdge p = prev.get(curr);
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
        res.algorithm = "ASTAR";
        res.route = routeNodes;
        res.distanceKm = Math.round(totDist * 10000.0) / 10000.0;
        res.travelTimeMin = Math.round(totTime * 10000.0) / 10000.0;
        res.fuelLiters = Math.round(totFuel * 10000.0) / 10000.0;
        res.fuelCost = Math.round(totFuelCost * 10000.0) / 10000.0;
        res.tollCost = Math.round(totToll * 10000.0) / 10000.0;
        res.totalCost = Math.round(gScore.get(target) * 10000.0) / 10000.0;
        res.nodesExplored = nodesExplored;
        res.executionTimeUs = execUs;

        return res;
    }
}
