package com.benchmark.models;

import com.fasterxml.jackson.annotation.JsonProperty;

/**
 * Origin-destination route calculation request specification.
 */
public class Query {
    /** Unique query index within the dataset. */
    public int id;
    /** Departure vertex identifier. */
    public int source;
    /** Target arrival vertex identifier. */
    public int destination;
    /** Optimization objective: "FASTEST", "CHEAPEST", or "BALANCED". */
    public String optimization;

    /** Pathfinding algorithm: "DIJKSTRA" or "ASTAR". Defaults to "DIJKSTRA". */
    @JsonProperty("algorithm")
    public String algorithm = "DIJKSTRA";

    /** Default constructor for JSON deserialization. */
    public Query() {}

    /**
     * Constructs a Query with all parameters.
     *
     * @param id           Query identifier.
     * @param source       Departure vertex ID.
     * @param destination  Arrival vertex ID.
     * @param optimization Cost objective strategy.
     * @param algorithm    Pathfinding algorithm ("DIJKSTRA" or "ASTAR").
     */
    public Query(int id, int source, int destination, String optimization, String algorithm) {
        this.id = id;
        this.source = source;
        this.destination = destination;
        this.optimization = optimization;
        this.algorithm = algorithm != null ? algorithm : "DIJKSTRA";
    }
}
