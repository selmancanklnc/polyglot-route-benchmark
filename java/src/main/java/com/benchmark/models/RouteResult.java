package com.benchmark.models;

import com.fasterxml.jackson.annotation.JsonProperty;
import java.util.List;

public class RouteResult {
    public int source;
    public int destination;
    public String optimization;
    public String algorithm;
    public List<Integer> route;

    @JsonProperty("distance_km")
    public double distanceKm;

    @JsonProperty("travel_time_min")
    public double travelTimeMin;

    @JsonProperty("fuel_liters")
    public double fuelLiters;

    @JsonProperty("fuel_cost")
    public double fuelCost;

    @JsonProperty("toll_cost")
    public double tollCost;

    @JsonProperty("total_cost")
    public double totalCost;

    @JsonProperty("nodes_explored")
    public int nodesExplored;

    @JsonProperty("execution_time_us")
    public long executionTimeUs;

    public RouteResult() {}
}
