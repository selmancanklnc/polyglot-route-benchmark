package com.benchmark.models;

import com.fasterxml.jackson.annotation.JsonProperty;

public class Edge {
    public int id;
    public int source;
    public int target;
    public double distance;

    @JsonProperty("speed_limit")
    public double speedLimit;

    @JsonProperty("average_speed")
    public double averageSpeed;

    @JsonProperty("road_type")
    public String roadType;

    public double slope;

    @JsonProperty("elevation_difference")
    public double elevationDifference;

    @JsonProperty("traffic_factor")
    public double trafficFactor;

    @JsonProperty("wind_speed")
    public double windSpeed;

    @JsonProperty("wind_direction")
    public double windDirection;

    @JsonProperty("toll_cost")
    public double tollCost;

    // Computed edge metrics
    @JsonProperty("travel_time_min")
    public double travelTimeMin;

    @JsonProperty("fuel_liters")
    public double fuelLiters;

    @JsonProperty("fuel_cost")
    public double fuelCost;

    @JsonProperty("cost_fastest")
    public double costFastest;

    @JsonProperty("cost_cheapest")
    public double costCheapest;

    @JsonProperty("cost_balanced")
    public double costBalanced;

    public Edge() {}
}
