package com.benchmark.models;

import com.fasterxml.jackson.annotation.JsonProperty;
import java.util.Map;

public class Config {
    @JsonProperty("air_density")
    public double airDensity;

    @JsonProperty("gravity")
    public double gravity;

    @JsonProperty("fuel_cost_weight")
    public double fuelCostWeight;

    @JsonProperty("time_weight")
    public double timeWeight;

    @JsonProperty("toll_weight")
    public double tollWeight;

    @JsonProperty("road_speed_factors")
    public Map<String, Double> roadSpeedFactors;

    public Config() {}

    public Config(double airDensity, double gravity, double fuelCostWeight,
                  double timeWeight, double tollWeight, Map<String, Double> roadSpeedFactors) {
        this.airDensity = airDensity;
        this.gravity = gravity;
        this.fuelCostWeight = fuelCostWeight;
        this.timeWeight = timeWeight;
        this.tollWeight = tollWeight;
        this.roadSpeedFactors = roadSpeedFactors;
    }
}
