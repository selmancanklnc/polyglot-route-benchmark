package com.benchmark.models;

import com.fasterxml.jackson.annotation.JsonProperty;

public class Vehicle {
    @JsonProperty("vehicle_mass")
    public double vehicleMass;

    @JsonProperty("drag_coefficient")
    public double dragCoefficient;

    @JsonProperty("frontal_area")
    public double frontalArea;

    @JsonProperty("rolling_resistance_coefficient")
    public double rollingResistanceCoefficient;

    @JsonProperty("engine_efficiency")
    public double engineEfficiency;

    @JsonProperty("fuel_energy_density")
    public double fuelEnergyDensity;

    @JsonProperty("fuel_price")
    public double fuelPrice;

    public Vehicle() {}

    public Vehicle(double vehicleMass, double dragCoefficient, double frontalArea,
                   double rollingResistanceCoefficient, double engineEfficiency,
                   double fuelEnergyDensity, double fuelPrice) {
        this.vehicleMass = vehicleMass;
        this.dragCoefficient = dragCoefficient;
        this.frontalArea = frontalArea;
        this.rollingResistanceCoefficient = rollingResistanceCoefficient;
        this.engineEfficiency = engineEfficiency;
        this.fuelEnergyDensity = fuelEnergyDensity;
        this.fuelPrice = fuelPrice;
    }
}
