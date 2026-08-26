package com.benchmark.physics;

import com.benchmark.models.Config;
import com.benchmark.models.Edge;
import com.benchmark.models.Vehicle;

/**
 * Vehicle dynamics and energy consumption calculation engine.
 *
 * <p>Implements the three-force mechanical resistance model for a vehicle traversing
 * a directed road segment under real-world physical conditions:
 *
 * <ul>
 *   <li>Aerodynamic Drag: {@code F_drag = 0.5 * rho * Cd * A * v_apparent^2}</li>
 *   <li>Slope Gravitational Force: {@code F_slope = m * g * sin(theta)}</li>
 *   <li>Rolling Resistance: {@code F_roll = Crr * m * g * cos(theta)}</li>
 * </ul>
 *
 * <p>Net mechanical energy is then converted to volumetric fuel consumption through
 * powertrain thermal efficiency and the energy density of the fuel. All methods are
 * static and thread-safe.
 */
public class Physics {

    /**
     * Computes the speed attenuation multiplier for a given longitudinal road gradient.
     *
     * <p>Uphill gradients reduce traversal speed proportionally; downhill gradients provide
     * a modest speed improvement from gravitational assist. The result is floored at 0.5
     * to prevent degenerate velocity values on extreme inclines.
     *
     * @param slopeRad Longitudinal incline angle in radians.
     * @return Dimensionless speed multiplier applied to nominal traversal velocity.
     */
    public static double calculateSlopeFactor(double slopeRad) {
        if (slopeRad > 0.0) {
            return Math.max(0.5, 1.0 - 2.0 * Math.sin(slopeRad));
        }
        return 1.0 + 0.5 * Math.abs(Math.sin(slopeRad));
    }

    /**
     * Evaluates the effective physical traversal velocity along a road segment.
     *
     * <p>Considers statutory speed limits, historical average flow speed, traffic
     * congestion index, road classification multiplier, and road gradient effects.
     * The result is clamped to [10.0, 140.0] km/h.
     *
     * @param edge   Road segment with speed limit and environmental parameters.
     * @param config Simulation configuration including road speed factor lookup table.
     * @return Effective traversal velocity in km/h.
     */
    public static double calculateEffectiveSpeed(Edge edge, Config config) {
        double baseSpeed = Math.min(edge.speedLimit, edge.averageSpeed);
        double roadFactor = 1.0;
        if (config.roadSpeedFactors != null && config.roadSpeedFactors.containsKey(edge.roadType)) {
            roadFactor = config.roadSpeedFactors.get(edge.roadType);
        }
        double slopeFactor = calculateSlopeFactor(edge.slope);
        double effSpeed = baseSpeed * edge.trafficFactor * roadFactor * slopeFactor;
        return Math.max(10.0, Math.min(140.0, effSpeed));
    }

    /**
     * Evaluates the aerodynamic drag force resisting vehicular motion.
     *
     * <p>The wind vector projection onto the longitudinal axis reduces apparent vehicle
     * speed. Only the headwind component is subtracted per the simplified 1D drag model.
     *
     * <p>Formula: {@code F_drag = 0.5 * rho * Cd * A * max(0, v - v_wind*cos(phi))^2}
     *
     * @param vMs     Vehicle velocity relative to road surface (m/s).
     * @param edge    Road segment with wind speed and incident angle.
     * @param vehicle Vehicular aerodynamic constants (Cd, frontal area).
     * @param config  Atmospheric air density.
     * @return Aerodynamic drag force in Newtons.
     */
    public static double calculateAerodynamicDrag(double vMs, Edge edge, Vehicle vehicle, Config config) {
        double windRad = edge.windDirection * (Math.PI / 180.0);
        double windOpposing = edge.windSpeed * Math.cos(windRad);
        double apparentSpeed = Math.max(0.0, vMs - windOpposing);
        return 0.5 * config.airDensity * vehicle.dragCoefficient * vehicle.frontalArea * (apparentSpeed * apparentSpeed);
    }

    /**
     * Evaluates the longitudinal gravitational resistance component along the road incline.
     *
     * <p>Formula: {@code F_slope = m * g * sin(theta)}
     *
     * @param edge    Road segment slope angle in radians.
     * @param vehicle Vehicle mass.
     * @param config  Gravitational acceleration constant.
     * @return Slope force in Newtons.
     */
    public static double calculateSlopeForce(Edge edge, Vehicle vehicle, Config config) {
        return vehicle.vehicleMass * config.gravity * Math.sin(edge.slope);
    }

    /**
     * Evaluates Coulomb tire-pavement rolling friction force.
     *
     * <p>Formula: {@code F_roll = Crr * m * g * cos(theta)}
     *
     * @param edge    Road segment slope angle in radians (affects normal force).
     * @param vehicle Vehicle mass and rolling resistance coefficient.
     * @param config  Gravitational acceleration constant.
     * @return Rolling resistance force in Newtons.
     */
    public static double calculateRollingResistance(Edge edge, Vehicle vehicle, Config config) {
        return vehicle.rollingResistanceCoefficient * vehicle.vehicleMass * config.gravity * Math.cos(edge.slope);
    }

    /**
     * Performs complete physics derivation and multi-objective cost synthesis for an edge.
     *
     * <p>Mutates the provided edge with:
     * <ul>
     *   <li>{@code travelTimeMin} — segment traversal duration in minutes</li>
     *   <li>{@code fuelLiters} — volumetric fuel consumed in liters</li>
     *   <li>{@code fuelCost} — monetary fuel expenditure ($)</li>
     *   <li>{@code costFastest}, {@code costCheapest}, {@code costBalanced} — per-mode costs</li>
     * </ul>
     *
     * <p>Invoked once per edge at graph construction time; results are cached for the
     * lifetime of the graph.
     *
     * @param edge    Mutable road segment to populate with computed metrics.
     * @param vehicle Vehicular physical constants.
     * @param config  Atmospheric parameters and multi-objective weight coefficients.
     */
    public static void computeEdgeMetrics(Edge edge, Vehicle vehicle, Config config) {
        double vKmh = calculateEffectiveSpeed(edge, config);
        double vMs = vKmh / 3.6;

        double tHours = edge.distance / vKmh;
        edge.travelTimeMin = tHours * 60.0;
        double tSeconds = tHours * 3600.0;

        double fDrag  = calculateAerodynamicDrag(vMs, edge, vehicle, config);
        double fSlope = calculateSlopeForce(edge, vehicle, config);
        double fRoll  = calculateRollingResistance(edge, vehicle, config);

        double fTotal          = Math.max(0.0, fDrag + fSlope + fRoll);
        double powerWatts      = fTotal * vMs;
        double energyMechJ     = powerWatts * tSeconds;

        double energyFuelJ = energyMechJ / vehicle.engineEfficiency;
        edge.fuelLiters    = energyFuelJ / vehicle.fuelEnergyDensity;
        edge.fuelCost      = edge.fuelLiters * vehicle.fuelPrice;

        edge.costFastest  = edge.travelTimeMin;
        edge.costCheapest = edge.fuelCost + edge.tollCost;
        edge.costBalanced = config.fuelCostWeight * edge.fuelCost
                + config.timeWeight * edge.travelTimeMin
                + config.tollWeight * edge.tollCost;
    }
}
