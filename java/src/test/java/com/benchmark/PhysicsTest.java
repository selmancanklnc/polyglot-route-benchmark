package com.benchmark;

import com.benchmark.models.Config;
import com.benchmark.models.Edge;
import com.benchmark.models.Vehicle;
import com.benchmark.physics.Physics;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;

import java.util.HashMap;
import java.util.Map;

import static org.junit.jupiter.api.Assertions.*;

/**
 * Unit verification suite for vehicle physics and aerodynamic calculations.
 *
 * <p>Validates each physical sub-formula independently against analytically expected
 * values derived from first-principles mechanics. Uses a consistent 1,400 kg
 * medium passenger vehicle specification matching the canonical benchmark dataset.
 */
public class PhysicsTest {

    private Vehicle vehicle;
    private Config config;

    /**
     * Initialises shared vehicle and configuration fixtures before each test method.
     */
    @BeforeEach
    public void setUp() {
        vehicle = new Vehicle(1400.0, 0.30, 2.2, 0.012, 0.30, 34.2e6, 40.0);
        Map<String, Double> speedFactors = new HashMap<>();
        speedFactors.put("HIGHWAY",  1.05);
        speedFactors.put("MAIN_ROAD", 0.95);
        config = new Config(1.225, 9.81, 0.40, 0.40, 0.20, speedFactors);
    }

    /**
     * Drag force must equal 0.5 * rho * Cd * A * v^2 in zero-wind conditions.
     */
    @Test
    @DisplayName("Aerodynamic drag matches F = 0.5 * rho * Cd * A * v^2 at zero wind")
    public void testAerodynamicDrag() {
        Edge edge = new Edge();
        edge.distance      = 10.0;
        edge.speedLimit    = 100.0;
        edge.averageSpeed  = 100.0;
        edge.roadType      = "HIGHWAY";
        edge.trafficFactor = 1.0;

        double vMs     = 100.0 / 3.6;
        double drag    = Physics.calculateAerodynamicDrag(vMs, edge, vehicle, config);
        double expected = 0.5 * 1.225 * 0.30 * 2.2 * (vMs * vMs);
        assertEquals(expected, drag, 1e-4,
            "Aerodynamic drag deviates from analytical formula F = 0.5*rho*Cd*A*v^2.");
    }

    /**
     * Slope force at 5° incline must match m * g * sin(5°).
     */
    @Test
    @DisplayName("Slope force matches F = m * g * sin(theta) at 5-degree incline")
    public void testSlopeForce() {
        double slopeAngle = 5.0 * (Math.PI / 180.0);
        Edge edge = new Edge();
        edge.distance     = 1.0;
        edge.speedLimit   = 60.0;
        edge.averageSpeed = 60.0;
        edge.roadType     = "MAIN_ROAD";
        edge.slope        = slopeAngle;

        double fSlope   = Physics.calculateSlopeForce(edge, vehicle, config);
        double expected = 1400.0 * 9.81 * Math.sin(slopeAngle);
        assertEquals(expected, fSlope, 1e-3,
            "Slope force deviates from m * g * sin(theta).");
    }

    /**
     * Rolling resistance on flat ground must equal Crr * m * g.
     */
    @Test
    @DisplayName("Rolling resistance matches F = Crr * m * g on flat road")
    public void testRollingResistance() {
        Edge edge = new Edge();
        edge.distance     = 1.0;
        edge.speedLimit   = 60.0;
        edge.averageSpeed = 60.0;
        edge.roadType     = "MAIN_ROAD";
        edge.slope        = 0.0;

        double fRoll    = Physics.calculateRollingResistance(edge, vehicle, config);
        double expected = 0.012 * 1400.0 * 9.81 * 1.0;
        assertEquals(expected, fRoll, 1e-3,
            "Rolling resistance deviates from Crr * m * g * cos(theta).");
    }

    /**
     * Verifies that costFastest equals travelTimeMin and costCheapest equals fuel + toll.
     */
    @Test
    @DisplayName("Edge cost fields are internally consistent after computeEdgeMetrics")
    public void testEdgeMetricsConsistency() {
        Edge edge = new Edge();
        edge.distance      = 5.0;
        edge.speedLimit    = 120.0;
        edge.averageSpeed  = 110.0;
        edge.roadType      = "HIGHWAY";
        edge.slope         = 0.01;
        edge.trafficFactor = 0.9;
        edge.windSpeed     = 5.0;
        edge.tollCost      = 15.0;

        Physics.computeEdgeMetrics(edge, vehicle, config);
        assertTrue(edge.travelTimeMin > 0,
            "Travel time must be positive for a non-zero distance edge.");
        assertTrue(edge.fuelLiters > 0,
            "Fuel consumption must be positive for any traversed edge.");
        assertEquals(edge.travelTimeMin, edge.costFastest, 1e-9,
            "FASTEST cost must equal travel time in minutes.");
        assertEquals(edge.fuelCost + 15.0, edge.costCheapest, 1e-5,
            "CHEAPEST cost must equal fuel_cost + toll_cost.");
    }
}
