"""Unit verification suite for vehicle physics and aerodynamic calculations.

Validates each physical sub-formula independently against analytically
expected values derived from first-principles mechanics. All test cases use
a consistent 1,400 kg medium passenger vehicle specification matching the
benchmark dataset vehicle constants.
"""

import math
import sys
import os
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

from models import Vehicle, Config, Edge
from physics import (
    calculate_slope_factor,
    calculate_effective_speed,
    calculate_aerodynamic_drag,
    calculate_slope_force,
    calculate_rolling_resistance,
    compute_edge_metrics
)

# ---------------------------------------------------------------------------
# Shared test fixtures
# ---------------------------------------------------------------------------

def _make_vehicle() -> Vehicle:
    """Returns the canonical benchmark vehicle specification."""
    return Vehicle(
        vehicle_mass=1400.0,
        drag_coefficient=0.30,
        frontal_area=2.2,
        rolling_resistance_coefficient=0.012,
        engine_efficiency=0.30,
        fuel_energy_density=34.2e6,
        fuel_price=40.0
    )


def _make_config() -> Config:
    """Returns the canonical benchmark atmospheric and weight configuration."""
    return Config(
        air_density=1.225,
        gravity=9.81,
        fuel_cost_weight=0.40,
        time_weight=0.40,
        toll_weight=0.20,
        road_speed_factors={
            "HIGHWAY": 1.05,
            "MAIN_ROAD": 0.95,
            "CITY_ROAD": 0.70,
            "RURAL_ROAD": 0.85,
            "MOUNTAIN_ROAD": 0.60
        }
    )


# ---------------------------------------------------------------------------
# Test cases
# ---------------------------------------------------------------------------

class TestPhysics(unittest.TestCase):
    """Isolated unit tests for vehicle dynamics and energy consumption formulas."""

    def setUp(self) -> None:
        """Initialises shared vehicle and configuration fixtures before each test."""
        self.vehicle = _make_vehicle()
        self.config = _make_config()

    def test_aerodynamic_drag_zero_wind(self) -> None:
        """Drag force must equal 0.5 * rho * Cd * A * v^2 in zero-wind conditions."""
        edge = Edge(
            id=1, source=0, target=1, distance=10.0, speed_limit=100.0, average_speed=100.0,
            road_type="HIGHWAY", slope=0.0, elevation_difference=0.0, traffic_factor=1.0,
            wind_speed=0.0, wind_direction=0.0, toll_cost=0.0
        )
        v_ms = 100.0 / 3.6
        drag = calculate_aerodynamic_drag(v_ms, edge, self.vehicle, self.config)
        expected = 0.5 * 1.225 * 0.30 * 2.2 * (v_ms ** 2)
        self.assertAlmostEqual(drag, expected, places=4,
                               msg="Aerodynamic drag deviates from analytical formula.")

    def test_slope_force_five_degrees(self) -> None:
        """Slope force at 5° incline must match m * g * sin(5°)."""
        slope_angle = math.radians(5.0)
        edge = Edge(
            id=1, source=0, target=1, distance=1.0, speed_limit=60.0, average_speed=60.0,
            road_type="MAIN_ROAD", slope=slope_angle, elevation_difference=87.0,
            traffic_factor=1.0, wind_speed=0.0, wind_direction=0.0, toll_cost=0.0
        )
        f_slope = calculate_slope_force(edge, self.vehicle, self.config)
        expected = 1400.0 * 9.81 * math.sin(slope_angle)
        self.assertAlmostEqual(f_slope, expected, places=3,
                               msg="Slope force deviates from m * g * sin(theta).")

    def test_rolling_resistance_flat_road(self) -> None:
        """Rolling resistance on flat ground must equal Crr * m * g."""
        edge = Edge(
            id=1, source=0, target=1, distance=1.0, speed_limit=60.0, average_speed=60.0,
            road_type="MAIN_ROAD", slope=0.0, elevation_difference=0.0,
            traffic_factor=1.0, wind_speed=0.0, wind_direction=0.0, toll_cost=0.0
        )
        f_roll = calculate_rolling_resistance(edge, self.vehicle, self.config)
        expected = 0.012 * 1400.0 * 9.81 * 1.0
        self.assertAlmostEqual(f_roll, expected, places=3,
                               msg="Rolling resistance deviates from Crr * m * g * cos(theta).")

    def test_edge_metrics_cost_consistency(self) -> None:
        """Verifies that cost_fastest equals travel_time and cost_cheapest equals fuel + toll."""
        edge = Edge(
            id=1, source=0, target=1, distance=5.0, speed_limit=120.0, average_speed=110.0,
            road_type="HIGHWAY", slope=0.01, elevation_difference=50.0,
            traffic_factor=0.9, wind_speed=5.0, wind_direction=0.0, toll_cost=15.0
        )
        compute_edge_metrics(edge, self.vehicle, self.config)
        self.assertGreater(edge.travel_time_min, 0.0,
                           msg="Travel time must be positive for a non-zero distance edge.")
        self.assertGreater(edge.fuel_liters, 0.0,
                           msg="Fuel consumption must be positive for any traversed edge.")
        self.assertEqual(edge.cost_fastest, edge.travel_time_min,
                         msg="FASTEST cost must equal travel time in minutes.")
        self.assertAlmostEqual(edge.cost_cheapest, edge.fuel_cost + 15.0, places=5,
                               msg="CHEAPEST cost must equal fuel_cost + toll_cost.")


if __name__ == "__main__":
    unittest.main(verbosity=2)
