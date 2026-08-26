"""Domain models and data structures for vehicle routing simulation.

Defines vertex topologies, edge dynamics, vehicular constants, atmospheric
configurations, and query execution results.
"""

from dataclasses import dataclass
from typing import List, Dict, Optional

@dataclass
class Node:
    """Represents a discrete geographic vertex in the spatial road network.
    
    Attributes:
        id: Unique integer identifier for the vertex.
        x: Local Cartesian X coordinate in kilometers.
        y: Local Cartesian Y coordinate in kilometers.
        elevation: Elevation above sea level in meters.
    """
    id: int
    x: float
    y: float
    elevation: float

@dataclass
class Vehicle:
    """Thermodynamic and aerodynamic physical parameters of the vehicle.
    
    Attributes:
        vehicle_mass: Gross vehicle mass including baseline payload (kg).
        drag_coefficient: Dimensionless aerodynamic drag coefficient (Cd).
        frontal_area: Projected cross-sectional frontal area (m^2).
        rolling_resistance_coefficient: Dimensionless rolling resistance (Crr).
        engine_efficiency: Powertrain thermal-to-mechanical conversion efficiency.
        fuel_energy_density: Specific volumetric energy density of fuel (J/L).
        fuel_price: Financial unit tariff per liter of fuel ($).
    """
    vehicle_mass: float
    drag_coefficient: float
    frontal_area: float
    rolling_resistance_coefficient: float
    engine_efficiency: float
    fuel_energy_density: float
    fuel_price: float

@dataclass
class Config:
    """Atmospheric constants, physical bounds, and multi-objective weight coefficients.
    
    Attributes:
        air_density: Standard sea-level air density (kg/m^3).
        gravity: Acceleration due to earth gravity (m/s^2).
        fuel_cost_weight: Optimization weight assigned to fuel expenditure.
        time_weight: Optimization weight assigned to travel duration.
        toll_weight: Optimization weight assigned to toll fees.
        road_speed_factors: Speed multiplier mapping by road type classification.
    """
    air_density: float
    gravity: float
    fuel_cost_weight: float
    time_weight: float
    toll_weight: float
    road_speed_factors: Dict[str, float]

@dataclass
class Edge:
    """Directed road segment connecting two vertices with physical properties.
    
    Attributes:
        id: Unique identifier for the edge segment.
        source: Origin node ID.
        target: Destination node ID.
        distance: Longitudinal segment length in kilometers.
        speed_limit: Legal maximum speed limit in km/h.
        average_speed: Historical unconstrained speed in km/h.
        road_type: Classification string (e.g. HIGHWAY, MAIN_ROAD).
        slope: Longitudinal road gradient angle in radians.
        elevation_difference: Altitude delta (target - source) in meters.
        traffic_factor: Traffic flow index in interval (0.0, 1.0].
        wind_speed: Ambient wind velocity magnitude in m/s.
        wind_direction: Incident wind angle in degrees (0 = direct headwind).
        toll_cost: Monetary highway surcharge ($).
    """
    id: int
    source: int
    target: int
    distance: float
    speed_limit: float
    average_speed: float
    road_type: str
    slope: float
    elevation_difference: float
    traffic_factor: float
    wind_speed: float
    wind_direction: float
    toll_cost: float
    
    # Synthesized physical and economic metrics
    travel_time_min: float = 0.0
    fuel_liters: float = 0.0
    fuel_cost: float = 0.0
    cost_fastest: float = 0.0
    cost_cheapest: float = 0.0
    cost_balanced: float = 0.0

@dataclass
class Query:
    """Origin-destination route calculation request.
    
    Attributes:
        id: Unique query index.
        source: Origin vertex ID.
        destination: Destination vertex ID.
        optimization: Objective criterion (FASTEST, CHEAPEST, BALANCED).
        algorithm: Pathfinding search algorithm (DIJKSTRA, ASTAR).
    """
    id: int
    source: int
    destination: int
    optimization: str
    algorithm: str

@dataclass
class RouteResult:
    """Comprehensive performance and physical evaluation of an optimal route.
    
    Attributes:
        source: Origin vertex identifier.
        destination: Target vertex identifier.
        optimization: Evaluated optimization mode.
        algorithm: Pathfinding algorithm utilized.
        route: Ordered list of traversed node identifiers.
        distance_km: Total cumulative path distance in kilometers.
        travel_time_min: Total path traversal duration in minutes.
        fuel_liters: Total consumed fuel volume in liters.
        fuel_cost: Total financial cost of consumed fuel ($).
        toll_cost: Aggregate highway toll charges ($).
        total_cost: Evaluated objective cost function value.
        nodes_explored: Number of graph vertices settled during search.
        execution_time_us: Microseconds elapsed during pathfinding search.
    """
    source: int
    destination: int
    optimization: str
    algorithm: str
    route: List[int]
    distance_km: float
    travel_time_min: float
    fuel_liters: float
    fuel_cost: float
    toll_cost: float
    total_cost: float
    nodes_explored: int = 0
    execution_time_us: int = 0
