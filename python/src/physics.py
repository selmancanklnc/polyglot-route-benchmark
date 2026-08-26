"""Vehicle Dynamics & Energy Calculation Physics Module.

Implements rigorous physical modeling of road resistances:
1. Aerodynamic drag as a function of relative apparent wind speed.
2. Incline gravitational resistance based on longitudinal road gradient.
3. Coulomb rolling resistance derived from normal tire force.
4. Powertrain thermal energy conversion and volumetric fuel expenditure.
"""

import math
from models import Vehicle, Config, Edge

def calculate_slope_factor(slope_rad: float) -> float:
    """Calculates speed attenuation factor due to road incline.
    
    Args:
        slope_rad: Longitudinal incline angle in radians.
        
    Returns:
        Multiplier factor applied to steady-state speed.
    """
    if slope_rad > 0:
        return max(0.5, 1.0 - 2.0 * math.sin(slope_rad))
    else:
        return 1.0 + 0.5 * abs(math.sin(slope_rad))

def calculate_effective_speed(edge: Edge, config: Config) -> float:
    """Evaluates the effective physical traversal velocity along an edge segment.
    
    Considers statutory limits, congestion indices, road typology multipliers,
    and gravitational slope limits. Bounded in [10.0, 140.0] km/h.
    
    Args:
        edge: Edge data structure containing segment properties.
        config: Simulation environment configuration.
        
    Returns:
        Effective velocity in km/h.
    """
    base_speed = min(edge.speed_limit, edge.average_speed)
    road_factor = config.road_speed_factors.get(edge.road_type, 1.0)
    slope_factor = calculate_slope_factor(edge.slope)
    
    eff_speed = base_speed * edge.traffic_factor * road_factor * slope_factor
    return max(10.0, min(140.0, eff_speed))

def calculate_aerodynamic_drag(v_ms: float, edge: Edge, vehicle: Vehicle, config: Config) -> float:
    """Evaluates the aerodynamic drag force opposing vehicular motion.
    
    Formula: F_drag = 0.5 * rho * Cd * A * (v - v_wind_parallel)^2
    
    Args:
        v_ms: Vehicle velocity relative to road in m/s.
        edge: Edge properties including wind speed and incident angle.
        vehicle: Vehicle aerodynamic attributes (Cd, frontal area).
        config: Atmospheric properties (air density).
        
    Returns:
        Resistive aerodynamic force in Newtons.
    """
    wind_rad = math.radians(edge.wind_direction)
    wind_opposing = edge.wind_speed * math.cos(wind_rad)
    apparent_speed = max(0.0, v_ms - wind_opposing)
    
    return 0.5 * config.air_density * vehicle.drag_coefficient * vehicle.frontal_area * (apparent_speed ** 2)

def calculate_slope_force(edge: Edge, vehicle: Vehicle, config: Config) -> float:
    """Evaluates the longitudinal component of gravitational force.
    
    Formula: F_slope = m * g * sin(theta)
    
    Args:
        edge: Edge entity containing road slope angle theta.
        vehicle: Vehicle mass properties.
        config: Gravitational acceleration constant.
        
    Returns:
        Gravitational slope force in Newtons.
    """
    return vehicle.vehicle_mass * config.gravity * math.sin(edge.slope)

def calculate_rolling_resistance(edge: Edge, vehicle: Vehicle, config: Config) -> float:
    """Evaluates tire-pavement rolling friction force.
    
    Formula: F_roll = Crr * m * g * cos(theta)
    
    Args:
        edge: Edge entity containing road slope angle theta.
        vehicle: Vehicle mass and rolling resistance coefficient.
        config: Gravitational acceleration constant.
        
    Returns:
        Rolling resistance force in Newtons.
    """
    return vehicle.rolling_resistance_coefficient * vehicle.vehicle_mass * config.gravity * math.cos(edge.slope)

def compute_edge_metrics(edge: Edge, vehicle: Vehicle, config: Config) -> None:
    """Performs end-to-end physics derivation and cost calculation for an edge.
    
    Mutates the provided edge in-place with calculated traversal duration,
    mechanical energy expenditure, volumetric fuel consumption, and multi-objective costs.
    
    Args:
        edge: The target Edge instance to populate.
        vehicle: Vehicular physical properties.
        config: Global simulation parameters and objective weights.
    """
    # 1. Effective speed (km/h) and velocity (m/s)
    v_kmh = calculate_effective_speed(edge, config)
    v_ms = v_kmh / 3.6
    
    # 2. Travel time
    t_hours = edge.distance / v_kmh
    edge.travel_time_min = t_hours * 60.0
    t_seconds = t_hours * 3600.0
    
    # 3. Physical resistance forces
    f_drag = calculate_aerodynamic_drag(v_ms, edge, vehicle, config)
    f_slope = calculate_slope_force(edge, vehicle, config)
    f_roll = calculate_rolling_resistance(edge, vehicle, config)
    
    f_total = max(0.0, f_drag + f_slope + f_roll)
    
    # 4. Power (Watts) & Mechanical Energy (Joules)
    power_watts = f_total * v_ms
    energy_mech_joules = power_watts * t_seconds
    
    # 5. Fuel consumption & costs
    energy_fuel_joules = energy_mech_joules / vehicle.engine_efficiency
    edge.fuel_liters = energy_fuel_joules / vehicle.fuel_energy_density
    edge.fuel_cost = edge.fuel_liters * vehicle.fuel_price
    
    # 6. Costs per optimization strategy
    edge.cost_fastest = edge.travel_time_min
    edge.cost_cheapest = edge.fuel_cost + edge.toll_cost
    edge.cost_balanced = (
        config.fuel_cost_weight * edge.fuel_cost +
        config.time_weight * edge.travel_time_min +
        config.toll_weight * edge.toll_cost
    )
