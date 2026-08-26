#!/usr/bin/env python3
"""
Deterministic Road Network Dataset Generator
Generates synthetic 2D/3D road network graphs with physical parameters:
- Nodes: ID, latitude/longitude (x, y coords in km), elevation (m)
- Edges: source, target, distance (km), speed_limit (km/h), average_speed (km/h),
         road_type, elevation_difference (m), slope (radians), traffic_factor (0.1-1.0),
         wind_speed (m/s), wind_direction (deg), toll_cost (currency)
- Vehicle: mass (kg), Cd, frontal_area (m^2), Crr, engine_efficiency, fuel_density (J/L), fuel_price
- Configuration: physics constants, optimization weights
- Queries: benchmark queries with source, destination, optimization mode, algorithm
"""

import json
import math
import random
import os
import sys

ROAD_TYPES = ["HIGHWAY", "MAIN_ROAD", "CITY_ROAD", "RURAL_ROAD", "MOUNTAIN_ROAD"]

DEFAULT_VEHICLE = {
    "vehicle_mass": 1400.0,              # kg
    "drag_coefficient": 0.30,           # Cd
    "frontal_area": 2.2,                # m^2
    "rolling_resistance_coefficient": 0.012, # Crr
    "engine_efficiency": 0.30,          # 30% thermal efficiency
    "fuel_energy_density": 34.2e6,      # Joules per liter (Gasoline ~34.2 MJ/L)
    "fuel_price": 40.0                  # Currency per liter (e.g. TL)
}

DEFAULT_CONFIG = {
    "air_density": 1.225,               # kg/m^3 (sea level standard)
    "gravity": 9.81,                    # m/s^2
    "fuel_cost_weight": 0.40,
    "time_weight": 0.40,
    "toll_weight": 0.20,
    "road_speed_factors": {
        "HIGHWAY": 1.05,
        "MAIN_ROAD": 0.95,
        "CITY_ROAD": 0.70,
        "RURAL_ROAD": 0.85,
        "MOUNTAIN_ROAD": 0.60
    }
}

def generate_graph(num_nodes: int, num_edges: int, seed: int = 42, num_queries: int = 20):
    rng = random.Random(seed)
    
    grid_side = max(1, int(math.sqrt(num_nodes)))
    spacing = 1.0  # ~1 km grid spacing base
    
    nodes = []
    for i in range(num_nodes):
        row = i // grid_side
        col = i % grid_side
        x = col * spacing + rng.uniform(-0.3 * spacing, 0.3 * spacing)
        y = row * spacing + rng.uniform(-0.3 * spacing, 0.3 * spacing)
        elev = 100.0 + 300.0 * math.sin(x * 0.1) * math.cos(y * 0.1) + rng.uniform(0, 50.0)
        nodes.append({
            "id": i,
            "x": round(x, 4),
            "y": round(y, 4),
            "elevation": round(elev, 2)
        })
        
    edge_set = set()
    edges = []
    
    for i in range(num_nodes):
        row = i // grid_side
        col = i % grid_side
        if col + 1 < grid_side and (i + 1) < num_nodes:
            edge_set.add((i, i + 1))
            edge_set.add((i + 1, i))
        south_idx = i + grid_side
        if south_idx < num_nodes:
            edge_set.add((i, south_idx))
            edge_set.add((south_idx, i))
            
    attempts = 0
    while len(edge_set) < num_edges and attempts < num_edges * 10:
        attempts += 1
        u = rng.randint(0, num_nodes - 1)
        offset = rng.randint(1, min(50, max(2, num_nodes - 1)))
        v = (u + offset) % num_nodes
        if u != v:
            edge_set.add((u, v))
            edge_set.add((v, u))

    edge_id = 0
    for u, v in sorted(edge_set):
        n_u = nodes[u]
        n_v = nodes[v]
        dx = n_v["x"] - n_u["x"]
        dy = n_v["y"] - n_u["y"]
        horizontal_dist = math.sqrt(dx*dx + dy*dy)
        dh = (n_v["elevation"] - n_u["elevation"]) / 1000.0
        actual_distance = math.sqrt(horizontal_dist*horizontal_dist + dh*dh)
        actual_distance = max(0.1, actual_distance)
        
        slope_rad = math.asin(min(1.0, max(-1.0, (n_v["elevation"] - n_u["elevation"]) / (actual_distance * 1000.0))))
        
        rt_roll = rng.random()
        if rt_roll < 0.15:
            road_type = "HIGHWAY"
            speed_limit = 120.0
            avg_speed = 110.0
            toll_cost = round(rng.choice([0.0, 15.0, 30.0, 50.0]), 2)
        elif rt_roll < 0.40:
            road_type = "MAIN_ROAD"
            speed_limit = 90.0
            avg_speed = 80.0
            toll_cost = 0.0
        elif rt_roll < 0.70:
            road_type = "CITY_ROAD"
            speed_limit = 50.0
            avg_speed = 40.0
            toll_cost = 0.0
        elif rt_roll < 0.85:
            road_type = "RURAL_ROAD"
            speed_limit = 70.0
            avg_speed = 60.0
            toll_cost = 0.0
        else:
            road_type = "MOUNTAIN_ROAD"
            speed_limit = 60.0
            avg_speed = 45.0
            toll_cost = 0.0

        traffic_factor = round(rng.uniform(0.60, 1.00), 3)
        wind_speed = round(rng.uniform(0.0, 12.0), 2)
        wind_direction = round(rng.uniform(0.0, 360.0), 1)
        
        edges.append({
            "id": edge_id,
            "source": u,
            "target": v,
            "distance": round(actual_distance, 4),
            "speed_limit": speed_limit,
            "average_speed": avg_speed,
            "road_type": road_type,
            "slope": round(slope_rad, 6),
            "elevation_difference": round(n_v["elevation"] - n_u["elevation"], 2),
            "traffic_factor": traffic_factor,
            "wind_speed": wind_speed,
            "wind_direction": wind_direction,
            "toll_cost": toll_cost
        })
        edge_id += 1
        if len(edges) >= num_edges:
            break

    queries = []
    modes = ["FASTEST", "CHEAPEST", "BALANCED"]
    algorithms = ["DIJKSTRA", "ASTAR"]
    for q in range(num_queries):
        src = (q * 17) % num_nodes
        dst = (num_nodes - 1 - (q * 23) % num_nodes)
        if src == dst:
            dst = (src + num_nodes // 2) % num_nodes
        mode = modes[q % len(modes)]
        algo = algorithms[(q // len(modes)) % len(algorithms)]
        queries.append({
            "id": q,
            "source": src,
            "destination": dst,
            "optimization": mode,
            "algorithm": algo
        })
        
    return {
        "metadata": {
            "num_nodes": len(nodes),
            "num_edges": len(edges),
            "num_queries": len(queries),
            "seed": seed
        },
        "vehicle": DEFAULT_VEHICLE,
        "configuration": DEFAULT_CONFIG,
        "nodes": nodes,
        "edges": edges,
        "queries": queries
    }

def main():
    output_dir = os.path.dirname(os.path.abspath(__file__))
    
    # 1. Deterministic fixture for unit/cross-tests (10 nodes, 26 edges, 6 queries)
    fixture_data = generate_graph(num_nodes=10, num_edges=26, seed=42, num_queries=6)
    with open(os.path.join(output_dir, "test_fixture.json"), "w") as f:
        json.dump(fixture_data, f, indent=2)
    print(f"Generated test_fixture.json ({len(fixture_data['nodes'])} nodes, {len(fixture_data['edges'])} edges)")

    # 2. Small dataset (1,000 nodes, 5,000 edges, 24 queries)
    small_data = generate_graph(num_nodes=1000, num_edges=5000, seed=42, num_queries=24)
    with open(os.path.join(output_dir, "small.json"), "w") as f:
        json.dump(small_data, f, indent=2)
    print(f"Generated small.json ({len(small_data['nodes'])} nodes, {len(small_data['edges'])} edges)")

    # 3. Medium dataset (10,000 nodes, 50,000 edges, 50 queries)
    medium_data = generate_graph(num_nodes=10000, num_edges=50000, seed=42, num_queries=50)
    with open(os.path.join(output_dir, "medium.json"), "w") as f:
        json.dump(medium_data, f)
    print(f"Generated medium.json ({len(medium_data['nodes'])} nodes, {len(medium_data['edges'])} edges)")

    # 4. Large dataset (50,000 nodes, 250,000 edges, 100 queries)
    large_data = generate_graph(num_nodes=50000, num_edges=250000, seed=42, num_queries=100)
    with open(os.path.join(output_dir, "large.json"), "w") as f:
        json.dump(large_data, f)
    print(f"Generated large.json ({len(large_data['nodes'])} nodes, {len(large_data['edges'])} edges)")

if __name__ == "__main__":
    main()
