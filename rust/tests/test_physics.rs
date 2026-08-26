use route_optimization_rust::models::{Config, Edge, Vehicle};
use route_optimization_rust::physics::*;
use std::collections::HashMap;
use std::f64::consts::PI;

fn get_test_vehicle() -> Vehicle {
    Vehicle {
        vehicle_mass: 1400.0,
        drag_coefficient: 0.30,
        frontal_area: 2.2,
        rolling_resistance_coefficient: 0.012,
        engine_efficiency: 0.30,
        fuel_energy_density: 34.2e6,
        fuel_price: 40.0,
    }
}

fn get_test_config() -> Config {
    let mut road_speed_factors = HashMap::new();
    road_speed_factors.insert("HIGHWAY".to_string(), 1.05);
    road_speed_factors.insert("MAIN_ROAD".to_string(), 0.95);
    Config {
        air_density: 1.225,
        gravity: 9.81,
        fuel_cost_weight: 0.40,
        time_weight: 0.40,
        toll_weight: 0.20,
        road_speed_factors,
    }
}

#[test]
fn test_aerodynamic_drag() {
    let v = get_test_vehicle();
    let c = get_test_config();
    let edge = Edge {
        id: 1,
        source: 0,
        target: 1,
        distance: 10.0,
        speed_limit: 100.0,
        average_speed: 100.0,
        road_type: "HIGHWAY".to_string(),
        slope: 0.0,
        elevation_difference: 0.0,
        traffic_factor: 1.0,
        wind_speed: 0.0,
        wind_direction: 0.0,
        toll_cost: 0.0,
        travel_time_min: 0.0,
        fuel_liters: 0.0,
        fuel_cost: 0.0,
        cost_fastest: 0.0,
        cost_cheapest: 0.0,
        cost_balanced: 0.0,
    };

    let v_ms = 100.0 / 3.6;
    let drag = calculate_aerodynamic_drag(v_ms, &edge, &v, &c);
    let expected = 0.5 * 1.225 * 0.30 * 2.2 * (v_ms.powi(2));
    assert!((drag - expected).abs() < 1e-4);
}

#[test]
fn test_slope_force() {
    let v = get_test_vehicle();
    let c = get_test_config();
    let slope_angle = 5.0 * (PI / 180.0);
    let edge = Edge {
        id: 1,
        source: 0,
        target: 1,
        distance: 1.0,
        speed_limit: 60.0,
        average_speed: 60.0,
        road_type: "MAIN_ROAD".to_string(),
        slope: slope_angle,
        elevation_difference: 87.0,
        traffic_factor: 1.0,
        wind_speed: 0.0,
        wind_direction: 0.0,
        toll_cost: 0.0,
        travel_time_min: 0.0,
        fuel_liters: 0.0,
        fuel_cost: 0.0,
        cost_fastest: 0.0,
        cost_cheapest: 0.0,
        cost_balanced: 0.0,
    };

    let f_slope = calculate_slope_force(&edge, &v, &c);
    let expected = 1400.0 * 9.81 * slope_angle.sin();
    assert!((f_slope - expected).abs() < 1e-3);
}

#[test]
fn test_rolling_resistance() {
    let v = get_test_vehicle();
    let c = get_test_config();
    let edge = Edge {
        id: 1,
        source: 0,
        target: 1,
        distance: 1.0,
        speed_limit: 60.0,
        average_speed: 60.0,
        road_type: "MAIN_ROAD".to_string(),
        slope: 0.0,
        elevation_difference: 0.0,
        traffic_factor: 1.0,
        wind_speed: 0.0,
        wind_direction: 0.0,
        toll_cost: 0.0,
        travel_time_min: 0.0,
        fuel_liters: 0.0,
        fuel_cost: 0.0,
        cost_fastest: 0.0,
        cost_cheapest: 0.0,
        cost_balanced: 0.0,
    };

    let f_roll = calculate_rolling_resistance(&edge, &v, &c);
    let expected = 0.012 * 1400.0 * 9.81 * 1.0;
    assert!((f_roll - expected).abs() < 1e-3);
}

#[test]
fn test_edge_metrics_consistency() {
    let v = get_test_vehicle();
    let c = get_test_config();
    let mut edge = Edge {
        id: 1,
        source: 0,
        target: 1,
        distance: 5.0,
        speed_limit: 120.0,
        average_speed: 110.0,
        road_type: "HIGHWAY".to_string(),
        slope: 0.01,
        elevation_difference: 50.0,
        traffic_factor: 0.9,
        wind_speed: 5.0,
        wind_direction: 0.0,
        toll_cost: 15.0,
        travel_time_min: 0.0,
        fuel_liters: 0.0,
        fuel_cost: 0.0,
        cost_fastest: 0.0,
        cost_cheapest: 0.0,
        cost_balanced: 0.0,
    };

    compute_edge_metrics(&mut edge, &v, &c);
    assert!(edge.travel_time_min > 0.0);
    assert!(edge.fuel_liters > 0.0);
    assert_eq!(edge.cost_fastest, edge.travel_time_min);
    assert!((edge.cost_cheapest - (edge.fuel_cost + 15.0)).abs() < 1e-5);
}
