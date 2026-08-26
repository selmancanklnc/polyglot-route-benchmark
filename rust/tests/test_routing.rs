use route_optimization_rust::astar::solve_astar;
use route_optimization_rust::dijkstra::solve_dijkstra;
use route_optimization_rust::engine::load_dataset;

#[test]
fn test_fixture_routing() {
    let fixture_path = "../datasets/test_fixture.json";
    let (graph, queries) = load_dataset(fixture_path).expect("Failed to load test_fixture.json");

    assert_eq!(graph.nodes.len(), 10);
    assert_eq!(graph.edges.len(), 26);
    assert_eq!(queries.len(), 6);

    for q in &queries {
        let res_d = solve_dijkstra(&graph, q.source, q.destination, &q.optimization);
        let res_a = solve_astar(&graph, q.source, q.destination, &q.optimization);

        assert!(res_d.is_some());
        assert!(res_a.is_some());

        let d = res_d.unwrap();
        let a = res_a.unwrap();

        assert!((d.total_cost - a.total_cost).abs() < 1e-4);
        assert_eq!(d.route, a.route);
    }
}

#[test]
fn test_edge_cases() {
    let fixture_path = "../datasets/test_fixture.json";
    let (graph, _) = load_dataset(fixture_path).expect("Failed to load test_fixture.json");

    // Unreachable
    let res_unreachable = solve_dijkstra(&graph, 0, 9999, "FASTEST");
    assert!(res_unreachable.is_none());

    // Same source and target
    let res_same = solve_dijkstra(&graph, 3, 3, "FASTEST");
    assert!(res_same.is_some());
    let same = res_same.unwrap();
    assert_eq!(same.route, vec![3]);
    assert_eq!(same.total_cost, 0.0);
}
