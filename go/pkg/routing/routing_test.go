package routing_test

import (
	"math"
	"route_optimization_go/pkg/engine"
	"route_optimization_go/pkg/routing"
	"testing"
)

func TestFixtureRouting(t *testing.T) {
	fixturePath := "../../../datasets/test_fixture.json"
	g, queries, err := engine.LoadDataset(fixturePath)
	if err != nil {
		t.Fatalf("Failed to load dataset: %v", err)
	}

	if len(g.Nodes) != 10 {
		t.Fatalf("Expected 10 nodes, got %d", len(g.Nodes))
	}
	if len(g.Edges) != 26 {
		t.Fatalf("Expected 26 edges, got %d", len(g.Edges))
	}
	if len(queries) != 6 {
		t.Fatalf("Expected 6 queries, got %d", len(queries))
	}

	for _, q := range queries {
		resD := routing.SolveDijkstra(g, q.Source, q.Destination, q.Optimization)
		resA := routing.SolveAStar(g, q.Source, q.Destination, q.Optimization)

		if resD == nil || resA == nil {
			t.Fatalf("Query failed: %v", q)
		}

		if math.Abs(resD.TotalCost-resA.TotalCost) > 1e-4 {
			t.Fatalf("Cost mismatch for query %v: Dijkstra=%f, AStar=%f", q, resD.TotalCost, resA.TotalCost)
		}

		if len(resD.Route) != len(resA.Route) {
			t.Fatalf("Route length mismatch for query %v: Dijkstra=%v, AStar=%v", q, resD.Route, resA.Route)
		}

		for i := range resD.Route {
			if resD.Route[i] != resA.Route[i] {
				t.Fatalf("Route node mismatch at index %d: %d vs %d", i, resD.Route[i], resA.Route[i])
			}
		}
	}
}

func TestEdgeCases(t *testing.T) {
	fixturePath := "../../../datasets/test_fixture.json"
	g, _, err := engine.LoadDataset(fixturePath)
	if err != nil {
		t.Fatalf("Failed to load dataset: %v", err)
	}

	// Unreachable
	resUnreachable := routing.SolveDijkstra(g, 0, 9999, "FASTEST")
	if resUnreachable != nil {
		t.Fatalf("Expected nil for unreachable node")
	}

	// Same source and target
	resSame := routing.SolveDijkstra(g, 3, 3, "FASTEST")
	if resSame == nil || len(resSame.Route) != 1 || resSame.Route[0] != 3 || resSame.TotalCost != 0.0 {
		t.Fatalf("Unexpected result for same node: %v", resSame)
	}
}
