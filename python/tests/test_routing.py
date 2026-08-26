"""Integration tests for Dijkstra and A* routing algorithms.

Verifies graph loading correctness, cross-algorithm path identity and cost
equivalence on the deterministic test fixture, and edge-case handling
(unreachable vertices, trivial zero-cost self-routes).
"""

import sys
import os
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

from engine import load_dataset, execute_query
from dijkstra import solve_dijkstra
from astar import solve_astar
from models import Query


class TestRouting(unittest.TestCase):
    """Integration tests for the complete routing pipeline on the canonical test fixture.

    Uses the deterministic 10-node, 26-edge ``test_fixture.json`` dataset
    (generated with seed=42) to ensure reproducible, environment-independent
    test outcomes.
    """

    def setUp(self) -> None:
        """Loads the canonical test fixture graph and query set before each test."""
        self.fixture_path = os.path.join(
            os.path.dirname(__file__), '..', '..', 'datasets', 'test_fixture.json'
        )
        self.graph, self.queries = load_dataset(self.fixture_path)

    def test_fixture_graph_dimensions(self) -> None:
        """Validates that the test fixture contains exactly 10 nodes, 26 edges, and 6 queries."""
        self.assertEqual(len(self.graph.nodes), 10,
                         msg="Test fixture should contain exactly 10 nodes.")
        self.assertEqual(len(self.graph.edges), 26,
                         msg="Test fixture should contain exactly 26 directed edges.")
        self.assertEqual(len(self.queries), 6,
                         msg="Test fixture should contain exactly 6 query specifications.")

    def test_dijkstra_astar_cost_equivalence(self) -> None:
        """Asserts that Dijkstra and A* produce identical optimal cost and path sequences.

        Since both algorithms are guaranteed-optimal with an admissible heuristic,
        any discrepancy in route or cost constitutes a correctness regression.
        """
        for q in self.queries:
            res_d = solve_dijkstra(self.graph, q.source, q.destination, q.optimization)
            res_a = solve_astar(self.graph, q.source, q.destination, q.optimization)

            self.assertIsNotNone(res_d, msg=f"Dijkstra returned None for query {q.id}.")
            self.assertIsNotNone(res_a, msg=f"A* returned None for query {q.id}.")
            self.assertAlmostEqual(
                res_d.total_cost, res_a.total_cost, places=4,
                msg=f"Cost mismatch between Dijkstra and A* on query {q.id}."
            )
            self.assertEqual(
                res_d.route, res_a.route,
                msg=f"Path identity mismatch between Dijkstra and A* on query {q.id}."
            )

    def test_unreachable_destination_returns_none(self) -> None:
        """Ensures that routing to a non-existent node returns None gracefully."""
        result = solve_dijkstra(self.graph, 0, 9999, "FASTEST")
        self.assertIsNone(result,
                          msg="Routing to a non-existent node must return None.")

    def test_trivial_same_source_target(self) -> None:
        """Verifies zero-cost degenerate route when source equals destination."""
        result = solve_dijkstra(self.graph, 3, 3, "FASTEST")
        self.assertIsNotNone(result,
                             msg="Self-route should always return a valid RouteResult.")
        self.assertEqual(result.route, [3],
                         msg="Self-route path must contain only the origin node.")
        self.assertEqual(result.total_cost, 0.0,
                         msg="Self-route cost must be exactly 0.0.")


if __name__ == "__main__":
    unittest.main(verbosity=2)
