"""Road network graph construction and adjacency index for route optimization.

Constructs a weighted directed graph from raw node and edge datasets, executing
full physics derivation on each edge at load time. Provides a unified interface
for cost weight retrieval across all three optimization modes.
"""

from typing import List, Dict, Tuple
from models import Node, Edge, Vehicle, Config
from physics import compute_edge_metrics


class Graph:
    """Weighted directed road network graph with pre-computed physical edge costs.

    At instantiation, each edge undergoes a complete vehicle physics evaluation
    (aerodynamic drag, slope gradient, rolling resistance, fuel energy conversion),
    yielding all three objective cost fields (FASTEST, CHEAPEST, BALANCED).
    The resulting graph is immutable after construction and safe for concurrent reads.

    Attributes:
        nodes: Dictionary mapping node ID to Node geometry.
        edges: Ordered list of all directed road segments.
        adj: Adjacency mapping from node ID to a list of (target_id, Edge) tuples.
        vehicle: Vehicular physical constants used during edge metric computation.
        config: Atmospheric and optimization weight parameters.
    """

    __slots__ = ('nodes', 'adj', 'edges', 'vehicle', 'config')

    def __init__(self, nodes: List[Node], edges: List[Edge], vehicle: Vehicle, config: Config) -> None:
        """Initializes the graph, triggers physics derivation, and builds adjacency structures.

        Args:
            nodes: Ordered list of spatial graph vertices.
            edges: Ordered list of directed road segment entities.
            vehicle: Vehicular aerodynamic and thermodynamic constants.
            config: Atmospheric parameters and multi-objective weight coefficients.
        """
        self.nodes: Dict[int, Node] = {n.id: n for n in nodes}
        self.edges: List[Edge] = edges
        self.vehicle: Vehicle = vehicle
        self.config: Config = config

        # Build forward adjacency list: node_id -> [(target_id, edge), ...]
        self.adj: Dict[int, List[Tuple[int, Edge]]] = {n.id: [] for n in nodes}
        for e in edges:
            compute_edge_metrics(e, vehicle, config)
            if e.source in self.adj:
                self.adj[e.source].append((e.target, e))

    def get_edge_weight(self, edge: Edge, mode: str) -> float:
        """Returns the pre-computed objective cost for the specified optimization mode.

        Args:
            edge: The road segment whose cost is to be retrieved.
            mode: Optimization strategy identifier. One of:
                - ``"FASTEST"``  – minimizes travel time (minutes).
                - ``"CHEAPEST"`` – minimizes total monetary expenditure (fuel + tolls).
                - ``"BALANCED"`` – evaluates weighted combination of all three objectives.

        Returns:
            Scalar edge traversal cost for the priority queue relaxation step.
        """
        if mode == "FASTEST":
            return edge.cost_fastest
        elif mode == "CHEAPEST":
            return edge.cost_cheapest
        elif mode == "BALANCED":
            return edge.cost_balanced
        else:
            return edge.cost_balanced
