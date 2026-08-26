// Package graph constructs and indexes the weighted directed road network.
//
// Builds a forward adjacency structure from raw vertex and edge datasets,
// executing full physics evaluation on each edge during construction.
// The resulting Graph is read-only after NewGraph returns, making it safe
// for concurrent access across multiple goroutines without additional locking.
package graph

import (
	"route_optimization_go/pkg/models"
	"route_optimization_go/pkg/physics"
)

// HalfEdge encodes a compact outgoing edge entry in the adjacency list.
// Stores the target node identifier and an index into the Graph's edge slice
// to avoid duplicating edge data in memory.
type HalfEdge struct {
	// TargetNode is the identifier of the destination vertex.
	TargetNode int
	// EdgeIndex is the position of this edge in Graph.Edges.
	EdgeIndex int
}

// Graph is a weighted directed road network with pre-computed physical edge costs.
//
// Fields are exported to allow read access from pathfinding algorithms without
// requiring accessor methods, keeping hot-path code allocation-free.
type Graph struct {
	// Nodes maps vertex identifier to spatial Node geometry.
	Nodes map[int]models.Node
	// Edges is the ordered slice of all directed road segments.
	Edges []models.Edge
	// Vehicle holds the vehicular physical constants for this benchmark run.
	Vehicle models.Vehicle
	// Config holds atmospheric parameters and optimization weight coefficients.
	Config models.Config
	// Adj is the forward adjacency index: node_id -> []HalfEdge.
	Adj map[int][]HalfEdge
}

// NewGraph constructs the Graph, runs physics derivation on all edges,
// and builds the forward adjacency index.
//
// Parameters:
//
//	nodes   - Complete list of spatial graph vertices.
//	edges   - Complete list of directed road segments (modified in-place).
//	vehicle - Vehicular aerodynamic and thermodynamic constants.
//	config  - Atmospheric parameters and multi-objective weight coefficients.
//
// Returns: Pointer to a fully constructed, read-only Graph.
func NewGraph(nodes []models.Node, edges []models.Edge, vehicle models.Vehicle, config models.Config) *Graph {
	nodeMap := make(map[int]models.Node, len(nodes))
	adj := make(map[int][]HalfEdge, len(nodes))

	for _, n := range nodes {
		nodeMap[n.ID] = n
		adj[n.ID] = make([]HalfEdge, 0, 4)
	}

	for i := range edges {
		physics.ComputeEdgeMetrics(&edges[i], &vehicle, &config)
		adj[edges[i].Source] = append(adj[edges[i].Source], HalfEdge{
			TargetNode: edges[i].Target,
			EdgeIndex:  i,
		})
	}

	return &Graph{
		Nodes:   nodeMap,
		Edges:   edges,
		Vehicle: vehicle,
		Config:  config,
		Adj:     adj,
	}
}

// GetEdgeWeight returns the pre-computed objective cost for the specified optimization mode.
//
// Parameters:
//
//	edge - The road segment whose cost is to be selected.
//	mode - Optimization strategy: "FASTEST", "CHEAPEST", or "BALANCED".
//
// Returns: Scalar edge traversal cost for priority queue relaxation.
func (g *Graph) GetEdgeWeight(edge *models.Edge, mode string) float64 {
	switch mode {
	case "FASTEST":
		return edge.CostFastest
	case "CHEAPEST":
		return edge.CostCheapest
	default:
		return edge.CostBalanced
	}
}
