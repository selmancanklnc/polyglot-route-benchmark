// Package routing provides Dijkstra and A* shortest path implementations
// for multi-objective vehicle route optimization over physical road networks.
//
// All functions are goroutine-safe (graph is read-only after construction)
// and are suitable for concurrent dispatch via goroutine pools.
package routing

import (
	"container/heap"
	"math"
	"route_optimization_go/pkg/graph"
	"route_optimization_go/pkg/models"
	"time"
)

// DijkstraItem is a priority queue entry representing a vertex with its accumulated cost.
type DijkstraItem struct {
	cost  float64
	node  int
	index int
}

type DijkstraPQ []*DijkstraItem

func (pq DijkstraPQ) Len() int           { return len(pq) }
func (pq DijkstraPQ) Less(i, j int) bool { return pq[i].cost < pq[j].cost }
func (pq DijkstraPQ) Swap(i, j int) {
	pq[i], pq[j] = pq[j], pq[i]
	pq[i].index = i
	pq[j].index = j
}
func (pq *DijkstraPQ) Push(x interface{}) {
	n := len(*pq)
	item := x.(*DijkstraItem)
	item.index = n
	*pq = append(*pq, item)
}
func (pq *DijkstraPQ) Pop() interface{} {
	old := *pq
	n := len(old)
	item := old[n-1]
	old[n-1] = nil
	item.index = -1
	*pq = old[0 : n-1]
	return item
}

// PrevEdge stores the predecessor vertex and edge index for path backtracking.
type PrevEdge struct {
	PrevNode  int
	EdgeIndex int
}

// SolveDijkstra computes the cost-optimal route between two vertices using Dijkstra's algorithm.
//
// Implements a binary min-heap (via container/heap) with lazy deletion for stale entries.
// The graph is traversed using pre-computed multi-objective edge costs. Path reconstruction
// backtracks the predecessor map to yield the complete ordered node sequence.
//
// Parameters:
//
//	g      - Pre-built Graph with physics-evaluated edge costs.
//	source - Origin vertex identifier.
//	target - Destination vertex identifier.
//	mode   - Optimization objective: "FASTEST", "CHEAPEST", or "BALANCED".
//
// Returns: Pointer to RouteResult with path and metrics, or nil if unreachable.
func SolveDijkstra(g *graph.Graph, source, target int, mode string) *models.RouteResult {
	startTime := time.Now()

	if _, ok := g.Nodes[source]; !ok {
		return nil
	}
	if _, ok := g.Nodes[target]; !ok {
		return nil
	}

	if source == target {
		return &models.RouteResult{
			Source:          source,
			Destination:     target,
			Optimization:    mode,
			Algorithm:       "DIJKSTRA",
			Route:           []int{source},
			DistanceKm:      0.0,
			TravelTimeMin:   0.0,
			FuelLiters:      0.0,
			FuelCost:        0.0,
			TollCost:        0.0,
			TotalCost:       0.0,
			NodesExplored:   1,
			ExecutionTimeUs: time.Since(startTime).Microseconds(),
		}
	}

	dist := make(map[int]float64)
	prev := make(map[int]PrevEdge)
	pq := &DijkstraPQ{}
	heap.Init(pq)

	dist[source] = 0.0
	heap.Push(pq, &DijkstraItem{cost: 0.0, node: source})

	nodesExplored := 0
	found := false

	for pq.Len() > 0 {
		top := heap.Pop(pq).(*DijkstraItem)
		u := top.node
		cost := top.cost

		if currDist, ok := dist[u]; ok && cost > currDist {
			continue
		}

		nodesExplored++

		if u == target {
			found = true
			break
		}

		for _, halfEdge := range g.Adj[u] {
			v := halfEdge.TargetNode
			edge := &g.Edges[halfEdge.EdgeIndex]
			weight := g.GetEdgeWeight(edge, mode)
			newDist := cost + weight

			if currV, ok := dist[v]; !ok || newDist < currV {
				dist[v] = newDist
				prev[v] = PrevEdge{PrevNode: u, EdgeIndex: halfEdge.EdgeIndex}
				heap.Push(pq, &DijkstraItem{cost: newDist, node: v})
			}
		}
	}

	execUs := time.Since(startTime).Microseconds()

	if !found {
		return nil
	}

	// Backtrack predecessor map to reconstruct the optimal node sequence
	routeNodes := make([]int, 0)
	routeEdges := make([]int, 0)
	curr := target
	routeNodes = append(routeNodes, curr)

	for curr != source {
		p := prev[curr]
		routeEdges = append(routeEdges, p.EdgeIndex)
		curr = p.PrevNode
		routeNodes = append(routeNodes, curr)
	}

	// Reverse route nodes and edges
	for i, j := 0, len(routeNodes)-1; i < j; i, j = i+1, j-1 {
		routeNodes[i], routeNodes[j] = routeNodes[j], routeNodes[i]
	}
	for i, j := 0, len(routeEdges)-1; i < j; i, j = i+1, j-1 {
		routeEdges[i], routeEdges[j] = routeEdges[j], routeEdges[i]
	}

	var totDist, totTime, totFuel, totFuelCost, totToll float64
	for _, idx := range routeEdges {
		e := &g.Edges[idx]
		totDist += e.Distance
		totTime += e.TravelTimeMin
		totFuel += e.FuelLiters
		totFuelCost += e.FuelCost
		totToll += e.TollCost
	}

	return &models.RouteResult{
		Source:          source,
		Destination:     target,
		Optimization:    mode,
		Algorithm:       "DIJKSTRA",
		Route:           routeNodes,
		DistanceKm:      math.Round(totDist*10000.0) / 10000.0,
		TravelTimeMin:   math.Round(totTime*10000.0) / 10000.0,
		FuelLiters:      math.Round(totFuel*10000.0) / 10000.0,
		FuelCost:        math.Round(totFuelCost*10000.0) / 10000.0,
		TollCost:        math.Round(totToll*10000.0) / 10000.0,
		TotalCost:       math.Round(dist[target]*10000.0) / 10000.0,
		NodesExplored:   nodesExplored,
		ExecutionTimeUs: execUs,
	}
}
