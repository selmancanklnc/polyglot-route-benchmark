package routing

import (
	"container/heap"
	"math"
	"route_optimization_go/pkg/graph"
	"route_optimization_go/pkg/models"
	"time"
)

type AStarItem struct {
	fScore float64
	gScore float64
	node   int
	index  int
}

type AStarPQ []*AStarItem

func (pq AStarPQ) Len() int           { return len(pq) }
func (pq AStarPQ) Less(i, j int) bool { return pq[i].fScore < pq[j].fScore }
func (pq AStarPQ) Swap(i, j int) {
	pq[i], pq[j] = pq[j], pq[i]
	pq[i].index = i
	pq[j].index = j
}
func (pq *AStarPQ) Push(x interface{}) {
	n := len(*pq)
	item := x.(*AStarItem)
	item.index = n
	*pq = append(*pq, item)
}
func (pq *AStarPQ) Pop() interface{} {
	old := *pq
	n := len(old)
	item := old[n-1]
	old[n-1] = nil
	item.index = -1
	*pq = old[0 : n-1]
	return item
}

func AStarHeuristic(uNode, targetNode *models.Node, mode string, config *models.Config) float64 {
	dx := targetNode.X - uNode.X
	dy := targetNode.Y - uNode.Y
	distKm := math.Sqrt(dx*dx + dy*dy)
	minTimeMin := (distKm / 140.0) * 60.0

	switch mode {
	case "FASTEST":
		return minTimeMin
	case "BALANCED":
		return config.TimeWeight * minTimeMin
	default:
		return 0.0
	}
}

func SolveAStar(g *graph.Graph, source, target int, mode string) *models.RouteResult {
	startTime := time.Now()

	targetNode, okTarget := g.Nodes[target]
	sourceNode, okSource := g.Nodes[source]
	if !okTarget || !okSource {
		return nil
	}

	if source == target {
		return &models.RouteResult{
			Source:          source,
			Destination:     target,
			Optimization:    mode,
			Algorithm:       "ASTAR",
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

	gScore := make(map[int]float64)
	prev := make(map[int]PrevEdge)
	pq := &AStarPQ{}
	heap.Init(pq)

	h0 := AStarHeuristic(&sourceNode, &targetNode, mode, &g.Config)
	gScore[source] = 0.0
	heap.Push(pq, &AStarItem{fScore: h0, gScore: 0.0, node: source})

	nodesExplored := 0
	found := false

	for pq.Len() > 0 {
		top := heap.Pop(pq).(*AStarItem)
		u := top.node
		gVal := top.gScore

		if currG, ok := gScore[u]; ok && gVal > currG {
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
			tentativeG := gVal + weight

			if currV, ok := gScore[v]; !ok || tentativeG < currV {
				gScore[v] = tentativeG
				prev[v] = PrevEdge{PrevNode: u, EdgeIndex: halfEdge.EdgeIndex}
				vNode := g.Nodes[v]
				h := AStarHeuristic(&vNode, &targetNode, mode, &g.Config)
				heap.Push(pq, &AStarItem{
					fScore: tentativeG + h,
					gScore: tentativeG,
					node:   v,
				})
			}
		}
	}

	execUs := time.Since(startTime).Microseconds()

	if !found {
		return nil
	}

	// Reconstruct path
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
		Algorithm:       "ASTAR",
		Route:           routeNodes,
		DistanceKm:      math.Round(totDist*10000.0) / 10000.0,
		TravelTimeMin:   math.Round(totTime*10000.0) / 10000.0,
		FuelLiters:      math.Round(totFuel*10000.0) / 10000.0,
		FuelCost:        math.Round(totFuelCost*10000.0) / 10000.0,
		TollCost:        math.Round(totToll*10000.0) / 10000.0,
		TotalCost:       math.Round(gScore[target]*10000.0) / 10000.0,
		NodesExplored:   nodesExplored,
		ExecutionTimeUs: execUs,
	}
}
