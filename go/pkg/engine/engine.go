package engine

import (
	"encoding/json"
	"os"
	"route_optimization_go/pkg/graph"
	"route_optimization_go/pkg/models"
	"route_optimization_go/pkg/routing"
	"sync"
)

func LoadDataset(filepath string) (*graph.Graph, []models.Query, error) {
	data, err := os.ReadFile(filepath)
	if err != nil {
		return nil, nil, err
	}

	var ds models.Dataset
	if err := json.Unmarshal(data, &ds); err != nil {
		return nil, nil, err
	}

	g := graph.NewGraph(ds.Nodes, ds.Edges, ds.Vehicle, ds.Configuration)
	return g, ds.Queries, nil
}

func ExecuteQuery(g *graph.Graph, q *models.Query) *models.RouteResult {
	if q.Algorithm == "ASTAR" {
		return routing.SolveAStar(g, q.Source, q.Destination, q.Optimization)
	}
	return routing.SolveDijkstra(g, q.Source, q.Destination, q.Optimization)
}

func RunSequential(g *graph.Graph, queries []models.Query) []models.RouteResult {
	results := make([]models.RouteResult, 0, len(queries))
	for i := range queries {
		if res := ExecuteQuery(g, &queries[i]); res != nil {
			results = append(results, *res)
		}
	}
	return results
}

func RunParallel(g *graph.Graph, queries []models.Query, numWorkers int) []models.RouteResult {
	results := make([]models.RouteResult, len(queries))
	jobs := make(chan int, len(queries))

	var wg sync.WaitGroup
	for w := 0; w < numWorkers; w++ {
		wg.Add(1)
		go func() {
			defer wg.Done()
			for idx := range jobs {
				if res := ExecuteQuery(g, &queries[idx]); res != nil {
					results[idx] = *res
				}
			}
		}()
	}

	for i := range queries {
		jobs <- i
	}
	close(jobs)

	wg.Wait()

	// Filter non-empty
	filtered := make([]models.RouteResult, 0, len(results))
	for _, r := range results {
		if len(r.Route) > 0 {
			filtered = append(filtered, r)
		}
	}
	return filtered
}
