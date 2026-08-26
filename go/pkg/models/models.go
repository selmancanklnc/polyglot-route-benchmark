package models

type Node struct {
	ID        int     `json:"id"`
	X         f64     `json:"x"`
	Y         f64     `json:"y"`
	Elevation float64 `json:"elevation"`
}

type f64 = float64

type Vehicle struct {
	VehicleMass                  float64 `json:"vehicle_mass"`
	DragCoefficient              float64 `json:"drag_coefficient"`
	FrontalArea                  float64 `json:"frontal_area"`
	RollingResistanceCoefficient float64 `json:"rolling_resistance_coefficient"`
	EngineEfficiency             float64 `json:"engine_efficiency"`
	FuelEnergyDensity            float64 `json:"fuel_energy_density"`
	FuelPrice                    float64 `json:"fuel_price"`
}

type Config struct {
	AirDensity       float64            `json:"air_density"`
	Gravity          float64            `json:"gravity"`
	FuelCostWeight   float64            `json:"fuel_cost_weight"`
	TimeWeight       float64            `json:"time_weight"`
	TollWeight       float64            `json:"toll_weight"`
	RoadSpeedFactors map[string]float64 `json:"road_speed_factors"`
}

type Edge struct {
	ID                  int     `json:"id"`
	Source              int     `json:"source"`
	Target              int     `json:"target"`
	Distance            float64 `json:"distance"`
	SpeedLimit          float64 `json:"speed_limit"`
	AverageSpeed        float64 `json:"average_speed"`
	RoadType            string  `json:"road_type"`
	Slope               float64 `json:"slope"`
	ElevationDifference float64 `json:"elevation_difference"`
	TrafficFactor       float64 `json:"traffic_factor"`
	WindSpeed           float64 `json:"wind_speed"`
	WindDirection       float64 `json:"wind_direction"`
	TollCost            float64 `json:"toll_cost"`

	// Computed edge metrics
	TravelTimeMin float64 `json:"travel_time_min,omitempty"`
	FuelLiters    float64 `json:"fuel_liters,omitempty"`
	FuelCost      float64 `json:"fuel_cost,omitempty"`
	CostFastest   float64 `json:"cost_fastest,omitempty"`
	CostCheapest  float64 `json:"cost_cheapest,omitempty"`
	CostBalanced  float64 `json:"cost_balanced,omitempty"`
}

type Query struct {
	ID           int    `json:"id"`
	Source       int    `json:"source"`
	Destination  int    `json:"destination"`
	Optimization string `json:"optimization"`
	Algorithm    string `json:"algorithm"`
}

type Dataset struct {
	Vehicle       Vehicle `json:"vehicle"`
	Configuration Config  `json:"configuration"`
	Nodes         []Node  `json:"nodes"`
	Edges         []Edge  `json:"edges"`
	Queries       []Query `json:"queries"`
}

type RouteResult struct {
	Source          int     `json:"source"`
	Destination     int     `json:"destination"`
	Optimization    string  `json:"optimization"`
	Algorithm       string  `json:"algorithm"`
	Route           []int   `json:"route"`
	DistanceKm      float64 `json:"distance_km"`
	TravelTimeMin   float64 `json:"travel_time_min"`
	FuelLiters      float64 `json:"fuel_liters"`
	FuelCost        float64 `json:"fuel_cost"`
	TollCost        float64 `json:"toll_cost"`
	TotalCost       float64 `json:"total_cost"`
	NodesExplored   int     `json:"nodes_explored"`
	ExecutionTimeUs int64   `json:"execution_time_us"`
}
