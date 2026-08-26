package physics

import (
	"math"
	"route_optimization_go/pkg/models"
	"testing"
)

func getTestVehicle() models.Vehicle {
	return models.Vehicle{
		VehicleMass:                  1400.0,
		DragCoefficient:              0.30,
		FrontalArea:                  2.2,
		RollingResistanceCoefficient: 0.012,
		EngineEfficiency:             0.30,
		FuelEnergyDensity:            34.2e6,
		FuelPrice:                    40.0,
	}
}

func getTestConfig() models.Config {
	return models.Config{
		AirDensity:     1.225,
		Gravity:        9.81,
		FuelCostWeight: 0.40,
		TimeWeight:     0.40,
		TollWeight:     0.20,
		RoadSpeedFactors: map[string]float64{
			"HIGHWAY":   1.05,
			"MAIN_ROAD": 0.95,
		},
	}
}

func TestAerodynamicDrag(t *testing.T) {
	v := getTestVehicle()
	c := getTestConfig()
	edge := models.Edge{
		Distance:     10.0,
		SpeedLimit:   100.0,
		AverageSpeed: 100.0,
		RoadType:     "HIGHWAY",
		TrafficFactor: 1.0,
	}
	vMs := 100.0 / 3.6
	drag := CalculateAerodynamicDrag(vMs, &edge, &v, &c)
	expected := 0.5 * 1.225 * 0.30 * 2.2 * (vMs * vMs)
	if math.Abs(drag-expected) > 1e-4 {
		t.Fatalf("Drag mismatch: got %f, expected %f", drag, expected)
	}
}

func TestSlopeForce(t *testing.T) {
	v := getTestVehicle()
	c := getTestConfig()
	slopeAngle := 5.0 * (math.Pi / 180.0)
	edge := models.Edge{
		Distance:     1.0,
		SpeedLimit:   60.0,
		AverageSpeed: 60.0,
		RoadType:     "MAIN_ROAD",
		Slope:        slopeAngle,
	}
	fSlope := CalculateSlopeForce(&edge, &v, &c)
	expected := 1400.0 * 9.81 * math.Sin(slopeAngle)
	if math.Abs(fSlope-expected) > 1e-3 {
		t.Fatalf("Slope force mismatch: got %f, expected %f", fSlope, expected)
	}
}

func TestRollingResistance(t *testing.T) {
	v := getTestVehicle()
	c := getTestConfig()
	edge := models.Edge{
		Distance:     1.0,
		SpeedLimit:   60.0,
		AverageSpeed: 60.0,
		RoadType:     "MAIN_ROAD",
		Slope:        0.0,
	}
	fRoll := CalculateRollingResistance(&edge, &v, &c)
	expected := 0.012 * 1400.0 * 9.81 * 1.0
	if math.Abs(fRoll-expected) > 1e-3 {
		t.Fatalf("Rolling resistance mismatch: got %f, expected %f", fRoll, expected)
	}
}

func TestEdgeMetricsConsistency(t *testing.T) {
	v := getTestVehicle()
	c := getTestConfig()
	edge := models.Edge{
		Distance:      5.0,
		SpeedLimit:    120.0,
		AverageSpeed:  110.0,
		RoadType:      "HIGHWAY",
		Slope:         0.01,
		TrafficFactor: 0.9,
		WindSpeed:     5.0,
		TollCost:      15.0,
	}
	ComputeEdgeMetrics(&edge, &v, &c)
	if edge.TravelTimeMin <= 0 || edge.FuelLiters <= 0 {
		t.Fatalf("Invalid travel time or fuel liters: %v", edge)
	}
	if edge.CostFastest != edge.TravelTimeMin {
		t.Fatalf("Cost fastest mismatch")
	}
	if math.Abs(edge.CostCheapest-(edge.FuelCost+15.0)) > 1e-5 {
		t.Fatalf("Cost cheapest mismatch")
	}
}
