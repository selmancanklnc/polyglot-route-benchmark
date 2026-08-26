// Package physics implements the vehicle dynamics and energy consumption model.
//
// Evaluates the three primary mechanical resistance forces acting on a vehicle
// traversing a road segment under specified environmental conditions:
//
//  1. Aerodynamic Drag: F_drag = 0.5 * rho * Cd * A * v_apparent^2
//  2. Slope Gravitational Force: F_slope = m * g * sin(theta)
//  3. Rolling Resistance: F_roll = Crr * m * g * cos(theta)
//
// Mechanical energy is then converted to fuel volume via powertrain thermal
// efficiency and the volumetric energy density of the fuel.
//
// All exported functions are stateless and goroutine-safe.
package physics

import (
	"math"
	"route_optimization_go/pkg/models"
)

// CalculateSlopeFactor returns the speed attenuation multiplier for a given road gradient.
//
// Uphill gradients reduce effective speed proportionally; downhill gradients produce
// a minor speed improvement from gravitational assist. The result is floored at 0.5
// to prevent unrealistic speed values on extreme inclines.
//
// Parameters:
//
//	slopeRad - Longitudinal road incline angle in radians.
//
// Returns: Dimensionless speed multiplier.
func CalculateSlopeFactor(slopeRad float64) float64 {
	if slopeRad > 0.0 {
		return math.Max(0.5, 1.0-2.0*math.Sin(slopeRad))
	}
	return 1.0 + 0.5*math.Abs(math.Sin(slopeRad))
}

// CalculateEffectiveSpeed evaluates the physical traversal velocity along a road segment.
//
// Considers statutory speed limits, historical average flow, traffic congestion index,
// road classification multiplier, and gravitational slope effects.
// Result is clamped to [10.0, 140.0] km/h.
//
// Parameters:
//
//	edge   - Road segment with speed and environmental parameters.
//	config - Simulation configuration with road speed factor lookup.
//
// Returns: Effective traversal velocity in km/h.
func CalculateEffectiveSpeed(edge *models.Edge, config *models.Config) float64 {
	baseSpeed := math.Min(edge.SpeedLimit, edge.AverageSpeed)
	roadFactor := 1.0
	if f, ok := config.RoadSpeedFactors[edge.RoadType]; ok {
		roadFactor = f
	}
	slopeFactor := CalculateSlopeFactor(edge.Slope)
	effSpeed := baseSpeed * edge.TrafficFactor * roadFactor * slopeFactor
	if effSpeed < 10.0 {
		return 10.0
	}
	if effSpeed > 140.0 {
		return 140.0
	}
	return effSpeed
}

// CalculateAerodynamicDrag evaluates the resistive aerodynamic force on the vehicle.
//
// The wind vector is projected onto the longitudinal axis of travel. Only the
// opposing (headwind) component reduces apparent speed. Crosswind components are
// excluded per the simplified 1D drag model.
//
// Formula: F_drag = 0.5 * rho * Cd * A * max(0, v - v_wind*cos(phi))^2
//
// Parameters:
//
//	vMs     - Vehicle velocity in m/s.
//	edge    - Road segment with wind speed and direction.
//	vehicle - Aerodynamic constants (Cd, frontal area).
//	config  - Atmospheric air density.
//
// Returns: Aerodynamic drag force in Newtons.
func CalculateAerodynamicDrag(vMs float64, edge *models.Edge, vehicle *models.Vehicle, config *models.Config) float64 {
	windRad := edge.WindDirection * (math.Pi / 180.0)
	windOpposing := edge.WindSpeed * math.Cos(windRad)
	apparentSpeed := math.Max(0.0, vMs-windOpposing)
	return 0.5 * config.AirDensity * vehicle.DragCoefficient * vehicle.FrontalArea * (apparentSpeed * apparentSpeed)
}

// CalculateSlopeForce evaluates the longitudinal gravitational resistance along the incline.
//
// Formula: F_slope = m * g * sin(theta)
//
// Positive slope produces resistive force; negative slope yields a propulsive assist.
//
// Parameters:
//
//	edge    - Road segment slope angle in radians.
//	vehicle - Vehicle mass.
//	config  - Gravitational acceleration constant.
//
// Returns: Slope force component in Newtons.
func CalculateSlopeForce(edge *models.Edge, vehicle *models.Vehicle, config *models.Config) float64 {
	return vehicle.VehicleMass * config.Gravity * math.Sin(edge.Slope)
}

// CalculateRollingResistance evaluates Coulomb tire-pavement rolling friction.
//
// Formula: F_roll = Crr * m * g * cos(theta)
//
// Parameters:
//
//	edge    - Road segment slope angle in radians (affects normal force).
//	vehicle - Vehicle mass and rolling resistance coefficient.
//	config  - Gravitational acceleration constant.
//
// Returns: Rolling resistance force in Newtons.
func CalculateRollingResistance(edge *models.Edge, vehicle *models.Vehicle, config *models.Config) float64 {
	return vehicle.RollingResistanceCoefficient * vehicle.VehicleMass * config.Gravity * math.Cos(edge.Slope)
}

// ComputeEdgeMetrics performs complete physics derivation and cost synthesis for an edge.
//
// Mutates the provided edge pointer in-place with:
//   - TravelTimeMin: traversal duration in minutes
//   - FuelLiters: volumetric fuel consumption
//   - FuelCost: monetary cost of consumed fuel
//   - CostFastest, CostCheapest, CostBalanced: per-mode objective costs
//
// Called once per edge at graph construction time; results are cached for the
// lifetime of the Graph.
//
// Parameters:
//
//	edge    - Mutable road segment to populate.
//	vehicle - Vehicular physical constants.
//	config  - Atmospheric parameters and objective weight coefficients.
func ComputeEdgeMetrics(edge *models.Edge, vehicle *models.Vehicle, config *models.Config) {
	vKmh := CalculateEffectiveSpeed(edge, config)
	vMs := vKmh / 3.6

	tHours := edge.Distance / vKmh
	edge.TravelTimeMin = tHours * 60.0
	tSeconds := tHours * 3600.0

	fDrag := CalculateAerodynamicDrag(vMs, edge, vehicle, config)
	fSlope := CalculateSlopeForce(edge, vehicle, config)
	fRoll := CalculateRollingResistance(edge, vehicle, config)

	fTotal := math.Max(0.0, fDrag+fSlope+fRoll)
	powerWatts := fTotal * vMs
	energyMechJoules := powerWatts * tSeconds

	energyFuelJoules := energyMechJoules / vehicle.EngineEfficiency
	edge.FuelLiters = energyFuelJoules / vehicle.FuelEnergyDensity
	edge.FuelCost = edge.FuelLiters * vehicle.FuelPrice

	edge.CostFastest = edge.TravelTimeMin
	edge.CostCheapest = edge.FuelCost + edge.TollCost
	edge.CostBalanced = config.FuelCostWeight*edge.FuelCost +
		config.TimeWeight*edge.TravelTimeMin +
		config.TollWeight*edge.TollCost
}
