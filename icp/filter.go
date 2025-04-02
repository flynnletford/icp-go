package icp

import (
	"math"

	"github.com/flynnletford/icp-go/point"
	"gonum.org/v1/gonum/spatial/kdtree"
)

func Filter(points *point.Points3D, params *Params) (*kdtree.Tree, *point.Points3D) {

	voxelized := Voxelize(*points, params.FilterParams.VoxelSize)
	tree := kdtree.New(voxelized, false)

	return tree, voxelized
}

func RemoveFloorAndCeiling(points *point.Points3D) []*point.Point3D {

	const floor float64 = -0.3
	const ceiling = 1.5

	filtered := make([]*point.Point3D, 0, points.Len())

	for _, p := range points.Raw() {
		if p.Z > floor && p.Z < ceiling {
			filtered = append(filtered, p)
		}
	}

	return filtered
}

// Mean and standard deviation helper functions
func mean(data []float64) float64 {
	sum := 0.0
	for _, v := range data {
		sum += v
	}
	return sum / float64(len(data))
}

func stdDev(data []float64, meanVal float64) float64 {
	sum := 0.0
	for _, v := range data {
		sum += (v - meanVal) * (v - meanVal)
	}
	return math.Sqrt(sum / float64(len(data)))
}

func removeNonNormalPoints(points *point.Points3D, numNeighbors int) *point.Points3D {

	tree := kdtree.New(points, false)

	maxDistance := 0.2 // 20 cm

	filteredPoints := make([]*point.Point3D, 0, points.Len())

	for _, point := range points.Raw() {
		nKeeper := kdtree.NewNKeeper(numNeighbors + 1) // +1 because the point itself is included.
		tree.NearestSet(nKeeper, point)

		allWithinDistance := true
		for _, dist := range nKeeper.Heap[1:] { // Skip the first one since this is the point itself.
			if dist.Dist > maxDistance {
				allWithinDistance = false
				break
			}
		}

		if allWithinDistance {
			filteredPoints = append(filteredPoints, point)
		}
	}

	wrapped := point.Points3D(filteredPoints)

	return &wrapped
}

// removeOutliersKDTree uses KD-Tree for fast nearest neighbor searches. It returns the filtered kdtree and the filtered raw points.
func removeOutliersKDTree(points *point.Points3D, numNeighbors int, zThreshold float64) *point.Points3D {

	distances := computeDistances(points, numNeighbors)
	filteredPoints := filterPoints(points, distances, zThreshold)

	return filteredPoints
}

func computeDistances(points *point.Points3D, numNeighbors int) []float64 {

	tree := kdtree.New(points, false)

	// Compute mean distance to k nearest neighbors.
	distances := make([]float64, points.Len())

	for i, point := range points.Raw() {

		nKeeper := kdtree.NewNKeeper(numNeighbors + 1) // +1 because the point itself is included.
		tree.NearestSet(nKeeper, point)

		sum := 0.0
		for _, dist := range nKeeper.Heap[1:] { // Skip the first one since this is the point itself.
			if dist.Comparable == point {
				continue
			}
			sum += dist.Dist
		}
		distances[i] = sum / float64(numNeighbors)
	}

	return distances
}

func filterPoints(points *point.Points3D, distances []float64, zThreshold float64) *point.Points3D {

	distMean := mean(distances)
	distStd := stdDev(distances, distMean)

	filteredPointsRaw := make([]*point.Point3D, 0, points.Len())
	for i, p := range points.Raw() {
		var zScore float64
		if distStd > 0 {
			zScore = (distances[i] - distMean) / distStd
		} else {
			zScore = 0 // Treat all points as non-outliers in this case.
		}
		if math.Abs(zScore) < zThreshold {
			filteredPointsRaw = append(filteredPointsRaw, p)
		}
	}

	filteredPoints := point.Points3D(filteredPointsRaw)

	return &filteredPoints
}
