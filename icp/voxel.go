package icp

import (
	"math"

	"github.com/flynnletford/icp-go/point"
)

type Voxel struct {
	I int // Voxel index on the x axis.
	J int // Voxel index on the y axis.
	K int // Voxel index on the z axis.
}

// Voxelize converts a set of 3D points into a voxelized set while preserving original coordinates.
func Voxelize(points []*point.Point3D, voxelSize float64) *point.Points3D {
	voxelMap := make(map[Voxel]*point.Point3D) // Stores the first point mapped to each voxel

	for i := range points {
		v := Voxel{
			I: int(math.Floor(points[i].X / voxelSize)),
			J: int(math.Floor(points[i].Y / voxelSize)),
			K: int(math.Floor(points[i].Z / voxelSize)),
		}

		// Store only the first encountered point per voxel.
		if _, exists := voxelMap[v]; !exists {
			voxelMap[v] = points[i]
		}
	}

	// Convert back to a slice of points where we only have one point per voxel.
	result := make([]*point.Point3D, 0, len(voxelMap))
	for _, p := range voxelMap {
		result = append(result, p)
	}

	val := point.Points3D(result)
	return &val
}
