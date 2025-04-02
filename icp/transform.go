package icp

import (
	"math"

	"github.com/flynnletford/icp-go/point"
	"gonum.org/v1/gonum/mat"
)

// NewIdentityHomogeneous creates a 4x4 identity homogeneous transformation matrix
func NewIdentityHomogeneous() *mat.Dense {
	// Create a 4x4 identity matrix
	T := mat.NewDense(4, 4, nil)
	for i := 0; i < 4; i++ {
		T.Set(i, i, 1.0)
	}
	return T
}

// HomogeneousTransform creates a 4x4 homogeneous transformation matrix from a 3x3 rotation matrix and an (x, y, z) translation.
func HomogeneousTransform(R *mat.Dense, point *point.Point3D) *mat.Dense {
	// Create a 4x4 identity matrix
	T := mat.NewDense(4, 4, nil)
	for i := 0; i < 4; i++ {
		T.Set(i, i, 1.0)
	}

	// Copy rotation matrix into upper-left 3x3 block
	for i := 0; i < 3; i++ {
		for j := 0; j < 3; j++ {
			T.Set(i, j, R.At(i, j))
		}
	}

	// Set translation vector (last column, first three rows)
	T.Set(0, 3, point.X)
	T.Set(1, 3, point.Y)
	T.Set(2, 3, point.Z)

	return T
}

// ExtractTranslationYaw extracts (x, y, z) translation and yaw (rotation around Z-axis) from a 4x4 homogeneous transformation matrix.
func ExtractTranslationYaw(T *mat.Dense) (*point.Point3D, float64) {
	// Extract translation components
	x := T.At(0, 3)
	y := T.At(1, 3)
	z := T.At(2, 3)

	// Extract yaw from rotation part
	r11 := T.At(0, 0)
	r21 := T.At(1, 0)
	yaw := math.Atan2(r21, r11) // atan2(row 2, col 1 / row 1, col 1)

	point := &point.Point3D{
		X: x,
		Y: y,
		Z: z,
	}

	return point, yaw
}

func TransformPoint(p *point.Point3D, tform *mat.Dense) *point.Point3D {

	x := tform.At(0, 0)*p.X + tform.At(0, 1)*p.Y + tform.At(0, 2)*p.Z + tform.At(0, 3)
	y := tform.At(1, 0)*p.X + tform.At(1, 1)*p.Y + tform.At(1, 2)*p.Z + tform.At(1, 3)
	z := tform.At(2, 0)*p.X + tform.At(2, 1)*p.Y + tform.At(2, 2)*p.Z + tform.At(2, 3)

	return &point.Point3D{
		X: x,
		Y: y,
		Z: z,
	}
}
