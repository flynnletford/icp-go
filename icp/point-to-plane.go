package icp

import (
	"math"
	"time"

	"github.com/flynnletford/icp-go/point"
	"github.com/pkg/errors"
	"github.com/team-rocos/go-common/transform"
	"gonum.org/v1/gonum/mat"
)

// PointToPlane performs ICP using point-to-plane error minimization.
func PointToPlane(source *point.Points3D, target *point.Points3D, params *Params) (*Result, error) {

	startTime := time.Now()

	_, transformed := Filter(source, params)
	tree, _ := Filter(target, params)

	if err := ComputeNormals(tree, target, params.NumNeighborsNormals); err != nil {
		return nil, errors.Wrap(err, "failed to compute normals")
	}

	// Visualize(target)

	// Initialise our final transform calculated.
	finalTransform := transform.Matrix4Identity()

	for iter := 0; iter < params.MaxIterations; iter++ {
		// Step 1: Find closest points in target.
		correspondences := make([][2]*point.Point3D, 0, transformed.Len())

		for _, p := range transformed.Raw() {
			nearest, _ := tree.Nearest(p)
			if nearest == nil {
				continue
			}
			// if dist > params.MaxCorrespondenceDistance {
			// 	continue
			// }

			targetPoint := nearest.(*point.Point3D)
			correspondences = append(correspondences, [2]*point.Point3D{p, targetPoint})
		}

		// Step 2: Construct Ax = b system
		A := mat.NewDense(6, 6, nil)
		b := mat.NewVecDense(6, nil)

		for _, pair := range correspondences {
			src, tgt := pair[0], pair[1]

			// Compute residual = (R * src + t - tgt) ⋅ normal
			r := mat.NewVecDense(3, []float64{src.X - tgt.X, src.Y - tgt.Y, src.Z - tgt.Z})
			n := mat.NewVecDense(3, []float64{tgt.Nx, tgt.Ny, tgt.Nz})
			residual := mat.Dot(r, n)

			// Compute Jacobian
			J := []float64{
				src.Y*tgt.Nz - src.Z*tgt.Ny, // d(res)/d(rx)
				src.Z*tgt.Nx - src.X*tgt.Nz, // d(res)/d(ry)
				src.X*tgt.Ny - src.Y*tgt.Nx, // d(res)/d(rz)
				tgt.Nx, tgt.Ny, tgt.Nz,      // d(res)/d(tx, ty, tz)
			}

			// Update A and b
			for i := 0; i < 6; i++ {
				b.SetVec(i, b.AtVec(i)-residual*J[i])
				for j := 0; j < 6; j++ {
					A.Set(i, j, A.At(i, j)+J[i]*J[j])
				}
			}
		}

		// Step 3: Solve Ax = b using least squares
		var x mat.VecDense
		if err := x.SolveVec(A, b); err != nil {
			return nil, errors.Wrap(err, "failed to solve linear system")
		}

		// Step 4: Update transformation (small-angle approximation).
		rotationUpdate := SmallAngleRotation(x.AtVec(0), x.AtVec(1), x.AtVec(2))
		// translationUpdate := &point.Point3D{X: x.AtVec(3), Y: x.AtVec(4), Z: x.AtVec(5)}

		elements := [4][4]float64{
			{rotationUpdate.At(0, 0), rotationUpdate.At(0, 1), rotationUpdate.At(0, 2), x.AtVec(3)},
			{rotationUpdate.At(1, 0), rotationUpdate.At(1, 1), rotationUpdate.At(1, 2), x.AtVec(4)},
			{rotationUpdate.At(2, 0), rotationUpdate.At(2, 1), rotationUpdate.At(2, 2), x.AtVec(5)},
			{0, 0, 0, 1},
		}

		tform := transform.NewMatrix4FromElements(elements)

		TransformPoints(transformed, tform)

		// Update our transform.
		finalTransform = finalTransform.Dot(tform)

		// Step 5: Check convergence
		if isWithinThreshold(tform, params.Tolerance) {
			break
		}
	}

	result := &Result{
		FinalTransform:    finalTransform,
		TransformedPoints: transformed,
		ElapsedTime:       time.Since(startTime),
		NumTargetPoints:   target.Len(),
		NumSourcePoints:   target.Len(),
	}

	return result, nil
}

// SmallAngleRotation creates a small rotation matrix using Rodrigues' formula.
func SmallAngleRotation(rx, ry, rz float64) *mat.Dense {
	theta := math.Sqrt(rx*rx + ry*ry + rz*rz)
	if theta < 1e-6 {
		return mat.NewDense(3, 3, []float64{1, 0, 0, 0, 1, 0, 0, 0, 1})
	}

	ux, uy, uz := rx/theta, ry/theta, rz/theta
	cosT, sinT := math.Cos(theta), math.Sin(theta)

	return mat.NewDense(3, 3, []float64{
		cosT + ux*ux*(1-cosT), ux*uy*(1-cosT) - uz*sinT, ux*uz*(1-cosT) + uy*sinT,
		uy*ux*(1-cosT) + uz*sinT, cosT + uy*uy*(1-cosT), uy*uz*(1-cosT) - ux*sinT,
		uz*ux*(1-cosT) - uy*sinT, uz*uy*(1-cosT) + ux*sinT, cosT + uz*uz*(1-cosT),
	})
}
