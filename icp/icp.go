package icp

import (
	"time"

	"github.com/pkg/errors"
	"github.com/team-rocos/go-common/transform"

	"github.com/flynnletford/icp-go/point"
	"gonum.org/v1/gonum/mat"
	"gonum.org/v1/gonum/spatial/kdtree"
)

type Result struct {
	FinalTransform    *transform.Matrix4 `json:"finalTransform"`
	TransformedPoints *point.Points3D    `json:"transformedPoints"`
	ElapsedTime       time.Duration      `json:"elapsedTime"`
	NumTargetPoints   int                `json:"numTargetPoints"`
	NumSourcePoints   int                `json:"numSourcePoints"`
}

func ICPRefine(source *point.Points3D, target *point.Points3D, params *Params) (*Result, error) {

	startTime := time.Now()

	planeResult, err := PointToPlane(source.Copy(), target.Copy(), DefaultParams)
	if err != nil {
		return nil, err
	}

	pointResult, err := PointToPoint(planeResult.TransformedPoints.Copy(), target.Copy(), DefaultParams)
	if err != nil {
		return nil, err
	}

	planeTform := planeResult.FinalTransform
	pointTform := pointResult.FinalTransform

	totalTransform := planeTform.Dot(pointTform)

	result := &Result{
		FinalTransform:    totalTransform,
		TransformedPoints: pointResult.TransformedPoints,
		ElapsedTime:       time.Since(startTime),
		NumTargetPoints:   target.Len(),
		NumSourcePoints:   target.Len(),
	}

	return result, nil
}

func PointToPoint(source *point.Points3D, target *point.Points3D, params *Params) (*Result, error) {

	startTime := time.Now()

	_, transformed := Filter(source, params)
	targetTree, _ := Filter(target, params)

	// Initialise our final transform calculated.
	finalTransform := transform.Matrix4Identity()

	for i := 0; i < params.MaxIterations; i++ {

		// TODO: only transform points inside closest points as required.
		closest, _ := closestPoints(transformed, targetTree)

		tform, err := computeOptimalTransform(transformed, closest, params.MaxCorrespondenceDistance)
		if err != nil {
			return nil, err
		}

		TransformPoints(transformed, tform)

		// Update our transform.
		finalTransform = finalTransform.Dot(tform)

		// Check if our current transform is within the tolerance.
		if isWithinThreshold(tform, params.Tolerance) {
			break
		}
	}

	result := &Result{
		FinalTransform:    finalTransform,
		TransformedPoints: transformed,
		ElapsedTime:       time.Since(startTime),
		NumTargetPoints:   target.Len(),
		NumSourcePoints:   source.Len(),
	}

	return result, nil
}

func computeCentroid(points *point.Points3D) *point.Point3D {
	var sumX, sumY, sumZ float64
	n := float64(len(points.Raw()))

	for _, p := range points.Raw() {
		sumX += p.X
		sumY += p.Y
		sumZ += p.Z
	}

	return &point.Point3D{X: sumX / n, Y: sumY / n, Z: sumZ / n}
}

func closestPoints(source *point.Points3D, target *kdtree.Tree) (*point.Points3D, float64) {
	closest := make([]*point.Point3D, source.Len())

	sum := 0.0
	for i, sourcePoint := range source.Raw() {
		closestPoint, dist := closestPoint(sourcePoint, target)
		closest[i] = closestPoint
		sum += dist
	}

	sum /= float64(source.Len())

	closestPoints := point.Points3D(closest)
	return &closestPoints, sum
}

func closestPoint(p *point.Point3D, points *kdtree.Tree) (*point.Point3D, float64) {
	val, dist := points.Nearest(p)
	return val.(*point.Point3D), dist
}

func computeOptimalTransform(source *point.Points3D, closestTargetPoints *point.Points3D, maxCorrespondenceDistance float64) (*transform.Matrix4, error) {
	centroidSource := computeCentroid(source)
	centroidTarget := computeCentroid(closestTargetPoints)

	H := mat.NewDense(3, 3, nil)

	numValid := 0

	for i := range source.Raw() {

		src := source.Raw()[i]
		tgt := closestTargetPoints.Raw()[i]

		// Skip if the distance between the source and target points is too large.
		if src.Distance(tgt) > maxCorrespondenceDistance {
			continue
		}

		numValid++

		s := src.Subtract(centroidSource).ToArray()
		t := tgt.Subtract(centroidTarget).ToArray()

		for j := 0; j < 3; j++ {
			for k := 0; k < 3; k++ {
				H.Set(j, k, H.At(j, k)+t[j]*s[k])
			}
		}
	}

	if numValid == 0 {
		return nil, errors.New("no valid correspondences found")
	}

	var svd mat.SVD
	if ok := svd.Factorize(H, mat.SVDThin); !ok {
		return nil, errors.New("failed to factorize matrix")
	}

	U := mat.NewDense(3, 3, nil)
	V := mat.NewDense(3, 3, nil)
	svd.UTo(U)
	svd.VTo(V)
	VT := mat.NewDense(3, 3, nil)
	VT.CloneFrom(V.T())

	R := mat.NewDense(3, 3, nil)
	R.Mul(U, VT)

	// Compute determinant of R
	detR := mat.Det(R)

	// If det(R) < 0, flip the sign of the last column of U
	if detR < 0 {
		for i := 0; i < 3; i++ {
			U.Set(i, 2, -U.At(i, 2)) // Flip last column of U
		}

		// Recompute R = U * VT
		R.Mul(U, VT)
	}

	tx := centroidTarget.X - (R.At(0, 0)*centroidSource.X + R.At(0, 1)*centroidSource.Y + R.At(0, 2)*centroidSource.Z)
	ty := centroidTarget.Y - (R.At(1, 0)*centroidSource.X + R.At(1, 1)*centroidSource.Y + R.At(1, 2)*centroidSource.Z)
	tz := centroidTarget.Z - (R.At(2, 0)*centroidSource.X + R.At(2, 1)*centroidSource.Y + R.At(2, 2)*centroidSource.Z)

	return transform.NewMatrix4FromSlice([]float64{
		R.At(0, 0), R.At(0, 1), R.At(0, 2), tx,
		R.At(1, 0), R.At(1, 1), R.At(1, 2), ty,
		R.At(2, 0), R.At(2, 1), R.At(2, 2), tz,
		0, 0, 0, 1,
	})
}

func TransformPoints(points *point.Points3D, tform *transform.Matrix4) {

	// If the transform is the identity matrix, return early.
	if isIdentity(tform) {
		return
	}

	for i, p := range points.Raw() {

		newVec3 := tform.MulVec3(&transform.Vector3{X: p.X, Y: p.Y, Z: p.Z})

		points.Raw()[i] = &point.Point3D{
			X: newVec3.X,
			Y: newVec3.Y,
			Z: newVec3.Z,
		}
	}
}

func isWithinThreshold(tform *transform.Matrix4, threshold float64) bool {
	return tform.Translation().Length() < threshold
}

func isIdentity(tform *transform.Matrix4) bool {
	return tform.Translation().Length() == 0 && tform.Quaternion().X == 0 && tform.Quaternion().Y == 0 && tform.Quaternion().Z == 0 && tform.Quaternion().W == 1
}
