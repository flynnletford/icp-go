package icp

import (
	"fmt"

	"github.com/flynnletford/icp-go/point"
	"gonum.org/v1/gonum/mat"
	"gonum.org/v1/gonum/spatial/kdtree"
)

// ComputeNormals calculates normals using PCA on k-nearest neighbors.
func ComputeNormals(tree *kdtree.Tree, points *point.Points3D, k int) error {

	for _, p := range points.Raw() {
		nKeeper := kdtree.NewNKeeper(k + 1) // +1 to include the point itself
		tree.NearestSet(nKeeper, p)

		// Collect k neighbors
		neighbors := make([]*point.Point3D, 0, k)
		for _, item := range nKeeper.Heap[1:] { // Skip first (itself)
			neighbors = append(neighbors, item.Comparable.(*point.Point3D))
		}

		// Compute covariance matrix
		cov := mat.NewDense(3, 3, nil)
		cx, cy, cz := pointsMean(neighbors)
		for _, n := range neighbors {
			dx, dy, dz := n.X-cx, n.Y-cy, n.Z-cz
			cov.Set(0, 0, cov.At(0, 0)+dx*dx)
			cov.Set(0, 1, cov.At(0, 1)+dx*dy)
			cov.Set(0, 2, cov.At(0, 2)+dx*dz)
			cov.Set(1, 1, cov.At(1, 1)+dy*dy)
			cov.Set(1, 2, cov.At(1, 2)+dy*dz)
			cov.Set(2, 2, cov.At(2, 2)+dz*dz)
		}
		cov.Set(1, 0, cov.At(0, 1))
		cov.Set(2, 0, cov.At(0, 2))
		cov.Set(2, 1, cov.At(1, 2))

		// Compute SVD.
		var svd mat.SVD
		if ok := svd.Factorize(cov, mat.SVDFull); !ok {
			return fmt.Errorf("failed to compute SVD")
		}
		U := mat.NewDense(3, 3, nil)
		svd.UTo(U)

		// Normal is the eigenvector with smallest singular value (last column)
		p.Nx = U.At(0, 2)
		p.Ny = U.At(1, 2)
		p.Nz = U.At(2, 2)
	}

	return nil
}

func pointsMean(points []*point.Point3D) (float64, float64, float64) {
	var cx, cy, cz float64
	n := float64(len(points))
	for _, p := range points {
		cx += p.X
		cy += p.Y
		cz += p.Z
	}
	return cx / n, cy / n, cz / n
}
