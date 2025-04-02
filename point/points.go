package point

import "gonum.org/v1/gonum/spatial/kdtree"

type Points3D []*Point3D

func (p *Points3D) Raw() []*Point3D {
	if p == nil || len(*p) == 0 {
		return []*Point3D{}
	}
	return *p
}

func (p *Points3D) Copy() *Points3D {
	points := make(Points3D, len(*p))

	for i, point := range *p {
		points[i] = &Point3D{X: point.X, Y: point.Y, Z: point.Z}
	}

	return &points
}

// Point3D :: kdtree.Interface
var _ kdtree.Interface = &Points3D{}

// Index returns the ith element of the list of points.
func (p *Points3D) Index(i int) kdtree.Comparable {
	return (*p)[i]
}

// Len returns the length of the list.
func (p *Points3D) Len() int {
	return len(*p)
}

func (p *Points3D) Pivot(d kdtree.Dim) int {
	return plane{Points3D: p, Dim: d}.Pivot()
}

func (p *Points3D) Slice(start, end int) kdtree.Interface {
	points := *p
	if start < 0 || end > len(points) || start >= end {
		return &Points3D{}
	}
	sliced := Points3D(points[start:end])
	return &sliced
}

// plane is required to help points.
type plane struct {
	kdtree.Dim
	*Points3D
}

func (p plane) Less(i, j int) bool {
	points := *p.Points3D
	return points[i].Compare(points[j], p.Dim) < 0
}

// Slice returns a slice of the list using zero-based half open indexing equivalent to built-in slice indexing.
func (p plane) Pivot() int {
	return kdtree.Partition(p, kdtree.MedianOfMedians(p))
}

func (p plane) Slice(start, end int) kdtree.SortSlicer {
	points := *p.Points3D
	if start < 0 || end > len(points) || start >= end {
		return plane{Dim: p.Dim, Points3D: &Points3D{}}
	}

	newSlice := Points3D(points[start:end])
	return plane{Dim: p.Dim, Points3D: &newSlice}
}

func (p plane) Swap(i, j int) {

	points := *p.Points3D

	if i < 0 || i >= points.Len() {
		return
	}

	if j < 0 || j >= points.Len() {
		return
	}

	points[i], points[j] = points[j], points[i]
}
