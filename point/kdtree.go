package point

import "gonum.org/v1/gonum/spatial/kdtree"

// Pointd3D :: kdtree.Comparable
var _ kdtree.Comparable = &Point3D{}

// Compare returns the signed distance of a from the plane passing through
// b and perpendicular to the dimension d.
//
// Given c = a.Compare(b, d):
//
//	c = a_d - b_d
func (p *Point3D) Compare(c kdtree.Comparable, d kdtree.Dim) float64 {
	q := c.(*Point3D)
	switch d {
	case 0:
		return p.X - q.X
	case 1:
		return p.Y - q.Y
	case 2:
		return p.Z - q.Z
	default:
		return 0
	}
}

// Dims returns the number of dimensions described in the Comparable.
func (p *Point3D) Dims() int {
	return 3
}

// Distance returns the squared Euclidean distance between the receiver and the parameter.
func (p *Point3D) Distance(c kdtree.Comparable) float64 {
	q := c.(*Point3D)
	xDelta := p.X - q.X
	yDelta := p.Y - q.Y
	zDelta := p.Z - q.Z
	return xDelta*xDelta + yDelta*yDelta + zDelta*zDelta
}
