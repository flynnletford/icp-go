package point

import (
	"math"
)

type Point3D struct {
	X float64 `json:"x"`
	Y float64 `json:"y"`
	Z float64 `json:"z"`

	// Normals - only used if computed.
	Nx float64 `json:"nx"`
	Ny float64 `json:"ny"`
	Nz float64 `json:"nz"`
}

func (p *Point3D) Subtract(q *Point3D) *Point3D {
	return &Point3D{X: p.X - q.X, Y: p.Y - q.Y, Z: p.Z - q.Z}
}

func (p *Point3D) ToArray() []float64 {
	return []float64{p.X, p.Y, p.Z}
}

func (p *Point3D) Length() float64 {
	return math.Sqrt(p.X*p.X + p.Y*p.Y + p.Z*p.Z)
}

func (p *Point3D) Euclidean(q *Point3D) float64 {

	xDelta := p.X - q.X
	yDelta := p.Y - q.Y
	zDelta := p.Z - q.Z

	return math.Sqrt(xDelta*xDelta + yDelta*yDelta + zDelta*zDelta)
}
