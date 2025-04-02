package ply

import (
	"bufio"
	"errors"
	"fmt"
	"os"
	"strings"

	"github.com/flynnletford/icp-go/point"
)

func Read(filePath string, twoD bool) (*point.Points3D, error) {
	file, err := os.Open(filePath)
	if err != nil {
		return nil, err
	}
	defer file.Close()

	points := make([]*point.Point3D, 0)

	scanner := bufio.NewScanner(file)

	i := -1
	headerDone := false
	for scanner.Scan() {
		i++
		line := scanner.Text()

		if i == 0 {
			if !strings.Contains(line, "ply") {
				return nil, errors.New("incorrect header; missing ply heading")
			}
			continue
		}
		if !headerDone {
			if strings.Contains(line, "end_header") {
				headerDone = true
				continue
			} else {
				continue
			}
		}

		if len(line) == 0 {
			continue
		}

		var x, y, z float64
		if _, err := fmt.Fscanf(strings.NewReader(line), "%v %v %v", &x, &y, &z); err != nil {
			return nil, err
		}

		if twoD {
			z = 0
		}

		point := &point.Point3D{
			X: x,
			Y: y,
			Z: z,
		}

		if point.Length() < 1.5 {
			continue // Avoid robot body.
		}

		points = append(points, point)
	}
	if err := scanner.Err(); err != nil {
		return nil, err
	}

	filteredPoints := point.Points3D(points)

	return &filteredPoints, nil
}

func Write(filePath string, points *point.Points3D) error {
	os.Remove(filePath)

	newFile, err := os.Create(filePath)
	if err != nil {
		return err
	}

	if err := writePlyHeader(newFile, points.Len()); err != nil {
		return err
	}

	for _, p := range points.Raw() {
		if err := writeXYZToFile(newFile, p.X, p.Y, p.Z); err != nil {
			return err
		}
	}

	return nil
}

const plyHeaderFormat string = `ply
format ascii 1.0
element vertex %v
property float x
property float y
property float z
end_header
`

func plyHeader(numVertices int) string {
	return fmt.Sprintf(plyHeaderFormat, numVertices)
}

func writePlyHeader(file *os.File, numPoints int) error {

	if _, err := file.WriteString(plyHeader(numPoints)); err != nil {
		return err
	}

	return nil
}

func writeXYZToFile(f *os.File, x float64, y float64, z float64) error {
	if _, err := f.WriteString(fmt.Sprintf("%v %v %v\n", x, y, z)); err != nil {
		return err
	}

	return nil
}
