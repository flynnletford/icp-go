package main

import (
	"fmt"
	"log"

	"github.com/flynnletford/icp-go/icp"
	"github.com/flynnletford/icp-go/ply"
	"github.com/team-rocos/go-common/transform"
)

func main() {

	// Example usage of the ICP algorithm to align two point clouds.
	// We'll load in two PLY files, perform ICP, and print the results.
	// The source file was captured at x: 2m, y: 0m, z: 0m.
	// The target file was captured at x: 1m, y: 0m, z: 0m.
	sourceFile := "../pointCloudFiles/2m.ply"
	targetFile := "../pointCloudFiles/1m.ply"

	source, err := ply.Read(sourceFile, false)
	if err != nil {
		log.Fatalf("Failed to read source PLY: %v", err)
	}

	target, err := ply.Read(targetFile, false)
	if err != nil {
		log.Fatalf("Failed to read target PLY: %v", err)
	}

	result, err := icp.PointToPlane(source, target, icp.DefaultParams)
	if err != nil {
		log.Fatalf("ICP failed: %v", err)
	}

	translation := result.FinalTransform.Translation()
	yaw := transform.EulerFromQuaternion(result.FinalTransform.Quaternion()).Yaw

	fmt.Println("ICP Alignment Complete")
	fmt.Printf("Estimated Rotation Matrix:\n%+v\n", result.FinalTransform.Elements())
	fmt.Printf("Estimated Translation: X=%.3f, Y=%.4f, Z=%.4f\n", translation.X, translation.Y, translation.Z)
	fmt.Printf("Estimated Yaw: %.3f\n", yaw)
	fmt.Printf("Time Elapsed: %v\n", result.ElapsedTime)
	fmt.Printf("Num Target Points: %d\n", result.NumTargetPoints)
	fmt.Printf("Num Source Points: %d\n", result.NumSourcePoints)

	if err := ply.Write("transformed_source.ply", result.TransformedPoints); err != nil {
		log.Fatalf("Failed to write transformed points: %v", err)
	}
}
