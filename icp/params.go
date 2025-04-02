package icp

type Params struct {
	MaxIterations int           `json:"maxIterations"`
	Tolerance     float64       `json:"tolerance"`
	FilterParams  *FilterParams `json:"filterParams"`

	// Source and target points will not be matched during correspondence finding if their distance exceeds this value.
	MaxCorrespondenceDistance float64 `json:"maxCorrespondenceDistance"`

	// Number of neighbors to consider when computing normals.
	// Smaller values: 10-20 will result in maintaining sharp features. More prone to noise.
	// Larger values: 30-50 will result in smoother surfaces. Less prone to noise at the cost of blurring features and computational load.
	NumNeighborsNormals int `json:"numNeighborsNormals"`
}

type FilterParams struct {
	// The dimensions of the cube which we use to voxel downsample the input point cloud.
	VoxelSize float64 `json:"voxelSize"`

	// We average the distance from each point to the specfied number of neighbors.
	// A higher value will result in more aggressive outlier removal.
	NumNeighbors int `json:"numNeighbors"`

	// We calcaulte the mean and standard deviation of the distribution of distances from each point to its N nearest neighbors.
	// ZThreshold represents the number of standard deviations a point must be from the mean before we consider it an outlier.
	// A higher value will result in more aggressive outlier removal.
	ZThreshold float64 `json:"zTreshold"`
}

var DefaultParams *Params = &Params{
	MaxIterations:             100,
	Tolerance:                 1e-4,
	MaxCorrespondenceDistance: 2.0,
	NumNeighborsNormals:       30, // 30 seems good.
	FilterParams:              DefaultFilterParams,
}

var DefaultFilterParams *FilterParams = &FilterParams{
	VoxelSize:    0.02,
	NumNeighbors: 20,
	ZThreshold:   1.5,
}
