package ahrs

import (
	"math"

	"gonum.org/v1/gonum/num/quat"
	"gonum.org/v1/gonum/spatial/r3"
)

type EulerAngles struct {
	Q     float64
	R     float64
	S     float64
	Order RotationOrder
}

func NormalizeQuaternion(q quat.Number) (normalized quat.Number) {
	// Can be optimized with fast inverse sqrt algorithm
	magnitudeRecip := 1 / math.Sqrt(q.Real*q.Real+q.Imag*q.Imag+q.Jmag*q.Jmag+q.Kmag*q.Kmag)
	normalized.Real = q.Real * magnitudeRecip
	normalized.Imag = q.Imag * magnitudeRecip
	normalized.Jmag = q.Jmag * magnitudeRecip
	normalized.Kmag = q.Kmag * magnitudeRecip
	return normalized
}

func mulQuatVec(q quat.Number, v r3.Vec) (result quat.Number) {
	result.Real = -q.Imag*v.X - q.Jmag*v.Y - q.Kmag*v.Z
	result.Imag = q.Real*v.X + q.Jmag*v.Z - q.Kmag*v.Y
	result.Jmag = q.Real*v.Y - q.Imag*v.Z + q.Kmag*v.X
	result.Kmag = q.Real*v.Z + q.Imag*v.Y - q.Jmag*v.X
	return result
}
