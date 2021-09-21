package ahrs

import (
	"math"

	"gonum.org/v1/gonum/num/quat"
	"gonum.org/v1/gonum/spatial/r3"
)

// ZYX sequence in radians
type EulerAngles struct {
	Q float64
	R float64
	S float64
}

type EulerZYX struct {
	EulerAngles
}

func ZYXFromQuaternion(q quat.Number) (att EulerZYX) {
	qwqwm2 := q.Real*q.Real - 0.5
	att.Q = math.Atan2(q.Jmag*q.Kmag-q.Real*q.Imag, qwqwm2+q.Kmag*q.Kmag)
	att.R = -math.Asin(2 * (q.Imag*q.Kmag + q.Real*q.Jmag))
	att.S = math.Atan2(q.Imag*q.Jmag-q.Real*q.Kmag, qwqwm2+q.Imag*q.Imag)
	return att
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

func MulQuatVec(q quat.Number, v r3.Vec) (result quat.Number) {
	result.Real = -q.Imag*v.X - q.Jmag*v.Y - q.Kmag*v.Z
	result.Imag = q.Real*v.X + q.Jmag*v.Z - q.Kmag*v.Y
	result.Jmag = q.Real*v.Y - q.Imag*v.Z + q.Kmag*v.X
	result.Kmag = q.Real*v.Z + q.Imag*v.Y - q.Jmag*v.X
	return result
}

// Returns R3 vector with x:Roll, y:Pitch, z:Yaw
// What is this implementation?
func quatToEuler(quat quat.Number) (euler r3.Vec) {
	w, x, y, z := quat.Real, quat.Imag, quat.Jmag, quat.Kmag //quat.W, quat.X(), quat.Y(), quat.Z()
	// roll (x-axis rotation)
	sinr_cosp := 2 * (w*x + y*z)
	cosr_cosp := 1 - 2*(x*x+y*y)
	euler.X = math.Atan2(sinr_cosp, cosr_cosp)

	// pitch (y-axis rotation)
	sinp := 2 * (w*y - z*x)
	if math.Abs(sinp) >= 1 {
		euler.Y = math.Copysign(math.Pi/2, sinp)
	} else {
		euler.Y = math.Asin(sinp)
	}

	// yaw (z-axis rotation)
	siny_cosp := 2 * (w*z + x*y)
	cosy_cosp := 1 - 2*(y*y+z*z)
	euler.Z = math.Atan2(siny_cosp, cosy_cosp)
	return euler
}
