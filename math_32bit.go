package ahrs

import (
	"github.com/go-gl/mathgl/mgl32"
)

const (
	pi  = 3.14159265358979323846264338327950288419716939937510582097494459 // https://oeis.org/A000796
	pi2 = pi * pi
)

func MulQuatVec32(q mgl32.Quat, v mgl32.Vec3) (result mgl32.Quat) {
	result.W = -q.X()*v[0] - q.Y()*v[1] - q.Z()*v[2]
	result.V[0] = q.W*v[0] + q.Y()*v[2] - q.Z()*v[1]
	result.V[1] = q.W*v[1] - q.X()*v[2] + q.Z()*v[0]
	result.V[2] = q.W*v[2] + q.X()*v[1] - q.Y()*v[0]
	return result
}

func atan2_32(x, y float32) float32 {
	a := min_32(abs_32(x), abs_32(y)) / max_32(abs_32(x), abs_32(y))
	s := a * a
	r := ((-0.0464964749*s+0.15931422)*s-0.327622764)*s*a + a
	if abs_32(y) > abs_32(x) {
		r = pi/2 - r
	}
	if x < 0 {
		r = pi - r
	}
	if y < 0 {
		r = -r
	}
	return r
}

// Bhaskara I's sine approximation formula
// https://en.wikipedia.org/wiki/Bhaskara_I%27s_sine_approximation_formula
func sin_32(x float32) float32 {
	if x < 0 {
		x = abs_32(x)
		return 16 * x * (pi - x) / (5*pi2 - 4*x*(pi-x))
	}
	return 16 * x * (pi - x) / (5*pi2 - 4*x*(pi-x))
}

// Bhaskara I's cosine approximation formula
// https://en.wikipedia.org/wiki/Bhaskara_I%27s_sine_approximation_formula
func cos_32(x float32) float32 {
	x2 := x * x
	return (pi2 - 4*x2) / (pi2 + x2)
}

func abs_32(a float32) float32 {
	if a < 0 {
		return -a
	}
	return a
}

func max_32(a, b float32) float32 {
	if a > b {
		return a
	}
	return b
}

func min_32(a, b float32) float32 {
	if a < b {
		return a
	}
	return b
}

func scaledVec32FromInt(scale float32, x, y, z int32) (result mgl32.Vec3) {
	result[0] = scale * float32(x)
	result[1] = scale * float32(y)
	result[2] = scale * float32(z)
	return result
}
