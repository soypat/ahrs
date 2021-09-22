package ahrs

import (
	"math"

	"gonum.org/v1/gonum/num/quat"
	"gonum.org/v1/gonum/spatial/r3"
)

type RotationMatrix struct {
	xx, xy, xz float64
	yx, yy, yz float64
	zx, zy, zz float64
}

func RotationMatrixFromQuat(q quat.Number) (r RotationMatrix) {
	qwqw := q.Real * q.Real
	qwqx := q.Real * q.Imag
	qwqy := q.Real * q.Jmag
	qwqz := q.Real * q.Kmag
	qxqy := q.Imag * q.Jmag
	qxqz := q.Imag * q.Kmag
	qyqz := q.Jmag * q.Kmag
	r.xx = 2 * (qwqw - .5*q.Imag*q.Imag)
	r.xy = 2 * (qxqy + qwqz)
	r.xz = 2.0 * (qxqz - qwqy)
	r.yx = 2.0 * (qxqy - qwqz)
	r.yy = 2.0 * (qwqw - 0.5 + q.Jmag*q.Jmag)
	r.yz = 2.0 * (qyqz + qwqx)
	r.zx = 2.0 * (qxqz + qwqy)
	r.zy = 2.0 * (qyqz - qwqx)
	r.zz = 2.0 * (qwqw - 0.5 + q.Kmag*q.Kmag)
	return r
}

func (r *RotationMatrix) MulVec(v r3.Vec) (result r3.Vec) {
	result.X = r.xx*v.X + r.xy*v.Y + r.xz*v.Z
	result.Y = r.yx*v.X + r.yy*v.Y + r.yz*v.Z
	result.Z = r.zx*v.X + r.zy*v.Y + r.zz*v.Z
	return result
}

// Mul Calculates A*B
func (A *RotationMatrix) Mul(B *RotationMatrix) (result RotationMatrix) {
	result.xx = A.xx*B.xx + A.xy*B.yx + A.xz*B.zx
	result.xy = A.xx*B.xy + A.xy*B.yy + A.xz*B.zy
	result.xz = A.xx*B.xz + A.xy*B.yz + A.xz*B.zz
	result.yx = A.yx*B.xx + A.yy*B.yx + A.yz*B.zx
	result.yy = A.yx*B.xy + A.yy*B.yy + A.yz*B.zy
	result.yz = A.yx*B.xz + A.yy*B.yz + A.yz*B.zz
	result.zx = A.zx*B.xx + A.zy*B.yx + A.zz*B.zx
	result.zy = A.zx*B.xy + A.zy*B.yy + A.zz*B.zy
	result.zz = A.zx*B.xz + A.zy*B.yz + A.zz*B.zz
	return result
}

type RotationOrder int

const (
	orderUndefined RotationOrder = iota
	OrderXYZ
	OrderYXZ
	OrderZXY
	OrderZYX
	OrderYZX
	OrderXZY
	orderLen
)

// Euler angle calculations from
// https://github.com/mrdoob/three.js/blob/8ff5d832eedfd7bc698301febb60920173770899/src/math/Euler.js

// TaitBryan returns intrinsic angles of a TaitBryan rotation.
// Assumes the upper 3x3 of r is a pure rotation matrix (i.e, unscaled)
func (r *RotationMatrix) TaitBryan(order RotationOrder) (taitBryanAngles EulerAngles) {
	const lim1 = 0.9999999
	taitBryanAngles.Order = order
	switch order {

	case OrderXYZ:
		taitBryanAngles.R = math.Asin(clamp(r.xz, -1, 1))
		if math.Abs(r.xz) < lim1 {
			taitBryanAngles.Q = math.Atan2(-r.yz, r.zz)
			taitBryanAngles.S = math.Atan2(-r.xy, r.xx)
		} else {
			taitBryanAngles.Q = math.Atan2(r.zy, r.yy)
			taitBryanAngles.S = 0
		}

	case OrderYXZ:
		taitBryanAngles.Q = math.Asin(-clamp(r.yz, -1, 1))
		if math.Abs(r.yz) < lim1 {
			taitBryanAngles.R = math.Atan2(r.xz, r.zz)
			taitBryanAngles.S = math.Atan2(r.yx, r.yy)
		} else {
			taitBryanAngles.R = math.Atan2(-r.zx, r.xx)
			taitBryanAngles.S = 0
		}

	case OrderZXY:
		taitBryanAngles.Q = math.Asin(clamp(r.zy, -1, 1))
		if math.Abs(r.zy) < lim1 {
			taitBryanAngles.R = math.Atan2(-r.zx, r.zz)
			taitBryanAngles.S = math.Atan2(-r.xy, r.yy)
		} else {
			taitBryanAngles.R = 0
			taitBryanAngles.S = math.Atan2(r.yx, r.xx)
		}

	case OrderZYX:
		taitBryanAngles.R = math.Asin(-clamp(r.zx, -1, 1))
		if math.Abs(r.zx) < lim1 {
			taitBryanAngles.Q = math.Atan2(r.zy, r.zz)
			taitBryanAngles.S = math.Atan2(r.yx, r.xx)
		} else {
			taitBryanAngles.Q = 0
			taitBryanAngles.S = math.Atan2(-r.xy, r.yy)
		}

	case OrderYZX:
		taitBryanAngles.S = math.Asin(clamp(r.yx, -1, 1))
		if math.Abs(r.yx) < lim1 {
			taitBryanAngles.Q = math.Atan2(-r.yz, r.yy)
			taitBryanAngles.R = math.Atan2(-r.zx, r.xx)
		} else {
			taitBryanAngles.Q = 0
			taitBryanAngles.R = math.Atan2(r.xz, r.zz)
		}

	case OrderXZY:
		taitBryanAngles.S = math.Asin(-clamp(r.xy, -1, 1))
		if math.Abs(r.xy) < lim1 {
			taitBryanAngles.Q = math.Atan2(r.zy, r.yy)
			taitBryanAngles.R = math.Atan2(r.xz, r.xx)
		} else {
			taitBryanAngles.Q = math.Atan2(-r.yz, r.zz)
			taitBryanAngles.R = 0
		}

	default:
		panic("undefined or unimplemented rotation order")
	}

	// Invert result since three.js has different rotation convention
	taitBryanAngles.Q *= -1
	taitBryanAngles.R *= -1
	taitBryanAngles.S *= -1
	return taitBryanAngles
}

func clamp(v, min, max float64) float64 {
	return math.Max(min, math.Min(max, v))
}
