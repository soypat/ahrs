package ahrs

import (
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
