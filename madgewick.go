package ahrs

import (
	"math"

	"gonum.org/v1/gonum/num/quat"
)

type MadgwickFilter struct {
	Quaternion [4]float64
	Beta       float64
}

func NewMadgwickFilter(beta float64) *MadgwickFilter {
	return &MadgwickFilter{
		Quaternion: [4]float64{1, 0, 0, 0},
		Beta:       beta,
	}
}

func (mf *MadgwickFilter) UpdateARS(ax, ay, az, gx, gy, gz, samplePeriod float64) {
	q1, q2, q3, q4 := mf.Quaternion[0], mf.Quaternion[1], mf.Quaternion[2], mf.Quaternion[3]
	var norm, s1, s2, s3, s4, qDot1, qDot2, qDot3, qDot4 float64

	_2q1 := 2 * q1
	_2q2 := 2 * q2
	_2q3 := 2 * q3
	_2q4 := 2 * q4
	_4q1 := 4 * q1
	_4q2 := 4 * q2
	_4q3 := 4 * q3
	_8q2 := 8 * q2
	_8q3 := 8 * q3
	q1q1 := q1 * q1
	q2q2 := q2 * q2
	q3q3 := q3 * q3
	q4q4 := q4 * q4

	norm = math.Sqrt(ax*ax + ay*ay + az*az)
	if norm == 0 {
		return
	}
	norm = 1 / norm
	ax *= norm
	ay *= norm
	az *= norm

	s1 = _4q1*q3q3 + _2q3*ax + _4q1*q2q2 - _2q2*ay
	s2 = _4q2*q4q4 - _2q4*ax + 4*q1q1*q2 - _2q1*ay - _4q2 + _8q2*q2q2 + _8q2*q3q3 + _4q2*az
	s3 = 4*q1q1*q3 + _2q1*ax + _4q3*q4q4 - _2q4*ay - _4q3 + _8q3*q2q2 + _8q3*q3q3 + _4q3*az
	s4 = 4*q2q2*q4 - _2q2*ax + 4*q3q3*q4 - _2q3*ay

	norm = 1 / math.Sqrt(s1*s1+s2*s2+s3*s3+s4*s4)
	s1 *= norm
	s2 *= norm
	s3 *= norm
	s4 *= norm

	qDot1 = 0.5*(-q2*gx-q3*gy-q4*gz) - mf.Beta*s1
	qDot2 = 0.5*(q1*gx+q3*gz-q4*gy) - mf.Beta*s2
	qDot3 = 0.5*(q1*gy-q2*gz+q4*gx) - mf.Beta*s3
	qDot4 = 0.5*(q1*gz+q2*gy-q3*gx) - mf.Beta*s4

	q1 += qDot1 * samplePeriod
	q2 += qDot2 * samplePeriod
	q3 += qDot3 * samplePeriod
	q4 += qDot4 * samplePeriod

	norm = 1 / math.Sqrt(q1*q1+q2*q2+q3*q3+q4*q4)
	mf.Quaternion[0] = q1 * norm
	mf.Quaternion[1] = q2 * norm
	mf.Quaternion[2] = q3 * norm
	mf.Quaternion[3] = q4 * norm
}

func (mf *MadgwickFilter) GetQuaternion() quat.Number {
	return quat.Number{
		Real: mf.Quaternion[0],
		Imag: mf.Quaternion[1],
		Jmag: mf.Quaternion[2],
		Kmag: mf.Quaternion[3],
	}
}
