package ahrs

import (
	"math"

	"gonum.org/v1/gonum/num/quat"
	"gonum.org/v1/gonum/spatial/r3"
)

const (
	initialGain = 10.0
	// Seconds
	initializationPeriod = 3.
)

var quatIdentity = quat.Number{Real: 1}

// NewXioAHRS instances a AHRS system with a IMU+heading sensor.
// Subsequent calls to Update read from IMU.
func NewXioAHRS(gain float64, imuWithMagnetometer IMUHeading) *XioAHRS {
	f := NewXioARS(gain, imuWithMagnetometer)
	f.ahrs = imuWithMagnetometer
	return f
}

// NewFusionAHRS instances a AHRS system with only IMU sensor readings.
// Calls to Update read from IMU.
func NewXioARS(gain float64, imu IMU) *XioAHRS {
	if imu == nil {
		panic("nil IMU in NewFusionAHRS")
	}
	f := &XioAHRS{
		gain:       gain,
		maxMFS:     1e200,
		attitude:   quatIdentity,
		rampedGain: initialGain,
		ars:        imu,
	}
	if !(gain > 0) {
		f.gain = initialGain
	}
	return f
}

// Taken shamelessly from xioTechnologies/Fusion on github.
type XioAHRS struct {
	gain float64
	// Magnetic field limits (squared)
	maxMFS, minMFS float64
	attitude       quat.Number
	acceleration   r3.Vec
	rampedGain     float64
	ahrs           IMUHeading
	ars            IMU
}

func (f *XioAHRS) getVectors() (accel, rot, magnet r3.Vec) {
	ax, ay, az := f.ars.Acceleration()
	gx, gy, gz := f.ars.AngularVelocity()

	accel = scaledVecFromInt(1e-6, ax, ay, az)
	rot = scaledVecFromInt(1e-6, gx, gy, gz)
	if f.ahrs == nil {
		magnet = r3.Vec{X: 1} // prevent singularities
		return
	}
	mx, my, mz := f.ahrs.North()
	magnet = scaledVecFromInt(1, mx, my, mz)
	return accel, rot, magnet
}

func (f *XioAHRS) SetGain(gain float64) { f.gain = gain }

func (f *XioAHRS) SetMagneticField(min, max float64) {
	f.minMFS = min * min
	f.maxMFS = max * max
}

// Update updates the internal quaternion
func (f *XioAHRS) Update(samplePeriod float64) {
	q := f.attitude
	var accel, gyro, magnet r3.Vec = f.getVectors()

	// Half feedback error calculation
	var hfe, halfWest, aux, gd2 r3.Vec
	// If measurement is invalid, end calculation
	mfs := 0.0
	if accel.X == 0 && accel.Y == 0 && accel.Z == 0 {
		goto ENDCALC
	}

	// Calculate direction of gravity assumed by quaternion
	gd2 = r3.Vec{ // half gravity
		X: q.Imag*q.Kmag - q.Real*q.Jmag,
		Y: q.Real*q.Imag + q.Jmag*q.Kmag,
		Z: q.Real*q.Real - .5 + q.Kmag*q.Kmag,
	} // equal to 3rd column of rotation matrix representation scaled by 0.5
	hfe = r3.Cross(r3Normalize(accel), gd2)

	// Abandon magnetometer feedback calculation if magnetometer measurement invalid
	mfs = r3.Norm2(magnet)
	if f.ahrs == nil || mfs < f.minMFS || mfs > f.maxMFS {
		goto ENDCALC
	}

	// Compute direction of 'magnetic west' assumed by quaternion
	halfWest = r3.Vec{
		X: q.Imag*q.Jmag + q.Real*q.Kmag,
		Y: q.Real*q.Real - 0.5 + q.Jmag*q.Jmag,
		Z: q.Jmag*q.Kmag - q.Real*q.Imag,
	} // equal to 2nd column of rotation matrix representation scaled by 0.5

	// calculate magnetometer feedback error
	aux = r3.Cross(accel, magnet)
	hfe = r3.Add(hfe, r3.Cross(r3Normalize(aux), halfWest))
ENDCALC:

	if f.gain == 0 {
		f.rampedGain = 0
	}

	feedbackGain := f.gain
	if f.rampedGain > f.gain {
		f.rampedGain -= (initialGain - f.gain) * samplePeriod / initializationPeriod
	}
	halfGyro := r3.Scale(0.5, gyro)

	// apply feedback to gyro
	halfGyro = r3.Add(halfGyro, r3.Scale(feedbackGain, hfe))
	f.attitude = quat.Add(f.attitude, mulQuatVec(f.attitude, r3.Scale(samplePeriod, halfGyro)))

	// Normalize quaternion
	f.attitude = NormalizeQuaternion(f.attitude)

	// Calculate linear acceleration
	gravity := r3.Vec{
		X: 2.0 * (q.Imag*q.Kmag - q.Real*q.Jmag),
		Y: 2.0 * (q.Real*q.Imag + q.Jmag*q.Kmag),
		Z: 2.0 * (q.Real*q.Real - 0.5 + q.Kmag*q.Kmag),
	}
	f.acceleration = r3.Sub(accel, gravity)

	// no magnetometer correction discards change in Yaw.
	if f.ahrs == nil {
		f.setYaw(0)
	}
}

func (f *XioAHRS) setYaw(yaw float64) {
	q := f.GetQuaternion()
	// Calculate inverse yaw
	iyaw := math.Atan2(q.Imag*q.Jmag+q.Real*q.Kmag, q.Real*q.Real-.5+q.Imag*q.Imag) // Euler angle of conjugate
	//half inverse yaw minus offset?
	hiymo := 0.5 * (iyaw - yaw)
	iyawQuat := quat.Number{
		Real: math.Cos(hiymo),
		Kmag: -math.Sin(hiymo),
	}
	f.attitude = quat.Mul(iyawQuat, f.attitude)
}

func scaledVecFromInt(scale float64, x, y, z int32) (result r3.Vec) {
	result.X = scale * float64(x)
	result.Y = scale * float64(y)
	result.Z = scale * float64(z)
	return result
}

func r3Normalize(v r3.Vec) (normalized r3.Vec) {
	normalized = v.Scale(1 / r3.Norm(v))
	return normalized
}

func (f *XioAHRS) GetQuaternion() quat.Number {
	return f.attitude
}

// func (f *FusionAHRS) GetEulerAngles() r3.Vec {
// 	return quatToEuler(f.attitude)
// }

func (f *XioAHRS) GetLinearAcceleration() r3.Vec {
	return f.acceleration
}
