package ahrs

import (
	"github.com/go-gl/mathgl/mgl32"
)

// NewXioAHRS instances a AHRS system with a IMU+heading sensor.
// Subsequent calls to Update read from IMU.
func NewXioAHRS32(gain float64, imuWithMagnetometer IMUHeading) *XioAHRS32 {
	f := NewXioARS32(gain, imuWithMagnetometer)
	f.ahrs = imuWithMagnetometer
	return f
}

// NewFusionAHRS instances a AHRS system with only IMU sensor readings.
// Calls to Update read from IMU.
func NewXioARS32(gain float64, imu IMU) *XioAHRS32 {
	println("warning: running untested code XioARS32")
	if imu == nil {
		panic("nil IMU in NewFusionAHRS")
	}
	f := &XioAHRS32{
		gain:       float32(gain),
		maxMFS:     1e20,
		attitude:   mgl32.QuatIdent(),
		rampedGain: float32(initialGain),
		ars:        imu,
	}
	if !(gain > 0) {
		f.gain = initialGain
	}
	return f
}

// Taken shamelessly from xioTechnologies/Fusion on github.
type XioAHRS32 struct {
	gain float32
	// Magnetic field limits (squared)
	maxMFS, minMFS float32

	attitude     mgl32.Quat
	acceleration mgl32.Vec3
	rampedGain   float32
	ahrs         IMUHeading
	ars          IMU
}

func (f *XioAHRS32) getVectors() (accel, rot, magnet mgl32.Vec3) {
	ax, ay, az := f.ars.Acceleration()
	gx, gy, gz := f.ars.AngularVelocity()

	accel = scaledVec32FromInt(1e-6, ax, ay, az)
	rot = scaledVec32FromInt(1e-6, gx, gy, gz)
	if f.ahrs == nil {
		magnet = mgl32.Vec3{0: 1} // prevent singularities
		return
	}
	mx, my, mz := f.ahrs.North()
	magnet = scaledVec32FromInt(1, mx, my, mz)
	return accel, rot, magnet
}

func (f *XioAHRS32) SetGain(gain float32) { f.gain = gain }

func (f *XioAHRS32) SetMagneticField(min, max float32) {
	f.minMFS = min * min
	f.maxMFS = max * max
}

// Update updates the internal quaternion
func (f *XioAHRS32) Update(samplePeriod float32) {
	q := f.attitude
	var accel, gyro, magnet mgl32.Vec3 = f.getVectors()

	// Half feedback error calculation
	var hfe, halfWest, aux, gd2 mgl32.Vec3
	// If measurement is invalid, end calculation
	var mfs float32
	if accel[0] == 0 && accel[1] == 0 && accel[2] == 0 {
		goto ENDCALC
	}

	// Calculate direction of gravity assumed by quaternion
	gd2 = mgl32.Vec3{ // half gravity
		q.X()*q.Z() - q.W*q.Y(),
		q.W*q.X() + q.Y()*q.Z(),
		q.W*q.W - .5 + q.Z()*q.Z(),
	} // equal to 3rd column of rotation matrix representation scaled by 0.5
	aux = accel.Normalize()
	hfe = aux.Cross(gd2)

	// Abandon magnetometer feedback calculation if magnetometer measurement invalid
	mfs = magnet.Dot(magnet) // Norm2 of magnet
	if f.ahrs == nil || mfs < f.minMFS || mfs > f.maxMFS {
		goto ENDCALC
	}

	// Compute direction of 'magnetic west' assumed by quaternion
	halfWest = mgl32.Vec3{
		q.X()*q.Y() + q.W*q.Z(),
		q.W*q.W - 0.5 + q.Y()*q.Y(),
		q.Y()*q.Z() - q.W*q.X(),
	} // equal to 2nd column of rotation matrix representation scaled by 0.5

	// calculate magnetometer feedback error
	aux = accel.Cross(magnet)
	hfe = hfe.Add(aux.Normalize().Cross(halfWest))

ENDCALC:

	if f.gain == 0 {
		f.rampedGain = 0
	}

	feedbackGain := f.gain
	if f.rampedGain > f.gain {
		f.rampedGain -= (initialGain - f.gain) * samplePeriod / initializationPeriod
	}
	halfGyro := gyro.Mul(0.5)

	// apply feedback to gyro
	halfGyro = halfGyro.Add(hfe.Mul(feedbackGain))
	f.attitude = f.attitude.Add(MulQuatVec32(f.attitude, halfGyro.Mul(samplePeriod)))

	// Normalize quaternion
	f.attitude = f.attitude.Normalize()

	// Calculate linear acceleration
	gravity := mgl32.Vec3{
		2.0 * (q.X()*q.Z() - q.W*q.Y()),
		2.0 * (q.W*q.X() + q.Y()*q.Z()),
		2.0 * (q.W*q.W - 0.5 + q.Z()*q.Z()),
	}
	f.acceleration = accel.Sub(gravity)

	// no magnetometer correction discards change in Yaw.
	if f.ahrs == nil {
		f.setYaw(0)
	}
}

func (f *XioAHRS32) setYaw(yaw float32) {
	q := f.GetQuaternion()
	// Calculate inverse yaw
	iyaw := atan2_32(q.X()*q.Y()+q.W*q.Z(), q.W*q.W-.5+q.X()*q.X()) // Euler angle of conjugate
	//half inverse yaw minus offset?
	hiymo := 0.5 * (iyaw - yaw)
	iyawQuat := mgl32.Quat{
		W: cos_32(hiymo),
	}
	iyawQuat.V[2] = -sin_32(hiymo)
	f.attitude = iyawQuat.Mul(f.attitude)
}

// func scaledVecFromInt(scale float64, x, y, z int32) (result r3.Vec) {
// 	result.X = scale * float64(x)
// 	result.Y = scale * float64(y)
// 	result.Z = scale * float64(z)
// 	return result
// }

// func r3Normalize(v r3.Vec) (normalized r3.Vec) {
// 	normalized = v.Scale(1 / r3.Norm(v))
// 	return normalized
// }

func (f *XioAHRS32) GetQuaternion() mgl32.Quat {
	return f.attitude
}

// func (f *FusionAHRS) GetEulerAngles() r3.Vec {
// 	return quatToEuler(f.attitude)
// }

func (f *XioAHRS32) GetLinearAcceleration() mgl32.Vec3 {
	return f.acceleration
}
