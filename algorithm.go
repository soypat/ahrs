package ahrs

import (
	"gonum.org/v1/gonum/num/quat"
	"gonum.org/v1/gonum/spatial/r3"
)

var (
	initialGain = 10.0
	// Seconds
	initializationPeriod = 3.
	quatIdentity         = quat.Number{1, 0, 0, 0}
)

// Taken shamelessly from xioTechnologies/Fusion on github.
type FusionAHRS struct {
	gain float64
	// Magnetic field limits (squared)
	maxMFS, minMFS float64
	attitude       quat.Number
	acceleration   r3.Vec
	rampedGain     float64
	zeroYawPending bool
	ahrs           AHRS
}

func (f *FusionAHRS) Init(gain float64, ahrs AHRS) error {
	f.gain = gain
	f.maxMFS = 1e200
	f.attitude = quatIdentity
	f.rampedGain = initialGain
	f.ahrs = ahrs
	return nil
}
func (f *FusionAHRS) getVectors() (accel, rot, magnet r3.Vec) {
	ax, ay, az := f.ahrs.Acceleration()
	gx, gy, gz := f.ahrs.AngularVelocity()
	mx, my, mz := f.ahrs.North()
	accel = scaledVecFromInt(1./1000., ax, ay, az)
	rot = scaledVecFromInt(1./1000., gx, gy, gz)
	magnet = scaledVecFromInt(1, mx, my, mz)
	return accel, rot, magnet
}
func (f *FusionAHRS) SetGain(gain float64) { f.gain = gain }

func (f *FusionAHRS) SetMagneticField(min, max float64) {
	f.minMFS = min * min
	f.maxMFS = max * max
}

func (f *FusionAHRS) Update(samplePeriod float64) {
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
	if mfs < f.minMFS || mfs > f.maxMFS {
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
	f.attitude = quat.Add(f.attitude, MulQuatVec(f.attitude, r3.Scale(samplePeriod, halfGyro)))

	// Normalize quaternion
	f.attitude = NormalizeQuaternion(f.attitude)

	// Calculate linear acceleration
	gravity := r3.Vec{
		X: 2.0 * (q.Imag*q.Kmag - q.Real*q.Jmag),
		Y: 2.0 * (q.Real*q.Imag + q.Jmag*q.Kmag),
		Z: 2.0 * (q.Real*q.Real - 0.5 + q.Kmag*q.Kmag),
	}
	f.acceleration = r3.Sub(accel, gravity)
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

func (f *FusionAHRS) GetQuaternion() quat.Number {
	return f.attitude
}

func (f *FusionAHRS) GetLinearAcceleration() r3.Vec {
	return f.acceleration
}
