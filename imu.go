// AHRS Attitude-Heading Reference Systems are used to
// estimate the position and orientation of a body
// in 3D space (such as a drone or helicopter). These
// usually consist in an accelerometer, gyroscope and
// the occasionaly magnetometer.

package ahrs

// IMU represents an IMU or INS sensor such as
// the MPU6050 or MPU6250
type IMU interface {
	// Acceleration returns sensor accelerations in micro gravities
	Acceleration() (ax, ay, az int32)
	// AngularVelocity returns sensor angular velocity in micro radians
	AngularVelocity() (gx, gy, gz int32)
}

// IMUHeading represents a group of sensors such as the MPU9250
// which have accelerometer, gyroscope and magnetometer.
// These sensors are ideal for sensor fusion techniques
// such as Madgwick and Mahony algorithms.
type IMUHeading interface {
	IMU
	// North returns the direction of the measured magnetic field
	// in nanoteslas.
	North() (mx, my, mz int32)
}
