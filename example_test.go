package ahrs_test

import (
	"fmt"
	"time"

	"github.com/soypat/ahrs"
)

type MyIMU struct {
}

func (MyIMU) Acceleration() (ax, ay, az int32) {
	return 0, 0, 1_000_000 // 1g in z
}

func (MyIMU) AngularVelocity() (gx, gy, gz int32) {
	return 0, 0, 0 // still imu
}

func ExampleXioARS() {
	var imu MyIMU
	estimator := ahrs.NewXioARS(1, imu)
	dt := time.Second
	tick := time.NewTicker(dt)
	for range tick.C {
		estimator.Update(dt.Seconds())
		attitude := estimator.GetQuaternion()
		rot := ahrs.RotationMatrixFromQuat(attitude)

		eulerAngles := rot.TaitBryan(ahrs.OrderXYZ)
		fmt.Printf("IMU attitude: %+v", eulerAngles)
	}
}
