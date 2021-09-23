# ahrs
Estimation algorithm based off xioTechnologies Fusion library for Go.


```go
package main

import (
	"fmt"
	"time"

	"github.com/soypat/ahrs"
)

func main() {
    imu := YourIMUInterface()
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
```