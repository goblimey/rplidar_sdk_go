package main

import (
	"time"

	"github.com/goblimey/rplidar_sdk_go/driver"
)

func main() {
	// driver := driver.ConcreteRPlidarDriver.MakeDriver("/dev/ttyUSB0")

	driver := driver.MakeDriver("/dev/ttyUSB0")

	defer driver.DestroyDriver()

	driver.CheckRPLIDARHealth()

	driver.StartMotor()

	time.Sleep(time.Second * 10)

	driver.StopMotor()

	//
}
