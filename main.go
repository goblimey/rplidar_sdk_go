package main

import (
	"C"
	"log"
	"time"

	"github.com/goblimey/rplidar_sdk_go/constants"
	"github.com/goblimey/rplidar_sdk_go/driver"
)

func main() {
	driver := driver.MakeDriver(true, "/dev/ttyUSB0", 0)

	defer driver.DestroyDriver()

	err := driver.Connect("/dev/ttyUSB0", 0, 0)

	if err != nil {
		return
	}

	devInfo, err := driver.GetDeviceInfo(1000)
	if err != nil {
		return
	}

	log.Printf("%s", devInfo.SerialNumber)

	if !driver.GetHealth(1000) {
		return
	}

	driver.StartMotor()

	driver.StartScan()

	time.Sleep(time.Second)

	data := driver.GrabScanData(720, 1000)

	driver.StopMotor()

	sortedData, err := driver.AscendScanData(data)

	driver.FreeRplidarScanResultsPacked(data)

	/*
			printf("%s theta: %03.2f Dist: %08.2f Q: %d \n",
		                    (nodes[pos].sync_quality & RPLIDAR_RESP_MEASUREMENT_SYNCBIT) ?"S ":"  ",
		                    (nodes[pos].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f,
		                    nodes[pos].distance_q2/4.0f,
		                    nodes[pos].sync_quality >> RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT);
	*/
	for i, node := range sortedData {
		s := " "
		if node.SyncQuality&constants.RPLIDAR_RESP_MEASUREMENT_SYNCBIT == 1 {
			s = "S"
		}
		log.Printf("%3d: %s theta: %03.2f Dist: %08.2f Q: %d \n", i, s,
			float32((node.AngleQ6Checkbit>>constants.RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0),
			float32(node.DistanceQ2/4.0),
			node.SyncQuality>>constants.RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT)
	}

	unpacked := driver.UnPack(data)

	scans := uint(unpacked.Scans)

	log.Printf("%d scans", scans)

}
