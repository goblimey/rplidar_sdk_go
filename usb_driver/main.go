package USBDriver

import "github.com/goblimey/rplidar_sdk_go/driver"

func main() {

	nodes := make([]driver.RplidarResponseMeasurementNode, 6)

	// xxxx xxbb
	nodes[0].SyncQuality = 0xfd // 1111 1101 - 63/1
	// xxxx xxxx x xxx xxx x
	nodes[0].AngleQ6Checkbit = 0x2c1 // 0010 1010 0001 - 5.5/1
	// xxxx xxxx xxxx . xxxx
	nodes[0].DistanceQ2 = 0x14 // 1 0100 - 1.5

	nodes[1].SyncQuality = 0x04      // 0000 0100 - 1/0
	nodes[0].AngleQ6Checkbit = 0x210 // 0010 0001 0000 - 4.25/0
	nodes[1].DistanceQ2 = 0x0        // 0000 0000 1000 0010

	_ = MakeDriver(true)

}
