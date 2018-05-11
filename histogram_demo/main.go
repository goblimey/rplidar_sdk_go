package main

import (
	"flag"
	"fmt"

	"github.com/goblimey/rplidar_sdk_go/driver"
	"github.com/goblimey/rplidar_sdk_go/utils"
)

// These values are set from the command line arguments.
var verbose bool // verbose mode
var port string  // USB port
var speed uint   // speed of the port 115200

func init() {
	flag.BoolVar(&verbose, "verbose", false, "enable verbose logging")
	flag.BoolVar(&verbose, "v", false, "enable verbose logging (shorthand)")
	flag.StringVar(&port, "port", "", "the USB port on which the RPLidar device is connected")
	flag.UintVar(&speed, "speed", 115200, "the speed of the USB port (default 115200)")
}

func main() {

	flag.Parse()

	nodes := make([]driver.RplidarResponseMeasurementNode, 6)

	nodes[0].SyncQuality = 1          // 0000 0001 - 0/1
	nodes[0].AngleQ6Checkbit = 0x21   // 0000 0000 0010 0001 - 0.5/1
	nodes[0].DistanceQ2 = 0x81        // 0000 0000 1000 1000- 32.5
	nodes[1].SyncQuality = 0x6        // 0000 00110 - 1/10
	nodes[1].AngleQ6Checkbit = 0x1e41 // 0001 1110 0100 0001 - 60.25/0
	nodes[1].DistanceQ2 = 0x81        // 0000 0000 1000 1000- 32.5
	nodes[2].SyncQuality = 0xb        // 0000 1011 - 2/01
	nodes[2].AngleQ6Checkbit = 0x3c01 // 0011 1100 0000 0001 - 120/1
	nodes[2].DistanceQ2 = 0x81        // 0000 0000 1000 1000- 32.5
	nodes[3].SyncQuality = 0x0d       // 0000 1101 - 3/01
	nodes[3].AngleQ6Checkbit = 0x5a01 // 0101 1010 0000 0001- 180.25/0
	nodes[3].DistanceQ2 = 0x0182      // 0000 0001 1000 0010 - 96.5
	nodes[4].SyncQuality = 0x12       // 0001 0010 - 4/10
	nodes[4].AngleQ6Checkbit = 0x7801 // 0111 1000 0000 0001 - 240/01
	nodes[4].DistanceQ2 = 0x0182      // 0000 0001 1000 0010 - 96.55
	nodes[5].SyncQuality = 0x15       // 0001 0101 - 5/01
	nodes[5].AngleQ6Checkbit = 0x9601 // 1001 0110 0000 0001 - 300/1 288
	nodes[5].DistanceQ2 = 0x0182      // 0000 0001 1000 0010 - 96.55

	if verbose {
		fmt.Printf("main: %d scans\n", len(nodes))
		utils.PrintNodes(nodes)
		fmt.Println("")
		fmt.Println("")
	}

	utils.PlotHistogram(nodes)
}
