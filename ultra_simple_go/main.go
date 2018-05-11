package main

import (
	"C"
	"log"
	"time"

	"github.com/goblimey/rplidar_sdk_go/driver"
	USBDriver "github.com/goblimey/rplidar_sdk_go/usb_driver"
)
import "flag"

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
	usbDriver := USBDriver.MakeDriver(verbose)

	defer tidyup(usbDriver)

	opResult := usbDriver.Connect(port, speed, 0)

	if driver.IsFail(opResult) {
		log.Printf("failed to connect - %s", driver.OpResultToString(opResult))
		return
	}

	healthInfo, opResult := usbDriver.GetHealth(1000)
	if driver.IsFail(opResult) {
		log.Printf("failed to get health info - %s", driver.OpResultToString(opResult))
		return
	}

	if healthInfo.Status != driver.RRplidarStatusOK {
		log.Printf("health status - %d", healthInfo.ErrorCode)
		return
	}

	log.Printf("health status OK")

	devInfo, opResult := usbDriver.GetDeviceInfo(1000)
	if driver.IsFail(opResult) {
		log.Printf("failed to get device info - %s", driver.OpResultToString(opResult))
		return
	}

	log.Printf("%s", devInfo.SerialNumber)

	usbDriver.StartMotor()

	time.Sleep(time.Second * 10)

	usbDriver.StartScan(false, false)

	time.Sleep(time.Second * 5)

	opResult, sortedData := usbDriver.GrabAndSortScanData(720, 1000)

	if driver.IsFail(opResult) {
		log.Printf("failed to grab data - %s", driver.OpResultToString(opResult))
	}

	log.Printf("main: %d scans", len(sortedData))

	for i, node := range sortedData {
		syncBit := " "
		if node.SyncBit() {
			syncBit = "S"
		}
		checkBit := " "
		if node.CheckBit() {
			checkBit = "C"
		}
		log.Printf("main: %3d: %s %s theta: %3d %2d %03.2f Dist: %4d %2d %08.2f Q: %2d\n",
			i, syncBit, checkBit,
			node.AngleAsInt()>>6, node.AngleAsInt()&0x3f, node.AngleAsFloat32(),
			node.DistanceQ2>>2, node.DistanceQ2&0x4, node.DistanceAsFloat32(), node.Quality())
	}
}

func tidyup(usbDriver driver.Driver) {
	usbDriver.StopMotor()
	usbDriver.DestroyDriver()
}
