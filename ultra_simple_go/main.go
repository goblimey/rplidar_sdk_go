package main

import (
	"C"
	"log"
	"time"

	"github.com/goblimey/rplidar_sdk_go/driver"
	USBDriver "github.com/goblimey/rplidar_sdk_go/usb_driver"
	"github.com/goblimey/rplidar_sdk_go/utils"
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

	opResult, sortedData := usbDriver.GrabAndSortScanData(720, 10000)

	if driver.IsFail(opResult) {
		log.Printf("failed to grab data - %s", driver.OpResultToString(opResult))
	}

	log.Printf("main: %d scans", len(sortedData))

	utils.PrintNodes(sortedData)
}

func tidyup(usbDriver driver.Driver) {
	usbDriver.StopMotor()
	usbDriver.DestroyDriver()
}
