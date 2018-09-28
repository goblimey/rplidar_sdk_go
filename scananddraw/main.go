package main

import (
	// "C"
	"flag"
	"fmt"
	"image/color"
	"log"
	"os"
	"regexp"
	"strings"
	"time"

	"github.com/fogleman/gg"
	"github.com/goblimey/rplidar_sdk_go/driver"
	USBDriver "github.com/goblimey/rplidar_sdk_go/usb_driver"
)

// scananddraw runs a scan and draws the result as a .png.

//Point contains the x and y coordinates of a point.
type Point struct {
	X float64
	Y float64
}

// These values are set from the command line arguments.
var verbose bool      // verbose mode
var port string       // USB port
var speed uint        // speed of the port 115200
var delaySecs uint    // Delay for this many seconds after issuing the startMotor command.
var timeout uint      // timeout when grabbing a scan
var outputFile string // The output file (a .png)

func init() {
	flag.BoolVar(&verbose, "verbose", false, "verbose mode")
	flag.BoolVar(&verbose, "v", false, "verbose mode")
	flag.StringVar(&port, "port", "", "the USB port on which the RPLidar device is connected")
	flag.UintVar(&speed, "speed", 115200, "the speed of the USB port")
	flag.UintVar(&delaySecs, "delay", 10, "Delay for this many seconds after issuing the startMotor command")
	flag.UintVar(&delaySecs, "d", 10, "Delay for this many seconds after issuing the startMotor command")
	flag.UintVar(&timeout, "t", 2000, "the timeout in milliseconds when grabbling a scan")
	flag.UintVar(&timeout, "timeout", 2000, "the timeout in milliseconds when grabbling a scan")
	flag.StringVar(&outputFile, "f", "", "the output file - a .png")
	flag.StringVar(&outputFile, "file", "", "the output file - a .png")
}

const imgBound int = 1000

func main() {

	flag.Parse()

	// Ensure that the output file is writable

	f, err := os.Create(outputFile)
	if err != nil {
		fmt.Printf("cannot open %s for writing - %s\n", outputFile, err.Error())
		return
	}
	f.Close()

	measurements := scan(720, timeout, delaySecs)

	if measurements == nil {
		return
	}

	points := createMap(measurements)

	drawMap(outputFile, points)
}

func scan(requestedScans uint, timeout uint, delay uint) []driver.RplidarResponseMeasurementNode {
	usbDriver := USBDriver.MakeDriver(verbose)

	defer tidyup(usbDriver)

	opResult := usbDriver.Connect(port, speed, 0)

	if driver.IsFail(opResult) {
		log.Printf("failed to connect - %s", driver.OpResultToString(opResult))
		return nil
	}

	healthInfo, opResult := usbDriver.GetHealth(timeout)
	if driver.IsFail(opResult) {
		log.Printf("failed to get health info - %s", driver.OpResultToString(opResult))
		return nil
	}
	// opResult OK just means that the connection worked and we got some status back.
	if healthInfo.ErrorCode == 0 {
		log.Printf("health status OK -  %d\n", healthInfo.Status)
	} else {
		log.Printf("health status bad - error code %d status %d\n", healthInfo.ErrorCode, healthInfo.Status)
		return nil
	}

	devInfo, opResult := usbDriver.GetDeviceInfo(timeout)
	if driver.IsFail(opResult) {
		log.Printf("failed to get device info - %s", driver.OpResultToString(opResult))
		return nil
	}

	log.Printf("model %d serial number %s\n", devInfo.Model, devInfo.SerialNumber)
	log.Printf("Hardware version %d firmware %d/%d\n", devInfo.HardwareVersion, devInfo.MajorFimwareVersion, devInfo.MinorFirmwareVersion)

	_, motorControSupport := usbDriver.CheckMotorCtrlSupport(100)

	if verbose {
		if motorControSupport {
			log.Println("motor control supported")
		} else {
			log.Println("motor control not supported")
		}
	}

	opResult, expressMode := usbDriver.CheckExpressScanSupported(100)

	if verbose {
		if driver.IsOK(opResult) {
			if expressMode {
				log.Println("express mode supported")
			} else {
				log.Println("express mode not supported")
			}
		} else {
			log.Printf("failed to get express support status - %s", driver.OpResultToString(opResult))
		}
	}

	usbDriver.StartMotor()

	// Ideally this should be 120 seconds to warm up the device to its ideal teperature.
	time.Sleep(time.Second * time.Duration(delay))

	// Run the scan in express mode if supported.
	usbDriver.StartScan(false, true)

	time.Sleep(time.Second * 1)

	opResult, measurements := usbDriver.GrabAndSortScanData(requestedScans, timeout)

	if driver.IsFail(opResult) {
		log.Printf("failed to grab data - %s", driver.OpResultToString(opResult))

		return nil
	}

	if verbose {
		log.Printf("main: %d scans", len(measurements))
	}

	if verbose {
		opResult, frequency, is4kmode := usbDriver.GetFrequency(expressMode, uint(len(measurements)))
		if driver.IsOK(opResult) {
			log.Printf("frequency %f is4kmode %v", frequency, is4kmode)
		} else {
			log.Printf("failed to get frequency - %s", driver.OpResultToString(opResult))
		}
	}

	if verbose {
		opResult, duration, expressDuration := usbDriver.GetSampleDuration_uS(timeout)
		if driver.IsOK(opResult) {
			log.Printf("opResult %d Sample Duration %d us %d", opResult, duration, expressDuration)
		} else {
			log.Printf("failed to get sample duration - %s", driver.OpResultToString(opResult))
		}
	}

	return measurements
}

// createMap() takes a list of measurements and returns a list of paths, each a list of
// connected points.
func createMap(measurements []driver.RplidarResponseMeasurementNode) [][]Point {
	m := "createMap"

	// path[] is a list of paths, each a list of points.  A path of length 1
	// represents an isolated point, otherwise the path is a list of joined-up lines.
	paths := make([][]Point, 1, len(measurements))
	paths[0] = make([]Point, 0, len(measurements))
	// i is the index of the list of paths, j is the index of a path.  i is
	// initially -1 because we always increment it before starting a path.
	i := -1
	j := 0
	// lastMeasurement is the measurement from the last trip round the loop.
	// Initially it's a dummy, set up to be an invalid measurement.
	lastMeasurement := driver.RplidarResponseMeasurementNode{0, 0, 0}

	// Loop through the measurements and create the paths.  A list of
	// contiguous valid measurements forms a path - invalid measurements
	// separate two paths.

	for _, measurement := range measurements {

		// The measurement list contains valid and invalid values.  A list of valid
		// values form a path.  Invalid values mark a break between two paths.  If
		// this is a valid measurement and the previous one was valid, add the
		// endpoint to the current path.  If this is valid and the previous one was
		// invalid, start a new path.

		if measurement.Invalid() {
			if verbose {
				fmt.Printf("%s: ignoring {%f %f}\n", m,
					measurement.AngleAsFloat64(), measurement.DistanceAsFloat64())
			}
			lastMeasurement = measurement
			continue
		}

		// This measurement is valid.

		if lastMeasurement.Invalid() {
			// This measurement is valid, the last was invalid, so it's the first in a
			// new path.  Lengthen the list of paths by one and add a new path, initially
			// of length zero.
			i++
			j = 0
			paths = paths[:i+1]
			paths[i] = make([]Point, 0, len(measurements))
		}

		if verbose {
			log.Printf("%s: path[%d][%d] {%f %f}\n", m, i, j,
				measurement.AngleAsFloat64(), measurement.DistanceAsFloat64())
		}

		// Lengthen the path by one and add the point.
		paths[i] = paths[i][:j+1]
		x, y := measurement.EndPoint()
		paths[i][j] = Point{x, y}

		// Prepare for the next iteration
		lastMeasurement = measurement
		j++
	}

	return paths
}

// drawMap creates or opens the .png file named by outputFile and draws the paths
// of points.
func drawMap(png string, paths [][]Point) {
	m := "drawMap"

	// Find the biggest x value or the biggest y value if it's bigger.  The sign
	// is ignored in these comparisons,This is
	// used to determine mapSize, the length of the sides of a square.  The map is
	// drwawn within that.  It's a little bigger than 2*max, to give a border.

	// max is the biggest x or y coordinate, ignoring sign (so -3 is bigger than 2).  It's
	// used to calcuate the size of the frame in which the map is drawn.  All x and y values
	// are greater than zero, so if the max value is 0, it's not been set yet.
	max := 0.0
	for i := range paths {
		for j := range paths[i] {

			x := paths[i][j].X
			y := paths[i][j].Y

			if x < 0 {
				x = 1 - x // invert negative x.
			}
			if y < 0 {
				y = 1 - y // Invert negative y.
			}

			if max <= 0 {
				max = x // Set max for the first time.
			} else if x > max {
				// x is the biggest value so far.
				max = x
			}

			if y > max {
				// y is the biggest value so far.
				max = y
			}
		}
	}

	// This loop draws the paths.  Each path is of length 1 (an isolated point)
	// or length greater than 1 (a connected path of points).
	//
	// Draw the paths and the isolated points on the png canvas.  Max is the
	// biggest x and y value, so draw the map in a canvas which is a little bigger
	// than max.
	//
	// In the map, the x and y coordinates both run from -mapSize/2 to mapSize/2,
	// increasing towards the North.  The canvas runs from 0 to imgBound, increasing as
	// you head South.  So we have to invert the y coordinates and shift and scale both
	// by a suitable scaleFactor.  The gg graphics package handles most of that for us.

	intMax := int(max)
	intMax = ((intMax / 1000) + 1) * 1000
	max = float64(intMax)
	mapSize := max * 2            // The map runs from +x to -x.
	axisLineLength := max * 0.025 // The length of the short lines marking the axes.

	if verbose {
		log.Printf("%s: max %f mapSize %f\n", m, max, mapSize)
	}

	dc := gg.NewContext(imgBound, imgBound)

	// Set the background to white.
	dc.MoveTo(0, 0)
	dc.LineTo(0, float64(imgBound))
	dc.LineTo(float64(imgBound), float64(imgBound))
	dc.LineTo(float64(imgBound), 0)
	dc.ClosePath()
	dc.SetColor(color.White)
	dc.Fill()

	// Invert to correct the direction of the canvas.
	dc.InvertY()

	// Set the origin at the centre of the canvas,
	dc.Translate(float64(imgBound/2), float64(imgBound/2))

	// Scale the data to the image
	scaleFactor := float64(imgBound) / float64(mapSize)
	if verbose {
		log.Printf("%s: scaleFactor %f\n", m, scaleFactor)
	}
	dc.Scale(scaleFactor, scaleFactor)

	// Draw short black lines over each of the axes, one close to the origin, one up to the edge.
	dc.SetColor(color.Black)

	// y
	dc.DrawLine(0, 150, 0, 150+axisLineLength*2)
	dc.DrawLine(0, -150, 0, -1*(150+axisLineLength*2))
	dc.DrawLine(150, 0, 150+axisLineLength*2, 0)
	dc.DrawLine(-150, 0, -1*(150+axisLineLength*2), 0)

	dc.Stroke()

	dc.DrawLine(0, max-axisLineLength, 0, max)
	dc.DrawLine(max-axisLineLength, 0, max, 0)
	dc.DrawLine(-1*(max-axisLineLength), 0, -1*max, 0)
	dc.DrawLine(0, -1*(max-axisLineLength), 0, -1*max)
	dc.Stroke()

	// Draw red circles to show the scale
	dc.SetRGB255(255, 0, 0)
	for i := 2000; float64(i) < max; i += 2000 {
		dc.DrawCircle(0, 0, float64(i))
		dc.Stroke()
	}

	// Rotate to point the RPLidar device to the North
	dc.Rotate(gg.Radians(90))

	// Draw the device at the centre with its cable pointing South.
	dc.SetRGB(0, 0, 0)
	dc.DrawCircle(0, 0, 75/2)
	dc.Stroke()
	dc.DrawCircle(0, 0, 63/2)
	dc.Fill()
	dc.SetLineWidth(3)
	dc.DrawLine(-75/2, 0, -75, 0)
	dc.Stroke()
	dc.SetLineWidth(1)

	// Draw the paths and isolated points in green.
	dc.SetRGB255(0, 0, 255)
	for i := range paths {
		if len(paths[i]) >= 1 {
			// Move to the start of the path.
			p := paths[i][0]
			dc.MoveTo(p.X, p.Y)

			if len(paths[i]) == 1 {
				// This is an isolated point.
				dc.LineTo(p.X, p.Y)
				dc.Stroke()
			} else {
				// Draw the lines in the path.
				for j := 1; j < len(paths[i]); j++ {
					p := paths[i][j]
					dc.LineTo(p.X, p.Y)
				}
			}
		}
		dc.Stroke()
	}

	// Write the RGBA to the PNG image file.

	err := dc.SavePNG(png)
	if err != nil {
		log.Printf(err.Error())
		return
	}
}

// stripSpaces removes all white space from the start and end of a string and replaces all
// white space within the string with a single space.
func stripSpaces(s string) (string, error) {
	s = strings.TrimSpace(s)
	re, err := regexp.Compile("  +")
	if err != nil {
		return s, err
	}

	return re.ReplaceAllLiteralString(s, " "), nil
}

func tidyup(usbDriver driver.Driver) {
	usbDriver.StopMotor()
	usbDriver.DestroyDriver()
}
