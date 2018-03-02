package driver

// There must be no blank lines within these clever comments:

// #cgo CFLAGS: -I${SRCDIR}/../include -DDEBUG -g
// #cgo CXXFLAGS: -I${SRCDIR}/../include -DDEBUG -g
// #cgo CPPFLAGS: -I${SRCDIR}/../include -DDEBUG -g
// #cgo LDFLAGS: -L${SRCDIR}/../lib -lrplidar_sdk -lrt -lstdc++ -lpthread
/*
#include <stdio.h>
#include <stdlib.h>
#include "rptypes.h"
#include "driver.h"
*/
import "C"

type ConcreteDevInfo struct {
	MajorFimwareVersion  int
	MinorFirmwareVersion int
	HardwareVersion      int
}

type ConcreteRPlidarDriver struct {
	driver   C.RPDriver
	port     string
	_devInfo ConcreteDevInfo
}

func MakeDriver(port string) ConcreteRPlidarDriver {
	driver := ConcreteRPlidarDriver{}
	driver.SetPort(port)
	// create the C++ driver instance
	cppPort := C.CString(port)
	cdriver := C.RPDriverInit(cppPort, C._u32(0))
	driver.SetDriver(cdriver)
	return driver
}

func (d *ConcreteRPlidarDriver) DestroyDriver() {
	C.RPDriverFree(d.driver)
	d.driver = nil
}

func (d *ConcreteRPlidarDriver) SetDriver(driver C.RPDriver) {
	d.driver = driver
}

func (d *ConcreteRPlidarDriver) SetPort(port string) {
	d.port = port
}

/*

func (d ConcreteRPlidarDriver) DevInfo() (nfo ConcreteDevInfo, err Error) {
	var opresult C.uint
	o := d.cdriver_getDeviceInfo(devinfo)
	opresult = d.cdriver_getDeviceInfo(devinfo)

	if uint(opresult) < 0 {
		return nil, error.New("failed to get device info")
	}

	fmt.Printf("RPLIDAR S/N ")
	for i := 0; i < 16; i++ {
		fmt.Printf("%02X", uint(devinfo.serialnum[C.uint(i)]))
		fmt.Printf("\n")
	}

	d._devInfo = ConcreteDevInfo(
		int(d, cdriver.getMajorVersionNumber()),
		int(d, cdriver.getMajorVersionNumber()),
		int(d, cdriver.getHardwareId()))

	fmt.Fprintf("Firmware Ver: %d.%02d\nHardware Rev: %d\n",
		d.DevInfo.MajorFimwareVersion,
		d.DevInfo.MinorFirmwareVersion,
		d.DevInfo.HardwareVersion)

	return d._devInfo
}
*/
func (d ConcreteRPlidarDriver) CheckRPLIDARHealth() bool {
	var result C.int = C.RPDriverCheckRPLidarHealth(d.driver)
	return (int(result) == 1)
}

func (d ConcreteRPlidarDriver) StartMotor() {
	C.RPDriverStartMotor(d.driver)
}

func (d ConcreteRPlidarDriver) Stop() {
	C.RPDriverStop(d.driver)
}

func (d ConcreteRPlidarDriver) StopMotor() {
	C.RPDriverStopMotor(d.driver)
}
