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
#include "rplidar_cmd.h"
#include "rplidar_go_if.h"
#include "driver.h"
*/
import "C"

import (
	"errors"
	"fmt"
	"log"
)


const moduleName = "ConcreteProxyDriver"


type ConcreteDevInfo struct {
	Model               uint
	MajorFimwareVersion uint
	SyncQuality         uint
	HardwareVersion     uint
	SerialNumber        string
}


type RplidarResponseMeasurementNode struct {
	SyncQuality     uint8
	AngleQ6Checkbit uint16
	DistanceQ2      uint16
}


type RplidarResponseMeasurementNodeUnpacked struct {
	Sync     bool
	Quality  uint
	Check    bool
	Angle    uint
	Distance uint
}


type RplidarScanResults struct {
	Scans uint
}


type ConcreteProxyDriver struct {
	driver  C.ProxyDriver
	verbose bool
}


func MakeDriver(verbose bool, port string, baud_rate uint) ConcreteProxyDriver {
	if verbose {
		log.Printf("%s MakeDriver()", moduleName)
	}
	driver := ConcreteProxyDriver{}

	cppPort := C.CString(port)
	cppBaudRate := C.uint(baud_rate)
	cdriver := C.ProxyDriverInit(cppPort, cppBaudRate)
	driver.SetDriver(cdriver)
	return driver
}


func (d *ConcreteProxyDriver) DestroyDriver() {
	if d.verbose {
		log.Printf("%s DestroyDriver()", moduleName)
	}
	C.ProxyDriverFree(_cgoCheckPointer(C.ProxyDriver(d.driver)).(C.ProxyDriver))
	d.driver = nil
}


func (d *ConcreteProxyDriver) SetDriver(driver C.ProxyDriver) {
	if d.verbose {
		log.Printf("%s SetDriver()", moduleName)
	}
	d.driver = driver
}


func (d *ConcreteProxyDriver) SetVerbose(verbose bool) {
	if verbose {
		log.Printf("%s SetVerbose() %v", moduleName, verbose)
	}
	d.verbose = verbose
}


func (d ConcreteProxyDriver) Connect(port string, baudRate uint, flag uint) error {
	if d.verbose {
		log.Printf("%s Connect(%s, %d, %d)", moduleName, port, baudRate, flag)
	}
	if baudRate == 0 {
		baudRate = 115200
	}
	result := uint(C.ProxyDriverConnect(_cgoCheckPointer(C.ProxyDriver(d.driver)).(C.ProxyDriver), _Cfunc_CString(port), C.uint(baudRate), C.uint(flag)))
	if result != 0 {
		if d.verbose {
			log.Printf("%s Connect(): failed %d", moduleName, result)
			message := fmt.Sprintf("failed to connect - error code %d", result)
			return errors.New(message)
		}
		if d.verbose {
			log.Printf("%s Connect(): success", moduleName)
		}
	}
	return nil
}


func (d ConcreteProxyDriver) Disconnect() {
	if d.verbose {
		log.Printf("%s Disconnect()", moduleName)
	}
	C.ProxyDriverDisconnect(_cgoCheckPointer(C.ProxyDriver(d.driver)).(C.ProxyDriver))
}


func (d ConcreteProxyDriver) IsConnected() bool {
	if d.verbose {
		log.Printf("%s IsConnected()", moduleName)
	}
	result := int(C.ProxyDriverIsConnected(_cgoCheckPointer(C.ProxyDriver(d.driver)).(C.ProxyDriver))) != 0
	if d.verbose {
		log.Printf("%s IsConnected() returning %v", moduleName, result)
	}
	return result
}


func (d ConcreteProxyDriver) Reset(timeout uint) uint {
	if d.verbose {
		log.Printf("%s Reset()", moduleName)
	}
	result := uint(C.ProxyDriverReset(_cgoCheckPointer(C.ProxyDriver(d.driver)).(C.ProxyDriver), C.uint(timeout)))
	if result == 0 {
		log.Printf("%s Reset(): success", moduleName)
	} else {
		log.Printf("%s Reset(): failed %d", moduleName, result)
	}
	return result
}


func (d ConcreteProxyDriver) GetDeviceInfo(timeout uint) (ConcreteDevInfo, error) {
	if d.verbose {
		log.Printf("%s GetDeviceInfo(): timeout %d", moduleName, timeout)
	}
	result := ConcreteDevInfo{}
	var cinfo *C.rplidar_device_info_unpacked
	cinfo = C.ProxyDriverGetDeviceInfo(_cgoCheckPointer(C.ProxyDriver(d.driver)).(C.ProxyDriver), C.uint(timeout))
	if cinfo == nil {
		return result, errors.New("failed to get device info")
	}

	result.Model = uint(cinfo.model)
	result.MajorFimwareVersion = uint(cinfo.majorFirmwareVersion)
	result.SyncQuality = uint(cinfo.minorFirmwareVersion)
	result.HardwareVersion = uint(cinfo.hardwareVersion)
	result.SerialNumber = C.GoString(cinfo.serialNumber)

	C.ProxyDriverFreeRplidarDeviceInfoUnpacked(_cgoCheckPointer(C.ProxyDriver(d.driver)).(C.ProxyDriver), _cgoCheckPointer((*C.rplidar_device_info_unpacked)(cinfo)).(*C.rplidar_device_info_unpacked))
	return result, nil
}


func (d ConcreteProxyDriver) GetHealth(timeout uint) bool {
	if d.verbose {
		log.Printf("%s GetHealth(): timeout %d ms", moduleName, timeout)
	}
	healthy := int(C.ProxyDriverGetHealth(_cgoCheckPointer(C.ProxyDriver(d.driver)).(C.ProxyDriver), C.uint(timeout)))
	if healthy == 0 {
		if d.verbose {
			log.Printf("%s GetHealth(): health check failed", moduleName)
		}
		return false
	}
	if d.verbose {
		log.Printf("%s GetHealth(): healthy", moduleName)
	}
	return true
}


func (d ConcreteProxyDriver) StartMotor() {
	if d.verbose {
		log.Printf("%s StartMotor()", moduleName)
	}
	C.ProxyDriverStartMotor(_cgoCheckPointer(C.ProxyDriver(d.driver)).(C.ProxyDriver))
}


func (d ConcreteProxyDriver) StopMotor() {
	if d.verbose {
		log.Printf("%s StopMotor()", moduleName)
	}
	C.ProxyDriverStopMotor(_cgoCheckPointer(C.ProxyDriver(d.driver)).(C.ProxyDriver))
}


func (d ConcreteProxyDriver) Stop(timeout uint) {
	if d.verbose {
		log.Printf("%s Stop()", moduleName)
	}
	C.ProxyDriverStop(_cgoCheckPointer(C.ProxyDriver(d.driver)).(C.ProxyDriver), C.uint(timeout))
}


func (d ConcreteProxyDriver) StartScan() uint {
	if d.verbose {
		log.Printf("%s StartScan()", moduleName)
	}
	result := uint(C.ProxyDriverStartScan(_cgoCheckPointer(C.ProxyDriver(d.driver)).(C.ProxyDriver)))
	if d.verbose {
		log.Printf("%s StartScan():returning", moduleName)
	}
	return result
}


func (d ConcreteProxyDriver) GrabScanData(scans uint, timeout uint) *C.rplidar_scan_results_packed {
	if d.verbose {
		log.Printf("%s GrabScanData()", moduleName)
	}
	var cscans = C.uint(scans)
	if d.verbose {
		log.Printf("%s GrabScanData(): returning $d scans\n", moduleName, uint(cscans))
	}

	result := C.ProxyDriverGrabScanData(_cgoCheckPointer(C.ProxyDriver(d.driver)).(C.ProxyDriver), cscans, C.uint(timeout))
	if d.verbose {
		log.Printf("%s FreeRplidarScanResultsPacked(): returning\n", moduleName)
	}
	return result
}


func (d ConcreteProxyDriver) FreeRplidarScanResultsPacked(data *C.rplidar_scan_results_packed) {
	if d.verbose {
		log.Printf("%s FreeRplidarScanResultsPacked()", moduleName)
	}
	C.ProxyDriverFreeRplidarScanResultsPacked(_cgoCheckPointer(C.ProxyDriver(d.driver)).(C.ProxyDriver), _cgoCheckPointer((*C.rplidar_scan_results_packed)(data)).(*C.rplidar_scan_results_packed))
	if d.verbose {
		log.Printf("%s FreeRplidarScanResultsPacked(): returning", moduleName)
	}
}


func (d ConcreteProxyDriver) GetRplidarResponseMeasurementNode(data *C.rplidar_scan_results_packed, i uint) (RplidarResponseMeasurementNode, error) {
	if d.verbose {
		log.Printf("%s GetRplidarResponseMeasurementNode()", moduleName)
	}
	var cnode *C.rplidar_response_measurement_node_unpacked
	cnode, err := C.ProxyDriverGet_rplidar_response_measurement_node_t(d.driver, data, C.uint(i))
	node := RplidarResponseMeasurementNode{}
	if err != nil {
		return node, err
	}
	node.SyncQuality = uint8(cnode.sync_quality)
	node.AngleQ6Checkbit = uint16(cnode.angle_q6_checkbit)
	node.DistanceQ2 = uint16(cnode.distance_q2)
	if d.verbose {
		log.Printf("%s GetRplidarResponseMeasurementNode(): returning", moduleName)
	}
	return node, err
}


func (d ConcreteProxyDriver) AscendScanData(data *C.rplidar_scan_results_packed) ([]RplidarResponseMeasurementNode, error) {
	if d.verbose {
		log.Printf("%s AscendScanData()", moduleName)
	}
	var sortedScanData *C.rplidar_scan_results_packed = C.ProxyDriverAscendScanData(_cgoCheckPointer(C.ProxyDriver(d.driver)).(C.ProxyDriver), _cgoCheckPointer((*C.rplidar_scan_results_packed)(data)).(*C.rplidar_scan_results_packed))
	scans := uint(sortedScanData.scans)
	result := make([]RplidarResponseMeasurementNode, scans)
	var i uint
	for i = 0; i < scans; i++ {
		var cnode *C.rplidar_response_measurement_node_unpacked = C.ProxyDriverGet_rplidar_response_measurement_node_t(_cgoCheckPointer(C.ProxyDriver(d.driver)).(C.ProxyDriver), _cgoCheckPointer((*C.rplidar_scan_results_packed)(data)).(*C.rplidar_scan_results_packed), C.uint(i))
		if cnode == nil {
			message := fmt.Sprintf("%s AscendScanData(): array index out of bounds %d", moduleName, i)
			return nil, errors.New(message)
		}
		result[i].SyncQuality = uint8(cnode.sync_quality)
		result[i].AngleQ6Checkbit = uint16(cnode.angle_q6_checkbit)
		result[i].DistanceQ2 = uint16(cnode.distance_q2)
	}
	return result, nil
}


func (d ConcreteProxyDriver) UnPack(data *C.rplidar_scan_results_packed) *RplidarScanResults {
	if d.verbose {
		log.Printf("%s UnPack()", moduleName)
	}
	cresults := C.ProxyDriverUnPack(_cgoCheckPointer(C.ProxyDriver(d.driver)).(C.ProxyDriver), _cgoCheckPointer((*C.rplidar_scan_results_packed)(data)).(*C.rplidar_scan_results_packed))
	if d.verbose {
		log.Printf("%s UnPack(): unpacking to Go structure", moduleName)
	}
	results := RplidarScanResults{}
	results.Scans = uint(cresults.scans)

	return &results
}
