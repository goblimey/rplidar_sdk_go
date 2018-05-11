package USBDriver

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
	"log"

	"github.com/goblimey/rplidar_sdk_go/driver"
)

const moduleName = "USBDriver"

type USBDriver struct {
	driver  C.ProxyDriver
	verbose bool
}

func MakeDriver(verbose bool) driver.Driver {
	if verbose {
		log.Printf("%s MakeDriver()", moduleName)
	}
	driver := USBDriver{}
	v := 0
	if verbose {
		v = -1
	}
	cdriver := C.ProxyDriverInit(C.int(v))
	driver.SetDriver(cdriver)
	return &driver
}

func (d *USBDriver) DestroyDriver() {
	if d.verbose {
		log.Printf("%s DestroyDriver()", moduleName)
	}
	C.ProxyDriverFree(d.driver)
	d.driver = nil
}

func (d *USBDriver) SetDriver(driver C.ProxyDriver) {
	if d.verbose {
		log.Printf("%s SetDriver()", moduleName)
	}
	d.driver = driver
}

func (d *USBDriver) SetProxyDriver(driver driver.Driver) {
	if d.verbose {
		log.Printf("%s SetDriver()", moduleName)
	}
}

func (d *USBDriver) SetVerbose(verbose bool) {
	if verbose {
		log.Printf("%s SetVerbose() %v", moduleName, verbose)
	}
	d.verbose = verbose
}

func (d USBDriver) Connect(port string, baudRate uint, flag uint) uint {
	if d.verbose {
		log.Printf("%s Connect(%s, %d, %d)", moduleName, port, baudRate, flag)
	}
	result := uint(C.ProxyDriverConnect(_cgoCheckPointer(C.ProxyDriver(d.driver)).(C.ProxyDriver), C.CString(port), C.uint(baudRate), C.uint(flag)))
	if result != 0 {
		if d.verbose {
			log.Printf("%s Connect(): failed %d", moduleName, driver.OpResultToString(result))
		}
		if d.verbose {
			log.Printf("%s Connect(): success", moduleName)
		}
	}
	return result
}

func (d USBDriver) Disconnect() {
	if d.verbose {
		log.Printf("%s Disconnect()", moduleName)
	}
	C.ProxyDriverDisconnect(d.driver)
	if d.verbose {
		log.Printf("%s Disconnect(): returning", moduleName)
	}
}

func (d USBDriver) IsConnected() bool {
	if d.verbose {
		log.Printf("%s IsConnected()", moduleName)
	}
	result := int(C.ProxyDriverIsConnected(d.driver)) != 0
	if d.verbose {
		log.Printf("%s IsConnected() returning %v", moduleName, result)
	}
	return result
}

func (d USBDriver) Reset(timeout uint) uint {
	if d.verbose {
		log.Printf("%s Reset()", moduleName)
	}
	result := uint(C.ProxyDriverReset(d.driver, C.uint(timeout)))
	if result == 0 {
		log.Printf("%s Reset(): success", moduleName)
	} else {
		log.Printf("%s Reset(): failed %d", moduleName, result)
	}
	return result
}

func (d USBDriver) GetHealth(timeout uint) (driver.HealthInfo, uint) {
	if d.verbose {
		log.Printf("%s GetHealth(): timeout %d ms", moduleName, timeout)
	}

	result := driver.HealthInfo{}

	var response C.health_response = C.ProxyDriverGetHealth(d.driver, C.uint(timeout))

	opResult := uint(response.op_result)

	if driver.IsFail(opResult) {
		if d.verbose {
			log.Printf("%s GetHealth(): failed - %d", moduleName, opResult)
		}
		return result, opResult
	}

	result.Status = uint8(response.health_info.status)
	result.ErrorCode = uint16(response.health_info.error_code)

	if d.verbose {
		log.Printf("%s GetHealth(): returning", moduleName)
	}
	return result, opResult
}

func (d USBDriver) GetDeviceInfo(timeout uint) (driver.DeviceInfo, uint) {
	if d.verbose {
		log.Printf("%s GetDeviceInfo(): timeout %d", moduleName, timeout)
	}
	result := driver.DeviceInfo{}
	var cinfo *C.rplidar_device_info_unpacked
	cinfo = C.ProxyDriverGetDeviceInfo(d.driver, C.uint(timeout))
	opResult := uint(cinfo.op_result)
	if driver.IsFail(opResult) {
		return result, opResult
	}

	result.Model = uint(cinfo.model)
	result.MajorFimwareVersion = uint(cinfo.majorFirmwareVersion)
	result.SyncQuality = uint(cinfo.minorFirmwareVersion)
	result.HardwareVersion = uint(cinfo.hardwareVersion)
	result.SerialNumber = C.GoString(cinfo.serialNumber)

	return result, opResult
}

func (d USBDriver) GetSampleDuration_uS(timeout uint) (uint, uint16, uint16) {
	if d.verbose {
		log.Printf("%s getSampleDuration_uS(): timeout %d", moduleName, timeout)
	}
	var response C.rplidar_response_sample_rate_unpacked
	response = C.ProxyDriverGetSampleDuration_uS(d.driver, C._u32(timeout))
	opResult := uint(response.op_result)
	if driver.IsFail(opResult) {
		if d.verbose {
			log.Printf("%s getSampleDuration_uS(): failed", moduleName)
		}
		return opResult, 0, 0
	}
	sampleDurationUS := uint16(response.std_sample_duration_us)
	expressSampleDurationUs := uint16(response.express_sample_duration_us)
	if d.verbose {
		log.Printf("%s getSampleDuration_uS(): success sample  %d express %d", moduleName,
			sampleDurationUS, expressSampleDurationUs)
	}
	return opResult, sampleDurationUS, expressSampleDurationUs
}

func (d USBDriver) SetMotorPWM(pwm uint16) uint {
	if d.verbose {
		log.Printf("%s SetMotorPWM(): pwm %d", moduleName, pwm)
	}
	cOpResult := C.ProxyDriverSetMotorPWM(d.driver, C._u16(pwm))
	opResult := uint(cOpResult)
	if driver.IsFail(opResult) {
		if d.verbose {
			log.Printf("%s SetMotorPWM(): failed", moduleName)
		}
	} else {
		if d.verbose {
			log.Printf("%s SetMotorPWM(): success", moduleName)
		}
	}
	return opResult
}

func (d USBDriver) StartMotor() uint {
	if d.verbose {
		log.Printf("%s StartMotor()", moduleName)
	}

	cOpResult := C.ProxyDriverStartMotor(_cgoCheckPointer(C.ProxyDriver(d.driver)).(C.ProxyDriver))
	opResult := uint(cOpResult)
	if driver.IsFail(opResult) {
		if d.verbose {
			log.Printf("%s StartMotor(): failed", moduleName)
		}
	} else {
		if d.verbose {
			log.Printf("%s StartMotor(): success", moduleName)
		}
	}
	return opResult
}

func (d USBDriver) StopMotor() uint {
	if d.verbose {
		log.Printf("%s StopMotor()", moduleName)
	}

	cOpResult := C.ProxyDriverStopMotor(d.driver)
	opResult := uint(cOpResult)
	if driver.IsFail(opResult) {
		if d.verbose {
			log.Printf("%s StopMotor(): failed", moduleName)
		}
	} else {
		if d.verbose {
			log.Printf("%s StopMotor(): success", moduleName)
		}
	}
	return opResult
}

func (d USBDriver) CheckMotorCtrlSupport(timeout uint) (uint, bool) {
	if d.verbose {
		log.Printf("%s CheckMotorCtrlSupport()", moduleName)
	}

	var result C.rplidar_motor_ctrl_support_status
	result = C.ProxyDriverCheckMotorCtrlSupport(d.driver, C._u32(timeout))
	opResult := uint(result.op_result)
	if driver.IsFail(opResult) {
		if d.verbose {
			log.Printf("%s ProxyDriverCheckMotorCtrlSupport(): failed", moduleName)
		}
		return opResult, false
	}
	if int(result.support) == 0 {
		if d.verbose {
			log.Printf("%s ProxyDriverCheckMotorCtrlSupport(): unsupported", moduleName)
		}
		return opResult, false
	}
	if d.verbose {
		log.Printf("%s ProxyDriverCheckMotorCtrlSupport(): supported", moduleName)
	}
	return opResult, true
}

func (d USBDriver) GetFrequency(inExpressMode bool, count uint) (uint, float32, bool) {
	em := 0
	if inExpressMode {
		em = -1
	}
	var cdata C.frequency_data
	cdata = C.ProxyDriverGetFrequency(d.driver, C.int(em), C.uint(count))
	opResult := uint(cdata.op_result)
	if driver.IsFail(opResult) {
		return opResult, 0.0, false
	}
	frequency := float32(cdata.frequency)
	is4kmode := true
	if int(cdata.is4kmode) == 0 {
		is4kmode = false
	}
	return opResult, frequency, is4kmode
}

func (d USBDriver) CheckExpressScanSupported(timeout uint) (uint, bool) {
	if d.verbose {
		log.Printf("%s CheckExpressScanSupported(): timeout %d", moduleName, timeout)
	}
	result := C.ProxyDriverCheckExpressScanSupported(d.driver, C.uint(timeout))
	opResult := uint(result.op_result)
	if driver.IsFail(opResult) {
		if d.verbose {
			log.Printf("%s CheckExpressScanSupported(): failed", moduleName)
		}
		return opResult, false
	}
	supported := false
	if int(result.supported) != 0 {
		supported = true
	}
	if d.verbose {
		log.Printf("%s CheckExpressScanSupported(): returning %v", moduleName, supported)
	}
	return opResult, supported
}

func (d USBDriver) StartScan(force bool, autoExpress bool) uint {
	if d.verbose {
		log.Printf("%s StartScan()", moduleName)
	}
	intForce := 0
	if force {
		intForce = -1
	}
	intAutoExpress := 0
	if autoExpress {
		intAutoExpress = -1
	}
	result := uint(C.ProxyDriverStartScan(d.driver, C.int(intForce), C.int(intAutoExpress)))

	if d.verbose {
		log.Printf("%s StartScan():returning %d", moduleName, result)
	}
	return result
}

func (d USBDriver) StartScanNormal(force bool, timeout uint) uint {
	if d.verbose {
		log.Printf("%s StartScanNormal()", moduleName)
	}
	intForce := 0
	if force {
		intForce = -1
	}
	result := uint(C.ProxyDriverStartScanNormal(d.driver, C.int(intForce), C._u32(timeout)))

	if d.verbose {
		log.Printf("%s StartScanNormal():returning %d", moduleName, result)
	}
	return result
}

func (d USBDriver) StartScanExpress(fixedAngle bool, timeout uint) uint {
	if d.verbose {
		log.Printf("%s StartScanExpress()", moduleName)
	}
	intFixedAngle := 0
	if fixedAngle {
		intFixedAngle = -1
	}

	result := uint(C.ProxyDriverStartScanExpress(d.driver, C.int(intFixedAngle), C._u32(timeout)))

	if d.verbose {
		log.Printf("%s StartScanExpress():returning %d", moduleName, result)
	}
	return result
}

func (d USBDriver) Stop(timeout uint) uint {
	if d.verbose {
		log.Printf("%s Stop()", moduleName)
	}
	C.ProxyDriverStop(d.driver, C.uint(timeout))

	cOpResult := C.ProxyDriverStop(d.driver, C.uint(timeout))
	opResult := uint(cOpResult)
	if driver.IsFail(opResult) {
		if d.verbose {
			log.Printf("%s Stop(): failed", moduleName)
		}
	} else {
		if d.verbose {
			log.Printf("%s Stop(): success", moduleName)
		}
	}
	return opResult
}

func (d USBDriver) GrabScanData(scans uint, timeout uint) (uint, []driver.RplidarResponseMeasurementNode) {
	if d.verbose {
		log.Printf("%s GrabScanData()", moduleName)
	}
	var cscans = C.uint(scans)
	if d.verbose {
		log.Printf("%s GrabScanData(): got %d scans\n", moduleName, uint(cscans))
	}

	var cresults *C.rplidar_scan_results_unpacked = C.ProxyDriverGrabScanData(d.driver, cscans, C.uint(timeout))
	opResult := uint(cresults.op_result)
	if driver.IsFail(opResult) {
		if d.verbose {
			log.Printf("%s GrabScanData(): failed - %s", moduleName, driver.OpResultToString(opResult))
		}
		return opResult, nil
	}

	// Success
	returnedScans := int(cresults.scans)
	if d.verbose {
		log.Printf("%s GrabScanData(): got %d scans", moduleName, returnedScans)
	}
	nodes := make([]driver.RplidarResponseMeasurementNode, returnedScans)
	for i := 0; i < returnedScans; i++ {
		cnode := C.ProxyDriverGetRplidarResponseMeasurementNode(d.driver, cresults, C.uint(i))
		nodes[i].AngleQ6Checkbit = uint16(cnode.angle_q6_checkbit)
		nodes[i].SyncQuality = uint8(cnode.sync_quality)
		nodes[i].DistanceQ2 = uint16(cnode.distance_q2)
	}
	// Free the space malloced by the C++ proxy
	C.ProxyDriverFreeRplidarScanResultsUnpacked(d.driver, cresults)

	if d.verbose {
		log.Printf("%s GrabScanData(): returning %d nodes\n", moduleName, len(nodes))
	}
	return opResult, nodes
}

func (d USBDriver) GrabAndSortScanData(scans uint, timeout uint) (uint, []driver.RplidarResponseMeasurementNode) {
	if d.verbose {
		log.Printf("%s GrabAndSortScanData()", moduleName)
	}
	var cscans = C.uint(scans)
	if d.verbose {
		log.Printf("%s GrabAndSortScanData(): got %d scans\n", moduleName, uint(cscans))
	}

	var cresults *C.rplidar_scan_results_unpacked = C.ProxyDriverGrabAndSortScanData(d.driver, cscans, C.uint(timeout))
	opResult := uint(cresults.op_result)
	if driver.IsFail(opResult) {
		return opResult, nil
	}
	// success
	returnedScans := int(cresults.scans)
	if d.verbose {
		log.Printf("%s GrabAndSortScanData(): got %d scans", moduleName, returnedScans)
	}
	results := make([]driver.RplidarResponseMeasurementNode, returnedScans)
	for i := 0; i < returnedScans; i++ {
		cnode := C.ProxyDriverGetRplidarResponseMeasurementNode(d.driver, cresults, C.uint(i))
		results[i].AngleQ6Checkbit = uint16(cnode.angle_q6_checkbit)
		results[i].SyncQuality = uint8(cnode.sync_quality)
		results[i].DistanceQ2 = uint16(cnode.distance_q2)
	}
	// Free the space malloced by the C++ proxy
	C.ProxyDriverFreeRplidarScanResultsUnpacked(d.driver, cresults)

	if d.verbose {
		log.Printf("%s GrabAndSortScanData(): returning %d nodes\n", moduleName, len(results))
	}
	return opResult, results
}
