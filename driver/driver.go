package driver

// Driver defines the Go driver interface to the RPLIDAR device.  The implemenation is a Go wrapper around a C++ driver
// supplied by the manufacturer Slamtec. RPLidar is a family of device that use a laser beam mounted on a spinning platform to scan nearby objects in a circle
// and return a list of angle and distance values. The original purpose was to produce a floor plan of the room in which
// a robot vacuum cleaner was running.
//
// For instructuons on building and using the driver, see https://www.github.com/goblimey/rplidar_sdk_go.
//
// This is the copyright notice for the underlying C++ driver:
//
//  RPLIDAR SDK
//
//  Copyright (c) 2009 - 2014 RoboPeak Team
//  http://www.robopeak.com
//  Copyright (c) 2014 - 2016 Shanghai Slamtec Co., Ltd.
//  http://www.slamtec.com
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
// THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
// OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
// WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
// OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
// EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

const (
	// operation status values
	ResultOK                    = 0
	ResultFailBit               = 0x80000000
	ResultAlreadyDone           = 0x20
	ResultInvalidData           = (0x8000 | ResultFailBit)
	ResultOperationfail         = (0x8001 | ResultFailBit)
	ResultOperationTimeout      = (0x8002 | ResultFailBit)
	ResultOperationStop         = (0x8003 | ResultFailBit)
	ResultOperationNotSupported = (0x8004 | ResultFailBit)
	ResultFormatNotSupported    = (0x8005 | ResultFailBit)
	ResultInsufficientMemory    = (0x8006 | ResultFailBit)

	// bit masks
	RplidarAnswerTypeDevInfo                        = 0x4
	RplidarAnswerTypeDevHealth                      = 0x6
	RplidarAnswerTypeMeasurement                    = 0x81
	RplidarAnswerTypeMeasurementCapsuled            = 0x82
	RplidarAnswerTypeSampleRate                     = 0x15
	RplidarAnsTypeAccBoardFlag                      = 0xFF
	RplidarResponseAccBoardFlagMotorCtrlSupportMask = (0x1)
	RRplidarStatusOK                                = 0x0
	RplidarStatusWarning                            = 0x1
	RplidarStatusError                              = 0x2

	// RplidarResponseMeasurementSyncBit masks off the Sync Bit of the SyncQuality field of an
	// RplidarResponseMeasurementNode.
	RplidarResponseMeasurementSyncBit = (0x1 << 0)
	// RplidarResponseMeasurementQualityShift is used to extract the quality component of the SyncQuality field of an
	// RplidarResponseMeasurementNode.
	RplidarResponseMeasurementQualityShift = 2
	// RplidarResponseMeasurementCheckBit is used to extract the lower check bit from the AngleQ6CheckBit field of an
	// RplidarResponseMeasurementNode.
	RplidarResponseMeasurementCheckBit = (0x1 << 0)
	// RplidarResponseMeasurementAngleShift is used to extract the angle component from the AngleQ6CheckBit field of an
	// RplidarResponseMeasurementNode.
	RplidarResponseMeasurementAngleShift = 1
	// RplidarResponseMeasurementExpAngleMask masks off the check bit and inverse check bit of the AngleQ6CheckBit field
	// of an RplidarResponseMeasurementNode.
	RplidarResponseMeasurementExpAngleMask = (0x3)
	// RplidarResponseMeasurementExpDistanceMask is used to extract the whole part of the distance as a scaled integer with
	RplidarResponseMeasurementExpDistanceMask = (0xFC)
	RplidarResponseMeasurementExpSync1        = 0xA
	RplidarResponseMeasurementExpSync2        = 0x5
	RplidarResponseMeasurementExpSyncBit      = (0x1 << 15)
	// RplidarResponseMeasurementAngleConversion is used to convert the angle component of the AngleQ6CheckbBit field of an
	// RplidarResponseMeasurementNodehe to a float.  The angle is a fixed point value with 15 bits, 9 bits whole part, 6 bits
	// fractional part.
	RplidarResponseMeasurementAngleConversion = 64.0

	// Timeout values

	DefaultTimeout = 2000
)

// HealthInfo contains the system health information returned by GetHealth.  It contains
// the status of the RPLidar device and an error code.
type HealthInfo struct {
	Status    uint8
	ErrorCode uint16
}

// DeviceInfo contains the device information returned by GetDeviceInfo.   It contains the model number of the RPLidar device,
// the major firmware version, the inor firmware version, the haradware version and the serial number, which is unique to
// each device.
type DeviceInfo struct {
	Model                uint
	MajorFimwareVersion  uint
	MinorFirmwareVersion uint
	HardwareVersion      uint
	SerialNumber         string
}

// IsOK returns true if the result returned by an operation (the opResult) indicates that it wassuccessful.
func IsOK(x uint) bool {
	return ((x) & ResultFailBit) == 0
}

// IsFail returns true if the result returned by an operation (the opResult) indicates that it failed.  Note that
// a scan operation can return a timeout failure while also returning partial scan data.
func IsFail(x uint) bool {
	return ((x)&ResultFailBit != 0)
}

// RplidarResponseMeasurementNode holds a measurement of angle and distance along with sync and quality bits.  The
// angle and distance components are fixed point numbers with a whole component and a fractional component.  The angle
// is 9 bits whole, six bits fractional and the distance is 14 bits whole and two bits fractional.  The type has
// methods that extract and convert the values.
type RplidarResponseMeasurementNode struct {
	SyncQuality     uint8  // syncbit:1;syncbit_inverse:1;quality:6;
	AngleQ6Checkbit uint16 // check_bit:1;angle_q6:15;
	DistanceQ2      uint16
}

// OpResultToString takes an operation result value (opResult) and returns an appropriate error message.
func OpResultToString(opResult uint) string {
	switch opResult {
	case ResultOK:
		return ""
	case ResultInvalidData:
		return "Invalid data"
	case ResultFailBit:
		return "failed"
	case ResultAlreadyDone:
		return "already done"
	case ResultOperationfail:
		return "operation fail"
	case ResultOperationTimeout:
		return "operation timed out"
	case ResultOperationStop:
		return "stop"
	case ResultOperationNotSupported:
		return "operation not supported"
	case ResultFormatNotSupported:
		return "format not supported"
	case ResultInsufficientMemory:
		return "insufficient memory"
	default:
		return "unknown error"
	}
}

// SyncBit extract the sync bit from the measurement node.
func (mn RplidarResponseMeasurementNode) SyncBit() bool {
	if mn.SyncQuality&RplidarResponseMeasurementSyncBit == 0 {
		return false
	}
	return true
}

// CheckBit extract the check bit from the measurement node.
func (mn RplidarResponseMeasurementNode) CheckBit() bool {
	if mn.AngleQ6Checkbit&RplidarResponseMeasurementCheckBit == 0 {
		return false
	}
	return true
}

// Quality extracts the quality value from the measurement node.
func (mn RplidarResponseMeasurementNode) Quality() uint16 {
	return uint16(mn.SyncQuality >> RplidarResponseMeasurementQualityShift)
}

// AngleAsInt extracts the angle from the measurement node as a fixed point integer with
// a 9 bit whole part and a 6 bit fractional part.
func (mn RplidarResponseMeasurementNode) AngleAsInt() uint16 {
	return uint16(mn.AngleQ6Checkbit >> RplidarResponseMeasurementAngleShift)
}

// AngleAsFloat32 extracts the angle from the measurement node and returns it as a float.
// The angle will be in the range 0.0 to 511.99
func (mn RplidarResponseMeasurementNode) AngleAsFloat32() float32 {
	return float32(mn.AngleQ6Checkbit>>RplidarResponseMeasurementAngleShift) / 64.0
}

// AngleAsInt extracts the distance from the measurement node as a fixed point integer with
// a 14 bit whole part and 2 bit fractional part.
func (mn RplidarResponseMeasurementNode) DistanceAsInt() uint16 {
	return uint16(mn.DistanceQ2)
}

// AngleAsFloat32 extracts the angle from the measurement node and returns it as a float.
// The angle will be in the range 0.0 to 511.99
func (mn RplidarResponseMeasurementNode) DistanceAsFloat32() float32 {
	return float32(mn.DistanceQ2) / 4.0
}

type Driver interface {
	// DestroyDriver cleans up the data created by making the Driver.  Whenever a drier is made there should be
	// a deferred call to DestroyDriver to clear the resources that were allocated.
	DestroyDriver()

	// SetProxyDriver sets the proxy driver.
	SetProxyDriver(driver Driver)

	// SetVerbose sets the verbose flag which turns debugging on and off.
	SetVerbose(verbose bool)

	// Connect connects to the given USB port at the given baud rate. On a Windows machine the port can be
	// something like "com10" or "\\.\com10", on another OS something like "/dev/ttyUSB4".  If the port is an
	// empty string the default is used - \\.\com3" on a Windows machine otherwise "/dev/ttyUSB0".  If the
	// baud rate is 0, the default 115200 is used.  The flag is reserved for future use and should always be
	// set to zero.
	Connect(port string, baudRate uint, flag uint) uint

	// Disconnect disconnects from the RPLIDAR and closes the USB port.
	Disconnect()

	// IsConnected returns true if the driver is connected to a USB port.
	IsConnected() bool

	// Reset asks the RPLIDAR core system to reset itself. The host system can use the Reset operation to help the
	// RPLIDAR escape the self-protection mode.
	// The operation timeout for the serial port communication is in milliseconds.  If the timeout is zero, the
	// default of 2000 milliseconds is used.
	Reset(timeout uint) uint

	// Retrieve the health status of the RPLIDAR.  The host system can use this operation to check whether the
	// RPLIDAR is in the self-protection mode.  The HealthInfo object tetirned in cludes an error code, not to
	// be confused with the opResult value, which describes errors found whule fetching the data, for exaple,
	// a request timeout.
	// The operation timeout for the serial port communication is in milliseconds.  If the timeout is zero, the
	// default of 2000 milliseconds is used.
	GetHealth(timeout uint) (HealthInfo, uint)

	// GetDeviceInfo gets the device information of the RPLIDAR include the serial number, firmware version,
	// device model etc.
	// The operation timeout for the serial port communication is in milliseconds.  If the timeout is zero, the
	// default of 2000 milliseconds is used.
	GetDeviceInfo(timeout uint) (DeviceInfo, uint)

	// GetSampleDuration_uS get the sample duration information of the RPLIDAR.
	// The operation timeout for the serial port communication is in milliseconds.  If the timeout is zero, the
	// default of 2000 milliseconds is used.
	// The return values are:
	//    opResult (showing success or failure)
	//    sampleDurationUS - in normal mode, the smaple duration in microseconds, otherwise zero
	//    expressSampleDurationUs - in express mode the sample duration in milliseconds, otherwise zero
	GetSampleDuration_uS(timeout uint) (uint, uint16, uint16)

	// SetMotorPWM sets the RPLIDAR's motor pwm when using accessory board.. The comment in the C++ layer says
	// "currently  valid for A2 only".  This may be out of date.
	//
	// pwm - the motor pwm value to set
	//
	// The opResult is returned.
	SetMotorPWM(pwm uint16) uint

	// StartMotor starts the RPLIDAR's motor when using the accessory board.
	StartMotor() uint

	// StopMotor stops the RPLIDAR's motor when using the accessory boar
	StopMotor() uint

	// CheckMotorCtrlSupport checkss whether the device supports motor control.  Note: this API will disable grab.
	// The operation timeout for the serial port communication is in milliseconds.  If the timeout is zero, the
	// default of 2000 milliseconds is used.
	//
	// The return values are the opResult and a boolean which is true if this RPLIDAR supports motor control.
	CheckMotorCtrlSupport(timeout uint) (uint, bool)

	// GetFrequency calcuates the RPLIDAR's current scanning frequency from the given scan data.
	// Please refer to the application note doc for details.
	// Remark: the calcuation will be incorrect if the specified scan data doesn't contains enough data.
	//
	// inExpressMode Indicate whether the RPLIDAR is in express mode.
	//
	// count is the number of sample nodes inside the given buffer.
	//
	// returns:
	//     opResult - 0 if successful, non zero if an error occured
	//     frequency - the scanning frequency (in HZ) calcuated by the interface.
	//     is4kmode - true if the RPLIDAR is working on 4k sample rate mode.
	GetFrequency(inExpressMode bool, count uint) (uint, float32, bool)

	// CheckExpressScanSupported checks whether the RPLIDAR device supports express mode.
	// The operation timeout for the serial port communication is in milliseconds.  If the timeout is zero, the
	// default of 2000 milliseconds is used.
	// Returns the operation result and (if successful) true if the device supports express mode.
	CheckExpressScanSupported(timeout uint) (uint, bool)

	// StartScan asks the RPLIDAR core system to enter the scan mode(Normal/Express, Express mode is 4k mode).
	// A background thread in the C++ layer will be created by the RPLIDAR driver to fetch the scan data continuously.
	// User Application can use the grabScanData() interface to retrieved the scan data cached previous by this background
	// thread.
	//
	// force - force the core system to output scan data whether the scanning motor is rotating or not.
	//
	// autoExpressMode - force the core system to try express mode first, if the system does not support express mode, it
	// will use normal mode.
	//
	// Return opResult.
	StartScan(force bool, autoExpress bool) uint

	// StartScanNormal starts a scan running in normal mode.
	// A background thread in the C++ layer will be created by the RPLIDAR driver to fetch the scan data continuously.
	// User Application can use the grabScanData() interface to retrieved the scan data cached previous by this background
	// thread.
	// The operation timeout for the serial port communication is in milliseconds.  If the timeout is zero, the
	// default of 2000 milliseconds is used.
	StartScanNormal(force bool, timeout uint) uint

	// StartScanExpress starts a scan in express mode (assuming that the hardware supports express mode).
	// A background thread in the C++ layer will be created by the RPLIDAR driver to fetch the scan data continuously.
	// User Application can use the grabScanData() interface to retrieved the scan data cached previous by this background
	// thread.
	// The operation timeout for the serial port communication is in milliseconds.  If the timeout is zero, the
	// default of 2000 milliseconds is used.
	StartScanExpress(fixedAngle bool, timeout uint) uint

	// Stop ask the RPLIDAR core system to stop the current scan operation and enter idle state. The C++ background thread
	// is also terminated.
	// The operation timeout for the serial port communication is in milliseconds.  If the timeout is zero, the
	// default of 2000 milliseconds is used.
	Stop(timeout uint) uint

	// GrabScanData starts a C++ background thread which waits and grabs a complete 0-360 degree scan data previously received.
	// The grabbed scan data returned by this interface always has the following charactistics:
	//
	// 1) The first node of the grabbed data array (nodebuffer[0]) must be the first sample of a scan, i.e. the start_bit == 1
	// 2) All data nodes belong to exactly ONE complete 360-degrees's scan
	// 3) There may be less than the requested number of nodes.
	// 4) Valid nodes have a distance value greater than zero.  If the distance value is not zero, the other values can't be
	// trusted.
	// 5) The data may include a list of invalid nodes at the beginning or at the end.
	// 6) The angle data in one scan may not be ascending. (One teason is that it may start part-way round the circle, so you
	// may get 10-360 followed by 0-9.)
	//
	// You can use the method GrabAndSortScanData to get the nodes in ascending order of angle and correct the angle values in
	// the invalid nodes.
	//
	// scans: The number of measurements wanted.
	//
	// timeout: Max duration allowed to wait for a complete scan data.  Nothing will be stored to the nodebuffer if a complete
	// 360-degrees' scan data is not ready in time.   The caller application can set the timeout value to Zero(0) to make this
	// interface always returns immediately to achieve non-block operation.
	//
	// The interface will return ResultOperationTimeout to indicate that no complete 360-degrees' scan could be retrieved
	// within the given timeout duration.
	GrabScanData(scans uint, timeout uint) (uint, []RplidarResponseMeasurementNode)

	// GrabAndSortScanData makes the same call to the C++ layer as GrabScanData and then calls ascendScanData to sort the
	// measurement nodes in ascending order of angle.
	GrabAndSortScanData(scans uint, timeout uint) (uint, []RplidarResponseMeasurementNode)
}
