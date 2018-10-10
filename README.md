# rplidar_sdk_go
A Go wrapper for the RPLidar SDK

This project defines a Go driver for the Slamtec RPLIDAR device.
It wraps a C++ driver
supplied by Slamtec.

See the end of this page for detailed Go docs.

RPLidar is a range of devices intended for Simultaneous 
Localisation And Mapping (SLAM) applications.
An RPLidar uses a laser sensor
mounted on a spinning platform to scan the nearby objects in a circle.
It returns a list of angle and distance values.
The original purpose of the device was to produce a floor plan of the room in which
a robot vacuum cleaner was running.
By moving the robot around,
taking a series of scans from different points
and merging them,
an accurate view of the room that it's working in
can be produced.

More advanced version of the scanner
are now used for all sorts of purposes including solid modelling
and robot vision.

The scanner connects to a host computer via USB so it's very easy to get it working.


Slamtec's C++ driver is open source and
can be used under Windows, Linux or the Mac.
This package allows Go software to use Slamtec's driver,
but so far I've only got it working under Linux on an Intel processor.
my attempts to get it working under Windows failed due to incompatible tools.
I'm assuming that it will work on an ARM-based system such as a Raspberry Pi,
but I haven't tried it yet.
I haven't tried the Mac version either.

The photo setup.jpg
shows the RPLidar working, controlled by my grabanddraw program.
The resulting floor plan is in floorplan.png.
My program is very simple
and it has some shortcomings
that cause glitches in the scan.
The scanner is actually pretty effective,
especialy in the environment that it was designed for. 

RPLidar is a range of scanners.
Different models produce results of different accuracy.
Currently the most accurate (and most expensive)
is the A3 which claims a range 0f 150mm - 25m,
angular resolution of 0.3 degrees
and an accuracy in the distance measurement of less than 1%.

I have the A2 model.
I ran some fairly simple tests to check the accuracy of my scanner
and wrote them up
[here](https://groups.google.com/forum/#!topic/lidar-mapping/scWmwVFyKMU).

To get the Go driver working, you need to download the RPLidar SDK and build it.
You need a C++ compiler to do that.
If you don't already have one, the free [GNU compiler](https://gcc.gnu.org/) should do the job.
See the Download links on the right of that page.
You will also need the free [GNU Make tool](https://www.gnu.org/software/make/)
if you don't have one of those.
The Windows version requires a particular version of Visual Studio.
 (VS-2014 with Service Pack 2, from memory.)
 Best of luck getting that to work.

Download the RPLidar SDK from [here](https://www.slamtec.com/en/Support/).

Unzip the distribution into a directory somewhere.  Unzipping it produces a directory rplidar.

The implementation of the interface is in rplidar/sdk/sdk/src/rplidar_driver.cpp.
There are example applications in rplidar/sdk/app.
The comments in those apps give a lot of useful clues about how the driver works.

To build the library:

    $ cd rplidar/sdk
    $ make clean
    $ make

In a linux environment, this produces a library in sdk/output/Linux/Release/librplidar_sdk.a

Next, download the Go wrapper:

    $ go get github.com/goblimey/rplidar_sdk_go/simple_grabber_go

THIS WILL THROW AN ERROR because it needs a copy of the library in the right place before it can build the wrapper.

In your Go projects directory you now have github.com/goblimey/rplidar_sdk_go and within that a directory lib.

Take the C++ library that you built earlier and copy it into the lib directory.

Now you can build the Go wrapper and the applications:

    $ go install github.com/goblimey/rplidar_sdk_go/simple_grabber_go
    $ go install github.com/goblimey/rplidar_sdk_go/ultra_simple_go
    $ go install github.com/goblimey/rplidar_sdk_go/grabanddraw

Run the applications something like this:

    $ grabanddraw -f scan.png   # grab a scan and draw it in scan.png.

or you can run it in verbose mode and override some of te default options:

    $ grabanddraw -v -f=scan.png -port=/dev/ttyUSB1 -speed=96000

On Linux the default port is /dev/ttyUSB0.

The speed value defines the baud rate which the host computer uses
to talk across the USB connection.
The default is 115200 baud,
which matches the speed that the scanner uses by default.
Both ends must use the same speed.
If they disagree, communication will fail.

The -v option turn on verbose mode, which producing lots of tracing messages.

The name of the USB port differs for different systems and depends on how many USB devices you have plugged in.
On Linux your USB ports are /dev/ttyUSB0, /dev/ttyUSB1 and so on.  If your RPLidar device is the only USB device
plugged in, it will be on /dev/ttyUSB0.

The Go examples are based on Slamtec's C++ examples.
The simplest is ultra_simple_go.
It starts the scanner, grabs a scan and displays the scan data on the standard output channel.
The grabanddraw program is a bit more useful.
It grabs a scan and draws the resulting floor plan
as a .png file.
It contains a bit of useful geometry work.

See below for the godoc describing the driver interface.


The RPLidar Device
==================

The documentation for the A3 model is (here)[https://www.slamtec.com/en/Lidar/A3Spec].

I did some experiments with my A2 model,
which I wrote up in the Google group lidar mapping
[here](https://groups.google.com/forum/#!topic/lidar-mapping/scWmwVFyKMU).

The default speed of the spinning platform is 720 revs per minute (12 per second).
A scan produces a list of measurements,
each containing an angle in degrees and a distance in millimetres.
This is the distance to the nearest object at that angle.
The device has a data cable coming out and
the front (0 degrees) is the point opposite that.

When the grabanddraw program runs a scan, it draws it with the device pointing upwards.

The control software runs on a host computer connected to the scanner via a USB cable.
The StartScan operations starts the device scanning.
It scans continuously, collecting measurements until the controller issues a Stop operation.

If the GrabScanData operation is run while the device is scanning,
it grabs a full circle of measurements from the device
starting at whatever angle the platform is pointing.
So you might get 300 measurements starting at 35.6 degrees and incrementing by
1.8 degrees each time,
round to 360 degrees and then continuing,
up to 33.8 degrees.

The scanner may not see an object at every angle.
When it can't get a measurement it returns an angle of zero and a distance of zero
to indicate an invalid value.
The list of measurements is usually a mixture of valid and invalid values,
something like this:
```
      angle     distance
        0           0           invalid
        0           0           invalid
       35.6      1025.3         valid
       37.4      1023.6         valid
       39.2      1022.1         valid
```
and so on.

The distance values are reported to fractions of a millimetre,
but in reality the accuracy is much less than that.
It's different for each RPLidar model.
The more you pay, the more accuracy you get.
My device claims to be accurate to about 10 millimetres at this range
so the valid distance values shown above are really something like
1020-1030, 1020-1030 and 1017-1027.

The invalid angle values
and the fact that the list is out of order can make it difficult to process the data.
The GrabAndSortScanData operation corrects the angle parts of the measurements
and sorts the list by angle.
So in that list all the angle values are non-zero
and the invalid measurements are indicated by just a distance of zero:
```
      angle     distance
       32.0         0           invalid
       33.8         0           invalid
       35.6      1025.3         valid
       37.4      1023.6         valid
       39.2      1022.1         valid
```

The Driver
==========

In the Go layer,
the Driver interface defines a set of types used to return
results,
some constants used to interpret those data
and the methods that implement the operations.

For example, GrabScan() grabs scan dataand return it as a slice of
RplidarResponseMeasurementNode objects,
each of which contains an angle and a distance measurement plus other stuff such
as check bits and a quality measure.
These data are packed together into integer values.

Angle and distance measurements are returned as fixed point integers.
The angle shares space with other values
so you have to extract it using bit shifting and masking.
It has 9 bits whole part, 6 bits fractional,
so the fractional part is 0 to 63.
You can extract these parts in the same way,
or just convert the value to a floating point number
by dividing it by 64.0.

The distance value uses 16 bits, 14 bits whole part, 2 bits fractional,
so the fractional part is 0 to 3.
Dividing it by 4.0
gives the float value.

The Go wrapper includes methods to do all this, for example:

   angle := measurementNode.AngleAsFloat64()

extracts the angle value as a 64-bit float.

   angle := measurementNode.AngleAsFloat32()

extracts it as a 32-bit float.

The RPLidar has a limited range,
so the numbers involved are always small.
If memory space is at a premium, you may want keep them in 32-bit form,
but software such as the math library tend to work in 64-bit mode,
which leads to lots of type conversions:

    radians := float64(measurement.AngleAsFloat32()) * math.Pi / 180
    x = math.Cos(radians) * float64(distance)

Most operations return an unsigned int, the operation result.
This can have various bits set to indicate different
kinds of error.
The IsOK and IsFail methods of the driver return true or false as appropriate.
The driver provides constants such as
ResultAlreadyDone that can be used to interpret the operation result.
See the example programs for usage.

The OpResultToString method converts the operation result into a string that can be use in
an error message.

Some errors may be recoverable, for
example, if the operation result is a timeout, a retry may work.

A long operation such as a scan grab may time out but
still produce some data.


Implementation Details
=====================

Go code can't call C++ code directly.
It can call a C function using the cgo mechanism, which can call a C++
method.
For cgo to work, the C++ code has to follow certain rules.
The code in the RPLidar C++ driver code doesn't,
so Go code can't call it directly.
I had to create an extra layer of C++, a proxy driver that provides a simpler interface that cgo can handle.

The end result is the Driver interface
and a concrete type USBDriver that implements it.
Each method of the concrete type
uses cgo to call a C interface function,
which calls a method in the C++ proxy,
which calls a method in the Slamtec driver.
This seems a little excessive,
but remember that
your application is talking across a wire
to a physical device with a spinning platform.
Given the delays involved in that,
the overhead of a few extra method calls is insignificant.

The usb_driver.go file contains some magic to glue the Go and C++ components together:

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

Behind the scenes, the go command runs the C++ compiler, which can compile both C++ and C code.
The #cgo comment lines specify directives for the C++ compiler.

    ${SRCDIR} defines the full path name of the current directory.
    -I says where to find the C and C++ header files.
    -l specifies the name of the library files, mostly standard ones in a known place.
    -L specifies the directory containing the non-standard library file.
    -DDEBUG turns on debugging code.
    -g include symbol table data.

The compiler knows where to find the standard libraries but
it needs to be told where to find the Slamtec library that you built and copied.
(-lrplidar_sdk means the file "librplidar_sdk.a").

The C++ library contains very little debugging code,
so enabling it doesn't do very much.
 
Including symbol table data with -g allows the system to add
source code line numbers to error messages.
The symbol table is also used by the debugger.

The #include lines are the C++ equivalent of a Go import directive.

The "C" import allows the Go code to refer to the C code, for example:

     driver C.RPDriver   

creates a variable driver of type RPDriver.
That's one of the types declared in the C layer.
It's the C interface to the C++ Driver class.

A Go method can call a C++ method like so:

	    C.RPDriverStartMotor(d.driver)

That gets turned into a call of the method startMotor()
in the class RPDriver.

Fortunately,
the Go wrapper takes care of all that malarky,
so you don't have to worry about it.

The cgo tool has a nasty habit of
occasionally rewriting the original source code,
including removing all comments.
For this reason
I avoid putting comments in any code that's processed by cgo,
in this case usb_driver.go.
I put them in the interface instead.

An alternative approach to implementing this Go driver would be to mimic the C++ driver line for line,
but whoever did that would need to maintain it as Slamtec releases
new versons.
As long as they respect their own interface in
any future version of the C++ driver,
this wrapper should work with it.

For more details of Go calling C++ code via a C interface,
read [the manual](https://golang.org/cmd/cgo/).
I also found
[this Go project](github.com/burke/howto-go-with-cpp) very useful.
It provides a simple worked example of Go calling C++.

I used [this Go library](https://github.com/fogleman/gg) to handle the graphics in the
grabanddraw program.

I had to write some layers of C and C++ for this project,
which was interesting,
as I haven't used those languages for over twenty years
Those bits of the solution
could probably be improved.

The godoc follows.
You can prodce this yourself like so:

    godoc cmd/github.com/goblimey/rplidar_sdk_go/driver


```
PACKAGE DOCUMENTATION

package driver
    import "github.com/goblimey/rplidar_sdk_go/driver"


CONSTANTS

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

    DefaultTimeout = 2000
)

FUNCTIONS

func IsFail(x uint) bool
    IsFail returns true if the result returned by an operation (the
    opResult) indicates that it failed. Note that a scan operation can
    return a timeout failure while also returning partial scan data.

func IsOK(x uint) bool
    IsOK returns true if the result returned by an operation (the opResult)
    indicates that it wassuccessful.

func OpResultToString(opResult uint) string
    OpResultToString takes an operation result value (opResult) and returns
    an appropriate error message.

TYPES

type DeviceInfo struct {
    Model                uint
    MajorFimwareVersion  uint
    MinorFirmwareVersion uint
    HardwareVersion      uint
    SerialNumber         string
}
    DeviceInfo contains the device information returned by GetDeviceInfo. It
    contains the model number of the RPLidar device, the major firmware
    version, the inor firmware version, the haradware version and the serial
    number, which is unique to each device.

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
    Driver is a wrapper around the C++ RPLidar driver.

type HealthInfo struct {
    Status    uint8
    ErrorCode uint16
}
    HealthInfo contains the system health information returned by GetHealth.
    It contains the status of the RPLidar device and an error code.

type RplidarResponseMeasurementNode struct {
    SyncQuality     uint8  // syncbit:1;syncbit_inverse:1;quality:6;
    AngleQ6Checkbit uint16 // check_bit:1;angle_q6:15;
    DistanceQ2      uint16
}
    RplidarResponseMeasurementNode holds a measurement of angle and distance
    along with sync and quality bits. The angle and distance components are
    fixed point numbers with a whole component and a fractional component.
    The angle is 9 bits whole, six bits fractional and the distance is 14
    bits whole and two bits fractional. The type has methods that extract
    and convert the values.

func (mn RplidarResponseMeasurementNode) AngleAsFloat32() float32
    AngleAsFloat32 extracts the angle from the measurement node and returns
    it as a float32. The angle is in degrees and goes anticlockwise.

func (mn RplidarResponseMeasurementNode) AngleAsFloat64() float64
    AngleAsFloat64 extracts the angle from the measurement node and returns
    it as a float64. The angle is in degrees and goes anticlockwise.

func (mn RplidarResponseMeasurementNode) AngleAsInt() uint16
    AngleAsInt extracts the angle from the measurement node as a fixed point
    integer with a 9 bit whole part and a 6 bit fractional part.

func (mn RplidarResponseMeasurementNode) CheckBit() bool
    CheckBit extract the check bit from the measurement node.

func (mn RplidarResponseMeasurementNode) DistanceAsFloat32() float32
    DistanceAsFloat32 extracts the angle from the measurement node and
    returns it as a float32. The distance is in millimetres, although it's
    less accurate than that, depending on the device.

func (mn RplidarResponseMeasurementNode) DistanceAsFloat64() float64
    DistanceAsFloat64 extracts the angle from the measurement node and
    returns it as a float64. The distance is in millimetres, although it's
    less accurate than that, depending on the device.

func (mn RplidarResponseMeasurementNode) DistanceAsInt() uint16
    DistanceAsInt extracts the distance from the measurement node as a fixed
    point integer with a 14 bit whole part and 2 bit fractional part. The
    distance is in millimetres, although it's less accurate than that,
    depending on the device.

func (mn RplidarResponseMeasurementNode) EndPoint() (x, y float64)
    EndPoint gets the x and y coordinates of the endpoint of the
    measurement.

func (mn RplidarResponseMeasurementNode) Invalid() bool
    Invalid is true if the distance value is 0 or less. (Actually in a real
    measurement it will never be less.)

func (mn RplidarResponseMeasurementNode) Quality() uint16
    Quality extracts the quality value from the measurement node.

func (mn RplidarResponseMeasurementNode) SyncBit() bool
    SyncBit extract the sync bit from the measurement node.

func (mn RplidarResponseMeasurementNode) Valid() bool
    Valid is true if the distance value is greater than 0


```
