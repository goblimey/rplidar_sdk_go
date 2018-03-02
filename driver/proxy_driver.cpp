#include <stdio.h>
#include <stdlib.h>
#include "rplidar.h"
#include "proxy_driver_impl.hpp"
#include "debug.h"

#define CLASS "ProxyDriver"

using namespace rp::standalone::rplidar;

/* methods of class ProxyDriver */
      
ProxyDriver::ProxyDriver(char * com_path, _u32 com_baudrate) {
    debug2(CLASS, "ProxyDriver()");
    if (com_baudrate == 0) {
        this->baud_rate = 115200;
    } 

    // read serial port from the command line...
    if (com_path == NULL) {
        // Set to the default value: e.g. "com3"
#ifdef _WIN32
        // use Windoze default com port
        this->com_path = "\\\\.\\com3";
#else
        this->com_path = "/dev/ttyUSB0";
#endif
        
    } else {
        this->com_path = com_path;
    }

    // create the driver instance

    debug2(CLASS, "ProxyDriver() creating the serial port driver");

    this->rpdriver = RPlidarDriver::CreateDriver(RPlidarDriver::DRIVER_TYPE_SERIALPORT);
 
    if (this->rpdriver == NULL) {
        fprintf(stderr, "insufficent memory, exit\n");
        return;
    }

    // make connection...
    debug4(CLASS, "ProxyDriver() connecting to the serial port", this->com_path, debugInt2String(this->baud_rate));
    if (IS_FAIL(this->rpdriver->connect(this->com_path, this->baud_rate))) {
        fprintf(stderr, "Error, cannot bind to the specified serial port %s.\n",
            this->com_path);
        debug2(CLASS, "ProxyDriver disposing of driver");
        RPlidarDriver::DisposeDriver(this->rpdriver);
        this->rpdriver = NULL;
        return;
    }

    // Display the device serial number, firmware and hardware version number..
    printf("RPLIDAR S/N: ");
    printf("\n"
        "Firmware Ver: %d.%02d\nHardware Rev: %d\n",
         GetMajorVersionNumber(),
         GetMinorVersionNumber(),
         GetFirmwareId());

    u_result     op_result;
    rplidar_response_device_info_t devInfo;
	// retrieve the device info
    op_result = this->rpdriver->getDeviceInfo(devInfo);

    if (IS_FAIL(op_result)) {
        fprintf(stderr, "Error, cannot get device info.\n");
        debug2(CLASS, "~ProxyDriver disposing of driver");
        RPlidarDriver::DisposeDriver(this->rpdriver);
        this->rpdriver = NULL;
        return;
    }

    // print out the device serial number, firmware and hardware version number..
    printf("RPLIDAR S/N: ");
    for (int pos = 0; pos < 16 ;++pos) {
        printf("%02X", devInfo.serialnum[pos]);
    }

    printf("\n"
            "Firmware Ver: %d.%02d\n"
            "Hardware Rev: %d\n"
            , devInfo.firmware_version>>8
            , devInfo.firmware_version & 0xFF
            , (int)devInfo.hardware_version);



    // check health...
    debug2(CLASS, "check health");
    if (this->CheckRPLidarHealth() == 0) {
        debug2(CLASS, "~ProxyDriver disposing of driver");
        RPlidarDriver::DisposeDriver(this->rpdriver);
        this->rpdriver = NULL;
        return;
    }
}

ProxyDriver::~ProxyDriver() {
    debug2(CLASS, "~ProxyDriver disposing of driver");
    RPlidarDriver::DisposeDriver(this->rpdriver);
    this->rpdriver = NULL;
}

/*
// Factory method.
ProxyDriver ProxyDriver::CreateDriver(char * com_path, _u32 com_baudrate) {
    return ProxyDriver(com_path, com_baudrate);
}
*/

int ProxyDriver::GetMajorVersionNumber() {
    debug2(CLASS, "GetMajorVersionNumber()");
    u_result op_result;
    rplidar_response_device_info_t devInfo;
	// retrieve the device info and set the field
    op_result = this->rpdriver->getDeviceInfo(devInfo);

    if (IS_FAIL(op_result)) {
        fprintf(stderr, "error getting device info - %d", op_result);
        return -1;
    }
    return devInfo.firmware_version>>8;
}

int ProxyDriver::GetMinorVersionNumber() {
    debug2(CLASS, "GetMinorVersionNumber()");
    u_result op_result;
    rplidar_response_device_info_t devInfo;
	// retrieve the device info and set the field
    op_result = this->rpdriver->getDeviceInfo(devInfo);

    if (IS_FAIL(op_result)) {
        fprintf(stderr, "error getting device info - %d", op_result);
        return -1;
    }
    return devInfo.firmware_version & 0xFF;
}

int ProxyDriver::GetFirmwareId() {
    debug2(CLASS, "GetFirmwareId()");
    u_result op_result;
    rplidar_response_device_info_t devInfo;
	// retrieve the device info and set the field
    op_result = this->rpdriver->getDeviceInfo(devInfo);

    if (IS_FAIL(op_result)) {
        fprintf(stderr, "error getting device info - %d", op_result);
        return -1;
    }
    return (int)devInfo.hardware_version;
}

// Have to return 1 or 0 - CGO doesn't like bool.
int ProxyDriver::CheckRPLidarHealth() {
    debug2(CLASS, "CheckRPLidarHealth()");
    u_result     op_result;
    rplidar_response_device_health_t healthinfo;

    op_result = this->rpdriver->getHealth(healthinfo);
    if (IS_OK(op_result)) { // the macro IS_OK is the preferred way to judge whether the operation is succeed.
        printf("RPLidar health status : %d\n", healthinfo.status);
        if (healthinfo.status == RPLIDAR_STATUS_ERROR) {
            fprintf(stderr, "Error, rplidar internal error detected. Please reboot the device to retry.\n");
            // enable the following code if you want rplidar to be reboot by software
            // this->driver->reset();
            return 0;
        } else {
            return 1;
        }

    } else {
        fprintf(stderr, "Error, cannot retrieve the lidar health code: %x\n", op_result);
        return 0;
    }
}

void ProxyDriver::StartMotor() {
    debug2(CLASS, "StartMotor()");
    this->rpdriver->startMotor();
}

void ProxyDriver::Stop() {
    debug2(CLASS, "Stop()");
    this->rpdriver->stop();
}

void ProxyDriver::StopMotor() {
    debug2(CLASS, "StopMotor()");
    this->rpdriver->stopMotor();
}
