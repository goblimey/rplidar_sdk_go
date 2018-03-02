#include <stdio.h>
#include <stdlib.h>
#include "rptypes.h"
#include "driver.hpp"
#include "debug.h"

#define CLASS "cxxRPDriver"

/* Implementation of class cxxRPDriver */
      
cxxRPDriver::cxxRPDriver(char * com_path, _u32 com_baudrate) {
    debug2(CLASS, "cxxRPDriver(): calling ProxyDriver()");
    rpdriver = new ProxyDriver(com_path, com_baudrate);
    debug2(CLASS, "cxxRPDriver(): returning");
    return;
}

cxxRPDriver::~cxxRPDriver() {
    debug2(CLASS, "~cxxRPDriver(): calling ~ProxyDriver()");
    this->rpdriver->~ProxyDriver();
    this->rpdriver = NULL;
    debug2(CLASS, "~cxxRPDriver(): returning");
}

int cxxRPDriver::GetMajorVersionNumber() {
    debug2(CLASS, "GetMajorVersionNumber(): calling GetMajorVersionNumber()");
    int result = rpdriver->GetMajorVersionNumber();
    debug3(CLASS, "GetMajorVersionNumber(): returning", debugInt2String(result));
    return result;
}

int cxxRPDriver::GetMinorVersionNumber() {
    debug2(CLASS, "GetMinorVersionNumber(): calling GetMinorVersionNumber()");
    int result = rpdriver->GetMinorVersionNumber();
    debug3(CLASS, "GetMinorVersionNumber(): returning", debugInt2String(result));
    return result;
}

int cxxRPDriver::GetFirmwareId() {
    debug2(CLASS, "GetFirmwareId(): calling GetFirmwareId()");
    int result = rpdriver->GetFirmwareId();
    debug3(CLASS, "GetFirmwareId(): returning", debugInt2String(result));
    return result;
}

// Have to return 1 or 0 - CGO doesn't like bool.
int cxxRPDriver::CheckRPLidarHealth() {
    debug2(CLASS, "CheckRPLidarHealth(): calling CheckRPLidarHealth()");
    int result = this->rpdriver->CheckRPLidarHealth();
    debug3(CLASS, "CheckRPLidarHealth(): returning", debugInt2String(result));
    return result;
}

void cxxRPDriver::StartMotor() {
    debug2(CLASS, "StartMotor(): calling StartMotor()");
    this->rpdriver->StartMotor();
    debug2(CLASS, "StartMotor(): returning");
}

void cxxRPDriver::Stop() {
    debug2(CLASS, "Stop(): calling Stop()");
    this->rpdriver->Stop();
    debug2(CLASS, "Stop(): returning");
}

void cxxRPDriver::StopMotor() {
    debug2(CLASS, "StopMotor(): calling StopMotor()");
    this->rpdriver->StopMotor();
    debug2(CLASS, "StopMotor(): returning");
}
