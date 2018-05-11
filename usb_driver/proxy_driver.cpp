#include <stdio.h>
#include <stdlib.h>
#include "rplidar.h"
#include "driver.h"
#include "rplidar_go_if.h"
#include "proxy_driver_impl.hpp"
#include "debug.hpp"
#include <string.h>

#define CLASS "cxxProxyDriver"

using namespace rp::standalone::rplidar;

/* methods of class cxxProxyDriver */
      
cxxProxyDriver::cxxProxyDriver(int verbose) {

    debug(verbose, CLASS, "cxxProxyDriver() creating the serial port driver");

    if (verbose == 0) {
        this->verbose = false;
    } else {
        this->verbose = true;
    }

    this->rpdriver = RPlidarDriver::CreateDriver(RPlidarDriver::DRIVER_TYPE_SERIALPORT);
 
    if (this->rpdriver == NULL) {
        fprintf(stderr, "insufficent memory, exit\n");
        return;
    }
    return;
}

cxxProxyDriver::~cxxProxyDriver() {
    debug(verbose, CLASS, "~cxxProxyDriver disposing of driver");
    RPlidarDriver::DisposeDriver(this->rpdriver);
    this->rpdriver = NULL;
}

unsigned int cxxProxyDriver::Connect(const char * port_path, unsigned int baudrate, unsigned int flag) {
    debug(verbose, CLASS, "Connect():");
    char * opt_com_path = NULL;
    if (port_path != NULL && strcmp(port_path, "") != 0) {
        // port_path is set
        opt_com_path = (char*)port_path;
    }
    if (opt_com_path == NULL) {
#ifdef _WIN32
        // use default com port
        opt_com_path = (char*)"\\\\.\\com3";
#else
        opt_com_path = (char*)"/dev/ttyUSB0";
#endif
    }
    debug(verbose, CLASS, "port", opt_com_path, "baudrate", debugInt2String(verbose, baudrate));
    
    u_result op_result = this->rpdriver->connect(opt_com_path, baudrate, flag);
    if (IS_FAIL(op_result)) {
        debug(verbose, CLASS, "Connect(): error connecting", debugInt2HexString(verbose, op_result));
        return op_result;
    }

    debug(verbose, CLASS, "Connect(): connected successfully");
    return op_result;
}

void cxxProxyDriver::Disconnect() {
    debug(verbose, CLASS, "Disconnect()"); 
    this->rpdriver->disconnect();
    debug(verbose, CLASS, "Disconnect(): disconnected"); 
}

int cxxProxyDriver::IsConnected() {
    debug(verbose, CLASS, "IsConnected()"); 
    bool result = this->rpdriver->isConnected();
    if (result) {
        debug(verbose, CLASS, "IsConnected(): returning 1");
    } else {
        debug(verbose, CLASS, "IsConnected(): returning 0");
    }
    return result?1:0;
}

unsigned int cxxProxyDriver::Reset(unsigned int timeout) {
    debug(verbose, CLASS, "Reset(): timeout", timeout); 
    u_result op_result = this->rpdriver->reset(timeout);
    if (IS_FAIL(op_result)) {
        fprintf(stderr, "reset(): error resetting - %d", op_result);
        return op_result;
    }

    debug(verbose, CLASS, "Reset(): reset successfully");
    return op_result;
}

health_response cxxProxyDriver::GetHealth(unsigned int timeout) {
    debug(verbose, CLASS, "GetHealth()");
    health_response result;
    rplidar_response_device_health_t health_info;

    result.op_result = this->rpdriver->getHealth(health_info, timeout);
    if (IS_FAIL(result.op_result)) {
        return result;
    }

    rplidar_response_device_health_unpacked health_info_unpacked;
    health_info_unpacked.status = health_info.status;
    health_info_unpacked.error_code = health_info.error_code;
    result.health_info = health_info_unpacked;
    debug(verbose, CLASS, "GetHealth(): health status", result.health_info.status);
    
    if (result.health_info.status == RplidarStatusError) {
        fprintf(stderr, "%s GetHealth(): Error, rplidar internal error detected. Please reboot the device to retry.\n", CLASS);
    }

    return result;
}

rplidar_device_info_unpacked* cxxProxyDriver::GetDeviceInfo(unsigned int timeout) {
    debug(verbose, CLASS, "GetDeviceInfo()");
    rplidar_device_info_unpacked* result = 
            (rplidar_device_info_unpacked*) malloc(sizeof(rplidar_device_info_unpacked)); 
    rplidar_response_device_info_t devInfo;
	// retrieve the device info and set the field
    result->op_result = this->rpdriver->getDeviceInfo(devInfo, timeout);

    if (IS_FAIL(result->op_result)) {
        fprintf(stderr, "error getting device info - %d", result->op_result);
        return result;
    }

    result->model = devInfo.model;
    result->majorFirmwareVersion = devInfo.firmware_version>>8;
    result->minorFirmwareVersion = devInfo.firmware_version & 0xFF;
    result->hardwareVersion = (int)devInfo.hardware_version;
    char* sn = (char*)malloc((RPLIDAR_SERIAL_NUMBER_LENGTH * 2) + 1);
    *sn = '\0';
    char* buffer = (char*)malloc(3);
    for (int i = 0; i < RPLIDAR_SERIAL_NUMBER_LENGTH; ++i) {
        sprintf(buffer, "%02X", devInfo.serialnum[i]);
        strcat(sn, buffer);
    }
    free(buffer);
    result->serialNumber = sn;
    return result;
}

rplidar_response_sample_rate_unpacked cxxProxyDriver::GetSampleDuration_uS(_u32 timeout) {
    debug(verbose, CLASS, "getSampleDuration_uS()");
    u_result     op_result;
    rplidar_response_sample_rate_unpacked result;
    rplidar_response_sample_rate_t rateInfo;
    op_result = this->rpdriver->getSampleDuration_uS(rateInfo, timeout);
    result.op_result = op_result;
    if (IS_OK(op_result)) {
        result.std_sample_duration_us = rateInfo.std_sample_duration_us;
        result.express_sample_duration_us = rateInfo.express_sample_duration_us;
    }
    return result;
}

unsigned int cxxProxyDriver::SetMotorPWM(_u16 pwm) {
    debug(verbose, CLASS, "setMotorPWM()");
    u_result op_result = this->rpdriver->setMotorPWM(pwm);
    if IS_FAIL(op_result) {
        debug(verbose, CLASS, "failed");
    } else {
        debug(verbose, CLASS, "success");
    }
    return op_result;
}


unsigned int cxxProxyDriver::StartMotor() {
    debug(verbose, CLASS, "StartMotor()");
    return this->rpdriver->startMotor();
}

unsigned int cxxProxyDriver::StopMotor() {
    debug(verbose, CLASS, "StopMotor()");
    return this->rpdriver->stopMotor();
}

rplidar_motor_ctrl_support_status cxxProxyDriver::CheckMotorCtrlSupport(_u32 timeout) {
    debug(verbose, CLASS, "CheckMotorCtrlSupport()");
    bool support = false;
    rplidar_motor_ctrl_support_status status;
    status.op_result = this->rpdriver->checkMotorCtrlSupport(support, timeout);

    if (IS_FAIL(status.op_result)) {
        debug(verbose, CLASS, "CheckMotorCtrlSupport() failed");
        return status;
    }
    debug(verbose, CLASS, "CheckMotorCtrlSupport() returning");
    status.support = support;
    return status;
}

frequency_data cxxProxyDriver::GetFrequency(int inExpressMode, size_t count) {
    debug(verbose, CLASS, "GetFrequency()");
    float frequency;
    float& refFrequency = frequency;
    bool is4kmode;
    bool& refIs4kmode = is4kmode;
    u_result op_result = this->rpdriver->getFrequency(inExpressMode, count, refFrequency, refIs4kmode);
    frequency_data fd;
    if (IS_FAIL(op_result)) {
        fd.op_result = op_result;
        fd.frequency = 0.0;
        fd.is4kmode = 0;
    } else {
        fd.op_result = op_result;
        fd.frequency = frequency;
        fd.is4kmode = is4kmode ? 1 : 0;
    }
    debug(verbose, CLASS, "GetFrequency() returning");
    return fd;
}

express_mode_support_status cxxProxyDriver::CheckExpressScanSupported(unsigned int timeout) {
    debug(verbose, CLASS, "checkExpressScanSupported()");
    bool supported;
    express_mode_support_status status;
    status.op_result = this->rpdriver->checkExpressScanSupported(supported, timeout);
    if IS_FAIL(status.op_result) {
        return status;
    }
    status.supported = supported?-1:0;
    debug(verbose, CLASS, "checkExpressScanSupported() returning");
    return status;

}

unsigned int cxxProxyDriver::StartScan(bool force, bool autoExpressMode) {
    debug(verbose, CLASS, "StartScan()");
    u_result result = this->rpdriver->startScan(force, autoExpressMode);
    debug(verbose, CLASS, "StartScan() returning");
    return result;
}

unsigned int cxxProxyDriver::StartScanNormal(bool force, _u32 timeout) {
    debug(verbose, CLASS, "StartScanNormal()");
    u_result result = this->rpdriver->startScanNormal(force, timeout);
    debug(verbose, CLASS, "StartScanNormal() returning");
    return result;
}

unsigned int cxxProxyDriver::StartScanExpress(bool fixedAngle, _u32 timeout) {
    debug(verbose, CLASS, "startScanExpress()");
    u_result result = this->rpdriver->startScanExpress(fixedAngle, timeout);
    debug(verbose, CLASS, "startScanExpress() returning");
    return result;
}

unsigned int cxxProxyDriver::Stop(unsigned int timeout) {
    debug(verbose, CLASS, "Stop(): timeout", timeout);
    return this->rpdriver->stop(timeout);
}

rplidar_scan_results_unpacked* cxxProxyDriver::GrabScanData(unsigned int givenScans, unsigned int timeout) {
    debug(verbose, CLASS, "GrabScanData()");

    size_t scans = givenScans;
    size_t & refScans = scans;
    
    // This is freed before the method returns.
    rplidar_response_measurement_node_t* nodes =
            (rplidar_response_measurement_node_t*)
            malloc(scans * sizeof(rplidar_response_measurement_node_t));

    // This must be freed by the caller.
    rplidar_scan_results_unpacked* result = 
        (rplidar_scan_results_unpacked*) malloc(sizeof(rplidar_scan_results_unpacked));

    result->op_result = this->rpdriver->grabScanData(nodes, refScans, timeout);

    if (IS_FAIL(result->op_result)) {
        fprintf(stderr, "%s GrabScanData(): error while scanning data - %d\n", CLASS, result->op_result);
        free(nodes);
        return result;
    }

    debug(verbose, CLASS, "GrabScanData(): got", debugInt2String(verbose, scans), "scans");

    // This must be freed by the caller.
    result->nodes = unpackScanResult(scans, nodes);
    
    result->scans = scans;   // number of scans collected

    // Free the packed scan data.
    free(nodes);

    debug(verbose, CLASS, "GrabScanData(): returning", debugInt2String(verbose, scans), "scans");
    return result;   
}

rplidar_scan_results_unpacked* cxxProxyDriver::GrabAndSortScanData(unsigned int givenScans, unsigned int timeout) {
    debug(verbose, CLASS, "GrabAndSortScanData()");
    
    size_t scans = givenScans;
    size_t & refScans = scans;

    // This is freed before the method returns.
    rplidar_response_measurement_node_t* nodes =
            (rplidar_response_measurement_node_t*)
            malloc(scans * sizeof(rplidar_response_measurement_node_t));
    
    rplidar_scan_results_unpacked* result = 
        (rplidar_scan_results_unpacked*) malloc(sizeof(rplidar_scan_results_unpacked));

    result->op_result = this->rpdriver->grabScanData(nodes, refScans, timeout);

    if (IS_FAIL(result->op_result)) {
        fprintf(stderr, "%s GrabAndSortScanData(): error while scanning data - %d\n", CLASS, result->op_result);
        free(nodes);
        return result;
    }

    debug(verbose, CLASS, "GrabAndSortScanData(): sorting", debugInt2String(verbose, scans), "scans");

    result->op_result = this->rpdriver->ascendScanData(nodes, scans);

    if (IS_FAIL(result->op_result)) {
        fprintf(stderr, "%s GrabAndSortScanData(): error while sorting data - %d\n", CLASS, result->op_result);
        free(nodes);
        return result;
    }
    debug(verbose, CLASS, "GrabAndSortScanData(): unpacking", debugInt2String(verbose, scans), "scans");

    // This allocates memory that must be freed by the caller.
     result->nodes = unpackScanResult(scans, nodes);

    result->scans = scans;

    // Free the packed results before returning
    free(nodes);
    
    debug(verbose, CLASS, "GrabAndSortScanData(): returning", debugInt2String(verbose, scans), "scans");
    return result;   
}

// unpackScanResults() copies the packed measurement nodes to an unpacked version that the Go layer can access.
rplidar_response_measurement_node_unpacked* cxxProxyDriver::unpackScanResult(unsigned int scans, rplidar_response_measurement_node_t* nodes) {
    // This must be freed by the caller.
    rplidar_response_measurement_node_unpacked* unpackedNodes =
        (rplidar_response_measurement_node_unpacked*)
        malloc(scans * sizeof(rplidar_response_measurement_node_unpacked));

    // Upack the data so that the Go layer can handle it.
    for (int i = 0; i < scans; i++) {
        (unpackedNodes+i)->sync_quality = (nodes+i)->sync_quality;
        (unpackedNodes+i)->angle_q6_checkbit = (nodes+i)->angle_q6_checkbit;
        (unpackedNodes+i)->distance_q2 = (nodes+i)->distance_q2;
        if (verbose) {
            debug(verbose, CLASS, "unpackScanResult(): ", debugInt2String(verbose, i), "sync_quality", 
                debugInt2HexString(verbose, (unpackedNodes+i)->sync_quality),
                "angle_q6_checkbit", debugInt2HexString(verbose, (unpackedNodes+i)->angle_q6_checkbit),
                "distance_q2", debugInt2HexString(verbose, (unpackedNodes+i)->distance_q2));
        }
    }

    return unpackedNodes;
}




