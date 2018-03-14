#include <stdio.h>
#include <stdlib.h>
#include "rplidar.h"
#include "rplidar_go_if.h"
#include "proxy_driver_impl.hpp"
#include "debug.hpp"
#include <string.h>

#define CLASS "cxxProxyDriver"

using namespace rp::standalone::rplidar;

/* methods of class cxxProxyDriver */
      
cxxProxyDriver::cxxProxyDriver(char * com_path, unsigned int com_baudrate) {
    debug(CLASS, "cxxProxyDriver()");
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

    debug(CLASS, "cxxProxyDriver() creating the serial port driver");

    this->rpdriver = RPlidarDriver::CreateDriver(RPlidarDriver::DRIVER_TYPE_SERIALPORT);
 
    if (this->rpdriver == NULL) {
        fprintf(stderr, "insufficent memory, exit\n");
        return;
    }
}

cxxProxyDriver::~cxxProxyDriver() {
    debug(CLASS, "~cxxProxyDriver disposing of driver");
    RPlidarDriver::DisposeDriver(this->rpdriver);
    this->rpdriver = NULL;
}

unsigned int cxxProxyDriver::Connect(const char * port_path, unsigned int baudrate, unsigned int flag) {
    debug(CLASS, "Connect():");
    char * opt_com_path = NULL;
    if (opt_com_path == NULL) {
#ifdef _WIN32
        // use default com port
        opt_com_path = "\\\\.\\com3";
#else
        opt_com_path = "/dev/ttyUSB0";
#endif
    }
    u_result op_result = this->rpdriver->connect(opt_com_path, baudrate, flag);
    if (IS_FAIL(op_result)) {
        fprintf(stderr, "Connect(): error connecting - %d\n", op_result);
        return op_result;
    }

    debug(CLASS, "Connect(): connected successfully");
    return op_result;
}

void cxxProxyDriver::Disconnect() {
    debug(CLASS, "Disconnect()"); 
    this->rpdriver->disconnect();
    debug(CLASS, "Disconnect(): disconnected"); 
}

int cxxProxyDriver::IsConnected() {
    debug(CLASS, "IsConnected()"); 
    bool result = this->rpdriver->isConnected();
    if (result) {
        debug(CLASS, "IsConnected(): returning 1");
    } else {
        debug(CLASS, "IsConnected(): returning 0");
    }
    return result?1:0;
}

unsigned int cxxProxyDriver::Reset(unsigned int timeout) {
    debug(CLASS, "Reset(): timeout", timeout); 
    u_result op_result = this->rpdriver->reset(timeout);
    if (IS_FAIL(op_result)) {
        fprintf(stderr, "reset(): error resetting - %d", op_result);
        return op_result;
    }

    debug(CLASS, "Reset(): reset successfully");
    return op_result;
}

rplidar_device_info_unpacked* cxxProxyDriver::GetDeviceInfo(unsigned int timeout) {
    debug(CLASS, "GetDeviceInfo()"); 
    u_result op_result;
    rplidar_response_device_info_t devInfo;
	// retrieve the device info and set the field
    op_result = this->rpdriver->getDeviceInfo(devInfo, timeout);

    if (IS_FAIL(op_result)) {
        fprintf(stderr, "error getting device info - %d", op_result);
        return NULL;
    }

    rplidar_device_info_unpacked* result = 
            (rplidar_device_info_unpacked*) malloc(sizeof(rplidar_device_info_unpacked));
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

void cxxProxyDriver::FreeRplidarDeviceInfoUnpacked(rplidar_device_info_unpacked* data) {
    free(data->serialNumber);
    free(data);
}

uint cxxProxyDriver::GetHealth(unsigned int timeout) {
    debug(CLASS, "GetHealth()");
    u_result     op_result;
    rplidar_response_device_health_t healthinfo;

    op_result = this->rpdriver->getHealth(healthinfo, timeout);
    if (IS_OK(op_result)) { // the macro IS_OK is the preferred way to judge whether the operation is succeed.
        debug(CLASS, "GetHealth(): health status", healthinfo.status);
        if (healthinfo.status == RPLIDAR_STATUS_ERROR) {
            fprintf(stderr, "Error, rplidar internal error detected. Please reboot the device to retry.\n");
            // enable the following code if you want rplidar to be reboot by software
            // this->driver->reset();
            return 0;
        } else {
            return 1;
        }

    } else {
        fprintf(stderr, "%s GetHealth(): Error, cannot retrieve the lidar health - code: %x\n", CLASS, op_result);
        return 0;
    }
}

void cxxProxyDriver::StartMotor() {
    debug(CLASS, "StartMotor()");
    this->rpdriver->startMotor();
}

void cxxProxyDriver::Stop(unsigned int timeout) {
    debug(CLASS, "Stop(): timeout", timeout);
    this->rpdriver->stop(timeout);
}

void cxxProxyDriver::StopMotor() {
    debug(CLASS, "StopMotor()");
    this->rpdriver->stopMotor();
}

unsigned int cxxProxyDriver::StartScan(bool force, bool autoExpressMode) {
    debug(CLASS, "StartScan()");
    u_result result = this->rpdriver->startScan(force, autoExpressMode);
    debug(CLASS, "StartScan() returning");
    return result;
}

rplidar_scan_results_packed* cxxProxyDriver::GrabScanData(unsigned int scans, unsigned int timeout) {
    debug(CLASS, "GrabScanData()");
    size_t stscans = scans;
    size_t & refscans = stscans;
    rplidar_response_measurement_node_t* nodes =
            (rplidar_response_measurement_node_t*)
            malloc(scans * sizeof(rplidar_response_measurement_node_t));
    u_result op_result;
    op_result = this->rpdriver->grabScanData(nodes, refscans, timeout);

    if (!IS_OK(op_result)) {
        fprintf(stderr, "%s GrabScanData(): error while scanning data - %d\n", CLASS, op_result);
        return NULL;
    } else {
        debug(CLASS, "GrabScanData(): returning %d scans", stscans);
        rplidar_scan_results_packed* result = 
            (rplidar_scan_results_packed*) malloc(sizeof(rplidar_scan_results_packed));
        result->scans = scans;   // number of scans collected
        result->nodes = nodes;   // the scans
        return result;   
    }
}

void cxxProxyDriver::FreeRplidarScanResultsPacked(rplidar_scan_results_packed* data) {
    free(data->nodes);
    free(data);
}

_rplidar_response_measurement_node_t* cxxProxyDriver::Get_rplidar_response_measurement_node_t(rplidar_scan_results_packed* data, uint i) {
    if (i >= data->scans) {
        return NULL;
    }
    return data->nodes + i * sizeof(_rplidar_response_measurement_node_t);
}

rplidar_scan_results_packed* cxxProxyDriver::AscendScanData(rplidar_scan_results_packed* data) {
    debug(CLASS, "AscendScanData()");
    if (data == NULL) {
         fprintf(stderr, "%s AscendScanData(): data is null", CLASS);
         return NULL;
    }
    if (data->scans == 0) {
        fprintf(stderr, "%s AscendScanData(): no nodes in the data", CLASS);
    }
    debug(CLASS, "AscendScanData():", debugInt2String(data->scans), "nodes");
    u_result op_result;
    op_result = this->rpdriver->ascendScanData(data->nodes, data->scans);
    if (!IS_OK(op_result)) {
        fprintf(stderr, "%s AscendScanData(): error while sorting the data - %d", CLASS, op_result);
        return NULL;
    }
    return data;
}

// Copies a packed data structure into its unpacked equivalent
rplidar_scan_results_unpacked_p cxxProxyDriver::UnPack(rplidar_scan_results_packed* data) {
    debug(CLASS, "UnPack()");
    if (data == NULL) {
         fprintf(stderr, "%s UnPack(): data is null", CLASS);
         return NULL;
    }
    if (data->scans == 0) {
        fprintf(stderr, "%s UnPack(): no nodes in the data", CLASS);
    }
    debug(CLASS, "UnPack(): unpacking", debugInt2String(data->scans), "nodes");
    rplidar_scan_results_unpacked* result =
        (rplidar_scan_results_unpacked*) malloc(sizeof(rplidar_scan_results_unpacked));
    rplidar_response_measurement_node_unpacked_t* nodes = 
            (rplidar_response_measurement_node_unpacked_t*) 
            malloc(data->scans * sizeof(rplidar_response_measurement_node_unpacked_t));
    // Copy the data over, breaking out the fields.
    for (int i = 0; i < data->scans ; ++i) {
        nodes[i].sync = data->nodes[i].sync_quality & RPLIDAR_RESP_MEASUREMENT_SYNCBIT;
        nodes[i].quality = data->nodes[i].sync_quality >> RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT;
        nodes[i].checkbit = data->nodes[i].angle_q6_checkbit & RPLIDAR_RESP_MEASUREMENT_CHECKBIT;
        nodes[i].angle = (data->nodes[i].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT);
        nodes[i].distance = data->nodes[i].distance_q2;
    }
    result->scans = data->scans;
    result->nodes = nodes;
    return result;
}


