#include <stdio.h>
#include <stdlib.h>
#include "rptypes.h"
#include "rplidar_cmd.h"
#include "rplidar_go_if.h"
#include "proxy_driver.hpp"
#include "driver.h"

#ifdef __cplusplus
extern "C" {
#endif

	typedef void* ProxyDriver;

	ProxyDriver ProxyDriverInit(char * com_path, unsigned int com_baudrate) {
        cxxProxyDriver * ret = new cxxProxyDriver(com_path, com_baudrate);
	    return (void*)ret;
    }

	void ProxyDriverFree(ProxyDriver d) {
        cxxProxyDriver * driver = (cxxProxyDriver*)d;
	    delete driver;
    }

    unsigned int ProxyDriverConnect(ProxyDriver d, const char* port_path, unsigned int baudrate, unsigned int flag) {
        cxxProxyDriver * driver = (cxxProxyDriver*)d;
	    return driver->Connect(NULL, baudrate, flag);
    }

    void ProxyDriverDisconnect(ProxyDriver d) {
        cxxProxyDriver * driver = (cxxProxyDriver*)d;
	    return driver->Disconnect();
    }
    
    int ProxyDriverIsConnected(ProxyDriver d) {
        cxxProxyDriver * driver = (cxxProxyDriver*)d;
	    return driver->IsConnected();
    }
    
    unsigned int ProxyDriverReset(ProxyDriver d, unsigned int timeout) {
        cxxProxyDriver * driver = (cxxProxyDriver*)d;
	    return driver->Reset(timeout);
    }

    rplidar_device_info_unpacked* ProxyDriverGetDeviceInfo(ProxyDriver d, unsigned int timeout) {
        cxxProxyDriver * driver = (cxxProxyDriver*)d;
        return driver->GetDeviceInfo(timeout);
    }

    void ProxyDriverFreeRplidarDeviceInfoUnpacked(ProxyDriver d, rplidar_device_info_unpacked* data) {
        cxxProxyDriver * driver = (cxxProxyDriver*)d;
	    driver->FreeRplidarDeviceInfoUnpacked(data);
    }

    unsigned int ProxyDriverGetHealth(ProxyDriver d, unsigned int timeout) {
        cxxProxyDriver * driver = (cxxProxyDriver*)d;
    	return driver->GetHealth(timeout);
    }
    
    void ProxyDriverStartMotor(ProxyDriver d) {
        cxxProxyDriver * driver = (cxxProxyDriver*)d;
	    driver->StartMotor();
    }

    void ProxyDriverStop(ProxyDriver d, unsigned int timeout) {
        cxxProxyDriver * driver = (cxxProxyDriver*)d;
	    driver->Stop(timeout);
    }

    void ProxyDriverStopMotor(ProxyDriver d) {
        cxxProxyDriver * driver = (cxxProxyDriver*)d;
	    driver->StopMotor();
    }

    unsigned int ProxyDriverStartScan(ProxyDriver d) {
        cxxProxyDriver * driver = (cxxProxyDriver*)d;
	    return driver->StartScan(false, true);  
    }
    

    rplidar_scan_results_packed_p ProxyDriverGrabScanData(ProxyDriver d, unsigned int scans, unsigned int timeout) {
        cxxProxyDriver * driver = (cxxProxyDriver*)d;
	    rplidar_scan_results_packed* result = driver->GrabScanData(scans, timeout);
        return result;
    }

    rplidar_response_measurement_node_unpacked* 
            ProxyDriverGet_rplidar_response_measurement_node_t(ProxyDriver d, rplidar_scan_results_packed* data, unsigned int i) {
        cxxProxyDriver * driver = (cxxProxyDriver*)d;
	    rplidar_response_measurement_node_t* packed = 
                driver->Get_rplidar_response_measurement_node_t(data, i);
        rplidar_response_measurement_node_unpacked* result = 
                (rplidar_response_measurement_node_unpacked*) malloc(sizeof(rplidar_response_measurement_node_unpacked));
        result->sync_quality = packed->sync_quality;
        result->angle_q6_checkbit = packed->angle_q6_checkbit;
        result->distance_q2 = packed->distance_q2;
        return result;
    }

    void ProxyDriverFreeRplidarScanResultsPacked(ProxyDriver d, rplidar_scan_results_packed* data) {
        cxxProxyDriver * driver = (cxxProxyDriver*)d;
        driver->FreeRplidarScanResultsPacked(data);
    }

    rplidar_scan_results_packed_p ProxyDriverAscendScanData(ProxyDriver d, rplidar_scan_results_packed_p r) {
        cxxProxyDriver * driver = (cxxProxyDriver*)d;
	    rplidar_scan_results_packed* result = driver->AscendScanData(r);
        return result;
    }

    rplidar_scan_results_unpacked_p ProxyDriverUnPack(ProxyDriver d, rplidar_scan_results_packed_p r) {
        cxxProxyDriver * driver = (cxxProxyDriver*)d;
	    rplidar_scan_results_unpacked_p result = driver->UnPack(r);
        return result;
    }

#ifdef __cplusplus
}
#endif