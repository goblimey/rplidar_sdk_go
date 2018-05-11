#include <stdio.h>
#include <stdlib.h>
#include "rptypes.h"
#include "rplidar_cmd.h"
#include "rplidar_go_if.h"
#include "driver.h"
#include "proxy_driver.hpp"

#ifdef __cplusplus
extern "C" {
#endif

	typedef void* ProxyDriver;

	ProxyDriver ProxyDriverInit(int verbose) {
        cxxProxyDriver * ret = new cxxProxyDriver(verbose);
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


    health_response ProxyDriverGetHealth(ProxyDriver d, unsigned int timeout) {
        cxxProxyDriver * driver = (cxxProxyDriver*)d;
    	return driver->GetHealth(timeout);
    }

    rplidar_device_info_unpacked* ProxyDriverGetDeviceInfo(ProxyDriver d, unsigned int timeout) {
        cxxProxyDriver * driver = (cxxProxyDriver*)d;
        return driver->GetDeviceInfo(timeout);
    }

    void ProxyDriverFreeRplidarDeviceInfoUnpacked(ProxyDriver d, rplidar_device_info_unpacked* data) {
        free(data->serialNumber);
        free(data);
    }

    rplidar_response_sample_rate_unpacked ProxyDriverGetSampleDuration_uS(ProxyDriver d, _u32 timeout) {
        cxxProxyDriver * driver = (cxxProxyDriver*)d;
    	return driver->GetSampleDuration_uS(timeout);
    }

    unsigned int ProxyDriverSetMotorPWM(ProxyDriver d, _u16 pwm) {
        cxxProxyDriver * driver = (cxxProxyDriver*)d;
    	return driver->SetMotorPWM(pwm);
    }

    
    unsigned int ProxyDriverStartMotor(ProxyDriver d) {
        cxxProxyDriver * driver = (cxxProxyDriver*)d;
	    return driver->StartMotor();
    }

    unsigned int ProxyDriverStopMotor(ProxyDriver d) {
        cxxProxyDriver * driver = (cxxProxyDriver*)d;
	    return driver->StopMotor();
    }

     rplidar_motor_ctrl_support_status ProxyDriverCheckMotorCtrlSupport(ProxyDriver d, _u32 timeout) {
        cxxProxyDriver * driver = (cxxProxyDriver*)d;
	    return driver->CheckMotorCtrlSupport(timeout);
    }

    frequency_data ProxyDriverGetFrequency(ProxyDriver d, int inExpressMode, unsigned int count) {
        cxxProxyDriver * driver = (cxxProxyDriver*)d;
	    return driver->GetFrequency(inExpressMode, count);
    }

    express_mode_support_status ProxyDriverCheckExpressScanSupported(ProxyDriver d, unsigned int timeout) {
        cxxProxyDriver * driver = (cxxProxyDriver*)d;
	    return driver->CheckExpressScanSupported(timeout);

    }

    unsigned int ProxyDriverStartScan(ProxyDriver d, int force, int autoExpressMode) {
        cxxProxyDriver * driver = (cxxProxyDriver*)d;
	    return driver->StartScan(force, autoExpressMode);
    }

    unsigned int ProxyDriverStartScanNormal(ProxyDriver d, int force, _u32 timeout) {
        cxxProxyDriver * driver = (cxxProxyDriver*)d;
	    return driver->StartScanNormal(force != 0, timeout);
    }

    unsigned int ProxyDriverStartScanExpress(ProxyDriver d, int fixedAngle, _u32 timeout) {
        cxxProxyDriver * driver = (cxxProxyDriver*)d;
	    return driver->StartScanExpress(fixedAngle != 0, timeout);
    }

    unsigned int ProxyDriverStop(ProxyDriver d, unsigned int timeout) {
        cxxProxyDriver * driver = (cxxProxyDriver*)d;
	    return driver->Stop(timeout);
    }

    rplidar_scan_results_unpacked* ProxyDriverGrabScanData(ProxyDriver d, unsigned int scans, unsigned int timeout) {
        cxxProxyDriver * driver = (cxxProxyDriver*)d;
	    rplidar_scan_results_unpacked* result = driver->GrabScanData(scans, timeout);
        return result;
    }

    rplidar_scan_results_unpacked* ProxyDriverGrabAndSortScanData(ProxyDriver d, unsigned int scans, unsigned int timeout) {
        cxxProxyDriver * driver = (cxxProxyDriver*)d;
	    rplidar_scan_results_unpacked* result = driver->GrabAndSortScanData(scans, timeout);
        return result;
    }

    rplidar_response_measurement_node_unpacked* ProxyDriverGetRplidarResponseMeasurementNode(ProxyDriver d, rplidar_scan_results_unpacked* data, uint i) {
        if (i >= data->scans) {
            return NULL;
        }
        return (data->nodes) + i;
    }

    void ProxyDriverFreeRplidarScanResultsUnpacked(ProxyDriver d, rplidar_scan_results_unpacked* data) {
        free(data->nodes);
        free(data);
    }

#ifdef __cplusplus
}
#endif