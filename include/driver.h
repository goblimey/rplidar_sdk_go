#ifndef _DRIVER_H_
#define _DRIVER_H_

#ifdef __cplusplus
extern "C" {
#endif

    // typedef char* _str;

    typedef struct _rplidar_response_measurement_node_unpacked {
        _u8    sync_quality;      // syncbit:1;syncbit_inverse:1;quality:6;
        _u16   angle_q6_checkbit; // check_bit:1;angle_q6:15;
        _u16   distance_q2;
    } rplidar_response_measurement_node_unpacked;

	typedef void* ProxyDriver;
	ProxyDriver ProxyDriverInit(char * com_path, unsigned int com_baudrate);
	void ProxyDriverFree(ProxyDriver);
    unsigned int ProxyDriverConnect(ProxyDriver, const char* port_path, unsigned int baudrate, unsigned int flag);
    void ProxyDriverDisconnect(ProxyDriver);
    int ProxyDriverIsConnected(ProxyDriver);
    unsigned int ProxyDriverReset(ProxyDriver, unsigned int timeout);
    rplidar_device_info_unpacked* ProxyDriverGetDeviceInfo(ProxyDriver, unsigned int);
    void ProxyDriverFreeRplidarDeviceInfoUnpacked(ProxyDriver, rplidar_device_info_unpacked* data);
    unsigned int ProxyDriverGetHealth(ProxyDriver, unsigned int);
    void ProxyDriverStartMotor(ProxyDriver);
    void ProxyDriverStop(ProxyDriver, unsigned int);
    void ProxyDriverStopMotor(ProxyDriver);
    unsigned int ProxyDriverStartScan(ProxyDriver);    
    rplidar_scan_results_packed* ProxyDriverGrabScanData(ProxyDriver, unsigned int, unsigned int);
    rplidar_response_measurement_node_unpacked* ProxyDriverGet_rplidar_response_measurement_node_t(ProxyDriver, rplidar_scan_results_packed*, unsigned int);
    void ProxyDriverFreeRplidarScanResultsPacked(ProxyDriver, rplidar_scan_results_packed*);
    rplidar_scan_results_packed_p ProxyDriverAscendScanData(ProxyDriver, rplidar_scan_results_packed_p);
    rplidar_scan_results_unpacked_p ProxyDriverUnPack(ProxyDriver, rplidar_scan_results_packed_p);
#ifdef __cplusplus
}
#endif

#endif