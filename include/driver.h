#ifndef _DRIVER_H_
#define _DRIVER_H_

#ifdef __cplusplus
extern "C" {
#endif

    // Data structures to allow the C++ RPLidar library to be called from Go.

#define RPLIDAR_SERIAL_NUMBER_LENGTH 16

typedef struct _rplidar_response_device_health_unpacked {
    _u8   status;
    _u16  error_code;
} rplidar_response_device_health_unpacked;

typedef struct _health_response {
    u_result op_result;
    rplidar_response_device_health_unpacked health_info;
} health_response;

    typedef struct _rplidar_device_info_unpacked {
        u_result op_result;
        unsigned int model;
        unsigned int majorFirmwareVersion;
        unsigned int minorFirmwareVersion;
        unsigned int hardwareVersion;
        char* serialNumber;
    } rplidar_device_info_unpacked;

    typedef rplidar_device_info_unpacked* rplidar_device_info_unpacked_p;

    typedef struct rplidar_response_measurement_node_unpacked_t {
        _u8   sync;
        _u8   quality;
        _u8   checkbit;
        _u16  angle;
        _u16  distance;
        
    } rplidar_response_measurement_node_unpacked_t;

    typedef struct rplidar_scan_results_packed {
        size_t scans;
        rplidar_response_measurement_node_t* nodes;
    } rplidar_scan_results_packed;

    typedef struct _rplidar_response_measurement_node_unpacked {
        _u8    sync_quality;      // syncbit:1;syncbit_inverse:1;quality:6;
        _u16   angle_q6_checkbit; // check_bit:1;angle_q6:15;
        _u16   distance_q2;
    } rplidar_response_measurement_node_unpacked;

    typedef struct rplidar_scan_results_unpacked {
        u_result op_result;
        unsigned int scans;
        rplidar_response_measurement_node_unpacked* nodes;
    } rplidar_scan_results_unpacked;

    typedef struct _rplidar_motor_ctrl_support_status {
        u_result op_result;
        int support;
    } rplidar_motor_ctrl_support_status;

    typedef struct _frequency_data {
        unsigned int op_result;
        float frequency;
        int is4kmode;
    } frequency_data;

    
    typedef struct _rplidar_response_sample_rate_unpacked {
        unsigned int op_result;
        _u16  std_sample_duration_us;
        _u16  express_sample_duration_us;
    } rplidar_response_sample_rate_unpacked;

    typedef struct _express_mode_support_status {
        unsigned int op_result;
        int supported;
    } express_mode_support_status;

	typedef void* ProxyDriver;
	ProxyDriver ProxyDriverInit(int);
	void ProxyDriverFree(ProxyDriver);
    unsigned int ProxyDriverConnect(ProxyDriver, const char* port_path, unsigned int baudrate, unsigned int flag);
    void ProxyDriverDisconnect(ProxyDriver);
    int ProxyDriverIsConnected(ProxyDriver);
    unsigned int ProxyDriverReset(ProxyDriver, unsigned int timeout);
    health_response ProxyDriverGetHealth(ProxyDriver, unsigned int);
    rplidar_device_info_unpacked* ProxyDriverGetDeviceInfo(ProxyDriver, unsigned int);
    void ProxyDriverFreeRplidarDeviceInfoUnpacked(ProxyDriver, rplidar_device_info_unpacked*) ;
    rplidar_response_sample_rate_unpacked ProxyDriverGetSampleDuration_uS(ProxyDriver, _u32 timeout);
    unsigned int ProxyDriverSetMotorPWM(ProxyDriver, _u16 pwm);   
    unsigned int ProxyDriverStartMotor(ProxyDriver);
    unsigned int ProxyDriverStopMotor(ProxyDriver);
    rplidar_motor_ctrl_support_status ProxyDriverCheckMotorCtrlSupport(ProxyDriver, _u32 timeout);
    frequency_data ProxyDriverGetFrequency(ProxyDriver d, int inExpressMode, unsigned int count);
    express_mode_support_status ProxyDriverCheckExpressScanSupported(ProxyDriver, unsigned int);
    unsigned int ProxyDriverStartScan(ProxyDriver, int, int);
    unsigned int ProxyDriverStartScanNormal(ProxyDriver, int force, _u32 timeout);
    unsigned int ProxyDriverStartScanExpress(ProxyDriver, int fixedAngle, _u32 timeout);
    unsigned int ProxyDriverStop(ProxyDriver, unsigned int);  
    rplidar_scan_results_unpacked* ProxyDriverGrabScanData(ProxyDriver, unsigned int, unsigned int);
    rplidar_scan_results_unpacked* ProxyDriverGrabAndSortScanData(ProxyDriver, unsigned int scans, unsigned int timeout);
    rplidar_response_measurement_node_unpacked* ProxyDriverGetRplidarResponseMeasurementNode(ProxyDriver, rplidar_scan_results_unpacked*, unsigned int i);
    void ProxyDriverFreeRplidarScanResultsUnpacked(ProxyDriver, rplidar_scan_results_unpacked*);
#ifdef __cplusplus
}
#endif

#endif