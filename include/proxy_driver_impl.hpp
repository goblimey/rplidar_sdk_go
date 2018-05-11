#ifndef _PROXY_DRIVER_HPP_
#define _PROXY_DRIVER_HPP_

class cxxProxyDriver {
public:
    cxxProxyDriver(int);
    ~cxxProxyDriver();
    unsigned int Connect(const char* port_path, _u32 baudrate, _u32 flag);
    void Disconnect();
    int IsConnected();
    unsigned int Reset(_u32 timeout);
    health_response GetHealth(unsigned int timeout);
    rplidar_device_info_unpacked* GetDeviceInfo(unsigned int timeout);
    rplidar_response_sample_rate_unpacked GetSampleDuration_uS(_u32 timeout);
    unsigned int SetMotorPWM(_u16 pwm);
    unsigned int StartMotor();
    unsigned int StopMotor();
    rplidar_motor_ctrl_support_status CheckMotorCtrlSupport(_u32);
    frequency_data GetFrequency(int inExpressMode, size_t count);
    express_mode_support_status CheckExpressScanSupported(unsigned int);
    unsigned int StartScan(bool force, bool autoExpressMode);
    unsigned int StartScanNormal(bool force, _u32 timeout);
    unsigned int StartScanExpress(bool fixedAngle, _u32 timeout);
    unsigned int Stop(_u32 timeout);
    rplidar_scan_results_unpacked* GrabScanData(unsigned int, _u32 timeout);
    rplidar_scan_results_unpacked* GrabAndSortScanData(unsigned int scans, unsigned int timeout);
private:
    rplidar_response_measurement_node_unpacked* unpackScanResult(unsigned int scans, rplidar_response_measurement_node_t* nodes);
    rp::standalone::rplidar::RPlidarDriver * rpdriver;
    char * com_path;
    _u32 baud_rate;
    bool verbose;
};

#endif