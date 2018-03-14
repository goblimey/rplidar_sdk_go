#ifndef _PROXY_DRIVER_HPP_
#define _PROXY_DRIVER_HPP_

class cxxProxyDriver {
public:
    cxxProxyDriver(char * com_path, _u32 com_baudrate);
    ~cxxProxyDriver();
    unsigned int Connect(const char* port_path, _u32 baudrate, _u32 flag);
    void Disconnect();
    int IsConnected();
    unsigned int Reset(_u32 timeout);
    rplidar_device_info_unpacked* GetDeviceInfo(_u32 timeout);
    void FreeRplidarDeviceInfoUnpacked(rplidar_device_info_unpacked* data);
    uint GetHealth(_u32 timeout);
    void StartMotor();
    void Stop(_u32 timeout);
    void StopMotor();
    unsigned int StartScan(bool force, bool autoExpressMode);
    rplidar_scan_results_packed* GrabScanData(unsigned int, _u32 timeout);
    _rplidar_response_measurement_node_t* Get_rplidar_response_measurement_node_t(rplidar_scan_results_packed*, uint);
    void FreeRplidarScanResultsPacked(rplidar_scan_results_packed*);
    rplidar_scan_results_packed_p AscendScanData(rplidar_scan_results_packed_p);
    rplidar_scan_results_unpacked_p UnPack(rplidar_scan_results_packed_p);
};

#endif