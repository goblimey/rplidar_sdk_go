#ifndef _PROXY_DRIVER_HPP_
#define _PROXY_DRIVER_HPP_

class ProxyDriver {
public:
    ProxyDriver(char * com_path, _u32 com_baudrate);
    ~ProxyDriver();
    int GetMajorVersionNumber();
    int GetMinorVersionNumber();
    int GetFirmwareId();
    int CheckRPLidarHealth();
    void StartMotor();
    void Stop();
    void StopMotor();
private:
    rp::standalone::rplidar::RPlidarDriver * rpdriver;
    char * com_path;
    _u32 baud_rate;
};

#endif