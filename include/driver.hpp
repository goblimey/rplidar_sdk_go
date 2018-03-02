#ifndef _MY_PACKAGE_DRIVER_HPP_
#define _MY_PACKAGE_DRIVER_HPP_

#include "proxy_driver.hpp"
class RPlidarDriver;

class cxxRPDriver {
public:
    cxxRPDriver(char * com_path, _u32 com_baudrate);
    ~cxxRPDriver();
    int GetMajorVersionNumber();
    int GetMinorVersionNumber();
    int GetFirmwareId();
    int CheckRPLidarHealth();
    void StartMotor();
    void Stop();
    void StopMotor();
private:
    ProxyDriver * rpdriver;
    char * com_path;
    _u32 baud_rate;
};

#endif