#include <stdio.h>
#include <stdlib.h>
#include "rptypes.h"
#include "driver.hpp"
#include "driver.h"

#ifdef __cplusplus
extern "C" {
#endif

	typedef void* RPDriver;

	RPDriver RPDriverInit(char * com_path, _u32 com_baudrate) {
        cxxRPDriver * ret = new cxxRPDriver(com_path, com_baudrate);
	    return (void*)ret;
    }

	void RPDriverFree(RPDriver d) {
        cxxRPDriver * driver = (cxxRPDriver*)d;
	    delete driver;
    }
/*
    RPDriver DriverCreateDriver(RPDriver d, char * com_path, _u32 com_baudrate) {
        cxxRPDriver * driver = (cxxRPDriver*)d;
	    return (Driver)(driver->CreateDriver(com_path, com_baudrate));
    }
*/

    int RPDriverGetMajorVersionNumber(RPDriver d) {
        cxxRPDriver * driver = (cxxRPDriver*)d;
	    return driver->GetMajorVersionNumber();
    }

    int RPDriverGetMinorVersionNumber(RPDriver d) {
        cxxRPDriver * driver = (cxxRPDriver*)d;
	    return driver->GetMinorVersionNumber();
    }

    int DriverGetFirmwareId(RPDriver d) {
        cxxRPDriver * driver = (cxxRPDriver*)d;
	    return driver->GetFirmwareId();
    }

    int RPDriverCheckRPLidarHealth(RPDriver d) {
        cxxRPDriver * driver = (cxxRPDriver*)d;
    	return driver->CheckRPLidarHealth();
    }
    
    void RPDriverStartMotor(RPDriver d) {
        cxxRPDriver * driver = (cxxRPDriver*)d;
	    driver->StartMotor();
    }

    void RPDriverStop(RPDriver d) {
        cxxRPDriver * driver = (cxxRPDriver*)d;
	    driver->Stop();
    }

    void RPDriverStopMotor(RPDriver d) {
        cxxRPDriver * driver = (cxxRPDriver*)d;
	    driver->StopMotor();
    }

#ifdef __cplusplus
}
#endif