#ifndef _DRIVER_H_
#define _DRIVER_H_

#ifdef __cplusplus
extern "C" {
#endif

    typedef char* _str;

	typedef void* RPDriver;
	RPDriver RPDriverInit(_str com_path, _u32 com_baudrate);
	void RPDriverFree(RPDriver);
	int RPDriverGetMajorVersionNumber(RPDriver);
    int RPDriverGetMinorVersionNumber(RPDriver);
    int RPDriverGetFirmwareId(RPDriver);
    int RPDriverCheckRPLidarHealth(RPDriver);
    void RPDriverStartMotor(RPDriver);
    void RPDriverStop(RPDriver);
    void RPDriverStopMotor(RPDriver);
#ifdef __cplusplus
}
#endif

#endif