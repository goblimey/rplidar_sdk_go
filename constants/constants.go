package constants

const (
	RPLIDAR_ANS_TYPE_DEVINFO                            = 0x4
	RPLIDAR_ANS_TYPE_DEVHEALTH                          = 0x6
	RPLIDAR_ANS_TYPE_MEASUREMENT                        = 0x81
	RPLIDAR_ANS_TYPE_MEASUREMENT_CAPSULED               = 0x82
	RPLIDAR_ANS_TYPE_SAMPLE_RATE                        = 0x15
	RPLIDAR_ANS_TYPE_ACC_BOARD_FLAG                     = 0xFF
	RPLIDAR_RESP_ACC_BOARD_FLAG_MOTOR_CTRL_SUPPORT_MASK = (0x1)
	RPLIDAR_STATUS_OK                                   = 0x0
	RPLIDAR_STATUS_WARNING                              = 0x1
	RPLIDAR_STATUS_ERROR                                = 0x2
	RPLIDAR_RESP_MEASUREMENT_SYNCBIT                    = (0x1 << 0)
	RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT              = 2
	RPLIDAR_RESP_MEASUREMENT_CHECKBIT                   = (0x1 << 0)
	RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT                = 1
	RPLIDAR_RESP_MEASUREMENT_EXP_ANGLE_MASK             = (0x3)
	RPLIDAR_RESP_MEASUREMENT_EXP_DISTANCE_MASK          = (0xFC)
	RPLIDAR_RESP_MEASUREMENT_EXP_SYNC_1                 = 0xA
	RPLIDAR_RESP_MEASUREMENT_EXP_SYNC_2                 = 0x5
	RPLIDAR_RESP_MEASUREMENT_EXP_SYNCBIT                = (0x1 << 15)
)
