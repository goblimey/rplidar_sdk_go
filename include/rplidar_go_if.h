#ifndef _RPLIDAR_STRUCTS_H_
#define _RPLIDAR_STRUCTS_H_

// Data structures to allow the C++ RPLidar library to be called from Go.

#define RPLIDAR_SERIAL_NUMBER_LENGTH 16

typedef struct rplidar_device_info_unpacked {
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

typedef rplidar_scan_results_packed* rplidar_scan_results_packed_p;

typedef struct rplidar_scan_results_unpacked {
    unsigned int scans;
    rplidar_response_measurement_node_unpacked_t* nodes;
} rplidar_scan_results_unpacked;

typedef rplidar_scan_results_unpacked* rplidar_scan_results_unpacked_p;

#endif