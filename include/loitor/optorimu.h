#ifndef OPTORIMU_H
#define OPTORIMU_H

typedef struct 
{
    unsigned char num;
    float rx,ry,rz;
    float ax,ay,az;
    float qw,qx,qy,qz;
    double timestamp;
}visensor_imudata;

bool visensor_query_imu_update();
bool visensor_mark_imu_update();
bool visensor_erase_imu_update();

/**
 * @brief visensor_open_port
 * @param dev_str example: "/dev/ttyUSB0"
 * @return fd>0:file descriptor
 *         fd<0:error
 */
int visensor_open_port(const char* dev_str);

/**
 * @brief visensor_set_opt
 * @param fd
 * @param nSpeed
 * @param nBits
 * @param nEvent
 * @param nStop
 * @return
 */
int visensor_set_opt(int fd,int nSpeed, int nBits, char nEvent, int nStop);

/**
 * @brief visensor_send_imu_frame
 * @param fd file descriptor
 * @param data buffer
 * @param len size of buffer
 * @return 0 if succeed
 */
int visensor_send_imu_frame(int fd, unsigned char* data, int len);

/**
 * @brief visensor_get_imu_frame
 * @param fd file descriptor
 * @param imu_frame imu_frame buffer, the size must be 32!
 * @param timestamp double type, the unit is second.
 * @return 0 if succeed
 *         -1 if timeout
 */
int visensor_get_imu_frame(int fd, unsigned char* imu_frame, double* timestamp);

/**
 * @brief visensor_parse_imu_frame
 * @param imu_frame  imu_frame buffer, the size must be 32!
 * @param timestamp double type, the unit is second.
 * @param acc_offset accelerator bias from IMU calibrator program.
 * @param imudata_struct
 * @return 0
 */
int visensor_parse_imu_frame(unsigned char* imu_frame,double timestamp, short int* acc_offset,visensor_imudata *imudata_struct);

#endif
