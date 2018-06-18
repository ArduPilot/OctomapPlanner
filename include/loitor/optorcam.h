#include <sys/time.h>
#include "optorimu.h"

#ifndef OPTORUSBCAM_H
#define OPTORUSBCAM_H
#define IMU_FRAME_LEN 32
#define IMG_WIDTH_VGA 	640
#define IMG_HEIGHT_VGA 	480
#define IMG_SIZE_VGA 	(IMG_WIDTH_VGA*IMG_HEIGHT_VGA)
#define IMG_BUF_SIZE_VGA (IMG_SIZE_VGA+0x200)
#define IMG_WIDTH_WVGA 	752
#define IMG_HEIGHT_WVGA 	480
#define IMG_SIZE_WVGA 	(IMG_WIDTH_WVGA*IMG_HEIGHT_WVGA)
#define IMG_BUF_SIZE_WVGA (IMG_SIZE_WVGA+0x200)

// setters & getters
void visensor_set_auto_EG(int E_AG);			// 0-ManEG | 1-AutoEG with limits | 2-AutoE&ManG | 3- fully auto
void visensor_set_exposure(int _man_exp);
void visensor_set_gain(int _man_gain);
void visensor_set_max_autoExp(int max_exp);
void visensor_set_min_autoExp(int min_exp);
void visensor_set_resolution(bool set_wvga);
void visensor_set_fps_mode(bool fps_mode);
void visensor_set_current_HB(int HB);
void visensor_set_desired_bin(int db);
void visensor_set_cam_selection_mode(int _visensor_cam_selection);
void visensor_set_imu_bias(float bx,float by,float bz);
void visensor_set_imu_portname(char* input_name);
void visensor_set_current_mode(int _mode);

int visensor_get_EG_mode();
int visensor_get_exposure();
int visensor_get_gain();
int visensor_get_max_autoExp();
int visensor_get_min_autoExp();
bool visensor_get_resolution();
int visensor_get_fps();
int visensor_get_current_HB();
int visensor_get_desired_bin();
int visensor_get_cam_selection_mode();
float visensor_get_imu_G_bias_x();
float visensor_get_imu_G_bias_y();
float visensor_get_imu_G_bias_z();
const char* visensor_get_imu_portname();

void visensor_save_current_settings();

float visensor_get_hardware_fps();

void visensor_load_settings(const char* settings_file);

int visensor_img_width();
int visensor_img_height();
int visensor_Start_Cameras();
void visensor_Close_Cameras();

bool visensor_is_leftcam_open();
bool visensor_is_rightcam_open();

/**
 * @brief visensor_is_left_img_new Check this function's return value before you wanna use visensor_get_left_latest_img() to get a new image.
 * @return true if new image is ready
 *          false if image is not ready
 */
bool visensor_is_left_img_new();

/**
 * @brief visensor_is_right_img_new Check this function's return value before you wanna use visensor_get_right_latest_img() to get a new image.
 * @return true if new image is ready
 *          false if image is not ready
 */
bool visensor_is_right_img_new();

/**
 * @brief visensor_get_left_latest_img This function is non-blocking and ALWAYS return the lastest image in buffer!
 *          Generally, you should check if new image is ready by the function: visensor_is_left_img_new()
 * @param img Your image pointer. For OpenCV, you can use cv::Mat im, im.data.
 * @param timestamp Timestamp in seconds, but with the precision of microseconds(us).
 * @param imudata (Optional)The pointer to your IMU data pack.
 * @return 0 if success, otherwise failed.
 */
int visensor_get_left_latest_img(unsigned char* img, double* timestamp, visensor_imudata* imudata=NULL);

/**
 * @brief visensor_get_right_latest_img This function is non-blocking and ALWAYS return the lastest image in buffer!
 *          Generally, you should check if new image is ready by the function: visensor_is_right_img_new()
 * @param img Your image pointer. For OpenCV, you can use cv::Mat im, im.data.
 * @param timestamp Timestamp in seconds, but with the precision of microseconds(us).
 * @param imudata (Optional)The pointer to your IMU data pack. Generally if you are using stereo camera
 *          and have already got the IMU data with left image, this parameter will be not necessary as it is the same with the left camera.
 * @return 0 if success, otherwise failed.
 */
int visensor_get_right_latest_img(unsigned char* img, double* timestamp, visensor_imudata* imudata=NULL);

/**
 * @brief visensor_imu_have_fresh_data Check this function's return value before you wanna use visensor_get_imudata_latest() to get new IMU data.
 * @return true if new IMU data is ready
 *          false if IMU data is not ready
 */
bool visensor_imu_have_fresh_data();

/**
 * @brief visensor_get_imudata_latest This function is non-blocking and ALWAYS return the lastest imudata in buffer!
 *          Generally, you should check if new IMU data is ready by the function: visensor_imu_have_fresh_data()
 * @param imudata
 * @return 0 if success
 *         -1 if data is not available
 */
int visensor_get_imudata_latest(visensor_imudata* imudata);

int visensor_Start_IMU();
void visensor_Close_IMU();


#endif
