#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <sys/time.h>

#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "optorusb.h"
#include "optorcam.h"
#include "optorimu.h"

using namespace std;
using namespace cv;
bool close_img_viewer=false;
bool visensor_Close_IMU_viewer=false;

// 当前左右图像的时间戳
timeval left_stamp,right_stamp;

/**
 * @brief opencv_showimg
 * @return
 */
void *opencv_showimg(void*)
{
    Mat img_left(Size(visensor_img_width(),visensor_img_height()),CV_8UC1);
    double left_timestamp;
    Mat img_right(Size(visensor_img_width(),visensor_img_height()),CV_8UC1);
    double right_timestamp;
    visensor_imudata img_imudata;
    while(!close_img_viewer)
    {
        if(visensor_is_leftcam_open())
        {
            if(visensor_is_left_img_new())
            {
                visensor_get_left_latest_img(img_left.data,&left_timestamp,&img_imudata);
                printf("L-Time: %8.6f, IMUTime: %8.6f\n",left_timestamp,img_imudata.timestamp);
                imshow("left",img_left);
            }
        }
       if(visensor_is_rightcam_open())
        {
            if(visensor_is_right_img_new())
            {
                visensor_get_right_latest_img(img_right.data,&right_timestamp);
                printf("R-Time: %8.6f\n",right_timestamp);
                imshow("right",img_right);
            }
        }
        waitKey(1);
    }
    pthread_exit(NULL);
}


void* show_imuData(void *)
{
    visensor_imudata imudata;
    while(!visensor_Close_IMU_viewer)
    {
        if(visensor_imu_have_fresh_data())
        {
            visensor_get_imudata_latest(&imudata);
            /*printf("IMUTime:%8.6f, Gyr: %8.4f,%8.4f,%8.4f, Acc: %8.4f,%8.4f,%8.4f, Quat(WXYZ): %8.4f,%8.4f,%8.4f,%8.4f\n",
                   imudata.timestamp,
                   imudata.rx,imudata.ry,imudata.rz,
                   imudata.ax,imudata.ay,imudata.az,
                   imudata.qw,imudata.qx,imudata.qy,imudata.qz);*/
        }
        usleep(100);
    }
    pthread_exit(NULL);
}

int main(int argc, char* argv[])
{

    /************************ Start Cameras ************************/
    visensor_load_settings("../optor_VISensor_Setups.txt");
    /*
    // 手动设置相机参数
    visensor_set_current_mode(5);
    visensor_set_auto_EG(0);
    visensor_set_exposure(50);
    visensor_set_gain(200);
    visensor_set_cam_selection_mode(2);
    visensor_set_resolution(false);
    visensor_set_fps_mode(true);
    // 保存相机参数到原配置文件
    visensor_save_current_settings();
    */

    int r = visensor_Start_Cameras();
    if(r<0)
    {
        printf("Opening cameras failed...\r\n");
        return r;
    }
    /************************** Start IMU **************************/
    int fd=visensor_Start_IMU();
    if(fd<0)
    {
        printf("visensor_open_port error...\r\n");
        return 0;
    }
    printf("visensor_open_port success...\r\n");
    /************************ ************ ************************/

    usleep(100000);

    //Create img_show thread
    pthread_t showimg_thread;
    int temp;
    if(temp = pthread_create(&showimg_thread, NULL, opencv_showimg, NULL))
        printf("Failed to create thread opencv_showimg\r\n");
    //Create show_imuData thread
    pthread_t showimu_thread;
    if(temp = pthread_create(&showimu_thread, NULL, show_imuData, NULL))
        printf("Failed to create thread show_imuData\r\n");

    while(1)
    {
        // Do - Nothing :)
        //cout<<visensor_get_imu_portname()<<endl;
        //cout<<visensor_get_hardware_fps()<<endl;
        sleep(1);
    }

    /* shut-down viewers */
    close_img_viewer=true;
    visensor_Close_IMU_viewer=true;
    if(showimg_thread !=0)
    {
        pthread_join(showimg_thread,NULL);
    }
    if(showimu_thread !=0)
    {
        pthread_join(showimu_thread,NULL);
    }

    /* close cameras */
    visensor_Close_Cameras();
    /* close IMU */
    visensor_Close_IMU();

    return 0;
}
