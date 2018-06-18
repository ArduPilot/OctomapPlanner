#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <getopt.h>
#include <fcntl.h>
#include <string.h>
#include <math.h>
#include <pthread.h>
#include <sys/time.h>
#include <linux/types.h>

#include <iostream>
#include <algorithm>
#include <fstream>
#include <string>
#include <vector>
#include <iomanip>
#include <sstream>

#include <libusb-1.0/libusb.h>

#include "loitor/optorimu.h"
#include "loitor/optorusb.h"


#define LOG			printf
#define	DEBUGLOG	printf
#define	CODELOG		printf("File:%s\r\nLine:%d \r\n",__FILE__,__LINE__);

/*PARAMETER MACRO*/
#define IMG_WIDTH_VGA 	640
#define IMG_HEIGHT_VGA 	480
#define IMG_SIZE_VGA 	(IMG_WIDTH_VGA*IMG_HEIGHT_VGA)
#define IMG_BUF_SIZE_VGA (IMG_SIZE_VGA+0x200)
#define IMG_WIDTH_WVGA 	752
#define IMG_HEIGHT_WVGA 	480
#define IMG_SIZE_WVGA 	(IMG_WIDTH_WVGA*IMG_HEIGHT_WVGA)
#define IMG_BUF_SIZE_WVGA (IMG_SIZE_WVGA+0x200)
#define FRAME_CLUST 54

/*CAMERA CONTROL INSTRUCTION MACRO*/
#define CAPTURE_27			0xA1
#define CAPTURE_54			0xA2
#define CAPTURE_108			0xA6
#define STANDBY_SHORT		0xA3
#define STANDBY_LONG		0xA4
#define GET_CAM_LR			0xA5
#define CAM_I2C_R			0xA7
#define CAM_I2C_W			0xA8
#define auto_expo			1
#define EXP_VAL				50
#define GAI_VAL				100

#define CAM_I2C_ADDR		0x5C //0x5C

#define IMU_FRAME_LEN 32
pthread_t imu_thread;
int imu_fd=0;
bool imu_close=false;

using namespace std;

#define IMU_FIFO_SIZE 20
visensor_imudata IMU_FIFO[IMU_FIFO_SIZE];
int imu_fifo_idx=0;

bool shut_down_tag=false;

bool left_fresh=false;
bool right_fresh=false;

bool allow_settings_change=true;

/****************************************************/
int data_mode;
int modes_settings[13][4];
int EG_mode;
int man_exp;
int man_gain;
int auto_EG_top;
int auto_EG_bottom;
int auto_EG_des;
int auto_E_man_G_Etop;
int auto_E_man_G_Ebottom;
int auto_E_man_G;
int agc_aec_skip_frame;
int VI_FIFO_matcher;

double imu_acc_bias_X;
double imu_acc_bias_Y;
double imu_acc_bias_Z;

string imu_port_name;
/****************************************************/

cyusb_handle *pcam1_handle;
cyusb_handle *pcam2_handle;
int gFPS = 27;
int gSelectCam = 0;
int HORIZ_BLK=150;

pthread_t cam1_capture_thread;
pthread_t cam2_capture_thread;

int gFrameCam1=0;
int gFrameCam2=0;
struct img_s
{
    unsigned int count;
    unsigned char data[IMG_BUF_SIZE_WVGA];
    double timestamp;
    unsigned char pass;
};

struct img_s im1[FRAME_CLUST],im2[FRAME_CLUST];

timeval visensor_startTime;

bool last_is_good=false;
bool save_img=false;
bool save_img1=false;

bool visensor_resolution_status=false;
int current_HB=194;
int visensor_cam_selection=0;
int current_FPS=27;

string visensor_settings_file_name;

float visensor_get_hardware_fps();

int visensor_get_imudata_from_timestamp(visensor_imudata* imudata, double timestamp);

int visensor_img_width()
{
    return (visensor_resolution_status==1?IMG_WIDTH_WVGA:IMG_WIDTH_VGA);
}
int visensor_img_height()
{
    return (visensor_resolution_status==1?IMG_HEIGHT_WVGA:IMG_HEIGHT_VGA);
}

bool visensor_is_leftcam_open()
{
    if(pcam1_handle==NULL)
        return false;
    return true;
}
bool visensor_is_rightcam_open()
{
    if(pcam2_handle==NULL)
        return false;
    return true;
}
void visensor_load_settings(const char* settings_file)
{
    /******************************* Load Settings File ***************************************/

    LOG("\r\n**********************************************\r\n");
    LOG("Loading Settings File...\r\n");

    visensor_settings_file_name=settings_file;

    ifstream ifs_settings(visensor_settings_file_name.c_str());//,ios::in|ios::binary|ios::ate);
    if(!ifs_settings )
    {
        cerr << "Error opening file: "<<visensor_settings_file_name <<endl;
        exit(EXIT_FAILURE);
    }
    else
    {
        cout<<"Successfully opened settings file!"<<endl;
    }

    long file_size=ifs_settings.tellg();

    //cout<<"filesize:"<<file_size<<endl;

    std::string filename;
    std::string line_settings;
    std::string timestamp;
    std::vector<string> settingsList;
    int linecount=0;
    while(std::getline(ifs_settings,line_settings))
    {
        linecount++;
        if(line_settings.at(0)!='#')
            settingsList.push_back(line_settings);
    }
    //cout <<"line get end............"<<endl;
    for (vector<string>::iterator iter = settingsList.begin(); iter != settingsList.end(); ++iter)
    {
        cout << *iter << endl;
    }
    ifs_settings.close();


    std::string token;

    // convert to File_Settings Object
    istringstream(settingsList.at(1))>>data_mode;
    //cout << "data_mode: "<<data_mode<<endl;

    for(int i=3; i<=15; i++)
    {
        if(i==9)continue;
        int visensor_cam_selection=0,_HB=0,res=0,fps=0;

        istringstream iss_line(settingsList.at(i));
        //The first word, ignore
        std::getline(iss_line, token, ',');

        //The second word, lrs
        std::getline(iss_line, token, ',');
        if(token[0]=='l')visensor_cam_selection=2;
        else if(token[0]=='r')visensor_cam_selection=1;
        else if(token[0]=='s')visensor_cam_selection=0;

        //_HB
        std::getline(iss_line, token, ',');
        istringstream(token)>>_HB;

        //WVGA or VGA?
        std::getline(iss_line, token, ',');
        res=(token[0]=='V'?0:1);

        //FPS
        std::getline(iss_line, token, ',');
        istringstream(token)>>fps;

        // fill data
        if(i<9)// “HighSpeed Pre-Set”
        {
            modes_settings[i-3][0]=visensor_cam_selection;
            modes_settings[i-3][1]=_HB;
            modes_settings[i-3][2]=res;
            modes_settings[i-3][3]=fps;
        }
        else// “Normal Pre-Set”
        {
            modes_settings[i-4][0]=visensor_cam_selection;
            modes_settings[i-4][1]=_HB;
            modes_settings[i-4][2]=res;
            modes_settings[i-4][3]=fps;
        }

    }

    // Manual Mode
    {
        int visensor_cam_selection=0,_HB=0,res=0,fps=0;

        istringstream(settingsList.at(17))>>visensor_cam_selection;
        istringstream(settingsList.at(18))>>_HB;
        token = settingsList.at(19);
        res=(token[0]=='V'?0:1);
        istringstream(settingsList.at(20))>>fps;

        modes_settings[12][0]=visensor_cam_selection;
        modes_settings[12][1]=_HB;
        modes_settings[12][2]=res;
        modes_settings[12][3]=fps;
    }

    // EXP&GAIN
    istringstream(settingsList.at(22))>>EG_mode;

    // manual E&G
    {
        int _exp=0,_gain=0;

        istringstream iss_line(settingsList.at(23));
        //The first word, ignore
        std::getline(iss_line, token, ',');

        std::getline(iss_line, token, ',');
        istringstream(token)>>_exp;
        std::getline(iss_line, token, ',');
        istringstream(token)>>_gain;

        // fill data
        man_exp=_exp;
        man_gain=_gain;
    }

    // auto
    {
        int _top=0,_bottom=0,_des=0;

        istringstream iss_line(settingsList.at(24));
        //The first word, ignore
        std::getline(iss_line, token, ',');

        std::getline(iss_line, token, ',');
        istringstream(token)>>_top;

        std::getline(iss_line, token, ',');
        istringstream(token)>>_bottom;

        std::getline(iss_line, token, ',');
        istringstream(token)>>_des;
        // fill data
        auto_EG_top=_top;
        auto_EG_bottom=_bottom;
        auto_EG_des=_des;
    }

    // AE_MG
    {
        int _top=0,_bottom=0,_des=0,_gain=0;

        istringstream iss_line(settingsList.at(25));

        //The first word, ignore
        std::getline(iss_line, token, ',');

        std::getline(iss_line, token, ',');
        istringstream(token)>>_top;
        std::getline(iss_line, token, ',');
        istringstream(token)>>_bottom;
        std::getline(iss_line, token, ',');
        istringstream(token)>>_des;
        std::getline(iss_line, token, ',');
        istringstream(token)>>_gain;
        // fill data
        auto_E_man_G_Etop=_top;
        auto_E_man_G_Ebottom=_bottom;
        auto_E_man_G=_gain;
    }
    // imu_port_name
    {
        istringstream iss_line(settingsList.at(26));
        std::getline(iss_line, token, ',');

        imu_port_name = token.c_str();

        std::getline(iss_line, token, ',');
        istringstream(token)>>VI_FIFO_matcher;
    }
    // imu acc bias
    {
        istringstream iss_line(settingsList.at(28));
        //The first word, ignore
        std::getline(iss_line, token, ',');
        std::getline(iss_line, token, ',');
        istringstream(token)>>imu_acc_bias_X;
    }
    {
        istringstream iss_line(settingsList.at(29));
        //The first word, ignore
        std::getline(iss_line, token, ',');
        std::getline(iss_line, token, ',');
        istringstream(token)>>imu_acc_bias_Y;
    }
    {
        istringstream iss_line(settingsList.at(30));
        //The first word, ignore
        std::getline(iss_line, token, ',');
        std::getline(iss_line, token, ',');
        istringstream(token)>>imu_acc_bias_Z;
    }

    // show settings
//    cout<<endl<<"data_mode : "<<data_mode<<endl;
//    for(int i=0; i<13; i++)
//    {
//        cout<<"mode : "<<(i+1)<<endl;
//        cout<<"fps : "<<modes_settings[i][3]<<endl;
//        cout<<"HB : "<<modes_settings[i][1]<<endl;
//        cout<<"visensor_cam_selection : "<<modes_settings[i][0]<<endl;
//        cout<<"res : "<<modes_settings[i][2]<<endl<<endl;
//    }
//    cout<<"EG_mode : "<<EG_mode<<endl;
//    cout<<"man_exp : "<<man_exp<<endl;
//    cout<<"man_gain : "<<man_gain<<endl;
//    cout<<"auto_EG_top : "<<auto_EG_top<<endl;
//    cout<<"auto_EG_bottom : "<<auto_EG_bottom<<endl;
//    cout<<"auto_EG_des : "<<auto_EG_des<<endl;
//    cout<<"auto_E_man_G_Etop : "<<auto_E_man_G_Etop<<endl;
//    cout<<"auto_E_man_G_Ebottom : "<<auto_E_man_G_Ebottom<<endl;
//    cout<<"auto_E_man_G : "<<auto_E_man_G<<endl;
//    cout<<"VI_FIFO_matcher : "<<VI_FIFO_matcher<<endl;
//    cout<<"imu_acc_bias_X :  "<<imu_acc_bias_X<<endl;
//    cout<<"imu_acc_bias_Y :  "<<imu_acc_bias_Y<<endl;
//    cout<<"imu_acc_bias_Z :  "<<imu_acc_bias_Z<<endl;
//    cout<<"imu_port_name :  "<<imu_port_name<<endl<<endl;

    // 预设模式+手动模式
    current_FPS=modes_settings[data_mode-1][3];
    gFPS = current_FPS;
    visensor_resolution_status=modes_settings[data_mode-1][2];
    current_HB=modes_settings[data_mode-1][1];
    visensor_cam_selection=modes_settings[data_mode-1][0];

    /****************************************************************************/


    if(visensor_cam_selection==0)gSelectCam = 3;
    else if(visensor_cam_selection==2)gSelectCam = 1;
    else if(visensor_cam_selection==1)gSelectCam = 2;

    //Add: AGC/AEC Skip frames
    agc_aec_skip_frame = (int)(visensor_get_hardware_fps()/10);
}

// setters
void visensor_set_auto_EG(int E_AG)
{
    if(!allow_settings_change)
    {
        cout<<"settings FIXED !"<<endl;
        return;
    }
    EG_mode=E_AG;
}
void visensor_set_exposure(int _man_exp)
{
    if(!allow_settings_change)
    {
        cout<<"settings FIXED !"<<endl;
        return;
    }
    man_exp=_man_exp;
}
void visensor_set_gain(int _man_gain)
{
    if(!allow_settings_change)
    {
        cout<<"settings FIXED !"<<endl;
        return;
    }
    man_gain=_man_gain;
    auto_E_man_G=_man_gain;
}
void visensor_set_max_autoExp(int max_exp)
{
    if(!allow_settings_change)
    {
        cout<<"settings FIXED !"<<endl;
        return;
    }
    auto_EG_top=max_exp;
}
void visensor_set_min_autoExp(int min_exp)
{
    if(!allow_settings_change)
    {
        cout<<"settings FIXED !"<<endl;
        return;
    }
    auto_EG_bottom=min_exp;
}
void visensor_set_resolution(bool set_wvga)
{
    if(!allow_settings_change)
    {
        cout<<"settings FIXED !"<<endl;
        return;
    }
    visensor_resolution_status=set_wvga;
}
void visensor_set_fps_mode(bool fps_mode)
{
    if(!allow_settings_change)
    {
        cout<<"settings FIXED !"<<endl;
        return;
    }
    if(fps_mode==true)
    {
        gFPS=54;
        current_FPS=54;
    }
    else
    {
        gFPS=27;
        current_FPS=27;
    }
}
void visensor_set_current_HB(int HB)
{
    if(!allow_settings_change)
    {
        cout<<"settings FIXED !"<<endl;
        return;
    }
    current_HB=HB;
}
void visensor_set_desired_bin(int db)
{
    if(!allow_settings_change)
    {
        cout<<"settings FIXED !"<<endl;
        return;
    }
    auto_EG_des=db;
}
void visensor_set_cam_selection_mode(int _visensor_cam_selection)
{
    if(!allow_settings_change)
    {
        cout<<"settings FIXED !"<<endl;
        return;
    }
    visensor_cam_selection=_visensor_cam_selection;
}
void visensor_set_imu_bias(float bx,float by,float bz)
{
    if(!allow_settings_change)
    {
        cout<<"settings FIXED !"<<endl;
        return;
    }
    imu_acc_bias_X=bx;
    imu_acc_bias_Y=by;
    imu_acc_bias_Z=bz;
}
void visensor_set_imu_portname(char* input_name)
{
    if(!allow_settings_change)
    {
        cout<<"settings FIXED !"<<endl;
        return;
    }
    imu_port_name=input_name;
}
void visensor_set_current_mode(int _mode)
{
    if(!allow_settings_change||_mode>13||_mode<=0)
    {
        cout<<"settings FIXED !"<<endl;
        return;
    }
    data_mode=_mode;
    // 预设模式+手动模式
    current_FPS=modes_settings[data_mode-1][3];
    gFPS=current_FPS;
    visensor_resolution_status=modes_settings[data_mode-1][2];
    current_HB=modes_settings[data_mode-1][1];
    visensor_cam_selection=modes_settings[data_mode-1][0];
}
// getters
int visensor_get_EG_mode()
{
    return EG_mode;
}
int visensor_get_exposure()
{
    return man_exp;
}
int visensor_get_gain()
{
    return man_gain;
}
int visensor_get_max_autoExp()
{
    return auto_EG_top;
}
int visensor_get_min_autoExp()
{
    return auto_EG_bottom;
}
bool visensor_get_resolution()
{
    return visensor_resolution_status;
}
int visensor_get_fps()
{
    return current_FPS;
}
int visensor_get_current_HB()
{
    return current_HB;
}
int visensor_get_desired_bin()
{
    return auto_EG_des;
}
int visensor_get_cam_selection_mode()
{
    return visensor_cam_selection;
}
float visensor_get_imu_G_bias_x()
{
    return imu_acc_bias_X;
}
float visensor_get_imu_G_bias_y()
{
    return imu_acc_bias_Y;
}
float get_imu_bias_z()
{
    return imu_acc_bias_Z;
}
const char* visensor_get_imu_portname()
{
    return imu_port_name.c_str();
}

void visensor_save_current_settings()
{
    // 保存设置参数到原配置文件
    ofstream f;
    f.open(visensor_settings_file_name.c_str());
    f.close();
    f.open(visensor_settings_file_name.c_str());

    f << fixed;
    // start writing settings file
    f<<'#'<<endl<<"Mode"<<endl;
    f<<data_mode<<endl<<'#'<<endl<<"HighSpeed Pre-Set"<<endl;
    f<<"m1,left,"<<modes_settings[0][1]<<",VGA,108"<<endl;
    f<<"m2,right,"<<modes_settings[1][1]<<",VGA,108"<<endl;
    f<<"m3,left,"<<modes_settings[2][1]<<",WVGA,108"<<endl;
    f<<"m4,right,"<<modes_settings[3][1]<<",WVGA,108"<<endl;
    f<<"m5,stereo,"<<modes_settings[4][1]<<",VGA,54"<<endl;
    f<<"m6,stereo,"<<modes_settings[5][1]<<",WVGA,54"<<endl;

    f<<"#"<<endl<<"Normal Pre-Set"<<endl;

    f<<"m7,left,"<<modes_settings[6][1]<<",VGA,54"<<endl;
    f<<"m8,right,"<<modes_settings[7][1]<<",VGA,54"<<endl;
    f<<"m9,left,"<<modes_settings[8][1]<<",WVGA,54"<<endl;
    f<<"m10,right,"<<modes_settings[9][1]<<",WVGA,54"<<endl;
    f<<"m11,stereo,"<<modes_settings[10][1]<<",VGA,27"<<endl;
    f<<"m12,stereo,"<<modes_settings[11][1]<<",WVGA,27"<<endl;

    f<<"#"<<endl<<"m13,Manual Mode"<<endl<<modes_settings[12][0]
     <<endl<<modes_settings[12][1]<<endl;
    if(modes_settings[12][2]==0)f<<"VGA"<<endl;
    else f<<"WVGA"<<endl;
    f<<modes_settings[12][3]<<endl;

    f<<"#"<<endl<<"EG_mode"<<endl<<EG_mode<<endl;
    f<<"manual,"<<man_exp<<","<<man_gain<<endl;
    f<<"auto,"<<auto_EG_top<<","<<auto_EG_bottom<<","<<auto_EG_des<<endl;
    f<<"autoexp_manualgain,"<<auto_E_man_G_Etop<<","<<auto_E_man_G_Ebottom<<","<<auto_EG_des<<","<<auto_E_man_G<<endl;


    f<<imu_port_name<<","<<VI_FIFO_matcher<<endl<<"#"<<endl<<"IMU-acc-bias"<<endl;
    f<<"Gx,"<<imu_acc_bias_X<<endl;
    f<<"Gy,"<<imu_acc_bias_Y<<endl;

    f<<"Gz,"<<imu_acc_bias_Z<<endl<<"#"<<endl;

    f.close();
}



/*****************************************************************************/

int fps_control()
{
    switch(gFPS)
    {
    case 27:
        return CAPTURE_27;
        break;
    case 54:
        return CAPTURE_54;
        break;
    case 108:
        return CAPTURE_108;
        break;
    default:
        return CAPTURE_27;
        break;
    }
    return CAPTURE_27;
}

cyusb_handle* get_cam_no_handle(int cam_no)
{
    if(cam_no==1)
        return pcam1_handle;
    else if(cam_no==2)
        return pcam2_handle;
    else
    {
        printf("Wrong cam number!\r\n");
        return NULL;
    }
}

int control_camera(int cam_no, unsigned char instruction)
{
    unsigned char buf[1];
    int r = cyusb_control_transfer(get_cam_no_handle(cam_no),0xC0,instruction,0,0,buf,0,1000);
    if(r != 0)
    {
        //printf("cam%d,i:0x%02X,r:%d\r\n",cam_no,instruction,r);
    }
    return r;
}

int camera_i2c_read(int cam_no, unsigned char reg,int* value)
{
    unsigned char buf[3];
    int r = cyusb_control_transfer(get_cam_no_handle(cam_no),0xC0,CAM_I2C_R,(CAM_I2C_ADDR<<8)+reg,0,buf,3,1000);
    if(r != 3)
    {
        printf("cam%d,i:0x%02X,r:%d\r\n",cam_no,CAM_I2C_R,r);
        return r;
    }
    if(buf[0]!=0)
    {
        printf("I2C Read failed...Cam:%d, Addr:0x%02X, Reg:0x%02X, I2C_return:%d\r\n",cam_no,CAM_I2C_ADDR,reg,buf[0]);
        return r;
    }
    *value = buf[1]+buf[2]<<8;
    //printf("I2C Read Success...Cam:%d, Addr:0x%02X, Reg:0x%02X, Result:0x%04X\r\n",cam_no,CAM_I2C_ADDR,reg,*value);
    return r;
}

int camera_i2c_write(int cam_no, unsigned char reg,int value)
{
    unsigned char buf[1];
    int r = cyusb_control_transfer(get_cam_no_handle(cam_no),0xC0,CAM_I2C_W,(CAM_I2C_ADDR<<8)+reg,value,buf,1,1000);
    if(r != 1)
    {
        printf("cam%d,i:0x%02X,r:%d\r\n",cam_no,CAM_I2C_W,r);
        return r;
    }
    if(buf[0]!=0)
    {
        printf("I2C Write failed...Cam:%d, Addr:0x%02X, Reg:0x%02X, Write:0x%04X, I2C_return:%d\r\n",cam_no,CAM_I2C_ADDR,reg,value,buf[0]);
        return r;
    }
    //printf("I2C Write Success...Cam:%d, Addr:0x%02X, Reg:0x%02X, Write:0x%04X\r\n",cam_no,CAM_I2C_ADDR,reg,value);
    return r;
}

int check_img(int cam_no,unsigned char *pImg,unsigned char *pImgPass)
{
//	printf("Cam image check enter:%d\n",visensor_resolution_status);
    if(!visensor_resolution_status)*pImgPass = (pImg[IMG_SIZE_VGA+0]==0xFF&&pImg[IMG_SIZE_VGA+1]==0x00&&pImg[IMG_SIZE_VGA+2]==0xFE&&pImg[IMG_SIZE_VGA+3]==0x01);
    else *pImgPass = (pImg[IMG_SIZE_WVGA+0]==0xFF&&pImg[IMG_SIZE_WVGA+1]==0x00&&pImg[IMG_SIZE_WVGA+2]==0xFE&&pImg[IMG_SIZE_WVGA+3]==0x01);
    if(*pImgPass == 0)
    {
        control_camera(cam_no,fps_control());
        //printf("Cam%d image check error!\n",cam_no);
    }
}

float visensor_get_hardware_fps()
{
    if(gFPS==54)
    {
        if(!visensor_resolution_status)return 45700.0f/(current_HB+640.0f);
        else return 45700.0f/(current_HB+752.0f);
    }
    else if(gFPS==27)
    {
        if(!visensor_resolution_status)return 0.5f*45700.0f/(current_HB+640.0f);
        else return 0.5f*45700.0f/(current_HB+752.0f);
    }
}
static void set_camreg_default(int cam_no)
{
    camera_i2c_write(cam_no,0x00,0x1324);
    camera_i2c_write(cam_no,0x01,0x0001);
    camera_i2c_write(cam_no,0x02,0x0004);
    camera_i2c_write(cam_no,0x03,0x01E0);

    camera_i2c_write(cam_no,0x04,0x02F0);
    camera_i2c_write(cam_no,0x05,0x005E);
    camera_i2c_write(cam_no,0x06,0x002D);
    camera_i2c_write(cam_no,0x07,0x0388);

    camera_i2c_write(cam_no,0x0B,0x01E0);
    camera_i2c_write(cam_no,0x35,0x0010);
    camera_i2c_write(cam_no,0xA5,0x003A);
    camera_i2c_write(cam_no,0xA6,0x0002);

    camera_i2c_write(cam_no,0xA8,0x0000);
    camera_i2c_write(cam_no,0xA9,0x0002);
    camera_i2c_write(cam_no,0xAA,0x0002);
    camera_i2c_write(cam_no,0xAB,0x0040);

    camera_i2c_write(cam_no,0xAC,0x0001);
    camera_i2c_write(cam_no,0xAD,0x01E0);
    camera_i2c_write(cam_no,0xAE,0x0014);
    camera_i2c_write(cam_no,0xAF,0x0003);
}

void *cam1_capture(void*)
{
    set_camreg_default(1);
    int r,transferred = 0;
    camera_i2c_write(1,0x07,0x0188);//Normal
    camera_i2c_write(1,0x06,0x002D);//VB
    // cmos设置
    camera_i2c_write(1,0x05,current_HB);//HB
    if(visensor_resolution_status==0)
        camera_i2c_write(1,0x04,IMG_WIDTH_VGA);//HW
    else
        camera_i2c_write(1,0x04,IMG_WIDTH_WVGA);//HW
    switch (EG_mode)
    {
    case 0:
        camera_i2c_write(1,0xAF,0x00);//AEC
        camera_i2c_write(1,0x0B,man_exp);//Exposure Time
        camera_i2c_write(1,0x35,man_gain);//Gain
        break;
    case 1:
        camera_i2c_write(1,0xAF,0x03);//AEC
        camera_i2c_write(1,0xA5,auto_EG_des);//AEC
        camera_i2c_write(1,0xA6,0x01);//AEC
        camera_i2c_write(1,0xA8,0x00);//AEC
        camera_i2c_write(1,0xAC,auto_EG_bottom);//AEC
        camera_i2c_write(1,0xAD,auto_EG_top);//AEC
        camera_i2c_write(1,0xAE,2);//AEC
        break;
    case 2:
        camera_i2c_write(1,0xAF,0x01);//AEC
        camera_i2c_write(1,0xA5,auto_EG_des);//AEC
        camera_i2c_write(1,0xA6,0x01);//AEC
        camera_i2c_write(1,0xA8,0x00);//AEC
        camera_i2c_write(1,0xAC,auto_E_man_G_Ebottom);//AEC
        camera_i2c_write(1,0xAD,auto_E_man_G_Etop);//AEC
        camera_i2c_write(1,0xAE,2);//AEC
        camera_i2c_write(1,0x35,auto_E_man_G);//Gain
        break;
    case 3:
        break;
    case 4:
        camera_i2c_write(1,0xA6,agc_aec_skip_frame);
        camera_i2c_write(1,0xA8,2);
        camera_i2c_write(1,0xA9,agc_aec_skip_frame);
        camera_i2c_write(1,0xAA,2);
        break;
    default:
        break;
    }

    control_camera(1,fps_control());
    usleep(100);

    int ctt=0;
    double time_offset=1.0f/visensor_get_hardware_fps();
    control_camera(1,STANDBY_SHORT);
    struct timeval cap_systime;
    int im_buf_size=0;
    uint32_t count=0;
    while(!shut_down_tag)
    {
        /*ctt++;
        if(ctt>=4)	//该数值越大，则帧同步的周期越长，同步频率越小
        {
            control_camera(1,STANDBY_SHORT);
            ctt = 0;
        }*/

        for(gFrameCam1=0; gFrameCam1<FRAME_CLUST; gFrameCam1++)
        {
            if(shut_down_tag)break;
            im1[gFrameCam1].pass=0;

            im_buf_size = (visensor_resolution_status==1?IMG_BUF_SIZE_WVGA:IMG_BUF_SIZE_VGA);
//            im_buf_size = (visensor_resolution_status==1?IMG_SIZE_WVGA:IMG_SIZE_VGA);
            r = cyusb_bulk_transfer(pcam1_handle, 0x82, im1[gFrameCam1].data, im_buf_size, &transferred, 2000);
            gettimeofday(&cap_systime,NULL);
            im1[gFrameCam1].timestamp = cap_systime.tv_sec+0.000001*cap_systime.tv_usec-time_offset;
            if(r)
            {
            	printf("cam1 bulk transfer returned: %d\n",r);
            }
           check_img(1,im1[gFrameCam1].data,&im1[gFrameCam1].pass);
            if(im1[gFrameCam1].pass==1)
            {
                count++;
                im1[gFrameCam1].count=count;
            }
        }
    }

    pthread_exit(NULL);
}

void *cam2_capture(void*)
{
    set_camreg_default(2);
    int r,transferred = 0;

    camera_i2c_write(2,0x07,0x0188);//Normal
    camera_i2c_write(2,0x06,0x002D);//VB
    // cmos设置
    camera_i2c_write(2,0x05,current_HB);//HB
    if(!visensor_resolution_status)camera_i2c_write(2,0x04,IMG_WIDTH_VGA);//HW
    else camera_i2c_write(2,0x04,IMG_WIDTH_WVGA);//HW
    switch (EG_mode) {
    case 0:
        camera_i2c_write(2,0xAF,0x00);//AEC
        camera_i2c_write(2,0x0B,man_exp);//Exposure Time
        camera_i2c_write(2,0x35,man_gain);//Gain
        break;
    case 1:
        camera_i2c_write(2,0xAF,0x03);//AEC
        camera_i2c_write(2,0xA5,auto_EG_des);//AEC
        camera_i2c_write(2,0xA6,0x01);//AEC
        camera_i2c_write(2,0xA8,0x00);//AEC
        camera_i2c_write(2,0xAC,auto_EG_bottom);//AEC
        camera_i2c_write(2,0xAD,auto_EG_top);//AEC
        camera_i2c_write(2,0xAE,2);//AEC
        break;
    case 2:
        camera_i2c_write(2,0xAF,0x01);//AEC
        camera_i2c_write(2,0xA5,auto_EG_des);//AEC
        camera_i2c_write(2,0xA6,0x01);//AEC
        camera_i2c_write(2,0xA8,0x00);//AEC
        camera_i2c_write(2,0xAC,auto_E_man_G_Ebottom);//AEC
        camera_i2c_write(2,0xAD,auto_E_man_G_Etop);//AEC
        camera_i2c_write(2,0xAE,2);//AEC
        camera_i2c_write(2,0x35,auto_E_man_G);//Gain
        break;
    case 3:
        break;
    case 4:
        camera_i2c_write(2,0xA6,agc_aec_skip_frame);
        camera_i2c_write(2,0xA8,2);
        camera_i2c_write(2,0xA9,agc_aec_skip_frame);
        camera_i2c_write(2,0xAA,2);
        break;
    default:
        break;
    }

    control_camera(2,fps_control());
    usleep(100);

    int ctt=0;
    double time_offset=1.0f/visensor_get_hardware_fps();
    control_camera(2,STANDBY_SHORT);
    struct timeval cap_systime;
    int im_buf_size=0;
    uint32_t count=0;
    while(!shut_down_tag)
    {
	/*ctt++;
        if(ctt>=4)	//该数值越大，则帧同步的周期越长，同步频率越小
        {
            control_camera(2,STANDBY_SHORT);
            ctt = 0;
        }*/
        for(gFrameCam2=0; gFrameCam2<FRAME_CLUST; gFrameCam2++)
        {
            if(shut_down_tag)break;
            im2[gFrameCam2].pass=0;
            im_buf_size = (visensor_resolution_status==1?IMG_BUF_SIZE_WVGA:IMG_BUF_SIZE_VGA);
	    //im_buf_size = (visensor_resolution_status==1?IMG_SIZE_WVGA:IMG_SIZE_VGA);
            r = cyusb_bulk_transfer(pcam2_handle, 0x82, im2[gFrameCam2].data, im_buf_size, &transferred, 2000);
            gettimeofday(&cap_systime,NULL);
            im2[gFrameCam2].timestamp = cap_systime.tv_sec+0.000001*cap_systime.tv_usec-time_offset;
            if(r)
	    {
		printf("cam2 bulk transfer returned: %d\n",r);
	    }	

	    //printf("cam2 bulk transfer returned: %d\n",r);
            check_img(2,im2[gFrameCam2].data,&im2[gFrameCam2].pass);
            if(im2[gFrameCam2].pass==1)
            {
                count++;
                im2[gFrameCam2].count=count;
            }
        }
    }

    pthread_exit(NULL);
}

static int visensor_get_latest_count(struct img_s im[], int* index=NULL)
{
    int max_count=0;
    int temp_index=-1;
    for(int i=0;i<FRAME_CLUST;i++)
    {
        if(im[i].pass)
        {
//        	printf("cam1 returned 110 \n");
            if(im[i].count>max_count)
            {
                temp_index = i;
                max_count = im[i].count;
            }
        }
    }
    if(index!=NULL)
        *index=temp_index;
    return max_count;
}

static bool visensor_is_img_new(struct img_s im[], int* current_count)
{
    int count = visensor_get_latest_count(im);
//    printf("cam1 returned: %d\n",count);
    if(count<=*current_count)
        return false;
    *current_count = count;
    return true;
}

bool visensor_is_left_img_new()
{
    static int current_count=-1;
    return visensor_is_img_new(im1,&current_count);
}

bool visensor_is_right_img_new()
{
    static int current_count=-1;
    return visensor_is_img_new(im2,&current_count);
}

int visensor_get_left_latest_img(unsigned char* img, double* timestamp, visensor_imudata* imudata)
{
    int index;
    int count = visensor_get_latest_count(im1, &index);
    if(index<0)
        return -1;

    int im_size = (visensor_resolution_status==1?IMG_SIZE_WVGA:IMG_SIZE_VGA);
    memcpy(img,im1[index].data,im_size);
    if(timestamp!=NULL)
        *timestamp = im1[index].timestamp;
    if(imudata!=NULL)
        visensor_get_imudata_from_timestamp(imudata,*timestamp);
    return 0;
}

int visensor_get_right_latest_img(unsigned char* img, double* timestamp, visensor_imudata* imudata)
{
    int index;
    int count = visensor_get_latest_count(im2, &index);
    if(index<0)
        return -1;

    int im_size = (visensor_resolution_status==1?IMG_SIZE_WVGA:IMG_SIZE_VGA);
    memcpy(img,im2[index].data,im_size);
    if(timestamp!=NULL)
        *timestamp = im2[index].timestamp;
    if(imudata!=NULL)
        visensor_get_imudata_from_timestamp(imudata,*timestamp);
    return 0;
}


int visensor_Start_Cameras()
{
    allow_settings_change=false;

    shut_down_tag=false;

    int ret_val=0;

    DEBUGLOG("Now opening cameras...\r\n");
    //Open usb devices
    int r;
    r = cyusb_open();
    if(r<1)
    {
        LOG("Error: No cameras found! r = %d\r\n",r);
        return -2;
    }
    else
    {
        printf("Number of device of interest found: %d\r\n",r);
    }
    //Get camera position
    cyusb_handle *pusb_handle;
    for(int i=0; i<r; i++)
    {
        unsigned char buf = 0;
        pusb_handle = cyusb_gethandle(i);
        int r_num = cyusb_control_transfer(pusb_handle,0xC0,GET_CAM_LR,0,0,&buf,1,1000);
        if(r_num != 1)
        {
            printf("Getting LR: %d error: i:0xA5,r:%d\r\n",i,r_num);
        }
        else
        {
            if(buf==0xF1)
            {
                pcam1_handle = pusb_handle;
                printf("Left camera found!\r\n");
            }
            else if(buf==0xF0)
            {
                pcam2_handle = pusb_handle;
                printf("Right camera found!\r\n");
            }
        }
    }
    //Check cameras
    if(gSelectCam&0x01)
        if(pcam1_handle==NULL)
        {
            LOG("Error: Left camera was not found!\r\n");
            ret_val= -2;
        }
    if(gSelectCam&0x02)
        if(pcam2_handle==NULL)
        {
            LOG("Error: Right camera was not found!\r\n");
            ret_val= -2;
        }

    if(!(ret_val<0))
    {
        //Create capture thread
        gettimeofday(&visensor_startTime,NULL);		// 初始化时间起点
        int temp=0;
        if(gSelectCam&0x01)
            if(temp = pthread_create(&cam1_capture_thread, NULL, cam1_capture, NULL))
                printf("Failed to create thread cam1_capture_thread\r\n");
       if(gSelectCam&0x02)
            if(temp = pthread_create(&cam2_capture_thread, NULL, cam2_capture, NULL))
                printf("Failed to create thread cam2_capture_thread\r\n");
        usleep(1000);

        return ret_val;
    }
    else return 0;
}

void visensor_Close_Cameras()
{
    shut_down_tag=true;
    /* Waiting for Camera threads */
    if(cam1_capture_thread !=0)
    {
        pthread_join(cam1_capture_thread,NULL);
    }
    if(cam2_capture_thread !=0)
    {
        pthread_join(cam2_capture_thread,NULL);
    }
    cyusb_close();
}

// 检测有无新数据
bool visensor_imu_have_fresh_data()
{
    if(visensor_query_imu_update())
    {
        visensor_erase_imu_update();
        return true;
    }
    return false;
}

int visensor_get_imudata_from_timestamp(visensor_imudata* imudata, double timestamp)
{
    int min_index=-1;
    double min_timeerror=-1.0;
    for(int i=0; i<IMU_FIFO_SIZE; i++)
    {
        double timeerror = IMU_FIFO[i].timestamp - timestamp;
        if(min_timeerror<0||fabs(timeerror)<min_timeerror)
        {
            min_timeerror = fabs(timeerror);
            min_index = i;
        }
    }
    if(min_index<0)
        return -1;
    memcpy(imudata, &IMU_FIFO[min_index], sizeof(visensor_imudata));
    return 0;
}

int visensor_get_imudata_latest(visensor_imudata* imudata)
{
    int latest_index=-1;
    double latest_timestamp = -1.0;
    for(int i=0; i<IMU_FIFO_SIZE; i++)
    {
        if(IMU_FIFO[i].num!=0xFF&&latest_timestamp < IMU_FIFO[i].timestamp)
        {
            latest_timestamp = IMU_FIFO[i].timestamp;
            latest_index = i;
        }
    }
    if(latest_index<0)
        return -1;
    memcpy(imudata, &IMU_FIFO[latest_index], sizeof(visensor_imudata));
    return 0;
}

void *imu_data_feed(void*)
{
    unsigned char imu_frame[IMU_FRAME_LEN];
    double timestamp;
    short int biasX=(short int)imu_acc_bias_X;
    short int biasY=(short int)imu_acc_bias_Y;
    short int biasZ=(short int)imu_acc_bias_Z;
    short int setaccoffset[3] = {biasX,  biasY, biasZ};

    // 在此函数中更新 IMU-FIFO 用于计算同步 VI-pair
    int counter=0;

    for(int i=0; i<IMU_FIFO_SIZE; i++)
    {
        IMU_FIFO[i].num=0xff;
        IMU_FIFO[i].rx=0;
        IMU_FIFO[i].ry=0;
        IMU_FIFO[i].rz=0;
        IMU_FIFO[i].ax=0;
        IMU_FIFO[i].ay=0;
        IMU_FIFO[i].az=0;
        IMU_FIFO[i].qw=0;
        IMU_FIFO[i].qx=0;
        IMU_FIFO[i].qy=0;
        IMU_FIFO[i].qz=0;
        IMU_FIFO[i].timestamp=0;
    }

    visensor_imudata visensor_imudata_pack;
    while(!imu_close)
    {
        if(visensor_get_imu_frame(imu_fd,imu_frame,&timestamp)==0)
        {
            visensor_parse_imu_frame(imu_frame,timestamp,setaccoffset,&visensor_imudata_pack);
            // 将imu更新标记打开
            visensor_mark_imu_update();

            IMU_FIFO[imu_fifo_idx]=visensor_imudata_pack;
            imu_fifo_idx = (imu_fifo_idx+1)%IMU_FIFO_SIZE;
        }
        usleep(100);
    }
    pthread_exit(NULL);
}

int visensor_Start_IMU()
{
    // serial port
    int fd=visensor_open_port(imu_port_name.c_str());
    if(fd<0)
    {
        printf("visensor_open_port error...\r\n");
        return 0;
    }
    printf("visensor_open_port success...\r\n");

    if(visensor_set_opt(fd,115200,8,'N',1)<0)
    {
        printf("visensor_set_opt error...\r\n");
        return 0;
    }
    printf("visensor_set_opt(fd,115200,8,'N',1) success...\r\n");

    static unsigned char sendframe[10]= {0x55,0xAA,0x02};
    short int setaccoffset[3] = {(short int)imu_acc_bias_X, (short int)imu_acc_bias_Y, (short int)imu_acc_bias_Z};
    memcpy(&sendframe[3],setaccoffset,6);
    sendframe[9]=sendframe[3]+sendframe[4]+sendframe[5]+sendframe[6]+sendframe[7]+sendframe[8];
    visensor_send_imu_frame(fd,sendframe,10);

    //Create imu_data thread
    imu_fd=fd;

    int temp;
    if(temp = pthread_create(&imu_thread, NULL, imu_data_feed, NULL))
        printf("Failed to create thread imu_data\r\n");

    return fd;
}

void visensor_Close_IMU()
{
    imu_close=true;
    /* Waiting for imu_thread */
    if(imu_thread !=0)
    {
        pthread_join(imu_thread,NULL);
    }
}
