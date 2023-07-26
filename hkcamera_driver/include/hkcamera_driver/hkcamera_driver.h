#ifndef _HKCameraDriver
#define _HKCameraDriver

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sys/unistd.h>
#include <time.h>
#include <string>
#include <vector>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include "HCNetSDK.h"
#include "hkcamera_driver/HKCameraFrame.h"
#include "hkcamera_driver/PTZCmd.h"

typedef struct _HKCameraInfo
{
    std::string ip;
    std::string userName;
    std::string passWord;
    std::string cameraName;
    NET_DVR_USER_LOGIN_INFO struLoginInfo;
    NET_DVR_DEVICEINFO_V40 struDeviceInfoV40;
    image_transport::Publisher image_pub_;
    int cameraIndex;
    int userID;
}HKCameraInfo;


// class HKCameraDriverSrv
// {
// private:
//     std::string nodeName;
//     ros::NodeHandle nh;
//     // ros::Subscriber sub;
//     ros::ServiceServer service;
//     std::vector<HKCameraInfo>CameraList;
// public:
//     HKCameraDriverSrv(/* args */);
//     HKCameraDriverSrv(std::string nodeName);
//     ~HKCameraDriverSrv();

//     void run();
//     bool ServiceCallback(hkcamera_driver::HKCameraFrame::Request &req, hkcamera_driver::HKCameraFrame::Response &res);
    
// };

class HKCameraDriverNode
{
private:
    std::string nodeName;

    ros::NodeHandle nh;
    sensor_msgs::Image img_;
    // image_transport::CameraPublisher image_pub_;
    
    ros::Subscriber cmd_sub;
    // boost::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_;

    std::vector<HKCameraInfo>cameraList;
    int image_width, image_height;
    int publishRate = 500;
public:
    HKCameraDriverNode(/* args */);
    HKCameraDriverNode(std::string nodeName);
    ~HKCameraDriverNode();

    void subCallBack(const hkcamera_driver::PTZCmd::ConstPtr& msg);
    void run();

};



#endif