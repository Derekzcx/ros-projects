#include "hkcamera_driver/hkcamera_driver.h"
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/fill_image.h>
#include <opencv2/opencv.hpp>

#define  HPR_OK 0
#define  HPR_ERROR -1

#define PTZ_UP         1
#define PTZ_DOWN       2
#define PTZ_LEFT       3
#define PTZ_RIGHT      4

#define SPEED_LIMIT_MIN   1
#define SPEED_LIMIT_MAX   7

#define TMP_IMG_FILE   "tmp.jpeg"

using namespace std;
using namespace cv;

// HKCameraDriverSrv::HKCameraDriverSrv(/* args */)
// {

// }

// HKCameraDriverSrv::HKCameraDriverSrv(std::string nd) : nh(nd.c_str())
// {
//     nodeName = nd;

//     service = nh.advertiseService("CameraControl", &HKCameraDriverSrv::ServiceCallback, this);
// }

// HKCameraDriverSrv::~HKCameraDriverSrv()
// {
//     //logout
//     NET_DVR_Logout_V30(lUserID);
//     NET_DVR_Cleanup();
// }

// bool HKCameraDriverSrv::ServiceCallback(hkcamera_driver::HKCameraFrame::Request &req, hkcamera_driver::HKCameraFrame::Response &res)
// {
//     //
//     NET_DVR_JPEGPARA strPicPara = {0};
//     strPicPara.wPicQuality = 2;
//     strPicPara.wPicSize = 0;
//     int iRet;
//     ROS_INFO("Receive srv call\n");
//     iRet = NET_DVR_CaptureJPEGPicture(lUserID, struDeviceInfoV40.struDeviceV30.byStartChan, &strPicPara, TMP_IMG_FILE);
//     if (!iRet)
//     {
//         printf("pyd1---NET_DVR_CaptureJPEGPicture error, %d\n", NET_DVR_GetLastError());
//         return false;
//     }
//     else
//     {
//         ROS_INFO("GET frame success\n");
//     }
//     ros::Rate r(100);
//     r.sleep();
//     Mat img = imread(TMP_IMG_FILE);
//     if (img.empty())
//     {
//         printf("Get an empty image\n");
//         return false;
//     }
//     res.width = img.cols;
//     res.height = img.rows;
//     res.frameData = vector<uint8_t>(img.reshape(1, res.height * res.width * 3));
//     // res.frameData = 
//     return true;
// }

// void HKCameraDriverSrv::run()
// {
//     NET_DVR_Init();
//     NET_DVR_SetLogToFile(3, "./sdkLog");

//     struLoginInfo.bUseAsynLogin = false;
//     struLoginInfo.wPort = 8000;
//     memcpy(struLoginInfo.sDeviceAddress, "10.90.123.159", NET_DVR_DEV_ADDRESS_MAX_LEN);
//     memcpy(struLoginInfo.sUserName, "admin", NAME_LEN);
//     memcpy(struLoginInfo.sPassword, "cqupt311", NAME_LEN);

//     lUserID = NET_DVR_Login_V40(&struLoginInfo, &struDeviceInfoV40);

//     if (lUserID < 0)
//     {
//         printf("pyd---Login error, %d\n", NET_DVR_GetLastError());
//         printf("Press any key to quit...\n");

//         NET_DVR_Cleanup();
//     }
//     // cv::namedWindow("frame", cv::WINDOW_NORMAL);
//     ros::spin();
// }


void HKCameraDriverNode::subCallBack(const hkcamera_driver::PTZCmd::ConstPtr& msg)
{
    int index = msg->cameraIndex;
    ROS_INFO("Msg call back");
    if (index < 0 || index >= cameraList.size())
    {
        ROS_INFO("Camera%d does not exist", index);
        return;
    }
    int userID = cameraList[index].userID;    
    int speed = msg->speed < SPEED_LIMIT_MIN ? SPEED_LIMIT_MIN : msg->speed;
    speed %= (SPEED_LIMIT_MAX + 1);
    int time = msg->time * 1000;
    ROS_INFO("Get param: index=%d,operator_type=%d,speed=%d,time=%d", index, msg->PTZType, speed, time);
    switch (msg->PTZType)
    {
    case PTZ_UP:
        NET_DVR_PTZControl_Other(userID, 1, TILT_UP, 0);
        usleep(time);
        NET_DVR_PTZControl_Other(userID, 1, TILT_UP, 1);
        break;
    case PTZ_DOWN:
        NET_DVR_PTZControl_Other(userID, 1, TILT_DOWN, 0);
        usleep(time);
        NET_DVR_PTZControl_Other(userID, 1, TILT_DOWN, 1);
        break;
    case PTZ_LEFT:
        NET_DVR_PTZControl_Other(userID, 1, PAN_LEFT, 0);
        usleep(time);
        NET_DVR_PTZControl_Other(userID, 1, PAN_LEFT, 1);
        break;
    case PTZ_RIGHT:
        NET_DVR_PTZControl_Other(userID, 1, PAN_RIGHT, 0);
        usleep(time);
        NET_DVR_PTZControl_Other(userID, 1, PAN_RIGHT, 1);
        break;
    default:
        break;
    }
}

HKCameraDriverNode::HKCameraDriverNode()
{
    
}

HKCameraDriverNode::HKCameraDriverNode(std::string nd) : nh(nd.c_str())
{
    std::string pwd;
    //Login device
    NET_DVR_USER_LOGIN_INFO struLoginInfo = {0};
    NET_DVR_DEVICEINFO_V40 struDeviceInfoV40 = {0};

    nh.param<std::string>("hklib_path", pwd, std::string("/home/xtark/ros_ws/src/hkcamera_driver/lib"));
    ROS_INFO("Init hkcamera sdk...");
    if (!pwd.empty())
    {
        if (pwd[pwd.length() - 1] != '/')
            pwd += "/";

        char cryptoPath[2048] = {0};
        sprintf(cryptoPath, (pwd + std::string("/libcrypto.so.1.1")).c_str());
        NET_DVR_SetSDKInitCfg(NET_SDK_INIT_CFG_LIBEAY_PATH, cryptoPath);

        char sslPath[2048] = {0};
        sprintf(sslPath, (pwd + std::string("/libssl.so.1.1")).c_str());
        NET_DVR_SetSDKInitCfg(NET_SDK_INIT_CFG_SSLEAY_PATH, sslPath); 
            
        NET_DVR_LOCAL_SDK_PATH struComPath = {0};
        sprintf(struComPath.sPath, (pwd + std::string("")).c_str()); //HCNetSDKCom文件夹所在的路径
    }

    NET_DVR_Init();
    // NET_DVR_SetLogToFile(3, "./sdkLog");

    nodeName = nd;
    image_transport::ImageTransport it(nh);
    cmd_sub = nh.subscribe("CameraPTZCmd", 1, &HKCameraDriverNode::subCallBack, this);

    int cameraCount = 0;
    nh.param<int>("/camera_count", cameraCount, 0);
    ROS_INFO("Get %d cameras", cameraCount);
    for (int i = 0; i < cameraCount; i++)
    {
        HKCameraInfo info;
        info.cameraIndex = i;
        std::string cameraName = "/camera" + std::to_string(i);
        info.cameraName = cameraName;
        std::string ipParamName = cameraName + "_ip";
        std::string userIDParamName = cameraName + "_user";
        std::string pwParamName = cameraName + "_passwd";
        nh.param<std::string>(ipParamName, info.ip, std::string("10.90.123.159"));
        nh.param<std::string>(userIDParamName, info.userName, std::string("admin"));
        nh.param<std::string>(pwParamName, info.passWord, std::string("cqupt311"));
        std::string topicName = "image_raw" + std::to_string(i);
        ROS_INFO("camera%d:\nip=%s\nuser=%s\npasswd=%s\npublish_topic_name=%s\n", 
                    i, info.ip.c_str(), info.userName.c_str(), info.passWord.c_str(), topicName.c_str());

        struLoginInfo.bUseAsynLogin = false;
        struLoginInfo.wPort = 8000;
        memcpy(struLoginInfo.sDeviceAddress, info.ip.c_str(), NET_DVR_DEV_ADDRESS_MAX_LEN);
        memcpy(struLoginInfo.sUserName, info.userName.c_str(), NAME_LEN);
        memcpy(struLoginInfo.sPassword, info.passWord.c_str(), NAME_LEN);

        info.userID = NET_DVR_Login_V40(&struLoginInfo, &struDeviceInfoV40);

        info.struLoginInfo = struLoginInfo;
        info.struDeviceInfoV40 = struDeviceInfoV40;
        if (info.userID < 0)
        {
            printf("pyd---Login error, %d\n", NET_DVR_GetLastError());
            printf("Press any key to quit...\n");

            NET_DVR_Cleanup();
            continue;
        }
        // image_pub_ = it.advertiseCamera("image_raw", 1);
        info.image_pub_ = it.advertise(topicName.c_str(), 1);
        cameraList.push_back(info);
    }
}

HKCameraDriverNode::~HKCameraDriverNode()
{
    for (int i = 0; i < cameraList.size(); i++)
    {
        NET_DVR_Logout_V30(cameraList[i].userID);
    }
    
    NET_DVR_Cleanup();
}

void HKCameraDriverNode::run()
{
    ros::Rate loop_rate(publishRate);

    while (nh.ok())
    {

        for (int i = 0; i < cameraList.size(); i++)
        {
            HKCameraInfo info = cameraList[i];
            NET_DVR_JPEGPARA strPicPara = {0};
            strPicPara.wPicQuality = 2;
            strPicPara.wPicSize = 0;
            int iRet;
            iRet = NET_DVR_CaptureJPEGPicture(info.userID, info.struDeviceInfoV40.struDeviceV30.byStartChan, &strPicPara, TMP_IMG_FILE);
            if (!iRet)
            {
                printf("pyd1---NET_DVR_CaptureJPEGPicture error, %d\n", NET_DVR_GetLastError());
                continue;
            }
            ros::Rate r(1000);
            r.sleep();
            Mat img = imread(TMP_IMG_FILE);
            if (img.empty())
            {
                printf("Get an empty image\n");
                continue;
            }
            image_width = img.cols;
            image_height = img.rows;
            // img_.header.stamp = ros::Time::now();
            if (img.data == NULL)
            {
                ROS_INFO("Get empty image, img.data == NULL");
                continue;
            } 

            Mat CvImg;
            cv::resize(img, CvImg, cv::Size(640,480));

            sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();

            // publish the image
            info.image_pub_.publish(msg);           
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
}
