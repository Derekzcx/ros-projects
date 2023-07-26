#include <ros/ros.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
#include "hkcamera_driver/HKCameraFrame.h"

using namespace std;
using namespace cv;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "TestClient");
    ros::NodeHandle nh;
    ros::ServiceClient client = nh.serviceClient<hkcamera_driver::HKCameraFrame>("/hkcamera_driver_node/CameraControl");
    hkcamera_driver::HKCameraFrame srv;
    int w, h;
    ros::Rate r(100);
    namedWindow("Clientframe", WINDOW_NORMAL);
    while (ros::ok())
    {
        srv.request.index = 0;
        srv.request.index = 0;
        if (client.call(srv))
        {
            ROS_INFO("Call camera control service success");
            w = srv.response.width;
            h = srv.response.height;
            Mat img = Mat(srv.response.frameData).reshape(3, h);
            imshow("Clientframe", img);
        }
        else
        {
            ROS_INFO("Call camera control service failed");
        }
        waitKey(10);
        ros::spinOnce();
        r.sleep();
    }
    return 0;
}