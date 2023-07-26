#include <ros/ros.h>
#include "hkcamera_driver/hkcamera_driver.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "hkcamera_driver_node");
    // HKCameraDriverSrv hcd(std::string("hkcamera_driver_node"));
    HKCameraDriverNode hcd(std::string("~"));
    hcd.run();
    return 0;
}