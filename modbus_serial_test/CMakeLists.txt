cmake_minimum_required(VERSION 3.0.2)
project(modbus_serial_test)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES modbus_serial_test
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  /usr/local/include/modbus
  ${catkin_INCLUDE_DIRS}
)


link_directories(/usr/local/lib)
add_executable(modbus_test src/modbus_test.cpp)
target_link_libraries(modbus_test ${catkin_LIBRARIES} modbus)


