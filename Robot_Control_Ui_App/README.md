##适用于 ROS 机器人的UI控制界面
- Use qt5 to implement the ros robot human-machine interface
- 使用qt5实现ros机器人人机界面

***

### 二，安装教程
### 二，Installation tutorial
#### 1，首先安装ros对qt pkg的支持， 若要采用 Qt Creator 开发，需要确保安装ros插件，同时增加相关配置，详细可见博文：
https://blog.csdn.net/qq_43961980/article/details/125873542
#### 1，first install ros support for qt pkg
``` bash
sudo apt-get install ros-melodic-qt-create
```

``` bash
sudo apt-get install ros-melodic-qt-build
```
``` bash
sudo apt-get install qtmultimedia5-dev
```

#### 3，编译
Put the package in the ros src package directory：
将软件包放入ros src软件包目录下：
``` bash
catkin_make
```
#### 5，运行
``` bash
rosrun cyrobot_rviz_tree cyrobot_rviz_tree
```
