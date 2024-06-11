/**
 * @file /include/cyrobot_monitor/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef cyrobot_monitor_QNODE_HPP_
#define cyrobot_monitor_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

// To workaround boost/qt4 problems that won't be bugfixed. Refer to
//    https://bugreports.qt.io/browse/QTBUG-22829
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif
#include <string>
#include <QThread>
#include <QLabel>
#include <QStringListModel>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
#include <image_transport/image_transport.h>   //image_transport
#include <cv_bridge/cv_bridge.h>              //cv_bridge
#include <sensor_msgs/image_encodings.h>    //图像编码格式
#include <map>
#include <QLabel>
#include <QImage>
#include <QSettings>
#include "PTZCmd.h" // 云台控制信息格式
#include <std_msgs/String.h>
/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace cyrobot_monitor {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:
	QNode(int argc, char** argv );
	virtual ~QNode();
	bool init();
	bool init(const std::string &master_url, const std::string &host_url);
  void disinit();
    void move_base(char k,float speed_linear,float speed_trun);
    void set_goal(QString frame,double x,double y,double z,double w);
    void Sub_Image(QString topic,int frame_id);

    void Sub_Cancel(int frame_id);
   void Camera_control(const hkcamera_driver::PTZCmd &msg);
   void Motor_command_pub(const QString &msg);//统一处理下方升降指令

    QMap<QString,QString> get_topic_list();
	void run();

	/*********************
	** Logging
	**********************/
	enum LogLevel {
             Debug,
	         Info,
	         Warn,
	         Error,
	         Fatal
	 };

	QStringListModel* loggingModel() { return &logging_model; }
	void log( const LogLevel &level, const std::string &msg);


Q_SIGNALS:
	void loggingUpdated();
    void rosShutdown();
    void speed_x(double x);
    void speed_y(double y);
    void speed_z(double z); //增
    void power(float p);
    void Master_shutdown();
    void Show_image(int,QImage);
    void position(QString frame,double x,double y,double z,double w);

    void Show_motor_msgs(QString);//升降杆状态信息

private:
	int init_argc;
	char** init_argv;
	ros::Publisher chatter_publisher;
    ros::Subscriber cmdVel_sub;
    ros::Subscriber chatter_subscriber;
    ros::Subscriber pos_sub;
    ros::Subscriber power_sub;
    ros::Publisher goal_pub;
    ros::Publisher cmd_pub;

    ros::Publisher camera_control_pub; // 云台控制话题发送
    ros::Subscriber motor_states_sub;      // 升降杆状态订阅
    ros::Publisher motor_command_pub;  // 升降杆指令发布

    QStringListModel logging_model;

    ros::Subscriber cmdVelsend;
    //图像订阅
    ros::Subscriber image_sub_compressed;// 压缩图像数据
    ros::Subscriber image_sub_compressed1;
    ros::Subscriber image_sub_compressed2;
    ros::Subscriber image_sub_compressed3;
    ros::Subscriber image_sub_compressed5;
    ros::Subscriber image_sub_compressed6;

    image_transport::Subscriber image_sub0; //原始图像或识别图像
    image_transport::Subscriber image_sub1;
    image_transport::Subscriber image_sub2;
    image_transport::Subscriber image_sub3;
    image_transport::Subscriber image_sub5;
    image_transport::Subscriber image_sub6;

    //图像format
    QString video0_format;
    QString video1_format;
    QString video2_format;
    QString video3_format;
    QString video5_format;
    QString video6_format;

    QString odom_topic;
    QString power_topic;
    QString pose_topic;
    QString power_max;
    QString power_min;
    QImage Mat2QImage(cv::Mat const& src);
    void poseCallback(const geometry_msgs::PoseWithCovarianceStamped& pos);
    void speedCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void speedCallback_cmd(const geometry_msgs::TwistConstPtr& msg);//与控制建连接
    void powerCallback(const std_msgs::Float64 &message_holder);

    //升降杆
    void motor_states_Callback(const std_msgs::String &msg);//状态信息


    void imageCallback0_Compressed(const sensor_msgs::CompressedImageConstPtr& msg); // 为了图像数据的顺畅，设置video0 专门接受压缩图像
    void imageCallback1_Compressed(const sensor_msgs::CompressedImageConstPtr& msg);
    void imageCallback2_Compressed(const sensor_msgs::CompressedImageConstPtr& msg);
    void imageCallback3_Compressed(const sensor_msgs::CompressedImageConstPtr& msg);
    void imageCallback5_Compressed(const sensor_msgs::CompressedImageConstPtr& msg);
    void imageCallback6_Compressed(const sensor_msgs::CompressedImageConstPtr& msg);

    void imageCallback0(const sensor_msgs::ImageConstPtr& msg);
    void imageCallback1(const sensor_msgs::ImageConstPtr& msg);
    void imageCallback2(const sensor_msgs::ImageConstPtr& msg);
    void imageCallback3(const sensor_msgs::ImageConstPtr& msg);
    void imageCallback5(const sensor_msgs::ImageConstPtr& msg);
    void imageCallback6(const sensor_msgs::ImageConstPtr& msg);

    void myCallback(const std_msgs::Float64& message_holder);
};

}  // namespace cyrobot_monitor

#endif /* cyrobot_monitor_QNODE_HPP_ */
