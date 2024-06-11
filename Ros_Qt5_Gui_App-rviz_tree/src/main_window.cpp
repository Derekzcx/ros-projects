/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui->
 *
 * @date February 2011
 **/

/*
* 该函数是 Qt 主界面文件，基本上控件触发都由该文件完成
*/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/cyrobot_monitor/main_window.hpp"
#include <ros/ros.h>


/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace cyrobot_monitor {

using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
	: QMainWindow(parent),
    ui(new Ui::MainWindowDesign), 
    qnode(argc,argv)
{
    ui->setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
    //QObject::connect(ui->actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application
    initUis();
    initData();
    //读取配置文件
    ReadSettings();
    setWindowIcon(QIcon(":/images/robot.png"));
    ui->tab_manager->setCurrentIndex(0); // ensure the first tab is showing - qt-designer should have this already hardwired, but often loses it (settings?).
    //QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));
    
    /*********************
    ** Logging
    **********************/
    ui->view_logging->setModel(qnode.loggingModel());
    
    addtopic_form = new AddTopics();
    //绑定添加rviz话题信号
    connect(addtopic_form, SIGNAL(Topic_choose(QTreeWidgetItem *, QString)), this, SLOT(slot_choose_topic(QTreeWidgetItem *, QString)));
    
    /*********************
    ** 自动连接master
    **********************/
    if ( ui->checkbox_remember_settings->isChecked() ) {
        on_button_connect_clicked();
    }
    //云台控制输入控制
    ui->lineEdit_yuntai_speed->setText("1");
    ui->lineEdit_yuntai_time->setText("1");
    validator_speed = new QIntValidator(1,7,this);
    validator_time = new QIntValidator(1,10,this);
    ui->lineEdit_yuntai_speed->setValidator(validator_speed);
    ui->lineEdit_yuntai_time->setValidator(validator_time);
    //链接connect
    connections();

}


//析构函数
MainWindow::~MainWindow()
{
    if(validator_time != nullptr) delete validator_time;
    if(validator_speed != nullptr) delete validator_speed;
    if( base_cmd)
    {
        delete base_cmd;
        base_cmd = nullptr;
    }
    if(map_rviz_)
    {
        delete map_rviz_;
        map_rviz_ = nullptr;
    }
}

//初始化UI
void MainWindow::initUis()
{
    ui->groupBox_3->setEnabled(false);
    m_DashBoard_x =new CCtrlDashBoard(ui->widget_speed_x);
    m_DashBoard_x->setGeometry(ui->widget_speed_x->rect());
    m_DashBoard_x->setValue(0);
    m_DashBoard_y =new CCtrlDashBoard(ui->widget_speed_y);
    m_DashBoard_y->setGeometry(ui->widget_speed_y->rect());
    m_DashBoard_y->setValue(0);

    ui->tab_manager->setCurrentIndex(0);
    ui->tabWidget->setCurrentIndex(0);
    
    ui->pushButton_remove_topic->setEnabled(false);
    ui->pushButton_rename_topic->setEnabled(false);

    //qucik treewidget
    ui->treeWidget_quick_cmd->setHeaderLabels(QStringList()<<"key"<<"values");
    ui->treeWidget_quick_cmd->setHeaderHidden(true);
    
    ui->button_disconnect->setEnabled(false);

    //升降杆
    ui->textEdit_motor_states->clear();
    ui->pushButton_direction_up->setEnabled(false);// 升降杆默认初始状态为向上
    ui->lineEdit_speed_mode_speed->setText("30");
    ui->lineEdit_pos_mode_speed->setText("30");
    ui->lineEdit_pos_mode_pos->setText("0");
    ui->lineEdit_pos_mode_acc->setText("30");

    // 保存图片
    init_save_img();
}

void MainWindow::initData()
{
    m_mapRvizDisplays.insert("Axes", RVIZ_DISPLAY_AXES);
    m_mapRvizDisplays.insert("Camera", RVIZ_DISPLAY_CAMERA);
    m_mapRvizDisplays.insert("DepthCloud", RVIZ_DISPLAY_DEPTHCLOUD);
    m_mapRvizDisplays.insert("Effort", RVIZ_DISPLAY_EFFORT);
    m_mapRvizDisplays.insert("FluidPressure", RVIZ_DISPLAY_FLUIDPRESSURE);
    m_mapRvizDisplays.insert("Grid", RVIZ_DISPLAY_GRID);
    m_mapRvizDisplays.insert("GridCells", RVIZ_DISPLAY_GRIDCELLS);
    m_mapRvizDisplays.insert("Group", RVIZ_DISPLAY_GROUP);
    m_mapRvizDisplays.insert("Illuminance", RVIZ_DISPLAY_ILLUMINANCE);
    m_mapRvizDisplays.insert("Image", RVIZ_DISPLAY_IMAGE);
    m_mapRvizDisplays.insert("InterativerMarker", RVIZ_DISPLAY_INTERATIVEMARKER);
    m_mapRvizDisplays.insert("LaserScan", RVIZ_DISPLAY_LASERSCAN);
    m_mapRvizDisplays.insert("Map", RVIZ_DISPLAY_MAP);
    m_mapRvizDisplays.insert("Marker", RVIZ_DISPLAY_MARKER);
    m_mapRvizDisplays.insert("MarkerArray", RVIZ_DISPLAY_MARKERARRAY);
    m_mapRvizDisplays.insert("Odometry", RVIZ_DISPLAY_ODOMETRY);
    m_mapRvizDisplays.insert("Path", RVIZ_DISPLAY_PATH);
    m_mapRvizDisplays.insert("PointCloud", RVIZ_DISPLAY_POINTCLOUD);
    m_mapRvizDisplays.insert("PointCloud2", RVIZ_DISPLAY_POINTCLOUD2);
    m_mapRvizDisplays.insert("PointStamped", RVIZ_DISPLAY_POINTSTAMPED);
    m_mapRvizDisplays.insert("Polygon", RVIZ_DISPLAY_POLYGON);
    m_mapRvizDisplays.insert("Pose", RVIZ_DISPLAY_POSE);
    m_mapRvizDisplays.insert("PoseArray", RVIZ_DISPLAY_POSEARRAY);
    m_mapRvizDisplays.insert("PoseWithCovariance", RVIZ_DISPLAY_POSEWITHCOVARIANCE);
    m_mapRvizDisplays.insert("Range", RVIZ_DISPLAY_RANGE);
    m_mapRvizDisplays.insert("RelativeHumidity", RVIZ_DISPLAY_RELATIVEHUMIDITY);
    m_mapRvizDisplays.insert("RobotModel", RVIZ_DISPLAY_ROBOTMODEL);
    m_mapRvizDisplays.insert("TF", RVIZ_DISPLAY_TF);
    m_mapRvizDisplays.insert("Temperature", RVIZ_DISPLAY_TEMPERATURE);
    m_mapRvizDisplays.insert("WrenchStamped", RVIZ_DISPLAY_WRENCHSTAMPED);
}

//订阅video话题
void MainWindow::initVideos()
{

   QSettings video_topic_setting("video_topic","cyrobot_monitor");
   QStringList names=video_topic_setting.value("names").toStringList();
   QStringList topics=video_topic_setting.value("topics").toStringList();

   QStringList topics_2 = video_topic_setting.value("topics_2").toStringList();

   if(names.size()==6)
   {
       ui->label_v_name0->setText(names[0]);
       ui->label_v_name1->setText(names[1]);
       ui->label_v_name2->setText(names[2]);
       ui->label_v_name3->setText(names[3]);

       ui->label_v_name5->setText(names[4]);
       ui->label_v_name6->setText(names[5]);
   }
   if(topics.size()==6 || topics_2.size()==6) // 压缩图像显示 与 识别图像
   {
      if(ui->checkBox_identify0->isChecked() == false){
        if(topics[0]!=""){
          qnode.Sub_Cancel(7); // 取消video0的识别订阅（目前是默认当订阅者不启动时是采用shutdown不报错，暂定）
          qnode.Sub_Image(topics[0],0);
        }
      }else{
        if(topics_2[0]!=""){
          qnode.Sub_Cancel(0);
          qnode.Sub_Image(topics_2[0], 7);}
      }

      if(ui->checkBox_identify1->isChecked() == false){
        if(topics[1] != ""){
          qnode.Sub_Cancel(8);
          qnode.Sub_Image(topics[1], 1);
        }
      }else{
        if(topics_2[1] != ""){
          qnode.Sub_Cancel(1);
          qnode.Sub_Image(topics_2[1],8);
        }
      }

      if(ui->checkBox_identify2->isChecked() == false){
        if(topics[2] != ""){
          qnode.Sub_Cancel(9);
          qnode.Sub_Image(topics[2], 2);
        }
      }else{
        if(topics_2[2] != ""){
          qnode.Sub_Cancel(2);
          qnode.Sub_Image(topics_2[2], 9);
        }
      }

      if(ui->checkBox_identify3->isChecked() == false){
        if(topics[3] != "") {
          qnode.Sub_Cancel(10);
          qnode.Sub_Image(topics[3], 3);
        }
      }else{
        if(topics_2[3] != ""){
          qnode.Sub_Cancel(3);
          qnode.Sub_Image(topics_2[3], 10);
        }
      }

      if(ui->checkBox_identify5->isChecked() == false){
        if(topics[4] != ""){
          qnode.Sub_Cancel(12);
          qnode.Sub_Image(topics[4], 5);
        }
      }else{
        if(topics_2[4] != ""){
          qnode.Sub_Cancel(5);
          qnode.Sub_Image(topics_2[4], 12);
        }
      }

      if(ui->checkBox_identify6->isChecked() == false){
        if(topics[5] != ""){
          qnode.Sub_Cancel(13);
          qnode.Sub_Image(topics[5], 6);
        }
      }else{
        qnode.Sub_Cancel(6);
        if(topics_2[5] != ""){
          qnode.Sub_Cancel(13);
          qnode.Sub_Image(topics_2[5], 13);
        }
      }
      //采用异步多线程
      ros::AsyncSpinner spinner(6);
      spinner.start();
      //ros::waitForShutdown();// 该函数不会自旋
   }

   //链接槽函数
   connect(&qnode,SIGNAL(Show_image(int,QImage)),this,SLOT(slot_show_image(int,QImage)));

}


void MainWindow::slot_show_image(int frame_id, QImage image) // 图像显示操作
{
    switch (frame_id)
    {
    case 0:  // 基础界面
        ui->label_video0->setPixmap(QPixmap::fromImage(image).scaled(ui->label_video0->width(),ui->label_video0->height()));
        break;
    case 1:  // 基础界面
        ui->label_video1->setPixmap(QPixmap::fromImage(image).scaled(ui->label_video1->width(),ui->label_video1->height()));
        break;
    case 2:   // 其他界面
        ui->label_video2->setPixmap(QPixmap::fromImage(image).scaled(ui->label_video2->width(),ui->label_video2->height()));
        break;
    case 3:   // 其他界面
        ui->label_video3->setPixmap(QPixmap::fromImage(image).scaled(ui->label_video3->width(),ui->label_video3->height()));
        break;
    case 5:{  // 基础界面
        ui->label_video5->setPixmap(QPixmap::fromImage(image).scaled(ui->label_video5->width(), ui->label_video5->height()));
        break;
       }
    case 6:{  // 基础界面
        ui->label_video6->setPixmap(QPixmap::fromImage(image).scaled(ui->label_video6->width(), ui->label_video6->height()));
        break;
       }
    }
}

/************************ 保存图片的操作 ********************************/
// 保存图片板块
void MainWindow::init_save_img(){
  // 清除所有的图像勾选,可不做，默认是不勾选状态
  // 设置默认保存名
  ui->lineEdit_img_path->setAlignment(Qt::AlignRight); // 右对齐
  ui->lineEdit_img_path->setText("./");
  ui->lineEdit_img_name->setText("img");
}
void MainWindow::slot_open_save_img_path(){
  // 用于获取位置信息
  QString dirpath = QFileDialog::getExistingDirectory(this,"选择目录","./",QFileDialog::ShowDirsOnly);
  ui->lineEdit_img_path->setText(dirpath);
}
// 保存操作(调试废弃)
void MainWindow::option_save_img(int id){
  qDebug()<<"xxx__"<<id;
}
// 保存图片主函数
void MainWindow::slot_save_img(){
  // 位置和文件名为非空判断
  if(ui->lineEdit_img_name->text().isEmpty() || ui->lineEdit_img_path->text().isEmpty()){
    QMessageBox::information(this, "warring", "保存位置 或 保存名为空 ！！！");
    return;
  }
  //获取时间
  QDateTime now = QDateTime::currentDateTime();
  //获取位置
  QString save_path = ui->lineEdit_img_path->text();
  QString save_name = ui->lineEdit_img_name->text();
  // 处理保存
  QVector<bool> flag_save_success(7, true);

  for(int i=0; i<=6; i++){
    switch (i) {
      case 0:{
        if(!ui->checkBox_video_0->isChecked())break;
        const QPixmap *pixmap = ui->label_video0->pixmap();
        if(pixmap) {
          if(!pixmap->save(save_path+"/"+save_name+"_0_"+now.toString("yyyyMMdd_hhmmss")+".jpg")) flag_save_success[0] = false;
        }
        break;
      }
      case 1:{
        if(!ui->checkBox_video_1->isChecked())break;
        const QPixmap *pixmap = ui->label_video1->pixmap();
        if(pixmap) {
          if(!pixmap->save(save_path+"/"+save_name+"_1_"+now.toString("yyyyMMdd_hhmmss")+".jpg")) flag_save_success[1] = false;
        }
        break;
      }
      case 2:{
        if(!ui->checkBox_video_2->isChecked())break;
        const QPixmap *pixmap = ui->label_video2->pixmap();
        if(pixmap) {
          if(!pixmap->save(save_path+"/"+save_name+"_2_"+now.toString("yyyyMMdd_hhmmss")+".jpg")) flag_save_success[2] = false;
        }
        break;
      }
      case 3:{
        if(!ui->checkBox_video_3->isChecked())break;
        const QPixmap *pixmap = ui->label_video3->pixmap();
        if(pixmap) {
          if(!pixmap->save(save_path+"/"+save_name+"_3_"+now.toString("yyyyMMdd_hhmmss")+".jpg")) flag_save_success[3] = false;
        }
        break;
      }
      case 5:{
        if(!ui->checkBox_video_5->isChecked())break;
        const QPixmap *pixmap = ui->label_video5->pixmap();
        if(pixmap) {
          if(!pixmap->save(save_path+"/"+save_name+"_5_"+now.toString("yyyyMMdd_hhmmss")+".jpg")) flag_save_success[5] = false;
        }
        break;
      }
      case 6:{
        if(!ui->checkBox_video_6->isChecked())break;
        const QPixmap *pixmap = ui->label_video6->pixmap();
        if(pixmap) {
          if(!pixmap->save(save_path+"/"+save_name+"_6_"+now.toString("yyyyMMdd_hhmmss")+".jpg")) flag_save_success[6] = false;
        }
        break;
      }
    default:break;
    }
  }

  QString result_ = "已操作保存";
  QString result_error = "失败保存信息：";
  QString result_index = "";
  int counts = 0;
  for(int i=0; i<=6; i++){
    if(i==4) continue;
    if(!flag_save_success[i]) {
        ++counts;
        result_index = result_index + QString::number(i) + " ";
    }
  }
  result_ = result_ + "\n保存路径："+save_path;
  result_error = result_error +"失败个数为 "+ QString::number(counts) + " 个 \n具体信息：" + result_index;

  QMessageBox::information(this, "保存提示",result_ + "\n" +result_error);


}

/********************************************************/

void MainWindow::initRviz()
{
    ui->label_rvizShow->hide();
    map_rviz_=new QRviz(ui->verticalLayout_build_map,"qrviz");
    connect(map_rviz_, &QRviz::ReturnModelSignal, this, &MainWindow::RvizGetModel);
    map_rviz_->GetDisplayTreeModel();
    QMap<QString, QVariant> namevalue;
    namevalue.insert("Line Style", "Billboards");
    namevalue.insert("Color", QColor(160, 160, 160));
    namevalue.insert("Plane Cell Count", 10);
    map_rviz_->DisplayInit(RVIZ_DISPLAY_GRID, "Grid", true, namevalue);
    
    ui->pushButton_add_topic->setEnabled(true);
    ui->pushButton_rvizReadDisplaySet->setEnabled(true);
    ui->pushButton_rvizSaveDisplaySet->setEnabled(true);
}

void MainWindow::RvizGetModel(QAbstractItemModel *model)
{
    m_modelRvizDisplay = model;
    ui->treeView_rvizDisplayTree->setModel(model);
}

void MainWindow::connections()
{
    QObject::connect(&qnode, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView()));
    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(slot_rosShutdown()));
    QObject::connect(&qnode, SIGNAL(Master_shutdown()), this, SLOT(slot_rosShutdown()));
    //connect速度的信号
    //connect(&qnode,SIGNAL(speed_x(double)),this,SLOT(slot_speed_x(double)));
    //connect(&qnode,SIGNAL(speed_y(double)),this,SLOT(slot_speed_y(double)));
    connect(&qnode,SIGNAL(speed_x(double)),this,SLOT(slot_speed_x(double)));
    connect(&qnode,SIGNAL(speed_z(double)),this,SLOT(slot_speed_Z(double)));

    //电源的信号
    connect(&qnode,SIGNAL(power(float)),this,SLOT(slot_power(float)));
    //机器人位置信号
    connect(&qnode,SIGNAL(position(QString,double,double,double,double)),this,SLOT(slot_position_change(QString,double,double,double,double)));
    //绑定快捷按钮相关函数
    connect(ui->quick_cmd_add_btn,SIGNAL(clicked()),this,SLOT(quick_cmd_add()));
    connect(ui->quick_cmd_remove_btn,SIGNAL(clicked()),this,SLOT(quick_cmd_remove()));
    //绑定slider的函数
    connect(ui->horizontalSlider_raw,SIGNAL(valueChanged(int)),this,SLOT(on_Slider_raw_valueChanged(int)));
    connect(ui->horizontalSlider_linear,SIGNAL(valueChanged(int)),this,SLOT(on_Slider_linear_valueChanged(int)));
    //设置界面
    connect(ui->action_2,SIGNAL(triggered(bool)),this,SLOT(slot_setting_frame()));
    //绑定速度控制按钮
    connect(ui->pushButton_i,SIGNAL(clicked()),this,SLOT(slot_cmd_control()));
    connect(ui->pushButton_u,SIGNAL(clicked()),this,SLOT(slot_cmd_control()));
    connect(ui->pushButton_o,SIGNAL(clicked()),this,SLOT(slot_cmd_control()));
    connect(ui->pushButton_j,SIGNAL(clicked()),this,SLOT(slot_cmd_control()));
    connect(ui->pushButton_l,SIGNAL(clicked()),this,SLOT(slot_cmd_control()));
    connect(ui->pushButton_m,SIGNAL(clicked()),this,SLOT(slot_cmd_control()));
    connect(ui->pushButton_back,SIGNAL(clicked()),this,SLOT(slot_cmd_control()));
    connect(ui->pushButton_backr,SIGNAL(clicked()),this,SLOT(slot_cmd_control()));
    connect(ui->pushButton_k,SIGNAL(clicked()),this, SLOT(slot_cmd_control())); // 增加k 这个停止操作
    //connect(ui->p)
    //设置2D Pose
    connect(ui->set_pos_btn,SIGNAL(clicked()),this,SLOT(slot_set_2D_Pos()));
    //设置2D goal
    connect(ui->set_goal_btn,SIGNAL(clicked()),this,SLOT(slot_set_2D_Goal()));
    //设置MoveCamera
    connect(ui->move_camera_btn,SIGNAL(clicked()),this,SLOT(slot_move_camera_btn()));
    //设置Select
    connect(ui->set_select_btn,SIGNAL(clicked()),this,SLOT(slot_set_select()));
    //设置返航点
    connect(ui->set_return_btn,SIGNAL(clicked()),this,SLOT(slot_set_return_point()));
    //返航
    connect(ui->return_btn,SIGNAL(clicked()),this,SLOT(slot_return_point()));
    //左工具栏tab索引改变
    connect(ui->tab_manager,SIGNAL(currentChanged(int)),this,SLOT(slot_tab_manage_currentChanged(int)));
    //右工具栏索引改变
    connect(ui->tabWidget,SIGNAL(currentChanged(int)),this,SLOT(slot_tab_Widget_currentChanged(int)));
    //刷新话题列表
    connect(ui->refreash_topic_btn,SIGNAL(clicked()),this,SLOT(refreashTopicList()));

    //油门模式
    connect(ui->pushButton_mbot, SIGNAL(clicked()), this, SLOT(slot_accelerograph()));//开启油门模式

    //摄像头云台控制
    connect(ui->pushButton_yuntai_up, SIGNAL(clicked()), this, SLOT(slot_camera_control_up()));
    connect(ui->pushButton_yuntai_down, SIGNAL(clicked()), this, SLOT(slot_camera_control_down()));
    connect(ui->pushButton_yuntai_left, SIGNAL(clicked()), this, SLOT(slot_camera_control_left()));
    connect(ui->pushButton_yuntai_right, SIGNAL(clicked()), this, SLOT(slot_camera_control_right()));
//    ui->pushButton_yuntai_up->setShortcutEnabled(Qt::Key_Up);
//    ui->pushButton_yuntai_down->setShortcutEnabled(Qt::Key_Down);
//    ui->pushButton_yuntai_left->setShortcutEnabled(Qt::Key_Left);
//    ui->pushButton_yuntai_right->setShortcutEnabled(Qt::Key_Right);//与键盘关联
    // 升降杆的状态信息和按键绑定
    connect(&qnode,SIGNAL(Show_motor_msgs(QString)),this,SLOT(slot_show_motor_msgs(QString)));
    connect(ui->pushButton_direction_up, SIGNAL(clicked()), this, SLOT(slot_direction_up()));
    connect(ui->pushButton_direction_down, SIGNAL(clicked()), this, SLOT(slot_direction_down()));
    connect(ui->pushButton_motor_stop, SIGNAL(clicked()), this, SLOT(slot_motor_stop()));
    connect(ui->pushButton_speed_mode_execute, SIGNAL(clicked()), this, SLOT(slot_speed_mode_execute()));
    connect(ui->pushButton_set_zero, SIGNAL(clicked()), this, SLOT(slot_set_zero()));
    connect(ui->pushButton_back_zero, SIGNAL(clicked()), this, SLOT(slot_back_zero()));
    connect(ui->pushButton_pos_mode_execute, SIGNAL(clicked()), this, SLOT(slot_pos_mode_execute()));

    // 保存图片
    connect(ui->toolButton_open_img_path, SIGNAL(clicked()), this, SLOT(slot_open_save_img_path()));
    connect(ui->pushButton_save_img, SIGNAL(clicked()), this, SLOT(slot_save_img()));

}
//油门模式启动函数
void MainWindow::slot_accelerograph(){
  system("gnome-terminal -- bash -c 'source ~/ros_ws/devel/setup.bash ;roslaunch mbot_teleop mbot_teleop.launch'&");
}

//设置界面
void MainWindow::slot_setting_frame()
{
    if(set != nullptr)
    {
        delete set;
        set=new Settings();
        set->setWindowModality(Qt::ApplicationModal);
        set->show();
    }
    else
    {
        set=new Settings();
        set->setWindowModality(Qt::ApplicationModal);
        set->show();
    }
    //绑定set确认按钮点击事件
}
//刷新当前坐标
void MainWindow::slot_position_change(QString frame,double x,double y,double z,double w)
{
    //更新ui显示
    ui->label_frame->setText(frame);
    ui->label_x->setText(QString::number(x));
    ui->label_y->setText(QString::number(y));
    ui->label_z->setText(QString::number(z));
    ui->label_w->setText(QString::number(w));
}
//刷新返航地点
void MainWindow::slot_set_return_point()
{
    //更新ui返航点显示
    ui->label_return_x->setText(ui->label_x->text());
    ui->label_return_y->setText(ui->label_y->text());
    ui->label_return_z->setText(ui->label_z->text());
    ui->label_return_w->setText(ui->label_w->text());
    //写入setting
    QSettings settings("return-position", "cyrobot_monitor");
    settings.setValue("x",ui->label_x->text());
    settings.setValue("y",ui->label_y->text());
    settings.setValue("z",ui->label_z->text());
    settings.setValue("w",ui->label_w->text());
    //发出声音提醒
    if(media_player!=nullptr)
     {
         delete media_player;
         media_player=nullptr;
     }
     media_player=new QSoundEffect;
     media_player->setSource(QUrl::fromLocalFile("://media/refresh_return.wav"));
     media_player->play();

}
//返航
void MainWindow::slot_return_point()
{
    qnode.set_goal(ui->label_frame->text(),ui->label_return_x->text().toDouble(),ui->label_return_y->text().toDouble(),ui->label_return_z->text().toDouble(),ui->label_return_w->text().toDouble());
    if(media_player!=nullptr)
       {
           delete media_player;
           media_player=nullptr;
       }
       media_player=new QSoundEffect;
       media_player->setSource(QUrl::fromLocalFile("://media/start_return.wav"));
       media_player->play();
}
//设置导航当前位置按钮的槽函数
void MainWindow::slot_set_2D_Pos()
{
 map_rviz_->Set_Pos();
// ui->label_map_msg->setText("请在地图中选择机器人的初始位置");
}
//设置导航目标位置按钮的槽函数
void MainWindow::slot_set_2D_Goal()
{
  map_rviz_->Set_Goal();
//  ui->label_map_msg->setText("请在地图中选择机器人导航的目标位置");
}
void MainWindow::slot_move_camera_btn()
{
    map_rviz_->Set_MoveCamera();
    qDebug()<<"move camera";
}
void MainWindow::slot_set_select()
{
    map_rviz_->Set_Select();
}


//选中要添加的话题的槽函数
void MainWindow::slot_choose_topic(QTreeWidgetItem *choose, QString name)
{
    QString ClassID = choose->text(0);
    // 检查重名
    name = JudgeDisplayNewName(name);
    
    qDebug() << "choose topic ClassID:" << ClassID << ", name:" << name;
    QMap<QString, QVariant> namevalue;
    namevalue.clear();
    map_rviz_->DisplayInit(m_mapRvizDisplays[ClassID], name, true, namevalue);
}

///
/// \brief 检查重名
/// \param name
/// \return 
///
QString MainWindow::JudgeDisplayNewName(QString name)
{
    if (m_modelRvizDisplay != nullptr)
    {
        bool same = true;
        while (same)
        {
            same = false;
            for (int i = 0; i < m_modelRvizDisplay->rowCount(); i++)
            {
                //m_sRvizDisplayChooseName = index.data().value<QString>();
                if (m_modelRvizDisplay->index(i, 0).data().value<QString>() == name)
                {
                    if (name.indexOf("_") != -1)
                    {
                        int num = name.section("_", -1, -1).toInt();
                        name = name.left(name.length() - name.section("_", -1, -1).length() - 1);
                        if (num <= 1)
                        {
                            num = 2;
                        }
                        else
                        {
                            num++;
                        }
                        name = name + "_" + QString::number(num);
                    }
                    else {
                      name = name + "_2";
                    }
                    same = true;
                    break;
                }
            }
        }
    }
    return name;
}

//左工具栏索引改变
void MainWindow::slot_tab_manage_currentChanged(int index)
{
    switch (index) {
    case 0:

        break;
    case 1:

        ui->tabWidget->setCurrentIndex(1);
        break;
    case 2:
        break;

    }
}
//右工具栏索引改变
void MainWindow::slot_tab_Widget_currentChanged(int index)
{
    switch (index) {
    case 0:

        break;
    case 1:
        ui->tab_manager->setCurrentIndex(1);
        break;
    case 2:
        break;

    }
}
//速度控制相关按钮处理槽函数
void MainWindow::slot_cmd_control()
{

    QPushButton* btn=qobject_cast<QPushButton*>(sender());
    char key=btn->text().toStdString()[0];
    //速度
    float liner=ui->horizontalSlider_linear->value()*0.01f;
    float turn=ui->horizontalSlider_raw->value()*0.01f;
    bool is_all=ui->checkBox_use_all->isChecked();
    switch (key) {
        case 'u':
            qnode.move_base(is_all?'U':'u',liner,turn);
        break;
        case 'i':
            qnode.move_base(is_all?'I':'i',liner,turn);
        break;
        case 'o':
            qnode.move_base(is_all?'O':'o',liner,turn);
        break;
        case 'j':
            qnode.move_base(is_all?'J':'j',liner,turn);
        break;
        case 'l':
            qnode.move_base(is_all?'L':'l',liner,turn);
        break;
        case 'm':
            qnode.move_base(is_all?'M':'m',liner,turn);
        break;
        case ',':
            qnode.move_base(is_all?'<':',',liner,turn);
        break;
        case '.':
            qnode.move_base(is_all?'>':'.',liner,turn);
        break;
        case 'k':
            qnode.move_base(is_all?'K':'k',liner,turn); //增加停止按钮
        break;

    }
}
// 云台上转
void MainWindow::slot_camera_control_up(){
  hkcamera_driver::PTZCmd msg;
  msg.speed = ui->lineEdit_yuntai_speed->text().toInt();//速度在1——7之间
  msg.time = ui->lineEdit_yuntai_time->text().toInt();//运动时间1s
  msg.PTZType = 1;// 1为up
  if(ui->checkBox_yuntai_car_head->checkState()==Qt::Checked){
    msg.cameraIndex = 0;
    qnode.Camera_control(msg);
  }
  if(ui->checkBox_yuntai_raise_1->checkState()==Qt::Checked){
    msg.cameraIndex = 1;
    qnode.Camera_control(msg);
  }
  if(ui->checkBox_yuntai_raise_2->checkState()==Qt::Checked){
    msg.cameraIndex = 2;
    qnode.Camera_control(msg);
  }
  if(ui->checkBox_yuntai_raise_3->checkState()==Qt::Checked){
    msg.cameraIndex = 3;
    qnode.Camera_control(msg);
  }
}
// 云台下转
void MainWindow::slot_camera_control_down(){
  hkcamera_driver::PTZCmd msg;
  msg.speed =  ui->lineEdit_yuntai_speed->text().toInt();//速度在1——7之间
  msg.time =ui->lineEdit_yuntai_time->text().toInt();//运动时间1s
  msg.PTZType = 2;// 2为down
  if(ui->checkBox_yuntai_car_head->checkState()==Qt::Checked){
    msg.cameraIndex = 0;
    qnode.Camera_control(msg);
  }
  if(ui->checkBox_yuntai_raise_1->checkState()==Qt::Checked){
    msg.cameraIndex = 1;
    qnode.Camera_control(msg);
  }
  if(ui->checkBox_yuntai_raise_2->checkState()==Qt::Checked){
    msg.cameraIndex = 2;
    qnode.Camera_control(msg);
  }
  if(ui->checkBox_yuntai_raise_3->checkState()==Qt::Checked){
    msg.cameraIndex = 3;
    qnode.Camera_control(msg);
  }
}
// 云台左转
void MainWindow::slot_camera_control_left(){
  hkcamera_driver::PTZCmd msg;
  msg.speed =  ui->lineEdit_yuntai_speed->text().toInt();//速度在1——7之间
  msg.time = ui->lineEdit_yuntai_time->text().toInt();//运动时间1s
  msg.PTZType = 3;// 2为left
  if(ui->checkBox_yuntai_car_head->checkState()==Qt::Checked){
    msg.cameraIndex = 0;
    qnode.Camera_control(msg);
  }
  if(ui->checkBox_yuntai_raise_1->checkState()==Qt::Checked){
    msg.cameraIndex = 1;
    qnode.Camera_control(msg);
  }
  if(ui->checkBox_yuntai_raise_2->checkState()==Qt::Checked){
    msg.cameraIndex = 2;
    qnode.Camera_control(msg);
  }
  if(ui->checkBox_yuntai_raise_3->checkState()==Qt::Checked){
    msg.cameraIndex = 3;
    qnode.Camera_control(msg);
  }
}
// 云台右转
void MainWindow::slot_camera_control_right(){
  hkcamera_driver::PTZCmd msg;
  msg.speed =  ui->lineEdit_yuntai_speed->text().toInt();//速度在1——7之间
  msg.time = ui->lineEdit_yuntai_time->text().toInt();//运动时间1s
  msg.PTZType = 4;// 4为right
  if(ui->checkBox_yuntai_car_head->checkState()==Qt::Checked){
    msg.cameraIndex = 0;
    qnode.Camera_control(msg);
  }
  if(ui->checkBox_yuntai_raise_1->checkState()==Qt::Checked){
    msg.cameraIndex = 1;
    qnode.Camera_control(msg);
  }
  if(ui->checkBox_yuntai_raise_2->checkState()==Qt::Checked){
    msg.cameraIndex = 2;
    qnode.Camera_control(msg);
  }
  if(ui->checkBox_yuntai_raise_3->checkState()==Qt::Checked){
    msg.cameraIndex = 3;
    qnode.Camera_control(msg);
  }
}

/************************ 升降杆 *****************************/
//enum class Funs{
//    Stop,             // 急停码
//    Change_direction, // 改变允许方向
//    Speed_mode,       // 速度模式运行
//    Pos_mode,         // 速度模式运行
//    Set_zero,         // 设置归0位
//    Back_zero         // 归0
//};

void MainWindow::slot_direction_up(){
  QString tmp = "1:-1";
  qnode.Motor_command_pub(tmp);
  ui->pushButton_direction_up->setEnabled(false);
  ui->pushButton_direction_down->setEnabled(true);
}
void MainWindow::slot_direction_down(){
  QString tmp = "1:-1";
  qnode.Motor_command_pub(tmp);
  ui->pushButton_direction_down->setEnabled(false);
  ui->pushButton_direction_up->setEnabled(true);
}
void MainWindow::slot_motor_stop(){
  QString tmp = "0:-1";
  qnode.Motor_command_pub(tmp);
}
void MainWindow::slot_speed_mode_execute(){
  QString speed = ui->lineEdit_speed_mode_speed->text();
  QString tmp = "2:" + speed;
  qnode.Motor_command_pub(tmp);
}
void MainWindow::slot_pos_mode_execute(){
  QString speed = ui->lineEdit_pos_mode_speed->text();
  QString acc = ui->lineEdit_pos_mode_acc->text();
  QString pos = ui->lineEdit_pos_mode_pos->text();
  QString tmp = "3:" + pos + "," + speed + "," + acc;
  qnode.Motor_command_pub(tmp);
}
void MainWindow::slot_set_zero(){
  QString tmp = "4:-1";
  qnode.Motor_command_pub(tmp);
}
void MainWindow::slot_back_zero(){
  QString tmp = "5:-1";
  qnode.Motor_command_pub(tmp);
}
void MainWindow::slot_show_motor_msgs(QString msg){
   ui->textEdit_motor_states->append(msg);
}
/*****************************************************/
//滑动条处理槽函数
void MainWindow::on_Slider_raw_valueChanged(int v)
{
    ui->label_raw->setText(QString::number(v));
}
//滑动条处理槽函数
void MainWindow::on_Slider_linear_valueChanged(int v)
{
    ui->label_linear->setText(QString::number(v));
}
//快捷指令删除按钮
void MainWindow::quick_cmd_remove()
{
    QTreeWidgetItem *curr=ui->treeWidget_quick_cmd->currentItem();
    //没有选择节点
    if(curr == nullptr) return;
    //获取父节点
    QTreeWidgetItem* parent=curr->parent();
    //如果当前节点就为父节点
    if(parent == nullptr)
    {
        ui->treeWidget_quick_cmd->takeTopLevelItem(ui->treeWidget_quick_cmd->indexOfTopLevelItem(curr));
        delete curr;
    }
    else{
        ui->treeWidget_quick_cmd->takeTopLevelItem(ui->treeWidget_quick_cmd->indexOfTopLevelItem(parent));
        delete parent;
    }


}
//快捷指令添加按钮
void MainWindow::quick_cmd_add()
{
    QWidget *w=new QWidget;
    //阻塞其他窗体
    w->setWindowModality(Qt::ApplicationModal);
    QLabel *name=new QLabel;
    name->setText("名称:");
    QLabel *content=new QLabel;
    content->setText("脚本:");
    QLineEdit *name_val=new QLineEdit;
    QTextEdit *shell_val=new QTextEdit;
    QPushButton *ok_btn=new QPushButton;
    ok_btn->setText("ok");
    ok_btn->setIcon(QIcon("://images/ok.png"));
    QPushButton *cancel_btn=new QPushButton;
    cancel_btn->setText("cancel");
    cancel_btn->setIcon(QIcon("://images/false.png"));
    QHBoxLayout *lay1=new QHBoxLayout;
    lay1->addWidget(name);
    lay1->addWidget(name_val);
    QHBoxLayout *lay2=new QHBoxLayout;
    lay2->addWidget(content);
    lay2->addWidget(shell_val);
    QHBoxLayout *lay3=new QHBoxLayout;
    lay3->addWidget(ok_btn);
    lay3->addWidget(cancel_btn);
    QVBoxLayout *v1=new QVBoxLayout;
    v1->addLayout(lay1);
    v1->addLayout(lay2);
    v1->addLayout(lay3);

    w->setLayout(v1);
    w->show();

    connect(ok_btn,&QPushButton::clicked,[this,w,name_val,shell_val]
    {
        this->add_quick_cmd(name_val->text(),shell_val->toPlainText());
        w->close();
    });
}
//向treeWidget添加快捷指令
void MainWindow::add_quick_cmd(QString name,QString val)
{
    if(name=="") return;
    QTreeWidgetItem *head=new QTreeWidgetItem(QStringList()<<name);
    this->ui->treeWidget_quick_cmd->addTopLevelItem(head);
    QCheckBox *check=new QCheckBox;
    //记录父子关系
    this->widget_to_parentItem_map[check]=head;
    //连接checkbox选中的槽函数
    connect(check,SIGNAL(stateChanged(int)),this,SLOT(quick_cmds_check_change(int)));
    this->ui->treeWidget_quick_cmd->setItemWidget(head,1,check);
    QTreeWidgetItem *shell_content=new QTreeWidgetItem(QStringList()<<"shell");
    QTextEdit *shell_val=new QTextEdit;
    shell_val->setMaximumWidth(150);
    shell_val->setMaximumHeight(40);
    head->addChild(shell_content);
    shell_val->setText(val);
    this->ui->treeWidget_quick_cmd->setItemWidget(shell_content,1,shell_val);
}
//快捷指令按钮处理的函数
void MainWindow::quick_cmds_check_change(int state)
{

    QCheckBox* check = qobject_cast<QCheckBox*>(sender());
    QTreeWidgetItem *parent=widget_to_parentItem_map[check];
    QString bash = static_cast<QTextEdit *>(ui->treeWidget_quick_cmd->itemWidget(parent->child(0),1))->toPlainText();
    bool is_checked = state>1 ? true : false;
    if(is_checked)
    {
        quick_cmd = new QProcess;
        quick_cmd->start("bash");
        qDebug() << bash;
        quick_cmd->write(bash.toLocal8Bit()+'\n');
        connect(quick_cmd,SIGNAL(readyReadStandardOutput()),this,SLOT(cmd_output()));
         connect(quick_cmd,SIGNAL(readyReadStandardError()),this,SLOT(cmd_error_output()));
    }
    else{


    }

}
//执行一些命令的回显
void MainWindow::cmd_output()
{

    ui->cmd_output->append(quick_cmd->readAllStandardOutput());
}
//执行一些命令的错误回显
void MainWindow::cmd_error_output()
{
    ui->cmd_output->append("<font color=\"#FF0000\">"+quick_cmd->readAllStandardError()+"</font> ");
}



/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

void MainWindow::showNoMasterMessage() {
	QMessageBox msgBox;
	msgBox.setText("Couldn't find the ros master.");
	msgBox.exec();
    close();
}

void MainWindow::inform(QString strdata)
{
    QMessageBox m_r;
    m_r.setWindowTitle("提示");
    m_r.setText(strdata);
    m_r.exec();
}
bool MainWindow::AskInform(QString strdata)
{
    QMessageBox m_r;

    m_r.setWindowTitle("提示");
    m_r.setText(strdata);
    m_r.addButton(tr("确认"), QMessageBox::ActionRole);
    m_r.addButton(tr("取消"), QMessageBox::ActionRole);

    int isok = m_r.exec();
    if (isok == 1)
    {
        return false;
    }
    else
    {
        return true;
    }
}

/*
 * These triggers whenever the button is clicked, regardless of whether it
 * is already checked or not.
 */

void MainWindow::initTopicList()
{
    ui->topic_listWidget->clear();
    ui->topic_listWidget->addItem(QString("%1   (%2)").arg("Name","Type"));
    QMap<QString,QString> topic_list= qnode.get_topic_list();
    for(QMap<QString,QString>::iterator iter=topic_list.begin();iter!=topic_list.end();iter++)
    {
       ui->topic_listWidget->addItem(QString("%1   (%2)").arg(iter.key(),iter.value()));
    }
}
void MainWindow::refreashTopicList()
{
    initTopicList();
}
//当ros与master的连接断开时
void MainWindow::slot_rosShutdown()
{
    ui->label_robot_staue_img->setPixmap(QPixmap::fromImage(QImage("://images/offline.png")));
     ui->label_statue_text->setStyleSheet("color:red;");
    ui->label_statue_text->setText("离线");
    ui->button_connect->setEnabled(true);
    ui->line_edit_master->setReadOnly(false);
    ui->line_edit_host->setReadOnly(false);
    ui->line_edit_topic->setReadOnly(false);
    clean_all_msgs();//清除显示
}
void MainWindow::slot_power(float p)
{
    ui->label_power->setText(QString::number(static_cast<double>(p)).mid(0,5)+"V");
    double n=(static_cast<double>(p)-10)/1.5;
    int value = static_cast<int>(n*100);
    ui->progressBar->setValue(value>100?100:value);
    //当电量过低时发出提示
    if(n*100<=20)
    {
         ui->progressBar->setStyleSheet("QProgressBar::chunk {background-color: red;width: 20px;} QProgressBar {border: 2px solid grey;border-radius: 5px;text-align: center;}");
          // QMessageBox::warning(NULL, "电量不足", "电量不足，请及时充电！", QMessageBox::Yes , QMessageBox::Yes);
    }
    else{
        ui->progressBar->setStyleSheet("QProgressBar {border: 2px solid grey;border-radius: 5px;text-align: center;}");
    }
}
void MainWindow::slot_speed_x(double x)
{
    if(x>=0) ui->label_dir_x->setText("正向");
    else ui->label_dir_x->setText("反向");

    m_DashBoard_x->setValue(abs(x*100));
}
void MainWindow::slot_speed_y(double x)
{
    if(x>=0) ui->label_dir_y->setText("正向");
    else ui->label_dir_y->setText("反向");
    m_DashBoard_y->setValue(abs(x*100));
}

void MainWindow::slot_speed_Z(double z){
    if(z>=0) ui->label_dir_y->setText("正向");
    else ui->label_dir_y->setText("反向");
    m_DashBoard_y->setValue(abs(z*100));
}

void MainWindow::on_checkbox_use_environment_stateChanged(int state) {
	bool enabled;
	if ( state == 0 ) {
		enabled = true;
	} else {
		enabled = false;
	}
	ui->line_edit_master->setEnabled(enabled);
	ui->line_edit_host->setEnabled(enabled);
	//ui->line_edit_topic->setEnabled(enabled);
}

/*****************************************************************************
** Implemenation [Slots][manually connected]
*****************************************************************************/

/**
 * This function is signalled by the underlying model. When the model changes,
 * this will drop the cursor down to the last line in the QListview to ensure
 * the user can always see the latest log message.
 */
void MainWindow::updateLoggingView() {
        ui->view_logging->scrollToBottom();
}

/*****************************************************************************
** Implementation [Menu]
*****************************************************************************/

void MainWindow::on_actionAbout_triggered() {
    //QMessageBox::about(this, tr("About ..."),tr("<h2>PACKAGE_NAME Test Program 0.10</h2><p>Copyright Yujin Robot</p><p>This package needs an about description.</p>"));
}

/*****************************************************************************
** Implementation [Configuration]
*****************************************************************************/

void MainWindow::ReadSettings() {
    QSettings settings("Qt-Ros Package", "cyrobot_rviz_tree");
    restoreGeometry(settings.value("geometry").toByteArray());
    restoreState(settings.value("windowState").toByteArray());
    QString master_url = settings.value("master_url",QString("http://192.168.1.2:11311/")).toString();
    QString host_url = settings.value("host_url", QString("192.168.1.3")).toString();
    //QString topic_name = settings.value("topic_name", QString("/chatter")).toString();
    ui->line_edit_master->setText(master_url);
    ui->line_edit_host->setText(host_url);
    //ui->line_edit_topic->setText(topic_name);
    bool remember = settings.value("remember_settings", false).toBool();
    ui->checkbox_remember_settings->setChecked(remember);
    bool checked = settings.value("use_environment_variables", false).toBool();
    ui->checkbox_use_environment->setChecked(checked);
    if ( checked ) {
    	ui->line_edit_master->setEnabled(false);
    	ui->line_edit_host->setEnabled(false);
    	//ui->line_edit_topic->setEnabled(false);
    }

    QSettings return_pos("return-position","cyrobot_rviz_tree");
    ui->label_return_x->setText(return_pos.value("x",QString("0")).toString());
    ui->label_return_y->setText(return_pos.value("y",QString("0")).toString());
    ui->label_return_z->setText(return_pos.value("z",QString("0")).toString());
    ui->label_return_w->setText(return_pos.value("w",QString("0")).toString());

    //读取快捷指令的setting
    QSettings quick_setting("quick_setting","cyrobot_rviz_tree");
    QStringList ch_key=quick_setting.childKeys();
    for(auto c:ch_key)
    {
        add_quick_cmd(c,quick_setting.value(c,QString("")).toString());
    }

}

void MainWindow::WriteSettings() {
    QSettings settings("Qt-Ros Package", "cyrobot_rviz_tree");
    settings.setValue("master_url",ui->line_edit_master->text());
    settings.setValue("host_url",ui->line_edit_host->text());
    //settings.setValue("topic_name",ui->line_edit_topic->text());
    settings.setValue("use_environment_variables",QVariant(ui->checkbox_use_environment->isChecked()));
    settings.setValue("geometry", saveGeometry());
    //settings.setValue("windowState", saveState());
    settings.setValue("remember_settings",QVariant(ui->checkbox_remember_settings->isChecked()));

    //存下快捷指令的setting
    QSettings quick_setting("quick_setting","cyrobot_rviz_tree");
    quick_setting.clear();
    for(int i=0;i<ui->treeWidget_quick_cmd->topLevelItemCount();i++)
    {
        QTreeWidgetItem *top=ui->treeWidget_quick_cmd->topLevelItem(i);
        QTextEdit *cmd_val= static_cast<QTextEdit *>(ui->treeWidget_quick_cmd->itemWidget(top->child(0),1));
        quick_setting.setValue(top->text(0),cmd_val->toPlainText());
    }
}

void MainWindow::closeEvent(QCloseEvent *event)
{
    WriteSettings();
    QMainWindow::closeEvent(event);
}

}  // namespace cyrobot_monitor

///
/// \brief 增加topic
///
void cyrobot_monitor::MainWindow::on_pushButton_add_topic_clicked()
{
    addtopic_form->show();
}

///
/// \brief 删除topic
///
void cyrobot_monitor::MainWindow::on_pushButton_remove_topic_clicked()
{
    if (ui->treeView_rvizDisplayTree->currentIndex().row() >= 0)
    {
        m_sRvizDisplayChooseName_ = ui->treeView_rvizDisplayTree->currentIndex().data().value<QString>();
        map_rviz_->RemoveDisplay(m_sRvizDisplayChooseName_);
        if (ui->treeView_rvizDisplayTree->currentIndex().row() >= 0)
        {
            on_treeView_rvizDisplayTree_clicked(ui->treeView_rvizDisplayTree->currentIndex());
        }
        else
        {
            m_sRvizDisplayChooseName_.clear();
        }
    }
    else
    {
        inform("请选择Display后再执行此操作");
    }
}

///
/// \brief 重命名topic
///
void cyrobot_monitor::MainWindow::on_pushButton_rename_topic_clicked()
{
    if (ui->treeView_rvizDisplayTree->currentIndex().row() < 0)
    {
        inform("请选择Display后再执行此操作");
        return ;
    }
    QString dlgTitle = "重命名";
    QString txtlabel = "请输入名字：";
    QString defaultInupt = m_sRvizDisplayChooseName_;
    QLineEdit::EchoMode echoMode = QLineEdit::Normal;
    bool ok = false;
    QString newname = QInputDialog::getText(this, dlgTitle, txtlabel, echoMode, defaultInupt, &ok);
    if (ok && !newname.isEmpty())
    {
        if (newname != defaultInupt)
        {
            QString nosamename = JudgeDisplayNewName(newname);
            map_rviz_->RenameDisplay(defaultInupt, nosamename);
            m_sRvizDisplayChooseName_ = nosamename;
            if (nosamename != newname)
            {
                inform("命名重复！命名已自动更改为" + nosamename);
            }
        }
    }
    else if (ok)
    {
        inform("输入内容为空，重命名失败");
    }
}

void cyrobot_monitor::MainWindow::on_treeView_rvizDisplayTree_clicked(const QModelIndex &index)
{
    m_sRvizDisplayChooseName_ = index.data().value<QString>();
    if (index.parent().row() == -1)   // Display 的根节点
    {
        if (index.row() > 1)
        {
            ui->pushButton_remove_topic->setEnabled(true);
            ui->pushButton_rename_topic->setEnabled(true);
        }
        else
        {
            ui->pushButton_remove_topic->setEnabled(false);
            ui->pushButton_rename_topic->setEnabled(false);
        }
    }
    else
    {
        ui->pushButton_remove_topic->setEnabled(false);
        ui->pushButton_rename_topic->setEnabled(false);
    }
}

/// \brief 连接ROS
void cyrobot_monitor::MainWindow::on_button_connect_clicked()
{
    //如果使用环境变量
    if ( ui->checkbox_use_environment->isChecked() )
    {
        if ( !qnode.init() )
        {
            //showNoMasterMessage();
            QMessageBox::warning(nullptr, "失败", "连接ROS Master失败！请检查你的网络或连接字符串！", QMessageBox::Yes, QMessageBox::Yes);
            ui->label_robot_staue_img->setPixmap(QPixmap::fromImage(QImage("://images/offline.png")));
            ui->label_statue_text->setStyleSheet("color:red;");
            ui->label_statue_text->setText("离线");
            ui->tab_manager->setTabEnabled(1,false);
            ui->tabWidget->setTabEnabled(1,false);
            ui->groupBox_3->setEnabled(false);
            return ;
        }
    }
    //如果不使用环境变量
    else
    {
        if ( !qnode.init(ui->line_edit_master->text().toStdString(), ui->line_edit_host->text().toStdString()) )
        {
            QMessageBox::warning(nullptr, "失败", "连接ROS Master失败！请检查你的网络或连接字符串！", QMessageBox::Yes, QMessageBox::Yes);
            ui->label_robot_staue_img->setPixmap(QPixmap::fromImage(QImage("://images/offline.png")));
            ui->label_statue_text->setStyleSheet("color:red;");
            ui->label_statue_text->setText("离线");
            ui->tab_manager->setTabEnabled(1,false);
            ui->tabWidget->setTabEnabled(1,false);
            ui->groupBox_3->setEnabled(false);
            //showNoMasterMessage();
            return ;
        }
        else
        {
            ui->line_edit_master->setReadOnly(true);
            ui->line_edit_host->setReadOnly(true);
            ui->line_edit_topic->setReadOnly(true);
        }
    }
    ui->tab_manager->setTabEnabled(1,true);
    ui->tabWidget->setTabEnabled(1,true);
    ui->groupBox_3->setEnabled(true);
    //初始化rviz
    initRviz();
    
    
    ui->button_connect->setEnabled(false);
    ui->label_robot_staue_img->setPixmap(QPixmap::fromImage(QImage("://images/online.png")));
    ui->label_statue_text->setStyleSheet("color:green;");
    ui->label_statue_text->setText("在线");
    ui->button_disconnect->setEnabled(true);
    //初始化视频订阅的显示
    initVideos();
    //显示话题列表
    initTopicList();
}

/// \brief 断开ROS
void cyrobot_monitor::MainWindow::on_button_disconnect_clicked()
{
    qnode.disinit();
    map_rviz_->quit();
    delete map_rviz_;
    map_rviz_ = nullptr;
    ui->pushButton_add_topic->setEnabled(false);
    ui->pushButton_remove_topic->setEnabled(false);
    ui->pushButton_rename_topic->setEnabled(false);
    ui->pushButton_rvizReadDisplaySet->setEnabled(false);
    ui->pushButton_rvizSaveDisplaySet->setEnabled(false);
    ui->label_rvizShow->show();
    ui->button_disconnect->setEnabled(false);
    ui->button_connect->setEnabled(true);
    
}

void cyrobot_monitor::MainWindow::on_pushButton_rvizReadDisplaySet_clicked()
{
    if (map_rviz_ == nullptr)
    {
        return;
    }
    QString path = QFileDialog::getOpenFileName(this, "导入 RVIZ Display 配置", "/home/" + getUserName() + "/", "YAML(*.yaml);;ALL(*.*)");
    if (!path.isEmpty())
    {
        map_rviz_->ReadDisplaySet(path);
    }
}

void cyrobot_monitor::MainWindow::on_pushButton_rvizSaveDisplaySet_clicked()
{
    if (map_rviz_ == nullptr)
    {
        return;
    }
    QString path = QFileDialog::getSaveFileName(this, "导出 RVIZ Display 配置", "/home/" + getUserName() + "/", "YAML(*.yaml)");
    
    if (!path.isEmpty())
    {
        if (path.section('/', -1, -1).indexOf('.') < 0)
        {
            path = path + ".yaml";
        }
        map_rviz_->OutDisplaySet(path);
    }
}

QString cyrobot_monitor::MainWindow::getUserName()
{
    QString userPath = QStandardPaths::writableLocation(QStandardPaths::HomeLocation);
    QString userName = userPath.section("/", -1, -1);
    return userName;
}

void cyrobot_monitor::MainWindow::clean_all_msgs(){
  ui->label_video0->clear();
  ui->label_video1->clear();
  ui->label_video2->clear();
  ui->label_video3->clear();
  ui->label_video5->clear();
  ui->label_video6->clear();
  ui->topic_listWidget->clear();
  ui->progressBar->setValue(0);
  ui->label_power->setText(QString::number(0.0));
}

