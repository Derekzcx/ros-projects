#include "../include/cyrobot_monitor/settings.h"
#include "ui_settings.h"

Settings::Settings(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::Settings)
{
    ui->setupUi(this);
    QSettings video_topic_setting("video_topic","cyrobot_monitor");
    QStringList names=video_topic_setting.value("names").toStringList();
    QStringList topics=video_topic_setting.value("topics").toStringList();

    QStringList topics_2 = video_topic_setting.value("topics_2").toStringList(); // 识别

    if(names.size()==6)
    {
        ui->video0_name_set->setText(names[0]);
        ui->video0_name_set_2->setText(names[1]);
        ui->video0_name_set_3->setText(names[2]);
        ui->video0_name_set_4->setText(names[3]);

        ui->video0_name_set_5->setText(names[4]);
        ui->video0_name_set_6->setText(names[5]);
    }
    if(topics.size()==6)
    {
        ui->video0_topic_set->setText(topics[0]);
        ui->video0_topic_set_2->setText(topics[1]);
        ui->video0_topic_set_3->setText(topics[2]);
        ui->video0_topic_set_4->setText(topics[3]);

        ui->video0_topic_set_5->setText(topics[4]);
        ui->video0_topic_set_6->setText(topics[5]);
    }

    if(topics_2.size()==6){ //
      ui->video0_topic2_set->setText(topics_2[0]);
      ui->video1_topic2_set->setText(topics_2[1]);
      ui->video2_topic2_set->setText(topics_2[2]);
      ui->video3_topic2_set->setText(topics_2[3]);
      ui->video5_topic2_set->setText(topics_2[4]);
      ui->video6_topic2_set->setText(topics_2[5]);
    }

    QSettings main_setting("topic_setting","cyrobot_monitor");
    ui->lineEdit_odm->setText(main_setting.value("topic_odom","raw_odom").toString());
    ui->lineEdit_power->setText(main_setting.value("topic_power","power").toString());
    ui->lineEdit_power_min->setText(main_setting.value("power_min","10").toString());
    ui->lineEdit_power_max->setText(main_setting.value("power_max","12").toString());
//    ui->lineEdit_odom->setText(main_setting.value("odom_topic","amcl_pose").toString());
    connect(ui->pushButton,SIGNAL(clicked()),this,SLOT(slot_ok_btn_click()));
    connect(ui->pushButton_2,SIGNAL(clicked()),this,SLOT(slot_cancel_btn_click()));
}
void Settings::slot_ok_btn_click()
{
    QSettings main_setting("topic_setting","cyrobot_monitor");
    main_setting.setValue("topic_odom",ui->lineEdit_odm->text());
    main_setting.setValue("topic_power",ui->lineEdit_power->text());
    main_setting.setValue("power_min",ui->lineEdit_power_min->text());
    main_setting.setValue("power_max",ui->lineEdit_power_max->text());
//    main_setting.setValue("odom_topic",ui->lineEdit_odom->text());

    QSettings video_topic_setting("video_topic","cyrobot_monitor");
    QStringList name_data;
    QStringList topic_data;
    QStringList format_data;
    /*新增*/
    QStringList topic2_data; // 用于存储识别图像的话题名


    name_data.append(ui->video0_name_set->text());
    name_data.append(ui->video0_name_set_2->text());
    name_data.append(ui->video0_name_set_3->text());
    name_data.append(ui->video0_name_set_4->text());
    /*新增**/
    name_data.append(ui->video0_name_set_5->text());
    name_data.append(ui->video0_name_set_6->text());


    topic_data.append(ui->video0_topic_set->text());
    topic_data.append(ui->video0_topic_set_2->text());
    topic_data.append(ui->video0_topic_set_3->text());
    topic_data.append(ui->video0_topic_set_4->text());
    /*新增*/
    topic_data.append(ui->video0_topic_set_5->text());
    topic_data.append(ui->video0_topic_set_6->text());

    /*新增识别图像的setting 设定*/
    topic2_data.append(ui->video0_topic2_set->text());
    topic2_data.append(ui->video1_topic2_set->text());
    topic2_data.append(ui->video2_topic2_set->text());
    topic2_data.append(ui->video3_topic2_set->text());
    topic2_data.append(ui->video5_topic2_set->text());
    topic2_data.append(ui->video6_topic2_set->text());


    video_topic_setting.setValue("names",name_data);
    video_topic_setting.setValue("topics",topic_data);

    video_topic_setting.setValue("topics_2", topic2_data);

    QMessageBox::critical(NULL, "保存成功！", "保存成功，部分功能需重启后生效！", QMessageBox::Yes | QMessageBox::No, QMessageBox::Yes);
    this->close();
}
void Settings::slot_cancel_btn_click()
{
    this->close();
}
Settings::~Settings()
{
    delete ui;
}
