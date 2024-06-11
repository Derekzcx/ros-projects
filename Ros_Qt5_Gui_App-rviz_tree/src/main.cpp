/**
 * @file /src/main.cpp
 *
 * @brief Qt based gui.
 *
 * @date November 2010
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QApplication>
#include "../include/cyrobot_monitor/main_window.hpp"


/*****************************************************************************
** Main
*****************************************************************************/

int main(int argc, char **argv) {

    /*********************
    ** Qt
    **********************/
    QApplication app(argc, argv);                                       // Qt 界面程序对象
    cyrobot_monitor::MainWindow w(argc,argv);                           // 主界面对象
    w.show();                                                           // 主界面启动
    app.connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));   // 信号与槽 ，推出信号绑定处理
    int result = app.exec();

	return result;
}
