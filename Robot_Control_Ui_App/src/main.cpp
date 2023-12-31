/**
 * @file /src/main.cpp
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
    QApplication app(argc, argv);
    cyrobot_monitor::MainWindow w(argc,argv);
    w.show();
    app.connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));
    int result = app.exec();

	return result;
}
