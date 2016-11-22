#include "mainwindow.h"
#include <QApplication>
#include "ros/ros.h"
#include <QTimer>
#include "mainapplication.h"
#include "vicon.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "sendPosition");
    QApplication a(argc, argv);
    MainWindow w;
    w.setController( new MainApplication());
    w.setViconController(new Vicon());
    w.show();

    return a.exec();
}
