#-------------------------------------------------
#
# Project created by QtCreator 2016-03-23T07:36:27
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = SendPosition
TEMPLATE = app


SOURCES += main.cpp\
        mainwindow.cpp \
    mainapplication.cpp \
    vicon.cpp

HEADERS  += mainwindow.h \
    mainapplication.h \
    vicon.h

FORMS    += mainwindow.ui

QMAKE_CXXFLAGS += -std=c++11

INCLUDEPATH += "/home/mrfish/SourceCode/gnuplot-iostream/"
INCLUDEPATH += "/home/robot/SourceCode/gnuplot-iostream"
INCLUDEPATH += "/opt/ros/indigo/include/"
INCLUDEPATH += "/home/mrfish/catkin_ws/src/px-ros-pkg/px_comm/devel/include/"
INCLUDEPATH += "/home/robot/catkin_ws/devel/include/"
INCLUDEPATH += "/home/mrfish/catkin_ws/devel/include/"

LIBS += -L"/opt/ros/indigo/lib/" -lroscpp -lrosconsole -lroscpp_serialization  -lrostime -lxmlrpcpp -lcpp_common -lboost_system -lrosconsole_backend_interface
LIBS += -lrosconsole_log4cxx -lboost_iostreams

