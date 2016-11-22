#ifndef MAINAPPLICATION_H
#define MAINAPPLICATION_H

#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <iostream>
#include <vector>
#include <cmath>
#include <boost/tuple/tuple.hpp>
#include "gnuplot-iostream.h"
#include "geometry_msgs/PoseStamped.h"
#include "ros/ros.h"
#include "gazebo_msgs/LinkState.h"
#include "gazebo_msgs/LinkStates.h"
#include "gazebo_msgs/ModelState.h"
#include "gazebo_msgs/ModelStates.h"
#include <tf/transform_datatypes.h>
#include <boost/foreach.hpp>
#include "px_comm/OpticalFlow.h"
#include <QTimer>


class MainApplication
{
public:
    std::time_t now;
    int counter;
    //ros::Publisher pub_vision, pub_attitude, pub_pos, pub_mocap, pub_gt_orientation_camera;
    bool sendPosition, sendSetpointPosition, sendMocapPosition, running;
    double posX, posY, posZ, dt, t, t_prev;
    double posX_flow, posY_flow;
    char const *colorActive = "background-color: rgb(0, 255, 0);";
    //double deltaPosX, deltaPosY, deltaPosZ;
    //double posX_flow=0, posY_flow=0;
public:
    MainApplication();

    void init();

    void publish_FakeVisionPosition(QString xPosString, QString yPosString, QString zPosString, QString yawPosString);

    void publish_SetpointPosition(QString xSetpointPos, QString ySetpointPos, QString zSetpointPos, QString xSetpointQuad, QString ySetpointQuad, QString zSetpointQuad, QString wSetpointQuad);

    void publish_MocapPosition(QString xSetpointPos, QString ySetpointPos, QString zSetpointPos);

    void stop_disarmSequence();

    void publish_kill(bool state);

    void publish_startSequence(bool start);

    void test();

    void enablePx4Flow(bool enable);

    void enableTf2VisionEstimate(bool enable);

    void enableGazebo2VisionEstimate(bool enable);

    void enableOrbSlam2VisionEstimate(bool enable);

    void publish_RCchannels();

};

#endif // MAINAPPLICATION_H
