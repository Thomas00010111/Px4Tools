#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <iostream>
#include <vector>
#include <cmath>
#include <boost/tuple/tuple.hpp>
#include "gnuplot-iostream.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TransformStamped.h"
#include "ros/ros.h"
#include "gazebo_msgs/LinkState.h"
#include "gazebo_msgs/LinkStates.h"
#include "gazebo_msgs/ModelState.h"
#include "gazebo_msgs/ModelStates.h"
#include <tf/transform_datatypes.h>
#include <boost/foreach.hpp>
#include "px_comm/OpticalFlow.h"
#include <QTimer>
#include "mainapplication.h"
#include "visualization_msgs/Marker.h"
#include "tf2_msgs/TFMessage.h"
#include "tf/tfMessage.h"

//#include "std_msgs/String.h"

double posX=0, posY=0, posZ=0, dt=0, t=0, t_prev=0;
double posX_flow=0, posY_flow=0;

double deltaPosX=0, deltaPosY=0, deltaPosZ=0;
char const *colorActive = "background-color: rgb(0, 255, 0);";

MainApplication* mainApp;

std::time_t now;
extern int posEstMsg_seq, initalize;

int counter = 0;
ros::Publisher pub_attitude, pub_pos, pub_mocap, pub_gt_orientation_camera, pub_vision, pub_RcOverride;
ros::Subscriber sub, sub_map, sub_opticalFlow;

bool sendPosition = false, sendSetpointPosition = false, sendMocapPosition = false, running = true, requestViconPosition = false, enableOrb2Slam=false, rcChannels=false, kill=false;
//bool publish_RC_channels = false;


MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    mainApp = new MainApplication();
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::setController(MainApplication *controller)
{
    this->controller = controller;
    this->controller->init();
}

void MainWindow::setViconController(Vicon *controller)
{
    this->vicon_controller = controller;
    this->vicon_controller->init();
}

// /mavros/vision_pose/pose
void MainWindow::on_SendPositionBtn_toggled(bool checked)
{
    setColorButton(ui->SendPositionBtn, checked);
    sendPosition = checked;
}

void MainWindow::on_stopBtn_clicked()
{
    sendPosition = false;
}

void MainWindow::setColorButton(QPushButton *button, bool checked)
{
    if(checked)
        button->setStyleSheet(QString::fromUtf8(colorActive));
    else
    {
        button->setStyleSheet(QString::fromUtf8("some String to set grey again"));
    }
}


void MainWindow::on_sendMocapBtn_toggled(bool checked)
{
    setColorButton(ui->sendMocapBtn, checked);
    //std::cout << color.red() << color.green() << color.blue()<<std::endl;
    sendMocapPosition = checked;
}


//setpoint
void MainWindow::on_sendPositionBtn_toggled(bool checked)
{
    setColorButton(ui->sendPositionBtn, checked);
    sendSetpointPosition = checked;
}

void MainWindow::closeEvent (QCloseEvent *event)
{
    std::cout << event << std::endl;
    closeMainForm();
}

void MainWindow::closeMainForm()
{
    running = false;
    close();
    QApplication::quit();
}

void MainWindow::on_ExitBtn_clicked()
{
    closeMainForm();
}

void MainWindow::on_startloop_clicked()
{
    ui->startloop->setStyleSheet(QString::fromUtf8(colorActive));
    ros::Rate rate(10);
    while(ros::ok() && running)
    {
        if(sendPosition)
            controller->publish_FakeVisionPosition(ui->xTargetPos->text(), ui->yTargetPos->text(), ui->zTargetPos->text(), ui->yawTargetPos->text());

        if(sendSetpointPosition)
            controller->publish_SetpointPosition(ui->xSetpointPos->text(), ui->ySetpointPos->text(), ui->zSetpointPos->text(), ui->setpoint_x->text(), ui->setpoint_y->text(), ui->setpoint_z->text(), ui->setpoint_w->text());

        if(sendMocapPosition)
            controller->publish_MocapPosition(ui->xMocapPos->text(), ui->yMocapPos->text(), ui->zMocapPos->text());

        if(requestViconPosition)
            vicon_controller->request_viconData(ui->viconSubjectName->text(),ui->viconSegmentName->text(), ui->viconNumberMeasurements->value());

        if(ui->publishRcChCheckBox->isChecked())
        {
            controller->publish_startSequence(rcChannels);
            controller->publish_kill(kill);
            controller->publish_RCchannels();
        }

        deltaPosX = ui->deltaPosX->text().toDouble();
        deltaPosY = ui->deltaPosY->text().toDouble();
        deltaPosZ = ui->deltaPosZ->text().toDouble();

        rate.sleep();
        ros::spinOnce();
        QCoreApplication::processEvents();
    }
    ros::shutdown();
}

void MainWindow::on_map2PositionBtn_toggled(bool checked)
{
    setColorButton(ui->map2PositionBtn, checked);
    controller-> enableTf2VisionEstimate(checked);
}


void MainWindow::on_pushButton_toggled(bool checked)
{
    setColorButton(ui->pushButton, checked);
    controller->enablePx4Flow(checked);
}

void MainWindow::on_btnGazebo2VisionEstimate_toggled(bool checked)
{
    setColorButton(ui->btnGazebo2VisionEstimate, checked);
    controller->enableGazebo2VisionEstimate(checked);
}

void MainWindow::on_vicon2VisionBtn_toggled(bool checked)
{
    setColorButton(ui->vicon2VisionBtn, checked);
    requestViconPosition = checked;
}

void MainWindow::on_setIpViconBtn_clicked()
{
    ;
}


void MainWindow::on_orbSlam2Btn_toggled(bool checked)
{
    setColorButton(ui->orbSlam2Btn, checked);
    controller->enableOrbSlam2VisionEstimate(checked);
}

void MainWindow::on_StartSequenceButton_toggled(bool checked)
{
    rcChannels = checked;
    setColorButton(ui->StartSequenceButton, checked);
    if(!checked)
    {
        controller->stop_disarmSequence();
    }
}


void MainWindow::on_killButton_toggled(bool checked)
{
    kill = checked;
    setColorButton(ui->killButton, checked);
    ui->StartSequenceButton->setChecked(false);
}


//When deleting leads to some difficulties with qt-designer
void MainWindow::on_sendMocapBtn_clicked()
{
}

void MainWindow::on_stopSendingPosition_clicked()
{
}

void retrieveViconDataCallback(const geometry_msgs::TransformStampedConstPtr &msg)
{
    std::cout<<"Vicon Callback  x: " << msg->transform.translation.x << "    y: " << msg->transform.translation.y << std::endl;
    // Publish to vision estimation node
        geometry_msgs::PoseStamped posEstMsg;
        posEstMsg.header.stamp = ros::Time::now();
        posEstMsg.header.seq=posEstMsg_seq++;
        posEstMsg.pose.position.x = msg->transform.translation.x;
        posEstMsg.pose.position.y = msg->transform.translation.y;
        posEstMsg.pose.position.z = msg->transform.translation.z;
        posEstMsg.pose.orientation.x = msg->transform.rotation.x;
        posEstMsg.pose.orientation.y = msg->transform.rotation.y;
        posEstMsg.pose.orientation.z = msg->transform.rotation.z;
        posEstMsg.pose.orientation.w = msg->transform.rotation.w;
        std::cout <<  "publishing /tf (Camera) on /mavros/vision_pose/pose" << std::endl;
        pub_vision.publish(posEstMsg);
}

void retrieveMapCallback(const tf2_msgs::TFMessageConstPtr &msg)
{
    //std::cout<<"MAP ns: " << msg->ns << "   received x: " << msg->pose.position.x << "   y: " << msg->pose.position.y << "    z: " << msg->pose.position.z << std::endl;

    // Publish to vision estimation node
    for(const geometry_msgs::TransformStamped &m : msg->transforms)
    {
        if(m.child_frame_id == "ORB_SLAM/Camera")
        {
            std::cout << m << std::endl;
            geometry_msgs::PoseStamped posEstMsg;
            posEstMsg.header.stamp = ros::Time::now();
            posEstMsg.header.seq=posEstMsg_seq++;
            posEstMsg.pose.position.x = m.transform.translation.x;
            posEstMsg.pose.position.y = m.transform.translation.y;
            posEstMsg.pose.position.z = m.transform.translation.z;
            posEstMsg.pose.orientation.x = m.transform.rotation.x;
            posEstMsg.pose.orientation.y = m.transform.rotation.y;
            posEstMsg.pose.orientation.z = m.transform.rotation.z;
            posEstMsg.pose.orientation.w = m.transform.rotation.w;
            std::cout <<  "publishing /tf (Camera) on /mavros/vision_pose/pose" << std::endl;
            pub_vision.publish(posEstMsg);
         }
    }
}

void retrieveOrbSlam2Callback(const tf2_msgs::TFMessageConstPtr &msg)
{
    //std::cout<<"MAP ns: " << msg->ns << "   received x: " << msg->pose.position.x << "   y: " << msg->pose.position.y << "    z: " << msg->pose.position.z << std::endl;
    std::cout << "retrieveOrbSlam2Callback - 1" <<std::endl;

    // Publish to vision estimation node
    for(const geometry_msgs::TransformStamped &m : msg->transforms)
    {
        std::cout << "retrieveOrbSlam2Callback - 2: " << m.child_frame_id << std::endl;
        if(m.child_frame_id == "pose")
        {
            std::cout << m << std::endl;
            geometry_msgs::PoseStamped posEstMsg;
            posEstMsg.header.stamp = ros::Time::now();
            posEstMsg.header.seq=posEstMsg_seq++;
            posEstMsg.pose.position.x = m.transform.translation.x;
            posEstMsg.pose.position.y = m.transform.translation.y;
            posEstMsg.pose.position.z = m.transform.translation.z;
            posEstMsg.pose.orientation.x = m.transform.rotation.x;
            posEstMsg.pose.orientation.y = m.transform.rotation.y;
            posEstMsg.pose.orientation.z = m.transform.rotation.z;
            posEstMsg.pose.orientation.w = m.transform.rotation.w;
            std::cout <<  "publishing /tf (Camera) on /mavros/vision_pose/pose" << std::endl;
            pub_vision.publish(posEstMsg);
         }
    }
}


void opticalFlowCallback(const px_comm::OpticalFlowConstPtr& msg)
{
    double nsec = ((double) msg->header.stamp.nsec)/1000000000.0;
    assert (nsec < 1);
    t = (double) msg->header.stamp.sec + nsec;

    if(initalize)
    {
        t_prev = t;
        initalize = 0;
        return;
    }

    if((t-t_prev)<=0)
    {
        //do not change dt
        now = std::time(0);   // get time now
        std::cout <<  "[" << ctime(&now) << "] ERROR   t=" << t << "   t_prev=" << t_prev << "\n";
    }
    else
    {
        dt = t-t_prev;
    }

    //position
    posX_flow += msg->velocity_x * dt;
    posY_flow += msg->velocity_y * dt ;
    t_prev=t;

    // Publish to vision estimation node
    geometry_msgs::PoseStamped posEstMsg;
    posEstMsg.header.stamp = ros::Time::now();
    posEstMsg.header.seq=posEstMsg_seq++;
    posEstMsg.pose.position.x = posX_flow;
    posEstMsg.pose.position.y = posY_flow;
    posEstMsg.pose.position.z = msg->ground_distance;

    now = std::time(0);   // get time now
    std::cout <<  "publishing optical flow (px4) on /mavros/vision_pose/pose" << std::endl;
    pub_vision.publish(posEstMsg);
}

void gazeboModelStateCallback(const gazebo_msgs::ModelStates::ConstPtr &msg)
{
    std::string modelName = "iris";
    for (uint i = 0; i < msg->name.size (); i++)
    {
        if (modelName == msg->name[i].substr(0,modelName.length()))
        {
            std::cout << msg->name[i] << std::endl;
            std::cout << msg->pose[i] << std::endl;

            geometry_msgs::PoseStamped gt_camera_msg;
            gt_camera_msg.header.frame_id="world";

            gt_camera_msg.header.stamp = ros::Time::now();
            gt_camera_msg.header.seq=posEstMsg_seq++;
            //    std::cout << xPosString.toDouble() <<"; " <<yPosString.toDouble()<<"; "  << zPosString.toDouble() << std::endl;
            gt_camera_msg.pose.position.x = msg->pose[i].position.x;
            gt_camera_msg.pose.position.y = msg->pose[i].position.y;
            gt_camera_msg.pose.position.z = msg->pose[i].position.z;

            gt_camera_msg.pose.orientation.x = msg->pose[i].orientation.x;
            gt_camera_msg.pose.orientation.y = msg->pose[i].orientation.y;
            gt_camera_msg.pose.orientation.z = msg->pose[i].orientation.z;
            gt_camera_msg.pose.orientation.w = msg->pose[i].orientation.w;

            std::cout<<"gt_camera_msg"<<std::endl;
            pub_gt_orientation_camera.publish(gt_camera_msg);

            //change vision position by adding x,y,z to ground truth camera position
            gt_camera_msg.pose.position.x = deltaPosX + gt_camera_msg.pose.position.x;
            gt_camera_msg.pose.position.y = deltaPosY + gt_camera_msg.pose.position.y;
            gt_camera_msg.pose.position.z = deltaPosZ + gt_camera_msg.pose.position.z;
            std::cout <<  "publishing gt_camera_msg on /mavros/vision_pose/pose" << std::endl;
            pub_vision.publish(gt_camera_msg);
        }
    }
}




/*


//void retrieveMapCallback(const visualization_msgs::MarkerConstPtr &msg)
//{
//    //std::cout<<"MAP ns: " << msg->ns << "   received x: " << msg->pose.position.x << "   y: " << msg->pose.position.y << "    z: " << msg->pose.position.z << std::endl;

//    // Publish to vision estimation node
//    if(msg->ns == "Camera")
//    {
//        std::cout<<"MAP ns: " << msg->ns << "MAP type: " << msg->type << std::endl;
//        geometry_msgs::PoseStamped posEstMsg;
//        posEstMsg.header.stamp = ros::Time::now();
//        posEstMsg.header.seq=posEstMsg_seq++;
//        posEstMsg.pose.position.x = msg->pose.position.x;
//        posEstMsg.pose.position.y = msg->pose.position.y;
//        posEstMsg.pose.position.z = msg->pose.position.y;
//        posEstMsg.pose.orientation.x = msg->pose.orientation.x;
//        posEstMsg.pose.orientation.y = msg->pose.orientation.y;
//        posEstMsg.pose.orientation.z = msg->pose.orientation.y;
//        std::cout<<"MAP received x: " << msg->pose.position.x << "   y: " << msg->pose.position.y << "    z: " << msg->pose.position.z << std::endl;
//        std::cout<<"MAP received orient: " << msg->pose.orientation.x << "   y: " << msg->pose.orientation.y << "    z: " << msg->pose.orientation.z << "    w: " << msg->pose.orientation.w<< std::endl;
//        std::cout<<"MAP received scale: " << msg->scale.x << "   y: " << msg->scale.x  << "    z: " << msg->scale.x  << std::endl;

//        for(const geometry_msgs::Point &p : msg->points)
//        {
//            std::cout << "p: " << p << std::endl;
//        }

//        //std::cout<<"MAP received points: " << msg->points << std::endl;

//        now = std::time(0);   // get time now
//        pub_vision.publish(posEstMsg);
//    }
//}
void init()
{
    ros::NodeHandle n;
    pub_vision = n.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose",100);
    pub_mocap = n.advertise<geometry_msgs::PoseStamped>("/mavros/mocap/pose",100);
    pub_attitude = n.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_attitude/attitude",100);
    pub_pos = n.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local",100);
    pub_gt_orientation_camera = n.advertise<geometry_msgs::PoseStamped>("/gt_orientation_camera",100);
    sub = n.subscribe("/gazebo/model_states", 100, chatterCallback);
}

void publish_VisionPosition(QString xPosString, QString yPosString, QString zPosString)
{
    // Publish to vision estimation node
    geometry_msgs::PoseStamped posEstMsg;

        posEstMsg.header.stamp = ros::Time::now();
        posEstMsg.header.seq=posEstMsg_seq++;
    //    std::cout << xPosString.toDouble() <<"; " <<yPosString.toDouble()<<"; "  << zPosString.toDouble() << std::endl;
        posEstMsg.pose.position.x = xPosString.toDouble();
        posEstMsg.pose.position.y = yPosString.toDouble();
        posEstMsg.pose.position.z = zPosString.toDouble();

    //    posEstMsg.pose.orientation.x = 0.0;
    //    posEstMsg.pose.orientation.y = 0.0;
    //    posEstMsg.pose.orientation.z = 0.0;
        posEstMsg.pose.orientation.w = 0.0;

        //now = std::time(0);   // get time now
        //name_log_file << "[" << ctime(&now) << "]  dt=" << dt <<"    position.x= " << posX << "   position.y= " << posY << "    position.z = "<< msg->ground_distance << "\n"
        pub_vision.publish(posEstMsg);
        std::cout << counter++ << "   sending vision estimate" << std::endl;
}

void publish_SetpointPosition(QString xSetpointPos, QString ySetpointPos, QString zSetpointPos)
{

    // Publish to vision estimation node
    geometry_msgs::PoseStamped setpointPosMsg;
    setpointPosMsg.header.frame_id="world";

    setpointPosMsg.header.stamp = ros::Time::now();
    setpointPosMsg.header.seq=posEstMsg_seq++;
//    std::cout << xPosString.toDouble() <<"; " <<yPosString.toDouble()<<"; "  << zPosString.toDouble() << std::endl;
    setpointPosMsg.pose.position.x = xSetpointPos.toDouble();
    setpointPosMsg.pose.position.y = ySetpointPos.toDouble();
    setpointPosMsg.pose.position.z = zSetpointPos.toDouble();

    //double yaw_degrees = 0;  // North
    //double yaw = radians(yaw_degrees);
//    tf::Quaternion quaternion;
//    quaternion.setEuler(0,0,0);
    setpointPosMsg.pose.orientation.x = 0.0;
    setpointPosMsg.pose.orientation.y = 0.0;
    setpointPosMsg.pose.orientation.z = 0.0;
    setpointPosMsg.pose.orientation.w = 0.0;

    //now = std::time(0);   // get time now
    //name_log_file << "[" << ctime(&now) << "]  dt=" << dt <<"    position.x= " << posX << "   position.y= " << posY << "    position.z = "<< msg->ground_distance << "\n";
    pub_pos.publish(setpointPosMsg);
}

void publish_MocapPosition(QString xSetpointPos, QString ySetpointPos, QString zSetpointPos)
{

    // Publish to vision estimation node
    geometry_msgs::PoseStamped mocapPosMsg;
    mocapPosMsg.header.frame_id="world";

    mocapPosMsg.header.stamp = ros::Time::now();
    mocapPosMsg.header.seq=posEstMsg_seq++;
    //std::cout << xPosString.toDouble() <<"; " <<yPosString.toDouble()<<"; "  << zPosString.toDouble() << std::endl;
    mocapPosMsg.pose.position.x = xSetpointPos.toDouble();
    mocapPosMsg.pose.position.y = ySetpointPos.toDouble();
    mocapPosMsg.pose.position.z = zSetpointPos.toDouble();

    //double yaw_degrees = 0;  // North
    //double yaw = radians(yaw_degrees);
    //tf::Quaternion quaternion;
    //quaternion.setEuler(0,0,0);
    //quaternion = quaternion_from_euler(0, 0, yaw);
    //setpointPosMsg.pose.orientation = Quaternion(*quaternion);
    //setpointPosMsg.pose.orientation = quaternion;
    mocapPosMsg.pose.orientation.x = 0.0;
    mocapPosMsg.pose.orientation.y = 0.0;
    mocapPosMsg.pose.orientation.z = 0.0;
    mocapPosMsg.pose.orientation.w = 0.0;

    std::cout<<"publishMocap"<<std::endl;
    pub_mocap.publish(mocapPosMsg);
}


void test()
{
    std::cout<<"TIMEOUT!!"<<std::endl;
}

void chatterCallback(const gazebo_msgs::ModelStates::ConstPtr &msg)
{
    std::string modelName = "iris";
    for (uint i = 0; i < msg->name.size (); i++)
    {
        if (modelName == msg->name[i].substr(0,modelName.length()))
        {
            std::cout <<  "publishing gt_camera_msg" << std::endl;
            std::cout << msg->name[i] << std::endl;
            std::cout << msg->pose[i] << std::endl;

            geometry_msgs::PoseStamped gt_camera_msg;
            gt_camera_msg.header.frame_id="world";

            gt_camera_msg.header.stamp = ros::Time::now();
            gt_camera_msg.header.seq=posEstMsg_seq++;
            //    std::cout << xPosString.toDouble() <<"; " <<yPosString.toDouble()<<"; "  << zPosString.toDouble() << std::endl;
            gt_camera_msg.pose.position.x = msg->pose[i].position.x;
            gt_camera_msg.pose.position.y = msg->pose[i].position.y;
            gt_camera_msg.pose.position.z = msg->pose[i].position.z;

            gt_camera_msg.pose.orientation.x = msg->pose[i].orientation.x;
            gt_camera_msg.pose.orientation.y = msg->pose[i].orientation.y;
            gt_camera_msg.pose.orientation.z = msg->pose[i].orientation.z;
            gt_camera_msg.pose.orientation.w = msg->pose[i].orientation.w;

            std::cout<<"gt_camera_msg"<<std::endl;
            pub_gt_orientation_camera.publish(gt_camera_msg);

            //change vision position by adding x,y,z to ground truth camera position
            gt_camera_msg.pose.position.x = deltaPosX + gt_camera_msg.pose.position.x;
            gt_camera_msg.pose.position.y = deltaPosY + gt_camera_msg.pose.position.y;
            gt_camera_msg.pose.position.z = deltaPosZ + gt_camera_msg.pose.position.z;
            pub_vision.publish(gt_camera_msg);
        }
    }
}


//---------------------------------------------------------


*/


