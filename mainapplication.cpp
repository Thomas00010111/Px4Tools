#include "mainapplication.h"
#include <iostream>
#include "px_comm/OpticalFlow.h"
#include "ros/ros.h"
#include "visualization_msgs/Marker.h"
#include "tf2_msgs/TFMessage.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf/tfMessage.h"
#include "mavros_msgs/OverrideRCIn.h"


//double deltaPosX=0, deltaPosY=0, deltaPosZ=0;
extern void gazeboModelStateCallback(const gazebo_msgs::ModelStates::ConstPtr &msg);
extern void opticalFlowCallback(const  px_comm::OpticalFlowConstPtr& msg);
extern void retrieveMapCallback(const tf2_msgs::TFMessageConstPtr &msg);
extern void retrieveOrbSlam2Callback(const tf2_msgs::TFMessageConstPtr &msg);

extern ros::Subscriber sub, sub_map, sub_opticalFlow;
extern ros::Publisher pub_attitude, pub_pos, pub_mocap, pub_gt_orientation_camera, pub_vision, pub_RcOverride;
extern tf2::Transform * origin_tf;

mavros_msgs::OverrideRCIn channels;

int initalize = 1;
int posEstMsg_seq=1;
int count = 0;


MainApplication::MainApplication()
{
    counter = 0;
    sendPosition = false;
    sendSetpointPosition = false;
    sendMocapPosition = false;
    running = true;
    posX=0;
    posY=0;
    posZ=0;
    dt=0;
    t=0;
    t_prev=0;
    posX_flow=0;
    posY_flow=0;
    channels.channels = {1500, 1500, 1102, 1500, 2067, 960, 1000, 1514};
}

void MainApplication::init()
{
    ros::NodeHandle n;
    pub_vision = n.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose",100);
    pub_mocap = n.advertise<geometry_msgs::PoseStamped>("/mavros/mocap/pose",100);
    pub_attitude = n.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_attitude/attitude",100);
    pub_pos = n.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local",100);
    pub_gt_orientation_camera = n.advertise<geometry_msgs::PoseStamped>("/gt_orientation_camera",100);
    pub_RcOverride = n.advertise<mavros_msgs::OverrideRCIn>("/mavros/rc/override",100);
}

void MainApplication::publish_FakeVisionPosition(QString xPosString, QString yPosString, QString zPosString, QString yawPosString)
{
    // Publish to vision estimation node
    // ENU, right handed
    // x: east (forward), y: noth (left), z: up (up)
    // see https://github.com/mavlink/mavros/issues/523
    geometry_msgs::PoseStamped posEstMsg;
    posEstMsg.header.stamp = ros::Time::now();
    posEstMsg.header.seq=posEstMsg_seq++;
    posEstMsg.pose.position.x = xPosString.toDouble();
    posEstMsg.pose.position.y = yPosString.toDouble();
    posEstMsg.pose.position.z = zPosString.toDouble();

    //Roation about z-axis
    tf2::Quaternion q;
    q.setEulerZYX(yawPosString.toDouble(), 0, 0);

    posEstMsg.pose.orientation.x = q.getX();
    posEstMsg.pose.orientation.y = q.getY();
    posEstMsg.pose.orientation.z = q.getZ();
    posEstMsg.pose.orientation.w = q.getW();

    //now = std::time(0);   // get time now
    //name_log_file << "[" << ctime(&now) << "]  dt=" << dt <<"    position.x= " << posX << "   position.y= " << posY << "    position.z = "<< msg->ground_distance << "\n"
    pub_vision.publish(posEstMsg);
    std::cout << counter++ << "   sending fake vision estimate" << std::endl;
}

void MainApplication::publish_SetpointPosition(QString xSetpointPos, QString ySetpointPos, QString zSetpointPos, QString xSetpointQuad, QString ySetpointQuad, QString zSetpointQuad, QString wSetpointQuad)
{
    // see https://github.com/mavlink/mavros/issues/523
    // Publish to vision estimation node
    geometry_msgs::PoseStamped setpointPosMsg;
    setpointPosMsg.header.frame_id="world";

    setpointPosMsg.header.stamp = ros::Time::now();
    setpointPosMsg.header.seq=posEstMsg_seq++;
//    std::cout << xPosString.toDouble() <<"; " <<yPosString.toDouble()<<"; "  << zPosString.toDouble() << std::endl;
    setpointPosMsg.pose.position.x = xSetpointPos.toDouble();
    setpointPosMsg.pose.position.y = ySetpointPos.toDouble();
    setpointPosMsg.pose.position.z = zSetpointPos.toDouble();

    //Roation about z-axis
    tf2::Quaternion q;
    q.setEulerZYX(-xSetpointQuad.toDouble()-3.14156, 0, 0);

    //double yaw_degrees = 0;  // North
    //double yaw = radians(yaw_degrees);
//    tf::Quaternion quaternion;
//    quaternion.setEuler(0,0,0);
    setpointPosMsg.pose.orientation.x = q.getX();
    setpointPosMsg.pose.orientation.y = q.getY();
    setpointPosMsg.pose.orientation.z = q.getZ();
    setpointPosMsg.pose.orientation.w = q.getW();

    //now = std::time(0);   // get time now
    //name_log_file << "[" << ctime(&now) << "]  dt=" << dt <<"    position.x= " << posX << "   position.y= " << posY << "    position.z = "<< msg->ground_distance << "\n";
    pub_pos.publish(setpointPosMsg);
}

void  MainApplication::stop_disarmSequence()
{
    channels.channels[2] = 1102;
    channels.channels[3] = 1500;
    count = 0;
}

void MainApplication::publish_RCchannels()
{
    pub_RcOverride.publish(channels);
}

//overrides RC to start props ( 8 channels: 1500	1500	1383	1498	2067	960	1514	1514)
void MainApplication::publish_startSequence(bool start)
{
    if(start)
    {
        if(count>= 0 and count <40){
            //push channel 4 to right
            //rcChannels.channels = {1500,	1500, 1102, 1912, 2067, 960, 1000, 1514};
            channels.channels[3] = 1912;
        }
        else{
            //channel 4 ideal
            // startSequence.channels = {1500, 1500, 1102, 1502, 2067, 960, 1514, 1514}; zero throttle
            //rcChannels.channels = {1500, 1500, 1383, 1502, 2067, 960, 1000, 1514};
            channels.channels[2] = 1200;
            channels.channels[3] = 1502;
            //pub_RcOverride.publish(startSequence);
        }
         count++;
    }
    else
    {
        channels.channels[2] = 1102;
        channels.channels[3] = 1000;
        count = 0;
    }

}

void MainApplication::publish_kill(bool kill)
{
    if (kill)
    {
        // set channel 7 to high and thrust to 0
        channels.channels[2] = 1102;
        channels.channels[6] = 1514;
    }
    else
    {
        // no kill signal
        channels.channels[6] = 1000;
    }
}



void MainApplication::publish_MocapPosition(QString xSetpointPos, QString ySetpointPos, QString zSetpointPos)
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


void MainApplication::test()
{
    std::cout<<"TIMEOUT!!"<<std::endl;
}


void MainApplication::enableTf2VisionEstimate(bool enable)
{
    if (enable)
    {
        std::cout<<"Subscribing to /tf"<<std::endl;
        ros::NodeHandle n;
        sub_map= n.subscribe("/tf", 100, retrieveMapCallback);
   }
   else
    {
        sub_map.shutdown();
    }
}

void MainApplication::enableOrbSlam2VisionEstimate(bool enable)
{
    if (enable)
    {
        std::cout<<"Subscribing to OrbSlam2 /tf"<<std::endl;
        ros::NodeHandle n;
        sub_map= n.subscribe("/tf", 100, retrieveOrbSlam2Callback);
   }
   else
    {
        sub_map.shutdown();
    }
}


void MainApplication::enableGazebo2VisionEstimate(bool enable)
{
    if (enable)
    {
        ros::NodeHandle n;
        sub = n.subscribe("/gazebo/model_states", 100, gazeboModelStateCallback);
   }
   else
    {
        sub.shutdown();
    }
}


void MainApplication::enablePx4Flow(bool enable)
{
    if (enable)
    {
        ros::NodeHandle n;
        sub_opticalFlow = n.subscribe("/px4flow/opt_flow", 100, opticalFlowCallback);
   }
   else
    {
        initalize = 1;
        sub_opticalFlow.shutdown();
    }
}


