#include "vicon.h"
#include "mainapplication.h"
#include <iostream>
#include "px_comm/OpticalFlow.h"
#include "ros/ros.h"
#include "visualization_msgs/Marker.h"
#include "tf2_msgs/TFMessage.h"
#include <boost/algorithm/string.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include "geometry_msgs/TransformStamped.h"

#ifndef VICON
Vicon::Vicon()
{}

void Vicon::init()
{}

void Vicon::setIpAndPort(QString ip_port)
{}

void Vicon::setParamsOrigin(QString paramPath)
{}

void Vicon::request_viconData(QString subjectName, QString segmentName, int measurements)
{}

void Vicon::enableVicon2VisionEstimate(bool enable, QString topic)
{}
#else
#include <vicon_bridge/viconGrabPose.h>

extern void retrieveViconDataCallback(const geometry_msgs::TransformStampedConstPtr &msg);
tf2::Transform * origin_tf;
ros::Subscriber sub_vicon;
extern ros::Publisher pub_vision;
extern int posEstMsg_seq;
geometry_msgs::PoseStamped prevSuccesMsg;

Vicon::Vicon()
{}

void Vicon::init()
{
    ros::NodeHandle n;
    pub_vision = n.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose",100);
}

void Vicon::setIpAndPort(QString ip_port)
{
    ros::NodeHandle n;
    std::string ip = ip_port.toStdString();
    boost::algorithm::trim(ip);
    std::cout << "set: " <<  ip << std::endl;
    n.setParam("/vicon/datastream_hostport", ip);
}

void Vicon::setParamsOrigin(QString paramPath)
{
    std::string subPathParam = paramPath.toStdString();
    boost::algorithm::trim(subPathParam);

    ros::NodeHandle n;
    double w_orientation; n.getParam(subPathParam + "zero_pose/orientation/w", w_orientation);
    double x_orientation; n.getParam(subPathParam + "zero_pose/orientation/x", x_orientation);
    double y_orientation; n.getParam(subPathParam + "zero_pose/orientation/y", y_orientation);
    double z_orientation; n.getParam(subPathParam + "zero_pose/orientation/z", z_orientation);
    tf2::Quaternion quaternion_origin(x_orientation,y_orientation,z_orientation,w_orientation);

    double x_pose; n.getParam(subPathParam + "zero_pose/position/x", x_pose);
    double y_pose; n.getParam(subPathParam + "zero_pose/position/y", y_pose);
    double z_pose; n.getParam(subPathParam + "zero_pose/position/z", z_pose);
    tf2::Vector3 pose_origin(x_pose, y_pose, z_pose);

    //origin_tf = new tf2::Transform(quaternion_origin, pose_origin);
}

void Vicon::request_viconData(QString subjectName, QString segmentName, int measurements)
{
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<vicon_bridge::viconGrabPose>("/vicon/grab_vicon_pose");
    vicon_bridge::viconGrabPoseRequest req;
    req.n_measurements = measurements;
    req.subject_name = subjectName.toStdString();
    req.segment_name = segmentName.toStdString();

    vicon_bridge::viconGrabPoseResponse msg;

    bool success = client.call( req , msg );

    if(!success) return;

    if((msg.pose.pose.position.x != msg.pose.pose.position.x) || (msg.pose.pose.position.y != msg.pose.pose.position.y) || (msg.pose.pose.position.z != msg.pose.pose.position.z))
    {
        return; //if nan do not send
    }

    geometry_msgs::PoseStamped posEstMsg;
    posEstMsg.header.stamp = ros::Time::now();
    posEstMsg.header.seq=posEstMsg_seq++;
    posEstMsg.pose.position.x = msg.pose.pose.position.x;
    posEstMsg.pose.position.y = msg.pose.pose.position.y;
    posEstMsg.pose.position.z = msg.pose.pose.position.z;
    posEstMsg.pose.orientation.x = msg.pose.pose.orientation.x;
    posEstMsg.pose.orientation.y = msg.pose.pose.orientation.y;
    posEstMsg.pose.orientation.z = msg.pose.pose.orientation.z;
    posEstMsg.pose.orientation.w = msg.pose.pose.orientation.w;
    std::cout <<  "publishing viconGrabPoseResponse on /mavros/vision_pose/pose" << std::endl;
    pub_vision.publish(posEstMsg);

}

void Vicon::enableVicon2VisionEstimate(bool enable, QString topic)
{
//    if (enable)
//    {
//        ros::NodeHandle n;
//        std::string vicon_topic = topic.toStdString();
//        boost::algorithm::trim(vicon_topic);
//        sub_vicon = n.subscribe( vicon_topic, 100, retrieveViconDataCallback);
//   }
//   else
//    {
//        sub_vicon.shutdown();
//    }
}

#endif
