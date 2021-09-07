/*
 * 物体已经从相机坐标系转换到机械臂基坐标系
 * 在参数服务器上设置物体在基坐标系下的位姿
 * 方便subscribe_and_go.cpp执行
*/

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>


#include<iostream> //C++标准输入输出库
#include <vector>
#include "std_msgs/String.h"

#include <tf/transform_listener.h>
#include <find_object_2d/ObjectsStamped.h>
using namespace std;

class TfExample
{
public:
    TfExample() :
    objFramePrefix_("object")
  {
      ros::NodeHandle pnh_("~");
    pnh_.param("target_frame_id", targetFrameId_, targetFrameId_);
    pnh_.param("object_prefix", objFramePrefix_, objFramePrefix_);
    ROS_INFO("this is constructor");

    ros::NodeHandle nh;
    subs_ = nh.subscribe("objectsStamped", 1, &TfExample::objectsDetectedCallback, this);
  }

    void objectsDetectedCallback(const find_object_2d::ObjectsStampedConstPtr & msg)
    {
      ROS_INFO("jump into callback");
      if(msg->objects.data.size())
      {
        targetFrameId_ = "j2n6s300_link_base";

        std::string targetFrameId = targetFrameId_;

        if(targetFrameId.empty())
        {
          targetFrameId = msg->header.frame_id;
        }
        char multiSubId = 'b';
        int previousId = -1;
        for(unsigned int i=0; i<msg->objects.data.size(); i+=12)
        {
          // get data
          int id = (int)msg->objects.data[i];

          std::string multiSuffix;
          if(id == previousId)
          {
            multiSuffix = std::string("_") + multiSubId++;
          }
          else
          {
            multiSubId = 'b';
          }
          previousId = id;

         // std::cout<<"multiSuffix: "<<multiSuffix<<"  "<<"id: "<<id<<std::endl;

          // "object_1", "object_1_b", "object_1_c", "object_2"
          //std::string objectFrameId = QString("%1_%2%3").arg(objFramePrefix_.c_str()).arg(id).arg(multiSuffix).toStdString();
          std::string tmp1 = "_";
          std::string objectFrameId = std::string(objFramePrefix_.c_str()+tmp1+to_string(id)+multiSuffix);

          //std::cout<<targetFrameId<<std::endl;
          //std::cout<<objectFrameId<<std::endl;
          tf::StampedTransform pose;
          try
          {
            // Get transformation from "object_#" frame to target frame
            // The timestamp matches the one sent over TF
            tfListener_.lookupTransform(targetFrameId, objectFrameId, msg->header.stamp, pose);
          }
          catch(tf::TransformException & ex)
          {
            ROS_WARN("%s",ex.what());
            continue;
          }

          // Here "pose" is the position of the object "id" in target frame.
          ROS_INFO("%s [x,y,z] [x,y,z,w] in \"%s\" frame: [%f,%f,%f] [%f,%f,%f,%f]",
                objectFrameId.c_str(), targetFrameId.c_str(),
                pose.getOrigin().x(), pose.getOrigin().y(), pose.getOrigin().z(),
                pose.getRotation().x(), pose.getRotation().y(), pose.getRotation().z(), pose.getRotation().w());


          ros::NodeHandle nh_;
          nh_.setParam("/x_under_base",pose.getOrigin().x());
          nh_.setParam("/y_under_base",pose.getOrigin().y());
          nh_.setParam("/z_under_base",pose.getOrigin().z());

          nh_.setParam("/qx_under_base",pose.getRotation().x());
          nh_.setParam("/qy_under_base",pose.getRotation().y());
          nh_.setParam("/qz_under_base",pose.getRotation().z());
          nh_.setParam("/qw_under_base",pose.getRotation().w());


          ROS_INFO("------------Params set successfully-------------------");

        }
      }
    }

private:
    std::string targetFrameId_;
    std::string objFramePrefix_;
    ros::Subscriber subs_;
    tf::TransformListener tfListener_;
    float poselist[3];

};


int main(int argc, char * argv[])
{
    ros::init(argc, argv, "find_and_pub");

    TfExample sync;
    ros::spin();

    return 0;
}
