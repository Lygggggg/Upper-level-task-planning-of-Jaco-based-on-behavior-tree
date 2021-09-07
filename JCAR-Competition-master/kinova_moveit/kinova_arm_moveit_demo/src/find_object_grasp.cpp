#include "ros/ros.h"

#include<iostream> //C++标准输入输出库 
#include <vector>
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"
#include <kinova_msgs/ArmPoseActionGoal.h> // 注意格式源文件名字为ArmPose.action
#include <kinova_driver/kinova_ros_types.h>

#include <actionlib/client/simple_action_client.h>
#include <kinova_msgs/SetFingersPositionAction.h>
#include <kinova_msgs/ArmPoseAction.h>
#include <kinova_msgs/HomeArm.h>
#include <kinova_msgs/ArmJointAnglesAction.h>
#include <tf/transform_listener.h>
#include <find_object_2d/ObjectsStamped.h>
//#include <QtCore/Qstd::string>
using namespace std;


const int FINGER_MAX1 = 6714;
const int FINGER_MAX2 = 6924;
const int FINGER_MAX3 = 7332;

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <find_object_2d/ObjectsStamped.h>
#include <tf/transform_broadcaster.h>

class TfExample
{
public:
	TfExample() :
		objFramePrefix_("object")
	{
		ros::NodeHandle pnh("~");
		pnh.param("target_frame_id", targetFrameId_, targetFrameId_);
		pnh.param("object_prefix", objFramePrefix_, objFramePrefix_);
		ROS_INFO("this is constructor");
		ros::NodeHandle nh;
		subs_ = nh.subscribe("objectsStamped", 1, &TfExample::objectsDetectedCallback, this);
	}

	// Here I synchronize with the ObjectsStamped topic to
	// know when the TF is ready and for which objects
	void objectsDetectedCallback(const find_object_2d::ObjectsStampedConstPtr & msg)
	{
		ROS_INFO("jump into callback");
		if(msg->objects.data.size())
		{
			targetFrameId_ = "j2n6s300_end_effector";
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

				std::cout<<"multiSuffix: "<<multiSuffix<<"  "<<"id: "<<id<<std::endl;
				// "object_1", "object_1_b", "object_1_c", "object_2"
				//std::string objectFrameId = QString("%1_%2%3").arg(objFramePrefix_.c_str()).arg(id).arg(multiSuffix).toStdString();
				std::string tmp1 = "_";
				std::string objectFrameId = std::string(objFramePrefix_.c_str()+tmp1+to_string(id)+multiSuffix);

				std::cout<<targetFrameId<<std::endl;
				std::cout<<objectFrameId<<std::endl;
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
	
				try
 					{
      					bool result=cartesian_pose_client(pose.getOrigin().x(),pose.getOrigin().y(),pose.getOrigin().z(),
														  pose.getRotation().x(),pose.getRotation().y(),pose.getRotation().z(),pose.getRotation().w());
      					if(result)
      					{
        					bool finger_re= finger_control_client(0.5,0.5,0.5);
        					ROS_INFO("Cartesian pose sent!.");
        					if(finger_re)
            				ROS_INFO("finger move!.");
      					}
     
 		 			}
				catch(ros::Exception)
       				{
        				ROS_INFO("program interrupted before completion.");
       				}
				
			}
		}
	}
	
	bool cartesian_pose_client(double x, double y, double z, double qx, double qy, double qz, double qw)//四元数
{


    actionlib::SimpleActionClient <kinova_msgs::ArmPoseAction> *client = new actionlib::SimpleActionClient<kinova_msgs::ArmPoseAction>(
            "/j2n6s300_driver/pose_action/tool_pose");//取的主题

    client->waitForServer();

    kinova_msgs::ArmPoseGoal pose_goal;//注意形式ArmPose.action
    pose_goal.pose.header.frame_id = "j2n6s300_link_base";

    pose_goal.pose.pose.position.x = x;
    pose_goal.pose.pose.position.y = y;
    pose_goal.pose.pose.position.z = z;

    pose_goal.pose.pose.orientation.x = qx;
    pose_goal.pose.pose.orientation.y = qy;
    pose_goal.pose.pose.orientation.z = qz;
    pose_goal.pose.pose.orientation.w = qw;


    client->sendGoal(pose_goal);


    if (client->waitForResult(ros::Duration(200.0)))//延时得高机械臂不会发送，只能自己检测坐标，奇异值也不会告诉所有信息都通过主题反馈
    {
        client->getResult();
        return true;
    } else {
        client->cancelAllGoals();
        ROS_WARN_STREAM("The gripper action timed-out");
        return false;
    }

}

    bool finger_control_client(float finger1, float finger2, float finger3)//0-1
{


    actionlib::SimpleActionClient <kinova_msgs::SetFingersPositionAction> *finger_client = new actionlib::SimpleActionClient<kinova_msgs::SetFingersPositionAction>(
            "j2n6s300_driver/fingers_action/finger_positions");

    finger_client->waitForServer();

    kinova_msgs::SetFingersPositionGoal finger_goal;

    finger_goal.fingers.finger1 = finger1 * FINGER_MAX1;
    finger_goal.fingers.finger2 = finger2 * FINGER_MAX2;
    finger_goal.fingers.finger3 = finger3 * FINGER_MAX3;


    finger_client->sendGoal(finger_goal);


    if (finger_client->waitForResult(ros::Duration(150.0))) {
        finger_client->getResult();
        return true;
    } else {
        finger_client->cancelAllGoals();
        ROS_WARN_STREAM("The gripper action timed-out");
        return false;
    }

}

private:
    std::string targetFrameId_;
    std::string objFramePrefix_;
    ros::Subscriber subs_;
    tf::TransformListener tfListener_;
};

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "find_object_grasp");
    TfExample sync;
    ros::spin();
}






