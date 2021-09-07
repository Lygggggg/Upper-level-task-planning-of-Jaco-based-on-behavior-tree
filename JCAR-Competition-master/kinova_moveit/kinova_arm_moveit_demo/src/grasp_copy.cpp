#include "ros/ros.h"

#include <string>
#include <sstream>

#include<iostream> //C++标准输入输出库 
#include <vector>
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"
#include <kinova_msgs/ArmPoseActionGoal.h> // 注意格式源文件名字为ArmPose.action
#include <ros/ros.h>
#include <kinova_driver/kinova_ros_types.h>

#include <actionlib/client/simple_action_client.h>
#include <kinova_msgs/SetFingersPositionAction.h>
#include <kinova_msgs/ArmPoseAction.h>
#include <kinova_msgs/HomeArm.h>
#include <kinova_msgs/ArmJointAnglesAction.h>

#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

using namespace std;

class JointControl
{
	public:
	JointControl()
	{
		pub_ = n_.advertise<trajectory_msgs::JointTrajectory>("/j2n6s300/effort_joint_trajectory_controller/command",1);
	}
	
	void callback()
	{

		trajectory_msgs::JointTrajectory jointCmd;
		trajectory_msgs::JointTrajectoryPoint point;
		
		jointCmd.header.stamp = ros::Time::now();
		jointCmd.header.frame_id = "liu";
		
		jointCmd.joint_names.resize(6);
		//jointCmd.points.resize(3);
		
		point.time_from_start = ros::Duration(2.0);
		
		point.positions.resize(6);
		point.velocities.resize(6);
		point.accelerations.resize(6);
		point.effort.resize(6);
		jointCmd.points.resize(1);
		
		ROS_INFO("------2------");
		size_t size = 6;
		for(size_t i=0;i<size;i++)
		{
			jointCmd.joint_names[i] = "j2n6s300_joint_"+ to_string(i+1);
			int j = i%3;
			point.positions[i] = j;
			point.velocities[i] = 0;
			point.accelerations[i] = 0;
			point.effort[i] = 0;

		}
		ROS_INFO("------3------");
		jointCmd.points[0] = point;
		
		pub_.publish(jointCmd);
		
	}
	
	private:
		ros::NodeHandle n_;
		ros::Publisher pub_;
	
};


int main(int argc, char **argv) {


    ros::init(argc, argv, "paragram1");
	
	JointControl moveit;
	
	
	ros::Rate loop_rate(1);
	
	int count = 0;
	while(ros::ok())
	{
		
		ROS_INFO("------1------");
		loop_rate.sleep();
		
		moveit.callback();
		
		++count;
	}

	}
