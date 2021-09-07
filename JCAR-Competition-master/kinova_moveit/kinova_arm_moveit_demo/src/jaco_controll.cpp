#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <kinova_msgs/SetFingersPositionAction.h>
#include <kinova_msgs/ArmPoseAction.h>
#include <kinova_msgs/HomeArm.h>
#include <kinova_msgs/ArmJointAnglesAction.h>

#include <gazebo_msgs/ModelStates.h>

typedef actionlib::SimpleActionClient< kinova_msgs::ArmPoseAction > TrajClient;

class RobotArm
{
	private:
	TrajClient* traj_client_;
	
	public:
	RobotArm()
	{
		traj_client_ = new TrajClient("");
		
		while(!traj_client_ -> waitForServer(ros::Duration(5.0)))
		{
			ROS_INFO("Waiting for the joint_trajectory_action server");
		}	
	}
	~RobotArm()
	{
		delete traj_client_;
	}
	
	
};

int main(){
}