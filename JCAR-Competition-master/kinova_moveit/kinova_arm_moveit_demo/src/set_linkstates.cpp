#include <ros/ros.h>
#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/LinkState.h>
#include <gazebo_msgs/ModelState.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv,"gazebo_set_states_client");
	ros::NodeHandle n;
	ros::Publisher pub_ = n.advertise<gazebo_msgs::LinkState>("/gazebo/set_link_state",1);
	gazebo_msgs::LinkState objstate;
	
	
	objstate.link_name = "object_on_table::j2n6s300_link_6";
	objstate.pose.position.x = 1.45;
	objstate.pose.position.y = -0.38;
	objstate.pose.position.z = -2.105;
	
	objstate.pose.orientation.x = -0.428;
	objstate.pose.orientation.y = -0.471;
	objstate.pose.orientation.z = -0.771;
	objstate.pose.orientation.w = 0.032;
	
	
	objstate.twist.linear.x = 0;
	objstate.twist.linear.y = 0;
	objstate.twist.linear.z = 0;
	
	objstate.twist.angular.x = 0;
	objstate.twist.angular.y = 0;
	objstate.twist.angular.z = 0;

	/*geometry_msgs::Pose pose;
	
	//geometry_msgs::Twist twist;
	
	geometry_msgs::Point position;
	geometry_msgs::Quaternion orientation;
	
	ros::Rate loop_rate(1);
	
	objstate.model_name = "object_on_table";
	
	ROS_INFO("---------1----------");
	
	loop_rate.sleep();
	
	position.x = 0.45;
	position.y = -0.38;
	position.z = -0.105;
	
	orientation.x = -0.428;
	orientation.y = -0.471;
	orientation.z = -0.771;
	orientation.w = 0.032;
	
	
	/*objstate.request.model_state.twist.linear.z =0.0;
	objstate.request.model_state.twist.angular.x =0.0;
	objstate.request.model_state.twist.angular.y =0.0;
	objstate.request.model_state.twist.angular.z =0.0;
	pose.position = position;
	pose.orientation = orientation;
	objstate.pose = pose;*/
	objstate.reference_frame = "world";
	
	
	ros::Rate loop_rate(0.4);
	
	ROS_INFO("---------1----------");
	
	loop_rate.sleep();
	
	pub_.publish(objstate);
}
    

   