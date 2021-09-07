#include <pick_place.h>
#include <ros/console.h>

#include <tf_conversions/tf_eigen.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <tf2_ros/transform_listener.h>

#include "ros/ros.h"
#include <cstdlib>

using namespace kinova;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "gripper_client");
	ROS_INFO("ready !");

	ros::NodeHandle n;
	ros::ServiceClient client = n.serviceClient<robot_grasp::G_pick_place>("/get_pick_place_routine");
  
	robot_grasp::G_pick_place srv;
	srv.request.transformed_param.pcaCentroid.resize(3);
	
	srv.request.transformed_param.num_obj_label = 2;
	std::cout << "size is " << srv.request.transformed_param.num_obj_label << std::endl;	
	//srv.request.transformed_param.pcaCentroid[0] = 0.240161;
	srv.request.transformed_param.pcaCentroid[0] = 0.55212;
	
	//srv.request.transformed_param.pcaCentroid[1] = -0.168802;
	srv.request.transformed_param.pcaCentroid[1] = 0.22027;

	//srv.request.transformed_param.pcaCentroid[2] = 0.231732;
	srv.request.transformed_param.pcaCentroid[2] = -0.04813;
	for( int i=0;i<3;i++ )
		std::cout << "data is " << srv.request.transformed_param.pcaCentroid[i] << std::endl;	

	//ROS_INFO("result: ", srv.request.transformed_param.pcaCentroid);

    if (client.call(srv))
  {
    ROS_INFO("result: %d", (int)srv.response.success);
  }
  	else
  {
    ROS_ERROR("Failed to call service robot_grasp::G_pick_place");
    return 1;
  }

  return 0;
}


