#include "ros/ros.h"
#include "std_msgs/String.h"
#include <moveit/move_group_interface/move_group_interface.h>
#include "dmp/dmp.h"
#include "dmp/waypoints.h"

#include <iostream>
#include <fstream>
#include <cassert>
#include <string>

using namespace std;


class Listener
{
public:
  std::vector<geometry_msgs::Pose> waypoints;
  int count = 0;
public:
  void chatterCallback(const dmp::waypoints::ConstPtr& sub_path);
  void movejoint(void);
};
// %EndTag(CLASS_WITH_DECLARATION)%

void Listener::chatterCallback(const dmp::waypoints::ConstPtr& sub_path)
{
  waypoints = sub_path->path;
  ++count;
}

void Listener::movejoint()
{
  moveit::planning_interface::MoveGroupInterface move_group("arm");
  move_group.setPoseReferenceFrame("j2n6s300_link_base");

  cout << "---------------Go to the home----------------" << endl;
  move_group.setNamedTarget("Home");
  move_group.move();

  int n_attempts = 0;
  int max_attempts = 5;
  bool result = false;

  moveit_msgs::RobotTrajectory trajectory;
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  double fraction;
  cout << "---------------execute dmp----------------" << endl;
  while (result != moveit::planning_interface::MoveItErrorCode::SUCCESS && n_attempts < max_attempts)
  {
    n_attempts ++;
    cout << "Planning attempt:" << n_attempts << endl;
    fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    my_plan.trajectory_ = trajectory;
    ROS_INFO_NAMED("tutorial", "Visualizing plan 4 (Cartesian path) (%.2f%% acheived)", fraction * 100.0);
    move_group.execute(my_plan);

    result = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ros::WallDuration(0.5).sleep();
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "dmp_subscribe");
  ros::NodeHandle nh;

  Listener listener;
  ros::Subscriber sub = nh.subscribe("pub_jiyuan_dmp_data", 1000, &Listener::chatterCallback, &listener);

  ros::Rate loop_rate(10);

  while (ros::ok() and listener.count<=1)
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  ros::AsyncSpinner spinner(1);
  spinner.start();
  std::cout<<listener.waypoints.size()<<std::endl;

  listener.movejoint();

  ros::shutdown();
  return 0;
}
