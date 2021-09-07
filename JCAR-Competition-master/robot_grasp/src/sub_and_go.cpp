#include <moveit/move_group_interface/move_group_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <iostream>
#include <fstream>
#include <cassert>
#include <string>

using namespace std;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sub_and_go");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  static const std::string PLANNING_GROUP = "arm";
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

  move_group.setPoseReferenceFrame("j2n6s300_link_base");

  cout << "---------------Read data----------------" << endl;
  ifstream file("/home/lni/Ros_ws/catkin_ws/src/JCAR-Competition-master/robot_grasp/data/pick_place_data.txt", ios::in);

  int num = 0;

  if (file.is_open() == false) {
      cerr << "Error!" << endl;
      exit(-1);
  }


  float a[1000][7];
  while (file >> a[num][0])
  {
      for (int i = 1; i < 7; i++) {
          file >> a[num][i];
      }
      num++;
  }
  cout << "Total " << num << " line of data" << endl;
  file.close();

  geometry_msgs::Pose target_pose;
  std::vector<geometry_msgs::Pose> waypoints;

  for (int i = 0;i<num;i++)
  {
    target_pose.position.x = a[i][0];
    target_pose.position.y = a[i][1];
    target_pose.position.z = a[i][2];

    target_pose.orientation.x = a[i][3];
    target_pose.orientation.y = a[i][4];
    target_pose.orientation.z = a[i][5];
    target_pose.orientation.w = a[i][6];
    waypoints.push_back(target_pose);

  }


  double getParam_pose_x;
  double getParam_pose_y;
  double getParam_pose_z;
  node_handle.getParam("/x_under_base", getParam_pose_x);
  node_handle.getParam("/y_under_base", getParam_pose_y);
  node_handle.getParam("/z_under_base", getParam_pose_z);

  geometry_msgs::PoseStamped get_pose;
  get_pose.header.frame_id = "j2n6s300_link_base";
  get_pose.header.stamp = ros::Time::now();
  get_pose.pose.position.x = getParam_pose_x;
  get_pose.pose.position.y = getParam_pose_y;
  get_pose.pose.position.z = getParam_pose_z + 0.02;

  get_pose.pose.orientation.x = 1.0;
  get_pose.pose.orientation.y = 0.0;
  get_pose.pose.orientation.z = 0.0;
  get_pose.pose.orientation.w = 0.0;

  cout << "---------------Go to the home----------------" << endl;
  move_group.setNamedTarget("Home");
  move_group.move();


  int n_attempts = 0;
  int max_attempts = 5;
  bool result;

  moveit_msgs::RobotTrajectory trajectory;
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  double fraction;

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


  move_group.setPoseTarget(get_pose);
  move_group.plan(my_plan);
  move_group.execute(my_plan);

  ros::shutdown();
  return 0;
}
