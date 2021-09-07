#include <ros/ros.h>
#include <pick_place.h>
#include <ros/console.h>

#include <tf_conversions/tf_eigen.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <tf2_ros/transform_listener.h>
const double FINGER_MAX = 6400;

#include <iostream>
#include <fstream>
#include <cassert>
#include <string>

float a[1000][7];
int num = 0;

using namespace std;
using namespace kinova;

PickPlace::PickPlace(ros::NodeHandle &nh):
    nh_(nh),tfListener(tfBuffer)
{
    ros::NodeHandle pn("~");

    nh_.param<std::string>("/robot_type",robot_type_,"j2n6s300");
    nh_.param<bool>("/robot_connected",robot_connected_,false);

    if (robot_connected_)
    {
        sub_pose_ = nh_.subscribe<geometry_msgs::PoseStamped>("/" + robot_type_ +"_driver/out/tool_pose", 1, &PickPlace::get_current_pose, this);
    }

    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model_ = robot_model_loader.getModel();

    planning_scene_.reset(new planning_scene::PlanningScene(robot_model_));
    planning_scene_monitor_.reset(new planning_scene_monitor::PlanningSceneMonitor("robot_description"));

    ROS_INFO("--------LOADING1--------");

    group_ = new moveit::planning_interface::MoveGroupInterface("arm");
    gripper_group_ = new moveit::planning_interface::MoveGroupInterface("gripper");


    group_->setEndEffectorLink(robot_type_ + "_end_effector");

    finger_client_ = new actionlib::SimpleActionClient<kinova_msgs::SetFingersPositionAction>
            ("/" + robot_type_ + "_driver/fingers_action/finger_positions", false);

    pub_co_ = nh_.advertise<moveit_msgs::CollisionObject>("/collision_object", 10);
    pub_aco_ = nh_.advertise<moveit_msgs::AttachedCollisionObject>("/attached_collision_object", 10);
    pub_planning_scene_diff_ = nh_.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);

    int arm_joint_num = robot_type_[3]-'0';
    joint_names_.resize(arm_joint_num);
    joint_values_.resize(joint_names_.size());
    for (uint i = 0; i<joint_names_.size(); i++)
    {
        joint_names_[i] = robot_type_ + "_joint_" + boost::lexical_cast<std::string>(i+1);
    }
    kinova_gripper_joint_model_group = group_->getCurrentState()->getJointModelGroup("gripper");

    define_cartesian_pose();

    result_ = false;

    group_->setNamedTarget("Home");
    ROS_INFO("Go to the Home position");
    result_ = static_cast<bool>(group_->plan(kinova_plan));
    group_->execute(kinova_plan);

    OperateGripper(false);

    // 执行基元
    group_->setPoseReferenceFrame("j2n6s300_link_base");

    moveit_msgs::RobotTrajectory trajectory;

    const double jump_threshold = 0.0;
    const double eef_step = 0.01;

    double fraction = group_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    kinova_plan.trajectory_ = trajectory;
    ROS_INFO_NAMED("tutorial", "Visualizing plan 4 (Cartesian path) (%.2f%% acheived)", fraction * 100.0);
    group_->execute(kinova_plan);


    // 到达识别到的物体位置上方
    group_->setPoseTarget(intermedia_pt_5);
    result_ = static_cast<bool>(group_->plan(kinova_plan));
    group_->execute(kinova_plan);

    OperateGripper(true);

}

PickPlace::~PickPlace()
{

    pub_co_.shutdown();
    pub_aco_.shutdown();
    pub_planning_scene_diff_.shutdown();

    // release memory
    delete group_;
    delete gripper_group_;
    delete finger_client_;
}

void read_txt()
{
  cout << "---------------Read data----------------" << endl;
  ifstream file("/home/lni/Ros_ws/catkin_ws/src/JCAR-Competition-master/robot_grasp/data/pick_place_data.txt", ios::in);

  if (file.is_open() == false) {
      cerr << "Error!" << endl;
      exit(-1);
  }

  while (file >> a[num][0])
  {
      for (int i = 1; i < 7; i++) {
          file >> a[num][i];
      }
      num++;
  }
  cout << "Total " << num << " line of data" << endl;
  file.close();
}

void PickPlace::get_current_pose(const geometry_msgs::PoseStampedConstPtr &msg)
{
    boost::mutex::scoped_lock lock(mutex_pose_);
    current_pose_ = *msg;
}

void PickPlace::define_cartesian_pose()
{

  read_txt();
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

    double getParam_x;
    double getParam_y;
    double getParam_z;
    nh_.getParam("/x_under_base", getParam_x);
    nh_.getParam("/y_under_base", getParam_y);
    nh_.getParam("/z_under_base", getParam_z);

    intermedia_pt_5.header.frame_id = "j2n6s300_link_base";
    intermedia_pt_5.header.stamp = ros::Time::now();
    intermedia_pt_5.pose.position.x = getParam_x;
    intermedia_pt_5.pose.position.y = getParam_y;
    intermedia_pt_5.pose.position.z = getParam_z + 0.01;
    intermedia_pt_5.pose.orientation.x = 1.0;
    intermedia_pt_5.pose.orientation.y = 0.0;
    intermedia_pt_5.pose.orientation.z = 0.0;
    intermedia_pt_5.pose.orientation.w = 0.0;

}

bool PickPlace::OperateGripper(bool close_gripper)
{
  moveit::core::RobotStatePtr gripper_current_state = gripper_group_->getCurrentState();
  std::vector<double> gripper_joint_positions;
  gripper_current_state->copyJointGroupPositions(kinova_gripper_joint_model_group,gripper_joint_positions);

  ROS_INFO("Operating gripper.");
  ROS_INFO("No. of joints in eef_group: %zd", gripper_joint_positions.size());
  if (close_gripper)
  {
    gripper_joint_positions[0] = 0.965;
    gripper_joint_positions[1] = 0.9655;//89
    gripper_joint_positions[2] = 0.9655;//90
  }
  else
  {
    gripper_joint_positions[0] = 0.73;
    gripper_joint_positions[1] = 0.73;
    gripper_joint_positions[2] = 0.73;
    ROS_INFO("Gripper opened.");
  }
  gripper_group_->setJointValueTarget(gripper_joint_positions);
  ros::Duration(1.0).sleep();
  bool success = static_cast<bool>(gripper_group_->move());
  return success;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sub_and_go2");
    ros::NodeHandle node;

    ros::AsyncSpinner spinner(4);
    spinner.start();
    kinova::PickPlace pick_place(node);
    ROS_INFO("Waiting for grasp!");
    ros::shutdown();
    return 0;
}
