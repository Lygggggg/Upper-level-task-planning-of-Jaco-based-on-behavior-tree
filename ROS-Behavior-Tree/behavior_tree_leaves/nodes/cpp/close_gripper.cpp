/* Copyright (C) 2015-2017 Michele Colledanchise - All Rights Reserved
*
*   Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"),
*   to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
*   and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
*   The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
*
*   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
*   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
*   WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <behavior_tree_core/BTAction.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <string>
using namespace std;

enum Status {RUNNING, SUCCESS, FAILURE};  // BT return status


class BTAction
{
protected:
    ros::NodeHandle nh_;
    ros::NodeHandle nnh_;
    // NodeHandle instance must be created before this line. Otherwise strange error may occur.
    actionlib::SimpleActionServer<behavior_tree_core::BTAction> as_;
    std::string action_name_;
    // create messages that are used to published feedback/result
    behavior_tree_core::BTFeedback feedback_;  // action feedback (SUCCESS, FAILURE)
    behavior_tree_core::BTResult result_;  // action feedback  (same as feedback for us)
    moveit::planning_interface::MoveGroupInterface* gripper;
    moveit::planning_interface::MoveGroupInterface* arm;
    tf::TransformListener* listener;
    const robot_state::JointModelGroup *kinova_gripper_joint_model_group;


public:
    explicit BTAction(std::string name) :
        as_(nh_, name, boost::bind(&BTAction::execute_callback, this, _1), false),
        action_name_(name)
    {
      gripper = new moveit::planning_interface::MoveGroupInterface("gripper");
      arm = new moveit::planning_interface::MoveGroupInterface("arm");
      listener = new tf::TransformListener;
      arm->allowReplanning(true);
      gripper->allowReplanning(true);
      arm->setMaxVelocityScalingFactor(0.6);
      arm->setMaxAccelerationScalingFactor(0.04);
      arm->setPoseReferenceFrame("j2n6s300_link_base");
        // Starts the action server
        as_.start();
    }

    ~BTAction(void)
    {
      delete gripper;
      delete arm;
      delete listener;
    }

    void execute_callback(const behavior_tree_core::BTGoalConstPtr &goal)
    {
      gripper->setNamedTarget("Open");

      int n_attempts = 0;
      int max_attempts = 5;
      bool gripper_res = false;

      //张开机械臂手爪的规划
      gripper_res = OperateGripper(false);
      if(gripper_res)
      {
        geometry_msgs::PoseStamped grasp_pose;
        grasp_pose.header.frame_id = "j2n6s300_link_base";
        grasp_pose.header.stamp = ros::Time::now();
        double grasp_x;
        double grasp_y;
        double grasp_z;
        // 这是从find_object中读取到的物体在基坐标系下位置；
        nnh_.getParam("/x_under_base",grasp_x);
        nnh_.getParam("/y_under_base",grasp_y);
        nnh_.getParam("/z_under_base",grasp_z);
        grasp_pose.pose.position.x = grasp_x;
        grasp_pose.pose.position.y = grasp_y;
        grasp_pose.pose.position.z = grasp_z;
        grasp_pose.pose.orientation.x = 1.0;
        grasp_pose.pose.orientation.y = 0.0;
        grasp_pose.pose.orientation.z = 0.0;
        grasp_pose.pose.orientation.w = 0.0;

        arm->setPoseTarget(grasp_pose);
        bool arm_res = false;
        gripper_res = false;
        // 机械臂执行往下抓取物体的规划
        moveit::planning_interface::MoveGroupInterface::Plan down_plan;

        arm->setStartStateToCurrentState();

        while (arm_res != moveit::planning_interface::MoveItErrorCode::SUCCESS && n_attempts < max_attempts)
        {
          n_attempts ++;
          cout << "Planning attempt:" << n_attempts << endl;
          arm->plan(down_plan);
          arm_res = (arm->execute(down_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

          if(arm_res)
            gripper_res = OperateGripper(true);
        }
        if(!arm_res) set_status(FAILURE);
        else
        {
          //向上提起机械臂0.08
          grasp_pose.pose.position.z += 0.11;
          grasp_pose.pose.orientation.x = 1.0;
          grasp_pose.pose.orientation.y = 0.0;
          grasp_pose.pose.orientation.z = 0.0;
          grasp_pose.pose.orientation.w = 0.0;
          arm->setPoseTarget(grasp_pose);
          moveit::planning_interface::MoveGroupInterface::Plan lift_plan;
          arm_res = false;
          arm->setStartStateToCurrentState();
          while (arm_res != moveit::planning_interface::MoveItErrorCode::SUCCESS && n_attempts < max_attempts)
          {
            n_attempts ++;
            cout << "Planning attempt:" << n_attempts << endl;
            arm->plan(lift_plan);
            arm_res = (arm->execute(lift_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
          }
          if(arm_res) set_status(SUCCESS);
          else set_status(FAILURE);
        }
      }
      else
      {
        set_status(FAILURE);
      }

    }

    bool OperateGripper(bool close_gripper)
    {
      // close_gripper:false为张开手爪
      //true为闭合手爪
      kinova_gripper_joint_model_group = arm->getCurrentState()->getJointModelGroup("gripper");
      moveit::core::RobotStatePtr gripper_current_state = gripper->getCurrentState();
      std::vector<double> gripper_joint_positions;
      gripper_current_state->copyJointGroupPositions(kinova_gripper_joint_model_group,gripper_joint_positions);

      ROS_INFO("Operating gripper.");
      if (close_gripper)
      {
        gripper_joint_positions[0] = 1.1;
        gripper_joint_positions[1] = 1.1;//89
        gripper_joint_positions[2] = 1.1;//90
        ROS_INFO("Gripper closed.");
      }
      else
      {
        gripper_joint_positions[0] = 0.73;
        gripper_joint_positions[1] = 0.73;
        gripper_joint_positions[2] = 0.73;
        ROS_INFO("Gripper opened.");
      }
      gripper->setJointValueTarget(gripper_joint_positions);
      bool success = static_cast<bool>(gripper->move());
      return success;
    }


    // returns the status to the client (Behavior Tree)
    void set_status(int status)
    {
        // Set The feedback and result of BT.action
        feedback_.status = status;
        result_.status = feedback_.status;
        // publish the feedback
        as_.publishFeedback(feedback_);
        // setSucceeded means that it has finished the action (it has returned SUCCESS or FAILURE).
        as_.setSucceeded(result_);

        switch (status)  // Print for convenience
        {
        case SUCCESS:
            ROS_INFO("Action %s Succeeded", ros::this_node::getName().c_str() );
            break;
        case FAILURE:
            ROS_INFO("Action %s Failed", ros::this_node::getName().c_str() );
            break;
        default:
            break;
        }
    }
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "close_gripper");
    BTAction bt_action(ros::this_node::getName());
    ros::spin();

    return 0;
}
