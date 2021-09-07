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
        // publish info to the console for the user
        ROS_INFO("Starting Action");
        geometry_msgs::PoseStamped pre_palce_position;
        double pre_palce_position_x;
        double pre_palce_position_y;
        double pre_palce_position_z;
        nh_.getParam("/place_position_x",pre_palce_position_x);
        nh_.getParam("/place_position_y",pre_palce_position_y);
        nh_.getParam("/place_position_z",pre_palce_position_z);

        pre_palce_position.header.stamp = ros::Time::now();
        pre_palce_position.header.frame_id = "j2n6s300_link_base";
        pre_palce_position.pose.position.x = pre_palce_position_x;
        pre_palce_position.pose.position.y = pre_palce_position_y;
        pre_palce_position.pose.position.z = pre_palce_position_z+0.02;
        pre_palce_position.pose.orientation.x = 1.0;
        pre_palce_position.pose.orientation.y = 0.0;
        pre_palce_position.pose.orientation.z = 0.0;
        pre_palce_position.pose.orientation.w = 0.0;
        arm->setPoseTarget(pre_palce_position);
        moveit::planning_interface::MoveGroupInterface::Plan plan;

        int n_attempts = 0;
        int max_attempts = 5;
        bool arm_res = false;
        bool gripper_res = false;

        while (arm_res != moveit::planning_interface::MoveItErrorCode::SUCCESS && n_attempts < max_attempts)
        {
          n_attempts ++;
          cout << "Planning attempt:" << n_attempts << endl;
          arm->plan(plan);
          arm_res = (arm->execute(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
          if(arm_res)
          {
            gripper_res = OperateGripper(false);
          }
        }

        if(gripper_res)
        {
          set_status(SUCCESS);
        }
        else
        {
          set_status(FAILURE);
        }
    }

    geometry_msgs::PoseStamped get_robot_state()
    {
      geometry_msgs::PoseStamped current_state;

      tf2_ros::Buffer tfBuffer;
      tf2_ros::TransformListener listener_(tfBuffer);
      geometry_msgs::TransformStamped frame_info;

      std::string target_frame = "j2n6s300_link_base";
      std::string source_frame = "j2n6s300_end_effector";
      try {
        ros::Time now = ros::Time::now();
        listener->waitForTransform(target_frame,source_frame,now,ros::Duration(1.0));
        frame_info = tfBuffer.lookupTransform(target_frame,source_frame,ros::Time(0));
      } catch (tf::TransformException ex) {
        ROS_ERROR("transfrom exception : %s",ex.what());
      }
      current_state.pose.position.x = frame_info.transform.translation.x;
      current_state.pose.position.y = frame_info.transform.translation.y;
      current_state.pose.position.z = frame_info.transform.translation.z;
      current_state.pose.orientation.x = frame_info.transform.rotation.x;
      current_state.pose.orientation.y = frame_info.transform.rotation.y;
      current_state.pose.orientation.z = frame_info.transform.rotation.z;
      current_state.pose.orientation.w = frame_info.transform.rotation.w;

      return current_state;
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
        gripper_joint_positions[0] = 0.965;
        gripper_joint_positions[1] = 0.9655;//89
        gripper_joint_positions[2] = 0.9655;//90
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
    ros::init(argc, argv, "open_gripper");
    ROS_INFO(" Enum: %d", RUNNING);
    ROS_INFO(" Action Ready for Ticks");
    BTAction bt_action(ros::this_node::getName());
    ros::spin();

    return 0;
}
