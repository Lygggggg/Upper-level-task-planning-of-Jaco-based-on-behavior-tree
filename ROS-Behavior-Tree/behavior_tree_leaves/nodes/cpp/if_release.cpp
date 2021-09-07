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
#include <string>
#include <vector>
#include "std_msgs/String.h"
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_listener.h>
#include <tf2_ros/buffer.h>

using namespace std;

enum Status {RUNNING, SUCCESS, FAILURE};


class BTAction
{
protected:
    ros::NodeHandle nh_;
    // NodeHandle instance must be created before this line. Otherwise strange errors may occur.
    actionlib::SimpleActionServer<behavior_tree_core::BTAction> as_;
    std::string action_name_;
    // create messages that are used to published feedback/result
    behavior_tree_core::BTFeedback feedback_;
    behavior_tree_core::BTResult result_;
private:
    moveit::planning_interface::MoveGroupInterface* move_group_;
    tf::TransformListener* listener;

public:
    explicit BTAction(std::string name) :
        as_(nh_, name, boost::bind(&BTAction::execute_callback, this, _1), false),
        action_name_(name)
    {
        // start the action server (action in sense of Actionlib not BT action)
      move_group_ = new moveit::planning_interface::MoveGroupInterface("arm");
      listener = new tf::TransformListener;
      move_group_->setPoseReferenceFrame("j2n6s300_link_base");
        as_.start();
        ROS_INFO("Condition Server Started");
    }

    ~BTAction(void)
    {
      delete move_group_;
     delete listener;
    }
    void execute_callback(const behavior_tree_core::BTGoalConstPtr &goal)
    {
      geometry_msgs::PoseStamped current_pose = get_robot_state();
      geometry_msgs::PoseStamped place_position;
      place_position.header.frame_id = "j2n6s300_link_base";
      place_position.header.stamp = ros::Time::now();
      place_position.pose.position.x = 0.4837;
      place_position.pose.position.y = -0.31853;
      place_position.pose.position.z = 0.01;
      place_position.pose.orientation.x = 1.0;
      place_position.pose.orientation.y = 0.0;
      place_position.pose.orientation.z = 0.0;
      place_position.pose.orientation.w = 0.0;

      nh_.setParam("/place_position_x",place_position.pose.position.x);
      nh_.setParam("/place_position_y",place_position.pose.position.y);
      nh_.setParam("/place_position_z",place_position.pose.position.z);


      double x_2 = pow((current_pose.pose.position.x-place_position.pose.position.x),2);
      double y_2 = pow((current_pose.pose.position.y-place_position.pose.position.y),2);
      double z_2 = pow((current_pose.pose.position.z-place_position.pose.position.z),2);


      double result = sqrt(x_2+y_2+z_2);

      if (result<0.1)
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

    //  returns the status to the client (Behavior Tree)
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
            ROS_INFO("Condition %s Succeeded", ros::this_node::getName().c_str() );
            break;
        case FAILURE:
            ROS_INFO("Condition %s Failed", ros::this_node::getName().c_str() );
            break;
        default:
            break;
        }
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "if_release");
    ROS_INFO(" Enum: %d", RUNNING);
    ROS_INFO(" condition Ready for Ticks");
    BTAction bt_action(ros::this_node::getName());
    ros::spin();
    return 0;
}
