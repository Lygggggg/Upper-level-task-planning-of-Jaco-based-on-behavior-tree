/*采用笛卡尔客户端控制实体机械臂，传入参数：位置从参数服务器读取；
 * 姿态固定抓取
*/
#include <ros/ros.h>
#include <kinova_msgs/ArmPoseActionGoal.h>
#include <kinova_driver/kinova_ros_types.h>

#include <actionlib/client/simple_action_client.h>
#include <kinova_msgs/SetFingersPositionAction.h>
#include <kinova_msgs/ArmPoseAction.h>
#include <kinova_msgs/HomeArm.h>
#include <kinova_msgs/ArmJointAnglesAction.h>

#include<iostream>

const int FINGER_MAX1 = 6400;
const int FINGER_MAX2 = 6400;
const int FINGER_MAX3 = 6400;

using namespace std;

bool Cartesian_client(double x,double y,double z,double qx,double qy,double qz,double qw)
{
  actionlib::SimpleActionClient <kinova_msgs::ArmPoseAction> *client = new actionlib::SimpleActionClient<kinova_msgs::ArmPoseAction>(
          "/j2n6s300_driver/pose_action/tool_pose");//取的主题

  client->waitForServer();
  kinova_msgs::ArmPoseGoal pose_goal;//注意形式ArmPose.action
  pose_goal.pose.header.frame_id = "j2n6s300_link_base";

  pose_goal.pose.pose.position.x = x;
  pose_goal.pose.pose.position.y = y;
  pose_goal.pose.pose.position.z = z;

  //从物体上方抓取的姿态

//  pose_goal.pose.pose.orientation.x = 0.673;
//  pose_goal.pose.pose.orientation.y = 0.74;
//  pose_goal.pose.pose.orientation.z = 0;
//  pose_goal.pose.pose.orientation.w =0.064;

  pose_goal.pose.pose.orientation.x = qx;
  pose_goal.pose.pose.orientation.y = qy;
  pose_goal.pose.pose.orientation.z = qz;
  pose_goal.pose.pose.orientation.w =qw;

  client->sendGoal(pose_goal);


  if (client->waitForResult(ros::Duration(200.0)))//延时得高机械臂不会发送，只能自己检测坐标，奇异值也不会告诉所有信息都通过主题反馈
  {
      client->getResult();
      return true;
  }
  else
  {
      client->cancelAllGoals();
      ROS_WARN_STREAM("The gripper action timed-out");
      return false;
  }
}

bool Finger_client(float finger1, float finger2, float finger3)//0-1
{


actionlib::SimpleActionClient <kinova_msgs::SetFingersPositionAction> *finger_client = new actionlib::SimpleActionClient<kinova_msgs::SetFingersPositionAction>(
        "j2n6s300_driver/fingers_action/finger_positions");

finger_client->waitForServer();

kinova_msgs::SetFingersPositionGoal finger_goal;

finger_goal.fingers.finger1 = finger1 * FINGER_MAX1;
finger_goal.fingers.finger2 = finger2 * FINGER_MAX2;
finger_goal.fingers.finger3 = finger3 * FINGER_MAX3;


finger_client->sendGoal(finger_goal);


if (finger_client->waitForResult(ros::Duration(150.0)))
{
    finger_client->getResult();
    return true;
}
else
{
    finger_client->cancelAllGoals();
    ROS_WARN_STREAM("The gripper action timed-out");
    return false;
}

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "subscribe_and_go");
  ros::NodeHandle nh;


  double pose_x1;
  double pose_y1;
  double pose_z1;
  nh.getParam("/x_under_base",  pose_x1);
  nh.getParam("/y_under_base", pose_y1);
  nh.getParam("/z_under_base", pose_z1);
      pose_x1=pose_x1+0.08101;
      pose_y1=pose_y1+0.04625;
      pose_z1=pose_z1+0.30104;
      double pose_qx1=0.58862;
      double pose_qy1=0.69094;
      double pose_qz1=-0.3656;
      double pose_qw1=-0.20607;

      try
      {
        bool result = Cartesian_client(pose_x1,pose_y1,pose_z1,pose_qx1,pose_qy1,pose_qz1,pose_qw1);
        if(result)
        {
          ROS_INFO("末端执行成功");
          bool result_finger = Finger_client(0,0,0);
          if(result_finger)
          {
            ROS_INFO("指尖执行成功");
          }
        }
      }
      catch(ros::Exception)
      {
        ROS_INFO("program interrupted before completion.");
      }


        float pose_x2;
        float pose_y2;
        float pose_z2;
        nh.getParam("/x_under_base",  pose_x2);
        nh.getParam("/y_under_base", pose_y2);
        nh.getParam("/z_under_base", pose_z2);

        double pose_qx2=0.50245;
        double pose_qy2=0.51912;
        double pose_qz2=-0.50306;
        double pose_qw2=-0.47433;

        try
        {
          bool result = Cartesian_client(pose_x2,pose_y2,pose_z2,pose_qx2,pose_qy2,pose_qz2,pose_qw2);
          if(result)
          {
            ROS_INFO("末端执行成功");
            bool result_finger = Finger_client(0.8,0.8,0.8);
            if(result_finger)
            {
              ROS_INFO("指尖执行成功");
            }
            else {
                ROS_INFO("Finger interrupted before completion.");
            }
          }
        }
        catch(ros::Exception)
        {
          ROS_INFO("program interrupted before completion.");
        }

        float pose_x3;
        float pose_y3;
        float pose_z3;
        nh.getParam("/x_under_base",  pose_x3);
        nh.getParam("/y_under_base", pose_y3);
        nh.getParam("/z_under_base", pose_z3);

        pose_z3 = pose_z3 + 0.04;
        double pose_qx3=0.50245;
        double pose_qy3=0.51912;
        double pose_qz3=-0.50306;
        double pose_qw3=-0.47433;

        try
        {
          bool result = Cartesian_client(pose_x3,pose_y3,pose_z3,pose_qx3,pose_qy3,pose_qz3,pose_qw3);
          if(result)
          {
            ROS_INFO("末端执行成功");
          }
        }
        catch(ros::Exception)
        {
          ROS_INFO("program interrupted before completion.");
        }


        double pose_x4 = pose_x3 - 0.20;
        double pose_y4 = pose_y3 + 0.05;
        double pose_z4 = pose_z3;
        double pose_qx4=0.50245;
        double pose_qy4=0.51912;
        double pose_qz4=-0.50306;
        double pose_qw4=-0.47433;

        try
        {
          bool result = Cartesian_client(pose_x4,pose_y4,pose_z4,pose_qx4,pose_qy4,pose_qz4,pose_qw4);
          if(result)
          {
            ROS_INFO("末端执行成功");
          }
        }
        catch(ros::Exception)
        {
          ROS_INFO("program interrupted before completion.");
        }


        double pose_x5 = pose_x3 - 0.20;
        double pose_y5 = pose_y3 + 0.05;
        double pose_z5 = pose_z3 - 0.03;
        double pose_qx5=0.50245;
        double pose_qy5=0.51912;
        double pose_qz5=-0.50306;
        double pose_qw5=-0.47433;

        try
        {
          bool result = Cartesian_client(pose_x5,pose_y5,pose_z5,pose_qx5,pose_qy5,pose_qz5,pose_qw5);
          if(result)
          {
            ROS_INFO("末端执行成功");
            bool result_finger = Finger_client(0,0,0);
            if(result_finger)
            {
              ROS_INFO("指尖执行成功");
            }
          }
        }
        catch(ros::Exception)
        {
          ROS_INFO("program interrupted before completion.");
        }

        double pose_x6 = pose_x5 + 0.07;
        double pose_y6 = pose_y5 - 0.03;
        double pose_z6 = pose_z5 + 0.20;
        double pose_qx6=0.50245;
        double pose_qy6=0.51912;
        double pose_qz6=-0.50306;
        double pose_qw6=-0.47433;

        try
        {
          bool result = Cartesian_client(pose_x6,pose_y6,pose_z6,pose_qx6,pose_qy6,pose_qz6,pose_qw6);
          if(result)
          {
            ROS_INFO("末端执行成功");
          }
        }
        catch(ros::Exception)
        {
          ROS_INFO("program interrupted before completion.");
        }

        ros::ServiceClient start_to_home = nh.serviceClient<kinova_msgs::HomeArm>("j2n6s300_driver/in/home_arm");
        kinova_msgs::HomeArm srv;
        try {
          start_to_home.call(srv);
          ROS_INFO("Go home.");
        } catch (ros::Exception) {
          ROS_INFO("Failed to go home.");

        }

}

