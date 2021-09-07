#include "ros/ros.h"

#include<iostream> //C++标准输入输出库 
#include <vector>
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"
#include <kinova_msgs/ArmPoseActionGoal.h> // 注意格式源文件名字为ArmPose.action
#include <ros/ros.h>
#include <kinova_driver/kinova_ros_types.h>

#include <actionlib/client/simple_action_client.h>
#include <kinova_msgs/SetFingersPositionAction.h>
#include <kinova_msgs/ArmPoseAction.h>
#include <kinova_msgs/HomeArm.h>
#include <kinova_msgs/ArmJointAnglesAction.h>


const int FINGER_MAX1 = 6714;
const int FINGER_MAX2 = 6924;
const int FINGER_MAX3 = 7332;


bool cartesian_pose_client(double x, double y, double z, double qx, double qy, double qz, double qw)//四元数
{


    actionlib::SimpleActionClient <kinova_msgs::ArmPoseAction> *client = new actionlib::SimpleActionClient<kinova_msgs::ArmPoseAction>(
            "/j2n6s300_driver/pose_action/tool_pose");//取的主题

    client->waitForServer();

    kinova_msgs::ArmPoseGoal pose_goal;//注意形式ArmPose.action
    pose_goal.pose.header.frame_id = "j2n6s300_link_base";

    pose_goal.pose.pose.position.x = x;
    pose_goal.pose.pose.position.y = y;
    pose_goal.pose.pose.position.z = z;

    pose_goal.pose.pose.orientation.x = qx;
    pose_goal.pose.pose.orientation.y = qy;
    pose_goal.pose.pose.orientation.z = qz;
    pose_goal.pose.pose.orientation.w = qw;


    client->sendGoal(pose_goal);


    if (client->waitForResult(ros::Duration(200.0)))//延时得高机械臂不会发送，只能自己检测坐标，奇异值也不会告诉所有信息都通过主题反馈
    {
        client->getResult();
        return true;
    } else {
        client->cancelAllGoals();
        ROS_WARN_STREAM("The gripper action timed-out");
        return false;
    }

}

bool finger_control_client(float finger1, float finger2, float finger3)//0-1
{


    actionlib::SimpleActionClient <kinova_msgs::SetFingersPositionAction> *finger_client = new actionlib::SimpleActionClient<kinova_msgs::SetFingersPositionAction>(
            "j2n6s300_driver/fingers_action/finger_positions");

    finger_client->waitForServer();

    kinova_msgs::SetFingersPositionGoal finger_goal;

    finger_goal.fingers.finger1 = finger1 * FINGER_MAX1;
    finger_goal.fingers.finger2 = finger2 * FINGER_MAX2;
    finger_goal.fingers.finger3 = finger3 * FINGER_MAX3;


    finger_client->sendGoal(finger_goal);


    if (finger_client->waitForResult(ros::Duration(150.0))) {
        finger_client->getResult();
        return true;
    } else {
        finger_client->cancelAllGoals();
        ROS_WARN_STREAM("The gripper action timed-out");
        return false;
    }

}

int main(int argc, char **argv) {


    ros::init(argc, argv, "paragram1");
    ros::NodeHandle node_handle;

    ros::ServiceClient start_to_home = node_handle.serviceClient<kinova_msgs::HomeArm>("j2n6s300_driver/in/home_arm");

    kinova_msgs::HomeArm srv;

    try 
    {
        start_to_home.call(srv);
        ROS_INFO("go to home");
    }
    catch (ros::Exception)
    {
        ROS_INFO("Failed to Home.");
    }


    double x, y, z, qx, qy, qz, qw; 
    float finger1,finger2,finger3;

    //double p[3] = {-0.15375, -0.11985, 0.34515};

    //位置1   过渡位置
    x=0.210128158331;
    y=-0.547414700985;
    z=0.503765463829;
    qx=0.623642385006;
    qy=0.355912834406;
    qz=0.459502398968;
    qw=0.522736787796;

    finger1 = finger2 = 0;
    finger3 = 0;
   
   ROS_INFO("Final xyz is %f , %f, %f", x,y,z);

try
   {
    bool result=cartesian_pose_client(x,y,z,qx,qy,qz,qw);
    if(result)
      {
      bool finger_re= finger_control_client(finger1,finger2,finger3);
      ROS_INFO("Cartesian pose sent!.");
      if(finger_re)
          ROS_INFO("finger move!.");
      }
    }
catch(ros::Exception)
     {
      ROS_INFO("program interrupted before completion.");
     }

      //位置2   物体上方
      
      //x=0.464318096638;
      //y=-0.394640952349;
      x=0.414318096638;
      y=-0.344640952349;
      //z= 0.1590745967627;
      z= 0.1690745967627;
      qx= -0.726872444153;
      qy=-0.536810040474;
      qz=-0.428228884935;
      qw= -0.0105513930321;
      
      finger1 = 0;
      finger2 = 0;
      finger3 = 0;

     ROS_INFO("Final xyz is %f , %f, %f", x,y,z);

try
  {
      bool result=cartesian_pose_client(x,y,z,qx,qy,qz,qw);
      if(result)
      {
        bool finger_re= finger_control_client(finger1,finger2,finger3);
        ROS_INFO("Cartesian pose sent!.");
  	if(finger_re)
	     ROS_INFO("finger move!.");
      }
      //sleep(3);
  }
catch(ros::Exception)
       {
        ROS_INFO("program interrupted before completion.");
       }

     //位置3    下落至物体并且闭合手抓
      x=0.454318096638;
      y=-0.384640952349;
      //z= -0.1250745967627;//原来
      z= -0.1050745967627;
      qx= -0.770358204842;
      qy=-0.471246421337;
      qz=-0.428228884935;
      qw= 0.032429471612;


      finger1 = 0.8;
      finger2 = 0.8;
      finger3 = 0.8;

     ROS_INFO("Final xyz is %f , %f, %f", x,y,z);
     

try
 {
      bool result=cartesian_pose_client(x,y,z,qx,qy,qz,qw);
      if(result)
      {
        bool finger_re= finger_control_client(finger1,finger2,finger3);
        ROS_INFO("Cartesian pose sent!.");
        if(finger_re)
             ROS_INFO("finger move!.");
      }
      //sleep(3);
  }
catch(ros::Exception)
       {
        ROS_INFO("program interrupted before completion.");
       }

      //位置5  抬起
      x=0.464318096638;
      y=-0.374640952349;
      //z=-0.0790745967627;
      z=-0.0690745967627;
      qx=-0.770358204842;
      qy=-0.471246421337;
      qz=-0.428228884935;
      qw= 0.032429471612;

      finger1 = 0.8;
      finger2 = 0.8;
      finger3 = 0.8;

     ROS_INFO("Final xyz is %f , %f, %f", x,y,z);
     //sleep(3);
 
  try{

      bool result=cartesian_pose_client(x,y,z,qx,qy,qz,qw);
      if(result)
      {
        bool finger_re= finger_control_client(finger1,finger2,finger3);
        ROS_INFO("Cartesian pose sent!.");
  if(finger_re)ROS_INFO("finger move!.");
      }
  }
  catch(ros::Exception)
       {

        ROS_INFO("program interrupted before completion.");
       }

      //位置6   移至目标点上方
      x=0.264318096638;
      y=-0.394640952349;
      //z= -0.0790745967627;
      z= -0.0690745967627;
      qx= -0.875064074993;
      qy=-0.219145417213;
      qz=-0.400171667337;
      qw= 0.161557257175;

      finger1 = 0.8;
      finger2 = 0.8;
      finger3 = 0.8;

     ROS_INFO("Final xyz is %f , %f, %f", x,y,z);
     //sleep(3);

  try
  {

      bool result=cartesian_pose_client(x,y,z,qx,qy,qz,qw);
      if(result)
      {
        bool finger_re= finger_control_client(finger1,finger2,finger3);
        ROS_INFO("Cartesian pose sent!.");
  	if(finger_re)
	    ROS_INFO("finger move!.");
      }
  }
  catch(ros::Exception)
       {

        ROS_INFO("program interrupted before completion.");
       }

      //位置    下去并释放   
      x=0.264318096638;
      y=-0.384640952349;
      //z= -0.1250745967627;
      z= -0.1410745967627;
      qx= -0.875064074993;
      qy=-0.219145417213;
      qz=-0.400171667337;
      qw= 0.161557257175;

      finger1 = 0;
      finger2 = 0;
      finger3 = 0;

     ROS_INFO("Final xyz is %f , %f, %f", x,y,z);
     //sleep(3);

  try
  {
      bool result=cartesian_pose_client(x,y,z,qx,qy,qz,qw);
      if(result)
      {
        bool finger_re= finger_control_client(finger1,finger2,finger3);
        ROS_INFO("Cartesian pose sent!.");
  if(finger_re)ROS_INFO("finger move!.");
      }
  }
  catch(ros::Exception)
       {
        ROS_INFO("program interrupted before completion.");
       }


//===================================
//位置   回到目标点上方

      x=0.264318096638;
      y=-0.364640952349;
      //z= -0.1250745967627;
      z= 0.1290745967627;
      qx= -0.875064074993;
      qy=-0.219145417213;
      qz=-0.400171667337;
      qw= 0.161557257175;

      finger1 = 0;
      finger2 = 0;
      finger3 = 0;

     ROS_INFO("Final xyz is %f , %f, %f", x,y,z);
     //sleep(3);

  try
  {

      bool result=cartesian_pose_client(x,y,z,qx,qy,qz,qw);
      if(result)
      {
        bool finger_re= finger_control_client(finger1,finger2,finger3);
        ROS_INFO("Cartesian pose sent!.");
  	if(finger_re)
	    ROS_INFO("finger move!.");
      }
  }
  catch(ros::Exception)
       {

        ROS_INFO("program interrupted before completion.");
       }


try {

      start_to_home.call(srv);
      ROS_INFO("go to home");
    }
catch (ros::Exception)
     {
      ROS_INFO("Failed to HOme.");
     }

      return 0;

}


