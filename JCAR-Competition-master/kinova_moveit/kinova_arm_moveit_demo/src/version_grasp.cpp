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

const int FINGER_MAX = 6400;
bool cartesian_pose_client(float x, float y, float z, float qx, float qy, float qz, float qw)//四元数
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


    if (client->waitForResult(ros::Duration(150.0)))//延时得高机械臂不会发送，只能自己检测坐标，奇异值也不会告诉所有信息都通过主题反馈
    {
        client->getResult();
        return true;
    } else {
        client->cancelAllGoals();
        ROS_WARN_STREAM("The gripper action timed-out");
        return false;
    }

}

bool finger_control_client(float finger1, float finger2, float finger3)
{


    actionlib::SimpleActionClient <kinova_msgs::SetFingersPositionAction> *finger_client = new actionlib::SimpleActionClient<kinova_msgs::SetFingersPositionAction>(
            "j2n6s300_driver/fingers_action/finger_positions"); 

    finger_client->waitForServer();

    kinova_msgs::SetFingersPositionGoal finger_goal;

    finger_goal.fingers.finger1 = finger1 * FINGER_MAX; 
    finger_goal.fingers.finger2 = finger2 * FINGER_MAX;
    finger_goal.fingers.finger3 = finger3 * FINGER_MAX;


    finger_client->sendGoal(finger_goal);


    if (finger_client->waitForResult(ros::Duration(100.0))) {
        finger_client->getResult();
        return true;
    } else {
        finger_client->cancelAllGoals();
        ROS_WARN_STREAM("The gripper action timed-out");
        return false;
    }

}

void MarkerCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    float Tf_Mat[3][3];

    const float tf[9] = {0.9993,0.0202,0.0302,0.0350,-0.7633,-0.6451,0.0100,0.6458,-0.6435};
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j) 
        {
            Tf_Mat[i][j] = tf[i*3+j];
        }
    }
//    ROS_INFO("I heard the pose from the robot"); 
    ROS_INFO("the marker pose with cam_frame position(x,y,z) is %f , %f, %f", msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);


    std::cout<<"\n \n"<<std::endl; 
    float x, y, z, qx, qy, qz, qw, finger1, finger2, finger3, p[3], pm[3] = {msg->pose.position.x, msg->pose.position.y,  msg->pose.position.z};

    float make_up_x=0.1543;
    float make_up_y=-0.18;

    ROS_INFO("the marker pose with pm position(x,y,z) is %f , %f, %f", pm[0], pm[1], pm[2]);

    for (int i = 0; i < 3; ++i) 
    {
        p[i] = Tf_Mat[i][0]*pm[0]+Tf_Mat[i][1]*pm[1]+Tf_Mat[i][2]*pm[2];
    }


//识别
    x=p[0]+make_up_x;
    y=p[1]+make_up_y;
    z=0.50;

    qx=0.269837409258;
    qy=0.0576613768935;
    qz=0.668386757374;
    qw=0.690740227699;
    finger1 = finger2 = 0;
    finger3 = 0;


    try {

        bool result = cartesian_pose_client(x, y, z, qx, qy, qz, qw);
        if (result) {
            bool finger_re = finger_control_client(finger1, finger2, finger3);

            ROS_INFO("Cartesian pose sent! %f , %f, %f", x, y, z);
            if (finger_re)ROS_INFO("finger move!.");
        }
    }
    catch (ros::Exception) {
        ROS_INFO("program interrupted before completion.");
    }


//抓取
    x=p[0]+make_up_x;
    y=p[1]+make_up_y;
    z=0.45;

    qx=0.269837409258;
    qy=0.0576613768935;
    qz=0.668386757374;
    qw=0.690740227699;
    finger1 = finger2 = 0.5;
    finger3 = 0.5;


    try {

        bool result = cartesian_pose_client(x, y, z, qx, qy, qz, qw);
        if (result) {
            bool finger_re = finger_control_client(finger1, finger2, finger3);

            ROS_INFO("Cartesian pose sent! %f , %f, %f", x, y, z);
            if (finger_re)ROS_INFO("finger move!.");
        }
    }
    catch (ros::Exception) {
        ROS_INFO("program interrupted before completion.");
    }



//抬起
   x=p[0]+make_up_x;
    y=p[1]+make_up_y;
    z=0.50;

    qx=0.269837409258;
    qy=0.0576613768935;
    qz=0.668386757374;
    qw=0.690740227699;
    finger1 = finger2 = 0.6;
    finger3 = 0.6;


    try {

        bool result = cartesian_pose_client(x, y, z, qx, qy, qz, qw);
        if (result) {
            bool finger_re = finger_control_client(finger1, finger2, finger3);

            ROS_INFO("Cartesian pose sent! %f , %f, %f", x, y, z);
            if (finger_re)ROS_INFO("finger move!.");
        }
    }
    catch (ros::Exception) {
        ROS_INFO("program interrupted before completion.");
    }


//放置
    x=p[0]+make_up_x;
    y=p[1]+make_up_y;
    z=0.45;

    qx=0.269837409258;
    qy=0.0576613768935;
    qz=0.668386757374;
    qw=0.690740227699;
    finger1 = finger2 = 0;
    finger3 = 0;


    try {

        bool result = cartesian_pose_client(x, y, z, qx, qy, qz, qw);
        if (result) {
            bool finger_re = finger_control_client(finger1, finger2, finger3);

            ROS_INFO("Cartesian pose sent! %f , %f, %f", x, y, z);
            if (finger_re)ROS_INFO("finger move!.");
        }
    }
    catch (ros::Exception) {
        ROS_INFO("program interrupted before completion.");
    }





}

int main(int argc, char **argv) 
{

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
      ROS_INFO("Failed to HOme.");
 }


    //将机械臂移动到一个安全位置，防止机械臂发生自身碰撞
 double x, y, z, qx, qy, qz, qw; 
 float finger1,finger2,finger3;
 x=0.210128158331;
 y= -0.5514700985;
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
     if(finger_re)ROS_INFO("finger move!.");
 }
 }
 catch(ros::Exception)
 {

     ROS_INFO("program interrupted before completion.");
 }


//使机械臂竖直向下
      x= 0.0383871689439;
      y=-0.691646237373;
      z= 0.592377603054;
      qx= 0.251010119915;
      qy=0.100153110921;
      qz=0.560334682465;
      qw= 0.782935798168;

      finger1 = 0;
      finger2 = 0;
      finger3 = 0;

     ROS_INFO("Final xyz is %f , %f, %f", x,y,z);

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





    //调用aruco_marker话题
    ros::Subscriber Marke_sub = node_handle.subscribe("aruco_tracker/pose", 1, MarkerCallback);

   


    ros::spin();
    return 0;
}


