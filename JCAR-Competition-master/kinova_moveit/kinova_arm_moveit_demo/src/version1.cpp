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

bool finger_control_client(float finger1, float finger2, float finger3)//0-1
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
    float Tf_Mat[4][4];

    const float tf[16] = {-0.7856,0.4983,0.3759,0.1077,0.1517,-0.6621,0.9137,-0.8502,0.6109,0.5212,0.2131,0.5638, 0, 0, 0, 1};
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            Tf_Mat[i][j] = tf[i*4+j];

        }
    }
//    ROS_INFO("I heard the pose from the robot"); 
    ROS_INFO("the marker pose with cam_frame position(x,y,z) is %f , %f, %f", msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);


    std::cout<<"\n \n"<<std::endl; 
    float x, y, z, qx, qy, qz, qw, finger1, finger2, finger3, p[4], pm[4] = {msg->pose.position.x, msg->pose.position.y,  msg->pose.position.z, 1};
    ROS_INFO("the marker pose with pm position(x,y,z) is %f , %f, %f", pm[0], pm[1], pm[2]);

    for (int i = 0; i < 4; ++i) {
        p[i] = Tf_Mat[i][0]*pm[0]+Tf_Mat[i][1]*pm[1]+Tf_Mat[i][2]*pm[2]+Tf_Mat[i][3]*pm[3];
    }
    x=p[0]+0.035;
    y=p[1]+0.01;//y可能需要补偿4cm y的原值为-0.528081440518
    z=-0.035;
    qx= -0.772631883621;
    qy= -0.471870988607;
    qz=-0.417701870203;
    qw= 0.0768298059702;
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



    x=p[0]+0.03;
    y=p[1]+0.01;//y可能需要补偿4cm y的原值为-0.528081440518
    z=-0.15;
    qx= -0.772631883621;
    qy= -0.471870988607;
    qz=-0.417701870203;
    qw= 0.0768298059702;
    finger1 = finger2 = 1;
    finger3 = 1;


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

int main(int argc, char **argv) {


    ros::init(argc, argv, "paragram1");
    ros::NodeHandle node_handle;

    ros::ServiceClient start_to_home = node_handle.serviceClient<kinova_msgs::HomeArm>("j2n6s300_driver/in/home_arm");

    kinova_msgs::HomeArm srv;

    try {

        start_to_home.call(srv);
        ROS_INFO("go to home");

    }
    catch (ros::Exception) {
        ROS_INFO("Failed to HOme.");
    }




    double x, y, z, qx, qy, qz, qw; 
    float finger1,finger2,finger3;

    //double p[3] = {-0.15375, -0.11985, 0.34515};
    x=0.210128158331;
    y= -0.547414700985;
    z=0.503765463829;
    qx=0.623642385006;
    qy=0.355912834406;
    qz=0.459502398968;
    qw=0.522736787796;
    finger1 = finger2 = 0;
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






    ros::Subscriber Marke_sub = node_handle.subscribe("aruco_tracker/pose", 1, MarkerCallback);
    ros::spin();
    return 0;
}


