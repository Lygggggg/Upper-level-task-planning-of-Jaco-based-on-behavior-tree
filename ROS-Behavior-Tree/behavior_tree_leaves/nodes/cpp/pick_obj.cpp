/*
 * c++版本的DMP泛化性能太差了！！！
*/

#include <ros/ros.h>
//#include "dmp/dmp.h"
#include <actionlib/server/simple_action_server.h>
#include <behavior_tree_core/BTAction.h>

#include <fstream>
#include <sstream>
#include <iostream>
#include <string>
#include <vector>
#include "std_msgs/String.h"
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_listener.h>
#include <tf2_ros/buffer.h>
//#include "dmp/waypoints.h"
#include <python2.7/Python.h>

//using namespace dmp;
using namespace std;

enum Status {RUNNING, SUCCESS, FAILURE};  // BT return status


class BTAction
{
protected:
    ros::NodeHandle nh_;
    ros::NodeHandle new_nh_;
    // NodeHandle instance must be created before this line. Otherwise strange error may occur.
    actionlib::SimpleActionServer<behavior_tree_core::BTAction> as_;
    std::string action_name_;
    // create messages that are used to published feedback/result
    behavior_tree_core::BTFeedback feedback_;  // action feedback (SUCCESS, FAILURE)
    behavior_tree_core::BTResult result_;  // action feedback  (same as feedback for us)


private:
    moveit::planning_interface::MoveGroupInterface* move_group_;
    tf::TransformListener* listener;
    //dmpClass dmpc;
public:
    explicit BTAction(std::string name) :
//      as_(nh_, name, false),
      as_(nh_, name, boost::bind(&BTAction::execute_callback, this, _1), false),
      action_name_(name)
    {
      move_group_ = new moveit::planning_interface::MoveGroupInterface("arm");
      listener = new tf::TransformListener;
      move_group_->setPoseReferenceFrame("j2n6s300_link_base");
      move_group_->setMaxVelocityScalingFactor(0.6);
      move_group_->setMaxAccelerationScalingFactor(0.04);
      // Starts the action server
      as_.start();
    }

    ~BTAction(void)
    {
      delete move_group_;
      delete listener;
    }

    void execute_callback(const behavior_tree_core::BTGoalConstPtr &goal)
    {
      std::cout<<"-----------this is the prompt-------------"<<std::endl;

      move_group_->setNamedTarget("Home");
      move_group_->move();

      std::string jiyuan_path = "/home/lni/Ros_ws/catkin_ws/src/dmp/data/jiyuan/";
      for (int i = 0; i<2;i++)
      {
        std::string file_name = (jiyuan_path+"jiyuan"+to_string(i)+".txt");
        vector<vector<double>> jiyuan_content = load_jiyuan(file_name);
        geometry_msgs::PoseStamped current_pose = get_robot_state();
        std::vector<double> robot_start;
        std::vector<double> robot_end;
        robot_start.push_back(current_pose.pose.position.x);
        robot_start.push_back(current_pose.pose.position.y);
        robot_start.push_back(current_pose.pose.position.z);
        robot_start.push_back(current_pose.pose.orientation.x);
        robot_start.push_back(current_pose.pose.orientation.y);
        robot_start.push_back(current_pose.pose.orientation.z);
        robot_start.push_back(current_pose.pose.orientation.w);
        int row = jiyuan_content.size();
        robot_end = jiyuan_content[row - 1];

        ROS_INFO("Get jiyuan_start and jiyuan_end");
        std::cout<<file_name<<std::endl;

        std::vector<geometry_msgs::Pose> way_points;
        try {
          ROS_INFO("DMP generalization in progress");
          way_points = dmp_(file_name, robot_start, robot_end);
        } catch (exception& e) {
          cout<<e.what()<<endl;
        }

        ROS_INFO("-----------------------Going to move jaco");
        try {
          bool flag = move_jaco(way_points);
          if(flag)
          {
            ROS_INFO("----------------------Jaco moved");
          }
        } catch (exception& e) {
          cout<<e.what()<<endl;
        }
      }

      std::string file_name = (jiyuan_path+"jiyuan"+to_string(2)+".txt");
      vector<vector<double>> jiyuan_content = load_jiyuan(file_name);
      geometry_msgs::PoseStamped current_pose = get_robot_state();
      std::vector<double> robot_start;
      std::vector<double> object_position;
      robot_start.push_back(current_pose.pose.position.x);
      robot_start.push_back(current_pose.pose.position.y);
      robot_start.push_back(current_pose.pose.position.z);
      robot_start.push_back(current_pose.pose.orientation.x);
      robot_start.push_back(current_pose.pose.orientation.y);
      robot_start.push_back(current_pose.pose.orientation.z);
      robot_start.push_back(current_pose.pose.orientation.w);
      int row = jiyuan_content.size();
      double getParam_pose_x;
      double getParam_pose_y;
      double getParam_pose_z;
      new_nh_.getParam("/x_under_base", getParam_pose_x);
      new_nh_.getParam("/y_under_base", getParam_pose_y);
      new_nh_.getParam("/z_under_base", getParam_pose_z);
      getParam_pose_z += 0.07;
      object_position = {getParam_pose_x,getParam_pose_y,getParam_pose_z};
      for (int i=3;i<7;i++) {
        object_position.push_back(jiyuan_content[row - 1][i]);
      }

      ROS_INFO("Get jiyuan_start and object_position");
      std::cout<<file_name<<std::endl;

      std::vector<geometry_msgs::Pose> way_points;
      try {
        ROS_INFO("DMP generalization in progress");
        way_points = dmp_(file_name, robot_start, object_position);
      } catch (exception& e) {
        cout<<e.what()<<endl;
      }

      ROS_INFO("-----------------------Going to move jaco");
      try {
        bool flag = move_jaco(way_points);
        if(flag)
        {
          ROS_INFO("----------------------Jaco moved");
        }
      } catch (exception& e) {
        cout<<e.what()<<endl;
      }
      set_status(SUCCESS);

    }

    std::vector<geometry_msgs::Pose> dmp_(std::string file_Path, std::vector<double> start_point, std::vector<double> end_point)
    {
      try
      {
        Py_Initialize();
        if( !Py_IsInitialized())
        {
        cout << "python init fail" << endl;
        exit(EXIT_FAILURE);
        }
      }
      catch (exception& e)
      {
        std::cout<<e.what()<<std::endl;
      }

      PyObject *pMoudle, *pFunc;
      PyObject *pArgs;


      //导入python dmp模块
      string path = "/home/lni/Ros_ws/catkin_ws/src/ROS-Behavior-Tree/behavior_tree_leaves/nodes/cpp";
      string chdir_cmd = string("sys.path.append(\"") + path + "\")";
      const char* cstr_cmd = chdir_cmd.c_str();
      PyRun_SimpleString("import sys");
      PyRun_SimpleString(cstr_cmd);
      pMoudle = PyImport_Import(PyString_FromString("dmp_jiyuan"));
      pFunc = PyObject_GetAttrString(pMoudle, "DMP_generation");

      pArgs = PyTuple_New(3);// dmp模块3个参数

      PyObject* p_start = PyList_New(7);
      PyObject* p_end = PyList_New(7);
      for (int item=0;item<7;item++)
      {
        PyList_SetItem(p_start,item, PyFloat_FromDouble(start_point[item]));
        PyList_SetItem(p_end,item, PyFloat_FromDouble(end_point[item]));
      }

      const char* Path = file_Path.c_str();
      PyTuple_SetItem(pArgs,0, PyString_FromString(Path));
      PyTuple_SetItem(pArgs,1,p_start);
      PyTuple_SetItem(pArgs,2,p_end);

      std::vector<double> list_points;
      PyObject * pValue;

      pValue = PyObject_CallObject(pFunc, pArgs);
      if (pValue)
      {
        cout<<"got pValue correctly!"<<endl;
        if(PyList_Check(pValue))
           {
                cout<<"pValue is list"<<endl;
                int SizeOfList = PyList_Size(pValue);//SizeOfList = 875
                cout<<"got SizeOfList: "<<SizeOfList<<endl; //sizeOfList = 875
                for(int i = 0; i < SizeOfList; i++)
                {
                    PyObject *ListItem = PyList_GetItem(pValue, i);//获取List对象中的每一个元素
                    double result = PyFloat_AsDouble(ListItem);
    //                cout<<"got ListItem "<<i <<": "<<result<<endl;
                    list_points.push_back(result);
                }
            }
        else
           ROS_ERROR("something wrong!");
      }
      else
      {
        ROS_ERROR("pValue is null!");
        Py_Finalize();
        exit(EXIT_FAILURE);;
      }
    //  way_point.resize(125,7);
      int x = 0;
      int size = list_points.size()/7;
      float result[size][7];
      geometry_msgs::Pose target_point;
      std::vector<geometry_msgs::Pose> way_points;
      for (int i = 0;i<size;i++)
      {
        for (int j = 0;j<7;j++)
        {
          result[i][j] = list_points[x];
          x++;
        }
        target_point.position.x = result[i][0];
        target_point.position.y = result[i][1];
        target_point.position.z = result[i][2];
        target_point.orientation.x = result[i][3];
        target_point.orientation.y = result[i][4];
        target_point.orientation.z = result[i][5];
        target_point.orientation.w = result[i][6];
        way_points.push_back(target_point);
      }
      return way_points;
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

    bool move_jaco(std::vector<geometry_msgs::Pose> way_points_)
    {
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
        fraction = move_group_->computeCartesianPath(way_points_, eef_step, jump_threshold, trajectory);
        int tra_size = trajectory.joint_trajectory.points.size();
        double dt = 0.0;
        for (int i = 0;i< tra_size; i++) {
          trajectory.joint_trajectory.points[i].time_from_start = ros::Duration(dt);
          dt += 0.04;
        }
        my_plan.trajectory_ = trajectory;
        ROS_INFO_NAMED("tutorial", "Visualizing plan(Cartesian path) (%.2f%% acheived)", fraction * 100.0);
        move_group_->execute(my_plan);
        result = (move_group_->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        ros::WallDuration(1).sleep();
      }
      if(n_attempts==4 && result == false)
      {
        ROS_ERROR("Execute failure");
      }
      return result;
    }

    vector<vector<double>> load_jiyuan(std::string dataPath)
    {
      std::vector<double> vec;
      std::vector<std::vector<double>> vvec;
      std::ifstream is(dataPath);
      while (!is.eof())
      {
        std::string line;
        std::getline(is, line);

        std::stringstream ss(line);
        for(int j=0; j<7; j++)
        {
          double d;
          ss >> d;
          vec.push_back(d);
        }
        vvec.push_back(vec);
        vec.clear();
      }

      is.close();
      vvec.pop_back();
      return vvec;
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
    ros::init(argc, argv, "pick_obj");
    ROS_INFO(" Enum: %d", RUNNING);
    ROS_INFO(" Action Ready for Ticks");
    BTAction bt_action(ros::this_node::getName());
    ros::spin();

    return 0;
}
