#include <ros/ros.h>
#include <behavior_tree.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ori_behavior_tree");
  try
      {
          int TickPeriod_milliseconds = 200;
          //int TickPeriod_milliseconds = 2000; // 2秒钟

          //int TickPeriod_milliseconds = 180000; //3分钟
          //int TickPeriod_milliseconds = 3000000; //5分钟
          //int TickPeriod_milliseconds = 6000000; //10分钟
          BT::ROSCondition* condition_node = new BT::ROSCondition("condition");  //双引号里对应
                                                                     //launch文件里的 nodename
          BT::ROSAction* action_node = new BT::ROSAction("action");
         

          BT::SequenceNode* robot_grasp = new BT::SequenceNode("robot_grasp");

          robot_grasp->AddChild(condition_node);
          robot_grasp->AddChild(action_node);

          

          Execute(robot_grasp, TickPeriod_milliseconds);  // from BehaviorTree.cpp
      }
      catch (BT::BehaviorTreeException& Exception)
      {
          std::cout << Exception.what() << std::endl;
      }

      return 0;
}
