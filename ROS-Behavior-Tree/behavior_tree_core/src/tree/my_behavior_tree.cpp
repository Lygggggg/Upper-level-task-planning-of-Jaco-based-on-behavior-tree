#include <ros/ros.h>
#include <behavior_tree.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "my_behavior_tree");
  try
      {
          //int TickPeriod_milliseconds = 200;
          //int TickPeriod_milliseconds = 2000; // 2秒钟
          int TickPeriod_milliseconds = 36000000; //60分钟
          BT::ROSCondition* condition_node0 = new BT::ROSCondition("condition0");  //双引号里对应
                                                                     //launch文件里的 nodename
          BT::ROSCondition* condition_node1 = new BT::ROSCondition("condition1");
          BT::ROSCondition* condition_node2 = new BT::ROSCondition("condition2");
          BT::ROSCondition* condition_node3 = new BT::ROSCondition("condition3");
          BT::ROSCondition* condition_node4 = new BT::ROSCondition("condition4");
          BT::ROSCondition* condition_node5 = new BT::ROSCondition("condition5");
          BT::ROSCondition* condition_node6 = new BT::ROSCondition("condition6");

          BT::ROSAction* action_node0 = new BT::ROSAction("action0");
          BT::ROSAction* action_node1 = new BT::ROSAction("action1");
          BT::ROSAction* action_node2 = new BT::ROSAction("action2");
          BT::ROSAction* action_node3 = new BT::ROSAction("action3");
          BT::ROSAction* action_node4 = new BT::ROSAction("action4");
          BT::ROSAction* action_node5 = new BT::ROSAction("action5");
          BT::ROSAction* action_node6 = new BT::ROSAction("action6");

          BT::SequenceNode* robot_grasp = new BT::SequenceNode("robot_grasp");

          robot_grasp->AddChild(condition_node0);
          robot_grasp->AddChild(action_node0);

          robot_grasp->AddChild(condition_node1);
          robot_grasp->AddChild(action_node1);

          robot_grasp->AddChild(condition_node2);
          robot_grasp->AddChild(action_node2);

          robot_grasp->AddChild(condition_node3);
          robot_grasp->AddChild(action_node3);

          robot_grasp->AddChild(condition_node4);
          robot_grasp->AddChild(action_node4);

          robot_grasp->AddChild(condition_node5);
          robot_grasp->AddChild(action_node5);

          robot_grasp->AddChild(condition_node6);
          robot_grasp->AddChild(action_node6);

          Execute(robot_grasp, TickPeriod_milliseconds);  // from BehaviorTree.cpp
      }
      catch (BT::BehaviorTreeException& Exception)
      {
          std::cout << Exception.what() << std::endl;
      }

      return 0;
}
