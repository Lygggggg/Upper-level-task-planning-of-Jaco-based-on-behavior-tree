#include <ros/ros.h>
#include <behavior_tree.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sequence_tree");
  try
      {
          //int TickPeriod_milliseconds = 200;
          //int TickPeriod_milliseconds = 2000; // 2秒钟
          //int TickPeriod_milliseconds = 60000; // 1 分钟
          //int TickPeriod_milliseconds = 180000;
          int TickPeriod_milliseconds = 12000000; //20分钟
          BT::ROSCondition* condition_node0 = new BT::ROSCondition("condition0");  //双引号里对应
                                                                     //launch文件里的 nodename
          BT::ROSCondition* condition_node1 = new BT::ROSCondition("condition1");

          BT::ROSAction* action_node0 = new BT::ROSAction("action0");
          BT::ROSAction* action_node1 = new BT::ROSAction("action1");
          BT::ROSAction* action_node2 = new BT::ROSAction("action2");


          BT::SequenceNode* robot_grasp = new BT::SequenceNode("robot_grasp");
          BT::SequenceNode* seq2 = new BT::SequenceNode("seq2");
          BT::SequenceNode* seq3 = new BT::SequenceNode("seq3");
          BT::SequenceNode* seq4 = new BT::SequenceNode("seq4");

          robot_grasp->AddChild(condition_node0);
          robot_grasp->AddChild(seq2);

          seq2->AddChild(action_node0);
          seq2->AddChild(seq3);

          seq3->AddChild(condition_node1);
          seq3->AddChild(seq4);

          seq4->AddChild(action_node1);
          seq4->AddChild(action_node2);

          Execute(robot_grasp, TickPeriod_milliseconds);  // from BehaviorTree.cpp
      }
      catch (BT::BehaviorTreeException& Exception)
      {
          std::cout << Exception.what() << std::endl;
      }

      return 0;
}
