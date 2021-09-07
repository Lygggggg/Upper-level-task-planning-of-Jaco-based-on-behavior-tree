#include <ros/ros.h>
#include <behavior_tree.h>
#include <decorators/negation_node.h>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "new_tree_like_guyue");
  try
      {
          int TickPeriod_milliseconds = 5000;
//          int TickPeriod_milliseconds = 180000; // 3分钟
//          int TickPeriod_milliseconds = 36000000; //60分钟
          BT::ROSCondition* condition_node = new BT::ROSCondition("have_object");  //双引号里对应
          BT::ROSCondition* condition_node2 = new BT::ROSCondition("if_release");
                                                                     //launch文件里的 nodename

          BT::ROSAction* action_node1 = new BT::ROSAction("close_gripper");
          BT::ROSAction* action_node2 = new BT::ROSAction("pick_obj");
          BT::ROSAction* action_node3 = new BT::ROSAction("place_obj");
          BT::ROSAction* action_node4 = new BT::ROSAction("open_gripper");
          BT::ROSAction* action_node5 = new BT::ROSAction("go_home");

          BT::FallbackNode* root = new BT::FallbackNode("root");
          BT::SequenceNodeWithMemory* grasp = new BT::SequenceNodeWithMemory("grasping");
          BT::SequenceNodeWithMemory* release = new BT::SequenceNodeWithMemory("release");
//          BT::DecoratorNode* decorator = new BT::DecoratorNode("decorator");

          BT::NegationNode* decorator = new BT::NegationNode("decorator");

          root->AddChild(grasp);
          root->AddChild(action_node2);

          grasp->AddChild(condition_node);
          grasp->AddChild(action_node1);
          grasp->AddChild(release);

          release->AddChild(decorator);
          decorator->AddChild(condition_node2);

          release->AddChild(action_node3);
          release->AddChild(action_node4);
          release->AddChild(action_node5);

          Execute(root, TickPeriod_milliseconds);  // from BehaviorTree.cpp
      }
      catch (BT::BehaviorTreeException& Exception)
      {
          std::cout << Exception.what() << std::endl;
      }

      return 0;
}
