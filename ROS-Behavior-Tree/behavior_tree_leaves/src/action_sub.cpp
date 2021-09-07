#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib_tutorials/FibonacciAction.h>

/*

using namespace actionlib_tutorials;
typedef actionlib::SimpleActionClient<FibonacciAction> Client;

class MyNode
{
public:
  MyNode() : ac("fibonacci", true)
  {
    ROS_INFO("Waiting for action server to start.");
    ac.waitForServer();
    ROS_INFO("Action server started, sending goal.");
  }

  void doStuff(int order)
  {
    FibonacciGoal goal;
    goal.order = order;

    // Need boost::bind to pass in the 'this' pointer
    ac.sendGoal(goal,
                boost::bind(&MyNode::doneCb, this, _1, _2),
                Client::SimpleActiveCallback(),
                Client::SimpleFeedbackCallback());
  }

  void doneCb(const actionlib::SimpleClientGoalState& state,
              const FibonacciResultConstPtr& result)
  {
    ROS_INFO("Finished in state [%s]", state.toString().c_str());
    ROS_INFO("Answer: %i", result->sequence.back());
    ros::shutdown();
  }

private:
  Client ac;
};

class SubscribeAndPublish
{
public:
  SubscribeAndPublish()
  {
                        //双引号是topic的名字                                       注册回调函数
     sub = n_.subscribe("cartesian_position", 1, &SubscribeAndPublish::callback_, this);
  }

                           //参数要改
  void callback_(const behavior_tree_core::jiyuanGoal& goal)
  {

   x_goal=1;
  }

private:
  ros::NodeHandle n_;
  ros::Subscriber sub;
  double x_goal;
};//End of class SubscribeAndPublish
*/

int main (int argc, char **argv)
{
  ros::init(argc, argv, "test_fibonacci_class_client");
  //SubscribeAndPublish sandp;
  //MyNode my_node;
  //my_node.doStuff(sandp.x_goal);
  ros::spin();
  return 0;
}

