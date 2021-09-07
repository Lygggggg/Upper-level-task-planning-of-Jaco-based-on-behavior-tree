#include <pick_place.h>
#include <ros/console.h>

#include <tf_conversions/tf_eigen.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <tf2_ros/transform_listener.h>
const double FINGER_MAX = 6400;

using namespace kinova;

tf::Quaternion EulerZYZ_to_Quaternion(double tz1, double ty, double tz2)
{
    tf::Quaternion q;
    tf::Matrix3x3 rot;
    tf::Matrix3x3 rot_temp;
    rot.setIdentity();

    rot_temp.setEulerYPR(tz1, 0.0, 0.0);
    rot *= rot_temp;
    rot_temp.setEulerYPR(0.0, ty, 0.0);
    rot *= rot_temp;
    rot_temp.setEulerYPR(tz2, 0.0, 0.0);
    rot *= rot_temp;
    rot.getRotation(q);
    return q;
}

PickPlace::PickPlace(ros::NodeHandle &nh):
    nh_(nh),tfListener(tfBuffer)
{	
    ros::NodeHandle pn("~");

    nh_.param<std::string>("/robot_type",robot_type_,"j2n6s300");
    nh_.param<bool>("/robot_connected",robot_connected_,false);

    if (robot_connected_)
    {
        sub_pose_ = nh_.subscribe<geometry_msgs::PoseStamped>("/" + robot_type_ +"_driver/out/tool_pose", 1, &PickPlace::get_current_pose, this);
    }

    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model_ = robot_model_loader.getModel();

    planning_scene_.reset(new planning_scene::PlanningScene(robot_model_));
    planning_scene_monitor_.reset(new planning_scene_monitor::PlanningSceneMonitor("robot_description"));

    ROS_INFO("--------LOADING1--------");

    group_ = new moveit::planning_interface::MoveGroupInterface("arm");
    gripper_group_ = new moveit::planning_interface::MoveGroupInterface("gripper");
		
    group_->setEndEffectorLink(robot_type_ + "_end_effector");

    finger_client_ = new actionlib::SimpleActionClient<kinova_msgs::SetFingersPositionAction>
            ("/" + robot_type_ + "_driver/fingers_action/finger_positions", false);

    pub_co_ = nh_.advertise<moveit_msgs::CollisionObject>("/collision_object", 10);
    pub_aco_ = nh_.advertise<moveit_msgs::AttachedCollisionObject>("/attached_collision_object", 10);
    pub_planning_scene_diff_ = nh_.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);

    int arm_joint_num = robot_type_[3]-'0';
    joint_names_.resize(arm_joint_num);
    joint_values_.resize(joint_names_.size());
    for (uint i = 0; i<joint_names_.size(); i++)
    {
        joint_names_[i] = robot_type_ + "_joint_" + boost::lexical_cast<std::string>(i+1);
    }
    kinova_gripper_joint_model_group = group_->getCurrentState()->getJointModelGroup("gripper");
    define_cartesian_pose();
    define_joint_values();

    result_ = false;

    group_->setNamedTarget("Home");
    ROS_INFO("Go to the Home position");
    result_ = static_cast<bool>(group_->plan(kinova_plan));
    group_->execute(kinova_plan);
//    evaluate_plan(*group_);

//    ros::WallDuration(0.5).sleep();
		
    OperateGripper(false);

//    group_->setPoseTarget(intermedia_pt_0);
//    result_ = static_cast<bool>(group_->plan(kinova_plan));
//    group_->execute(kinova_plan);
//    //evaluate_plan(*group_);

//    ros::WallDuration(1.0).sleep();

//    group_->setPoseTarget(intermedia_pt_1);
//    result_ = static_cast<bool>(group_->plan(kinova_plan));
//    group_->execute(kinova_plan);
//    //evaluate_plan(*group_);

//    ros::WallDuration(1.0).sleep();

    group_->setPoseTarget(intermedia_pt_2);
    result_ = static_cast<bool>(group_->plan(kinova_plan));
    group_->execute(kinova_plan);
//    evaluate_plan(*group_);

    ros::WallDuration(5.0).sleep();

    group_->setPoseTarget(intermedia_pt_3);
    result_ = static_cast<bool>(group_->plan(kinova_plan));
    group_->execute(kinova_plan);
//    evaluate_plan(*group_);

//    ros::WallDuration(1).sleep();

    group_->setPoseTarget(intermedia_pt_4);
    result_ = static_cast<bool>(group_->plan(kinova_plan));
    group_->execute(kinova_plan);

}


PickPlace::~PickPlace()
{
    pub_co_.shutdown();
    pub_aco_.shutdown();
    pub_planning_scene_diff_.shutdown();

    // release memory
    delete group_;
    delete gripper_group_;
    delete finger_client_;
}

void PickPlace::get_current_state(const sensor_msgs::JointStateConstPtr &msg)
{
    boost::mutex::scoped_lock lock(mutex_state_);
    current_state_ = *msg;
}

void PickPlace::get_current_pose(const geometry_msgs::PoseStampedConstPtr &msg)
{
    boost::mutex::scoped_lock lock(mutex_pose_);
    current_pose_ = *msg;
}

void PickPlace::define_cartesian_pose()
{
    intermedia_pt_0.header.frame_id = "j2n6s300_link_base";
    intermedia_pt_0.header.stamp = ros::Time::now();
    intermedia_pt_0.pose.position.x = 0.0;
    intermedia_pt_0.pose.position.y = 0.0;
    intermedia_pt_0.pose.position.z = 0.89836;
    intermedia_pt_0.pose.orientation.x = 0.71602;
    intermedia_pt_0.pose.orientation.y = 0.0;
    intermedia_pt_0.pose.orientation.z = 0.69808;
    intermedia_pt_0.pose.orientation.w = 0.0;

    intermedia_pt_1.header.frame_id = "j2n6s300_link_base";
    intermedia_pt_1.header.stamp = ros::Time::now();
    intermedia_pt_1.pose.position.x = 0.085989;
    intermedia_pt_1.pose.position.y = 0.0;
    intermedia_pt_1.pose.position.z = 0.6272;
    intermedia_pt_1.pose.orientation.x = 0.71851;
    intermedia_pt_1.pose.orientation.y = 0.0;
    intermedia_pt_1.pose.orientation.z = 0.69551;
    intermedia_pt_1.pose.orientation.w = 0.0;

    intermedia_pt_2.header.frame_id = "j2n6s300_link_base";
    intermedia_pt_2.header.stamp = ros::Time::now();
    intermedia_pt_2.pose.position.x = 0.35653;
    intermedia_pt_2.pose.position.y = 0.0;
    intermedia_pt_2.pose.position.z = 0.33787;
    intermedia_pt_2.pose.orientation.x = 1.0;
    intermedia_pt_2.pose.orientation.y = 0.0;
    intermedia_pt_2.pose.orientation.z = 0.0;
    intermedia_pt_2.pose.orientation.w = 0.0;

    intermedia_pt_3.header.frame_id = "j2n6s300_link_base";
    intermedia_pt_3.header.stamp = ros::Time::now();
    intermedia_pt_3.pose.position.x = 0.35704;
    intermedia_pt_3.pose.position.y = 0.34392;
    intermedia_pt_3.pose.position.z = 0.33698;
    intermedia_pt_3.pose.orientation.x = 1.0;
    intermedia_pt_3.pose.orientation.y = 0.0;
    intermedia_pt_3.pose.orientation.z = 0.0;
    intermedia_pt_3.pose.orientation.w = 0.0;

    intermedia_pt_4.header.frame_id = "j2n6s300_link_base";
    intermedia_pt_4.header.stamp = ros::Time::now();
    intermedia_pt_4.pose.position.x = 0.46955;
    intermedia_pt_4.pose.position.y = 0.3427;
    //intermedia_pt_4.pose.position.z = 0.62178;
    intermedia_pt_4.pose.position.z = 0.12178;
    intermedia_pt_4.pose.orientation.x = 1.0;
    intermedia_pt_4.pose.orientation.y = 0.0;
    intermedia_pt_4.pose.orientation.z = 0.0;
    intermedia_pt_4.pose.orientation.w = 0.0;

}

void PickPlace::define_joint_values()
{
    start_joint_.resize(joint_names_.size());
    //    getInvK(start_pose_, start_joint_);
    start_joint_[0] = 0 *M_PI/180.0;
    start_joint_[1] = 135 *M_PI/180.0;
    start_joint_[2] = 0 *M_PI/180.0;
    start_joint_[3] = 135 *M_PI/180.0;
    start_joint_[4] = 0 *M_PI/180.0;
    start_joint_[5] = 135 *M_PI/180.0;


    grasp_joint_.resize(joint_names_.size());
//    getInvK(grasp_pose, grasp_joint_);
    grasp_joint_[0] = 144.0 *M_PI/180.0;
    grasp_joint_[1] = 256.5 *M_PI/180.0;
    grasp_joint_[2] = 91.3 *M_PI/180.0;
    grasp_joint_[3] = 163.8 *M_PI/180.0;
    grasp_joint_[4] = 88.5 *M_PI/180.0;
    grasp_joint_[5] = 151.3 *M_PI/180.0;

    pregrasp_joint_.resize(joint_names_.size());
//    getInvK(pregrasp_pose, pregrasp_joint_);
    pregrasp_joint_[0] = 145.4 *M_PI/180.0;
    pregrasp_joint_[1] = 253.7 *M_PI/180.0;
    pregrasp_joint_[2] = 67.0 *M_PI/180.0;
    pregrasp_joint_[3] = 151.0 *M_PI/180.0;
    pregrasp_joint_[4] = 118.5 *M_PI/180.0;
    pregrasp_joint_[5] = 141.7 *M_PI/180.0;

//    postgrasp_joint_ = pregrasp_joint_;
    postgrasp_joint_.resize(joint_names_.size());
//    getInvK(pregrasp_pose, postgrasp_joint_);
    postgrasp_joint_[0] = 144 *M_PI/180.0;
    postgrasp_joint_[1] = 249 *M_PI/180.0;
    postgrasp_joint_[2] = 88 *M_PI/180.0;
    postgrasp_joint_[3] = 165 *M_PI/180.0;
    postgrasp_joint_[4] = 83 *M_PI/180.0;
    postgrasp_joint_[5] = 151 *M_PI/180.0;
}

void PickPlace::evaluate_plan(moveit::planning_interface::MoveGroupInterface &group)
{
    bool replan = true;
    int count = 0;

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    while (replan == true && ros::ok())
    {
        count = 0;
        result_ = false;

        double plan_time;
        while (result_ == false && count < 5)
        {
            count++;
            plan_time = 20+count*10;
            ROS_INFO("Setting plan time to %f sec", plan_time);
            group.setPlanningTime(plan_time);
            result_ = static_cast<bool>(group.plan(my_plan));
            std::cout << "at attemp: " << count << std::endl;
            ros::WallDuration(0.1).sleep();
        }

        if (result_ == true)
        {
            std::cout << "plan success at attemp: " << count << std::endl;

            replan = false;
            std::cout << "please input e to execute the plan, r to replan, others to skip: ";
            std::cin >> pause_;
            ros::WallDuration(0.5).sleep();
            if (pause_ == "r" || pause_ == "R" )
            {
                replan = true;
            }
            else
            {
                replan = false;
            }
        }
        else // not found
        {
            std::cout << "Exit since plan failed until reach maximum attemp: " << count << std::endl;
            replan = false;
            break;
        }
    }

    if(result_ == true)
    {
        if (pause_ == "e" || pause_ == "E")
        {
            group.execute(my_plan);
        }
    }
    ros::WallDuration(1.0).sleep();
}

bool PickPlace::OperateGripper(bool close_gripper)
{
  moveit::core::RobotStatePtr gripper_current_state = gripper_group_->getCurrentState();
  std::vector<double> gripper_joint_positions;
  gripper_current_state->copyJointGroupPositions(kinova_gripper_joint_model_group,gripper_joint_positions);
	
  ROS_INFO("Operating gripper.");
  ROS_INFO("No. of joints in eef_group: %zd", gripper_joint_positions.size());
  if (close_gripper)
  {
	  //ROS_INFO("-------IF--------");
    switch (num_of_label)
    {
			ROS_INFO("Switching Object.");
      case 1:
      {
      //"cube" ok
        gripper_joint_positions[0] = 0.965;
        gripper_joint_positions[1] = 0.9655;//89
        gripper_joint_positions[2] = 0.9655;//90
        ROS_INFO("Gripper closed.");
        break;
      }
      case 2:
      {
      //"cone" ok		  
        gripper_joint_positions[0] = 1.19;
        gripper_joint_positions[1] = 1.19;//89
        gripper_joint_positions[2] = 1.19;//90
        ROS_INFO("Gripper closed.");
        break;
      }
      case 3:
      {
      //"cylinder" ok
        gripper_joint_positions[0] = 1.3;
        gripper_joint_positions[1] = 1.3;//89
        gripper_joint_positions[2] = 1.3;//90
        ROS_INFO("Gripper closed.");
        break;
      }
      case 4:
      {
      //"fourPrism" ok
        gripper_joint_positions[0] = 1.35;
        gripper_joint_positions[1] = 1.35;//89
        gripper_joint_positions[2] = 1.35;//90
        ROS_INFO("Gripper closed.");
        break;
      }
      case 5:
      {
      //"fourPrismoid" ok
        gripper_joint_positions[0] = 1.27;
        gripper_joint_positions[1] = 1.26;//89
        gripper_joint_positions[2] = 1.26;//90
        ROS_INFO("Gripper closed.");
        break;
      }
      case 6:
      {
      //"fourPyramid" ok
        gripper_joint_positions[0] = 1.3;
        gripper_joint_positions[1] = 1.3;//89
        gripper_joint_positions[2] = 1.3;//90
        ROS_INFO("Gripper closed.");
        break;
      }
      case 7:
      {
      //"threePyramid"
        gripper_joint_positions[0] = 1.26;
        gripper_joint_positions[1] = 1.252;//89
        gripper_joint_positions[2] = 1.252;//90
        ROS_INFO("Gripper closed.");
        break;
      }
      case 8:
      {
      //"sphere" ok
        gripper_joint_positions[0] = 1.15;
        gripper_joint_positions[1] = 1.15;//89
        gripper_joint_positions[2] = 1.15;//90
        ROS_INFO("Gripper closed.");
        break;
      }
      case 9:
      {
      //"sixPrism" ok
        gripper_joint_positions[0] = 1.35;
        gripper_joint_positions[1] = 1.35;//89
        gripper_joint_positions[2] = 1.35;//90
        ROS_INFO("Gripper closed.");
        break;
      }
      case 10:
      {
      //"threePrism"
        gripper_joint_positions[0] = 1.32;
        gripper_joint_positions[1] = 1.322;//89
        gripper_joint_positions[2] = 1.32;//90
        ROS_INFO("Gripper closed.");
        break;
      }
    }
  }
  else
  {
    gripper_joint_positions[0] = 0.73;
    gripper_joint_positions[1] = 0.73;
    gripper_joint_positions[2] = 0.73;
    ROS_INFO("Gripper opened.");
  }
  gripper_group_->setJointValueTarget(gripper_joint_positions);
  ros::Duration(1.0).sleep();
  bool success = static_cast<bool>(gripper_group_->move());
  return success;
}

bool PickPlace::kinova_routine(robot_grasp::G_pick_place::Request &req,robot_grasp::G_pick_place::Response &res)
{
	if(req.transformed_param.pcaCentroid.size()==0)
	{
		std::cout<<"No obj to be grasped !"<<std::endl;
		return true;
	}
	num_of_label = req.transformed_param.num_obj_label;
	std::cout << "The label of object is " << num_of_label << std::endl;
	
	detected_param_obj_list = req.transformed_param.pcaCentroid;
	
	for( int i=0;i<3;i++ )
		std::cout << "The param of object is " << detected_param_obj_list[i] << std::endl;	

	// Place position setting;
    obj_place_pose.header.frame_id = "j2n6s300_link_base";
    obj_place_pose.header.stamp = ros::Time::now();
    obj_place_pose.pose.position.x = 0.537;
    obj_place_pose.pose.position.y = -0.25853;
    //obj_place_pose.pose.position.z = 0.042587;
    obj_place_pose.pose.position.z = 0.01;
    obj_place_pose.pose.orientation.x = 1.0;
    obj_place_pose.pose.orientation.y = 0.0;
    obj_place_pose.pose.orientation.z = 0.0;
    obj_place_pose.pose.orientation.w = 0.0;

	switch(num_of_label)
	{
		case 1://cube
			{
				obj_place_pose.pose.position.x = 0.4837;
    			obj_place_pose.pose.position.y = -0.31853;
				break;
			}
		case 2://"cone"
			{
				obj_place_pose.pose.position.x = 0.6057;
    			obj_place_pose.pose.position.y = -0.25853;
				break;
			}
		case 3:
			{
				obj_place_pose.pose.position.x = 0.4637;
    			obj_place_pose.pose.position.y = -0.25853;
				break;
			}
		case 4:
			{
				obj_place_pose.pose.position.x = 0.537;
    			obj_place_pose.pose.position.y = -0.295853;
				break;
			}
		case 5:
			{
				obj_place_pose.pose.position.x = 0.537;
    			obj_place_pose.pose.position.y = -0.17853;
				break;
			}
		case 6:
			{
				obj_place_pose.pose.position.x = 0.6057;
    			obj_place_pose.pose.position.y = -0.295853;
				break;
			}
		case 7://"threePyramid"
			{
				obj_place_pose.pose.position.x = 0.56257;
    			obj_place_pose.pose.position.y = -0.32153;
				break;
			}
		case 8:
			{
				obj_place_pose.pose.position.x = 0.4637;
    			obj_place_pose.pose.position.y = -0.295853;
				break;
			}
		case 9://"sixPrism"
			{
				obj_place_pose.pose.position.x = 0.50123;
    			obj_place_pose.pose.position.y = -0.20853;
				break;
			}
		case 10://"threePrism"
			{
				obj_place_pose.pose.position.x = 0.55383;
    			obj_place_pose.pose.position.y = -0.18453;
				break;
			}
	}

  pre_obj_place_pose = obj_place_pose;
	pre_obj_place_pose.pose.position.z = obj_place_pose.pose.position.z + 0.13;
	
	new_obj_place_pose = obj_place_pose;
	
	new_obj_place_pose.pose.position.x = obj_place_pose.pose.position.x - 0.03;
	new_obj_place_pose.pose.position.y = obj_place_pose.pose.position.y - 0.03;
	new_obj_place_pose.pose.position.z = obj_place_pose.pose.position.z + 0.15;
	
	//Grasp position setting;
	obj_grasp_pose.header.frame_id = "j2n6s300_link_base";
	obj_grasp_pose.header.stamp = ros::Time::now();
	obj_grasp_pose.pose.position.x = detected_param_obj_list[0];
	obj_grasp_pose.pose.position.y = detected_param_obj_list[1];
	obj_grasp_pose.pose.position.z = detected_param_obj_list[2]+0.032;
	obj_grasp_pose.pose.orientation.x = 1.0;
	obj_grasp_pose.pose.orientation.y = 0.0;
	obj_grasp_pose.pose.orientation.z = 0.0;
	obj_grasp_pose.pose.orientation.w = 0.0;

	obj_pre_grasp_pose = obj_grasp_pose;
	obj_pre_grasp_pose.pose.position.z = obj_grasp_pose.pose.position.z + 0.13;
	
	new_obj_grasp_pose = obj_grasp_pose;
	new_obj_grasp_pose.pose.position.x = obj_grasp_pose.pose.position.x - 0.02;
	new_obj_grasp_pose.pose.position.y = obj_grasp_pose.pose.position.y - 0.03;
	new_obj_grasp_pose.pose.position.z = obj_grasp_pose.pose.position.z + 0.15;	  
	  
	ROS_INFO("Start planning obj_pre_grasp_pose and execute.");
	//ros::WallDuration(0.5).sleep();
	group_->setPoseTarget(obj_pre_grasp_pose);
	result_ = static_cast<bool>(group_->plan(kinova_plan));
	group_->execute(kinova_plan);
	//evaluate_plan(*group_);

	ROS_INFO("Start planning obj_grasp_pose and execute.");
	//ros::WallDuration(0.5).sleep();
	group_->setPoseTarget(obj_grasp_pose);
	result_ = static_cast<bool>(group_->plan(kinova_plan));
	group_->execute(kinova_plan);
	//evaluate_plan(*group_);
	
	ROS_INFO("Closing gripper after reach the grasping-position.");
	//ros::WallDuration(0.5).sleep();
	OperateGripper(true);

	ROS_INFO("Lifting the object up.");
	//ros::WallDuration(0.5).sleep();
	group_->setPoseTarget(new_obj_grasp_pose);
	result_ = static_cast<bool>(group_->plan(kinova_plan));
	group_->execute(kinova_plan);
	//evaluate_plan(*group_);

	ROS_INFO("Start planning obj_pre_place_pose and execute.");
	//ros::WallDuration(0.5).sleep();
	group_->setPoseTarget(pre_obj_place_pose);
	result_ = static_cast<bool>(group_->plan(kinova_plan));
	group_->execute(kinova_plan);
	//evaluate_plan(*group_);
	
	ROS_INFO("Start planning obj_place_pose and execute.");
	//ros::WallDuration(0.5).sleep();
	group_->setPoseTarget(obj_place_pose);
	result_ = static_cast<bool>(group_->plan(kinova_plan));
	group_->execute(kinova_plan);
	//evaluate_plan(*group_);
	
	ROS_INFO("Open gripper after reach obj_place_pose.");
	//ros::WallDuration(0.5).sleep();
	result_ = OperateGripper(false);
	res.success = result_;

	ROS_INFO("Lifting the robotics arm up!");
	//ros::WallDuration(0.5).sleep();
	group_->setPoseTarget(new_obj_place_pose);
	result_ = static_cast<bool>(group_->plan(kinova_plan));
	group_->execute(kinova_plan);
	//evaluate_plan(*group_);
	
	return true;
}

void PickPlace::getInvK(geometry_msgs::Pose &eef_pose, std::vector<double> &joint_value)
{
    // TODO: transform cartesian command to joint space, and alway motion plan in joint space.
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "pick_place_demo");
    ros::NodeHandle node;
	
    ros::AsyncSpinner spinner(4);
    spinner.start();
    kinova::PickPlace pick_place(node);
    ROS_INFO("Waiting for grasp!");
	
    ros::MultiThreadedSpinner spinner2(4);
    ros::ServiceServer service = node.advertiseService("/get_pick_place_routine", &PickPlace::kinova_routine, &pick_place);
    spinner2.spin();
    return 0;
}
