#!/usr/bin/env python
# -*- coding: utf-8 -*-
import os , sys
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from moveit_msgs.msg import PlanningScene, ObjectColor
from moveit_msgs.msg import Grasp, GripperTranslation, MoveItErrorCodes
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

import numpy as np



class MoveGroupPythonIntefaceTutorial(object):
    def __init__(self):
        super(MoveGroupPythonIntefaceTutorial, self).__init__()

        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('move_group_python_interface_tutorial',anonymous=True)

        ## 实例化一个 "RobotCommander" 对象. 该对象是机器人与外界的接口：
        robot = moveit_commander.RobotCommander()

        ## 实例化一个 "PlanningSceneInterface" 对象。这个对象是机器人与周围环境的接口:
        scene = moveit_commander.PlanningSceneInterface()

        group = moveit_commander.MoveGroupCommander("arm")
        gripper_group = moveit_commander.MoveGroupCommander("gripper")

        self.robot = robot
        self.scene = scene
        self.group = group
        self.gripper_group = gripper_group

    def moveJoint(self,data):

        group = self.group
        group.allow_replanning(True)
        group.set_planning_time(5)

        max_attempts = 5
        waypoints = []
        for i in range(len(data)):
            target_pose = Pose()
            target_pose.position.x = data[i,0]
            target_pose.position.y = data[i,1]
            target_pose.position.z = data[i,2]
            target_pose.orientation.x = data[i,3]
            target_pose.orientation.y = data[i,4]
            target_pose.orientation.z = data[i,5]
            target_pose.orientation.w = data[i,6]

            waypoints.append(target_pose)

        group.set_pose_reference_frame('j2n6s300_link_base')
        result = None
        n_attempts = 0

        while result != MoveItErrorCodes.SUCCESS and n_attempts < max_attempts:
            n_attempts += 1
            rospy.loginfo("Planning attempt: " +  str(n_attempts))
            (plan , fraction) = group.compute_cartesian_path(waypoints,0.01,0.0)
            result = group.execute(plan, wait=True)
            print('Planning:',result)
            rospy.sleep(1)

    def moveFingers(self, open_gripper):
        gripper_group = self.gripper_group

        if(open_gripper):
            fingers_cmd = [0.3,0.3,0.3]
            rospy.loginfo("Gripper opened")
        else:
            fingers_cmd = [1.1,1.1,1.1]
            rospy.loginfo("Gripper closed")
        gripper_group.set_joint_value_target(fingers_cmd)
        rospy.sleep(1.0)
        gripper_group.go()


def main():
    dmp_grasp = MoveGroupPythonIntefaceTutorial()

    data = np.loadtxt("/home/lni/Ros_ws/catkin_ws/src/JCAR-Competition-master/robot_grasp/data/pick_place_data.txt")

    rospy.loginfo("go to home")
    dmp_grasp.group.set_named_target("Home")
    dmp_grasp.group.go()

    intermid_point = PoseStamped()
    intermid_point.header.stamp = rospy.Time.now()
    intermid_point.header.frame_id = 'j2n6s300_link_base'
    intermid_point.pose.position.x = data[0][0]
    intermid_point.pose.position.y = data[0][1]
    intermid_point.pose.position.z = data[0][2]
    intermid_point.pose.orientation.x = data[0][3]
    intermid_point.pose.orientation.y = data[0][4]
    intermid_point.pose.orientation.z = data[0][5]
    intermid_point.pose.orientation.w = data[0][6]
    dmp_grasp.group.set_pose_target(intermid_point)
    dmp_grasp.group.go()

    rospy.sleep(10)


    rospy.loginfo("execute dmp")
    dmp_grasp.moveJoint(data)
    rospy.loginfo("Operating gripper")
    dmp_grasp.moveFingers(True)

if __name__ == '__main__':
    main()
