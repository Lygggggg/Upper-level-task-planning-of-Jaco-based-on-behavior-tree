# This Python file uses the following encoding: utf-8
import rospy
import sys
import moveit_commander
from geometry_msgs.msg import Pose

def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('test')

    arm = moveit_commander.MoveGroupCommander('arm')

    waypoints = []

    waypoint = geometry_msgs.msg.Pose()

    print arm.get_current_pose().pose.position.x

if __name__ == '__main__':
    main()
