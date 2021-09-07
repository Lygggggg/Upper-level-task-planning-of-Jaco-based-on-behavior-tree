#! /usr/bin/env python3
# -*- coding: utf-8 -*-
import tf
import time
import sys
import tf2_ros
import rospy
import actionlib
import numpy as np
import behavior_tree_core.msg
from geometry_msgs.msg import Pose
import moveit_commander as mc

class BTAction(object):
  # create messages that are used to publish feedback/result
  _feedback = behavior_tree_core.msg.BTFeedback()
  _result   = behavior_tree_core.msg.BTResult()

  def __init__(self, name):
    self._action_name = name
    self._as = actionlib.SimpleActionServer(self._action_name, behavior_tree_core.msg.BTAction, execute_cb=self.execute_cb,
                                                auto_start = False)
    self._as.start()

    self.d = []# d 为 cost 做计算

    #self.TransMatrix = np.zeros((6,6))
    self.TransMatrix = np.array([[0,0,0,1,0,0],
                                [0,0,0,1,0,0],
                                [0,0,0,1,0,0],
                                [0.67,0,0,0,0.33,0],
                                [0,0.25,0,0,0,0.75],
                                [0,0,0,0,1,0]])
    self.is_first_running = True
    self.dmax = 10
    self.curr_jiyuan_index = 0

    mc.roscpp_initialize(sys.argv)
    self.arm = mc.MoveGroupCommander('arm')
    self.reference_frame = 'j2n6s300_link_base'
    self.end_effector_link = self.arm.get_end_effector_link()
    self.tfBuffer = tf2_ros.Buffer()
    self.listener = tf2_ros.TransformListener(self.tfBuffer)


    
  def execute_cb(self, goal):
      print (time.strftime("----------%H:%M:%S 进入condition回调", time.localtime()))
      arm = self.arm
      current_pose_position_x,current_pose_position_y,current_pose_position_z = self.get_current_xyz()
      grasp_x = rospy.get_param('/x_under_base')
      grasp_y = rospy.get_param('/y_under_base')
      grasp_z = rospy.get_param('/z_under_base')

      res = np.sqrt((current_pose_position_x-grasp_x)**2+(current_pose_position_y-grasp_y)**2
                    +(current_pose_position_z-grasp_z)**2)
      if res<0.1:

          self.set_status('SUCCESS')

      else:
          rospy.loginfo('------------未靠近物体--------------')
          self.set_status('FAILURE')


  def set_status(self,status):
      if status == 'SUCCESS':
        self._feedback.status = 1
        self._result.status = self._feedback.status
        rospy.loginfo('Action %s: Succeeded' % self._action_name)
        self._as.set_succeeded(self._result)
      elif status == 'FAILURE':
        self._feedback.status = 2
        self._result.status = self._feedback.status
        rospy.loginfo('Action %s: Failed' % self._action_name)
        self._as.set_succeeded(self._result)
      else:
        rospy.logerr('Action %s: has a wrong return status' % self._action_name)



  def get_current_xyz(self):
    #获取终端link名称

      frame_info = self.tfBuffer.lookup_transform("j2n6s300_link_base", "j2n6s300_end_effector", rospy.Time(0))

      current_pose_position_x = frame_info.transform.translation.x
      current_pose_position_y = frame_info.transform.translation.y
      current_pose_position_z = frame_info.transform.translation.z

      return current_pose_position_x,current_pose_position_y,current_pose_position_z

  def caculate_d(self,target_x,target_y,target_z):
      start_pose_position_x,start_pose_position_y,start_pose_position_z = self.get_current_xyz()
      dist = []
      jiyuan = get_jiyuan()
      #循环获取每个基元的最后一个位置
      for i in range(6):
          end_x = start_pose_position_x + (jiyuan[i][-1,0] - jiyuan[i][0,0])
          end_y = start_pose_position_y + (jiyuan[i][-1,1] - jiyuan[i][0,1])
          end_z = start_pose_position_z + (jiyuan[i][-1,2] - jiyuan[i][0,2])
          # 第 i 个基元执行后距离目标点的欧式距离
          dist.append(np.sqrt((end_x-target_x)**2 + (end_y-target_y)**2 + (end_z-target_z)**2))
      return dist

  def caculate_cost(self,d,curr_index,dmax):
       cost = np.zeros(6)  #类型和 d 不一样,是array
       #开始计算 cost 的每个元素,cost 是array
       for i in range(6):
           cost[i] = (-1)*self.TransMatrix[curr_index][i]*0.5 + (d[i]/dmax)*0.5
       return cost


if __name__ == '__main__':
  rospy.init_node('have_object')
  BTAction(rospy.get_name())
  rospy.spin()
