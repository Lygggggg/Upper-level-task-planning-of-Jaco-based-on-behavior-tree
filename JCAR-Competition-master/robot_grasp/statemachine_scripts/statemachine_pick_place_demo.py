#!/usr/bin/env python
import os , sys
import smach
import smach_ros

from gazebo_msgs.srv import (
    SpawnModel,
    DeleteModel,
)
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

import rospkg
import rospy

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
import moveit_commander
import moveit_msgs.msg
from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from moveit_msgs.msg import PlanningScene, ObjectColor
from moveit_msgs.msg import Grasp, GripperTranslation, MoveItErrorCodes

import numpy

GROUP_NAME_ARM = 'arm'
GROUP_NAME_GRIPPER = 'gripper'

def delete_gazebo_model(model):
    try:
        delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        resp_delete = delete_model(model)
    except rospy.ServiceException, e:
        rospy.loginfo("Delete Model service call failed: {0}".format(e))

class LoadGazeboModelState(smach.State):
    def __init__(self, name, model_path, input_keys = ['pose', 'reference_frame'], pose_cb = None):
        smach.State.__init__(self, input_keys=input_keys, outcomes=['succeeded'])
        self._name = name
        self._model_path = model_path
        # Set up a poses callback
        self._pose_cb = pose_cb
    def execute(self, userdata):
        # If a pose callback has been defined, use it to format
        # pose specified by the input keys in the userdata
        if self._pose_cb:
            try:
                pose = self._pose_cb(userdata)
            except Exception as e:
                rospy.logerr('Error when using poses callback to format poses: ' + repr(e))
                raise
        else:
            if 'pose' in userdata:
                pose = userdata.pose
            else:
                raise ValueError('Pose should be specified in userdata!')

        # Parse pose
        try:
            if isinstance(pose, PoseStamped):
                pose = pose.pose
            elif isinstance(pose, Pose):
                pose = pose
            elif isinstance(pose, list):
                position = Point(x=pose[0][0], y=pose[0][1], z=pose[0][2])
                orientation = Quaternion(x=pose[1][0], y=pose[1][1], z=pose[1][2], w=pose[1][3])
                pose = Pose(position=position, orientation=orientation)
            else:
                raise ValueError('Pose should be specified as a list, Pose or PoseStamped!')
        except Exception as e:
                rospy.logerr('Error when parsing Gazebo model pose: ' + repr(e))
                raise

        # Parse reference_frame
        try:
            if 'reference_frame' in userdata:
                reference_frame = userdata.reference_frame

                if isinstance(reference_frame, str):
                    pass
                elif isinstance(reference_frame, list):
                    if isinstance(reference_frame[0], str):
                        reference_frame = reference_frame[0]
                    else:
                        raise ValueError('The reference frame should be specified as a string!')
                else:
                    raise ValueError('The reference frame should be specified as a string!')
            else:
                raise ValueError('The reference frame should be specified in userdata!')
        except Exception as e:
                rospy.logerr('Error when parsing Gazebo model reference frame: ' + repr(e))
                raise

        # Load model SDF/URDF XML
        try:
            model_xml = ''
            with open(self._model_path, 'r') as model_file:
                model_xml = model_file.read().replace('\n', '')
        except Exception as e:
            rospy.logerr('Error when loading Gazebo model XML file: ' + repr(e))
            raise

        # Spawn model SDF/URDF
        try:
            if os.path.splitext(self._model_path)[1][1:].lower() == 'sdf':
                spawn_service_type = 'sdf'
            elif os.path.splitext(self._model_path)[1][1:].lower() == 'urdf':
                spawn_service_type = 'urdf'
        except Exception as e:
            rospy.logerr('Error when determining whether Gazebo model is SDF or URDF: ' + repr(e))
            raise

        try:
            spawn_service_proxy = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
            spawn_response = spawn_service_proxy(self._name, model_xml, "/",pose, reference_frame)
        except rospy.ServiceException, e:
            rospy.logerr('Spawn ' + spawn_service_type.upper() + ' service call failed: {0}'.format(e))

        return 'succeeded'

class MoveHome(smach.State):
    def __init__(self,timeout=15.0, input_keys = ['prefix','positions'], positions_cb = None):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'], input_keys=input_keys)
        self._timeout = timeout
        self._positions_cb = positions_cb

    def moveJoint(self,userdata):
        prefix = userdata.prefix
        positions = userdata.positions
        topic_name = '/' + prefix + '/effort_joint_trajectory_controller/command'
        pub = rospy.Publisher(topic_name, JointTrajectory, queue_size=1)
        jointCmd = JointTrajectory()
        point = JointTrajectoryPoint()
        jointCmd.header.stamp = rospy.Time.now() + rospy.Duration.from_sec(0.0);
        point.time_from_start = rospy.Duration.from_sec(5.0)
        for i in range(0, 6):
                jointCmd.joint_names.append(prefix +'_joint_'+str(i+1))
                point.positions.append(positions[i])
                point.velocities.append(0)
                point.accelerations.append(0)
                point.effort.append(0)
        jointCmd.points.append(point)
        rate = rospy.Rate(1)
        count = 0
        while (count < 10):
            pub.publish(jointCmd)
            count = count + 1
            rate.sleep()

    def execute(self,userdata):
        try:
            self.moveJoint(userdata)
        except Exception as e:
            rospy.logerr('Error when using limb interface to move to joint positions: ' + repr(e))
            raise
        return 'succeeded'

class OperateGripper(smach.State):
    def __init__(self,timeout=15.0, input_keys = ['prefix','fingers']):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'], input_keys=input_keys)
        self._timeout = timeout

    def moveFingers(self,userdata):
        prefix = userdata.prefix
        fingerscmd = userdata.fingers
        topic_name = '/' + prefix + '/effort_finger_trajectory_controller/command'
        pub = rospy.Publisher(topic_name, JointTrajectory, queue_size=1)
        jointCmd = JointTrajectory()
        point = JointTrajectoryPoint()
        jointCmd.header.stamp = rospy.Time.now() + rospy.Duration.from_sec(0.0);
        point.time_from_start = rospy.Duration.from_sec(5.0)
        for i in range(0, 3):
            jointCmd.joint_names.append(prefix +'_joint_finger_'+str(i+1))
            point.positions.append(fingerscmd[i])
            point.velocities.append(0)
            point.accelerations.append(0)
            point.effort.append(0)
        jointCmd.points.append(point)
        rate = rospy.Rate(10)
        count = 0
        while (count < 10):
            pub.publish(jointCmd)
            count = count + 1
            rate.sleep()

    def execute(self,userdata):
        try:
            self.moveFingers(userdata)
        except Exception as e:
            rospy.logerr('Error when using limb interface to move to joint positions: ' + repr(e))
            raise
        return 'succeeded'

class GoHome(smach.State):
    def __init__(self):
        smach.State.__init__(self,input_keys = ['prefix'], outcomes = ['succeeded', 'aborted'])


    def moveJoint(self,userdata):

        moveit_commander.roscpp_initialize(sys.argv)
        arm = MoveGroupCommander(GROUP_NAME_ARM)
        arm.allow_replanning(True)
        arm.set_pose_reference_frame('j2n6s300_link_base')
        arm.set_planning_time(5)

        max_attempts = 5

        prefix = userdata.prefix

        result = None
        n_attempts = 0

        while result != MoveItErrorCodes.SUCCESS and n_attempts < max_attempts:
            n_attempts += 1
            rospy.loginfo("Planning attempt: " +  str(n_attempts))
            arm.set_named_target('Home')
            arm.go()
            result = arm.go()
            print('Planning:',result)
            rospy.sleep(1)


    def execute(self,userdata):
        try:
            if 'prefix' in userdata:
                prefix = userdata.prefix
            else:
                raise ValueError('Limb should be specified in userdata!')
            self.moveJoint(userdata)
        except Exception as e:
            rospy.logerr('Error when using limb interface to move to joint positions: ' + repr(e))
            raise
        return 'succeeded'
	
class MotionPlanning(smach.State):
    def __init__(self):
        smach.State.__init__(self,input_keys = ['prefix','positions','orientations'], outcomes = ['succeeded', 'aborted'])


    def moveJoint(self,userdata):

        moveit_commander.roscpp_initialize(sys.argv)
        arm = MoveGroupCommander(GROUP_NAME_ARM)
        arm.allow_replanning(True)
        arm.set_pose_reference_frame('j2n6s300_link_base')
        arm.set_planning_time(5)

        max_attempts = 5

        prefix = userdata.prefix
        positions = userdata.positions
        orientations = userdata.orientations

        target_pose = PoseStamped()

        target_pose.header.stamp = rospy.Time.now()
        target_pose.header.frame_id = prefix+'_link_base'
        target_pose.pose.position.x = positions[0]
        target_pose.pose.position.y = positions[1]
        target_pose.pose.position.z = positions[2]
        target_pose.pose.orientation.x = orientations[0]
        target_pose.pose.orientation.y = orientations[1]
        target_pose.pose.orientation.z = orientations[2]
        target_pose.pose.orientation.w = orientations[3]

        result = None
        n_attempts = 0

        while result != MoveItErrorCodes.SUCCESS and n_attempts < max_attempts:
            n_attempts += 1
            rospy.loginfo("Planning attempt: " +  str(n_attempts))
            arm.set_pose_target(target_pose)
            result = arm.go()
            print('Planning:',result)
            rospy.sleep(1)


    def execute(self,userdata):
        try:
            if 'prefix' in userdata:
                prefix = userdata.prefix
            else:
                raise ValueError('Limb should be specified in userdata!')
            self.moveJoint(userdata)
        except Exception as e:
            rospy.logerr('Error when using limb interface to move to joint positions: ' + repr(e))
            raise
        return 'succeeded'

class Jiyuan(smach.State):
    def __init__(self):
        smach.State.__init__(self,input_keys = ['prefix','way_points'], outcomes = ['succeeded', 'aborted'])


    def moveJoint(self,userdata):

        moveit_commander.roscpp_initialize(sys.argv)
        arm = MoveGroupCommander(GROUP_NAME_ARM)
        arm.allow_replanning(True)
        arm.set_pose_reference_frame('j2n6s300_link_base')
        arm.set_planning_time(5)

        max_attempts = 5

        prefix = userdata.prefix
        way_points = userdata.way_points

        plan_list = []
        waypoints = []
        for i in range(len(way_points)):
            #target_pose = PoseStamped()
            #target_pose.header.stamp = rospy.Time.now()
            #target_pose.header.frame_id = prefix + '_link_base'
            #target_pose.pose.position.x = way_points[i,0]
            #target_pose.pose.position.y = way_points[i,1]
            #target_pose.pose.position.z = way_points[i,2]
            #target_pose.pose.orientation.x = way_points[i,3]
            #target_pose.pose.orientation.y = way_points[i,4]
            #target_pose.pose.orientation.z = way_points[i,5]
            #target_pose.pose.orientation.w = way_points[i,6]

            target_pose = arm.get_current_pose().pose
            target_pose.position.x = way_points[i,0]
            target_pose.position.y = way_points[i,1]
            target_pose.position.z = way_points[i,2]
            target_pose.orientation.x = way_points[i,3]
            target_pose.orientation.y = way_points[i,4]
            target_pose.orientation.z = way_points[i,5]
            target_pose.orientation.w = way_points[i,6]

            waypoints.append(target_pose)

        result = None
        n_attempts = 0

        while result != MoveItErrorCodes.SUCCESS and n_attempts < max_attempts:
            n_attempts += 1
            rospy.loginfo("Planning attempt: " +  str(n_attempts))
            (plan , fraction) = arm.compute_cartesian_path(waypoints,0.01,0.0)
            result = arm.execute(plan , wait=True)
            print('Planning:',result)
            rospy.sleep(1)

    def execute(self,userdata):
        try:
            if 'prefix' in userdata:
                prefix = userdata.prefix
            else:
                raise ValueError('Limb should be specified in userdata!')
            self.moveJoint(userdata)
        except Exception as e:
            rospy.logerr('Error when using limb interface to move to joint positions: ' + repr(e))
            raise
        return 'succeeded'

def main():
    rospy.init_node('Jaco_smach_pick_and_place_test')

    sm = smach.StateMachine(outcomes=['succeeded', 'aborted', 'preempted'])

    sm.userdata.cube_position = [[0.240161, -0.268802, 0.731732], [0.0, 0.0, 0.0, 0.0]]

    sm.userdata.block_model_pick_ref_frame = 'world'

    sm.userdata.threePyramid_position = [[0.172766, -0.168519, 0.725651], [0.0, 0.0, 0.0, 0.0]]

    sm.userdata.block_model_pick_ref_frame = 'world'

    sm.userdata.prefix = 'j2n6s300'

    sm.userdata.start_joint_positions = [0.0, 2.9, 1.3, 4.2, 1.4, 0.0]

    sm.userdata.fingers_cmd = [1.0,1.0,1.0]



    with sm:


        smach.StateMachine.add('Load_object',
                               LoadGazeboModelState('threePrism',rospkg.RosPack().get_path('robot_grasp')+'/models/grasp_object/threePrism/threePrism.sdf'),
                               transitions={'succeeded':'Move_to_Home_Position'},
                               remapping={'pose':'threePyramid_position',
                                          'reference_frame':'block_model_pick_ref_frame'})

        smach.StateMachine.add('Move_to_Home_Position',
                               GoHome(),
                               transitions={'succeeded':'Open_Gripper'},
                               remapping={'prefix':'prefix'})

#        smach.StateMachine.add('Move_to_Start_Position',
#                               GoHome(),
#                               transitions={'succeeded':'Open_Gripper'},
#                               remapping={'prefix':'prefix',
#                                          'positions':'start_joint_positions'})

        smach.StateMachine.add('Open_Gripper',
                               OperateGripper(),
                               transitions={'succeeded':'Pick_Object'},
                               remapping={'prefix':'prefix',
                                         'fingers':'fingers_cmd'})


        sm_pick_object = smach.StateMachine(outcomes=['succeeded', 'aborted', 'preempted'],
                                                        input_keys = ['prefix'])
        #sm_pick_object.userdata.prefix = 'j2n6s300'


        sm_pick_object.userdata.intermedia_pt_1_position = [0.35653, 0.0, 0.33787]
        sm_pick_object.userdata.intermedia_pt_1_orientation = [1.0, 0.0, 0.0, 0.0]

        sm_pick_object.userdata.intermedia_pt_2_position = [0.35704, 0.34392, 0.33698]
        sm_pick_object.userdata.intermedia_pt_2_orientation = [1.0, 0.0, 0.0, 0.0]

        sm_pick_object.userdata.intermedia_pt_3_position = [0.46955, 0.3427, 0.12178]
        sm_pick_object.userdata.intermedia_pt_3_orientation = [1.0, 0.0, 0.0, 0.0]

        sm_pick_object.userdata.way_point = numpy.loadtxt('/home/lni/Ros_ws/jaco_ws/src/JCAR-Competition-master/robot_grasp/statemachine_scripts/jiyuan0.txt')

        with sm_pick_object:
            smach.StateMachine.add('Move_to_media_point_1_position',
                                  MotionPlanning(),
                                  transitions = {'succeeded':'Execute_Jiyuan0'},
                                  remapping = {'prefix':'prefix',
                                               'positions':'intermedia_pt_1_position',
                                               'orientations':'intermedia_pt_1_orientation'})

            #smach.StateMachine.add('Move_to_media_point_2_position',
            #					  MotionPlanning(),
            #					  transitions = {'succeeded':'Move_to_media_point_3_position'},
            #					  remapping = {'prefix':'prefix',
            #								   'positions':'intermedia_pt_2_position',
            #								   'orientations':'intermedia_pt_2_orientation'})

            #smach.StateMachine.add('Move_to_media_point_3_position',
            #					  MotionPlanning(),
            #					  transitions = {'succeeded':'succeeded'},
            #					  remapping = {'prefix':'prefix',
            #								   'positions':'intermedia_pt_3_position',
            #								   'orientations':'intermedia_pt_3_orientation'})

            smach.StateMachine.add('Execute_Jiyuan0',
                                    Jiyuan(),
                                    transitions = {'succeeded':'succeeded'},
                                    remapping = {'prefix':'prefix',
                                                 'way_points':'way_point'})

        smach.StateMachine.add('Pick_Object',sm_pick_object,
                               transitions={'succeeded':'succeeded'},
                               remapping={'prefix':'prefix'})


    sis = smach_ros.IntrospectionServer('Jaco_smach_pick_and_place_test', sm, '/sm')
    sis.start()

    rospy.on_shutdown(lambda: delete_gazebo_model('threePrism'))
    #rospy.on_shutdown(lambda: delete_gazebo_model('cube'))

    try:
        outcome = sm.execute()

        print("Jaco SMACH Pick and Place Test Complete. Ctrl-C to exit.")

        rospy.spin()
    except Exception as e:
        rospy.logerr('Error when executing state machine: ' + repr(e))
        rospy.signal_shutdown('Error when executing state machine: ' + repr(e))

if __name__ == '__main__':
    main()
