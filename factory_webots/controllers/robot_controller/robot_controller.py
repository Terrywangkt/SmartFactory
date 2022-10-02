#!/usr/bin/env python
# coding:utf-8
import argparse
import rospy

from controller import Robot
from arm_joint_state_publisher import ArmJointStatePublisher
from arm_trajectory_follower import ArmTrajectoryFollower
from gripper_joint_state_publisher import GripperJointStatePublisher
from gripper_trajectory_follower import GripperTrajectoryFollower
from rosgraph_msgs.msg import Clock

parser = argparse.ArgumentParser()
parser.add_argument('--node-name', dest='nodeName', default='ur_driver', help='Specifies the name of the node.')
arguments, unknown = parser.parse_known_args()

rospy.init_node('robot_driver',disable_signals=True)

jointPrefix = rospy.get_param('prefix', '')
if jointPrefix:
    print('Setting prefix to %s' % jointPrefix)

robot = Robot()
nodeName = arguments.nodeName + '/' if arguments.nodeName != 'ur_driver' else ''
armJointStatePublisher = ArmJointStatePublisher(robot, jointPrefix, nodeName)
armTrajectoryFollower = ArmTrajectoryFollower(robot, armJointStatePublisher, jointPrefix, nodeName)
armTrajectoryFollower.start()
gripperJointStatePublisher = GripperJointStatePublisher(robot, jointPrefix, nodeName)
gripperTrajectoryFollower = GripperTrajectoryFollower(robot, gripperJointStatePublisher, jointPrefix, nodeName)
gripperTrajectoryFollower.start()

clockPublisher = rospy.Publisher('clock', Clock, queue_size=1)
if not rospy.get_param('use_sim_time', False):
    rospy.logwarn('use_sim_time is not set!')

timestep = int(robot.getBasicTimeStep())
ds = robot.getDevice("distance sensor")
ds.enable(timestep)
while robot.step(timestep) != -1 and not rospy.is_shutdown():
    # pulish simulation clock
    armJointStatePublisher.publish()
    armTrajectoryFollower.update()
    gripperJointStatePublisher.publish()
    gripperTrajectoryFollower.update()
    if (ds.getValue()<500):
        rospy.set_param("inGripper",True)
    else:
        rospy.set_param("inGripper",False)
        
    msg = Clock()
    time = robot.getTime()
    msg.clock.secs = int(time)
    # round prevents precision issues that can cause problems with ROS timers
    msg.clock.nsecs = int(round(1000 * (time - msg.clock.secs)) * 1.0e+6)
    clockPublisher.publish(msg)