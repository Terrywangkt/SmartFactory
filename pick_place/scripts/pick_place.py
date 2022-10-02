#!/usr/bin/env python
# coding:utf-8

#1.static object
# initialize
import rospy
import moveit_commander
import sys
import geometry_msgs.msg
import numpy as np
from tf.transformations import quaternion_from_euler
import math

#2. object in motion
def initArm(moveGroup):
    values = moveGroup.get_current_joint_values()
    values[1] = -0.25
    # moveGroup.set_joint_value_target(values)
    moveGroup.go(values,wait=True)
    result = moveGroup.stop()
    moveGroup.clear_pose_targets()
    return result
def getTarget():
    if rospy.has_param("target"):
        # in webots 2022 moveit and webots used the same corrdinate system, in elder system there is a transfer,counterclowise 90 degree around x axis

        target = rospy.get_param("target")
        return target
    else:
        return None
def getPresuppositionTarget():
    # speed 0.15 time 6
    if rospy.has_param("target"):
        # in webots 2022 moveit and webots used the same corrdinate system, in elder system there is a transfer,counterclowise 90 degree around x axis

        target = rospy.get_param("target")
        target[1] += 0.15*7
        return target
    else:
        return None


def addTarget(robot, scene, target):
    pose = geometry_msgs.msg.PoseStamped()
    pose.header.stamp = rospy.Time.now()
    pose.header.frame_id = robot.get_planning_frame()
    pose.pose.position.x = target[0]
    pose.pose.position.y = target[1]
    pose.pose.position.z = target[2]
    scene.add_box("target",pose=pose,size = (0.1,0.1,0.1))
    return pose.pose

def addBelt(robot, scene):
    pose = geometry_msgs.msg.PoseStamped()
    pose.header.stamp = rospy.Time.now()
    pose.header.frame_id = robot.get_planning_frame()
    belt = np.array([[1.5,0,0.05]]).T
    pose.pose.position.x = belt[0]
    pose.pose.position.y = belt[1]
    pose.pose.position.z = belt[2]
    q = quaternion_from_euler(np.deg2rad(90.0), 0.0, np.deg2rad(90))
    pose.pose.orientation = geometry_msgs.msg.Quaternion(*q)

    scene.add_box("belt",pose,(3,0.1,1.3))

distance = 0.3
def moveUp(armGroup, pose):
    p = geometry_msgs.msg.Pose()
    p.position.x = pose.position.x
    p.position.y = pose.position.y
    p.position.z = pose.position.z + distance
    # p.orientation.w = 1.0
    q = quaternion_from_euler(0,math.pi,math.pi/2)
    p.orientation = geometry_msgs.msg.Quaternion(*q)
    armGroup.set_pose_target(p)
    armGroup.go()
    armGroup.stop()
    armGroup.clear_pose_targets()
    print("moveup done")


def openGripper(gripperGroup):
    gripperGroup.set_named_target("gripper_open")
    gripperGroup.go()
    gripperGroup.stop()
    gripperGroup.clear_pose_targets()
    # print("opened")

def closeGripper(gripperGroup):
    # size of object 0.1
    values = gripperGroup.get_current_joint_values()
    # values[0] = 0.035
    # values[1] = 0.035
    values[0] = 0.08
    values[1] = 0.08

    gripperGroup.go(values)
    gripperGroup.stop()
    gripperGroup.clear_pose_targets()


def nearTarget(armGroup):
    points = []
    current = armGroup.get_current_pose().pose
    points.append(current)
    pose = geometry_msgs.msg.Pose()
    pose.position.x = current.position.x
    pose.position.y = current.position.y
    pose.position.z = current.position.z - 0.15
    q = quaternion_from_euler(0,math.pi,math.pi/2)
    pose.orientation = geometry_msgs.msg.Quaternion(*q)
    points.append(pose)
    fraction = 0.0
    retryCount = 100
    count = 0
    path = None
    while fraction <1 and count < retryCount:
        (path,fraction) = armGroup.compute_cartesian_path(points,0.01,0)
        count += 1
    if fraction == 1.0:
        armGroup.execute(path)
    else:
        rospy.logerr("compute_cartesian_path fraction smaller than 1.0, nearTarget")

def attach(robot, scene,armGroup):
    gripperGroupName = "gripper_group"
    links = robot.get_link_names(gripperGroupName)
    scene.attach_box(armGroup.get_end_effector_link(),"target",touch_links=links)
# attach and detach
def detach(scene,armGroup):
    scene.remove_attached_object(armGroup.get_end_effector_link(),"target")

list = [0.52, 0.26, 0, -0.26, -0.52]
index = 0
def move(armGroup):
    """
    move the target along x=axia
    :param armGroup:
    :return:
    """
    global index
    points=[]
    current = armGroup.get_current_pose().pose
    points.append(current)
    if index >4:
        index = 0
    pose = geometry_msgs.msg.Pose()
    pose.position.x =current.position.x + list[index]
    pose.position.y =current.position.y
    pose.position.z =current.position.z
    pose.orientation = current.orientation
    points.append(pose)
    fraction = 0.0
    retryCount = 100
    count = 0
    path = None
    while fraction <1 and count < retryCount:
        (path,fraction) = armGroup.compute_cartesian_path(points,0.01,0.0,avoid_collisions=False)
        count += 1
    if fraction == 1.0:
        armGroup.execute(path)
    else:
        rospy.logerr("compute_cartesian_path fraction smaller than 1.0,move function")

    index += 1


def farFromTarget(armGroup):
    points = []
    current = armGroup.get_current_pose().pose
    points.append(current)
    pose = geometry_msgs.msg.Pose()
    pose.position.x = current.position.x
    pose.position.y = current.position.y
    pose.position.z = current.position.z + 0.15
    q = quaternion_from_euler(0,math.pi,math.pi/2)
    pose.orientation = geometry_msgs.msg.Quaternion(*q)
    points.append(pose)
    fraction = 0.0
    path = None
    retryCount = 100
    count = 0
    while fraction <1 and count < retryCount:
        (path,fraction) = armGroup.compute_cartesian_path(points,0.01,0)
        count += 1
    if fraction == 1.0:
        armGroup.execute(path)
    else:
        rospy.logerr("compute_cartesian_path fraction smaller than 1.0,farFromTarget Function")


def goHome(armGroup):
    armGroup.set_named_target("arm_home")
    armGroup.go()
    armGroup.stop()
    armGroup.clear_pose_targets()


isPlace = False

def oncePickPlace(robot,scene,armGroup,gripperGroup):
    global isPlace
    isPlace = False
    rate = rospy.Rate(10)
    result = initArm(armGroup)
    if result == False:
        rospy.logerr("escape camera failure")

    addBelt(robot,scene)
    # start_time = rospy.Time.now().to_sec()
    # target = getTarget()
    target = getPresuppositionTarget()
    # print(target)



    if(target is not None):
        if (target is not None):
            if target[1] > 1.0:
                return
        pose = addTarget(robot,scene,target)
        rospy.sleep(0.1)
        # move arm about target
        moveUp(armGroup,pose)
        openGripper(gripperGroup)
        #close to target along z axiax
        nearTarget(armGroup)
        # wait unitl inGripper become true
        # in Gripper get from param service
        while isPlace == False:
            if rospy.get_param("inGripper",False):
                attach(robot,scene,armGroup)
                closeGripper(gripperGroup)
                # move the target
                move(armGroup)
                # open and detach
                openGripper(gripperGroup)
                detach(scene,armGroup)
                # away from target along z axis
                farFromTarget(armGroup)
                #     go to home position
                goHome(armGroup)
                isPlace = True
            else:
                rate.sleep()

if __name__ == '__main__':
    nodeName = 'pick_place_node'
    rospy.init_node(nodeName)
    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    armGroupName = "arm_group"
    gripperGroupName = "gripper_group"
    armGroup = moveit_commander.MoveGroupCommander(armGroupName)
    gripperGroup = moveit_commander.MoveGroupCommander(gripperGroupName)
    #  allow replanning
    armGroup.allow_replanning(True)
    armGroup.allow_replanning(True)
    rospy.sleep(1)
    objects = scene.get_objects()
    for item in objects:
        scene.remove_world_object(item)
    rate = rospy.Rate(10)
    while rospy.is_shutdown() ==False:
        oncePickPlace(robot,scene,armGroup,gripperGroup)
        rate.sleep()



    rospy.spin()
    #1. move armgroup away from camera
