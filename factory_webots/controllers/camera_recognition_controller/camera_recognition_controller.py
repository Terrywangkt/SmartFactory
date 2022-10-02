"""camera_recognition_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
from controller import Camera
from controller import CameraRecognitionObject
import rospy
# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())
rospy.init_node("camera_recognition_controller")
# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getDevice('motorname')
#  ds = robot.getDevice('dsname')
#  ds.enable(timestep)

camera = robot.getDevice("camera")
camera.enable(timestep)
camera.recognitionEnable(timestep)


# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()

    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    if (camera.hasRecognition()):
        nums = camera.getRecognitionNumberOfObjects()
        objects = camera.getRecognitionObjects()
        # print(nums)
        # for i in range(nums):
            # item = objects[i]
            # target = [1.5,0,0.15]
        # print(item)
        # print(item.get_position())
        # print(item.get_orientation())
        # print(item.get_size())
        # print(item.get_position_on_image())
        # print(item.get_size_on_image())
        # pos = item.get_position()
        # target[0]= target[0]+pos[2]
        # target[1]= target[1]+pos[1]
        # target[2]= target[2]
        # print(target)
        # rospy.set_param("target",target)
        if (nums > 0):
            item = objects[-1]
            target = [1.5,0,0.15]
            pos = item.get_position()
            target[0]= target[0]+pos[2]
            target[1]= target[1]+pos[1]
            target[2]= target[2]
            # print(target)
            rospy.set_param("target",target)
        else:
            if rospy.has_param("target"):
                rospy.delete_param("target")
# Enter here exit cleanup code.
