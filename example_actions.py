# Default libraries
import nep
import time
from rize import*

## Aditional libraries
import rospy
from FetchRobotActions import*

# Start ROS node
rospy.init_node('grippper_keyboard')

robot = ActionEngine("ROS","listen")

# Robot dependet classes and funtions
gripper = Gripper()
ftanim = FetchAnimations()

# Set function for each primitive
robot_actions = {
    "open_hand": gripper.open,
    "close_hand": gripper.close,
    "animation": ftanim.animation,
}

# Set actions
robot.setRobotActions(robot_actions)

# Spin node
robot.spin()
