import actionlib
from control_msgs.msg import GripperCommandAction
from control_msgs.msg import GripperCommandGoal
import rospy
from sensor_msgs.msg import JointState
from threading import Lock

from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from moveit_msgs.msg import MoveItErrorCodes
from moveit_python import MoveGroupInterface, PlanningSceneInterface

# Enrique: I recommend create a set of classes for the robot actions
class FetchAnimations():

    # ENRIQUE: Same that tutorial
    def __init__(self):
        # Create move group interface for a fetch robot
        self.move_group = MoveGroupInterface("arm_with_torso", "base_link")

        # Define ground plane
        # This creates objects in the planning scene that mimic the ground
        # If these were not in place gripper could hit the ground
        self.planning_scene = PlanningSceneInterface("base_link")
        self.planning_scene.removeCollisionObject("my_front_ground")
        self.planning_scene.removeCollisionObject("my_back_ground")
        self.planning_scene.removeCollisionObject("my_right_ground")
        self.planning_scene.removeCollisionObject("my_left_ground")
        self.planning_scene.addCube("my_front_ground", 2, 1.1, 0.0, -1.0)
        self.planning_scene.addCube("my_back_ground", 2, -1.2, 0.0, -1.0)
        self.planning_scene.addCube("my_left_ground", 2, 0.0, 1.2, -1.0)
        self.planning_scene.addCube("my_right_ground", 2, 0.0, -1.2, -1.0)

        # TF joint names
        self.joint_names = ["torso_lift_joint", "shoulder_pan_joint",
                    "shoulder_lift_joint", "upperarm_roll_joint",
                    "elbow_flex_joint", "forearm_roll_joint",
                    "wrist_flex_joint", "wrist_roll_joint"]

    # ENRIQUE: Rize primitive for action engine, 2 arguments: value and parameters
    def animation(self, value, parameters):
        if (value == "disco"):
            self.disco(parameters)
        else:
            self.wave(parameters)
        return "success"

    # ENRIQUE: Same that tutorial
    def disco(self,parameters):
        # ENRIQUE: Use this python dictionary to change the behavior of the action 
        print(parameters)

        # Lists of joint angles in the same order as in joint_names
        disco_poses = [[0.0, 1.5, -0.6, 3.0, 1.0, 3.0, 1.0, 3.0],
                    [0.133, 0.8, 0.75, 0.0, -2.0, 0.0, 2.0, 0.0],
                    [0.266, -0.8, 0.0, 0.0, 2.0, 0.0, -2.0, 0.0],
                    [0.385, -1.5, 1.1, -3.0, -0.5, -3.0, -1.0, -3.0],
                    [0.266, -0.8, 0.0, 0.0, 2.0, 0.0, -2.0, 0.0],
                    [0.133, 0.8, 0.75, 0.0, -2.0, 0.0, 2.0, 0.0],
                    [0.0, 1.5, -0.6, 3.0, 1.0, 3.0, 1.0, 3.0]]

        for pose in disco_poses:
            if rospy.is_shutdown():
                break

            # Plans the joints in joint_names to angles in pose
            self.move_group.moveToJointPosition(self.joint_names, pose, wait=False)

            # Since we passed in wait=False above we need to wait here
            self.move_group.get_move_action().wait_for_result()
            result = self.move_group.get_move_action().get_result()

            if result:
                # Checking the MoveItErrorCode
                if result.error_code.val == MoveItErrorCodes.SUCCESS:
                    rospy.loginfo("Disco!")
                else:
                    # If you get to this point please search for:
                    # moveit_msgs/MoveItErrorCodes.msg
                    rospy.logerr("Arm goal in state: %s",
                                self.move_group.get_move_action().get_state())
            else:
                rospy.logerr("MoveIt! failure no result returned.")

        # This stops all arm movement goals
        # It should be called when a program is exiting so movement stops
        self.move_group.get_move_action().cancel_all_goals()
        

    # ENRIQUE: Same that tutorial
    def wave(self,parameters):
        # ENRIQUE: Use this python dictionary to change the behavior of the action 
        print(parameters)

        # This is the wrist link not the gripper itself
        gripper_frame = 'wrist_roll_link'
        # Position and rotation of two "wave end poses"
        gripper_poses = [Pose(Point(0.042, 0.384, 1.826),
                            Quaternion(0.173, -0.693, -0.242, 0.657)),
                        Pose(Point(0.047, 0.545, 1.822),
                            Quaternion(-0.274, -0.701, 0.173, 0.635))]

        # Construct a "pose_stamped" message as required by moveToPose
        gripper_pose_stamped = PoseStamped()
        gripper_pose_stamped.header.frame_id = 'base_link'

        
        for pose in gripper_poses:
            # Finish building the Pose_stamped message
            # If the message stamp is not current it could be ignored
            gripper_pose_stamped.header.stamp = rospy.Time.now()
            # Set the message pose
            gripper_pose_stamped.pose = pose

            # Move gripper frame to the pose specified
            self.move_group.moveToPose(gripper_pose_stamped, gripper_frame)
            result = self.move_group.get_move_action().get_result()

            if result:
                # Checking the MoveItErrorCode
                if result.error_code.val == MoveItErrorCodes.SUCCESS:
                    rospy.loginfo("Hello there!")
                else:
                    # If you get to this point please search for:
                    # moveit_msgs/MoveItErrorCodes.msg
                    rospy.logerr("Arm goal in state: %s",
                                self.move_group.get_move_action().get_state())
            else:
                rospy.logerr("MoveIt! failure no result returned.")

        # This stops all arm movement goals
        # It should be called when a program is exiting so movement stops
        self.move_group.get_move_action().cancel_all_goals()


# ENRIQUE: Same that tutorial
class Gripper(object):

    OPEN_POSITION = 0.1
    CLOSED_POSITION = 0

    def __init__(self):
        self._lock = Lock()

        self.max_effort = 20.0
        self.position = None
        self._sub_pos = rospy.Subscriber('joint_states', JointState,
                                         self._set_state)

        self.action_name = 'gripper_controller/gripper_action'
        self.client = actionlib.SimpleActionClient(self.action_name,
                                                   GripperCommandAction)
        rospy.loginfo('Waiting for gripper_controller...')
        self.client.wait_for_server()
        rospy.loginfo('...connected.')

    def _set_state(self, joint_state):
        l_gripper_position = None
        r_gripper_position = None
        for joint, pos in zip(joint_state.name, joint_state.position):
            if joint == 'l_gripper_finger_joint':
                l_gripper_finger_pos = pos
            if joint == 'r_gripper_finger_joint':
                r_gripper_finger_pos = pos
        with self._lock:
            self.position = l_gripper_finger_pos + r_gripper_finger_pos

    def set_position(self, position):
        goal = GripperCommandGoal()
        goal.command.max_effort = self.max_effort
        goal.command.position = position
        # Fill in the goal here
        self.client.send_goal(goal)
        self.client.wait_for_result(rospy.Duration.from_sec(5.0))
        res = self.client.get_result()
        with self._lock:
            self.position = res.position

    # ENRIQUE: Rize primitive for action engine, 2 arguments: value and parameters
    def open(self, value, parameters):
        # ENRIQUE: Use this python dictionary to change the behavior of the action 
        print(parameters)
        self.set_position(self.OPEN_POSITION)
        return "success"

    # ENRIQUE: Rize primitive for action engine, 2 arguments: value and parameters
    def close(self, value, parameters):
        # ENRIQUE: Use this python dictionary to change the behavior of the action 
        print(parameters)
        self.set_position(self.CLOSED_POSITION)
        return "success"

    # Function added for RIZE




   