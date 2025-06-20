#!/usr/bin/env python3
import rospy
import json
from std_msgs.msg import String
from sensor_msgs.msg import JointState, Image
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
import actionlib
from kdl_parser_py.urdf import treeFromUrdfModel
from urdf_parser_py.urdf import URDF
from PyKDL import ChainFkSolverPos_recursive, ChainIkSolverPos_LMA, Frame, Vector, Rotation, JntArray
import numpy as np

# === Global Variables ===
current_joint_positions = None
joint_names = [
    "arm_1_joint", "arm_2_joint", "arm_3_joint", "arm_4_joint",
    "arm_5_joint", "arm_6_joint", "arm_7_joint"
]
number_of_joints = len(joint_names)
arm_client = None
gripper_client = None
head_client = None
status_pub = None

# ArUco detection state
global last_aruco_detected, bridge, aruco_dict, aruco_params
last_aruco_detected = False
bridge = CvBridge()
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_100)
aruco_params = aruco.DetectorParameters()

# === Joint State Callback ===
def joint_state_callback(msg):
    global current_joint_positions
    arr = JntArray(number_of_joints)
    try:
        for i, name in enumerate(joint_names):
            idx = msg.name.index(name)
            arr[i] = msg.position[idx]
        current_joint_positions = arr
    except ValueError:
        pass

# === Camera Callback for ArUco ===
def image_callback(msg):
    global last_aruco_detected
    try:
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)
        last_aruco_detected = ids is not None
    except Exception as e:
        rospy.logerr(f"[execute_action] ArUco detection error: {e}")
        last_aruco_detected = False

# === Primitive Actions ===
def get_current_arm_position():
    return current_joint_positions

def open_gripper():
    goal = FollowJointTrajectoryGoal()
    traj = JointTrajectory()
    traj.joint_names = ["gripper_left_finger_joint", "gripper_right_finger_joint"]
    pt = JointTrajectoryPoint(positions=[0.04, 0.04], time_from_start=rospy.Duration(1.0))
    traj.points.append(pt)
    goal.trajectory = traj
    gripper_client.send_goal(goal)
    gripper_client.wait_for_result()

def close_gripper():
    goal = FollowJointTrajectoryGoal()
    traj = JointTrajectory()
    traj.joint_names = ["gripper_left_finger_joint", "gripper_right_finger_joint"]
    pt = JointTrajectoryPoint(positions=[0.0, 0.0], time_from_start=rospy.Duration(1.0))
    traj.points.append(pt)
    goal.trajectory = traj
    gripper_client.send_goal(goal)
    gripper_client.wait_for_result()

def move_up():
    pose = get_current_arm_position()
    if pose:
        target = [pose[i] for i in range(number_of_joints)]
        target[2] += 0.05
        move_arm_to(target)

def move_arm_to(position):
    goal = FollowJointTrajectoryGoal()
    traj = JointTrajectory()
    traj.joint_names = joint_names
    pt = JointTrajectoryPoint(positions=position, time_from_start=rospy.Duration(3.0))
    traj.points.append(pt)
    goal.trajectory = traj
    arm_client.send_goal(goal)
    arm_client.wait_for_result()

def go_home_position():
    home = [0.0, -1.0, 0.0, 1.5, 0.0, 1.0, 0.0]
    move_arm_to(home)

def search_head():
    goal = FollowJointTrajectoryGoal()
    traj = JointTrajectory()
    traj.joint_names = ["head_1_joint", "head_2_joint"]
    pt = JointTrajectoryPoint(positions=[0.5, -0.3], time_from_start=rospy.Duration(2.0))
    traj.points.append(pt)
    goal.trajectory = traj
    head_client.send_goal(goal)
    head_client.wait_for_result()

# === Gripper Camera Actions ===
def detect_aruco_with_gripper_camera(timeout=5.0):
    rospy.loginfo("[execute_action] Detecting ArUco marker via gripper camera...")
    start = rospy.Time.now().to_sec()
    global last_aruco_detected
    while rospy.Time.now().to_sec() - start < timeout:
        if last_aruco_detected:
            rospy.loginfo("[execute_action] ArUco detected!")
            return True
        rospy.sleep(0.1)
    rospy.logwarn("[execute_action] ArUco detection timed out.")
    return False

def explore_area_near_target():
    rospy.loginfo("[execute_action] Exploring area to retry detection...")
    rospy.sleep(1.0)

# === Execute Sequence Callback ===
def sequence_callback(msg):
    data = json.loads(msg.data)
    action = data.get('action')
    sequence = data.get('sequence', [])

    status_pub.publish(f"started:{action}")
    rospy.loginfo(f"[execute_action] Starting action {action}")

    for primitive in sequence:
        try:
            rospy.loginfo(f"[execute_action] Executing {primitive}")
            status_pub.publish(f"started:{primitive}")
            result = globals()[primitive]() if primitive in globals() else None
            if primitive == 'detect_aruco_with_gripper_camera' and not result:
                raise Exception('ArUco not detected')
            status_pub.publish(f"completed:{primitive}")
        except Exception as e:
            rospy.logerr(f"[execute_action] Primitive {primitive} failed: {e}")
            status_pub.publish(f"failed:{primitive}")
            status_pub.publish(f"failed:{action}")
            return

    status_pub.publish(f"completed:{action}")
    rospy.loginfo(f"[execute_action] Completed action {action}")

# === Main ===
def main():
    global arm_client, gripper_client, head_client, status_pub

    rospy.init_node('execute_action', anonymous=True)
    arm_client = actionlib.SimpleActionClient(
        '/arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction
    )
    gripper_client = actionlib.SimpleActionClient(
        '/parallel_gripper_controller/follow_joint_trajectory', FollowJointTrajectoryAction
    )
    head_client = actionlib.SimpleActionClient(
        '/head_controller/follow_joint_trajectory', FollowJointTrajectoryAction
    )

    rospy.Subscriber('/joint_states', JointState, joint_state_callback)
    rospy.Subscriber('/gripper_camera/image_raw', Image, image_callback)
    rospy.Subscriber('/action_agent/execute_sequence', String, sequence_callback)
    status_pub = rospy.Publisher('/action_agent/status', String, queue_size=10)

    rospy.loginfo("[execute_action] Waiting for controllers...")
    arm_client.wait_for_server()
    gripper_client.wait_for_server()
    head_client.wait_for_server()
    rospy.loginfo("[execute_action] Controllers and camera subscriber connected.")

    rospy.spin()

if __name__ == '__main__':
    main()

