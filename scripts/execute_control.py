#!/usr/bin/env python3

import rospy
import json
import cv2
import cv2.aruco as aruco
import numpy as np
import tf
import tf_conversions
import tf2_ros
import threading
from threading import Lock
import actionlib
from math import pi

from sensor_msgs.msg import JointState, Image
from geometry_msgs.msg import PointStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from std_msgs.msg import String

from urdf_parser_py.urdf import URDF
from kdl_parser_py.urdf import treeFromUrdfModel
from PyKDL import ChainFkSolverPos_recursive, ChainIkSolverPos_LMA, Frame, Vector, Rotation, JntArray

from head_cam import HeadCamArucoDetector

# === Globals ===
joint_names = ["arm_1_joint", "arm_2_joint", "arm_3_joint", "arm_4_joint", "arm_5_joint", "arm_6_joint", "arm_7_joint"]
current_joint_positions = None
current_gripper_position = [0, 0]
current_head_position = [0, 0]
head_aruco_array = np.zeros(3)
gripper_aruco_array = np.zeros(3)
current_marker_id = None

# IK solvers
ik_solver_pos = None
fk_solver = None
chain = None
number_of_joints = 7

# ROS interfaces
arm_pub = None
gripper_client = None
head_client = None
arm_client = None

# message lock
head_aruco_locked = False
gripper_aruco_locked = False
plan_lock = Lock()
plan_executing = False


# === Interpolation and Control ===
def cosine_interp_vals(a, b, num_points):
    t = np.linspace(0, 1, num_points)
    t = (1 - np.cos(t * np.pi)) / 2
    return [(1-tt) * a + tt * b for tt in t]

def move_arm_joints(end_joint_list, duration):
    if current_joint_positions is None:
        rospy.logerr("Current joint positions not yet received.")
        return
    
    start_joint_list = [current_joint_positions[i] for i in range(number_of_joints)]
    traj_msg = JointTrajectory()
    traj_msg.joint_names = joint_names
    num_interp = 100

    for i in range(num_interp):
        t = i / (num_interp - 1)
        interp = [cosine_interp_vals(start_joint_list[j], end_joint_list[j], num_interp)[i] for j in range(number_of_joints)]
        point = JointTrajectoryPoint()
        point.positions = interp
        point.time_from_start = rospy.Duration(t * duration)
        traj_msg.points.append(point)

    arm_pub.publish(traj_msg)

def move_arm_cartesian(goal_position, goal_orientation, duration=3.0):
    desired_position = Vector(*goal_position)
    desired_orientation = Rotation.RPY(*goal_orientation)
    desired_frame = Frame(desired_orientation, desired_position)
    desired_joint_positions = JntArray(number_of_joints)

    result = ik_solver_pos.CartToJnt(current_joint_positions, desired_frame, desired_joint_positions)
    if result >= 0:
        move_arm_joints([desired_joint_positions[i] for i in range(number_of_joints)], duration)
    else:
        rospy.logerr("IK solution not found.")

# === Subscribers ===
def joint_state_callback(msg):
    global current_joint_positions, current_gripper_position, current_head_position
    current_joint_positions = JntArray(number_of_joints)
    try:
        for i, name in enumerate(joint_names):
            idx = msg.name.index(name)
            current_joint_positions[i] = msg.position[idx]

        l_idx = msg.name.index("gripper_left_finger_joint")
        current_gripper_position = [msg.position[l_idx]*2] * 2

        h1_idx = msg.name.index("head_1_joint")
        h2_idx = msg.name.index("head_2_joint")
        current_head_position = [msg.position[h1_idx], msg.position[h2_idx]]

    except Exception as e:
        rospy.logerr(f"Joint state callback failed: {e}")

def head_aruco_pose_callback(msg):
    global head_aruco_array, head_aruco_locked
    rospy.loginfo(f"[Callback] Got pose from head_cam: {msg.point}")
    if not head_aruco_locked:
        head_aruco_array = np.array([msg.point.x, msg.point.y, msg.point.z], dtype=np.float32)
        head_aruco_locked = True
        rospy.loginfo("[Callback] ArUco pose locked.")
    else:
        rospy.logdebug("[Callback] ArUco already locked, ignoring.")

def gripper_aruco_pose_callback(msg):
    global gripper_aruco_array, gripper_aruco_locked
    if not gripper_aruco_locked:
        gripper_aruco_array = np.array([msg.point.x, msg.point.y, msg.point.z], dtype=np.float32)
        gripper_aruco_locked = True

# === Instruction Handling ===
def instruction_callback(msg):
    global current_marker_id, plan_executing

    with plan_lock:
        if plan_executing:
            rospy.logwarn("Plan already executing. Ignoring new instruction.")
            return
        plan_executing = True

    try:
        data = json.loads(msg.data)
        for step in data.get("plan", []):
            current_marker_id = step.get("marker_id")
            for primitive in step.get("sequence", []):
                func = globals().get(primitive)
                if callable(func):
                    rospy.loginfo(f"Executing: {primitive}")
                    try:
                        func()
                    except Exception as e:
                        rospy.logerr(f"Primitive '{primitive}' failed: {e}")
                        break
                else:
                    rospy.logwarn(f"Unknown primitive: {primitive}")
    except Exception as e:
        rospy.logerr(f"Error in instruction_callback: {e}")
    finally:
        with plan_lock:
            plan_executing = False

# === Utility ===
def postion_adjust(x_pixel, y_pixel):
    x_diff = (200 - x_pixel) * 0.013 / 82
    y_diff = (343 - y_pixel) * 0.010 / 65
    return x_diff, y_diff

# === Primitives ===
def search_head():
    global head_aruco_locked
    head_aruco_locked = False

    rospy.sleep(0.1) 
    detector = HeadCamArucoDetector()
    # threading.Thread(target=detector.spin, daemon=True).start()
    # detector.search_for_marker(current_marker_id)

    # Start spinning in the background
    spin_thread = threading.Thread(target=detector.spin, daemon=True)
    spin_thread.start()

    # Start search in a background thread as well
    search_thread = threading.Thread(target=detector.search_for_marker, args=(current_marker_id,), daemon=True)
    search_thread.start()

    # Optional wait
    timeout = rospy.Time.now() + rospy.Duration(5.0)
    while not head_aruco_locked and rospy.Time.now() < timeout:
        rospy.sleep(0.1)

    if not head_aruco_locked:
        rospy.logwarn("Head ArUco pose not received in time.")
        detector.stop()
        raise RuntimeError("Head ArUco detection timeout.")
    
    rospy.loginfo("Head ArUco successfully locked.")
    detector.stop()

def move_to_open():
    offset = [-0.03, 0.0, -0.15]
    goal = head_aruco_array + np.array(offset)
    move_arm_cartesian(goal.tolist(), [0, 0, pi/2])

def detect_aruco_with_gripper_camera():
    global gripper_aruco_locked
    gripper_aruco_locked = False
    rospy.sleep(0.5)  # allow callback to capture the pose
    wait_time = 2.0
    timeout = rospy.Time.now() + rospy.Duration(wait_time)

    while not gripper_aruco_locked and rospy.Time.now() < timeout:
        rospy.sleep(0.1)

    if not gripper_aruco_locked:
        rospy.logwarn("Gripper ArUco pose not received in time.")
        return
    
    offset = [-0.03, 0.0]
    pos = gripper_aruco_array[:2] + np.array(offset)
    dx, dy = postion_adjust(pos[0], pos[1])
    goal = [pos[0]+dx, pos[1]+dy, -0.22]
    move_arm_cartesian(goal.tolist(), [0, 0, pi/2])

def open_gripper():
    send_gripper_goal([0.04, 0.04])

def close_gripper():
    send_gripper_goal([0.0, 0.0])

def move_down():
    rospy.loginfo("Simulated move_down primitive.")

def move_up():
    rospy.loginfo("Simulated move_up primitive.")

def get_current_arm_position():
    rospy.loginfo("Simulated get_current_arm_position primitive.")

def send_gripper_goal(positions):
    goal = FollowJointTrajectoryGoal()
    traj = JointTrajectory()
    traj.joint_names = ['gripper_left_finger_joint', 'gripper_right_finger_joint']
    pt = JointTrajectoryPoint()
    pt.positions = positions
    pt.time_from_start = rospy.Duration(1.0)
    traj.points.append(pt)
    goal.trajectory = traj
    gripper_client.send_goal(goal)
    gripper_client.wait_for_result()

# === Init & Main ===
def run():
    global ik_solver_pos, fk_solver, arm_pub, gripper_client, head_client, arm_client, current_joint_positions

    # Wait for simulated time to start running
    rospy.loginfo("Waiting for simulated time to be active...")
    while rospy.Time.now().to_sec() == 0:
        rospy.sleep(0.1)
    rospy.loginfo("Simulated time has started.")

    robot = URDF.from_parameter_server()
    ok, tree = treeFromUrdfModel(robot)
    if not ok:
        rospy.logerr("Failed to load robot model.")
        return

    base, ee = "torso_lift_link", "gripper_link"
    chain = tree.getChain(base, ee)

    ik_solver_pos = ChainIkSolverPos_LMA(chain)
    fk_solver = ChainFkSolverPos_recursive(chain)
    current_joint_positions = JntArray(chain.getNrOfJoints())

    rospy.Subscriber('/joint_states', JointState, joint_state_callback)
    rospy.Subscriber('/head_cam_aruco_pose', PointStamped, head_aruco_pose_callback)
    rospy.Subscriber('/gripper_cam_aruco_pose', PointStamped, gripper_aruco_pose_callback)
    rospy.Subscriber('/action_agent/execute_sequence', String, instruction_callback)

    arm_pub = rospy.Publisher('/arm_controller/command', JointTrajectory, queue_size=10)

    gripper_client = actionlib.SimpleActionClient('/parallel_gripper_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
    gripper_client.wait_for_server()

    head_client = actionlib.SimpleActionClient('/head_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
    head_client.wait_for_server()

    arm_client = actionlib.SimpleActionClient('/arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
    arm_client.wait_for_server()

    rospy.loginfo("Controllers connected. Waiting for plans.")
    rospy.spin()

if __name__ == "__main__":
    rospy.init_node("execute_control", anonymous=True)
    run()
