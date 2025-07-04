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
from gripper_cam import OneShotArucoDetector

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
execution_status_pub = None


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

    # arm_pub.publish(traj_msg)
    goal = FollowJointTrajectoryGoal()
    goal.trajectory = traj_msg

    # ! Debug or Execute
    # rospy.logdebug("debug mode: no execution")
    arm_client.send_goal(goal)
    arm_client.wait_for_result()
    rospy.loginfo("Arm execution finished")

def move_arm_cartesian(goal_position, goal_orientation, duration=3.5):
    desired_position = Vector(*goal_position)
    desired_orientation = Rotation.RPY(*goal_orientation)
    desired_frame = Frame(desired_orientation, desired_position)
    desired_joint_positions = JntArray(number_of_joints)

    result = ik_solver_pos.CartToJnt(current_joint_positions, desired_frame, desired_joint_positions)
    if result >= 0:
        move_arm_joints([desired_joint_positions[i] for i in range(number_of_joints)], duration)
    else:
        rospy.logerr("IK solution not found.")
        # ! Error solution 1
        go_home_position()
        raise RuntimeError("Gripper ArUco detection failed. Aborting plan.")
        # ! Error solution 2: TTS "I need help"

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
    global gripper_aruco_array, gripper_aruco_locked, gripper_aruco_miss
    if gripper_aruco_miss:
        rospy.logdebug("[Callback] Detection previously marked as failed. Ignoring message.")
        return
    
    rospy.loginfo(f"[Callback] Got pose from gripper_cam: {msg.point}")
    if not gripper_aruco_locked:
        gripper_aruco_array = np.array([msg.point.x, msg.point.y, msg.point.z], dtype=np.float32)
        gripper_aruco_locked = True
        rospy.loginfo("[Callback] ArUco pose locked.")
    else:
        rospy.logdebug("[Callback] ArUco already locked, ignoring.")

# === Instruction Handling ===
def instruction_callback(msg):
    global current_marker_id, plan_executing, execution_status_pub

    with plan_lock:
        if plan_executing:
            rospy.logwarn("Plan already executing. Ignoring new instruction.")
            return
        plan_executing = True
    
    if execution_status_pub:
        execution_status_pub.publish("received")

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
                        execution_status_pub.publish("failed")
                        break
                else:
                    rospy.logwarn(f"Unknown primitive: {primitive}")
    except Exception as e:
        rospy.logerr(f"Error in instruction_callback: {e}")
        execution_status_pub.publish("failed")
    finally:
        with plan_lock:
            plan_executing = False
        if execution_status_pub:
            execution_status_pub.publish("finished")
            rospy.sleep(1.0)
            execution_status_pub.publish("waiting")

# === Utility ===
def postion_adjust(x_pixel, y_pixel):
    x_diff = (200 - x_pixel) * 0.013 / 82
    y_diff = (343 - y_pixel) * 0.015 / 65
    return x_diff, y_diff

# === Primitives ===
def search_head(): # ready
    global head_aruco_locked
    head_aruco_locked = False

    rospy.sleep(0.1) 
    detector = HeadCamArucoDetector()

    # Start spinning in the background
    spin_thread = threading.Thread(target=detector.spin, daemon=True)
    spin_thread.start()
    
    # ! execution
    detector.search_for_marker(current_marker_id)

def move_to_open(): # TODO: adjust offset
    global stored_head_aruco
    rospy.loginfo("Execute move to open")
    
    head_aurco_postion = np.array([head_aruco_array[0], head_aruco_array[1], 0.0])
    if head_aruco_array[0]>0.8:
        offset = [-0.08, 0.0, -0.18]
    else:
        offset = [-0.03, 0.0, -0.18]

    goal = head_aurco_postion + np.array(offset)
    stored_head_aruco = head_aurco_postion.copy()

    # ! execution
    move_arm_cartesian(goal.tolist(), [0, 0, pi/2])

def move_to_close(): # TODO
    rospy.loginfo("Execute move to close")
    head_aurco_postion = np.array([head_aruco_array[0], head_aruco_array[1], 0.0])
    if head_aruco_array[0]>0.8:
        offset = [-0.08, 0.0, -0.22]
    else:
        offset = [-0.03, 0.0, -0.22]
    goal = head_aurco_postion + np.array(offset)

    # ! execution
    move_arm_cartesian(goal.tolist(), [0, 0, pi/2])


def detect_aruco_with_gripper_camera(): # TODO: adjust parameters
    global gripper_aruco_locked, gripper_aruco_miss, stored_head_aruco
    gripper_aruco_locked = False
    gripper_aruco_miss = False

    if current_joint_positions is None:
        rospy.logerr("No joint positions available. Cannot compute pose.")
        return

    rospy.loginfo("Starting gripper camera detection...")

    rospy.sleep(0.1)

    # Launch remote camera stream
    OneShotArucoDetector(target_marker_id=current_marker_id, auto_shutdown=False, start_remote=True)

    rospy.sleep(0.5)  # allow callback to capture the pose
    rospy.loginfo("Waiting for gripper ArUco pose...")

    # Determine primary direction based on gripper position
    if stored_head_aruco[0] >= 0.7 and -0.3 <= stored_head_aruco[1] <= 0.35:
        search_directions = ["down", "backward", "backward"]
        search_steps = [0.02, 0.04, 0.04]
    elif stored_head_aruco[0] >= 0.7 and stored_head_aruco[1] < -0.3:
        search_directions = ["down", "backward", "left", "left"]
        search_steps = [0.02, 0.04, 0.03, 0.03]
    elif stored_head_aruco[0] >= 0.7 and stored_head_aruco[1] > 0.35:
        search_directions = ["down", "backward", "right", "right"]
        search_steps = [0.02, 0.04, 0.03, 0.03]
    elif stored_head_aruco[0] < 0.7 and stored_head_aruco[1] < -0.3:
        search_directions = ["up", "left", "left"]
        search_steps = [0.01, 0.03, 0.03]
    elif stored_head_aruco[0] < 0.7 and stored_head_aruco[1] > 0.35:
        search_directions = ["up", "right", "right"]
        search_steps = [0.01, 0.03, 0.03]
    else:
        search_directions = ["left", "right", "forward", "backward"]
        search_steps = [0.02, 0.04, 0.02, 0.04]


    attempt = 0
    max_attempts = 4

    while not gripper_aruco_locked and not gripper_aruco_miss and attempt < max_attempts:
        direction = search_directions[attempt % len(search_directions)]
        step = search_steps[attempt % len(search_directions)]
        move_arm(direction, step)
        rospy.sleep(0.5)
        attempt += 1

    if not gripper_aruco_locked and not gripper_aruco_miss:
        rospy.logerr("Gripper ArUco marker not detected after search attempts.")
        gripper_aruco_miss=True
        # return
        # ! Error solution 1
        go_home_position()
        raise RuntimeError("Gripper ArUco detection failed. Aborting plan.")
        # ! Error solution 2: TTS "I need help"

    rospy.loginfo("Gripper ArUco pose received.")

    # Get current pose of gripper via FK
    current_pose = Frame()
    fk_solver.JntToCart(current_joint_positions, current_pose)
    pos = current_pose.p
    rot = current_pose.M
    rpy = rot.GetRPY()

    offset = np.array([0.0, 0.0, -0.22])

    aruco_pos = gripper_aruco_array[:2]
    dx, dy = postion_adjust(aruco_pos[1], aruco_pos[0])

    corrected_xy = [pos[0] + dx,
                    pos[1] + dy]
    
    goal = [
        corrected_xy[0] + offset[0],
        corrected_xy[1] + offset[1],
        offset[2]
    ]
    print(goal)

    # ! execution
    move_arm_cartesian(goal, [0, 0, pi/2], 1.5)

def open_gripper(): # ready
    rospy.loginfo("Execute open gripper")
    send_gripper_goal([1, 1])
    rospy.sleep(0.2)
    rospy.loginfo("Finish open gripper")


def close_gripper(): # ready
    rospy.loginfo("Execute close gripper")
    send_gripper_goal([0.035, 0.035])
    rospy.sleep(0.2)
    rospy.loginfo("Finish close gripper")

def send_gripper_goal(positions): # ready
    goal = FollowJointTrajectoryGoal()
    traj = JointTrajectory()
    traj.joint_names = ['gripper_left_finger_joint', 'gripper_right_finger_joint']
    pt = JointTrajectoryPoint()
    pt.positions = positions
    pt.time_from_start = rospy.Duration(1.5)
    traj.points.append(pt)
    goal.trajectory = traj

    # ! execution
    gripper_client.send_goal(goal)
    gripper_client.wait_for_result()

def rotate_head(pan, tilt, duration=2.0):
    goal = FollowJointTrajectoryGoal()
    trajectory = JointTrajectory()
    trajectory.joint_names = ['head_1_joint', 'head_2_joint']

    point = JointTrajectoryPoint()
    point.positions = [pan, tilt]
    point.time_from_start = rospy.Duration(duration)
    trajectory.points.append(point)

    goal.trajectory = trajectory
    head_client.send_goal(goal)
    head_client.wait_for_result()

def move_arm(direction, step=0.03): # ready
    if current_joint_positions is None:
        rospy.logwarn("No current joint positions. Cannot move arm.")
        return
    
    # Get current end-effector pose
    current_pose = Frame()
    rospy.sleep(0.5)

    fk_solver.JntToCart(current_joint_positions, current_pose)

    pos = current_pose.p
    rot = current_pose.M

    dx, dy, dz = 0.0, 0.0, 0.0

    if direction == "forward":
        rospy.loginfo("Execute gripper move forward.")
        dx = step
    elif direction == "backward":
        rospy.loginfo("Execute gripper move backwarad.")
        dx = -step
    elif direction == "left":
        rospy.loginfo("Execute gripper move left.")
        dy = step
    elif direction == "right":
        rospy.loginfo("Execute gripper move right.")
        dy = -step
    elif direction == "up":
        rospy.loginfo("Execute gripper move up.")
        dz = step
    elif direction == "down":
        rospy.loginfo("Execute gripper move down.")
        dz = -step
    else:
        rospy.logwarn(f"Unknown direction '{direction}' passed to move_arm.")
        return

    # Compute new goal position
    new_pos = [pos[0] + dx, pos[1] + dy, pos[2] + dz]
    rpy = rot.GetRPY()  # preserve orientation

    rospy.loginfo(f"Moving arm {direction}, new pos: {new_pos}")

    # ! execution
    move_arm_cartesian(new_pos, rpy, duration=1.0)

def move_up(step=0.04): # ready
    rospy.loginfo("Execute move_up primitive.")
    move_arm("up", step)

def move_down(step=0.02): # ready
    rospy.loginfo("Execute move_down primitive.")
    move_arm("down", step)

def move_left(step=0.03): # ready
    rospy.loginfo("Execute move_up primitive.")
    move_arm("left", step)

def move_right(step=0.03): # ready
    rospy.loginfo("Execute move_up primitive.")
    move_arm("right", step)

def move_forward(step=0.03): # ready
    rospy.loginfo("Execute move_up primitive.")
    move_arm("forward", step)

def move_backward(step=0.03): # ready
    rospy.loginfo("Execute move_up primitive.")
    move_arm("backward", step)

def get_current_arm_position(): # ready
    rospy.loginfo("get_current_arm_position primitive.")

def go_home_position(): # ready
    rospy.loginfo("Reseting to home position")
    home_joint_list = [0.07, 0.7, -1.3, 1.68, 0.72, -1.29, 0.16]

    # ! execution
    move_arm_joints(home_joint_list, 4.0)
    open_gripper()
    rotate_head(0.0, 0.0)
    rospy.loginfo("Home position reached.")
    rospy.sleep(0.1)

def move_away_clear_view(): # TODO: avoid arm blocking the view
    rospy.loginfo("move to clear view position")
    home_joint_list = [0.42, 0.9, -1.24, 1.49, 0.95, -1.39, 0.26]

    # ! execution
    move_arm_joints(home_joint_list, 3.0)
    rospy.loginfo("Clear view position reached.")
    rospy.sleep(0.2)

# === Init & Main ===
def run():
    global ik_solver_pos, fk_solver, arm_pub, gripper_client, head_client, arm_client, current_joint_positions, execution_status_pub

    # # ! for gazebo only: Wait for simulated time to start running
    # rospy.loginfo("Waiting for simulated time to be active...")
    # while rospy.Time.now().to_sec() == 0:
    #     rospy.sleep(0.1)
    # rospy.loginfo("Simulated time has started.")

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

    # arm_pub = rospy.Publisher('/arm_controller/command', JointTrajectory, queue_size=10)
    execution_status_pub = rospy.Publisher('/execution_status', String, queue_size=10)

    gripper_client = actionlib.SimpleActionClient('/parallel_gripper_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
    gripper_client.wait_for_server()

    head_client = actionlib.SimpleActionClient('/head_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
    head_client.wait_for_server()

    arm_client = actionlib.SimpleActionClient('/safe_arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
    arm_client.wait_for_server()

    while current_joint_positions is None and not rospy.is_shutdown():
        rospy.info("Waiting for the first joint state.")
        rospy.sleep(0.1)

    rospy.loginfo("Controllers connected. Waiting for plans.")
    execution_status_pub.publish("waiting")

    # open_gripper()
    # go_home_position()
    


    rospy.spin()

if __name__ == "__main__":
    rospy.init_node("execute_control", anonymous=True)
    run()
