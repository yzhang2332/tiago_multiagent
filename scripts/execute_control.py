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

def move_down():
    pose = get_current_arm_position()
    if pose:
        target = [pose[i] for i in range(number_of_joints)]
        target[2] -= 0.05
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

def move_to_open():
    position = [0.2, -1.2, 0.1, 1.7, 0.0, 1.2, 0.0]
    move_arm_to(position)

def move_to_close():
    position = [0.1, -1.1, 0.0, 1.5, 0.0, 1.0, 0.0]
    move_arm_to(position)

def search_head():
    goal = FollowJointTrajectoryGoal()
    traj = JointTrajectory()
    traj.joint_names = ["head_1_joint", "head_2_joint"]
    pt = JointTrajectoryPoint(positions=[0.5, -0.3], time_from_start=rospy.Duration(2.0))
    traj.points.append(pt)
    goal.trajectory = traj
    head_client.send_goal(goal)
    head_client.wait_for_result()

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
    try:
        data = json.loads(msg.data)
        plan = data.get('plan', [])

        for step in plan:
            action = step.get('action')
            sequence = step.get('sequence', [])
            marker_id = step.get('marker_id', None)

            status_pub.publish(f"started:{action}")
            rospy.loginfo(f"[execute_action] Starting action {action} targeting marker {marker_id}")

            for primitive in sequence:
                try:
                    rospy.loginfo(f"[execute_action] Executing {primitive}")
                    status_pub.publish(f"started:{primitive}")

                    if primitive == 'get_current_arm_position':
                        get_current_arm_position()
                        result = True
                    elif primitive in globals():
                        result = globals()[primitive]()
                    else:
                        raise Exception(f"Unknown primitive: {primitive}")

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

    except Exception as e:
        rospy.logerr(f"[execute_action] Failed to process plan: {e}")
        status_pub.publish("failed:plan")

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



import rospy
from PyKDL import Frame, Vector, Rotation, JntArray, ChainFkSolverPos_recursive, ChainIkSolverPos_LMA
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from std_msgs.msg import String
import numpy as np
import cv2
import cv2.aruco as aruco
from cv_bridge import CvBridge
from urdf_parser_py.urdf import URDF
from kdl_parser_py.urdf import treeFromUrdfModel

# === Setup global state ===
current_joint_positions = None
joint_names = ["arm_1_joint", "arm_2_joint", "arm_3_joint", "arm_4_joint", "arm_5_joint", "arm_6_joint", "arm_7_joint"]
number_of_joints = len(joint_names)

bridge = CvBridge()
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_100)
aruco_params = aruco.DetectorParameters()

# For IK
ik_solver = None
fk_solver = None
chain = None

# === Initialize KDL chain and solvers ===
def init_kdl():
    global ik_solver, fk_solver, chain

    robot_urdf = URDF.from_parameter_server()
    success, tree = treeFromUrdfModel(robot_urdf)
    if not success:
        rospy.logerr("Failed to extract KDL tree from URDF.")
        return

    # Use these link names from the other project (may need to change for your setup)
    base_link = "torso_lift_link"  # <-- validate this in your TF tree
    ee_link = "gripper_link"       # <-- validate this in your URDF or TF tree

    chain = tree.getChain(base_link, ee_link)
    ik_solver = ChainIkSolverPos_LMA(chain)
    fk_solver = ChainFkSolverPos_recursive(chain)

# === Joint State Callback ===
def joint_state_callback(msg):
    global current_joint_positions
    current_joint_positions = JntArray(number_of_joints)
    try:
        for i, name in enumerate(joint_names):
            idx = msg.name.index(name)
            current_joint_positions[i] = msg.position[idx]
    except ValueError:
        pass

# === Move to Cartesian Pose ===
def move_to_pose(position, orientation_rpy, duration):
    if current_joint_positions is None:
        rospy.logwarn("No joint state received yet.")
        return

    desired_frame = Frame(Rotation.RPY(*orientation_rpy), Vector(*position))
    q_out = JntArray(number_of_joints)
    result = ik_solver.CartToJnt(current_joint_positions, desired_frame, q_out)

    if result < 0:
        rospy.logerr("IK failed for desired pose.")
        return

    joint_dict = {joint_names[i]: q_out[i] for i in range(number_of_joints)}
    apply_joint_positions_with_interp(joint_dict, duration)

# === Interpolated Trajectory Publisher ===
def apply_joint_positions_with_interp(joint_position_dict, duration):
    pub = rospy.Publisher('/arm_controller/command', JointTrajectory, queue_size=10)

    start = [current_joint_positions[i] for i in range(number_of_joints)]
    end = [joint_position_dict[joint_names[i]] for i in range(number_of_joints)]
    num_points = 100

    # Cosine interpolation
    def cosine_interp(a, b, n):
        t = np.linspace(0, 1, n)
        t = (1 - np.cos(t * np.pi)) / 2
        return [(1-tt)*a + tt*b for tt in t]

    trajectory = JointTrajectory()
    trajectory.joint_names = joint_names

    for i in range(num_points):
        point = JointTrajectoryPoint()
        point.positions = [cosine_interp(start[j], end[j], num_points)[i] for j in range(number_of_joints)]
        point.time_from_start = rospy.Duration(i * duration / (num_points - 1))
        trajectory.points.append(point)

    pub.publish(trajectory)

# === ArUco Marker Pixel Pose Publisher ===
def image_callback(msg):
    try:
        cv_img = bridge.imgmsg_to_cv2(msg, "bgr8")
        gray = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)

        if ids is not None:
            for i, corner in enumerate(corners):
                marker_id = ids[i][0]
                avg_x = int(np.mean(corner[0][:, 0]))
                avg_y = int(np.mean(corner[0][:, 1]))

                # Simulated publishing of marker centroid
                msg = JointState()
                msg.name = [str(marker_id)]
                msg.position = [avg_x, avg_y]
                aruco_pub.publish(msg)
    except Exception as e:
        rospy.logerr(f"Aruco detection failed: {e}")

# === Pixel to World Correction (Simulated) ===
def pixel_to_world_correction(x_pixel, y_pixel):
    # Simulated calibration transform (replace with empirical or camera model based)
    x_offset = (200 - x_pixel) * 0.013 / 82
    y_offset = (343 - y_pixel) * 0.012 / 65
    return x_offset, y_offset

# === ROS Node Setup ===
def main():
    global aruco_pub
    rospy.init_node("enhanced_execute_control")

    rospy.Subscriber("/joint_states", JointState, joint_state_callback)
    rospy.Subscriber("/gripper_camera/image_raw", Image, image_callback)
    aruco_pub = rospy.Publisher("/aruco_pose_robot_camera", JointState, queue_size=10)

    init_kdl()
    rospy.loginfo("Enhanced control node started.")
    rospy.spin()

if __name__ == '__main__':
    main()
