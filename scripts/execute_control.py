#!/usr/bin/env python3
'''
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
'''


# import rospy
# import json
# from std_msgs.msg import String
# from sensor_msgs.msg import JointState, Image
# from cv_bridge import CvBridge
# import cv2
# import cv2.aruco as aruco
# from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
# from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
# from std_msgs.msg import Float32MultiArray, String
# from geometry_msgs.msg import PoseStamped, Quaternion
# import actionlib
# from kdl_parser_py.urdf import treeFromUrdfModel
# from urdf_parser_py.urdf import URDF
# from PyKDL import ChainFkSolverPos_recursive, ChainIkSolverPos_LMA, Frame, Vector, Rotation, JntArray
# import numpy as np
# from math import pi, inf
# import csv
# import sys

'''
# === Global Variables ===
current_joint_positions = None
last_detected_marker = None
joint_names = [
    "arm_1_joint", "arm_2_joint", "arm_3_joint", "arm_4_joint",
    "arm_5_joint", "arm_6_joint", "arm_7_joint"
]
number_of_joints = len(joint_names)
arm_client = None
gripper_client = None
head_client = None
status_pub = None

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

# === ArUco Marker Pose Callback ===
def aruco_pose_callback(msg):
    global last_detected_marker
    try:
        marker_id = int(msg.name[0])
        avg_x, avg_y = msg.position[0], msg.position[1]
        last_detected_marker = {
            'id': marker_id,
            'x': avg_x,
            'y': avg_y
        }
    except Exception as e:
        rospy.logerr(f"Failed to parse ArUco pose: {e}")

# === Pose Correction (Simulated) ===
def pixel_to_world_correction(x_pixel, y_pixel):
    x_offset = (200 - x_pixel) * 0.013 / 82
    y_offset = (343 - y_pixel) * 0.012 / 65
    return x_offset, y_offset

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
    rospy.loginfo("[execute_action] Waiting for ArUco marker from gripper camera...")
    start_time = rospy.Time.now().to_sec()
    while rospy.Time.now().to_sec() - start_time < timeout:
        if last_detected_marker:
            rospy.loginfo(f"[execute_action] Marker detected: {last_detected_marker}")
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
    rospy.Subscriber('/gripper_cam_aruco_pose', JointState, aruco_pose_callback)
    rospy.Subscriber('/head_cam_aruco_pose', JointState, aruco_pose_callback)
    rospy.Subscriber('/action_agent/execute_sequence', String, sequence_callback)
    status_pub = rospy.Publisher('/action_agent/status', String, queue_size=10)

    rospy.loginfo("[execute_action] Waiting for controllers...")
    arm_client.wait_for_server()
    gripper_client.wait_for_server()
    head_client.wait_for_server()
    rospy.loginfo("[execute_action] Controllers connected.")

    rospy.spin()

if __name__ == '__main__':
    main()
'''

import rospy
import json
from sensor_msgs.msg import JointState, Image
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from std_msgs.msg import Float32MultiArray, String
from geometry_msgs.msg import PoseStamped, Quaternion
import actionlib
import tf
import tf_conversions
import tf2_ros
from kdl_parser_py.urdf import treeFromUrdfModel
from urdf_parser_py.urdf import URDF
from PyKDL import ChainFkSolverPos_recursive, ChainIkSolverPos_LMA, Frame, Vector, Rotation, JntArray
import numpy as np
from math import pi, inf
import csv
import sys

# ! Smooth movement control
def cosine_interp_vals(a, b, num_points):
    t = np.linspace(0, 1, num_points)
    # Transform t to the cosine space
    t = (1 - np.cos(t * np.pi)) / 2
    return [(1-tt) * a + tt * b for tt in t]

def cosine_interp_list(a, b, num_points):
    lst = []
    for i in range(len(a)):
        lst.append(cosine_interp_vals(a[i], b[i], num_points))
    return lst

# Input should be a list of joint in list
def move_arm_joints(joint_angles_list, duration):
    # Create a JointTrajectory message
    traj_msg = JointTrajectory()
    traj_msg.joint_names = joint_names

    # Get the current position
    start_joint_list = [current_joint_positions[i] for i in range(number_of_joints)]
    end_joint_list = joint_angles_list

    num_interp = 100
    interp_lists = cosine_interp_list(start_joint_list, end_joint_list, num_interp)

    for i in range(num_interp):
        this_joint_list = [interp_lists[j][i] for j in range(len(interp_lists))]
        point = JointTrajectoryPoint()
        point.positions = this_joint_list
        point.time_from_start = rospy.Duration(i * (duration / (num_interp - 1)))
        traj_msg.points.append(point)

    arm_pub.publish(traj_msg)

# Input should be Cartesian pose (x, y, z + rpy angles)
def move_arm_cartesian(goal_position, goal_orientation, duration):
    global desired_frame, current_joint_positions, ik_solver_pos
    # Set desired frame based on the goal position and orientation
    desired_position = Vector(*goal_position)
    desired_orientation = Rotation.RPY(*goal_orientation)
    desired_frame = Frame(desired_orientation, desired_position)
    
    # Use IK to calculate desired joint positions
    ik_result = ik_solver_pos.CartToJnt(current_joint_positions, desired_frame, desired_joint_positions)
    if ik_result >= 0:
        joint_positions_list = [desired_joint_positions[i] for i in range(number_of_joints)]
        move_arm_joints(joint_positions_list, duration)
    else:
        rospy.logerr("Failed to find an IK solution for the desired position.")

# ! Subscriber callbacks
def joint_state_callback(msg):
    global current_joint_positions, current_gripper_position, current_head_position
    try:
        for i, name in enumerate(joint_names):
            index = msg.name.index(name)
            current_joint_positions[i] = msg.position[index]
        gripper_index = msg.name.index("gripper_left_finger_joint")
        current_gripper_position[0] = msg.position[gripper_index]*2
        current_gripper_position[1] = msg.position[gripper_index]*2
        head_1_index = msg.name.index("head_1_joint")
        head_2_index = msg.name.index("head_2_joint")
        current_head_position[0] = msg.position[head_1_index]
        current_head_position[1] = msg.position[head_2_index]
    except Exception as e:
        rospy.logerr(f"Error in joint_state_callback: {e}")

def aruco_pose_callback(msg):
    global aruco_array
    aruco_array = np.array([msg.name[0], msg.position[0],msg.position[1]], dtype=object)
    # msg.name[0] == "1"
    # aruco_markers_updated_array_2 = np.array([msg.name[0], msg.position[0],msg.position[1]], dtype=object)

def postion_adjust(x_aruco, y_aruco, object_number):
    X_update = (200-x_aruco)*0.013/82
    Y_update = (343-y_aruco)*0.010/65



# ! Direct control
def update_gripper_position(increment):
    global current_gripper_position

    # Update the current position based on the increment/decrement value
    new_position = [increment, increment]

    # Ensure the new position is within the allowable range
    # Assuming the gripper range is between 0 (fully closed) and 0.04 (fully open)
    new_position = [max(0.02, min(1, pos)) for pos in new_position]

    # Update the global variable
    current_gripper_position = new_position

    # Create and send the new goal to the action server
    goal = FollowJointTrajectoryGoal()
    trajectory = JointTrajectory()
    trajectory.joint_names = ['gripper_left_finger_joint', 'gripper_right_finger_joint']
    point = JointTrajectoryPoint()
    point.positions = current_gripper_position
    point.time_from_start = rospy.Duration(0.5)
    trajectory.points.append(point)
    goal.trajectory = trajectory
    
    gripper_client.send_goal(goal)
    gripper_client.wait_for_result()


def update_head_position(pan_degrees, tilt_degrees, duration):
    global current_head_position
    
    # Convert degrees to radians for ROS
    pan_radians = pan_degrees * 3.141592653589793 / 180.0
    tilt_radians = tilt_degrees * 3.141592653589793 / 180.0

    goal = FollowJointTrajectoryGoal()
    trajectory = JointTrajectory()
    trajectory.joint_names = ['head_1_joint', 'head_2_joint']
    point = JointTrajectoryPoint()
    point.positions =  [pan_radians, tilt_radians] 
    point.time_from_start = rospy.Duration(duration)
    trajectory.points.append(point)
    goal.trajectory = trajectory
    
    head_client.send_goal(goal)
    head_client.wait_for_result()

# TODO: defind primitives

# ! Main loop
def run():
    global ik_solver_pos, desired_joint_positions, joint_names, number_of_joints, fk_solver, arm_pub, gripper_client, desired_frame, current_gripper_position, current_joint_positions, current_head_position, head_client, arm_client

    # Load the robot model from the parameter server
    robot_urdf = URDF.from_parameter_server()

    # Generate a KDL tree from the URDF model
    success, kdl_tree = treeFromUrdfModel(robot_urdf)
    if not success:
        rospy.logerr("Failed to extract KDL tree from URDF robot model.")
        exit(1)

    # Specify the chain: from base link to end-effector link
    base_link = "torso_lift_link"
    end_effector_link = "gripper_link"

    chain = kdl_tree.getChain(base_link, end_effector_link)

    # Initialize the IK solver
    ik_solver_pos = ChainIkSolverPos_LMA(chain)

    # Initialize the joint array with the number of joints
    number_of_joints = chain.getNrOfJoints()
    desired_joint_positions = JntArray(number_of_joints)
    current_joint_positions = JntArray(number_of_joints)

    # Initialize Forward Kinematics solver
    fk_solver = ChainFkSolverPos_recursive(chain)

    # List of joint names for the robot's arm - adjust this list to match your configuration
    joint_names = ["arm_1_joint", "arm_2_joint", "arm_3_joint", 
                   "arm_4_joint", "arm_5_joint", "arm_6_joint", "arm_7_joint"]

    current_gripper_position = [0, 0]
    current_head_position = [0, 0]

    # Subscribe to the current joint state
    rospy.Subscriber('/joint_states', JointState, joint_state_callback)
    rospy.loginfo("Subscribed to joint states.")

    rospy.Subscriber('/gripper_cam_aruco_pose', JointState, aruco_pose_callback)
    rospy.loginfo("Subscribed to aruco pose")

    listener = tf.TransformListener()
    aruco_tf_pub = rospy.Publisher("/aruco_pose_tf", JointState, queue_size=1)
    aruco_sub = rospy.Subscriber("/aruco_pose", JointState, aruco_callback)
    aruco_pub = rospy.Publisher("/aruco_pose_TF", JointState, queue_size=1)


    # Wait to get first values to ensure everything is initialized properly
    rospy.wait_for_message('/joint_states', JointState)

    # Publisher for controlling the robot's arm
    arm_pub = rospy.Publisher('/arm_controller/command', JointTrajectory, queue_size=10)

    # Action clients for the gripper and head
    gripper_client = actionlib.SimpleActionClient('/parallel_gripper_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
    gripper_client.wait_for_server()

    head_client = actionlib.SimpleActionClient('/head_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
    head_client.wait_for_server()

    arm_client = actionlib.SimpleActionClient('/arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
    arm_client.wait_for_server()
    rospy.loginfo("arm server connected.")


    # TODO: subscribe action agent message

    # TODO: call control functions
    id_aruco = aruco_array[0]
    x_aruco = aruco_array[1]
    y_aruco = aruco_array[2]
    x_diff, y_diff = postion_adjust(x_aruco, y_aruco, id_aruco)

    goal_position = [X_pose+X_update, Y_pose+Y_update, -0.13]
            

if __name__ == "__main__":
    # Initialize ROS node
    rospy.init_node('execute_control')
    run()

