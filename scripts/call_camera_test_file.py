from head_cam import HeadCamArucoDetector

detector = HeadCamArucoDetector()

# Start ROS spinning in a thread
import threading
threading.Thread(target=detector.spin, daemon=True).start()

# When ready, trigger the search
detector.search_for_marker(1)




import rospy
import json
from sensor_msgs.msg import JointState, Image
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from std_msgs.msg import Float32MultiArray, String
from geometry_msgs.msg import PoseStamped, Quaternion, PointStamped
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

def move_arm_cartesian(goal_position, goal_orientation):
    global desired_frame, current_joint_positions, ik_solver_pos
    # Set desired frame based on the goal position and orientation
    desired_position = Vector(*goal_position)
    desired_orientation = Rotation.RPY(*goal_orientation)
    desired_frame = Frame(desired_orientation, desired_position)
    
    # Use IK to calculate desired joint positions
    ik_result = ik_solver_pos.CartToJnt(current_joint_positions, desired_frame, desired_joint_positions)
    if ik_result >= 0:
        joint_positions_list = [desired_joint_positions[i] for i in range(number_of_joints)]
        move_arm_joints(joint_positions_list, 5)
        print(joint_positions_list)
    else:
        rospy.logerr("Failed to find an IK solution for the desired position.")

def move_arm_joints(joint_angles_list, duration):
    # Create a JointTrajectory message
    traj_msg = JointTrajectory()
    traj_msg.joint_names = joint_names

    # traj_msg.header.stamp = rospy.Time.now() + rospy.Duration(0.1)


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
    aruco_array = np.array([msg.point.x, msg.point.y, msg.point.z], dtype=np.float32)
    print(aruco_array)

def postion_adjust(x_aruco, y_aruco):
    x_diff = (200-x_aruco)*0.013/82
    y_diff = (343-y_aruco)*0.010/65
    return x_diff, y_diff


rospy.init_node("move_arm_cartesian_test")

# Wait for simulated time to start running
rospy.loginfo("Waiting for simulated time to be active...")
while rospy.Time.now().to_sec() == 0:
    rospy.sleep(0.1)
rospy.loginfo("Simulated time has started.")


# ! remove this when test on real robot
arm_pub = rospy.Publisher('/arm_controller/command', JointTrajectory, queue_size=10)

# # Wait for publisher connection
# timeout = rospy.Time.now() + rospy.Duration(5.0)
# while arm_pub.get_num_connections() == 0 and rospy.Time.now() < timeout:
#     rospy.logwarn("Waiting for subscriber to connect to /arm_controller/command...")
#     rospy.sleep(0.2)

# if arm_pub.get_num_connections() == 0:
#     rospy.logerr("No subscriber to /arm_controller/command. Is the controller running?")
#     sys.exit(1)


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

# List of joint names for the robot's arm - adjust this list to match your configuration
joint_names = ["arm_1_joint", "arm_2_joint", "arm_3_joint", 
                "arm_4_joint", "arm_5_joint", "arm_6_joint", "arm_7_joint"]

current_gripper_position = [0, 0]
current_head_position = [0, 0]

rospy.Subscriber('/joint_states', JointState, joint_state_callback)
rospy.wait_for_message('/joint_states', JointState)
rospy.sleep(1.0)
rospy.loginfo("Subscribed to joint states.")

rospy.Subscriber('/gripper_cam_aruco_pose', PointStamped, aruco_pose_callback)
rospy.loginfo("Subscribed to aruco pose")


# from gripper_cam import detect_marker
import subprocess

def detect_marker(marker_id):
    subprocess.run(['python3', 'gripper_cam.py', str(marker_id)])

# Call separately, sequentially
detect_marker(1)

# Initialize Forward Kinematics solver
fk_solver = ChainFkSolverPos_recursive(chain)

x = -0.03

# goal_position = [0.6433327123656016+x, -0.08958365777810416, -0.15]
# goal_orientation = (0.0, 0.0, pi/2)

# move_arm_cartesian(goal_position, goal_orientation)
# rospy.spin()



x_aruco = aruco_array[0]
y_aruco = aruco_array[1]

print(x_aruco, y_aruco)

x_diff, y_diff = postion_adjust(x_aruco, y_aruco)
print(x_diff, y_diff)

goal_position = [0.6433327123656016+x+x_diff, -0.08958365777810416+y_diff, -0.22]
print(goal_position)
goal_orientation = (0.0, 0.0, pi/2)

move_arm_cartesian(goal_position, goal_orientation)
rospy.spin()

# [1.2123916722690562, 0.5380127911322425, -1.5483126385165813, 1.9186585105571596, 0.5052052087121506, -1.3765408352396855, 1.5034949379199503]
