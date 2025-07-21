#!/usr/bin/env python

import rospy
import actionlib
import numpy as np
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from std_msgs.msg import Float64

# Global variables
current_joint_positions = {}
arm_client = None
head_client = None
torso_pub = None  # Publisher for the torso

def joint_state_callback(msg):
    """
    Callback function for /joint_states to update current joint positions.
    """
    global current_joint_positions
    for i, name in enumerate(msg.name):
        current_joint_positions[name] = msg.position[i]

def get_current_joint_positions(joint_names):
    """
    Get the current joint positions for the specified joint names.
    """
    rospy.sleep(0.5)  # Wait to ensure callback has populated the data
    return [current_joint_positions.get(joint, 0.0) for joint in joint_names]

def move_torso(height, duration=2):
    """
    Move the torso to the specified height using a JointTrajectory message.
    """
    global torso_pub

    if torso_pub is None:
        rospy.logerr("Torso publisher not initialized")
        return

    # Define the JointTrajectory message
    trajectory = JointTrajectory()
    trajectory.joint_names = ["torso_lift_joint"]

    # Define the trajectory point
    point = JointTrajectoryPoint()
    point.positions = [height]
    point.time_from_start = rospy.Duration(duration)

    trajectory.points.append(point)

    # Publish the trajectory
    torso_pub.publish(trajectory)

    rospy.sleep(duration)  # Ensure movement completes


def sort_assemb_start():
    """
    Main function to start the sorting assembly position.
    """
    global arm_client, head_client, torso_pub

    # Initialize torso publisher
    torso_pub = rospy.Publisher('/torso_controller/command', JointTrajectory, queue_size=10)

    rospy.sleep(1)  # Give time for publisher to initialize

    # Move torso up to 0.35
    move_torso(0.35, duration=2)


    # Initialize action clients
    arm_client = actionlib.SimpleActionClient('/arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
    arm_client.wait_for_server()

    head_client = actionlib.SimpleActionClient('/head_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
    head_client.wait_for_server()

    # Rotate and tilt head
    # rotate_and_tilt_head(0, -10, 2)  # Rotate head 0 degrees, tilt down 10 degrees

    # Get current joint positions
    joint_names = ['arm_1_joint', 'arm_2_joint', 'arm_3_joint', 
                   'arm_4_joint', 'arm_5_joint', 'arm_6_joint', 'arm_7_joint']
    



    joint_angles_start = get_current_joint_positions(joint_names)
    joint_angles_end = [0.14, -1.5, -0.12, 1.87, 1.55, -1.39, 0.33]
    move_arm(joint_angles_start, joint_angles_end, t=6)
    rospy.sleep(7)


    joint_angles_start = get_current_joint_positions(joint_names)
    joint_angles_end = [0.07, 1.02, 0, 1.29, 1.6, -1.36, 0.19]
    move_arm(joint_angles_start, joint_angles_end, t=6)
    rospy.sleep(7)

    joint_angles_start = get_current_joint_positions(joint_names)
    # joint_angles_end = [0.07, 0.68, -1.77, 1.98, 0.68, -1.39, 0.49]
    joint_angles_end = [0.07, 0.7, -1.3, 1.68, 0.72, -1.29, 0.16]
    move_arm(joint_angles_start, joint_angles_end, t=6)
    rospy.sleep(7)
    




def move_arm(joint_angles_start, joint_angles_end, t, steps=50):
    """
    Move the robot arm with cosine-interpolated smooth motion.
    """
    global arm_client

    # Define the goal
    goal = FollowJointTrajectoryGoal()
    trajectory = JointTrajectory()
    trajectory.joint_names = [
        'arm_1_joint', 'arm_2_joint', 'arm_3_joint', 'arm_4_joint', 
        'arm_5_joint', 'arm_6_joint', 'arm_7_joint'
    ]

    # Generate interpolated points
    times = np.linspace(0, t, steps)
    for i in range(steps):
        alpha = (1 - np.cos(np.pi * (i / (steps - 1)))) / 2  # Cosine interpolation factor
        interpolated_positions = [
            start + alpha * (end - start)
            for start, end in zip(joint_angles_start, joint_angles_end)
        ]
        point = JointTrajectoryPoint()
        point.positions = interpolated_positions
        point.time_from_start = rospy.Duration(times[i])
        trajectory.points.append(point)

    # Set the trajectory in the goal
    goal.trajectory = trajectory

    # Send the goal and wait for the result
    # rospy.loginfo("Sending goal for arm movement with smooth interpolation...")
    arm_client.send_goal(goal)
    # if arm_client.wait_for_result(rospy.Duration(t + 1)):
    #     rospy.loginfo("Arm completed successfully with smooth motion.")
    # else:
    #     rospy.loginfo("Arm did not complete before the timeout.")


if __name__ == '__main__':
    rospy.init_node('start_position')

    # Subscribe to /joint_states
    rospy.Subscriber("/joint_states", JointState, joint_state_callback)

    # Start the sorting assembly
    sort_assemb_start()
