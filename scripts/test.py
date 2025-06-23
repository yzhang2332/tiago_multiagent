#!/usr/bin/env python

import rospy
import tf
import tf_conversions
from geometry_msgs.msg import PoseStamped
import tf2_ros
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState
from math import pi
from csv import writer
import numpy as np

from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import actionlib

class CoordinateTranslator:

    def __init__(self):

        rospy.init_node('coordinate_translator', anonymous=True)

        # Initialize the action client for controlling the head's movement
        self.head_client = actionlib.SimpleActionClient('/head_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        self.head_client.wait_for_server()

        # # Rotate the head 30 degrees to the right and tilt it 30 degrees down
        # self.rotate_and_tilt_head(-40, -55)  # Note: Assuming negative tilt means downwards
        # rospy.sleep(1)

        
        self.listener = tf.TransformListener()
        self.aruco_tf_pub = rospy.Publisher("/aruco_pose_tf", JointState, queue_size=1)

        self.aruco_sub = rospy.Subscriber("/aruco_pose", JointState, self.aruco_callback)
        self.aruco_pub = rospy.Publisher("/aruco_pose_TF", JointState, queue_size=1)


        rospy.sleep(0)  # Wait for the listener to get ready

        self.aruco_pose = np.ones((4,7))
        self.message_count = 0
        self.counter = 0

        # Set u CSV file
        # self.csv_file = open('aruco_data.csv', "w", newline ='')
        # self.csv_writer = writer(self.csv_file)
        # self.csv_writer.writerow(['Id','x', 'y', 'z' , 'roll', 'pitch', 'yaw'])
        rospy.sleep(0.3)


        # # Initialize the action client for controlling the head's movement
        # head_client = actionlib.SimpleActionClient('/head_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        # head_client.wait_for_server()

        # # Rotate the head 30 degrees to the right and tilt it 30 degrees down
        # self.rotate_and_tilt_head(head_client, -40, -55)  # Note: Assuming negative tilt means downwards


    def publish_static_transformation(self, x, y, z, roll, pitch, yaw):

        # Create a StaticTransformBroadcaster object
        static_broadcaster = tf2_ros.StaticTransformBroadcaster()

        # Create a TransformStamped message
        static_transform_stamped = TransformStamped()
        
        # Fill the message with the necessary data
        static_transform_stamped.header.stamp = rospy.Time.now()
        static_transform_stamped.header.frame_id = '/xtion_rgb_optical_frame'
        static_transform_stamped.child_frame_id = '/aruco_marker_frame'
        static_transform_stamped.transform.translation.x = x
        static_transform_stamped.transform.translation.y = y
        static_transform_stamped.transform.translation.z = z
        
        quaternion = tf_conversions.transformations.quaternion_from_euler(roll/180*pi, pitch/180*pi, yaw/180*pi)
        static_transform_stamped.transform.rotation.x = quaternion[0]
        static_transform_stamped.transform.rotation.y = quaternion[1]
        static_transform_stamped.transform.rotation.z = quaternion[2]
        static_transform_stamped.transform.rotation.w = quaternion[3]

        # Send the transformation
        static_broadcaster.sendTransform(static_transform_stamped)
        # print("static_broadcaster", self.marker_id, static_transform_stamped)
        # rospy.loginfo("Static transform published!")
        # rospy.loginfo("Sleeping for 0.5 seconds")
        rospy.sleep(0.01)


    def aruco_callback(self, msg: JointState):
        # self.message_count += 1
        # print(f"Message number{self.message_count}")

        self.marker_id = msg.name
        pose_vec = msg.position
        # if len(self.aruco_pose) == 0:
        # print(self.marker_id[0])
        # self.aruco_pose = pose_vec
        self.publish_static_transformation(pose_vec[0], pose_vec[1], pose_vec[2], pose_vec[3], pose_vec[4], pose_vec[5])
        trans, rot, rot_euler = self.get_transformation()
        # print("Point in base frame: ", trans)
        # print("Orientation in base frame (roll, pitch, yaw): ", rot_euler)
        # print(self.aruco_pose[0,0])
        # print(self.aruco_pose[int(self.marker_id[0])-1, 4])
        # self.head_rot()
        if int(self.marker_id[0]) < 5:
            self.message_count += 1
            # print(self.marker_id[0])
            print(f"Message number{self.message_count}")

            if abs(float(rot_euler[0])) < self.aruco_pose[int(self.marker_id[0])-1, 4]:
                self.aruco_pose[int(self.marker_id[0])-1, 0] = self.marker_id[0]
                self.aruco_pose[int(self.marker_id[0])-1, 1] = trans[0]
                self.aruco_pose[int(self.marker_id[0])-1, 2] = trans[1]
                self.aruco_pose[int(self.marker_id[0])-1, 3] = trans[2]
                self.aruco_pose[int(self.marker_id[0])-1, 4] = rot_euler[0]
                self.aruco_pose[int(self.marker_id[0])-1, 5] = rot_euler[1]
                self.aruco_pose[int(self.marker_id[0])-1, 6] = rot_euler[2]

        print(self.aruco_pose)

        # if self.message_count>=200:
        #     print(self.aruco_pose)
        #     if all(abs(x) < 0.1 for x in self.aruco_pose[:, 4]):
        #         print("TF Function was Successful")
        #     else:
        #         print("TF Function failed")
        #     for i in range(4):
        #         self.csv_writer.writerow(self.aruco_pose[i])
        #         pose_msg = JointState()
        #         # print()
        #         pose_msg.name = [str(self.aruco_pose[i, 0])]
        #         pose_msg.position = self.aruco_pose[i, 1:]
        #         # pose_msg.position = [x, y, z, quat[0], quat[1], quat[2], quat[3]]
        #         self.aruco_pub.publish(pose_msg)
        #         rospy.sleep(0.1)

        #     self.csv_file.close()
        #     rospy.signal_shutdown("Done")
        #     return
        
        # if self.message_count>=200:
        #     self.counter +=1
        #     print(self.aruco_pose)
        #     if self.counter == 8:
        #         self.counter = 1
        #     self.message_count = 0
        #     self.head_rot()
        #     rospy.sleep(3)

            # return
        
        # print(self.aruco_pose)



        # self.csv_writer.writerow([self.marker_id, trans[0], trans[1], trans[2], rot_euler[0], rot_euler[1], rot_euler[2]])


            # save_option = 1
            # if save_option == 1:
            #     # Define the path to your file
            #     file_path = '/home/pal/tiago_ws/src/woa_tiago/project/Data.txt'

            #     # Values you want to replace the first two zeros with
            #     value1 = trans[0]  # Change this to your desired value
            #     value2 = trans[1]  # Change this to your desired value

            #     # Open the file in append mode and add a new line with "0 0 0 0"
            #     with open(file_path, 'a') as file:
            #         file.write("\n0 0 0 0")  # Append new line at the end

            #     # Now, read all lines, modify the last one, and write everything back
            #     with open(file_path, 'r') as file:
            #         lines = file.readlines()  # Read all lines into a list

            #     # Modify the last line
            #     last_line = lines[-1].strip().split(' ')  # Split the last line into a list of its values
            #     last_line[0] = str(value1)  # Replace the first zero with value1
            #     last_line[1] = str(value2)  # Replace the second zero with value2
            #     lines[-1] = ' '.join(last_line) + '\n'  # Join back into a string and assign back to the last line

            #     # Write everything back to the file
            #     with open(file_path, 'w') as file:
            #         file.writelines(lines)  # Write all lines back into the file


    def get_transformation(self):
        (trans, rot) = self.listener.lookupTransform("/torso_lift_link", "/aruco_marker_frame", rospy.Time(0))
        rot_euler = tf_conversions.transformations.euler_from_quaternion(rot)

        aruco_tf_msg = JointState()
        aruco_tf_msg.name = self.marker_id
        aruco_tf_msg.position = [trans[0], trans[1], trans[2], rot_euler[0], rot_euler[1], rot_euler[2]]
        self.aruco_tf_pub.publish(aruco_tf_msg)
        # print("Publish aruco_tf")

        return trans, rot, rot_euler
    

    def rotate_and_tilt_head(self, pan_degrees, tilt_degrees):
        # global head_client
        
        # Convert degrees to radians for ROS
        pan_radians = pan_degrees * 3.141592653589793 / 180.0
        tilt_radians = tilt_degrees * 3.141592653589793 / 180.0

        # Define the goal for head movement
        goal = FollowJointTrajectoryGoal()
        trajectory = JointTrajectory()
        trajectory.joint_names = ['head_1_joint', 'head_2_joint']  # Assuming these are the correct joint names
        
        point = JointTrajectoryPoint()
        # Assign the new positions for pan (head_1_joint) and tilt (head_2_joint)
        point.positions = [pan_radians, tilt_radians]  
        point.time_from_start = rospy.Duration(5)  # Adjust the duration as needed for your setup
        trajectory.points.append(point)
        
        goal.trajectory = trajectory
        
        self.head_client.send_goal(goal)
        self.head_client.wait_for_result()
    
    def head_rot(self):
        if self.counter == 1:
            self.rotate_and_tilt_head(-25, -45)
        elif self.counter == 2:
            self.rotate_and_tilt_head(-30, -55)
        elif self.counter == 3:
            self.rotate_and_tilt_head(-35, -50)
        elif self.counter == 4:
            self.rotate_and_tilt_head(-40, -60)
        elif self.counter == 5:
            self.rotate_and_tilt_head(-35, -55)
        elif self.counter == 6:
            self.rotate_and_tilt_head(-30, -45)
        elif self.counter == 7:
            self.rotate_and_tilt_head(-25, -40)



    def run(self):
        rospy.spin()

def main():
    translator = CoordinateTranslator()
    translator.run()

if __name__ == '__main__':
    main()