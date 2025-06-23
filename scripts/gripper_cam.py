#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PointStamped
import numpy as np
import paramiko


def image_callback(msg, aruco_pub):
    bridge = CvBridge()
    try:
        # Convert ROS Image message to OpenCV image
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")

        # Convert to grayscale (required for ArUco detection)
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # Define the dictionary and parameters for ArUco marker detection
        aruco_dict = aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)  # You can choose different dictionaries
        parameters = aruco.DetectorParameters()

        # Detect ArUco markers in the image
        corners, ids, rejected_img_points = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        # If markers are detected, draw them
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(cv_image, corners, ids)

            for i, corner in enumerate(corners):
                # print(f"Marker ID: {ids[i][0]}")
                marker_id = ids[i][0]
                X_avg = int((corners[i][0][0][0]+corners[i][0][1][0])/2)
                Y_avg = int((corners[i][0][0][1]+corners[i][0][1][1])/2)


                marker_corners = corner[0]  # Extract the corner points from the list
                # Calculate the average X and Y for this marker
                avg_x = np.mean(marker_corners[:, 0])  # Average X-coordinate
                avg_y = np.mean(marker_corners[:, 1])  # Average Y-coordinate


                # publish aruco pose
                pose_msg = PointStamped()
                pose_msg.header.stamp = rospy.Time.now()
                pose_msg.header.frame_id = "camera_frame"  # or your real frame
                pose_msg.point.x = avg_x
                pose_msg.point.y = avg_y
                pose_msg.point.z = 0.0  # optional for 2D
                aruco_pub.publish(pose_msg)

        # Display the image
        cv2.imshow("Camera Feed", cv_image)
        cv2.waitKey(1)
    except Exception as e:
        rospy.logerr("Failed to convert image: %s", str(e))

def start_remote_script():
    hostname = "tiago-196c"
    port = 22
    username = "pal"
    password = "pal"  # Replace with actual password

    try:
        ssh = paramiko.SSHClient()
        ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        ssh.connect(hostname, port=port, username=username, password=password)

        command = "source /opt/ros/noetic/setup.bash && cd scripts && python3 camera.py"
        stdin, stdout, stderr = ssh.exec_command(command)

        # print("STDOUT:")
        # print(stdout.read().decode())

        # print("STDERR:")
        # print(stderr.read().decode())

        ssh.close()
    except Exception as e:
        print(f"SSH connection failed: {e}")

def main():
    start_remote_script() 
    rospy.init_node('gripper_cam', anonymous=True)
    aruco_pub = rospy.Publisher("/gripper_cam_aruco_pose", PointStamped, queue_size=10)

    rospy.Subscriber('/camera/image_raw', Image, image_callback, aruco_pub)
    rospy.spin()

if __name__ == '__main__':
    main()