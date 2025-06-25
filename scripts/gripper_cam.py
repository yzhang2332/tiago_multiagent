#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco
import numpy as np
import paramiko
import sys


def start_remote_script():
    hostname = "tiago-196c"
    port = 22
    username = "pal"
    password = "pal"

    try:
        ssh = paramiko.SSHClient()
        ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        ssh.connect(hostname, port=port, username=username, password=password)

        command = "source /opt/ros/noetic/setup.bash && cd scripts && python3 camera.py"
        ssh.exec_command(command)
        ssh.close()
        print("Remote camera script started.")
    except Exception as e:
        print(f"SSH connection failed: {e}")

def start_once(target_id):
    start_remote_script()
    rospy.init_node('one_shot_detector', anonymous=True)
    detector = OneShotArucoDetector(target_marker_id=target_id)
    rospy.spin()


class OneShotArucoDetector:
    def __init__(self, target_marker_id, auto_shutdown=True, start_remote=True):
        self.bridge = CvBridge()
        self.target_marker_id = target_marker_id
        self.auto_shutdown = auto_shutdown
        self.found = False

        if start_remote:
            start_remote_script()

        self.sub = rospy.Subscriber('/camera/image_raw', Image, self.image_callback)
        self.pub = rospy.Publisher('/gripper_cam_aruco_pose', PointStamped, queue_size=1)

        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_100)
        self.parameters = aruco.DetectorParameters()

        rospy.loginfo(f"Looking for ArUco marker ID {self.target_marker_id}...")

    def image_callback(self, msg):
        if self.found:
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)
            # rospy.loginfo("Start gripper aruco detection")

            if ids is not None:
                for i, corner in enumerate(corners):
                    marker_id = ids[i][0]
                    if marker_id == self.target_marker_id:
                        marker_corners = corner[0]
                        avg_x = float(np.mean(marker_corners[:, 0]))
                        avg_y = float(np.mean(marker_corners[:, 1]))

                        pose_msg = PointStamped()
                        # pose_msg.header.stamp = rospy.Time.now()
                        pose_msg.header.stamp = rospy.Time(0)
                        pose_msg.header.frame_id = "camera_frame"
                        pose_msg.point.x = avg_x
                        pose_msg.point.y = avg_y
                        pose_msg.point.z = 0.0
                        self.pub.publish(pose_msg)
                        rospy.sleep(0.2)

                        # print(f"Marker {marker_id} at ({avg_x:.2f}, {avg_y:.2f})")
                        self.found = True
                        if self.auto_shutdown:
                            rospy.signal_shutdown("Aruco marker detected. Shutting down.")


        except Exception as e:
            rospy.logerr("Image callback error: %s", str(e))

    def start(self):
        rospy.loginfo(f"Looking for ArUco marker ID {self.target_marker_id}...")
        # rospy.spin()

    

if __name__ == '__main__':
    import sys
    if len(sys.argv) < 2:
        print("Usage: python3 gripper_cam.py <marker_id>")
    else:
        target_id = int(sys.argv[1])
        rospy.init_node('gripper_cam_detector', anonymous=True)
        detector = OneShotArucoDetector(target_marker_id=target_id)
        detector.start()