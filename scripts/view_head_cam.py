#!/usr/bin/env python3

import rospy
import cv2
# from cv2 import aruco
import cv2.aruco as aruco
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

def image_callback(msg):
    bridge = CvBridge()
    window_name = 'Tiago Camera View'
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)  # Create a window that can be resized
    cv2.resizeWindow(window_name, 500, 480) 
    # Convert the ROS Image message to OpenCV2 format
    cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")

    # Convert to grayscale
    gray = cv2.cvtColor(cv2_img, cv2.COLOR_BGR2GRAY)

    # Initialize the detector parameters using default values
    # aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_1000)
    # parameters = aruco.DetectorParameters_create()
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
    parameters = cv2.aruco.DetectorParameters()

    # Detect the markers in the image
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

    if ids is not None:
        # Draw detected markers
        aruco.drawDetectedMarkers(cv2_img, corners, ids)
        for i, corner in zip(ids, corners):
            # Calculate marker center
            center = corner[0].mean(axis=0)
            # Display the ID near the marker
            cv2.putText(cv2_img, str(i[0]), (int(center[0]), int(center[1])), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,255), 2)
            # print(f"Detected ArUco marker ID: {i[0]}")

    # Display the resulting frame
    cv2.imshow(window_name, cv2_img)
    cv2.waitKey(3)

def main():
    rospy.init_node('tiago_aruco_detector')
    image_topic = "/xtion/rgb/image_raw"  # Updated topic
    # image_topic = "/camera/image_raw"
    rospy.Subscriber(image_topic, Image, image_callback)
    rospy.spin()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()