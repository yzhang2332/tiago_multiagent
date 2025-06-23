#!/usr/bin/env python3
import rospy
import cv2
import cv2.aruco as aruco
import numpy as np
import tf
import tf_conversions
import tf2_ros
import threading
import actionlib

from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped, TransformStamped
from cv_bridge import CvBridge
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class HeadCamArucoDetector:
    def __init__(self):
        rospy.init_node("head_cam_aruco_tf_node")

        self.bridge = CvBridge()
        self.head_client = actionlib.SimpleActionClient(
            '/head_controller/follow_joint_trajectory',
            FollowJointTrajectoryAction
        )
        self.head_client.wait_for_server()

        self.listener = tf.TransformListener()
        self.aruco_pub = rospy.Publisher("/head_cam_aruco_pose", PointStamped, queue_size=10)

        self.marker_to_find = None
        self.scanning = False
        self.lock = threading.Lock()

        self.ARUCO_DICT = aruco.getPredefinedDictionary(aruco.DICT_4X4_100)
        self.DETECTION_PARAMS = aruco.DetectorParameters()
        self.MARKER_LENGTH = 0.02  # Marker size in metres

        rospy.Subscriber("/xtion/rgb/image_rect_color", Image, self.image_callback)

    def rotate_head(self, pan, tilt, duration=2.0):
        goal = FollowJointTrajectoryGoal()
        trajectory = JointTrajectory()
        trajectory.joint_names = ['head_1_joint', 'head_2_joint']

        point = JointTrajectoryPoint()
        point.positions = [pan, tilt]
        point.time_from_start = rospy.Duration(duration)
        trajectory.points.append(point)

        goal.trajectory = trajectory
        self.head_client.send_goal(goal)
        self.head_client.wait_for_result()

    def image_callback(self, msg):
        with self.lock:
            if not self.scanning or self.marker_to_find is None:
                return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

            corners, ids, _ = aruco.detectMarkers(gray, self.ARUCO_DICT, parameters=self.DETECTION_PARAMS)

            if ids is not None and self.marker_to_find in ids.flatten():
                index = ids.flatten().tolist().index(self.marker_to_find)
                camera_matrix = np.array([
                    [523.5531882248562, 0.0, 329.4655423070979],
                    [0.0, 522.6007280941504, 227.58372039112177],
                    [0.0, 0.0, 1.0]
                ])

                dist_coeffs = np.array([0.03437334, -0.12169691, -0.00709335, 0.00239277, 0.0])

                rvec, tvec, _ = aruco.estimatePoseSingleMarkers(
                    corners[index],
                    self.MARKER_LENGTH,
                    camera_matrix,
                    dist_coeffs
                )


                self.publish_static_tf(tvec[0][0], rvec[0][0], self.marker_to_find)
                rospy.loginfo(f"tvec: {tvec[0][0]}, rvec: {rvec[0][0]}")

                rospy.sleep(0.1)

                try:
                    (trans, rot) = self.listener.lookupTransform(
                        "torso_lift_link",
                        f"aruco_marker_{self.marker_to_find}",
                        rospy.Time(0)
                    )

                    pt_msg = PointStamped()
                    pt_msg.header.frame_id = "torso_lift_link"
                    pt_msg.header.stamp = rospy.Time.now()
                    pt_msg.point.x = trans[0]
                    pt_msg.point.y = trans[1]
                    pt_msg.point.z = trans[2]

                    self.aruco_pub.publish(pt_msg)
                    rospy.loginfo(f"Found marker {self.marker_to_find} and published its pose.")

                    # with self.lock:
                    #     self.scanning = False
                    #     self.marker_to_find = None

                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    rospy.logwarn("TF lookup failed")

        except Exception as e:
            rospy.logerr(f"Image processing failed: {e}")

    def publish_static_tf(self, tvec, rvec, marker_id):
        static_broadcaster = tf2_ros.StaticTransformBroadcaster()
        static_transform = TransformStamped()
        static_transform.header.stamp = rospy.Time.now()
        static_transform.header.frame_id = "xtion_rgb_optical_frame"
        static_transform.child_frame_id = f"aruco_marker_{marker_id}"
        static_transform.transform.translation.x = tvec[0]
        static_transform.transform.translation.y = tvec[1]
        static_transform.transform.translation.z = tvec[2]

        q = tf_conversions.transformations.quaternion_from_euler(*rvec)
        static_transform.transform.rotation.x = q[0]
        static_transform.transform.rotation.y = q[1]
        static_transform.transform.rotation.z = q[2]
        static_transform.transform.rotation.w = q[3]

        static_broadcaster.sendTransform(static_transform)

    def search_for_marker(self, marker_id):
        with self.lock:
            self.marker_to_find = marker_id
            self.scanning = True

        head_positions = [(-0.3, -0.9)]
        i = 0

        rospy.loginfo(f"Started scanning for marker {marker_id}...")

        while not rospy.is_shutdown():
            with self.lock:
                if not self.scanning:
                    rospy.loginfo(f"Stopped scanning. Marker {marker_id} was found.")
                    return

            pan, tilt = head_positions[i % len(head_positions)]
            self.rotate_head(pan, tilt)
            rospy.sleep(5.0)
            i += 1

    def spin(self):
        rospy.spin()
