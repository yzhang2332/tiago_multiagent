#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import sys
import os

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
from utils.tcp import TCPClient, TCPServer

TCP_TARGET_IP = "tiago-pro-head-6c.local"
TCP_TARGET_PORT = 9091
TCP_LISTEN_PORT = 9092

client = TCPClient(TCP_TARGET_IP, TCP_TARGET_PORT)

def ros1_to_tcp_callback(msg):
    response = client.send(msg.data)
    rospy.loginfo(f"[ROS1 → TCP → ROS2] Sent: {msg.data} | Got: {response}")

def tcp_to_ros1_handler(message, addr):
    pub.publish(message)
    return "ack"

if __name__ == '__main__':
    rospy.init_node('ros1_tcp_bridge')

    sub = rospy.Subscriber('/ros1_to_tts', String, ros1_to_tcp_callback)
    pub = rospy.Publisher('/ros1_feedback', String, queue_size=10)

    server = TCPServer(port=TCP_LISTEN_PORT, handler=tcp_to_ros1_handler)
    server.start()

    rospy.loginfo("ROS1 TCP bridge running.")
    rospy.spin()
