#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import socketio

sio = socketio.Client()

def callback(msg):
    rospy.loginfo(f"Received: {msg.data}")
    if msg.data in ['start_listen', 'stop_listen']:
        try:
            sio.emit('command', msg.data)
        except Exception as e:
            rospy.logwarn(f"SocketIO send failed: {e}")

def listener():
    rospy.init_node('gif_display_bridge', anonymous=True)

    try:
        sio.connect('http://localhost:5000')
        rospy.loginfo("Connected to SocketIO server.")
    except Exception as e:
        rospy.logerr(f"Could not connect to server: {e}")
        return

    rospy.Subscriber('/listen_signal', String, callback)
    rospy.spin()
    sio.disconnect()

if __name__ == '__main__':
    listener()
